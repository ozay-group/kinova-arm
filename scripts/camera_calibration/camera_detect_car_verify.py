"""
camera_detect_car_verify.py
Description:
    This script uses the learned camera extrinsics to detect the car position.

    Please check the file camera_extrinsics.npy exists in the current folder
    
    The code iterates following steps with various arm poses:
        - Detect the car pose based on the attached Apriltag
        - Compuate the car pose expressed in Base frame given the camera extrinsics
        - Compute the target End Effector pose X_base_ee in Base frame such that the End Effector is located right above the car
        - Move the End Effector to the target position 

Author: Zexiang Liu
Date: May 2024
"""

""" Import Modules """
# setting path for imports
import sys
sys.path.append('../')

# general python modules
import numpy as np
import time
import cv2
import matplotlib.pyplot as plt

# intel realsense
import pyrealsense2 as rs

# apriltag detector
from dt_apriltags import Detector

from liegroups import SE3

# drake functions
# from pydrake.all import *
from pydrake.all import (
    StartMeshcat, RollPitchYaw, RotationMatrix, RigidTransform, LogVectorOutput,
    DiagramBuilder, Simulator, CameraInfo, ResetIntegratorFromFlags
)
# kinova station
from kinova_drake.kinova_station import (
    KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget)
from static_controller import StaticController
from calibrUtilities import plot_frame, averageSE3

""" Apriltag Detector """
tag_size = 0.0255
# Measured on the tag. (See documentation on how to measure april tag sizes. A link that can help explain:
# https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide)
at_detector = Detector(families='tagStandard41h12', # Configure AprilTag detector
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)


""" Configurations """
show_toplevel_system_diagram = False    # Make a plot of the diagram for inner workings of the stationn
show_state_plots = False                # Show the plot of Poses

n_dof = 6                               # number of degrees of freedom of the arm
gripper_type = "2f_85"                  # which gripper to use (hande or 2f_85)
time_step = 0.1                         # time step size (seconds)
n_sample = 40                          # number of images captured by camera


### load extrinsics ### 
with open('camera_extrinsics.npy', 'rb') as f:
    R_world_cam = np.load(f)
    p_world_cam = np.load(f)
R_world_cam = RotationMatrix(R_world_cam)
X_world_cam = RigidTransform(R_world_cam, p_world_cam.transpose())

with KinovaStationHardwareInterface(n_dof, None) as station:
# Note that unlike the simulation station, the hardware station needs to be used within a 'with' block.
# This is to allow for cleaner error handling, since the connection with the hardware needs to be
# closed properly even if there is an error (e.g. KeyboardInterrupt) during execution.
    """ Connect Station """
    builder = DiagramBuilder() # Create a Drake diagram
    station = builder.AddSystem(station) # Connect Station
    
    """ Connect Loggers """
    # Connect the state of block to a Logger
    q_logger = LogVectorOutput(station.GetOutputPort("measured_arm_position"), builder)
    q_logger.set_name("arm_position_logger")
    qd_logger = LogVectorOutput(station.GetOutputPort("measured_arm_velocity"), builder)
    qd_logger.set_name("arm_velocity_logger")
    tau_logger = LogVectorOutput(station.GetOutputPort("measured_arm_torque"), builder)
    tau_logger.set_name("arm_torque_logger")

    pose_logger = LogVectorOutput(station.GetOutputPort("measured_ee_pose"), builder)
    pose_logger.set_name("pose_logger")
    twist_logger = LogVectorOutput(station.GetOutputPort("measured_ee_twist"), builder)
    twist_logger.set_name("twist_logger")
    wrench_logger = LogVectorOutput(station.GetOutputPort("measured_ee_wrench"), builder)
    wrench_logger.set_name("wrench_logger")

    ''' Command Sequence & Control '''
    # pscs, controller = sequence_calibration_toward_wall.calibration_toward_wall(
    #     initial_move, roll, pitch, yaw, width, depth, height)
    cont = StaticController()

    controller = builder.AddSystem(cont)
    controller.set_name("controller")
    controller.ConnectToStation(builder, station)

    ''' Visualizer '''
    # meshcat = StartMeshcat()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_aspect('equal')
    plt.ion() # turn on interactive mode (for plot.show(block=False))
    plt.show()
    pose = np.eye(4)
    plot_frame(ax, pose)

    """ Build Diagram """
    diagram = builder.Build() # Build the system diagram and create default context
    diagram.set_name("toplevel_system_diagram")
    diagram_context = diagram.CreateDefaultContext()

    """ Simulation Sequence """
    # station.go_home(name="Home") # Set default arm positions
    
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)
    
    integration_scheme = "explicit_euler"
    ResetIntegratorFromFlags(simulator, integration_scheme, time_step)
    context = simulator.get_mutable_context()
    simulator.Initialize()
    simulator.AdvanceTo(2.0)

    cnt = 0
    se3_base_cam = []
    roll = 0
    width = 0
    
    while(1):
        # Detect April Tag

        pipeline = rs.pipeline() # Declare RealSense pipeline, encapsulating the actual device and sensors
        config = rs.config()

        pipeline_wrapper = rs.pipeline_wrapper(pipeline) # Get device product line for setting a supporting resolution
        pipeline_profile = config.resolve(pipeline_wrapper) 
        device = pipeline_profile.get_device()
            
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Enable depth stream
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30) # Enable color stream

        cfg = pipeline.start(config) # Start streaming the pipeline and get the configuration


        """ Camera Intrinsics """
        """ Get camera parameters [fx, fy, cx, cy] from RealSense camera
        cam_params = [ 386.738, 386.738, 321.281, 238.221 ]
        https://github.com/IntelRealSense/librealsense/issues/869
        https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.intrinsics.html
        """
        profile = cfg.get_stream(rs.stream.color)                       # Fetch stream profile for color stream
        intrinsics = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
        cam_params = [intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy]
        camera_info = CameraInfo(
            width=1920,
            height=1080,
            focal_x=intrinsics.fx,
            focal_y=intrinsics.fy,
            center_x=intrinsics.ppx,
            center_y=intrinsics.ppy,
        )
        print("Camera parameters: ", cam_params)
        se3_list = [] # list of se3 elements
        for i in range(n_sample):
            frames = pipeline.wait_for_frames() # Wait for a coherent pair of frames: depth and color
            
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                print("no frame detected.")
                continue
            
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # # Perform Apriltag detection
            gray_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
            # cv2.imwrite('test.png', gray_image)
            atag = at_detector.detect(
                    gray_image,
                    estimate_tag_pose=True,
                    camera_params=cam_params,
                    tag_size= tag_size
                    )
            
            """ Collect RealSense Data """
            if atag:
                pose = np.zeros([4,4])
                pose[:3, :3] = atag[0].pose_R
                pose[:3, 3] = np.squeeze(atag[0].pose_t, axis=1)
                pose[3, 3] = 1
                plot_frame(ax, pose @ np.diag([1.0, -1.0, -1.0, 1.0]), 0.5, 1)
                se3_list.append(SE3.from_matrix(pose).log())
        avg_pose, _ = averageSE3(se3_list)
        if avg_pose is not None:
            avg_pose = avg_pose @ np.diag([1.0, -1.0, -1.0, 1.0])

            plot_frame(ax, avg_pose, 10, 0.4)
            # compute the X_base_cam
            X_cam_atag = RigidTransform(RotationMatrix(avg_pose[:3, :3]), avg_pose[:3, 3])
            X_base_atag = X_world_cam.multiply(X_cam_atag).GetAsMatrix4()
            offset = np.eye(4)
            offset[0, 3] = -0.06
            offset[2, 3] = -0.028
            X_base_ee = X_base_atag @ offset
            print("Rotation estimate:", X_base_atag[0:3, 0:3])
            print("Translation estimate:", X_base_atag[:3, 3])
            print("Planned position:", X_base_ee[:3,3])
            pregrasp_pose = np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, X_base_ee[0,3], X_base_ee[1,3], 0.10])
            cont.SetTargetPose(pregrasp_pose)
            simulator.AdvanceTo(context.get_time() + 2.0)
            print("Please check if the gripper is moving to location of the car.")
        else:
            print("No AprilTag is detected.")
        pipeline.stop() # Stop streaming
        plt.draw()
        plt.pause(0.001)
        key = input("Move the car to a new position and press Enter to continue; or press Q to exit ...")
        if key == "q":
            break