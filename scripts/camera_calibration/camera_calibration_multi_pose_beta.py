"""
camera_calibration_multi_pose_beta.py
Description:
    This scripts determines the Camera Extrinsics.
    
    Iterate following steps with various arm poses:
        - Move the arm to the position where AprilTag can be seen by the camera
        - Detect the AprilTag pose within the captured frames from camera and average them
        - Compute the End Effector pose in Base frame
        - Compute the Base frame expressed in the Camera frame (X_cam_base)
    - Average over all X_cam_base and return the inverse (X_base_cam)

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
from ransac import ransac

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

# kortex api
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.Exceptions.KServerException import KServerException
import utilities # utilities helper module for kinova arm kinematics


# command sequences
import sequence_calibration_toward_atrium
import sequence_calibration_toward_wall

from calibrUtilities import plot_frame, averageSE3

def convertRPY2RigidTransform(pose):
    R = RollPitchYaw(pose[:3])
    return RigidTransform(R.ToRotationMatrix(), pose[3:])

""" Apriltag Detector """
tag_size = 0.0255
# Measured on the tag. (See documentation on how to measure april tag sizes. A link that can help explain:
# https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide)
at_detector = Detector(families='tagStandard41h12', # Configure AprilTag detector
                       nthreads=8,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

# replay = 0 randomize poses of end effector; replay = 1 use pre-recorded poses of end effector (if you have run the code with replay = 0 before)
replay = 0

""" Configurations """
args = utilities.parseConnectionArguments() # Set the IP address of Kinova Robot

show_toplevel_system_diagram = False    # Make a plot of the diagram for inner workings of the stationn
show_state_plots = False                # Show the plot of Poses

n_dof = 6                               # number of degrees of freedom of the arm
gripper_type = None                  # which gripper to use (hande or 2f_85)
time_step = 0.1                         # time step size (seconds)
n_sample = 400                          # number of images captured by camera


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
    fig = plt.figure()
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.set_aspect('equal')
    ax1.title.set_text("X_cam_ee")
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.set_aspect('equal')
    ax2.title.set_text("X_cam_base")
    plt.ion() # turn on interactive mode (for plot.show(block=False))
    plt.show()
    pose = np.eye(4)
    plot_frame(ax1, pose)
    plot_frame(ax2, pose)

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
    total_cnt = 20 # the total number of poses we take average with 
    se3_cam_base = []
    roll = 0
    width = 0
    if replay == 0:
        candidate_inputs = []
    else:
        with open('candidate_inputs.npy', 'rb') as f:
            candidate_inputs = np.load(f)
    
    while(cnt < total_cnt):
        print("=====", cnt+1, "/", total_cnt, "=====")
        if replay:
            [pitch, yaw, depth, height] = candidate_inputs[cnt]
        else:
            [pitch, yaw, depth, height] = np.random.uniform(-1, 1, size=4) 
        cont.SetCalibrationTargetPose(roll, pitch, yaw, width, depth, height)
        simulator.AdvanceTo(context.get_time() + 2.0)
        """ Collect Station Data """
        pose_log = pose_logger.FindLog(diagram_context)
        pose_log_times = pose_log.sample_times()
        pose_log_data = pose_log.data()
        print("")
        print("Target control frequency: %s Hz" % (1/time_step))
        print("Actual control frequency: %s Hz" % (1/time_step * simulator.get_actual_realtime_rate()))
        # plot_frame(ax, convertRPY2RigidTransform(pose_log_data[:,-1]).GetAsMatrix4())
        # plt.draw()
        # plt.pause(0.001)

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
        profile = cfg.get_stream(rs.stream.color)                       # Fetch stream profile for depth stream
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
        se3_list = [] # list of se3 elements
        img_cnt = 0
        for i in range(n_sample):
            frames = pipeline.wait_for_frames() # Wait for a coherent pair of frames: depth and color
            
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            
            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            # # Perform Apriltag detection
            gray_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
            # cv2.imwrite("./test"+str(img_cnt)+".jpg", gray_image)
            img_cnt += 1
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
                plot_frame(ax1, pose @ np.diag([1.0, -1.0, -1.0, 1.0]), 0.5, 1)
                se3_list.append(SE3.from_matrix(pose).log())
    
        # Compute the average of X_cam_ee
        if len(se3_list) > 0:
            avg_pose, inliers = averageSE3(se3_list)  
        if avg_pose is not None:
            avg_pose = avg_pose @ np.diag([1.0, -1.0, -1.0, 1.0])
            # compute the X_cam_base
            X_cam_atag = RigidTransform(RotationMatrix(avg_pose[:3, :3]), avg_pose[:3, 3])
            pose_ee = pose_log_data[:,-1]
            print("Pose log:", pose_log_data[:,-1])
            R_base_ee = RollPitchYaw(np.array([pose_ee[0], pose_ee[1], pose_ee[2]]))
            p_base_ee = pose_ee[3:]
            X_base_ee = RigidTransform(R_base_ee, p_base_ee)
            X_cam_ee = X_cam_atag
            X_cam_base = X_cam_ee.multiply(X_base_ee.inverse())
            std_se3 = np.std(inliers, axis=0) # standard deviation of estimated se3
            print("std: ", std_se3)
            if(len(inliers) >= 25 or replay):
                cnt += 1
                print(X_cam_base.GetAsMatrix4())
                se3_cam_base.append(SE3.from_matrix(X_cam_base.GetAsMatrix4(),normalize=True).log())
                plot_frame(ax2, X_cam_base.GetAsMatrix4())
                plot_frame(ax1, avg_pose, 10, 0.4)
                if replay == 0:
                    candidate_inputs.append([pitch, yaw, depth, height])
            else:
                break
            plt.draw()
            plt.pause(0.001)
        pipeline.stop() # Stop streaming
    avg_X_cam_base, _ = averageSE3(se3_cam_base, 0.1)
    if avg_X_cam_base is not None:
        R_base_cam = avg_X_cam_base[:3, :3].T 
        p_base_cam = - R_base_cam @ avg_X_cam_base[:3, 3]
        if replay == 0:
            with open('candidate_inputs.npy', 'wb') as f:
                np.save(f, candidate_inputs)
        
        with open('camera_extrinsics.npy', 'wb') as f:
            np.save(f, R_base_cam)
            np.save(f, p_base_cam)
        plot_frame(ax2, avg_X_cam_base, 10, 0.4)
        plt.draw()
        plt.show(block=True)
    else:
        print("Ransac fails. Sampled poses are too noisy.")