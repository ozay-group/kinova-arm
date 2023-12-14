"""

camera_calibration_single_pose.py
Description:
    This scripts determines the Camera Extrinsics.
    
    Perform following steps:
        - Move the arm to the position where AprilTag can be seen by the camera
        - Detect the AprilTag pose within the captured frames from camera and average them
        - Use forward kinematics to compute the End Effector pose in Base frame
        - Compute the Camera pose in Base frame
"""

""" Import Modules """
# setting path for imports
import sys
sys.path.append('../')

# general python modules
import numpy as np
import cv2
import matplotlib.pyplot as plt

# intel realsense
import pyrealsense2 as rs

# apriltag detector
from dt_apriltags import Detector

# drake functions
from pydrake.all import *
from pydrake.all import (
    Meshcat, RollPitchYaw, RotationMatrix, RigidTransform, LogVectorOutput,
    DiagramBuilder, Simulator, CameraInfo, ResetIntegratorFromFlags
)
# kinova station
from kinova_drake.kinova_station import (
    KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget)
from kinova_drake.controllers import (
    PSCommandSequenceController, PSCommandSequence, PartialStateCommand)

# kortex api
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.Exceptions.KServerException import KServerException
import utilities # utilities helper module for kinova arm kinematics

# command sequences
import sequence_calibration_toward_atrium
import sequence_calibration_toward_wall


""" Apriltag Detector """
tag_size = 0.025
# Measured on the tag. (See documentation on how to measure april tag sizes. A link that can help explain:
# https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide)
at_detector = Detector(families='tagStandard41h12', # Configure AprilTag detector
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)


""" Parameters """
hardware_control = True                 # Move arm to the calibration position
show_toplevel_system_diagram = False    # Make a plot of the diagram for inner workings of the stationn
show_state_plots = False                # Show the plot of Poses

n_dof = 6                               # number of degrees of freedom of the arm
gripper_type = "2f_85"                  # which gripper to use (hande or 2f_85)
time_step = 0.1                         # time step size (seconds)
n_sample = 1000                         # number of images captured by camera


if hardware_control:
    with KinovaStationHardwareInterface(n_dof) as station:
    # Note that unlike the simulation station, the hardware station needs to be used within a 'with' block.
    # This is to allow for cleaner error handling, since the connection with the hardware needs to be
    # closed properly even if there is an error (e.g. KeyboardInterrupt) during execution.
        
        """ Connect Station """
        builder = DiagramBuilder() # Create a Drake diagram
        station = builder.AddSystem(station) # Connect Station
        
        """ Connect Loggers """
        # Connect the state of block to a Logger
        # state_logger = LogVectorOutput(block_system.GetOutputPort("measured_block_pose"), builder)
        # state_logger.set_name("state_logger")
        
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

        #gp_logger = LogVectorOutput(station.GetOutputPort("measured_gripper_position"), builder)
        #gp_logger.set_name("gripper_position_logger")
        #gv_logger = LogVectorOutput(station.GetOutputPort("measured_gripper_velocity"), builder)
        #gv_logger.set_name("gripper_velocity_logger")


        ''' Command Sequence & Control '''
        # pscs, controller = sequence_calibration_toward_atrium.calibration_toward_atrium(
            # initial_move, roll, pitch, yaw, width, depth, height)
        pscs, controller = sequence_calibration_toward_wall.calibration_toward_wall(
            True, 0, 0, 0, 0, 0, 0)
        
        controller = builder.AddSystem(controller)
        controller.set_name("controller")
        controller.ConnectToStation(builder, station, time_step=time_step)


        """ Build Diagram """
        diagram = builder.Build() # Build the system diagram and create default context
        diagram.set_name("toplevel_system_diagram")
        diagram_context = diagram.CreateDefaultContext()
        
        if show_toplevel_system_diagram: # Show the overall system diagram
            plt.figure()
            plot_system_graphviz(diagram,max_depth=1)
            plt.show()


        """ Simulation Sequence """
        station.go_home(name="Home") # Set default arm positions
        
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)
        simulator.set_publish_every_time_step(False)
        
        integration_scheme = "explicit_euler"
        ResetIntegratorFromFlags(simulator, integration_scheme, time_step)
        context = simulator.get_mutable_context()
        simulator.Initialize()
        simulator.AdvanceTo(controller.cs.total_duration())


        """ Collect Station Data """
        pose_log = pose_logger.FindLog(diagram_context)
        pose_log_times = pose_log.sample_times()
        pose_log_data = pose_log.data()
        print(pose_log_data.shape)
        print("")
        print("Target control frequency: %s Hz" % (1/time_step))
        print("Actual control frequency: %s Hz" % (1/time_step * simulator.get_actual_realtime_rate()))

        if show_state_plots:
            pose_fig = plt.figure(figsize=(14,8))
            pose_ax_list = []
            for i in range(6):
                pose_ax_list.append(pose_fig.add_subplot(231+i) )
                plt.plot(pose_log_times,pose_log_data[i,:])
                plt.title('Pose #' + str(i))

            plt.show()


""" Start RealSense Pipeline """
pipeline = rs.pipeline() # Declare RealSense pipeline, encapsulating the actual device and sensors
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline) # Get device product line for setting a supporting resolution
pipeline_profile = config.resolve(pipeline_wrapper) 
device = pipeline_profile.get_device()
    
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Enable depth stream
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) # Enable color stream

cfg = pipeline.start(config) # Start streaming the pipeline and get the configuration

""" Camera Intrinsics """
""" Get camera parameters [fx, fy, cx, cy] from RealSense camera
cam_params = [ 386.738, 386.738, 321.281, 238.221 ]
https://github.com/IntelRealSense/librealsense/issues/869
https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.intrinsics.html
"""
profile = cfg.get_stream(rs.stream.depth)                       # Fetch stream profile for depth stream
intrinsics = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
cam_params = [intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy]
camera_info = CameraInfo(
    width=640,
    height=480,
    focal_x=intrinsics.fx,
    focal_y=intrinsics.fy,
    center_x=intrinsics.ppx,
    center_y=intrinsics.ppy,
)


""" Pipeline Streaming """
R_cam_atag = np.zeros(3)
p_cam_atag = np.zeros((3,1))

for i in range(n_sample):
    frames = pipeline.wait_for_frames() # Wait for a coherent pair of frames: depth and color
    
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue
    
    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    depth_colormap_dim = depth_colormap.shape
    color_colormap_dim = color_image.shape

    # If depth and color resolutions are different, resize color image to match depth image for display
    if depth_colormap_dim != color_colormap_dim:
        resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
        images = np.hstack((resized_color_image, depth_colormap))
    else:
        images = np.hstack((color_image, depth_colormap))

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)
    cv2.waitKey(1)

    # Perform Apriltag detection
    gray_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
    atag = at_detector.detect(
            gray_image,
            estimate_tag_pose=True,
            camera_params=cam_params,
            tag_size= tag_size
            )
    
    """ Collect RealSense Data """
    if not atag:
        i = i - 1
    else:
        R_cam_atag = R_cam_atag + atag[0].pose_R
        p_cam_atag = p_cam_atag + atag[0].pose_t
        # print(atag)

print('\n RealSense Streamig & Apriltag Data Collecting... \n')

print('Camera Intrinsics:')
print(pipeline_profile)
print(depth_frame.profile.as_video_stream_profile().intrinsics)

pipeline.stop() # Stop streaming
R_cam_atag = R_cam_atag/n_sample
p_cam_atag = p_cam_atag/n_sample
print()
print(f"Apriltag Pose: Rotation: \n {R_cam_atag} \n")
print(f"Apriltag Pose: Translation: \n {p_cam_atag}")
print()

""" Forward Kinematics """
args = utilities.parseConnectionArguments()
with utilities.DeviceConnection.createTcpConnection(args) as router: # Create connection to the device and get the router
    base = BaseClient(router) # Create required services
        
    # Current arm's joint angles (in home position)
    try:
        print("Getting Angles for every joint...")
        input_joint_angles = base.GetMeasuredJointAngles()
    except KServerException as ex:
        print("Unable to get joint angles")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))

    try:
        print("Computing Foward Kinematics using joint angles...")
        pose = base.ComputeForwardKinematics(input_joint_angles)
    except KServerException as ex:
        print("Unable to compute forward kinematics")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))

    print("Pose calculated : ")
    print("Coordinate (x, y, z)  : ({}, {}, {})".format(pose.x, pose.y, pose.z))
    print("Theta (theta_x, theta_y, theta_z)  : ({}, {}, {})".format(pose.theta_x, pose.theta_y, pose.theta_z))
    print()
    
# Compute the camera extrinsics
R_cam_atag = RotationMatrix(R_cam_atag)
p_cam_atag = p_cam_atag
X_cam_atag = RigidTransform(R_cam_atag, p_cam_atag)

R_base_ee = RollPitchYaw(np.array([pose.theta_x, pose.theta_y, pose.theta_z]))
p_base_ee = np.array([pose.x, pose.y, pose.z])
X_base_ee = RigidTransform(R_base_ee, p_base_ee)
            
X_cam_ee = X_cam_atag

X_base_cam = X_base_ee.multiply(X_cam_ee.inverse())
print(f"\n Camera Extrinsics (X_base_cam): \n {X_base_cam} \n")

# Save the computed camera extrinsics to the npy file
with open('camera_extrinsics.npy', 'wb') as f:
    np.save(f, X_base_cam.rotation().matrix())
    np.save(f, X_base_cam.translation())