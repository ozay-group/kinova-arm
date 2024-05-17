"""
camera_detect_car_verify.py
Description:
    This script uses the learned camera extrinsics to detect the car position, command the gripper to pick the car and slide it with maximal velocity

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
import math

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
from calibrUtilities import plot_frame
from AprilTrack import AprilTracker
from VelEstimate import velocity_estimator

def line_search(seq, target, guess = 0):
    """
    Inputs: seq : 1D monotonically increasing np.array
            target : target value
            guess: initial guess such that seq[guess] < target
    """
    left = guess
    right = len(seq)
    while(left < right-1):
        mid = math.floor((left + right)/2)
        if(seq[mid] <= target):
            left = mid
        else:
            right = mid
    return left
            

""" Apriltag Detector """

tracker = AprilTracker(n_sample=40, debug=False)

""" Configurations """

show_toplevel_system_diagram = False    # Make a plot of the diagram for inner workings of the stationn
show_state_plots = False                # Show the plot of Poses

n_dof = 6                               # number of degrees of freedom of the arm
gripper_type = "2f_85"                  # which gripper to use (hande or 2f_85)
time_step = 0.025                         # time step size (seconds)

### load extrinsics ### 
with open('camera_extrinsics.npy', 'rb') as f:
    R_world_cam = np.load(f)
    p_world_cam = np.load(f)
R_world_cam = RotationMatrix(R_world_cam)
X_world_cam = RigidTransform(R_world_cam, p_world_cam.transpose())

with KinovaStationHardwareInterface(n_dof) as station:
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

    gripper_logger = LogVectorOutput(station.GetOutputPort("measured_gripper_position"), builder)
    gripper_logger.set_name("gripper_logger")

    time_logger = LogVectorOutput(station.GetOutputPort("time"), builder)
    time_logger.set_name("time_logger")

    ''' Command Sequence & Control '''
    # pscs, controller = sequence_calibration_toward_wall.calibration_toward_wall(
    #     initial_move, roll, pitch, yaw, width, depth, height)
    cont = StaticController()

    controller = builder.AddSystem(cont)
    controller.set_name("controller")
    controller.ConnectToStation(builder, station)

    ''' Visualizer '''
    fig = plt.figure()
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)
    ax1.set_aspect('equal')
    ax1.set_xlabel("y position")
    ax2.set_xlabel("time")
    plt.ion() # turn on interactive mode (for plot.show(block=False))
    # Maximize the plot window
    mng = plt.get_current_fig_manager()
    mng.full_screen_toggle()
    plt.show()

    """ Build Diagram """
    diagram = builder.Build() # Build the system diagram and create default context
    diagram.set_name("toplevel_system_diagram")
    diagram_context = diagram.CreateDefaultContext()

    """ Simulation Sequence """
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)
    
    integration_scheme = "explicit_euler"
    ResetIntegratorFromFlags(simulator, integration_scheme, time_step)
    context = simulator.get_mutable_context()
    simulator.Initialize()
    simulator.AdvanceTo(0.05)

    cnt = 0
    slide_time_drake_idx = 0
    se3_base_cam = []
    roll = 0
    width = 0
    try: 
        while(1):
            # Detect April Tag
            t1 = time.time()
            avg_pose = tracker.detect()
            print("Detection time: ", time.time() - t1)
            if avg_pose is None:
                continue
            # ======== Move to car ===========
            #X_cam_atag = RigidTransform(RotationMatrix(avg_pose[:3, :3]), avg_pose[:3, 3])
            X_base_atag = avg_pose # X_world_cam.multiply(X_cam_atag).GetAsMatrix4()
            current_pose = np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, X_base_atag[0,3]-0.04, X_base_atag[1,3]-0.06, 0.10])
            cont.SetTargetPose(current_pose)
            simulator.AdvanceTo(context.get_time() + 0.05)
            # =========== Pregrasp Pose II ==============
            print("Pregrasp pose")
            current_pose[5] += -0.08
            cont.SetTargetPose(current_pose)
            simulator.AdvanceTo(context.get_time() + 0.05)
            # =========== Grab ==============
            print("Grab")
            cont.SetGripperPos(0.3)
            simulator.AdvanceTo(context.get_time() + 0.1)
            # =========== Move ===========
            print("Move")
            current_pose[3] = 0.35
            current_pose[4] = -0.4
            cont.SetTargetPose(current_pose)
            simulator.AdvanceTo(context.get_time() + 0.1)
            # ========== Release =========
            print("Release")
            cont.SetGripperPos(0)
            simulator.AdvanceTo(context.get_time() + 0.1)
            # =========== Move ===========
            print("Move")
            current_pose[4] += -0.07
            cont.SetTargetPose(current_pose)
            simulator.AdvanceTo(context.get_time() + 0.1)
            # =========== Grab ==============
            print("Grab")
            cont.SetGripperPos(0.5)
            simulator.AdvanceTo(context.get_time() + 0.1)

            # ======= Slide =======
            print("Sliding")
            tracker.start_track()
            release_pos = input("Enter release y position: ")
            slide_time_drake = context.get_time()
            cont.SetSlidingMode(release_pos = float(release_pos)) # slide 30 cm before release
            simulator.AdvanceTo(context.get_time() + 1)
            atag_pos = tracker.end_track()
            atag_vel = velocity_estimator(atag_pos[0], atag_pos[1], window_size=11, poly_order=1)
            # ====== Data Processing ======
            pose_log = pose_logger.FindLog(diagram_context)
            pose_log_times = pose_log.sample_times()
            slide_time_drake_idx = line_search(pose_log_times, slide_time_drake, slide_time_drake_idx)
            y_log_data = pose_log.data()[4, slide_time_drake_idx:]
            vy_log_data = twist_logger.FindLog(diagram_context).data()[4, slide_time_drake_idx:]
            gripper_log_data = gripper_logger.FindLog(diagram_context).data()[0, slide_time_drake_idx:]
            time_log = time_logger.FindLog(diagram_context)
            time_log_data = time_log.data()[0, slide_time_drake_idx:]
            # plt.plot(pose_log_times[slide_time_drake_idx:], y_log_data, label = "y position") 
            # plt.plot(pose_log_times[slide_time_drake_idx:], vy_log_data, label = "y velocity") 
            # plt.plot(pose_log_times[slide_time_drake_idx:], gripper_log_data, label = "gripper position") 
            ax1.plot(y_log_data, vy_log_data, label = "y velocity") 
            ax1.plot(y_log_data, gripper_log_data, label = "gripper position") 
            ax1.legend()
            ax2.plot(time_log_data, y_log_data, label = "EE y position")
            ax2.plot(atag_pos[0], atag_pos[1], label = "atag y position")
            ax2.plot(atag_pos[0], atag_vel[0], label = "atag y position (smoothed)")
            ax2.plot(atag_pos[0][:-1], atag_vel[1], label = "atag y velocity (esitmated)")
            ax2.legend()
            plt.draw()
            plt.pause(0.001)
            key = input("Move the car to a new position and press Enter to continue; or press Q to exit ...")
            if key == "q":
                break
            ax1.clear()
            ax2.clear()
    except KeyboardInterrupt:
        print("Interrupted by keyboard...")
    # except:
    #     print("Some exception occurred...")
        # simulator.Initialize()
        # cont.SetGripperPos(0)
        # simulator.AdvanceTo(context.get_time() + 0.5)    
        # pose_log = pose_logger.FindLog(diagram_context)
        # pose_log_data = pose_log.data()
        # current_pose = pose_log_data[:,-1].copy()
        # current_pose[5] += 0.05
        # cont.SetTargetPose(current_pose)
        # simulator.AdvanceTo(context.get_time() + 0.05)    
        # cont.SetTargetPose(np.array([np.pi/2.0, 0, np.pi/2.0, 0.57, 0, 0.42]))
        # simulator.AdvanceTo(context.get_time() + 0.05)    