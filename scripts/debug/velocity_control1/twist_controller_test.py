"""
move_arm_to_slider.py
Description:
    Trying to build a basic simulation where we move the gripper of the Kinova Gen3 6DoF
    to the target location and grip an object.
"""

import importlib
import sys
from urllib.request import urlretrieve

# from manipulation import running_as_notebook

# Imports
import numpy as np
import pydot
from ipywidgets import Dropdown, Layout
from IPython.display import display, HTML, SVG
import matplotlib.pyplot as plt

from pydrake.all import (
    AddMultibodyPlantSceneGraph, DiagramBuilder, 
    FindResourceOrThrow, GenerateHtml, InverseDynamicsController, 
    MultibodyPlant, Parser, Simulator, RigidTransform , RotationMatrix,
    ConstantValueSource, ConstantVectorSource, AbstractValue, 
    RollPitchYaw, LogVectorOutput, plot_system_graphviz )
from pydrake.multibody.jupyter_widgets import MakeJointSlidersThatPublishOnCallback
  
# setting path
from kinova_drake.kinova_station import KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget, KinovaStation, EndEffectorTarget
from kinova_drake.observers.camera_viewer import CameraViewer

# Setting up advanced controller
sys.path.append('/root/kinova-arm/drake/')
from twist_sequence_controller import Command, CommandSequence, TwistSequenceController

###############
## Functions ##
###############

def add_loggers_to_system(builder,station):
    # Loggers force certain outputs to be computed
    wrench_logger = LogVectorOutput(station.GetOutputPort("measured_ee_wrench"),builder)
    wrench_logger.set_name("wrench_logger")

    pose_logger = LogVectorOutput(station.GetOutputPort("measured_ee_pose"), builder)
    pose_logger.set_name("pose_logger")

    twist_logger = LogVectorOutput(station.GetOutputPort("measured_ee_twist"), builder)
    twist_logger.set_name("twist_logger")

    gripper_logger = LogVectorOutput(station.GetOutputPort("measured_gripper_velocity"), builder)
    gripper_logger.set_name("gripper_logger")

    return [ pose_logger, twist_logger, wrench_logger ]

def create_pusher_slider_scenario(time_step=0.001):
    """
    Description:
        Creates the 6 degree of freedom Kinova system in simulation. Anchors it to a "ground plane" and gives it the
        RobotiQ 2f_85 gripper.
        This should also initialize the meshcat visualizer so that we can easily view the robot.
    Usage:
        builder, controller, station, diagram, diagram_context = create_pusher_slider_scenario()
    """

    builder = DiagramBuilder()

    # Constants
    gripper_type = '2f_85'

    pusher_position = [0.8,0.5,0.25]
    # pusher_rotation=[0,np.pi/2,0]
    pusher_rotation=[0,0,0]

    # Start with the Kinova Station object
    station = KinovaStation(time_step=time_step,n_dof=6)

    plant = station.plant
    scene_graph = station.scene_graph

    station.AddArmWith2f85Gripper() # Adds arm with the 2f85 gripper

    station.AddGround()
    station.AddCamera()

    # X_pusher = RigidTransform()
    # X_pusher.set_translation(pusher_position)
    # X_pusher.set_rotation(RotationMatrix(RollPitchYaw(pusher_rotation)))
    # station.AddManipulandFromFile("pusher/pusher1_urdf.urdf",X_pusher)


    # Meshcat Stuff
    # Connect to Meshcat
    station.ConnectToMeshcatVisualizer(port=7001)

    station.Finalize() # finalize station (a diagram in and of itself)

    # Start assembling the overall system diagram
    builder = DiagramBuilder()
    station = builder.AddSystem(station)

    # Setup Gripper and End Effector Command Systems
    #create_end_effector_target(EndEffectorTarget.kPose,builder,station)
    #setup_gripper_command_systems(GripperTarget.kPosition,builder,station)

    # Setup loggers
    logger_list = add_loggers_to_system(builder,station)

    # Setup Controller
    cs = setup_triangle_command_sequence()
    print(cs)
    controller = setup_controller_and_connect_to_station(cs,builder,station)

    # Build the system diagram
    diagram = builder.Build()
    diagram.set_name("system_diagram")
    diagram_context = diagram.CreateDefaultContext()

    # context = diagram.CreateDefaultContext()
    diagram.Publish(diagram_context)

    # Return station
    return builder, controller, station, diagram, diagram_context, logger_list

def setup_triangle_command_sequence():
    """
    Description:
        Creates the command sequence that we need to achieve the infinity sequence.
    Notes:
        Each velocity is a six-dimensional vector where each dimension represents the following rates:
        - [roll, pitch, yaw, x, y, z]
    """
    # Constants
    triangle_side_duration = 3.0

    # Create the command sequence object
    cs = CommandSequence([])

    # 1. Initial Command (Move to Position Above Home for 5s)
    # Compute center as was done for the infinity demo
    infinity_center = np.array([0.5, 0.0, 0.5])
    infinity_center_pose = np.zeros((6,))
    infinity_center_pose[:3] = np.array([np.pi/2,0.0,0.5*np.pi])
    infinity_center_pose[3:] = infinity_center


    # cs.append(Command(
    #     name="pause1",
    #     target_twist=infinity_center_pose,
    #     duration=3.0,
    #     gripper_value=0.0))

    # 2. Upper Right Velocity Command
    deltap1 = np.zeros((6,))
    deltap1[3:] = np.array([0.2,0.2,0])
    cs.append(Command(
        name="upper_right",
        target_twist=deltap1/triangle_side_duration,
        duration=triangle_side_duration,
        gripper_value=0.5))

    # 3. Lower Right
    deltap2 = np.zeros((6,))
    deltap2[3:] = np.array([0.2,-0.2,0])
    cs.append(Command(
        name="upper_right",
        target_twist=deltap2/triangle_side_duration,
        duration=triangle_side_duration,
        gripper_value=0.8))    

    # 4. Return to STart
    deltap3 = np.zeros((6,))
    deltap3[3:] = np.array([-0.4,0,0])
    cs.append(Command(
        name="return_home",
        target_twist= deltap3/triangle_side_duration,
        duration=triangle_side_duration,
        gripper_value=1.0))   

    # 5. Pause
    init_velocity = np.zeros((6,))
    cs.append(Command(
        name="pause2",
        target_twist=init_velocity,
        duration=1,
        gripper_value=False))

    return cs

def setup_controller_and_connect_to_station(cs,builder,station):
    """
    Description:
        Defines the controller (PID) which is a CommandSequenceController as defined in
        kinova_drake.
    Inputs:
        cs = A CommandSequence object which helps define the CommandSequenceController.
    """

    # Create the controller and connect inputs and outputs appropriately
    #Kp = 10*np.eye(6)
    Kp = np.diag([2,2,2,10,10,10])*10^4
    Kd = 0*np.sqrt(Kp)

    controller = builder.AddSystem(TwistSequenceController(cs))
    print(controller)
    controller.set_name("controller")
    controller.ConnectToStation(builder, station)

    return controller

###############################################
# Important Flags

run = True
show_station_diagram = False
show_plots = True

time_step = 0.025

###############################################

# Building Diagram
builder, controller, station, diagram, diagram_context, logger_list = create_pusher_slider_scenario(time_step=0.001)

if show_station_diagram:
    # Show the station's system diagram
    plt.figure()
    plot_system_graphviz(diagram,max_depth=1)
    plt.show()

if run:
    # # First thing: send to home position
    station.go_home(diagram,diagram_context)

    # We use a simulator instance to run the example, but no actual simulation 
    # is being done: it's all on the hardware. 
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)  # Usually, this should be set to False. Otherwise, simulation will be very slow and won't look like real time.

    # We'll use a super simple integration scheme (since we only update a dummy state)
    # and set the maximum timestep to correspond to roughly 40Hz 
    integration_scheme = "explicit_euler"
    #ResetIntegratorFromFlags(simulator, integration_scheme, time_step)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(controller.cs.total_duration())

    # In the end Plot the data
    pose_logger = logger_list[0]
    twist_logger = logger_list[1]
    pose_log = pose_logger.FindLog(diagram_context)
    twist_log = twist_logger.FindLog(diagram_context)

    if show_plots:

        # Plot Data - First Half
        fig = plt.figure()
        ax_list1 = []

        for plt_index1 in range(6):
            ax_list1.append( fig.add_subplot(231+plt_index1) )
            plt.plot(pose_log.sample_times(),pose_log.data()[plt_index1,:])
            plt.title('Pose Element #' + str(plt_index1))

        # Plot Data - Second Half
        fig = plt.figure()
        ax_list2 = []

        for plt_index2 in range(6):
            ax_list2.append( fig.add_subplot(231+plt_index2) )
            plt.plot(twist_log.sample_times(),twist_log.data()[plt_index2,:])
            plt.title('Twist Element #' + str(plt_index2))

        # fig = plt.figure()
        # plt.plot(log_times,pose_log.data()[-1,:])

        plt.show()