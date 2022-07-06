"""
hw_car_sliding1.py
Description:
    Trying to build a script that will slide our cars with the maximum
    allowable cartesian velocity (0.5 m/s).
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
    RollPitchYaw, LogVectorOutput, plot_system_graphviz,
    ResetIntegratorFromFlags )
from pydrake.multibody.jupyter_widgets import MakeJointSlidersThatPublishOnCallback
  
# setting path
sys.path.append('/root/kinova_drake/')
from kinova_station import KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget, KinovaStation, EndEffectorTarget
from observers.camera_viewer import CameraViewer

# Setting up advanced controller
sys.path.append('/root/kinova-arm/drake/')
from partial_state_controller import PartialStateCommand, PSCSequence, HardwarePSCSequenceController

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

def create_sliding_scenario(station,time_step=0.001):
    """
    Description:
        Creates the 6 degree of freedom Kinova system in HARDWARE. Anchors it to a "ground plane" and gives it the
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

    # Extract some fields from Kinova_station
    # plant = station.plant
    # scene_graph = station.scene_graph

    # Meshcat Stuff
    # Connect to Meshcat
    # station.ConnectToMeshcatVisualizer(port=7001)

    # station.Finalize() # finalize station (a diagram in and of itself)

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
    controller = setup_controller_and_connect_to_station(cs,builder,station,time_step)

    # Log Controller Command Output
    # - For some reason, I can't log the command_type port. It is an AbstractValue port which seems to matter.

    ee_command_logger = LogVectorOutput(controller.GetOutputPort("ee_command"), builder)
    ee_command_logger.set_name("ee_command_logger")
    # ee_command_type_logger = LogAbstractOutput(controller.GetOutputPort("ee_command_type"), builder)
    # ee_command_type_logger.set_name("ee_command_type_logger")
    logger_list.append(ee_command_logger)
    # logger_list.append(ee_command_type_logger)

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
    triangle_side_duration = 5.0

    # Create the command sequence object
    ccs = PSCSequence([])

    # 1. Initial Command (Move to Position Above Home for 5s)
    # Compute center as was done for the infinity demo
    infinity_center = np.array([0.5, 0.0, 0.2])
    infinity_center_pose = np.zeros((6,))
    infinity_center_pose[:3] = np.array([np.pi/2,0.0,0.5*np.pi])
    infinity_center_pose[3:] = infinity_center


    ccs.append(PartialStateCommand(
        name="pause1",
        target_type= EndEffectorTarget.kPose,
        target_value=infinity_center_pose,
        duration=5.0,
        gripper_value=0.0))

    # 2. Upper Right Velocity Command
    deltap1 = np.zeros((6,))
    deltap1[3:] = np.array([0.15,0.15,0])
    ccs.append(PartialStateCommand(
        name="upper_right",
        target_type = EndEffectorTarget.kTwist,
        target_value=deltap1/triangle_side_duration,
        duration=triangle_side_duration,
        gripper_value=0.5))

    # 3. Lower Right
    deltap2 = np.zeros((6,))
    deltap2[3:] = np.array([0.15,-0.15,0])
    ccs.append(PartialStateCommand(
        name="upper_right",
        target_type = EndEffectorTarget.kTwist,
        target_value=deltap2/triangle_side_duration,
        duration=triangle_side_duration,
        gripper_value=0.8))    

    # 4. Return to STart
    deltap3 = np.zeros((6,))
    deltap3[3:] = np.array([-0.3,0,0])
    ccs.append(PartialStateCommand(
        name="return_home",
        target_type = EndEffectorTarget.kTwist,
        target_value= deltap3/triangle_side_duration,
        duration=triangle_side_duration,
        gripper_value=1.0))   

    # 5. Pause
    init_velocity = np.zeros((6,))
    ccs.append(PartialStateCommand(
        name="pause2",
        target_type = EndEffectorTarget.kTwist,
        target_value=init_velocity,
        duration=1.0,
        gripper_value=1.0))

    return ccs

def setup_controller_and_connect_to_station(cs,builder,station,time_step):
    """
    Description:
        Defines the controller (PID) which is a CommandSequenceController as defined in
        kinova_drake.
    Inputs:
        cs = A CommandSequence object which helps define the CommandSequenceController.
    """

    # Create the controller and connect inputs and outputs appropriately
    #Kp = 10*np.eye(6)
    Kp = np.diag([10.0,10,10,2,2,2])*np.power(10.0,-2)
    #Kd = 0*np.sqrt(Kp)

    controller = builder.AddSystem(HardwarePSCSequenceController(
        cs))
    controller.set_name("PSC Controller")
    controller.ConnectToStation(builder, station,time_step=time_step)

    return controller

###############################################
# Important Flags

run = True
show_station_diagram = False
show_plots = True

time_step = 0.1

n_dof = 6

###############################################

with KinovaStationHardwareInterface(n_dof) as station:
    # Building Diagram
    builder, controller, station, diagram, diagram_context, logger_list = create_sliding_scenario(station,time_step=time_step)

    if show_station_diagram:
        # Show the station's system diagram
        plt.figure()
        plot_system_graphviz(diagram,max_depth=1)
        plt.show()

    if run:
        # # First thing: send to home position
        station.go_home()

        # We use a simulator instance to run the example, but no actual simulation 
        # is being done: it's all on the hardware. 
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)
        simulator.set_publish_every_time_step(False)  # Usually, this should be set to False. Otherwise, simulation will be very slow and won't look like real time.

        # We'll use a super simple integration scheme (since we only update a dummy state)
        # and set the maximum timestep to correspond to roughly 40Hz 
        integration_scheme = "explicit_euler"
        ResetIntegratorFromFlags(simulator, integration_scheme, time_step)

        # Run simulation
        simulator.Initialize()
        simulator.AdvanceTo(controller.cs.total_duration())

        # In the end Plot the data
        pose_logger = logger_list[0]
        twist_logger = logger_list[1]
        ee_command_logger = logger_list[3]

        pose_log = pose_logger.FindLog(diagram_context)
        twist_log = twist_logger.FindLog(diagram_context)
        ee_command_log = ee_command_logger.FindLog(diagram_context)
        # ee_command_type_log = ee_command_type_logger.FindLog(diagram_context)

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
            # plt.plot(ee_command_type_log.sample_times(),ee_command_type_log.data()[:])

            # Plot Data - Command
            fig = plt.figure()
            ax_list3 = []

            for plt_index3 in range(6):
                ax_list3.append( fig.add_subplot(231+plt_index3) )
                plt.plot(ee_command_log.sample_times(),ee_command_log.data()[plt_index3,:])
                plt.title('Command Element #' + str(plt_index3))

            plt.show()