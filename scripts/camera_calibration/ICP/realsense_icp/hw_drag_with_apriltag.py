"""
hw_drag_with_apriltag.py

Description:
    Simple example of using our kinova manipulation station to drag a mini car
    at an a-priori known location. Runs on the real hardware.
    +
    This script aims to detect the apriltag in the scene captured by intel realsense camera (rgb depth camera)
    Then the pose of the object will be determined based on the pose of the apriltag
"""

# Setting path for imports
import sys
sys.path.append('../../../')

# Imports
from pydrake.all import *
from pydrake.all import (AddMultibodyPlantSceneGraph, Meshcat, MeshcatVisualizer,
                         DiagramBuilder, Simulator, LogVectorOutput)
import numpy as np
import matplotlib.pyplot as plt

import pyrealsense2 as rs
from dt_apriltags import Detector
import cv2

import tracker_system as ts
import simulation_utilities as su

from kinova_station import KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget
from controllers.basic import CommandSequenceController, CommandSequence, Command
from controllers.velocity import VelocityCommand, VelocityCommandSequence, VelocityCommandSequenceController

"""
Parameters
"""
# Make a plot of the inner workings of the station
show_station_diagram = False

# Number of DoF should be known for your robot (there are 6 and 7-DoF Versions of the Gen3)
n_dof = 6

# Make a plot of the diagram for this example, where only the inputs
# and outputs of the station are shown
show_toplevel_diagram = False

# Which gripper to use (hande or 2f_85)
gripper_type = "2f_85"

show_plots = True

time_step = 0.5

"""
Main: Building Diagram
"""
# Set up the kinova station
with KinovaStationHardwareInterface(n_dof) as station:

    if show_station_diagram:
        # Show the station's system diagram
        plt.figure()
        plot_system_graphviz(station,max_depth=1)
        plt.show()

    # Start assembling the overall system diagram
    builder = DiagramBuilder() # Create a new diagram builder
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step) # Add new Multibody plant and a new SceneGraph (for Visualization)
    block_system = builder.AddSystem(ts.BlockTrackerSystem(plant,scene_graph)) # Add a custom Tracker System to the diagram and connect them
    station = builder.AddSystem(station)

    # Connect the state of block to a Logger
    state_logger = LogVectorOutput(block_system.GetOutputPort("measured_block_pose"), builder)
    state_logger.set_name("state_logger")

    # Connect to Meshcat
    meshcat = Meshcat(port=7001) # this object provides an interface to Meshcat
    mesh_visual = MeshcatVisualizer(meshcat)
    mesh_visual.AddToBuilder(builder,scene_graph,meshcat)

    # Create the command sequence
    cs = CommandSequence([])
    cs.append(Command(
        name="initial move",
        target_pose=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.4, -0.4, 0.25]),
        duration=4,
        gripper_closed=False))
    cs.append(Command(
        name="pregrasp",
        target_pose=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.375, -0.4, 0.0125]),
        duration=1,
        gripper_closed=False))
    cs.append(Command(
        name="pause",
        target_pose=np.array([1.0*np.pi, 0.0*np.pi, 1.05*np.pi, 0.375, -0.4, 0.0125]),
        duration=0.5,
        gripper_closed=False))
    cs.append(Command(
        name="grasp",
        target_pose=np.array([1.0*np.pi, 0.0*np.pi, 1.05*np.pi, 0.35, -0.4, 0.0125]),
        duration=1,
        gripper_closed=True))
    cs.append(Command(
        name="accelerate",
        target_pose=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.325, 0.7, 0.0125]),
        duration=0.75,
        gripper_closed=True))
    cs.append(Command(
        name="release",
        target_pose=np.array([1.0*np.pi, 0.0, 1.0*np.pi, 0.3, 0.3, 0.5]),
        duration=0.5,
        gripper_closed=False))

    # Create the controller and connect inputs and outputs appropriately
    Kp = np.diag([100, 100, 100, 200, 200, 200])  # high gains needed to overcome
    Kd = 2*np.sqrt(Kp)                            # significant joint friction

    controller = builder.AddSystem(CommandSequenceController(
        cs,
        command_type=EndEffectorTarget.kWrench,  # wrench commands work best on hardware
        Kp=Kp,
        Kd=Kd))
    controller.set_name("controller")
    controller.ConnectToStation(builder, station)

    # Build the Diagram and create Default Context
    diagram = builder.Build() # Assemble all elements together
    diagram.set_name("system_diagram")
    diagram_context = diagram.CreateDefaultContext()

    if show_toplevel_diagram:
        # Show the overall system diagram
        plt.figure()
        plot_system_graphviz(diagram,max_depth=1)
        plt.show()

    # Set default arm positions
    station.go_home(name="Home")

    # Set initial pose and vectors
    block_system.SetInitialBlockState(diagram_context)

    # Set up simulation
    simulator = Simulator(diagram, diagram_context)
    block_system.context = block_system.plant.GetMyMutableContextFromRoot(diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    integration_scheme = "explicit_euler"
    time_step = 0.5
    ResetIntegratorFromFlags(simulator, integration_scheme, time_step)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(13.0)

    # Collect Data
    state_log = state_logger.FindLog(diagram_context)
    log_times  = state_log.sample_times()
    state_data = state_log.data()
    print(state_data.shape)

    # Print rate data
    print("")
    print("Target control frequency: %s Hz" % (1/time_step))
    print("Actual control frequency: %s Hz" % (1/time_step * simulator.get_actual_realtime_rate()))

    if show_plots:
        fig = plt.figure()
        ax_list1 = []

        for plt_index1 in range(6):
            ax_list1.append( fig.add_subplot(231+plt_index1) )
            plt.plot(log_times,state_data[plt_index1,:])
            plt.title('State #' + str(plt_index1))
        
        plt.show()