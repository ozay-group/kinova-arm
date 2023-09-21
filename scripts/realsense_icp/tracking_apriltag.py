"""
tracking_apriltag.py

Description:
    This script aims to detect the apriltag in the scene captured by intel realsense camera (rgb depth camera)
    Then the pose of the object will be determined based on the pose of the apriltag
"""

# Setting path for imports
import sys
sys.path.append('../')

# Imports
import matplotlib.pyplot as plt

from pydrake.all import (AddMultibodyPlantSceneGraph, Meshcat, MeshcatVisualizer,
                         DiagramBuilder, Simulator, LogVectorOutput)

import tracker_system as ts
import simulation_utilities as su

import numpy as np
from pydrake.all import *
import pyrealsense2 as rs
from dt_apriltags import Detector
import cv2

show_plots = True
time_step = 1e-2

"""
Main: Building Diagram
"""

# Create a new diagram builder
builder = DiagramBuilder()

# Add new Multibody plant and a new SceneGraph (for Visualization) to the diagram and connect them
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step)

# Add a custom Tracker System to the diagram and connect them
block_system = builder.AddSystem(ts.BlockTrackerSystem(plant,scene_graph))

# Connect the state of block to a Logger
state_logger = LogVectorOutput(block_system.GetOutputPort("measured_block_pose"), builder)
state_logger.set_name("state_logger")

# Connect to Meshcat
meshcat = Meshcat(port=7001) # this object provides an interface to Meshcat
mesh_visual = MeshcatVisualizer(meshcat)
mesh_visual.AddToBuilder(builder,scene_graph,meshcat)

# Build the Diagram and create Default Context
diagram = builder.Build() # Assemble all elements together
diagram_context = diagram.CreateDefaultContext()

# Set initial pose and vectors
block_system.SetInitialBlockState(diagram_context)

# Set up simulation
simulator = Simulator(diagram, diagram_context)
block_system.context = block_system.plant.GetMyMutableContextFromRoot(diagram_context)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

# Run simulation
simulator.Initialize()
simulator.AdvanceTo(15.0)

# Collect Data
state_log = state_logger.FindLog(diagram_context)
log_times  = state_log.sample_times()
state_data = state_log.data()
print(state_data.shape)

# command_log = su.command_logger.FindLog(diagram_context)
# log_times_c = command_log.sample_times()
# command_data = command_log.data()
# print(command_data.shape)

if show_plots:

    # Plot Data - First Half
    fig = plt.figure()
    ax_list1 = []

    for plt_index1 in range(6):
        ax_list1.append( fig.add_subplot(231+plt_index1) )
        plt.plot(log_times,state_data[plt_index1,:])
        plt.title('State #' + str(plt_index1))

    # Plot Data - Second Half
    # fig = plt.figure()
    # ax_list2 = []

    # for plt_index2 in range(6):
    #     ax_list2.append( fig.add_subplot(231+plt_index2) )
    #     plt.plot(log_times_c,command_data[plt_index2,:])
    #     plt.title('Command #' + str(plt_index2))

    plt.show()