"""
slider_init.py
Description:
    Initialize the slider object into the world.
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
    AddMultibodyPlantSceneGraph, Meshcat, MeshcatVisualizerCpp, DiagramBuilder, 
    FindResourceOrThrow, GenerateHtml, InverseDynamicsController, 
    MultibodyPlant, Parser, Simulator, RigidTransform , SpatialVelocity, RotationMatrix,
    AffineSystem, Diagram, LeafSystem, LogVectorOutput, CoulombFriction, HalfSpace )
from pydrake.multibody.jupyter_widgets import MakeJointSlidersThatPublishOnCallback

from pydrake.geometry import (Cylinder, GeometryInstance,
                                MakePhongIllustrationProperties)

##########################
## Function Definitions ##
##########################

def AddGround(plant):
    """
    Add a flat ground with friction
    """

    # Constants
    transparent_color = np.array([0.5,0.5,0.5,0])
    nontransparent_color = np.array([0.5,0.5,0.5,0.1])

    p_GroundOrigin = [0, 0.0, 0.0]
    R_GroundOrigin = RotationMatrix.MakeXRotation(0.0)
    X_GroundOrigin = RigidTransform(R_GroundOrigin,p_GroundOrigin)

    # Set Up Ground on Plant

    surface_friction = CoulombFriction(
            static_friction = 0.7,
            dynamic_friction = 0.5)
    plant.RegisterCollisionGeometry(
            plant.world_body(),
            X_GroundOrigin,
            HalfSpace(),
            "ground_collision",
            surface_friction)
    plant.RegisterVisualGeometry(
            plant.world_body(),
            X_GroundOrigin,
            HalfSpace(),
            "ground_visual",
            transparent_color)  # transparent

## Constants ##

show_plots = True

# Building Diagram
time_step = 0.002

builder = DiagramBuilder()

# plant = builder.AddSystem(MultibodyPlant(time_step=time_step)) #Add plant to diagram builder
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
block_as_model = Parser(plant=plant).AddModelFromFile("/root/kinova-arm/drake/manip_tests/slider/slider-block.urdf",'block_with_slots') # Save the model into the plant.

AddGround(plant)

plant.Finalize()

# Connect Block to Logger
# state_logger = LogVectorOutput(plant.get_body_spatial_velocities_output_port(), builder)
state_logger = LogVectorOutput(plant.get_state_output_port(block_as_model), builder)
state_logger.set_name("state_logger")

# Connect to Meshcat
meshcat0 = Meshcat(port=7001) # Object provides an interface to Meshcat
mCpp = MeshcatVisualizerCpp(meshcat0)
mCpp.AddToBuilder(builder,scene_graph,meshcat0)

diagram = builder.Build()

# diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()

# SetFreeBodyPose
p_WBlock = [0.0, 0.0, 0.1]
R_WBlock = RotationMatrix.MakeXRotation(np.pi/2.0) # RotationMatrix.MakeXRotation(-np.pi/2.0)
X_WBlock = RigidTransform(R_WBlock,p_WBlock)
plant.SetFreeBodyPose(plant.GetMyContextFromRoot(diagram_context),plant.GetBodyByName("body", block_as_model),X_WBlock)

plant.SetFreeBodySpatialVelocity(
    plant.GetBodyByName("body", block_as_model),
    SpatialVelocity(np.zeros(3),np.array([0.0,0.0,0.0])),
    plant.GetMyContextFromRoot(diagram_context))

diagram.Publish(diagram_context)


# Set up simulation
simulator = Simulator(diagram, diagram_context)
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

if show_plots:

    # Plot Data - First Half
    fig = plt.figure()
    ax_list1 = []

    for plt_index1 in range(6):
        ax_list1.append( fig.add_subplot(231+plt_index1) )
        plt.plot(log_times,state_data[plt_index1,:])
        plt.title('State #' + str(plt_index1))

    # Plot Data - Second Half
    fig = plt.figure()
    ax_list2 = []

    for plt_index2 in range(6,12):
        ax_list2.append( fig.add_subplot(231+plt_index2-6) )
        plt.plot(log_times,state_data[plt_index2,:])
        plt.title('State #' + str(plt_index2))

    fig = plt.figure()
    plt.plot(log_times,state_data[-1,:])

    plt.show()