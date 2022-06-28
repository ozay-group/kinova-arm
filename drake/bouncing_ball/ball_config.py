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

builder = DiagramBuilder()

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)

ball_as_model = Parser(plant=plant).AddModelFromFile(
            "/root/OzayGroupExploration/drake/bouncing_ball/ball_model/ball.urdf",
            'ball')
AddGround(plant)

plant.Finalize()

state_logger = LogVectorOutput(plant.get_state_output_port(ball_as_model), builder)
state_logger.set_name("state_logger")

# Connect to Meshcat
meshcat0 = Meshcat(port=7001) # Object provides an interface to Meshcat
mCpp = MeshcatVisualizerCpp(meshcat0)
mCpp.AddToBuilder(builder,scene_graph,meshcat0)

diagram = builder.Build()

diagram_context = diagram.CreateDefaultContext()
p_WBlock = [0.0, 0.0, 0.1]
R_WBlock = RotationMatrix.MakeXRotation(np.pi/2.0)
X_WBlock = RigidTransform(R_WBlock,p_WBlock)
plant.SetFreeBodyPose(
            plant.GetMyContextFromRoot(diagram_context),
            plant.GetBodyByName("ball", ball_as_model),
            X_WBlock)
plant.SetFreeBodySpatialVelocity(
            plant.GetBodyByName("ball", ball_as_model),
            SpatialVelocity(np.zeros(3),np.array([0.0,0.0,0.0])),
            plant.GetMyContextFromRoot(diagram_context))

diagram.Publish(diagram_context)

# Set up simulation
simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

# Run simulation
simulator.Initialize()
simulator.AdvanceTo(30.0)

# Collect Data
state_log = state_logger.FindLog(diagram_context)
log_times  = state_log.sample_times()
state_data = state_log.data()
print(state_data.shape)