"""
tracking_block1.py
Description:
    This simulation will insert the slider block into an "empty" world in drake.
    Then the pose of the block will be controlled by the pose detected by the April tags on our real
    slider block.
"""

import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
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
    AffineSystem, Diagram, LeafSystem, LogVectorOutput, CoulombFriction, HalfSpace,
    AbstractValue, BasicVector, RollPitchYaw, ConstantVectorSource, FixedOffsetFrame )
from pydrake.multibody.jupyter_widgets import MakeJointSlidersThatPublishOnCallback

from pydrake.geometry import (Cylinder, GeometryInstance,
                                MakePhongIllustrationProperties)

import pyrealsense2 as rs
from dt_apriltags import Detector
import cv2


##########################
## Function Definitions ##
##########################

def AddTriad(source_id,
             frame_id,
             scene_graph,
             length=.25,
             radius=0.01,
             opacity=1.,
             X_FT=RigidTransform(),
             name="frame"):
    """
    Adds illustration geometry representing the coordinate frame, with the
    x-axis drawn in red, the y-axis in green and the z-axis in blue. The axes
    point in +x, +y and +z directions, respectively.
    Args:
      source_id: The source registered with SceneGraph.
      frame_id: A geometry::frame_id registered with scene_graph.
      scene_graph: The SceneGraph with which we will register the geometry.
      length: the length of each axis in meters.
      radius: the radius of each axis in meters.
      opacity: the opacity of the coordinate axes, between 0 and 1.
      X_FT: a RigidTransform from the triad frame T to the frame_id frame F
      name: the added geometry will have names name + " x-axis", etc.
    """
    # x-axis
    X_TG = RigidTransform(RotationMatrix.MakeYRotation(np.pi / 2),
                          [length / 2., 0, 0])
    geom = GeometryInstance(X_FT.multiply(X_TG), Cylinder(radius, length),
                            name + " x-axis")
    geom.set_illustration_properties(
        MakePhongIllustrationProperties([1, 0, 0, opacity]))
    scene_graph.RegisterGeometry(source_id, frame_id, geom)

    # y-axis
    X_TG = RigidTransform(RotationMatrix.MakeXRotation(np.pi / 2),
                          [0, length / 2., 0])
    geom = GeometryInstance(X_FT.multiply(X_TG), Cylinder(radius, length),
                            name + " y-axis")
    geom.set_illustration_properties(
        MakePhongIllustrationProperties([0, 1, 0, opacity]))
    scene_graph.RegisterGeometry(source_id, frame_id, geom)

    # z-axis
    X_TG = RigidTransform([0, 0, length / 2.])
    geom = GeometryInstance(X_FT.multiply(X_TG), Cylinder(radius, length),
                            name + " z-axis")
    geom.set_illustration_properties(
        MakePhongIllustrationProperties([0, 0, 1, opacity]))
    scene_graph.RegisterGeometry(source_id, frame_id, geom)

def AddMultibodyTriad(frame, scene_graph, length=.25, radius=0.01, opacity=1.,nickname="triad frame"):
    """
    AddMultibodyTriad
    Description:
        Adds a MultibodyTriad (a multibody object which expresses a free body frame)
        to the plant.
    Usage:
        AddMultibodyTriad( plant.GetFrameByName("body"), self.scene_graph)
    """
    plant = frame.GetParentPlant()
    AddTriad(plant.get_source_id(),
             plant.GetBodyFrameIdOrThrow(frame.body().index()), scene_graph,
             length, radius, opacity, frame.GetFixedPoseInBodyFrame(),
             name=nickname + " - ")

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

def CreateScenario(plant_in, scene_graph_in):
    # Create block just to have something nice in the background
    # block_as_model = Parser(plant=plant_in).AddModelFromFile("/root/kinova-arm/drake/manip_tests/slider/slider-block.urdf","sanity-block") # Save the model

    # Add Origin Frame to the Scene
    AddMultibodyTriad( plant_in.world_frame(), scene_graph_in, nickname = "world frame1")

    ############################################################3

    # Add End Effector 1 to Image
    X_WorldEndeffector1 = RigidTransform(
        np.load("test_data3/X_WorldEndeffector.npy")
    )
    ee_frame1 = FixedOffsetFrame("ee1",plant.world_frame(),X_WorldEndeffector1)
    plant.AddFrame(ee_frame1)
    AddMultibodyTriad(plant_in.GetFrameByName("ee1"), scene_graph_in, nickname = "ee frame1")

    # Add Apriltag 1 to Image
    X_WorldApriltag1 = X_WorldEndeffector1.multiply(RigidTransform(
        np.load("test_data3/X_EndeffectorApriltag.npy")
    ))

    at1 = FixedOffsetFrame("at1",plant.world_frame(),X_WorldApriltag1)
    plant.AddFrame(at1)
    AddMultibodyTriad(plant_in.GetFrameByName("at1"), scene_graph_in, nickname = "at frame1")

    # Add Camera 1 to Image
    X_WorldRealsense1 = RigidTransform(
        np.load("test_data3/X_WorldRealsense.npy")
    )

    cam1 = FixedOffsetFrame("cam1",plant.world_frame(),X_WorldRealsense1)
    plant.AddFrame(cam1)
    AddMultibodyTriad(plant_in.GetFrameByName("cam1"), scene_graph_in, nickname = "cam frame1")

    ######################################################

    # Add End Effector 2 to Image
    X_WorldEndeffector2 = RigidTransform(
        np.load("test_data4/X_WorldEndeffector.npy")
    )

    ee_frame2 = FixedOffsetFrame("ee2",plant.world_frame(),X_WorldEndeffector2)
    plant.AddFrame(ee_frame2)
    AddMultibodyTriad(plant_in.GetFrameByName("ee2"), scene_graph_in, nickname = "ee frame2")

    # Add Apriltag 2 to Image
    X_WorldApriltag2 = X_WorldEndeffector2.multiply(RigidTransform(
        np.load("test_data4/X_EndeffectorApriltag.npy")
    ))

    at2 = FixedOffsetFrame("at2",plant.world_frame(),X_WorldApriltag2)
    plant.AddFrame(at2)
    AddMultibodyTriad(plant_in.GetFrameByName("at2"), scene_graph_in, nickname = "at frame2")

    # Add Camera 2 to Image
    X_WorldRealsense2 = RigidTransform(
        np.load("test_data4/X_WorldRealsense.npy")
    )

    cam2 = FixedOffsetFrame("cam2",plant.world_frame(),X_WorldRealsense2)
    plant.AddFrame(cam2)
    AddMultibodyTriad(plant_in.GetFrameByName("cam2"), scene_graph_in, nickname = "cam frame2")

#######################
## Class Definitions ##
#######################

show_plots = True

# Building Diagram
time_step = 0.002

builder = DiagramBuilder()

# Create Scene
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
CreateScenario(plant,scene_graph)

plant.Finalize() # Finalize Plant

# Connect to Meshcat
meshcat0 = Meshcat(port=7001) # Object provides an interface to Meshcat
mCpp = MeshcatVisualizerCpp(meshcat0)
mCpp.AddToBuilder(builder,scene_graph,meshcat0)

diagram = builder.Build()

# diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
diagram.Publish(diagram_context)


# Set up simulation
simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

# Run simulation
simulator.Initialize()
simulator.AdvanceTo(15.0)