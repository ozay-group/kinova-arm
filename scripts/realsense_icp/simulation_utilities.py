"""
simulation_utilities.py
Description:
    This simulation will insert the slider block into an "empty" world in drake.
    Then the pose of the block will be controlled by the pose detected by the April tags on our real
    slider block.
"""

# Setting path for imports
import sys
sys.path.append('../')

# Imports
import importlib
from urllib.request import urlretrieve

import numpy as np
import pydot
from ipywidgets import Dropdown, Layout
from IPython.display import display, HTML, SVG

import matplotlib.pyplot as plt

from pydrake.all import (
    AddMultibodyPlantSceneGraph, Meshcat, MeshcatVisualizer, DiagramBuilder,
    FindResourceOrThrow, GenerateHtml, InverseDynamicsController,
    MultibodyPlant, Parser, Simulator, RigidTransform , SpatialVelocity, RotationMatrix,
    AffineSystem, Diagram, LeafSystem, LogVectorOutput, CoulombFriction, HalfSpace,
    AbstractValue, BasicVector, RollPitchYaw, ConstantVectorSource, FixedOffsetFrame )
from pydrake.multibody.jupyter_widgets import MakeJointSlidersThatPublishOnCallback
from pydrake.geometry import (Cylinder, GeometryInstance, MakePhongIllustrationProperties)

import pyrealsense2 as rs
from dt_apriltags import Detector
import cv2

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