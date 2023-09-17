"""
config_demo.py
Description:
    Trying to support the basic meshcat visualizer from within a Drake container.
    Using this to visualize Kinova Gen3 6DoF
"""

# Setting path
import sys
sys.path.append('..')

# Imports
import importlib
from urllib.request import urlretrieve

import numpy as np
import pydot

from ipywidgets import Dropdown, Layout
from IPython.display import display, HTML, SVG

from pydrake.all import (
    AddMultibodyPlantSceneGraph, Meshcat, MeshcatVisualizer, DiagramBuilder,
    FindResourceOrThrow, GenerateHtml, InverseDynamicsController,
    MultibodyPlant, Parser, Simulator, RigidTransform, SpatialVelocity, RotationMatrix,
    AffineSystem, Diagram, LeafSystem, LogVectorOutput, CoulombFriction, HalfSpace,
    AbstractValue, BasicVector, ConstantValueSource, ConstantVectorSource,
    RollPitchYaw, LogVectorOutput )
from pydrake.multibody.jupyter_widgets import MakeJointSlidersThatPublishOnCallback

from pydrake.geometry import (Cylinder, GeometryInstance,
                                MakePhongIllustrationProperties)

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

def AddMultibodyTriad(frame, scene_graph, length=.25, radius=0.01, opacity=1.):
    plant = frame.GetParentPlant()
    AddTriad(plant.get_source_id(),
             plant.GetBodyFrameIdOrThrow(frame.body().index()), scene_graph,
             length, radius, opacity, frame.GetFixedPoseInBodyFrame())

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

#######################
## Class Definitions ##
#######################

class BlockHandlerSystem(LeafSystem):
    def __init__(self,plant,scene_graph):
        LeafSystem.__init__(self)

        # Constants
        self.block_name = 'block_with_slots'

        # Add the Block to the given plant
        self.plant = plant
        self.block_as_model = Parser(plant=self.plant).AddModelFromFile(
                "/root/OzayGroupExploration/drake/manip_tests/slider/slider-block.urdf",
                self.block_name) # Save the model

        AddGround(self.plant) #Add ground to plant

        # Add the Triad
        self.scene_graph = scene_graph
        AddMultibodyTriad( plant.GetFrameByName("body"), self.scene_graph)

        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()

        # Create Input Port for the Slider Block System
        self.desired_pose_port = self.DeclareVectorInputPort("desired_pose",
                                                                BasicVector(6))

        # Create Output Port which should share the pose of the block
        self.DeclareVectorOutputPort(
                "measured_block_pose",
                BasicVector(6),
                self.SetBlockPose,
                {self.time_ticket()}   # indicate that this doesn't depend on any inputs,
                )                      # but should still be updated each timestep

#######################
##  Main Processes   ##
#######################

# Building Diagram
builder = DiagramBuilder()

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
Parser(plant, scene_graph).AddModelFromFile("../models/gen3_6dof/urdf/GEN3-6DOF.urdf")
Parser(plant, scene_graph).AddModelFromFile("../models/simpleDesk2/simpleDesk2.urdf")

# Weld table to world frame, with rotation about x
p_RightTableO = [0, 0, 0]
R_RightTableO = RotationMatrix.MakeXRotation(np.pi/2.0)
X_WorldTable = RigidTransform(R_RightTableO,p_RightTableO)
plant.WeldFrames(
    plant.world_frame(), plant.GetFrameByName("simpleDesk"),X_WorldTable)

# Weld robot to table, with translation in x, y and z
p_PlaceOnTableO = [0.75,0.75,-0.20]
R_PlaceOnTableO = RotationMatrix.MakeXRotation(-np.pi/2.0)
X_TableRobot = RigidTransform(R_PlaceOnTableO,p_PlaceOnTableO)
plant.WeldFrames(
    plant.GetFrameByName("simpleDesk"),plant.GetFrameByName("base_link"),X_TableRobot)

plant.Finalize()

# Draw the frames
for body_name in ["base_link", "shoulder_link", "bicep_link", "forearm_link",
                  "spherical_wrist_1_link", "spherical_wrist_2_link", "bracelet_with_vision_link", "end_effector_link"]:
    AddMultibodyTriad(plant.GetFrameByName(body_name), scene_graph)

# Actuator inputs
torques = builder.AddSystem(ConstantVectorSource([0.02, 0.02, 0.1, 0.1, 0.1, 0.05]))
builder.Connect(torques.get_output_port(), plant.get_actuation_input_port())

# Connect to Meshcat
meshcat0 = Meshcat(port=7001) # Object provides an interface to Meshcat
mCpp = MeshcatVisualizer(meshcat0)
mCpp.AddToBuilder(builder,scene_graph,meshcat0)

diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
# diagram.Publish(context)

# Set up simulation
simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

# Run simulation
simulator.Initialize()
simulator.AdvanceTo(15.0)

# #Wait at end
# while True:
#     1
