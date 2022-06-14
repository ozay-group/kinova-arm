"""
autonomous_slider_pose_changing.py
Description:
    In this script, we initialize the slider block into the scene and
    then control its position using an Affine System block from Drake.
    This should prove that we can arbitrarily set object positions in the
    simulator based on runtime values of other blocks in the world.
"""

import importlib
import sys
from urllib.request import urlretrieve

# Start a single meshcat server instance to use for the remainder of this notebook.
server_args = []
from meshcat.servers.zmqserver import start_zmq_server_as_subprocess
proc, zmq_url, web_url = start_zmq_server_as_subprocess(server_args=server_args)

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
    AbstractValue, BasicVector, RollPitchYaw, ConstantVectorSource )
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
        self.block_as_model = Parser(plant=self.plant).AddModelFromFile("/root/kinova-arm/drake/manip_tests/slider/slider-block.urdf",self.block_name) # Save the model

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

        # Build the diagram

    def SetBlockPose(self, context, output):
        """
        Description:
            This function sets the desired pose of the block.
        """

        # Get Desired Pose from Port
        plant_context = self.context
        pose_as_vec = self.desired_pose_port.Eval(context)

        self.plant.SetFreeBodyPose(
            plant_context,
            self.plant.GetBodyByName("body", self.block_as_model),
            RigidTransform(RollPitchYaw(pose_as_vec[:3]),pose_as_vec[3:])
        )

        self.plant.SetFreeBodySpatialVelocity(
            self.plant.GetBodyByName("body", self.block_as_model),
            SpatialVelocity(np.zeros(3),np.array([0.0,0.0,0.0])),
            plant_context
            )

        X_WBlock = self.plant.GetFreeBodyPose(
            plant_context,
            self.plant.GetBodyByName("body", self.block_as_model)
        )

        pose_as_vector = np.hstack([RollPitchYaw(X_WBlock.rotation()).vector(), X_WBlock.translation()])

        # Create Output
        output.SetFromVector(pose_as_vector)

    def SetInitialBlockState(self,diagram_context):
        """
        Description:
            Sets the initial position to be slightly above the ground (small, positive z value)
            to be .
        """

        # Set Pose
        p_WBlock = [0.0, 0.0, 0.2]
        R_WBlock = RotationMatrix.MakeXRotation(np.pi/2.0) # RotationMatrix.MakeXRotation(-np.pi/2.0)
        X_WBlock = RigidTransform(R_WBlock,p_WBlock)
        self.plant.SetFreeBodyPose(
            self.plant.GetMyContextFromRoot(diagram_context),
            self.plant.GetBodyByName("body", self.block_as_model),
            X_WBlock)

        # Set Velocities
        self.plant.SetFreeBodySpatialVelocity(
            self.plant.GetBodyByName("body", self.block_as_model),
            SpatialVelocity(np.zeros(3),np.array([0.0,0.0,0.0])),
            self.plant.GetMyContextFromRoot(diagram_context))

show_plots = True

# Building Diagram
time_step = 0.002

builder = DiagramBuilder()

# plant = builder.AddSystem(MultibodyPlant(time_step=time_step)) #Add plant to diagram builder
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
block_handler_system = builder.AddSystem(BlockHandlerSystem(plant,scene_graph))

# Connect Handler to Logger
# state_logger = LogVectorOutput(plant.get_body_spatial_velocities_output_port(), builder)
state_logger = LogVectorOutput(
    block_handler_system.GetOutputPort("measured_block_pose"),
    builder)
state_logger.set_name("state_logger")

# Connect System To Handler
# Create system that outputs the slowly updating value of the pose of the block.
A = np.zeros((6,6))
B = np.zeros((6,1))
f0 = np.array([0.0,0.1,0.1,0.0,0.0,0.0])
C = np.eye(6)
D = np.zeros((6,1))
y0 = np.zeros((6,1))
x0 = np.array([0.0,0.0,0.0,0.0,0.2,0.5])
target_source2 = builder.AddSystem(
    AffineSystem(A,B,f0,C,D,y0)
    )
target_source2.configure_default_state(x0)

command_logger = LogVectorOutput(
    target_source2.get_output_port(),
    builder)
command_logger.set_name("command_logger")

# Connect the state of the block to the output of a slowly changing system.
builder.Connect(
    target_source2.get_output_port(),
    block_handler_system.GetInputPort("desired_pose"))

u0 = np.array([0.2])
affine_system_input = builder.AddSystem(ConstantVectorSource(u0))
builder.Connect(
    affine_system_input.get_output_port(),
    target_source2.get_input_port()    
)

# Connect to Meshcat
meshcat0 = Meshcat(port=7001) # Object provides an interface to Meshcat
mCpp = MeshcatVisualizerCpp(meshcat0)
mCpp.AddToBuilder(builder,scene_graph,meshcat0)

diagram = builder.Build()



# builder.Connect(
#     plant.get_state_output_port(block),
#     demux.get_input_port(0))

#Weld robot to table, with translation in x, y and z
# p_PlaceOnTable0 = [0.15,0.75,-0.20]
# R_PlaceOnTableO = RotationMatrix.MakeXRotation(-np.pi/2.0)
# X_TableRobot = RigidTransform(R_PlaceOnTableO,p_PlaceOnTable0)
# plant.WeldFrames(
#     plant.GetFrameByName("simpleDesk"),plant.GetFrameByName("base_link"),X_TableRobot)



# plant.Finalize()
# # Draw the frames
# for body_name in ["base_link", "shoulder_link", "bicep_link", "forearm_link", "spherical_wrist_1_link", "spherical_wrist_2_link", "bracelet_with_vision_link", "end_effector_link"]:
#     AddMultibodyTriad(plant.GetFrameByName(body_name), scene_graph)

# diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()

# Set initial pose and vectors
block_handler_system.SetInitialBlockState(diagram_context)

# meshcat.load()
diagram.Publish(diagram_context)


# Set up simulation
simulator = Simulator(diagram, diagram_context)
block_handler_system.context = block_handler_system.plant.GetMyMutableContextFromRoot(diagram_context)
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

command_log = command_logger.FindLog(diagram_context)
log_times_c = command_log.sample_times()
command_data = command_log.data()
print(command_data.shape)

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

    for plt_index2 in range(6):
        ax_list2.append( fig.add_subplot(231+plt_index2) )
        plt.plot(log_times_c,command_data[plt_index2,:])
        plt.title('Command #' + str(plt_index2))

    # fig = plt.figure()
    # plt.plot(log_times,state_data[-1,:])

    plt.show()