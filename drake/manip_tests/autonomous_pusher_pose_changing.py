"""
basictest4.py
Description:
    Trying to support the basic meshcat visualizer from within a Drake container.
    Using this to visualize Kinova Gen3 6DoF
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
    AddMultibodyPlantSceneGraph, ConnectMeshcatVisualizer, DiagramBuilder, 
    FindResourceOrThrow, GenerateHtml, InverseDynamicsController, 
    MultibodyPlant, Parser, Simulator, RigidTransform , RotationMatrix,
    AffineSystem, Diagram, LeafSystem, LogVectorOutput, CoulombFriction, HalfSpace )
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

    p_GroundOrigin = [0, 0.0, -0.5]
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
            nontransparent_color)  # transparent

#######################
## Class Definitions ##
#######################

class BlockWithSlots(Diagram):
    """
    """
    def __init__(self, time_step=0.002):
        """
        __init__
        Description:
            Called whenever the block is initialized.
        Usage:
            block1 = BlockWithSlots(time_step=0.004)
        """
        Diagram.__init__(self)
        self.set_name("Slider-Block-with-slots-for-masses")

        self.builder = DiagramBuilder()

        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=1e-4)
        self.scene_graph.set_name("scene_graph")

        # Add Block Model from File
        self.block_as_model = Parser(plant=self.plant).AddModelFromFile("/root/OzayGroupExploration/drake/manip_tests/slider/slider-block.urdf",'block_with_slots')
        print(self.block_as_model)
        self.plant.WeldFrames(self.plant.world_frame(),
                              self.plant.GetFrameByName("base",self.block_as_model))

        # Because we've named a model as 'block_with_slots' there should exist:
        #   - an output called block_with_slots_continuous_state

    def ConnectToMeshcatVisualizer(self, zmq_url=None):
        if zmq_url is None:
            # Start meshcat server. This saves the step of opening meshcat separately,
            # but does mean you need to refresh the page each time you re-run something.
            from meshcat.servers.zmqserver import start_zmq_server_as_subprocess
            proc, zmq_url, web_url = start_zmq_server_as_subprocess()

        # Defining self.meshcat in this way allows us to connect to 
        # things like a point-cloud visualizer later
        self.meshcat = ConnectMeshcatVisualizer(builder=self.builder,
                                 zmq_url = zmq_url,
                                 scene_graph=self.scene_graph,
                                 output_port=self.scene_graph.get_query_output_port())

    def Finalize(self):

        # Finalize plant
        self.plant.Finalize()

        # Set up the scene graph
        # self.builder.Connect(
        #         self.scene_graph.get_query_output_port(),
        #         self.plant.get_geometry_query_input_port())
        # self.builder.Connect(
        #         self.plant.get_geometry_poses_output_port(),
        #         self.scene_graph.get_source_pose_port(self.plant.get_source_id()))

        self.builder.BuildInto(self)

class BlockHandlerSystem(LeafSystem):
    def __init__(self,plant):
        LeafSystem.__init__(self)

        # Constants
        self.block_name = 'block_with_slots'

        # Add the Block to the given plant
        self.plant = plant
        self.block_as_model = Parser(plant=self.plant).AddModelFromFile("/root/OzayGroupExploration/drake/manip_tests/slider/slider-block.urdf",self.block_name) # Save the model

        # Create Input Port for the Slider Block System
        self.desired_pose_port = self.state_input_port = self.DeclareAbstractInputPort("desired_pose",
                                                            AbstractValue.Make(6))

        # Create Output Port which should share the pose of the block
        self.DeclareVectorOutputPort(
                "measured_block_pose",
                BasicVector(6),
                self.SetBlockPose,
                {self.time_ticket()}   # indicate that this doesn't depend on any inputs,
                )                      # but should still be updated each timestep

    def SetBlockPose(self, context, output):
        """
        Description:
            This function sets the desired pose of the block.
        """

        # Get Desired Pose from Port
        pose_as_vec = self.desired_pose_port.Eval()
        
        SetFreeBodyPose(
            self.plant,
            context,
            self.plant.GetBodyByName("body", self.block_as_model),
            self.ge
        )

# Building Diagram
time_step = 0.002

builder = DiagramBuilder()

# plant = builder.AddSystem(MultibodyPlant(time_step=time_step)) #Add plant to diagram builder
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
block_as_model = Parser(plant=plant).AddModelFromFile("/root/OzayGroupExploration/drake/manip_tests/slider/slider-block.urdf",'block_with_slots') # Save the model into the plant.

AddGround(plant)

plant.Finalize()

# Connect Block to Logger
# state_logger = LogVectorOutput(plant.get_body_spatial_velocities_output_port(), builder)
state_logger = LogVectorOutput(plant.get_state_output_port(block_as_model), builder)
state_logger.set_name("state_logger")

# Connect to Meshcat
meshcat = ConnectMeshcatVisualizer(builder=builder,
                                    zmq_url = zmq_url,
                                    scene_graph=scene_graph,
                                    output_port=scene_graph.get_query_output_port())

diagram = builder.Build()

# Create system that outputs the slowly updating value of the pose of the block.
A = np.zeros((6,6))
B = np.zeros((6,1))
f0 = np.array([0.0,0.0,0.0,1.2,0.0,0.0])
C = np.eye(6)
D = np.zeros((6,1))
y0 = np.zeros((6,1))
x0 = y0
# target_source2 = builder.AddSystem(
#     AffineSystem(A,B,f0,C,D,y0)
#     )
# target_source2.configure_default_state(x0)

# Connect the state of the block to the output of a slowly changing system.
# builder.Connect(
#     target_source2.get_output_port(),
#     block1.plant.GetInputPort("slider_block"))

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

# SetFreeBodyPose
# p_WBlock = [0, 0.0, 0.5]
# R_WBlock = RotationMatrix.MakeXRotation(0.0).multiply(
#         RotationMatrix.MakeZRotation(0.0))
# X_WBlock = RigidTransform(R_WBlock,p_WBlock)
# plant.SetFreeBodyPose(diagram_context,block_as_model,X_WBlock)

meshcat.load()
diagram.Publish(diagram_context)

if True:
    # Set up simulation
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(10.0)

    # Collect Data
    state_log = state_logger.FindLog(diagram_context)
    log_times  = state_log.sample_times()
    state_data = state_log.data()
    print(state_data.shape)

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