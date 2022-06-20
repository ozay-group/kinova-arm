"""
tracking_block1.py
Description:
    This simulation will insert the slider block into an "empty" world in drake.
    Then the pose of the block will be controlled by the pose detected by the April tags on our real
    slider block.
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

#######################
## Class Definitions ##
#######################

class BlockTrackerSystem(LeafSystem):
    def __init__(self,plant,scene_graph,target_serial_number=145422070360):
        """

        Usage:
            BlockTrackerSystem(plant,scene_graph)
            BlockTrackerSystem(plant,scene_graph,serial_number=-1)
        """
        LeafSystem.__init__(self)

        # Constants
        self.block_name = 'block_with_slots'

        # Add the Block to the given plant
        self.plant = plant
        self.block_as_model = Parser(plant=self.plant).AddModelFromFile("/root/OzayGroupExploration/drake/manip_tests/slider/slider-block.urdf",self.block_name) # Save the model

        # Add the Camera's frame to the image
        self.scene_graph = scene_graph
        self.X_WorldCamera = RigidTransform(
            RotationMatrix.MakeXRotation(np.pi/2+np.pi/7).multiply( RotationMatrix.MakeZRotation(np.pi) ),
            np.array([0.3,1.3,0.36])
        )
        self.camera_frame = FixedOffsetFrame("camera",plant.world_frame(),self.X_WorldCamera)
        self.plant.AddFrame(self.camera_frame)
        AddMultibodyTriad(plant.GetFrameByName("camera"), self.scene_graph)

        # Add Block's Frame to the Scene
        AddMultibodyTriad( plant.GetFrameByName("body"), self.scene_graph)

        # Add Tag's Frame to the Scene
        self.X_BlockTag = RigidTransform(
            RotationMatrix.MakeXRotation(np.pi/2).multiply(RotationMatrix.MakeZRotation(np.pi/2).multiply(RotationMatrix.MakeXRotation(np.pi/2))),
            np.array([0.0,0.0254,0.0254])
        )
        self.tag_frame = FixedOffsetFrame("smile_tag",plant.GetFrameByName("body"),self.X_BlockTag)
        self.plant.AddFrame(self.tag_frame)
        AddMultibodyTriad(plant.GetFrameByName("smile_tag"), self.scene_graph, nickname="smile tag frame")

        # Create Input Port for the Slider Block System
        # self.desired_pose_port = self.DeclareVectorInputPort("desired_pose",
        #                                                         BasicVector(6))

        # Create Output Port which should share the pose of the block
        self.DeclareVectorOutputPort(
                "measured_block_pose",
                BasicVector(6),
                self.DetectBlockPose,
                {self.time_ticket()}   # indicate that this doesn't depend on any inputs,
                )                      # but should still be updated each timestep

        # Setup Realsense Camera Tracking
        self.SetupAprilTagTracker(target_serial_number)
        self.last_pose = np.zeros(6)

        # Finalize Plant
        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()

    def SetupAprilTagTracker(self, serial_number):
        """
        SetupAprilTagTracker
        Description:
            Defines a serial tag tracker which will attempt to find one of the april tags on the
            3d printed block.
        """
        self.realsense = []
        # Configure depth and color streams
        self.realsense_pipeline = rs.pipeline()
        self.realsense_config = rs.config()

        # Get device product line for setting a supporting resolution
        self.realsense_pipeline_wrapper = rs.pipeline_wrapper(self.realsense_pipeline)
        self.realsense_pipeline_profile = self.realsense_config.resolve(self.realsense_pipeline_wrapper)
        self.realsense_device = self.realsense_pipeline_profile.get_device()
        self.realsense_device_product_line = str(self.realsense_device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in self.realsense_device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            raise Exception("The BlockTrackerSystem requires Depth camera with Color sensor")
            exit(0)

        if (serial_number > 0) and (serial_number != int(self.realsense_device.get_info(rs.camera_info.serial_number))):
            raise Exception("Expected camera with serial number "+ str(serial_number)+", but received camera with serial number "+str(self.realsense_device.get_info(rs.camera_info.serial_number)) + ".")

        self.realsense_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if self.realsense_device_product_line == 'L500':
            self.realsense_config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.realsense_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Define April Tag Detector
        self.at_detector = Detector(families='tagStandard41h12',
                        nthreads=4,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)

        # camera parameters
        self.camera_params = [ 386.738, 386.738, 321.281, 238.221 ] #These are the camera's focal length and focal center. Received from running aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        self.tag_size = 0.040084375 # Measured on the tag. (See documentation on how to measure april tag sizes. A link that can help explain: https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide)
        
        # Start streaming
        self.realsense_pipeline.start(self.realsense_config)

    def DetectBlockPose(self,context,output):
        """
        DetectBlockPose
        Description:
            This function attempts to estimate the pose of the block
            using the pose of one of the detected tags.
        """

        # Constants
        at_detector = self.at_detector
        cam_params  = self.camera_params
        tag_size    = self.tag_size

        pipeline    = self.realsense_pipeline

        plant_context = self.context

        # Compute Camera Frame's Pose w.r.t. World
        X_WorldCamera = self.X_WorldCamera

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            # Not enough frame data was received, output the last pose
            output.SetFromVector(self.last_pose)
            return

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())


        # Print whether or not detector detects anything.
        gray_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
        detection_info = at_detector.detect(
                            gray_image,
                            estimate_tag_pose=True,
                            camera_params=cam_params,
                            tag_size= tag_size
                            )

        # Process detection info
        current_pose = self.last_pose # Make the default be the last pose we output
        if len(detection_info) > 0:
            # Get First struct from detection_info (which should be a list)
            first_detection = detection_info[0]
            first_rotation_matrix = RotationMatrix(first_detection.pose_R)
            first_rpy = RollPitchYaw(first_rotation_matrix)
            first_translation_vector = first_detection.pose_t

            X_CameraTag = RigidTransform(first_rotation_matrix,first_translation_vector)

            # Compute Transformation of Tag's Pose in Block's Frame (lower corner beneath small hole)
            X_BlockTag = self.X_BlockTag

            X_WorldBlock = X_WorldCamera.multiply(X_CameraTag).multiply(X_BlockTag.inverse())

            current_pose = np.hstack([
                RollPitchYaw(X_WorldBlock.rotation()).vector(),
                X_WorldBlock.translation().reshape((3,))
                ])
            output.SetFromVector(current_pose)
            self.last_pose = current_pose

            print(context.get_time())

        # Force the current free body to have the target pose/rigid transform
        self.plant.SetFreeBodyPose(
            plant_context,
            self.plant.GetBodyByName("body", self.block_as_model),
            RigidTransform(RollPitchYaw(current_pose[:3]),current_pose[3:])
        )

        self.plant.SetFreeBodySpatialVelocity(
            self.plant.GetBodyByName("body", self.block_as_model),
            SpatialVelocity(np.zeros(3),np.array([0.0,0.0,0.0])),
            plant_context
            )

        # Set The Output of the block to be the current pose
        output.SetFromVector(current_pose)


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
block_handler_system = builder.AddSystem(BlockTrackerSystem(plant,scene_graph))

# Connect Handler to Logger
# state_logger = LogVectorOutput(plant.get_body_spatial_velocities_output_port(), builder)
state_logger = LogVectorOutput(
    block_handler_system.GetOutputPort("measured_block_pose"),
    builder)
state_logger.set_name("state_logger")

# Connect to Meshcat
meshcat0 = Meshcat(port=7001) # Object provides an interface to Meshcat
mCpp = MeshcatVisualizerCpp(meshcat0)
mCpp.AddToBuilder(builder,scene_graph,meshcat0)

diagram = builder.Build()

# diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()

# Set initial pose and vectors
block_handler_system.SetInitialBlockState(diagram_context)

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