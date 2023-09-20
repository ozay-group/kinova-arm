"""
kinova_system_affineleaf.py
Description:
    Trying to support the basic meshcat visualizer from within a Drake container.
    Using this to visualize Kinova Gen3 6DoF
"""

import sys
import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph, Meshcat, MeshcatVisualizer, DiagramBuilder,
    MultibodyPlant, Parser, Simulator, RigidTransform, SpatialVelocity, RotationMatrix,
    AffineSystem, LeafSystem, LogVectorOutput, CoulombFriction, HalfSpace, RgbdSensor,
    AbstractValue, BasicVector, ConstantVectorSource, RollPitchYaw)
from pydrake.geometry import DepthRenderCamera
from manipulation.scenarios import (
    AddMultibodyTriad,
    AddRgbdSensor,
    MakeManipulationStation,
)

" Function Definitions "

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


" Class Definitions "

class KinovaHandlerSystem(LeafSystem):
    def __init__(self,plant,scene_graph):
        LeafSystem.__init__(self)

        # Constants
        self.arm_name = 'kinova_arm'

        # Add the Arm to the given plant
        self.plant = plant
        self.arm_as_model = Parser(plant=self.plant).AddModelFromFile(
                "/home/krutledg/kinova/kinova-arm/data/models/gen3_6dof/urdf/GEN3-6DOF.urdf",self.arm_name) # Save the model

        AddGround(self.plant) #Add ground to plant

        # Add the Triad
        self.scene_graph = scene_graph
        AddMultibodyTriad( plant.GetFrameByName("body"), self.scene_graph)

        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()

        # Create Input Port for the Kinova Arm System
        self.desired_pose_port = self.DeclareVectorInputPort("desired_pose",
                                                                BasicVector(6))

        # Create Output Port which should share the pose of the arm
        self.DeclareVectorOutputPort(
                "measured_arm_pose",
                BasicVector(6),
                self.SetArmPose,
                {self.time_ticket()}   # indicate that this doesn't depend on any inputs,
                )                      # but should still be updated each timestep

    def SetArmPose(self, context, output):
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


" Main Processes"

builder = DiagramBuilder() # Building Diagram

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
Parser(plant, scene_graph).AddModelFromFile("/home/krutledg/kinova/kinova-arm/data/models/gen3_6dof/urdf/GEN3-6DOF.urdf")
Parser(plant, scene_graph).AddModelFromFile("/home/krutledg/kinova/kinova-arm/data/models/simpleDesk/simpleDesk.urdf")

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
torques = builder.AddSystem(ConstantVectorSource([0, 8.6, -1.5, -0.52, -0.9, -0.67]))
builder.Connect(torques.get_output_port(), plant.get_actuation_input_port())

# Connect to Meshcat
meshcat0 = Meshcat(port=7001) # Object provides an interface to Meshcat
mCpp = MeshcatVisualizer(meshcat0)
mCpp.AddToBuilder(builder,scene_graph,meshcat0)

# Add a depth camera to view the scene
depth_camera_properties = DepthRenderCamera(
    width=640, height=480, fov_y=np.pi / 4.0
)
depth_camera = builder.AddSystem(
    RgbdSensor(scene_graph, camera_info=depth_camera_properties)
)
builder.Connect(scene_graph.get_query_output_port(), depth_camera.query_object_input_port())

rendered_geometry = scene_graph.render(camera_transform, camera)

# Convert the rendered geometry to a point cloud
point_cloud = Meshcat.MeshcatPointCloud(rendered_geometry, sample_rate=1.0)


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