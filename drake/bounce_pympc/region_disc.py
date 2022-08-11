from pydrake.all import *

import numpy as np

import default_params as params

class Region(LeafSystem):
    def __init__(self, params):
        LeafSystem.__init__(self)
        self.params = params
    
    def AddToBuilder(self, builder, scene_graph):

        # Construct a multibody plant of safety regions (boxes)
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

        # frame offset from the world frame
        frame = plant.AddFrame(FixedOffsetFrame("planar_joint_frame", plant.world_frame(), RigidTransform(RotationMatrix.MakeXRotation(np.pi/2))))

        # Construct the safety region for the ball
        s_B = plant.AddRigidBody("safety_region_b", SpatialInertia(mass=1.0, p_PScm_E=np.array([0., 0., 0.]), G_SP_E=UnitInertia(1.0, 1.0, 1.0)))
        plant.RegisterVisualGeometry(s_B, RigidTransform(), Box(1.0, 2.0, 3.0), "safety_region_b", diffuse_color=[.9, .5, .5, 0.5])
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("safety_region_b"), RigidTransform(RotationMatrix.MakeXRotation(np.pi/2)))

        # Construct the safety region for the paddle
        s_P = plant.AddRigidBody("safety_region_p", SpatialInertia(mass=1.0, p_PScm_E=np.array([0., 0., 0.]), G_SP_E=UnitInertia(1.0, 1.0, 1.0)))
        plant.RegisterVisualGeometry(s_P, RigidTransform(), Box(1.0, 2.0, 3.0), "safety_region_p", diffuse_color=[.9, .5, .5, 0.5])
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("safety_region_p"), RigidTransform(RotationMatrix.MakeXRotation(np.pi/2)))

        plant.Finalize()

        # meshcat.Delete()
        # visualizer = MeshcatVisualizerCpp.AddToBuilder(
            # builder, scene_graph, meshcat)
        builder.AddSystem(self)
        return self


class R(LeafSystem):
    def __init__(self,params,plant,scene_graph):
        LeafSystem.__init__(self)

        # Constants
        self.block_name = 'Region'

        # Add the Block to the given plant
        self.plant = plant
        self.scene_graph = scene_graph

        # Construct the safety region for the ball
        self.s_B = self.plant.AddRigidBody("safety_region_b", SpatialInertia(mass=1.0, p_PScm_E=np.array([0., 0., 0.]), G_SP_E=UnitInertia(1.0, 1.0, 1.0)))
        self.plant.RegisterVisualGeometry(self.s_B, RigidTransform(), Box(1.0, 2.0, 3.0), "safety_region_b", diffuse_color=[.9, .5, .5, 0.5])
        self.plant.WeldFrames(self.plant.world_frame(), self.plant.GetFrameByName("safety_region_b"), RigidTransform(RotationMatrix.MakeXRotation(np.pi/2)))

        # Construct the safety region for the paddle
        # self.s_P = self.plant.AddRigidBody("safety_region_p", SpatialInertia(mass=1.0, p_PScm_E=np.array([0., 0., 0.]), G_SP_E=UnitInertia(1.0, 1.0, 1.0)))
        # self.plant.RegisterVisualGeometry(self.s_P, RigidTransform(), Box(1.0, 2.0, 3.0), "safety_region_p", diffuse_color=[.9, .5, .5, 0.5])
        # self.plant.WeldFrames(self.plant.world_frame(), self.plant.GetFrameByName("safety_region_p"), RigidTransform(RotationMatrix.MakeXRotation(np.pi/2)))

        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()

        # self.SetInitialBlockState(self.context)
        # Create Input Port for the Slider Block System
        # self.desired_pose_port = self.DeclareVectorInputPort("desired_pose", BasicVector(6))

        # Create Output Port which should share the pose of the block
        #self.DeclareVectorOutputPort(
         #       "measured_block_pose",
          #      BasicVector(6),
           #     self.SetBlockPose,
            #    {self.time_ticket()}   # indicate that this doesn't depend on any inputs,
             #   )                      # but should still be updated each timestep

        # Build the diagram

    # def SetBlockPose(self, context, output):
    #     """
    #     Description:
    #         This function sets the desired pose of the block.
    #     """

    #     # Get Desired Pose from Port
    #     plant_context = self.context

    #     self.plant.SetFreeBodyPose(
    #         plant_context,
    #         self.plant.GetBodyByName("safety_region_b", self.s_B),
    #         RigidTransform(RollPitchYaw(np.zeros(3)),np.array([0., 0., 0.]))
    #         )
        

    #     self.plant.SetFreeBodySpatialVelocity(
    #         self.plant.GetBodyByName("safety_region_b", self.s_B),
    #         SpatialVelocity(np.zeros(3),np.array([0.0,0.0,0.0])),
    #         plant_context
    #         )

    #     X_WBlock = self.plant.GetFreeBodyPose(
    #         plant_context,
    #         self.plant.GetBodyByName("safety_region_b", self.s_B)
    #     )

    #     pose_as_vector = np.hstack([RollPitchYaw(X_WBlock.rotation()).vector(), X_WBlock.translation()])

    #     # Create Output
    #     output.SetFromVector(pose_as_vector)

    @staticmethod
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
            self.plant.GetBodyByName("safety_region_b", self.s_B),
            X_WBlock)

        # Set Velocities
        self.plant.SetFreeBodySpatialVelocity(
            self.plant.GetBodyByName("safety_region_b", self.s_B),
            SpatialVelocity(np.zeros(3),np.array([0.0,0.0,0.0])),
            self.plant.GetMyContextFromRoot(diagram_context))
                 
builder = DiagramBuilder()
# Setup visualization
meshcat = StartMeshcat()
scene_graph = builder.AddSystem(SceneGraph())
MeshcatVisualizer(meshcat).AddToBuilder(builder, scene_graph, meshcat)


# Add modules to the diagram

# plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
# region = builder.AddSystem(R(params,plant,scene_graph))
# region = R(params).AddToBuilder(builder, scene_graph)

# Build the diagram
diagram = builder.Build()

# Set up a simulator to run this diagram
simulator = Simulator(diagram)
context = simulator.get_mutable_context()

# Set the initial conditions 
# context.SetDiscreteState(0,init_ball)
# context.SetDiscreteState(1,[])
context.SetDiscreteState(init_ball)
context.SetContinuousState(init_paddle)

# Try to run simulation 4 times slower than real time
simulator.set_target_realtime_rate(0.25)
simulator.AdvanceTo(1.0)