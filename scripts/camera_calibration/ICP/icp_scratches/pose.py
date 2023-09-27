import numpy as np
from pydrake.all import (
    AbstractValue,
    AngleAxis,
    Concatenate,
    DiagramBuilder,
    LeafSystem,
    MeshcatVisualizer,
    PiecewisePolynomial,
    PiecewisePose,
    PointCloud,
    RigidTransform,
    RollPitchYaw,
    Simulator,
    StartMeshcat,
)

# Start the visualizer.
meshcat = StartMeshcat()

model_directives = """
directives:
- add_directives:
    file: package://manipulation/clutter_w_cameras.dmd.yaml
- add_model:
    name: mustard
    file: package://drake/manipulation/models/ycb/sdf/006_mustard_bottle.sdf
    default_free_body_pose:
        base_link_mustard:
            translation: [.55, 0.1, 0.09515]
            rotation: !Rpy { deg: [-90, 0, 45] }
"""


# Takes 3 point clouds (in world coordinates) as input, and outputs and estimated pose for the mustard bottle.
class MustardIterativeClosestPoint(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        model_point_cloud = AbstractValue.Make(PointCloud(0))
        self.DeclareAbstractInputPort("cloud0", model_point_cloud)
        self.DeclareAbstractInputPort("cloud1", model_point_cloud)
        self.DeclareAbstractInputPort("cloud2", model_point_cloud)

        self.DeclareAbstractOutputPort(
            "X_WO",
            lambda: AbstractValue.Make(RigidTransform()),
            self.EstimatePose,
        )

        self.mustard = MustardPointCloud()
        meshcat.SetObject("icp_scene", self.mustard)

    def EstimatePose(self, context, output):
        pcd = []
        for i in range(3):
            cloud = self.get_input_port(i).Eval(context)
            pcd.append(
                cloud.Crop(
                    lower_xyz=[0.4, -0.2, 0.001], upper_xyz=[0.6, 0.3, 0.3]
                )
            )
        merged_pcd = Concatenate(pcd)
        down_sampled_pcd = merged_pcd.VoxelizedDownSample(voxel_size=0.005)
        meshcat.SetObject(
            "icp_observations", down_sampled_pcd, point_size=0.001
        )

        X_WOhat, chat = IterativeClosestPoint(
            self.mustard.xyzs(),
            down_sampled_pcd.xyzs(),
            meshcat=meshcat,
            meshcat_scene_path="icp_scene",
        )

        output.set_value(X_WOhat)


class PickAndPlaceTrajectory(LeafSystem):
    def __init__(self, plant):
        LeafSystem.__init__(self)
        self._gripper_body_index = plant.GetBodyByName("body").index()
        self.DeclareAbstractInputPort(
            "body_poses", AbstractValue.Make([RigidTransform()])
        )
        self.DeclareAbstractInputPort(
            "X_WO", AbstractValue.Make(RigidTransform())
        )

        self.DeclareInitializationUnrestrictedUpdateEvent(self.Plan)
        self._traj_X_G_index = self.DeclareAbstractState(
            AbstractValue.Make(PiecewisePose())
        )
        self._traj_wsg_index = self.DeclareAbstractState(
            AbstractValue.Make(PiecewisePolynomial())
        )

        self.DeclareAbstractOutputPort(
            "X_WG",
            lambda: AbstractValue.Make(RigidTransform()),
            self.CalcGripperPose,
        )
        self.DeclareVectorOutputPort("wsg_position", 1, self.CalcWsgPosition)

    def Plan(self, context, state):
        X_G = {
            "initial": self.get_input_port(0).Eval(context)[
                int(self._gripper_body_index)
            ]
        }
        X_O = {
            "initial": self.get_input_port(1).Eval(context),
            "goal": RigidTransform([0, -0.6, 0]),
        }
        X_GgraspO = RigidTransform(RollPitchYaw(np.pi / 2, 0, 0), [0, 0.22, 0])
        X_OGgrasp = X_GgraspO.inverse()
        X_G["pick"] = X_O["initial"] @ X_OGgrasp
        X_G["place"] = X_O["goal"] @ X_OGgrasp
        X_G, times = MakeGripperFrames(X_G)
        print(f"Planned {times['postplace']} second trajectory.")

        if False:  # Useful for debugging
            AddMeshcatTriad(meshcat, "X_Oinitial", X_PT=X_O["initial"])
            AddMeshcatTriad(meshcat, "X_Gprepick", X_PT=X_G["prepick"])
            AddMeshcatTriad(meshcat, "X_Gpick", X_PT=X_G["pick"])
            AddMeshcatTriad(meshcat, "X_Gplace", X_PT=X_G["place"])

        traj_X_G = MakeGripperPoseTrajectory(X_G, times)
        traj_wsg_command = MakeGripperCommandTrajectory(times)

        state.get_mutable_abstract_state(int(self._traj_X_G_index)).set_value(
            traj_X_G
        )
        state.get_mutable_abstract_state(int(self._traj_wsg_index)).set_value(
            traj_wsg_command
        )

    def start_time(self, context):
        return (
            context.get_abstract_state(int(self._traj_X_G_index))
            .get_value()
            .start_time()
        )

    def end_time(self, context):
        return (
            context.get_abstract_state(int(self._traj_X_G_index))
            .get_value()
            .end_time()
        )

    def CalcGripperPose(self, context, output):
        # Evaluate the trajectory at the current time, and write it to the
        # output port.
        output.set_value(
            context.get_abstract_state(int(self._traj_X_G_index))
            .get_value()
            .GetPose(context.get_time())
        )

    def CalcWsgPosition(self, context, output):
        # Evaluate the trajectory at the current time, and write it to the
        # output port.
        output.SetFromVector(
            context.get_abstract_state(int(self._traj_wsg_index))
            .get_value()
            .value(context.get_time())
        )


def icp_pick_and_place_demo():
    builder = DiagramBuilder()

    station = builder.AddSystem(
        MakeManipulationStation(model_directives, time_step=0.002)
    )
    plant = station.GetSubsystemByName("plant")

    icp = builder.AddSystem(MustardIterativeClosestPoint())
    builder.Connect(
        station.GetOutputPort("camera3_point_cloud"), icp.get_input_port(0)
    )
    builder.Connect(
        station.GetOutputPort("camera4_point_cloud"), icp.get_input_port(1)
    )
    builder.Connect(
        station.GetOutputPort("camera5_point_cloud"), icp.get_input_port(2)
    )

    plan = builder.AddSystem(PickAndPlaceTrajectory(plant))
    builder.Connect(
        station.GetOutputPort("body_poses"), plan.GetInputPort("body_poses")
    )
    builder.Connect(icp.GetOutputPort("X_WO"), plan.GetInputPort("X_WO"))

    robot = station.GetSubsystemByName(
        "iiwa_controller"
    ).get_multibody_plant_for_control()

    # Set up differential inverse kinematics.
    diff_ik = AddIiwaDifferentialIK(builder, robot)
    builder.Connect(
        diff_ik.get_output_port(), station.GetInputPort("iiwa_position")
    )
    builder.Connect(plan.GetOutputPort("X_WG"), diff_ik.get_input_port(0))
    builder.Connect(
        station.GetOutputPort("iiwa_state_estimated"),
        diff_ik.GetInputPort("robot_state"),
    )

    builder.Connect(
        plan.GetOutputPort("wsg_position"),
        station.GetInputPort("wsg_position"),
    )

    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, station.GetOutputPort("query_object"), meshcat
    )
    diagram = builder.Build()

    simulator = Simulator(diagram)
    context = simulator.get_context()

    simulator.Initialize()
    if False:  # draw the trajectory triads
        X_G_traj = (
            plan.GetMyContextFromRoot(context)
            .get_abstract_state(0)
            .get_value()
        )
        for t in np.linspace(X_G_traj.start_time(), X_G_traj.end_time(), 40):
            AddMeshcatTriad(
                meshcat,
                f"X_G/({t})",
                X_PT=X_G_traj.GetPose(t),
                length=0.1,
                radius=0.004,
            )

    if running_as_notebook:
        visualizer.StartRecording(False)
        simulator.AdvanceTo(plan.end_time(plan.GetMyContextFromRoot(context)))
        visualizer.PublishRecording()
    else:
        simulator.AdvanceTo(0.1)


icp_pick_and_place_demo()