import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    BaseField,
    Box,
    DepthImageToPointCloud,
    DiagramBuilder,
    Fields,
    MeshcatPointCloudVisualizer,
    MeshcatVisualizer,
    Parser,
    PixelType,
    PointCloud,
    Rgba,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    Simulator,
    SpatialInertia,
    StartMeshcat,
)
from manipulation.icp import IterativeClosestPoint
from manipulation.scenarios import AddMultibodyTriad, AddRgbdSensors, AddShape

# Start the visualizer.
meshcat = StartMeshcat()

def ToPointCloud(xyzs, rgbs=None):
    if rgbs:
        cloud = PointCloud(
            xyzs.shape[1], Fields(BaseField.kXYZs | BaseField.kRGBs)
        )
        cloud.mutable_rgbs()[:] = rgbs
    else:
        cloud = PointCloud(xyzs.shape[1])
    cloud.mutable_xyzs()[:] = xyzs
    return cloud

meshcat.Delete()


def visualize_red_foam_brick():
    """
    Visualize red foam brick in Meshcat.
    """
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.AddModelsFromUrl(
        "package://drake/examples/manipulation_station/models/061_foam_brick.sdf"
    )
    AddMultibodyTriad(plant.GetFrameByName("base_link"), scene_graph)
    plant.Finalize()

    # Setup Meshcat
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    diagram.ForcedPublish(context)


def generate_model_pointcloud(xrange, yrange, zrange, res):
    """
    Procedurally generate pointcloud of a rectangle for each side.
    """
    # Decide on how many samples
    x_lst = np.linspace(
        xrange[0], xrange[1], int((xrange[1] - xrange[0]) / res)
    )
    y_lst = np.linspace(
        yrange[0], yrange[1], int((yrange[1] - yrange[0]) / res)
    )
    z_lst = np.linspace(
        zrange[0], zrange[1], int((zrange[1] - zrange[0]) / res)
    )

    pcl_lst = []
    # Do XY Plane
    for x in x_lst:
        for y in y_lst:
            pcl_lst.append([x, y, zrange[0]])
            pcl_lst.append([x, y, zrange[1]])

    # Do YZ Plane
    for y in y_lst:
        for z in z_lst:
            pcl_lst.append([xrange[0], y, z])
            pcl_lst.append([xrange[1], y, z])

    # Do XZ Plane
    for x in x_lst:
        for z in z_lst:
            pcl_lst.append([x, yrange[0], z])
            pcl_lst.append([x, yrange[1], z])

    return np.array(pcl_lst).T


visualize_red_foam_brick()
model_pcl_np = generate_model_pointcloud(
    [-0.0375, 0.0375], [-0.025, 0.025], [0.0, 0.05], 0.002
)
meshcat.SetObject(
    "pcl_model", ToPointCloud(model_pcl_np), rgba=Rgba(0, 0, 1, 1)
)

def setup_clutter_station(X_WO, X_WC):
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.SetAutoRenaming(True)

    # Add the foam brick.
    if X_WO is not None:
        brick = parser.AddModelsFromUrl(
            "package://drake/examples/manipulation_station/models/061_foam_brick.sdf"
        )[0]
        plant.WeldFrames(
            plant.world_frame(), plant.GetFrameByName("base_link", brick), X_WO
        )

    bin1 = parser.AddModelsFromUrl(
        "package://drake/examples/manipulation_station/models/bin.sdf"
    )[0]
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("bin_base", bin1),
        RigidTransform(RollPitchYaw(0, 0, np.pi / 2), [-0.145, -0.63, 0.075]),
    )

    bin2 = parser.AddModelsFromUrl(
        "package://drake/examples/manipulation_station/models/bin.sdf"
    )[0]
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("bin_base", bin2),
        RigidTransform(RollPitchYaw(0, 0, np.pi), [0.5, -0.1, 0.075]),
    )

    # Add a mock camera model
    camera_instance = AddShape(
        plant,
        Box(width=0.1, depth=0.02, height=0.02),
        "camera",
        color=[0.4, 0.4, 0.4, 1.0],
    )
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("camera"), X_WC)
    AddMultibodyTriad(plant.GetFrameByName("camera"), scene_graph)
    plant.Finalize()

    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    AddRgbdSensors(builder, plant, scene_graph)

    # Send the point cloud to meshcat for visualization, too.
    #  meshcat_pointcloud = builder.AddSystem(MeshcatPointCloudVisualizer(meshcat, X_WP=X_WC, draw_period=1./5.))
    #  builder.Connect(to_point_cloud.point_cloud_output_port(), meshcat_pointcloud.get_input_port())

    diagram = builder.Build()
    diagram.set_name("depth_camera_demo_system")
    return diagram


# Set pose of the brick
X_WO = RigidTransform(
    RollPitchYaw(0, 0, np.pi / 5).ToRotationMatrix(),
    np.array([-0.1, -0.6, 0.09]),
)

# Setup CameraPose
X_WC = RigidTransform(
    RollPitchYaw(0, 0, 0)
    .ToRotationMatrix()
    .multiply(RollPitchYaw(-np.pi / 2.0 - np.pi / 3, 0, 0).ToRotationMatrix()),
    [-0.1, -0.8, 0.5],
)

# Take a pointcloud snapshot of the background to use for subtraction
diagram = setup_clutter_station(None, X_WC)
simulator = Simulator(diagram)
simulator.AdvanceTo(0.01)
context = simulator.get_context()
# Note: Use PointCloud here to make a copy of the data, since the diagram that
# owns it will be garbage collected.
scene_pcl_drake_background = PointCloud(
    diagram.GetOutputPort("camera_point_cloud").Eval(context)
)

# Take a pointcloud snapshot of the scene with the brick.
diagram = setup_clutter_station(X_WO, X_WC)
simulator = Simulator(diagram)
simulator.AdvanceTo(0.01)
context = simulator.get_context()
scene_pcl_drake = diagram.GetOutputPort("camera_point_cloud").Eval(context)

meshcat.Delete()

meshcat.SetObject(
    "pcl_model", ToPointCloud(model_pcl_np), rgba=Rgba(0, 0, 1, 1)
)
meshcat.SetObject("pcl_scene", scene_pcl_drake)
meshcat.SetObject("pcl_scene_background", scene_pcl_drake_background)

def segment_scene_pcl(
    scene_pcl_np,
    scene_rgb_np,
    scene_pcl_np_background,
    scene_rgb_np_background,
):
    """
    Inputs:
    scene_pcl_np: 3xN np.float32 array of pointclouds, each row containing xyz
                    position of each point in meters.
    scene_rgb_np: 3xN np.uint8   array of pointclouds, each row containing rgb
                    color data of each point.
    scene_pcl_np_background: 3xN np.float32 array of pointclouds, each row
                    containing xyz position of each point in meters.
    scene_rgb_np_background: 3xN np.uint8   array of pointclouds, each row
                    containing rgb color data of each point.

    Outputs:
    scene_pcl_np_filtered: 3xM np.float32 array of pointclouds that are on the
                    foam brick.
    """
    ####################
    # Fill your code here.

    scene_pcl_np_filtered = scene_pcl_np
    ####################

    return scene_pcl_np_filtered


scene_pcl_np_filtered = segment_scene_pcl(
    scene_pcl_drake.xyzs(),
    scene_pcl_drake.rgbs(),
    scene_pcl_drake_background.xyzs(),
    scene_pcl_drake_background.rgbs(),
)
meshcat.SetObject(
    "pcl_scene_filtered",
    ToPointCloud(scene_pcl_np_filtered),
    rgba=Rgba(0, 1, 0, 1),
)

initial_guess = RigidTransform()
initial_guess.set_translation([-0.145, -0.63, 0.09])
initial_guess.set_rotation(RotationMatrix.MakeZRotation(np.pi / 2))

X_MS_hat, chat = IterativeClosestPoint(
    p_Om=model_pcl_np,
    p_Ws=scene_pcl_np_filtered,
    X_Ohat=initial_guess,
    meshcat=meshcat,
    meshcat_scene_path="icp",
    max_iterations=25,
)
meshcat.SetObject(
    "pcl_estimated", ToPointCloud(model_pcl_np), rgba=Rgba(1, 0, 1, 1)
)
meshcat.SetTransform("pcl_estimated", X_MS_hat)

np.set_printoptions(precision=3, suppress=True)
X_OOhat = X_MS_hat.inverse().multiply(X_WO)

rpy = RollPitchYaw(X_OOhat.rotation()).vector()
xyz = X_OOhat.translation()

print("RPY Error: " + str(rpy))
print("XYZ Error: " + str(xyz))