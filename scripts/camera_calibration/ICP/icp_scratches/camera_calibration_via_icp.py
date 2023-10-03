import pyrealsense2 as rs
import numpy as np
import math
import itertools
import open3d as o3d
import cv2
from dt_apriltags import Detector
from pydrake.all import (
    DepthImageToPointCloud, PointCloud, Fields, BaseField,
    RollPitchYaw, RotationMatrix, RigidTransform, ConstantVectorSource,
    Meshcat, StartMeshcat, MeshcatVisualizer, MeshcatPointCloudVisualizer,
    DiagramBuilder, Parser, Simulator, AddMultibodyPlantSceneGraph,
    Rgba, CameraInfo, PixelType
)
from manipulation.icp import IterativeClosestPoint
from manipulation.scenarios import AddMultibodyTriad, AddRgbdSensors

" Pydrake Point Cloud "
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


" Apriltag Detector "
tag_size = 0.014
at_detector = Detector(families='tagStandard41h12', # Configure AprilTag detector
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)


" Drake Diagram "
builder = DiagramBuilder() # Create a Drake diagram

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.1) # Add MultibodyPlant and SceneGraph
parser = Parser(plant)
parser.AddModelFromFile("/home/krutledg/kinova/kinova_drake/models/gen3_6dof/urdf/GEN3-6DOF.urdf")
plant.Finalize()

meshcat = StartMeshcat() # Start the Meshcat visualizer
MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat) # Add MeshcatVisualizer
AddRgbdSensors(builder, plant, scene_graph) # Add RGB-D sensors to the robot

torques = builder.AddSystem(ConstantVectorSource(np.zeros(plant.num_actuators())))
builder.Connect(torques.get_output_port(), plant.get_actuation_input_port())

diagram = builder.Build() # Build the system diagram and create default context
diagram.set_name("system_diagram")
diagram_context = diagram.CreateDefaultContext()


" Drake Simulator "
simulator = Simulator(diagram, diagram_context)
context = simulator.get_mutable_context()
simulator.Initialize()


" RealSense Pipeline "
pc = rs.pointcloud() # Declare pointcloud object, for calculating pointclouds and texture mappings
points = rs.points() # We want the points object to be persistent so we can display the last cloud when a frame drops
pipe = rs.pipeline() # Declare RealSense pipeline, encapsulating the actual device and sensors
config = rs.config()
config.enable_stream(rs.stream.depth) # Enable depth stream

# Get device product line for setting a supporting resolution
pipeline_profile = config.resolve(rs.pipeline_wrapper(pipe))
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)
    
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


cfg = pipe.start(config)    # Start streaming with chosen configuration
colorizer = rs.colorizer()  # We'll use the colorizer to generate texture for our PLY

# Get camera parameters [fx, fy, cx, cy] from RealSense camera
profile = cfg.get_stream(rs.stream.depth)
intrinsics = profile.as_video_stream_profile().get_intrinsics()
cam_params = [intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy]

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipe.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Convert depth and color images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Perform AprilTag detection
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        tags = at_detector.detect(gray_image, estimate_tag_pose=True, camera_params=cam_params, tag_size=tag_size)

        # Convert depth image to point cloud
        depth_image = depth_frame.get_data()
        depth_scale = device.first_depth_sensor().get_depth_scale()
        camera_info = CameraInfo(
            width=640,  # Update with the actual width
            height=480,  # Update with the actual height
            focal_x=intrinsics.fx,
            focal_y=intrinsics.fy,
            center_x=intrinsics.ppx,
            center_y=intrinsics.ppy,
        )
        # Create an empty point cloud
        xyzs = []
        # Iterate through the depth image and add valid points to the point cloud
        for py in range(depth_frame.height):
            for px in range(depth_frame.width):
                depth_value = depth_frame.get_distance(px, py)
                if -10 < depth_value and depth_value < 10:
                    # Calculate 3D coordinates (x, y, z) using depth and camera parameters
                    z = depth_value
                    x = (px - intrinsics.ppx) * z / intrinsics.fx
                    y = (py - intrinsics.ppy) * z / intrinsics.fy
                    xyz = [x,y,z]
                    xyzs.append(xyz)

        scene_point_cloud = ToPointCloud(np.array(xyzs).T)
        meshcat.SetObject(
            "scene", scene_point_cloud, rgba=Rgba(0, 0, 1, 1)
        )      
        pixel_type = PixelType.kDepth32F
        point_cloud = PointCloud()
        pcl = DepthImageToPointCloud()
        pcdd = pcl.Convert(camera_info, depth_image, color_image, depth_scale, point_cloud)

        # Create a point cloud of the detected tags
        for tag in tags:
            for corner in tag.corners:
                depth_value = depth_frame.get_distance(int(corner[1]), int(corner[0]))
                if -10 < depth_value and depth_value < 10:
                    x = (corner[0] - intrinsics.ppx) * depth_value / intrinsics.fx
                    y = (corner[1] - intrinsics.ppy) * depth_value / intrinsics.fy
                    z = depth_value
                    xyz = [x,y,z]
                    xyzs.append(xyz)
                    
        tag_point_cloud = ToPointCloud(np.array(xyzs).T)

        # Perform ICP registration with the robot's point cloud and tag point cloud
        scene_point_cloud = scene_point_cloud.xyzs()
        tag_point_cloud = tag_point_cloud.xyzs()
        icp_result, _ = IterativeClosestPoint(tag_point_cloud, scene_point_cloud)

        # Update the robot's pose with the ICP result
        robot_pose = RigidTransform(
            RotationMatrix(icp_result.rotation()),
            icp_result.translation()
        )
        # Get the Context for plant from diagram's context.
        plant_context = plant.GetMyContextFromRoot(root_context=diagram_context)
        plant.SetFreeBodyPose(plant_context, plant.GetBodyByName("base_link"), robot_pose)
        

        # Advance the simulator
        simulator.AdvanceTo(simulator.get_context().get_time() + 0.01)

finally:
    # Stop streaming and close the Meshcat visualizer
    pipe.stop()
    meshcat.Delete()