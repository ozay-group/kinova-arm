from pydrake.all import *
import numpy as np
import matplotlib.pyplot as plt


## import kinova drake which is not in the subdirectories here
import sys
sys.path.append('/root/kinova_drake/')
from kinova_station import KinovaStation, EndEffectorTarget, GripperTarget
from observers.camera_viewer import CameraViewer

from simulated_camera import addSimulatedCamera
from timed_camera_viewer import TimedCameraViewer
########################### Parameters #################################

# Run a quick simulation
simulate = True

# If we're running a simulation, choose which sort of commands are
# sent to the arm and the gripper
ee_command_type = EndEffectorTarget.kTwist      # kPose, kTwist, or kWrench
gripper_command_type = GripperTarget.kPosition  # kPosition or kVelocity

# If we're running a simulation, whether to include a simulated camera
# and show the associated image
include_camera = True
show_camera_window = True

# Which gripper to use (hande or 2f_85)
gripper_type = "hande"

########################################################################

# Set up the kinova station
station = KinovaStation(time_step=0.002, n_dof=7)
station.SetupSinglePegScenario(gripper_type=gripper_type, arm_damping=False)
if include_camera:
    station.AddCamera(show_window=show_camera_window)
    # Manual position of camera to give a nice view of the scene
    X_camera = RigidTransform(RollPitchYaw(3.14/2, 3.14, 0), [0.5, 1.5, 0.45])
    # Create simulated camera with fixed orientation and add to the KinovaStation
    my_camera = addSimulatedCamera("sa_camera", X_camera, station.scene_graph, station.builder, show_window=show_camera_window)


station.ConnectToMeshcatVisualizer()
station.Finalize()


# Connect input ports to the kinova station
builder = DiagramBuilder()
builder.AddSystem(station)

# Set (constant) command to send to the system
if ee_command_type == EndEffectorTarget.kPose:
    pose_des = np.array([np.pi,0.0,0.0,
                            0.6,0.0,0.2])
    target_source = builder.AddSystem(ConstantVectorSource(pose_des))

elif ee_command_type == EndEffectorTarget.kTwist:
    twist_des = np.array([0.0,0.1,0.0,
                          0.0,0.0,0.0])
    target_source = builder.AddSystem(ConstantVectorSource(twist_des))

elif ee_command_type == EndEffectorTarget.kWrench:
    wrench_des = np.array([0.0,0.0,0.0,
                            0.0,0.0,0.0])
    target_source = builder.AddSystem(ConstantVectorSource(wrench_des))

else:
    raise RuntimeError("invalid end-effector target type")

# Send end-effector command and type
target_type_source = builder.AddSystem(ConstantValueSource(AbstractValue.Make(ee_command_type)))
builder.Connect(
        target_type_source.get_output_port(),
        station.GetInputPort("ee_target_type"))

builder.Connect(
        target_source.get_output_port(),
        station.GetInputPort("ee_target"))

target_source.set_name("ee_command_source")
target_type_source.set_name("ee_type_source")

# Set gripper command
if gripper_command_type == GripperTarget.kPosition:
    q_grip_des = np.array([0])   # open at 0, closed at 1
    gripper_target_source = builder.AddSystem(ConstantVectorSource(q_grip_des))

elif gripper_command_type == GripperTarget.kVelocity:
    v_grip_des = np.array([1.0])
    gripper_target_source = builder.AddSystem(ConstantVectorSource(v_grip_des))

# Send gripper command and type
gripper_target_type_source = builder.AddSystem(ConstantValueSource(
                                         AbstractValue.Make(gripper_command_type)))
builder.Connect(
        gripper_target_type_source.get_output_port(),
        station.GetInputPort("gripper_target_type"))

builder.Connect(
        gripper_target_source.get_output_port(),
        station.GetInputPort("gripper_target"))

gripper_target_source.set_name("gripper_command_source")
gripper_target_type_source.set_name("gripper_type_source")

# Loggers force certain outputs to be computed
wrench_logger = LogVectorOutput(station.GetOutputPort("measured_ee_wrench"),builder)
wrench_logger.set_name("wrench_logger")

pose_logger = LogVectorOutput(station.GetOutputPort("measured_ee_pose"), builder)
pose_logger.set_name("pose_logger")

twist_logger = LogVectorOutput(station.GetOutputPort("measured_ee_twist"), builder)
twist_logger.set_name("twist_logger")

gripper_logger = LogVectorOutput(station.GetOutputPort("measured_gripper_velocity"), builder)
gripper_logger.set_name("gripper_logger")

if include_camera:
    # Camera observer allows us to access camera data, and must be connected
    # to view the camera stream. Images are produced from the camera at a rate of 30 frames per second.
    camera_viewer = builder.AddSystem(TimedCameraViewer(fps=30))
    camera_viewer.set_name("camera_viewer")
    builder.Connect(
            station.GetOutputPort("camera_rgb_image"),
            camera_viewer.GetInputPort("color_image"))
    builder.Connect(
            station.GetOutputPort("camera_depth_image"),
            camera_viewer.GetInputPort("depth_image"))
            
    # Connect a second camera viewer to the standalone simulated camera, 
    sa_camera_viewer = builder.AddSystem(TimedCameraViewer(fps=30))
    sa_camera_viewer.set_name("sa_camera_viewer")
    builder.Connect(
            station.GetOutputPort("sa_camera_rgb_image"),
            sa_camera_viewer.GetInputPort("color_image"))
    builder.Connect(
            station.GetOutputPort("sa_camera_depth_image"),
            sa_camera_viewer.GetInputPort("depth_image"))
    

    # Convert the depth image to a point cloud
    point_cloud_generator = builder.AddSystem(DepthImageToPointCloud(
                                        CameraInfo(width=480, height=270, fov_y=np.radians(40)),
                                        fields=BaseField.kXYZs | BaseField.kRGBs))
    point_cloud_generator.set_name("point_cloud_generator")
    builder.Connect(
            station.GetOutputPort("camera_depth_image"),
            point_cloud_generator.depth_image_input_port())

    # Connect camera pose to point cloud generator
    builder.Connect(
            station.GetOutputPort("camera_transform"),
            point_cloud_generator.GetInputPort("camera_pose"))

    # Visualize the point cloud with meshcat
    # Rendered onto scene (looks like projection)
    meshcat_point_cloud = builder.AddSystem(
            MeshcatPointCloudVisualizer(station.meshcat, "point_cloud", 0.2))
    meshcat_point_cloud.set_name("point_cloud_viz")
    builder.Connect(
            point_cloud_generator.point_cloud_output_port(),
            meshcat_point_cloud.cloud_input_port())

# Build the system diagram
diagram = builder.Build()
diagram.set_name("toplevel_system_diagram")
diagram_context = diagram.CreateDefaultContext()

if simulate:
    # Set default arm positions
    station.go_home(diagram, diagram_context, name="Home")

    # Set starting position for any objects in the scene
    station.SetManipulandStartPositions(diagram, diagram_context)

    # Set up simulation
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    # Run simulation
    #simulator.Initialize()
    simulator.AdvanceTo(5.0)