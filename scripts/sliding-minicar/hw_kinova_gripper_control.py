##
#
# Simple example of using our kinova manipulation station to pick up a peg
# at an a-priori known location. Runs on the real hardware.
#
##

""" Imports """

import sys
sys.path.append('../') # setting path for imports

import numpy as np
import matplotlib.pyplot as plt
import cv2

from pydrake.all import *
from pydrake.all import (AddMultibodyPlantSceneGraph, Meshcat, MeshcatVisualizer,
                            DiagramBuilder, Simulator, MeshcatPointCloudVisualizer, LogVectorOutput)

from kinova_drake.kinova_station import (KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget)
from kinova_drake.controllers import (PSCommandSequenceController, PSCommandSequence, PartialStateCommand)
from kinova_drake.observers import CameraViewer


""" Parameters """

show_toplevel_system_diagram = False    # Make a plot of the diagram for inner workings of the stationn
show_state_plots = True

n_dof = 6                               # number of degrees of freedom of the arm
gripper_type = "2f_85"                  # which gripper to use (hande or 2f_85)
time_step = 0.1                         # time step size (seconds)


""" Main Script """

with KinovaStationHardwareInterface(n_dof) as station:
# Note that unlike the simulation station, the hardware station needs to be used within a 'with' block.
# This is to allow for cleaner error handling, since the connection with the hardware needs to be
# closed properly even if there is an error (e.g. KeyboardInterrupt) during execution.

    ''' Connect Station '''
    builder = DiagramBuilder()
    station = builder.AddSystem(station)
    # plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step) # Add new Multibody plant and a new SceneGraph (for Visualization)
    
    
    ''' Connect Camera '''
    # camera_viewer = builder.AddSystem(CameraViewer()) # connect camera image viewer
    # camera_viewer.set_name("camera_viewer")
    # 
    # builder.Connect(
    #         station.GetOutputPort("camera_rgb_image"),
    #         camera_viewer.GetInputPort("color_image"))
    # builder.Connect(
    #         station.GetOutputPort("camera_depth_image"),
    #         camera_viewer.GetInputPort("depth_image"))
    
    
    ''' Connect Loggers '''
    # Connect the state of block to a Logger
    # state_logger = LogVectorOutput(block_system.GetOutputPort("measured_block_pose"), builder)
    # state_logger.set_name("state_logger")
    
    q_logger = LogVectorOutput(station.GetOutputPort("measured_arm_position"), builder)
    q_logger.set_name("arm_position_logger")
    qd_logger = LogVectorOutput(station.GetOutputPort("measured_arm_velocity"), builder)
    qd_logger.set_name("arm_velocity_logger")
    tau_logger = LogVectorOutput(station.GetOutputPort("measured_arm_torque"), builder)
    tau_logger.set_name("arm_torque_logger")

    pose_logger = LogVectorOutput(station.GetOutputPort("measured_ee_pose"), builder)
    pose_logger.set_name("pose_logger")
    twist_logger = LogVectorOutput(station.GetOutputPort("measured_ee_twist"), builder)
    twist_logger.set_name("twist_logger")
    wrench_logger = LogVectorOutput(station.GetOutputPort("measured_ee_wrench"), builder)
    wrench_logger.set_name("wrench_logger")

    #gp_logger = LogVectorOutput(station.GetOutputPort("measured_gripper_position"), builder)
    #gp_logger.set_name("gripper_position_logger")
    #gv_logger = LogVectorOutput(station.GetOutputPort("measured_gripper_velocity"), builder)
    #gv_logger.set_name("gripper_velocity_logger")


    ''' Connect Meshcat '''
    meshcat = Meshcat(port=7001) # this object provides an interface to Meshcat
    # mesh_visual = MeshcatVisualizer(meshcat)
    # mesh_visual.AddToBuilder(builder,scene_graph,meshcat)
    
    
    ''' Command Sequence '''
    pscs = PSCommandSequence([]) # create the command sequence
    pscs.append(PartialStateCommand(
        name="initial move",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.5, 0.25]),
        gripper_value=0.0,
        duration=10))
    pscs.append(PartialStateCommand(
        name="move down",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.5, 0.025]),
        gripper_value=0.0,
        duration=10))
    pscs.append(PartialStateCommand(
        name="pregrasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.5, 0.025]),
        gripper_value=0.0,
        duration=7))
    pscs.append(PartialStateCommand(
        name="grasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.5, 0.025]),
        gripper_value=0.45,
        duration=3))
    pscs.append(PartialStateCommand(
        name="accelerate",
        target_type=EndEffectorTarget.kTwist,
        target_value=np.array([0.0*np.pi, 0.0*np.pi, 0.0*np.pi, 0.0, 25.0, 0.001]),
        gripper_value=0.45,
        duration=0.15))
    pscs.append(PartialStateCommand(
        name="release",
        target_type=EndEffectorTarget.kTwist,
        target_value=np.array([0.0*np.pi, 0.0*np.pi, 0.0*np.pi, 0.0, 25.0, 0.1]),
        gripper_value=0.0,
        duration=0.5))
    pscs.append(PartialStateCommand(
        name="end move",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([0.5*np.pi, 0.0*np.pi, 0.5*np.pi, 0.15, 0.0, 0.5]),
        gripper_value=0.0,
        duration=5))

    ''' Controller '''
    twist_Kp = np.diag([3.5, 3.5, 3.5, 3.0, 4.0, 6.5])*0.075
    twist_Kd = np.sqrt(twist_Kp)*0.35 + np.diag([0, 0, 0, 0, 0, 0.01])
    wrench_Kp = np.diag([75.0, 75, 75, 1500, 1500, 1500])
    wrench_Kd = np.sqrt(wrench_Kp)*0.125 + np.diag([0, 0, 0, 0, 0, 0])

    controller = builder.AddSystem(PSCommandSequenceController(
        pscs,
        twist_Kp = twist_Kp,
        twist_Kd = twist_Kp,
        wrench_Kp = wrench_Kp,
        wrench_Kd = wrench_Kd ))
    controller.set_name("controller")
    controller.ConnectToStation(builder, station, time_step=time_step)
    

    # # Convert the depth image to a point cloud
    # point_cloud_generator = builder.AddSystem(DepthImageToPointCloud(
    #                                     CameraInfo(width=480, height=270, fov_y=np.radians(40)),
    #                                     pixel_type=PixelType.kDepth16U))
    # point_cloud_generator.set_name("point_cloud_generator")
    # builder.Connect(
    #         station.GetOutputPort("camera_depth_image"),
    #         point_cloud_generator.depth_image_input_port())

    # # Connect camera pose to point cloud generator
    # builder.Connect(
    #         station.GetOutputPort("camera_transform"),
    #         point_cloud_generator.GetInputPort("camera_pose"))

    # # Visualize the point cloud with meshcat
    # meshcat_point_cloud = builder.AddSystem(MeshcatPointCloudVisualizer(meshcat=meshcat, path="../models/slider/slider-block.urdf"))
    # meshcat_point_cloud.set_name("point_cloud_viz")
    # builder.Connect(
    #         point_cloud_generator.point_cloud_output_port(),
    #         meshcat_point_cloud.GetInputPort("cloud"))
        
        
    ''' Build Diagram '''
    diagram = builder.Build() # build system diagram
    diagram.set_name("toplevel_system_diagram")
    diagram_context = diagram.CreateDefaultContext()

    if show_toplevel_system_diagram: # Show the overall system diagram
        plt.figure()
        plot_system_graphviz(diagram,max_depth=1)
        plt.show()


    ''' Simulation Sequence '''
    station.go_home(name="Home") # Set default arm positions
    
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    integration_scheme = "explicit_euler"
    ResetIntegratorFromFlags(simulator, integration_scheme, time_step)
    
    simulator.Initialize()
    simulator.AdvanceTo(controller.cs.total_duration())

    ''' Camera Image '''
    # color_image = camera_viewer.color_image_port.Eval(diagram_context)
    # cv2.imshow("rgb_image", color_image.data)
    # cv2.waitKey(0)
    # cv2.destroyWindow
    
    # depth_image = camera_viewer.depth_image_port.Eval(diagram_context)
    # cv2.imshow("depth_image", depth_image.data)
    # cv2.waitKey(0)
    # cv2.destroyWindow

    ''' Collect Data '''
    q_log = q_logger.FindLog(diagram_context)
    q_log_times = q_log.sample_times()
    q_log_data = q_log.data()
    print(q_log_data.shape)
    
    qd_log = qd_logger.FindLog(diagram_context)
    qd_log_times = qd_log.sample_times()
    qd_log_data = qd_log.data()
    print(qd_log_data.shape)
    
    tau_log = tau_logger.FindLog(diagram_context)
    tau_log_times = tau_log.sample_times()
    tau_log_data = tau_log.data()
    print(tau_log_data.shape)
    
    pose_log = pose_logger.FindLog(diagram_context)
    pose_log_times = pose_log.sample_times()
    pose_log_data = pose_log.data()
    print(pose_log_data.shape)
    
    twist_log = twist_logger.FindLog(diagram_context)
    twist_log_times = twist_log.sample_times()
    twist_log_data = twist_log.data()
    print(twist_log_data.shape)
    
    wrench_log = wrench_logger.FindLog(diagram_context)
    wrench_log_times = wrench_log.sample_times()
    wrench_log_data = wrench_log.data()
    print(wrench_log_data.shape)
    
    if show_state_plots:
        q_fig = plt.figure(figsize=(14,8))
        q_ax_list = []
        for i in range(6):
            q_ax_list.append(q_fig.add_subplot(231+i) )
            plt.plot(q_log_times,q_log_data[i,:])
            plt.title('q #' + str(i))
            
        qd_fig = plt.figure(figsize=(14,8))
        qd_ax_list = []
        for i in range(6):
            qd_ax_list.append(qd_fig.add_subplot(231+i) )
            plt.plot(qd_log_times,qd_log_data[i,:])
            plt.title('qd #' + str(i))
            
        tau_fig = plt.figure(figsize=(14,8))
        tau_ax_list = []
        for i in range(6):
            tau_ax_list.append(tau_fig.add_subplot(231+i) )
            plt.plot(tau_log_times,tau_log_data[i,:])
            plt.title('tau #' + str(i))
        
        pose_fig = plt.figure(figsize=(14,8))
        pose_ax_list = []
        for i in range(6):
            pose_ax_list.append(pose_fig.add_subplot(231+i) )
            plt.plot(pose_log_times,pose_log_data[i,:])
            plt.title('Pose #' + str(i))

        twist_fig = plt.figure(figsize=(14,8))
        twist_ax_list = []
        for i in range(6):
            twist_ax_list.append(twist_fig.add_subplot(231+i) )
            plt.plot(twist_log_times,twist_log_data[i,:])
            plt.title('Twist #' + str(i))
            
        wrench_fig = plt.figure(figsize=(14,8))
        wrench_ax_list = []
        for i in range(6):
            wrench_ax_list.append(wrench_fig.add_subplot(231+i) )
            plt.plot(wrench_log_times,wrench_log_data[i,:])
            plt.title('Wrench #' + str(i))
            
        plt.show()
    
    print("")
    print("Target control frequency: %s Hz" % (1/time_step))
    print("Actual control frequency: %s Hz" % (1/time_step * simulator.get_actual_realtime_rate()))

