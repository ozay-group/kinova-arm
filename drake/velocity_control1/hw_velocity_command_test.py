"""
move_arm_to_slider.py
Description:
    Trying to build a basic simulation where we move the gripper of the Kinova Gen3 6DoF
    to the target location and grip an object.
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
    ConstantValueSource, ConstantVectorSource, AbstractValue, 
    RollPitchYaw, LogVectorOutput, plot_system_graphviz,
    LeafSystem, BasicVector )
from pydrake.multibody.jupyter_widgets import MakeJointSlidersThatPublishOnCallback
  
# setting path
sys.path.append('/root/kinova_drake/')

from kinova_station import KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget, KinovaStation
from controllers.velocity import VelocityCommand, VelocityCommandSequence, VelocityCommandSequenceController
from observers.camera_viewer import CameraViewer

#######################
## Class Definitions ##
#######################

class VelocityCalculator(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        # Constants
        self.last_ee_pose = np.zeros(6)
        self.last_time = 0.0
        self.last_ee_velocity = np.zeros(6)
        
        # Create Input Port for The Current State
        self.pose_port = self.DeclareVectorInputPort("current_ee_pose",
                                                                BasicVector(6))

        # Create Output Port which should share the pose of the block
        self.DeclareVectorOutputPort(
                "estimated_ee_velocity",
                BasicVector(6),
                self.ComputeVelocity,
                {self.time_ticket()}   # indicate that this doesn't depend on any inputs,
                )                      # but should still be updated each timestep

        # Build the diagram

    def ComputeVelocity(self, context, output):
        """
        Description:
            This function computes the velocity in pose-space of the signal connected to the input port.
        """
        # Constants
        t = context.get_time()
        eps0 = 0.002

        if (t - self.last_time) < eps0:
            output.SetFromVector(self.last_ee_velocity)
        else:
            # Get Current Pose from Port
            current_ee_pose = self.pose_port.Eval(context)

            ee_estimated_velocity = (current_ee_pose - self.last_ee_pose)/(t-self.last_time)
            
            # Save last variables
            self.last_time = t
            self.last_ee_velocity = ee_estimated_velocity
            self.last_ee_pose = current_ee_pose
            
            print("ee_velocity")
            print(ee_estimated_velocity)

            # Create Output
            output.SetFromVector(ee_estimated_velocity)

###############
## Functions ##
###############

def add_loggers_to_system(builder,station):
    # Loggers force certain outputs to be computed
    wrench_logger = LogVectorOutput(station.GetOutputPort("measured_ee_wrench"),builder)
    wrench_logger.set_name("wrench_logger")

    pose_logger = LogVectorOutput(station.GetOutputPort("measured_ee_pose"), builder)
    pose_logger.set_name("pose_logger")

    twist_logger = LogVectorOutput(station.GetOutputPort("measured_ee_twist"), builder)
    twist_logger.set_name("twist_logger")

    gripper_logger = LogVectorOutput(station.GetOutputPort("measured_gripper_velocity"), builder)
    gripper_logger.set_name("gripper_logger")

    return pose_logger

def create_velocity_control_hw_scenario(station_in):
    """
    Description:
        Anchors it to a "ground plane" and gives it the
        RobotiQ 2f_85 gripper.
        This should also initialize the meshcat visualizer so that we can easily view the robot.
    Usage:
        builder, controller, diagram, diagram_context = create_velocity_control_hw_scenario(station)
    """

    builder = DiagramBuilder()

    # Constants
    gripper_type = '2f_85'
    dt = 0.001

    pusher_position = [0.8,0.5,0.25]
    # pusher_rotation=[0,np.pi/2,0]
    pusher_rotation=[0,0,0]

    # Start with the Kinova Station object
    # station = KinovaStationHardwareInterface(time_step=dt,n_dof=6)

    # Start assembling the overall system diagram
    builder = DiagramBuilder()
    builder.AddSystem(station)

    # Setup loggers
    pose_logger = add_loggers_to_system(builder,station)

    # Setup Controller
    cs = setup_triangle_command_sequence()
    controller, v_estimator = setup_controller_and_connect_to_station(cs,builder,station)

    # Log Velocity Estimator
    vel_estimate_logger = LogVectorOutput(v_estimator.GetOutputPort("estimated_ee_velocity"), builder)
    vel_estimate_logger.set_name("velocity_estimate_logger")

    # Build the system diagram
    diagram = builder.Build()
    diagram.set_name("system_diagram")
    diagram_context = diagram.CreateDefaultContext()

    # context = diagram.CreateDefaultContext()
    # station.meshcat.load()
    diagram.Publish(diagram_context)

    ## Set up initial positions ##

    # Set default arm positions
    # station.go_home(diagram, diagram_context)

    # Set starting position for any objects in the scene
    # station.SetManipulandStartPositions(diagram, diagram_context)

    # Return builder, controller, etc.
    return builder, controller, diagram, diagram_context, pose_logger, vel_estimate_logger

def setup_triangle_command_sequence():
    """
    Description:
        Creates the command sequence that we need to achieve the infinity sequence.
    Notes:
        Each velocity is a six-dimensional vector where each dimension represents the following rates:
        - [roll, pitch, yaw, x, y, z]
    """
    # Constants
    triangle_side_duration = 10.0

    # Create the command sequence object
    vcs = VelocityCommandSequence([])

    # 1. Initial Command (Pause for 5s)
    init_velocity = np.zeros((6,))
    vcs.append(VelocityCommand(
        name="pause1",
        target_velocity=init_velocity,
        duration=2,
        gripper_closed=False))

    # 2. Upper Right
    deltap1 = np.zeros((6,))
    deltap1[3:] = np.array([0.2,0.2,0])
    vcs.append(VelocityCommand(
        name="upper_right",
        target_velocity=deltap1/triangle_side_duration,
        duration=triangle_side_duration,
        gripper_closed=False))

    # 3. Lower Right
    deltap2 = np.zeros((6,))
    deltap2[3:] = np.array([0.2,-0.2,0])
    vcs.append(VelocityCommand(
        name="upper_right",
        target_velocity=deltap2/triangle_side_duration,
        duration=triangle_side_duration,
        gripper_closed=False))    

    # 4. Return to STart
    deltap3 = np.zeros((6,))
    deltap3[3:] = np.array([-0.4,0,0])
    vcs.append(VelocityCommand(
        name="return_home",
        target_velocity=deltap3/triangle_side_duration,
        duration=triangle_side_duration,
        gripper_closed=False))   

    # 5. Pause
    vcs.append(VelocityCommand(
        name="pause2",
        target_velocity=init_velocity,
        duration=2,
        gripper_closed=False))

    return vcs

def setup_controller_and_connect_to_station(cs,builder,station):
    """
    Description:
        Defines the controller (PID) which is a CommandSequenceController as defined in
        kinova_drake.
    Inputs:
        cs = A CommandSequence object which helps define the CommandSequenceController.
    """

    # Create the velocity estimator
    v_estimator = builder.AddSystem(VelocityCalculator())

    # Create the controller and connect inputs and outputs appropriately
    #Kp = 10*np.eye(6)
    # Kp = np.diag([0.2,0.2,0.2,2,2,2])
    # Kd = 2*np.sqrt(Kp)

    Kp = 10*np.eye(6)
    Kd = 2*np.sqrt(Kp)

    controller = builder.AddSystem(VelocityCommandSequenceController(
        cs,
        command_type=EndEffectorTarget.kWrench,  # wrench commands work best on hardware
        Kp=Kp,
        Kd=Kd))
    controller.set_name("controller")

    # Connect the Controller to the station
    builder.Connect(                                  # Send commands to the station
                controller.GetOutputPort("ee_command"),
                station.GetInputPort("ee_target"))
    builder.Connect(
            controller.GetOutputPort("ee_command_type"),
            station.GetInputPort("ee_target_type"))
    builder.Connect(
            controller.GetOutputPort("gripper_command"),
            station.GetInputPort("gripper_target"))
    builder.Connect(
            controller.GetOutputPort("gripper_command_type"),
            station.GetInputPort("gripper_target_type"))

    # Connect the Station to the Estimator
    builder.Connect(
        station.GetOutputPort("measured_ee_pose"),
        v_estimator.GetInputPort("current_ee_pose")
    )

    # Connect the Estimator to the Controller
    builder.Connect(                                        # Send estimated state information
            v_estimator.GetOutputPort("estimated_ee_velocity"), # to the controller
            controller.GetInputPort("ee_velocity"))
    builder.Connect(
            station.GetOutputPort("measured_ee_twist"),
            controller.GetInputPort("ee_twist"))
    #controller.ConnectToStation(builder, station)



    return controller, v_estimator

###############################################
# Important Flags

run = True
show_station_diagram = False
show_plots = True

n_dof = 6
###############################################

time_step = 0.025

# Building Diagram
with KinovaStationHardwareInterface(n_dof) as station:
    builder, controller, diagram, diagram_context, pose_logger, vel_estimate_logger = create_velocity_control_hw_scenario(station)

    if show_station_diagram:
        # Show the station's system diagram
        plt.figure()
        plot_system_graphviz(diagram,max_depth=1)
        plt.show()

    if run:
        # # First thing: send to home position
        # station.go_home(diagram,diagram_context)

        # We use a simulator instance to run the example, but no actual simulation 
        # is being done: it's all on the hardware. 
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)
        simulator.set_publish_every_time_step(False)  # Usually, this should be set to False. Otherwise, simulation will be very slow and won't look like real time.

        # We'll use a super simple integration scheme (since we only update a dummy state)
        # and set the maximum timestep to correspond to roughly 40Hz 
        integration_scheme = "explicit_euler"
        time_step = 0.025
        #ResetIntegratorFromFlags(simulator, integration_scheme, time_step)

        # Run simulation
        simulator.Initialize()
        simulator.AdvanceTo(controller.cs.total_duration())

        # Collect Data
        pose_log = pose_logger.FindLog(diagram_context)
        log_times  = pose_log.sample_times()
        pose_data = pose_log.data()
        print(pose_data.shape)

        vel_log = vel_estimate_logger.FindLog(diagram_context)
        vel_log_times = vel_log.sample_times()
        vel_data = vel_log.data()
        print(vel_data.shape)

        if show_plots:

            # Plot Data - First Half
            fig = plt.figure()
            ax_list1 = []

            for plt_index1 in range(6):
                ax_list1.append( fig.add_subplot(231+plt_index1) )
                plt.plot(log_times,pose_data[plt_index1,:])
                plt.title('Pose #' + str(plt_index1))

            fig2 = plt.figure()
            ax_list2 = []
            for plt_index2 in range(6):
                ax_list2.append( fig2.add_subplot(231+plt_index2) )
                plt.plot(vel_log_times[10:],vel_data[plt_index2,10:])
                plt.title('Vel #' + str(plt_index2))

            plt.show()

    #Wait at end
    input('Press ENTER to end python program.')