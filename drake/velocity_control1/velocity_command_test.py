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
    RollPitchYaw, LogVectorOutput, plot_system_graphviz )
from pydrake.multibody.jupyter_widgets import MakeJointSlidersThatPublishOnCallback
  
# setting path
sys.path.append('/root/kinova_drake/')

from kinova_station import KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget, KinovaStation
from controllers.velocity import VelocityCommand, VelocityCommandSequence, VelocityCommandSequenceController
from observers.camera_viewer import CameraViewer

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

def create_pusher_slider_scenario():
    """
    Description:
        Creates the 6 degree of freedom Kinova system in simulation. Anchors it to a "ground plane" and gives it the
        RobotiQ 2f_85 gripper.
        This should also initialize the meshcat visualizer so that we can easily view the robot.
    Usage:
        builder, controller, station, diagram, diagram_context = create_pusher_slider_scenario()
    """

    builder = DiagramBuilder()

    # Constants
    gripper_type = '2f_85'
    dt = 0.001

    pusher_position = [0.8,0.5,0.25]
    # pusher_rotation=[0,np.pi/2,0]
    pusher_rotation=[0,0,0]

    # Start with the Kinova Station object
    station = KinovaStation(time_step=dt,n_dof=6)

    plant = station.plant
    scene_graph = station.scene_graph

    station.AddArmWith2f85Gripper() # Adds arm with the 2f85 gripper

    station.AddGround()
    station.AddCamera()

    X_pusher = RigidTransform()
    X_pusher.set_translation(pusher_position)
    X_pusher.set_rotation(RotationMatrix(RollPitchYaw(pusher_rotation)))
    station.AddManipulandFromFile("pusher/pusher1_urdf.urdf",X_pusher)


    # station.SetupSinglePegScenario(gripper_type=gripper_type, arm_damping=False)
    station.ConnectToMeshcatVisualizer(zmq_url="tcp://127.0.0.1:6000")

    station.Finalize() # finalize station (a diagram in and of itself)

    # Start assembling the overall system diagram
    builder = DiagramBuilder()
    station = builder.AddSystem(station)

    # Setup Gripper and End Effector Command Systems
    #create_end_effector_target(EndEffectorTarget.kPose,builder,station)
    #setup_gripper_command_systems(GripperTarget.kPosition,builder,station)

    # Setup loggers
    add_loggers_to_system(builder,station)

    # Setup Controller
    cs = setup_triangle_command_sequence()
    controller = setup_controller_and_connect_to_station(cs,builder,station)

    # Build the system diagram
    diagram = builder.Build()
    diagram.set_name("system_diagram")
    diagram_context = diagram.CreateDefaultContext()

    # context = diagram.CreateDefaultContext()
    station.meshcat.load()
    diagram.Publish(diagram_context)

    ## Set up initial positions ##

    # Set default arm positions
    station.go_home(diagram, diagram_context, name="Home")

    # Set starting position for any objects in the scene
    station.SetManipulandStartPositions(diagram, diagram_context)

    # Return station
    return builder, controller, station, diagram, diagram_context

def setup_triangle_command_sequence():
    """
    Description:
        Creates the command sequence that we need to achieve the infinity sequence.
    Notes:
        Each velocity is a six-dimensional vector where each dimension represents the following rates:
        - [roll, pitch, yaw, x, y, z]
    """
    # Constants
    triangle_side_duration = 4.0

    # Create the command sequence object
    vcs = VelocityCommandSequence([])

    # 1. Initial Command (Pause for 5s)
    init_velocity = np.zeros((6,))
    vcs.append(VelocityCommand(
        name="pause1",
        target_velocity=init_velocity,
        duration=0.5,
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
        duration=5,
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

    # Create the controller and connect inputs and outputs appropriately
    #Kp = 10*np.eye(6)
    Kp = np.diag([2,2,2,10,10,10])*10^4
    Kd = 0*np.sqrt(Kp)

    controller = builder.AddSystem(VelocityCommandSequenceController(
        cs,
        command_type=EndEffectorTarget.kTwist,  # Twist commands seem most reliable in simulation
        Kp=Kp,
        Kd=Kd))
    controller.set_name("controller")
    controller.ConnectToStation(builder, station)

    return controller

###############################################
# Important Flags

run = True
show_station_diagram = True

###############################################

# Building Diagram
builder, controller, station, diagram, diagram_context = create_pusher_slider_scenario()

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

#Wait at end
while True:
    1