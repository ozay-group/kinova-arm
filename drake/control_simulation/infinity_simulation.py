## infinity_simulation.py
#   Description:
#       This function is meant to produce the infinity motion in the meshcat simulator for our Kinova arm.
#       We will use the comand sequence controller demonstrated in the peg_pickup_demo.py from kinova_drake
##

###########
# Imports #
###########

from pydrake.all import *
import numpy as np
import matplotlib.pyplot as plt

# Start a single meshcat server instance to use for the remainder of this notebook.
server_args = []
from meshcat.servers.zmqserver import start_zmq_server_as_subprocess
proc, zmq_url, web_url = start_zmq_server_as_subprocess(server_args=server_args)


## import kinova drake which is not in the subdirectories here
import sys
sys.path.append('/root/kinova_drake/')

from kinova_station import KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget, KinovaStation
from controllers import CommandSequenceController, CommandSequence, Command
from observers.camera_viewer import CameraViewer

####################
# Helper Functions #
####################

def setup_gripper_command_systems(gripper_command_type,builder,station):
    # Set gripper command
    if gripper_command_type == GripperTarget.kPosition:
        q_grip_des = np.array([0.0])   # open at 0, closed at 1
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

def create_end_effector_target(ee_command_type,builder,station):
    """
    create_end_effector_target
    Description:
        Creates the type of commands that are sent to the arm and the gripper.
        Options are kPose, kTwist, or kWrench.
    """
    #ee_command_type = EndEffectorTarget.kPose      # kPose, kTwist, or kWrench

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

    # Create the system which outputs the End Effector Command's Type
    target_type_source = builder.AddSystem(ConstantValueSource(AbstractValue.Make(ee_command_type)))

    # Name the new systems
    target_source.set_name("ee_command_source")
    target_type_source.set_name("ee_type_source")

    # Connecting the New Systems (target_type_source and target_source)
    builder.Connect(
            target_type_source.get_output_port(),
            station.GetInputPort("ee_target_type")) # connects to a port ON THE STATION with the given name
    builder.Connect(
            target_source.get_output_port(),
            station.GetInputPort("ee_target"))


def create_infinity_scenario():
    """
    Description:
        Creates the 6 degree of freedom Kinova system in simulation. Anchors it to a "ground plane" and gives it the
        RobotiQ 2f_85 gripper.
        This should also initialize the meshcat visualizer so that we can easily view the robot.
    Usage:
    """

    # Constants
    gripper_type = '2f_85'

    # Start with the Kinova Station object
    station = KinovaStation(time_step=0.001,n_dof=6)

    # station.AddArm()
    # station.Add2f85Gripper()
    station.AddArmWith2f85Gripper()

    station.AddGround()
    station.AddCamera()
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
    cs = setup_infinity_command_sequence()
    controller = setup_controller_and_connect_to_station(cs,builder,station)

    # Build the system diagram
    diagram = builder.Build()
    diagram.set_name("system_diagram")
    diagram_context = diagram.CreateDefaultContext()

    # context = diagram.CreateDefaultContext()
    station.meshcat.load()
    diagram.Publish(diagram_context)

    # Return station
    return builder, controller, station, diagram, diagram_context

def setup_infinity_command_sequence():
    """
    Description:
        Creates the command sequence that we need to achieve the infinity sequence.
    """
    # Constants
    num_points_in_discretized_curve = 7
    infinity_center = np.array([0.6, 0.0, 0.5])
    infinity_center_pose = np.zeros((6,))
    infinity_center_pose[:3] = np.array([np.pi/2,0.0,0.5*np.pi])
    infinity_center_pose[3:] = infinity_center

    curve_radius = 0.2
    curve_duration = 6.0

    infinity_left_lobe_center = infinity_center - np.array([0.0,0.25,0.0])
    infinity_left_lobe_center_pose = np.zeros((6,))
    infinity_left_lobe_center_pose[:3] = np.array([np.pi/2,0.0,0.5*np.pi])
    infinity_left_lobe_center_pose[3:] = infinity_left_lobe_center 

    infinity_right_lobe_center = infinity_center + np.array([0.0,0.25,0.0])
    infinity_right_lobe_center_pose = np.zeros((6,))
    infinity_right_lobe_center_pose[:3] = np.array([np.pi/2,0.0,0.5*np.pi])
    infinity_right_lobe_center_pose[3:] = infinity_right_lobe_center

    # Create the command sequence object
    cs = CommandSequence([])

    # 1. Initial Command
    cs.append(Command(
        name="centering",
        target_pose=infinity_center_pose,
        duration=6,
        gripper_closed=False))    

    # 2. Lower Left Part of Infinity
    cs.append(Command(
        name="lower_left",
        target_pose=infinity_left_lobe_center_pose - np.array([0,0,0,0,0,curve_radius]),
        duration=4,
        gripper_closed=False
    ))
    # 3. Execute curve for left lobe
    theta_list = np.linspace(np.pi/2,np.pi*1.5,num=num_points_in_discretized_curve)
    flipped_theta_list = np.flipud(theta_list)
    for theta_index in range(1,len(flipped_theta_list)):
        theta = flipped_theta_list[theta_index]
        cs.append(Command(
            name="left_curve"+str(theta_index),
            target_pose=infinity_left_lobe_center_pose + np.array([0.0,0.0,0.0,0,curve_radius*np.cos(theta),curve_radius*np.sin(theta)]),
            duration= curve_duration/(num_points_in_discretized_curve),
            gripper_closed = False
        ))

    # 4. Go to lower right Part of infinity (through center)
    cs.append(Command(
        name="left_curve_to_right",
        target_pose = infinity_right_lobe_center_pose - np.array([0,0,0,0,0,curve_radius]),
        duration=8,
        gripper_closed=False
    ))
    # 5. Execute curve for right lobe
    theta_list = np.linspace(np.pi*1.5,np.pi*2.5,num=num_points_in_discretized_curve)
    for theta_index in range(1,len(theta_list)):
        theta = theta_list[theta_index]
        cs.append(Command(
            name="right_curve"+str(theta_index),
            target_pose=infinity_right_lobe_center_pose + np.array([0.0,0.0,0.0,0,curve_radius*np.cos(theta),curve_radius*np.sin(theta)]),
            duration= curve_duration/(num_points_in_discretized_curve),
            gripper_closed = False
        ))

    #6. Get Back to Center
    cs.append(Command(
        name="right_curve_to_home",
        target_pose = infinity_center_pose,
        duration=4,
        gripper_closed=False
    ))

    return cs

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
    Kp = np.diag([10,10,10,2,2,2])
    Kd = 2*np.sqrt(Kp)

    controller = builder.AddSystem(CommandSequenceController(
        cs,
        command_type=EndEffectorTarget.kTwist,  # Twist commands seem most reliable in simulation
        Kp=Kp,
        Kd=Kd))
    controller.set_name("controller")
    controller.ConnectToStation(builder, station)

    return controller

#########################
# Simulation Parameters #
#########################

# Make a plot of the inner workings of the station
show_station_diagram = False

# Make a plot of the diagram for this example, where only the inputs
# and outputs of the station are shown
show_toplevel_diagram = False

# Which gripper to use (hande or 2f_85)
gripper_type = "2f_85"

run = True

##########################
# Setting Up the Station #
##########################

# Set up the kinova station
builder, controller, station, diagram, diagram_context = create_infinity_scenario()
#station.SetArmPositions(diagram,diagram_context,np.array([0.1,0.0,0.0,2.0,0.0,0.2]))

if run:

    # First thing: send to home position
    station.go_home(diagram,diagram_context)

    # We use a simulator instance to run the example, but no actual simulation 
    # is being done: it's all on the hardware. 
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)  # not sure if this is correct

    # We'll use a super simple integration scheme (since we only update a dummy state)
    # and set the maximum timestep to correspond to roughly 40Hz 
    integration_scheme = "explicit_euler"
    time_step = 0.025
    #ResetIntegratorFromFlags(simulator, integration_scheme, time_step)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(40.0)

while True:
    1

