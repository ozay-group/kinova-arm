'''control_module.py
    Description:
        This function construct controls and commands to manipulate the Kinova arm. 
        This is applicable both in simulation or hardware.
        This script takes the infinity_demo.py as an example.
'''

###########
# Imports #
###########

from pydrake.all import *
import numpy as np
import sys
sys.path.append('/root/kinova_drake/')
from controllers import CommandSequenceController, CommandSequence, Command
from kinova_station import KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget, KinovaStation
from observers.camera_viewer import CameraViewer

#####################
# Controller design #
#####################

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

#############################
# Example Controller Design #
#############################

def setup_example_command_sequence():
    """
    Description:
        This is an example of a command sequence.
        It creates the command sequence that we need to achieve the infinity sequence.
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

def setup_example_controller_and_connect_to_station(builder,station):
    """
    Description:
        Defines the controller (PID) which is a CommandSequenceController as defined in
        kinova_drake.
    Inputs:
        cs = A CommandSequence object which helps define the CommandSequenceController.
    """

    # Get the command sequence
    cs = setup_example_command_sequence()

    # Create the controller and connect inputs and outputs appropriately
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

################
# Design space #
################

def setup_command_sequence():
    """_summary_
        Creates a command sequence.
        ## A placeholder for now. ##
    Returns:
        _type_: _description_
    """
    # Create the command sequence object
    cs = CommandSequence([])
    return cs

def setup_controller_and_connect_to_station(builder,station):
    """_summary_

    Args:
        builder (_type_): _description_
        station (_type_): _description_
    """
     # Get the command sequence
    cs = setup_command_sequence()

    # Create the controller and connect inputs and outputs appropriately
    Kp = np.eye(6)
    Kd = np.zeros(6)
    Ki = np.zeros(6)

    controller = builder.AddSystem(CommandSequenceController(
        cs,
        command_type=EndEffectorTarget.kTwist,  # Twist commands seem most reliable in simulation
        Kp=Kp,
        Kd=Kd,
        Ki=Ki))
    controller.set_name("controller")
    controller.ConnectToStation(builder, station)
    return controller

########
# Main #
########

if __name__ == "__main__":
    """_summary_
        Make a plot of the diagram for this example, where only the inputs
        and outputs of the station are shown
    """
    show_toplevel_diagram = True
