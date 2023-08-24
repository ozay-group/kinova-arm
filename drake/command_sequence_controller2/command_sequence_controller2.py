from kinova_station import EndEffectorTarget, KinovaStation, KinovaStationHardwareInterface
from command_sequence_controller2.command_sequence2 import ComplexCommand, cCommandSequence
from command_sequence_controller2.complex_controller2 import  ComplexController

import numpy as np

from pydrake.all import (
    DependencyTicket, RotationMatrix, RollPitchYaw, DiscreteTimeDelay
)

class CommandSequenceController(ComplexController):
    """
    Description:
        A controller that attempts to execute each of the commands in the cCommandSequence
        object given to it.

        Sends gripper position commands as well as end-effector twist/wrench commands.
    """

    def __init__(self, command_sequence,
                        twist_Kp = np.diag([10,10,10,2,2,2]), twist_Kd = 2*np.sqrt(10)*np.eye(6),
                        wrench_Kp = np.diag([10,10,10,2,2,2])*10, wrench_Kd = 0.0*np.eye(6) ):
        """
        __init__
        Description:
            Constructor for CommandSequenceController objects.
        """
        ComplexController.__init__(self,command_type=EndEffectorTarget.kPose)

        self.cs = command_sequence

        #PD gains for Twist Controller
        self.twist_Kp = twist_Kp
        self.twist_Kd = twist_Kd

        # PD Gains for Wrench Controller
        self.wrench_Kp = wrench_Kp
        self.wrench_Kd = wrench_Kd

    def CalcGripperCommand(self,context,output):
        """
        Description:
            Computes the gripper command (position to consider).
        """
        t = context.get_time()

        cmd_pos = np.array([self.cs.current_command(t).gripper_target_value])
        output.SetFromVector(cmd_pos)

    def CalcEndEffectorCommand(self,context,output):
        """
        CalcEndEffectorCommand
        Description:
            Computes the output command to send to the controller. 
        """
        t = context.get_time()
        print("t = %s" % t)

        # Get Target End-Effector Target Type
        command_t = self.cs.current_command(t)

        if command_t.ee_target_type == EndEffectorTarget.kPose:
            # For Pose Control
            self.command_type = EndEffectorTarget.kTwist

            # Get target end-effector pose and twist
            target_pose = command_t.ee_target_value
            target_twist = np.zeros(6)

            # Get current end-effector pose and twist
            current_pose = self.ee_pose_port.Eval(context)
            current_twist = self.ee_twist_port.Eval(context)

            # Compute pose and twist errors
            pose_err  = target_pose - current_pose
            twist_err = target_twist - current_twist

            print(pose_err)

            # Use rotation matrices to compute the difference between current and
            # desired end-effector orientations. This helps avoid gimbal lock as well 
            # as issues like taking the shortest path from theta=0 to theta=2*pi-0.1
            R_current = RotationMatrix(RollPitchYaw(current_pose[:3]))
            R_target = RotationMatrix(RollPitchYaw(target_pose[:3]))
            R_err = R_target.multiply(R_current.transpose())
            pose_err[:3] = RollPitchYaw(R_err).vector()

            # Set command (i.e. end-effector twist or wrench) using a PD controller
            Kp = self.twist_Kp
            Kd = self.twist_Kd
            cmd = Kp@pose_err + Kd@twist_err

        elif command_t.ee_target_type == EndEffectorTarget.kTwist:
            # For Twist Control
            self.command_type = EndEffectorTarget.kWrench

            # Get target end-effector twist and wrench
            target_twist = command_t.ee_target_value
            target_wrench = np.zeros(6)

            # Get current end-effector pose and twist
            current_twist = self.ee_twist_port.Eval(context)
            current_wrench = self.ee_wrench_port.Eval(context)

            # Compute pose and twist errors
            twist_err = target_twist - current_twist
            wrench_err = target_wrench - current_wrench

            # # Use rotation matrices to compute the difference between current and
            # # desired end-effector orientations. This helps avoid gimbal lock as well 
            # # as issues like taking the shortest path from theta=0 to theta=2*pi-0.1
            # R_current = RotationMatrix(RollPitchYaw(current_pose[:3]))
            # R_target = RotationMatrix(RollPitchYaw(target_pose[:3]))
            # R_err = R_target.multiply(R_current.transpose())
            # pose_err[:3] = RollPitchYaw(R_err).vector()

            # Set command (i.e. end-effector twist or wrench) using a PD controller
            Kp = self.wrench_Kp
            Kd = self.wrench_Kd
            cmd = Kp@twist_err + Kd@wrench_err

        else:
            cmd = np.zeros(6)

        # print(cmd)

        # Return Output
        output.SetFromVector(cmd)

    def ConnectToStation(self, builder, station):
        """
        Connect inputs and outputs of this controller to the given kinova station (either
        hardware or simulation). 
        """

        if isinstance(station,KinovaStation):
            # If the station is the simulation station, then we need to insert something
            # between commands and wrench port in order to avoid algebraic loops.

            # Create a simple delay block
            delay_block = builder.AddSystem(DiscreteTimeDelay(
                station.plant.time_step(), # Setting the update_sec (width of each discrete step)
                1, # Setting the number of discrete steps to wait
                6  # Size of the input to the delay block
            ))

            #Connect: ee_command output port -> delay -> the station target
            builder.Connect(
                self.GetOutputPort("ee_command"),
                delay_block.get_input_port()
            )
            builder.Connect(                                  # Send commands to the station
                    delay_block.get_output_port(),
                    station.GetInputPort("ee_target"))
        elif isinstance(station,KinovaStationHardwareInterface):
            # If the station is the HARDWARE station, then we need to insert something
            # between commands and wrench port in order to avoid algebraic loops.

            # Create a simple delay block
            delay_block = builder.AddSystem(DiscreteTimeDelay(
                0.025, # Setting the update_sec (width of each discrete step)
                1, # Setting the number of discrete steps to wait
                6  # Size of the input to the delay block
            ))

            #Connect: ee_command output port -> delay -> the station target
            builder.Connect(
                self.GetOutputPort("ee_command"),
                delay_block.get_input_port()
            )
            builder.Connect(                                  # Send commands to the station
                    delay_block.get_output_port(),
                    station.GetInputPort("ee_target"))

        else:
            # If the station is the hardware station, then there is no need to insert something
            # between commands and wrench port

            builder.Connect(                                  # Send commands to the station
                    self.GetOutputPort("ee_command"),
                    station.GetInputPort("ee_target"))
        
        # Connect the command type port to the station
        builder.Connect(
                self.GetOutputPort("ee_command_type"),
                station.GetInputPort("ee_target_type"))

        # Connect Gripper Commands to the station
        builder.Connect(
                self.GetOutputPort("gripper_command"),
                station.GetInputPort("gripper_target"))
        builder.Connect(
                self.GetOutputPort("gripper_command_type"),
                station.GetInputPort("gripper_target_type"))

        builder.Connect(                                     # Send state information
                station.GetOutputPort("measured_ee_pose"),   # to the controller
                self.GetInputPort("ee_pose"))
        builder.Connect(
                station.GetOutputPort("measured_ee_twist"),
                self.GetInputPort("ee_twist"))
        builder.Connect(
                station.GetOutputPort("measured_ee_wrench"),
                self.GetInputPort("ee_wrench"))