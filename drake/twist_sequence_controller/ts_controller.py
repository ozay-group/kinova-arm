from kinova_station import EndEffectorTarget, KinovaStation
from twist_sequence_controller.command_sequence import Command, CommandSequence
from twist_sequence_controller.controller import  Controller

import numpy as np

from pydrake.all import (
    DependencyTicket, RotationMatrix, RollPitchYaw,
    DiscreteTimeDelay
)

class TwistSequenceController(Controller):
    """
    Description:
        A controller that attempts to execute each of the commands in the CommandSequence
        object given to it.

        Sends gripper position commands as well as end-effector twist/wrench commands.
    """

    def __init__(self, command_sequence,
                        Kp = np.diag([10,10,10,2,2,2])*10, Kd = np.eye(6)*np.power(10.0,-2)):
        """
        __init__
        Description:
            Constructor for CommandSequenceController objects.
        """
        Controller.__init__(self,command_type=EndEffectorTarget.kWrench)

        self.cs = command_sequence

        #PD gains for Twist Controller
        self.Kp = Kp
        self.Kd = Kd

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
        #print(command_t)

        # For Twist Control
        self.command_type = EndEffectorTarget.kTwist

        # Get target end-effector twist and wrench
        target_twist = command_t.ee_target_twist
        target_wrench = np.zeros(6)

        # Get current end-effector pose and twist
        current_twist = self.ee_twist_port.Eval(context)
        current_wrench = self.ee_wrench_port.Eval(context)

        # Compute pose and twist errors
        twist_err = target_twist - current_twist
        wrench_err = target_wrench - current_wrench

        # Set command (i.e. end-effector twist or wrench) using a PD controller
        Kp = self.Kp
        Kd = self.Kd
        cmd = Kp@twist_err + Kd@wrench_err

        #print(cmd)

        # Return Output
        output.SetFromVector(cmd)

    def ConnectToStation(self, builder, station):
        """
        Connect inputs and outputs of this controller to the given kinova station (either
        hardware or simulation). 
        """
        print(type(station))
        print(isinstance(station,KinovaStation))

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

        builder.Connect(                                    # Send state information
                station.GetOutputPort("measured_ee_twist"), # to the controller
                self.GetInputPort("ee_twist"))
        builder.Connect(
                station.GetOutputPort("measured_ee_wrench"),
                self.GetInputPort("ee_wrench"))