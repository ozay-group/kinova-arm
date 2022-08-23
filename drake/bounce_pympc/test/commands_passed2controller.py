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
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt

from pydrake.all import (
    AddMultibodyPlantSceneGraph, DiagramBuilder, 
    FindResourceOrThrow, GenerateHtml, InverseDynamicsController, 
    MultibodyPlant, Parser, Simulator, RigidTransform , RotationMatrix,
    ConstantValueSource, ConstantVectorSource, AbstractValue, 
    RollPitchYaw, LogVectorOutput, plot_system_graphviz,
    LeafSystem, BasicVector )
from pydrake.multibody.jupyter_widgets import MakeJointSlidersThatPublishOnCallback
  
# setting path
sys.path.append('/root/kinova_drake/')

from kinova_station import KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget, KinovaStation

sys.path.append('/root/kinova-arm/drake/')
# from controllers.velocity import VelocityCommand, VelocityCommandSequence, VelocityCommandSequenceController
from partial_state_controller.complex_controller import ComplexController
from partial_state_controller.partial_state_controller import HardwarePSCSequenceController
from partial_state_controller.partial_state_command_sequence import PartialStateCommand, PSCSequence
from observers.camera_viewer import CameraViewer

################################################################
class HardwarePSCSequenceController(ComplexController):
    """
    Description:
        A controller that attempts to execute each of the commands in the PSCSequence
        object given to it.

        Sends gripper position commands as well as end-effector twist/wrench commands.
    """

    def __init__(self,
                        twist_Kp = None, twist_Kd = None,
                        wrench_Kp = None, wrench_Kd = None ):
        """
        __init__
        Description:
            Constructor for CommandSequenceController objects.
        """
        ComplexController.__init__(self,command_type=EndEffectorTarget.kPose)

        self.comm = PartialStateCommand()

        #PD gains for Twist Controller
        if twist_Kp is None:
            twist_Kp = np.diag([10.0,10,10,2,2,2])*0.1

        if twist_Kd is None:
            twist_Kd = np.sqrt(0.0)*np.eye(6)

        self.twist_Kp = twist_Kp
        self.twist_Kd = twist_Kd

        # PD Gains for Wrench Controller
        if wrench_Kp is None:
            wrench_Kp = np.diag([75.0, 75, 75, 1500, 1500, 1500])

        if wrench_Kd is None:
            wrench_Kd =  0.05*np.sqrt(wrench_Kp)
            
        self.wrench_Kp = wrench_Kp
        self.wrench_Kd = wrench_Kd

        self.AbsValue = self.AbstractValue().Make(PartialStateCommand)
        self.AbsValue.set_value(self.comm)
        self.command_input_port = self.DeclareAbstractInputPort(name="command_input", model_value=self.AbsValue.get_value())

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
        command_t = self.comm

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

            # print(pose_err)
            # print("twist_err = ")
            # print(twist_err)

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

            # print("cmd = ")
            # print(cmd)

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

            print("target_twist = " + str(target_twist))
            print("twist_err = " + str(twist_err))

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
            # cmd = target_twist

        else:
            cmd = np.zeros(6)

        print(cmd)

        # Return Output
        output.SetFromVector(cmd)

    def SetEndEffectorCommandType(self, context, output):
        """
        SetEndEffectorCommandType
        Description:
            Sets the end effector command type.
        """
        # Get current command
        t = context.get_time()
        print("t = %s" % t)

        # Get Target End-Effector Target Type
        command_t = self.comm
        if command_t.ee_target_type == EndEffectorTarget.kPose:
            # For Pose Control
            ee_command_type = EndEffectorTarget.kTwist

        elif command_t.ee_target_type == EndEffectorTarget.kTwist:
            # For Twist Control
            ee_command_type = EndEffectorTarget.kWrench

        else:
            ee_command_type = EndEffectorTarget.kTwist

        print("Current command type = " + str(ee_command_type))
        
        output.SetFrom(AbstractValue.Make(ee_command_type))

    def ConnectToStation(self, builder, station, time_step=-1.0):
        """
        Connect inputs and outputs of this controller to the given kinova station (either
        hardware or simulation). 
        """

        # Construct Default Value for time_step
        if time_step < 0.0:
            if isinstance(station,KinovaStation):
                time_step = station.plant.time_step()
            else:
                raise Exception("Time step should be given when running ConnectToStation() on the HarwareKinovaStation.")

        # Create a simple delay block
        delay_block = builder.AddSystem(DiscreteTimeDelay(
            time_step, # Setting the update_sec (width of each discrete step)
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




################################################################
