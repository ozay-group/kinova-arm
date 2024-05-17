import numpy as np
import time
from kinova_drake.controllers import (
    BasicController)
from kinova_drake.kinova_station import EndEffectorTarget

from pydrake.all import BasicVector


class StaticController(BasicController):
    """
    A simple controller which drives the arm to a given target end-effector pose/twist
    and gripper states. See BasicController for IO details. 

    Sends exclusively gripper position commands and end-effector pose commands.
    """
    def __init__(self, command_type = EndEffectorTarget.kPose):

        BasicController.__init__(self, command_type=command_type)
        self.target_pose_ = np.array([0,0,0,0,0,0]) 
        self.SetTargetPose(np.array([np.pi/2.0, 0, np.pi/2.0, 0.57, 0, 0.42]))
        self.gripper_pos_ = 0.0
        self.delay_count_ = 0
        self.sliding_mode_ = 0
        self.release_pos_ = None
        self.sliding_t0 = time.time()
        self.gripper_position_port = self.DeclareVectorInputPort("gripper_position", 
                                                                   BasicVector(1))
    
    def SetTargetPose(self, pose):
        self.target_pose_ = pose

    def SetGripperPos(self, gripper_pos):
        self.gripper_pos_ = gripper_pos
    
    def SetSlidingMode(self, release_pos):
        self.sliding_mode_ = 1
        self.release_pos_ = release_pos
        self.current_vel_ = 0
        # switch EE command type to kTwist
        self.command_type = EndEffectorTarget.kTwist
        self.delay_count_ = 0

    def CalcGripperCommand(self, context, output):
        output.SetFromVector([self.gripper_pos_])

    def CalcEndEffectorCommand(self, context, output):
        """
        Compute and send an end-effector twist command. This is just a
        simple PD controller.
        """
        # Get target end-effector pose and twist
        pose = self.ee_pose_port.Eval(context)
        y = pose[4]
        if self.sliding_mode_ == 1:
            if y <= self.release_pos_: # keep the max velocity before release
                print("I am sliding...")
                target_twist = np.array([0,0,0,0,0.4,0]) # maximum speed in y direction
                output.SetFromVector(target_twist)
            else:
                print("I am stopping...")
                self.gripper_pos_ = 0
                # if self.delay_count_ < 5:
                #     # delay the deceleration for 5 time steps to count the time for the gripper to release
                #     target_twist = np.array([0,0,0,0,0.5,0]) # maximum speed in y direction
                #     self.delay_count_ += 1
                # else:
                target_twist = np.array([0,0,0,0,0.0,0]) # stop after reaching the release position
                output.SetFromVector(target_twist)
                if self.ee_twist_port.Eval(context)[4] < 1e-3: # the ee has stopped
                    self.sliding_mode_ = 0 # exit the sliding mode
                    self.delay_count_ = 0
                    self.command_type = EndEffectorTarget.kPose # switch back to position control mode
                    self.target_pose_ = pose
        else:
            target_pose = self.target_pose_
            output.SetFromVector(target_pose)

    def ConnectToStation(self, builder, station):
        """
        Connect inputs and outputs of this controller to the given kinova station (either
        hardware or simulation). 
        """
        builder.Connect(                                  # Send commands to the station
                self.GetOutputPort("ee_command"),
                station.GetInputPort("ee_target"))
        builder.Connect(
                self.GetOutputPort("ee_command_type"),
                station.GetInputPort("ee_target_type"))
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
                station.GetOutputPort("measured_gripper_position"),
                self.GetInputPort("gripper_position")
        )