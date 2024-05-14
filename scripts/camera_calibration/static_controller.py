import numpy as np

from kinova_drake.controllers import (
    BasicController)
from kinova_drake.kinova_station import EndEffectorTarget


class StaticController(BasicController):
    """
    A simple controller which drives the arm to a gien target end-effector pose
    and gripper states (open or closed). See BasicController for IO details. 

    Sends exclusively gripper position commands and end-effector pose commands.
    """
    def __init__(self, command_type = EndEffectorTarget.kPose):

        BasicController.__init__(self, command_type=command_type)
        self.target_pose_ = np.array([0,0,0,0,0,0]) 
        self.SetTargetPose(np.array([np.pi/2.0, 0, np.pi/2.0, 0.57, 0, 0.42]))
        self.gripper_pos_ = np.array([0.0])
    
    def SetCalibrationTargetPose(self, roll, pitch, yaw, width, depth, height):
        self.target_pose_ = np.array([
                            0.5*np.pi + 0.1*np.pi*roll,
                            0.0*np.pi + 0.1*np.pi*pitch,
                            0.6*np.pi + 0.1*np.pi*yaw,
                            0.6 + 0.1*width,
                            0.3 + 0.1*depth,
                            0.2 + 0.1*height
                            ])

    def SetTargetPose(self, pose):
        self.target_pose_ = pose

    def SetGripperPos(self, gripper_pos):
        self.gripper_pos_ = np.array(gripper_pos)


    def CalcGripperCommand(self, context, output):
        # t = context.get_time()
        output.SetFromVector(self.gripper_pos_)

    def CalcEndEffectorCommand(self, context, output):
        """
        Compute and send an end-effector twist command. This is just a
        simple PD controller.
        """
        # Get target end-effector pose and twist
        target_pose = self.target_pose_
        # target_twist = np.zeros(6)

        # # Get current end-effector pose and twist
        current_pose = self.ee_pose_port.Eval(context)
        # current_twist = self.ee_twist_port.Eval(context)

        # # Compute pose and twist errors
        # twist_err = target_twist - current_twist
        # print("Pose error:", target_pose - current_pose)

        # # Use rotation matrices to compute the difference between current and
        # # desired end-effector orientations. This helps avoid gimbal lock as well 
        # # as issues like taking the shortest path from theta=0 to theta=2*pi-0.1
        # R_current = RotationMatrix(RollPitchYaw(current_pose[:3]))
        # R_target = RotationMatrix(RollPitchYaw(target_pose[:3]))
        # R_err = R_target.multiply(R_current.transpose())
        # pose_err[:3] = RollPitchYaw(R_err).vector()

        # # Set command (i.e. end-effector twist or wrench) using a PD controller
        # cmd = self.Kp@pose_err + self.Kd@twist_err

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