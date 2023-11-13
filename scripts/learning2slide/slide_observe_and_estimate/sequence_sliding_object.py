
"""

sequence_sliding_object.py
Description:
    This scripts contains the command sequence and controller to slide an object on the table
    
"""

""" Imports """

import numpy as np
from kinova_drake.kinova_station import (KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget)
from kinova_drake.controllers import (PSCommandSequenceController, PSCommandSequence, PartialStateCommand)

def command_sequence():
    
    pscs = PSCommandSequence([])
    pscs.append(PartialStateCommand(
        name="initial move",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.35, 0.25]),
        gripper_value=0.0,
        duration=15.0))
    pscs.append(PartialStateCommand(
        name="move down",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.35, 0.025]),
        gripper_value=0.0,
        duration=13.0))
    pscs.append(PartialStateCommand(
        name="pregrasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.35, 0.025]),
        gripper_value=0.0,
        duration=4.0))
    pscs.append(PartialStateCommand(
        name="grasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.35, 0.025]),
        gripper_value=0.25,
        duration=3.0))
    pscs.append(PartialStateCommand(
        name="accelerate",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, 1.0, 0.025]),
        gripper_value=0.25,
        duration=1.0))
    pscs.append(PartialStateCommand(
        name="release",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, 0.05, 0.025]),
        gripper_value=0.0,
        duration=4.0))
            
    # pscs.append(PartialStateCommand(
    #     name="accelerate",
    #     target_type=EndEffectorTarget.kTwist,
    #     target_value=np.array([0.0*np.pi, 0.0*np.pi, 0.0*np.pi, 0.0035, 5.0, 0.0015]),
    #     gripper_value=0.25,
    #     duration=2.0))
    # pscs.append(PartialStateCommand(
    #     name="release",
    #     target_type=EndEffectorTarget.kTwist,
    #     target_value=np.array([0.0*np.pi, 0.0*np.pi, 0.0*np.pi, 0.0035, 5.0, 0.0015]),
    #     gripper_value=0.0,
    #     duration=1.0))
    
    # pscs.append(PartialStateCommand(
    #     name="home position",
    #     target_type=EndEffectorTarget.kPose,
    #     target_value=np.array([0.5*np.pi, 0.0*np.pi, 0.5*np.pi, 0.3, 0.0, 0.3]),
    #     gripper_value=0.0,
    #     duration=5.0))
    
    twist_Kp = np.diag([4.5, 3.5, 4.5, 5.0, 20.0, 6.5])*0.075
    twist_Kd = np.sqrt(twist_Kp)*0.35 + np.diag([0, 0, 0, 0, 0, 0.01])
    wrench_Kp = np.diag([75, 75, 75, 1000, 1500, 1000])
    wrench_Kd = np.sqrt(wrench_Kp)*1.5
    controller = PSCommandSequenceController(
        pscs,
        twist_Kp = twist_Kp,
        twist_Kd = twist_Kd,
        wrench_Kp = wrench_Kp,
        wrench_Kd = wrench_Kd )
    
    return pscs, controller