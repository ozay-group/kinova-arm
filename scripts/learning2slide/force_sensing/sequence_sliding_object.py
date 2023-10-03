
"""

sequence_sliding_object.py
Description:
    This scripts contains the command sequence and controller to slide an object on the table
    
"""

""" Imports """

import numpy as np
from kinova_drake.kinova_station import (KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget)
from kinova_drake.controllers import (PSCommandSequenceController, PSCommandSequence, PartialStateCommand)

def sliding_object():
    
    pscs = PSCommandSequence([])
    pscs.append(PartialStateCommand(
        name="initial move",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.5, 0.25]),
        gripper_value=0.0,
        duration=10))
    pscs.append(PartialStateCommand(
        name="move down",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.5, 0.025]),
        gripper_value=0.0,
        duration=10))
    pscs.append(PartialStateCommand(
        name="pregrasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.5, 0.025]),
        gripper_value=0.0,
        duration=7))
    pscs.append(PartialStateCommand(
        name="grasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.5, 0.025]),
        gripper_value=0.45,
        duration=3))
    pscs.append(PartialStateCommand(
        name="accelerate",
        target_type=EndEffectorTarget.kTwist,
        target_value=np.array([0.0*np.pi, 0.0*np.pi, 0.0*np.pi, 0.0, 25.0, 0.001]),
        gripper_value=0.45,
        duration=0.15))
    pscs.append(PartialStateCommand(
        name="release",
        target_type=EndEffectorTarget.kTwist,
        target_value=np.array([0.0*np.pi, 0.0*np.pi, 0.0*np.pi, 0.0, 25.0, 0.1]),
        gripper_value=0.0,
        duration=0.5))
    pscs.append(PartialStateCommand(
        name="end move",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([0.5*np.pi, 0.0*np.pi, 0.5*np.pi, 0.15, 0.0, 0.5]),
        gripper_value=0.0,
        duration=5))
    
    twist_Kp = np.diag([3.5, 3.5, 3.5, 3.0, 4.0, 6.5])*0.075
    twist_Kd = np.sqrt(twist_Kp)*0.35 + np.diag([0, 0, 0, 0, 0, 0.01])
    wrench_Kp = np.diag([75.0, 75, 75, 1500, 1500, 1500])
    wrench_Kd = np.sqrt(wrench_Kp)*0.125 + np.diag([0, 0, 0, 0, 0, 0])

    controller = PSCommandSequenceController(
        pscs,
        twist_Kp = twist_Kp,
        twist_Kd = twist_Kd,
        wrench_Kp = wrench_Kp,
        wrench_Kd = wrench_Kd )
    
    return pscs, controller