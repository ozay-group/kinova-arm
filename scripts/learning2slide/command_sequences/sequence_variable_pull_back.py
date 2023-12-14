
"""

variable_pull_back.py
Description:
    This scripts contains the command sequence and controller to pull back an object on the table and release
    
"""

""" Imports """

import numpy as np
from kinova_drake.kinova_station import (KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget)
from kinova_drake.controllers import (PSCommandSequenceController, PSCommandSequence, PartialStateCommand)

def command_sequence():
    
    windup_coefficient = 5.5747
    start_location = -0.1
    target_location = 0.65
    pull_dist = (target_location - start_location)/windup_coefficient
    
    reset_dist = 0.2
    pscs = PSCommandSequence([])
    pscs.append(PartialStateCommand(
        name="initial move",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, start_location, 0.25]),
        gripper_value=0.0,
        duration=15.0))
    pscs.append(PartialStateCommand(
        name="move down",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, start_location, 0.025]),
        gripper_value=0.0,
        duration=15.0))
    # T=30
    pscs.append(PartialStateCommand(
        name="pregrasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, start_location, 0.025]),
        gripper_value=0.0,
        duration=2.0))
    pscs.append(PartialStateCommand(
        name="grasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, start_location, 0.025]),
        gripper_value=0.75,
        duration=3.0))
    pscs.append(PartialStateCommand(
        name="pull back",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, start_location-reset_dist, 0.025]),
        gripper_value=0.75,
        duration=5.0))
    pscs.append(PartialStateCommand(
        name="reset wheels",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, start_location-reset_dist, 0.075]),
        gripper_value=0.75,
        duration=5.0))
    pscs.append(PartialStateCommand(
        name="replace",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, start_location, 0.025]),
        gripper_value=0.75,
        duration=15.0))
    # T=60
    pscs.append(PartialStateCommand(
        name="pull back",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, start_location-pull_dist, 0.025]),
        gripper_value=0.75,
        duration=5.0))
    pscs.append(PartialStateCommand(
        name="release",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, start_location-pull_dist, 0.025]),
        gripper_value=0.0,
        duration=5.0))
    # T=70  
    pscs.append(PartialStateCommand(
        name="home position",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, start_location-pull_dist, 0.1]),
        gripper_value=0.0,
        duration=5.0))
    
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