
"""

adaptive_pull_back.py
Description:
    This scripts contains the command sequence and controller to pull back an object on the table and release
    
"""

""" Imports """
import sys
sys.path.append('../')

import numpy as np
from kinova_drake.kinova_station import (KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget)
from adaptive_controller.adaptive_command_sequence_controller import AdaptiveCommandSequenceController
from adaptive_controller.adaptive_command_sequence import AdaptiveCommandSequence, AdaptiveCommand

def command_sequence():
    identification_pull = 0.15
    target_position = 0.65
    pscs = AdaptiveCommandSequence([])
    
    # T = 0
    # Initial Movement
    pscs.append(AdaptiveCommand(
        name="initial move",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2, 0.25]),
        gripper_value=0.0,
        duration=15.0))
    pscs.append(AdaptiveCommand(
        name="move down",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2, 0.025]),
        gripper_value=0.0,
        duration=15.0))
    
    # T=30
    # Measure the Reference Force
    pscs.append(AdaptiveCommand(
        name="pull back",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-identification_pull, 0.025]),
        gripper_value=0.0,
        duration=5.0))
    pscs.append(AdaptiveCommand(
        name="stabilize",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-identification_pull, 0.025]),
        gripper_value=0.75,
        duration=5.0))
    pscs.append(AdaptiveCommand(
        name="reference force",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-identification_pull, 0.025]),
        gripper_value=0.75,
        duration=10.0))
    pscs.append(AdaptiveCommand(
        name="return",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2, 0.025]),
        gripper_value=0.0,
        duration=10.0))
    
    # T = 60
    # Grasp and Pull Back
    pscs.append(AdaptiveCommand(
        name="grasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2, 0.025]),
        gripper_value=0.75,
        duration=3.0))
    pscs.append(AdaptiveCommand(
        name="pull back",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-identification_pull, 0.025]),
        gripper_value=0.75,
        duration=7.0))
    
    # T = 70
    # Measure Spring Coefficient
    pscs.append(AdaptiveCommand(
        name="spring coefficient",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-identification_pull, 0.025]),
        gripper_value=0.75,
        duration=30.0))

    # T = 100
    # Lift to reset and Replace to Origin
    pscs.append(AdaptiveCommand(
        name="reset wheels",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-identification_pull, 0.075]),
        gripper_value=0.75,
        duration=5.0))
    pscs.append(AdaptiveCommand(
        name="replace",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2, 0.025]),
        gripper_value=0.75,
        duration=10.0))
    pscs.append(AdaptiveCommand(
        name="pregrasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2, 0.025]),
        gripper_value=0.0,
        duration=3.0))
    pscs.append(AdaptiveCommand(
        name="grasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2, 0.025]),
        gripper_value=0.75,
        duration=2.0))
    
    # T = 120
    # Adaptive Pull Back & Release
    pscs.append(AdaptiveCommand(
        name="pull back",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, target_position, 0.025]),
        gripper_value=0.75,
        duration=5.0))
    pscs.append(AdaptiveCommand(
        name="release",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, target_position, 0.025]),
        gripper_value=0.0,
        duration=5.0))
    pscs.append(AdaptiveCommand(
        name="ending sequence",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, target_position, 0.025]),
        gripper_value=0.0,
        duration=5.0))
    
    # Controller Tuning
    twist_Kp = np.diag([4.5, 3.5, 4.5, 5.0, 20.0, 6.5])*0.075
    twist_Kd = np.sqrt(twist_Kp)*0.35 + np.diag([0, 0, 0, 0, 0, 0.01])
    wrench_Kp = np.diag([75, 75, 75, 1000, 1500, 1000])
    wrench_Kd = np.sqrt(wrench_Kp)*1.5
    
    controller = AdaptiveCommandSequenceController(
        pscs,
        twist_Kp = twist_Kp,
        twist_Kd = twist_Kd,
        wrench_Kp = wrench_Kp,
        wrench_Kd = wrench_Kd )
    
    return pscs, controller


