
"""

pull_back_and_release.py
Description:
    This scripts contains the command sequence and controller to pull back an object on the table and release
    
"""

""" Imports """

import numpy as np
from kinova_drake.kinova_station import (KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget)
from kinova_drake.controllers import (PSCommandSequenceController, PSCommandSequence, PartialStateCommand)

def command_sequence():
    pull_dist = 0.15
    
    pscs = PSCommandSequence([])
    # T = 0
    # Initial Movement
    pscs.append(PartialStateCommand(
        name="initial move",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2, 0.25]),
        gripper_value=0.0,
        duration=15.0))
    pscs.append(PartialStateCommand(
        name="move down",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2, 0.025]),
        gripper_value=0.0,
        duration=15.0))
    # T=30
    # Measure the Reference Force
    pscs.append(PartialStateCommand(
        name="pull back",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-pull_dist, 0.025]),
        gripper_value=0.0,
        duration=5.0))
    pscs.append(PartialStateCommand(
        name="stabilize",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-pull_dist, 0.025]),
        gripper_value=0.75,
        duration=5.0))
    pscs.append(PartialStateCommand(
        name="reference force",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-pull_dist, 0.025]),
        gripper_value=0.75,
        duration=10.0))
    pscs.append(PartialStateCommand(
        name="return",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2, 0.025]),
        gripper_value=0.0,
        duration=10.0))
    # T = 60
    # Grasp and Pull Back
    pscs.append(PartialStateCommand(
        name="grasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2, 0.025]),
        gripper_value=0.75,
        duration=3.0))
    pscs.append(PartialStateCommand(
        name="pull back",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-pull_dist, 0.025]),
        gripper_value=0.75,
        duration=7.0))
    # T = 70
    # Measure Spring Coefficient
    pscs.append(PartialStateCommand(
        name="spring coefficient",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-pull_dist, 0.025]),
        gripper_value=0.75,
        duration=30.0))
    # pscs.append(PartialStateCommand(
    #     name="pull back 02",
    #     target_type=EndEffectorTarget.kPose,
    #     target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-pull_dist*2/3, 0.025]),
    #     gripper_value=0.75,
    #     duration=10.0))
    # pscs.append(PartialStateCommand(
    #     name="pull back 03",
    #     target_type=EndEffectorTarget.kPose,
    #     target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-pull_dist*3/3, 0.025]),
    #     gripper_value=0.75,
    #     duration=10.0))

    # T = 100
    # Release and Return to Home Position
    pscs.append(PartialStateCommand(
        name="release",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-pull_dist, 0.025]),
        gripper_value=0.0,
        duration=5.0))
    pscs.append(PartialStateCommand(
        name="home position",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-pull_dist, 0.025]),
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

    # pscs.append(PartialStateCommand(
    #     name="pregrasp",
    #     target_type=EndEffectorTarget.kPose,
    #     target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2, 0.025]),
    #     gripper_value=0.0,
    #     duration=2.0))
    # pscs.append(PartialStateCommand(
    #     name="grasp",
    #     target_type=EndEffectorTarget.kPose,
    #     target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2, 0.025]),
    #     gripper_value=0.75,
    #     duration=3.0))
    # pscs.append(PartialStateCommand(
    #     name="pull back",
    #     target_type=EndEffectorTarget.kPose,
    #     target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-reset_dist, 0.025]),
    #     gripper_value=0.75,
    #     duration=5.0))
    # pscs.append(PartialStateCommand(
    #     name="reset wheels",
    #     target_type=EndEffectorTarget.kPose,
    #     target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2-reset_dist, 0.075]),
    #     gripper_value=0.75,
    #     duration=5.0))
    # pscs.append(PartialStateCommand(
    #     name="replace",
    #     target_type=EndEffectorTarget.kPose,
    #     target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.2, 0.025]),
    #     gripper_value=0.75,
    #     duration=15.0))