
"""

sequence_iterations_and_slide.py
Description:
    This scripts contains the command sequence and controller to slide an object on the table
    back and forth - in air, then drag, then slide!
    
"""

""" Imports """

import numpy as np
from kinova_drake.kinova_station import (KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget)
from kinova_drake.controllers import (PSCommandSequenceController, PSCommandSequence, PartialStateCommand)

def command_sequence():
    pscs = PSCommandSequence([])
    
    # Time = 0s, Initial movement sequence to the sliding start pose
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
        duration=15.0))

    # Time = 30s, Sliding in air, for reference characteristic generation
    for rep in range(10):
        pscs.append(PartialStateCommand(
            name="pregrasp",
            target_type=EndEffectorTarget.kPose,
            target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.35, 0.025]),
            gripper_value=0.0,
            duration=2.0))
        pscs.append(PartialStateCommand(
            name="accelerate",
            target_type=EndEffectorTarget.kPose,
            target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, 1.0, 0.025]),
            gripper_value=0.0,
            duration=1.0))
        pscs.append(PartialStateCommand(
            name="release",
            target_type=EndEffectorTarget.kPose,
            target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, 0.05, 0.025]),
            gripper_value=0.0,
            duration=2.0))
        pscs.append(PartialStateCommand(
            name="return",
            target_type=EndEffectorTarget.kPose,
            target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.35, 0.025]),
            gripper_value=0.0,
            duration=5.0))
    
    # Time = 130s, Minimal Actions
    pscs.append(PartialStateCommand(
        name="pregrasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.35, 0.025]),
        gripper_value=0.0,
        duration=2.5))
    pscs.append(PartialStateCommand(
        name="grasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.35, 0.025]),
        gripper_value=0.25,
        duration=2.5))
    
    # Time = 135, Dragging with the Minicar, initial estimation generation
    for rep in range(5):
        pscs.append(PartialStateCommand(
            name="grasp",
            target_type=EndEffectorTarget.kPose,
            target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.35, 0.025]),
            gripper_value=0.25,
            duration=2.0))
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
            gripper_value=0.25,
            duration=2.0))
        pscs.append(PartialStateCommand(
            name="return",
            target_type=EndEffectorTarget.kPose,
            target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.35, 0.025]),
            gripper_value=0.25,
            duration=5.0))

    # Time = 185s, Minimal Actions
    pscs.append(PartialStateCommand(
        name="pregrasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.35, 0.025]),
        gripper_value=0.0,
        duration=2.5))
    pscs.append(PartialStateCommand(
        name="grasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.35, 0.025]),
        gripper_value=0.25,
        duration=2.5))
    
    # Time = 190s, Actual Sliding Sequence
    pscs.append(PartialStateCommand(
        name="grasp",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.3, -0.35, 0.025]),
        gripper_value=0.25,
        duration=2.0))
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
        duration=2.0))
        
    # Time = 195s, Ending Sequence
    pscs.append(PartialStateCommand(
        name="home position",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([0.5*np.pi, 0.0*np.pi, 0.5*np.pi, 0.3, 0.0, 0.3]),
        gripper_value=0.0,
        duration=5.0))
    
    # Time = 200s
    
    # Controller Tuning
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