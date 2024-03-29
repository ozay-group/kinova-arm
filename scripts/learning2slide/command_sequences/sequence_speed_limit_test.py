
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
        name="move low",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([1.0*np.pi, 0.0*np.pi, 1.0*np.pi, 0.5, 0.0, 0.25]),
        gripper_value=0.0,
        duration=15.0))
    pscs.append(PartialStateCommand(
        name="speed up",
        target_type=EndEffectorTarget.kTwist,
        target_value=np.array([0.0*np.pi, 0.0*np.pi, 0.0*np.pi, 0.0, 0.0, 0.5]),
        gripper_value=0.0,
        duration=2.0))
    pscs.append(PartialStateCommand(
        name="home position",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([0.5*np.pi, 0.0*np.pi, 0.5*np.pi, 0.3, 0.0, 0.3]),
        gripper_value=0.0,
        duration=10.0))
    
    twist_Kp = np.diag([3.5, 3.5, 3.5, 3.0, 4.0, 6.5])*0.075
    twist_Kd = np.sqrt(twist_Kp)*0.35 + np.diag([0, 0, 0, 0, 0, 0.01])
    wrench_Kp = np.diag([75, 75, 75, 1500, 1500, 1500])
    wrench_Kd = np.diag([0.4, 0.4, 0.4, 2, 2, 2])

    controller = PSCommandSequenceController(
        pscs,
        twist_Kp = twist_Kp,
        twist_Kd = twist_Kd,
        wrench_Kp = wrench_Kp,
        wrench_Kd = wrench_Kd )
    
    return pscs, controller