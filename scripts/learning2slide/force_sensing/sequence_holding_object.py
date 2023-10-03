
"""

sequence_holding_object.py
Description:
    This scripts contains the command sequence and controller to hold an object in the air
    
"""

""" Imports """

import numpy as np
from kinova_drake.kinova_station import (KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget)
from kinova_drake.controllers import (PSCommandSequenceController, PSCommandSequence, PartialStateCommand)

def holding_object():
    
    pscs = PSCommandSequence([])
    pscs.append(PartialStateCommand(
        name="come front",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([0.5*np.pi, 0.0*np.pi, 0.5*np.pi, 0.6, 0.0, 0.3]),
        gripper_value=0.0,
        duration=7.5))
    pscs.append(PartialStateCommand(
        name="pause",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([0.5*np.pi, 0.0*np.pi, 0.5*np.pi, 0.6, 0.0, 0.3]),
        gripper_value=0.0,
        duration=10.0))
    pscs.append(PartialStateCommand(
        name="firm hold",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([0.5*np.pi, 0.0*np.pi, 0.5*np.pi, 0.6, 0.0, 0.3]),
        gripper_value=1.0,
        duration=10))


    twist_Kp = np.diag([3.5, 3.5, 3.5, 5.0, 5.0, 5.0])*0.075
    twist_Kd = np.sqrt(twist_Kp)*0.25 + np.diag([0, 0, 0, 0, 0, 0])
    wrench_Kp = np.diag([75.0, 75, 75, 1500, 1500, 1500])
    wrench_Kd = np.sqrt(wrench_Kp)*0.125 + np.diag([0, 0, 0, 0, 0, 0])

    controller = PSCommandSequenceController(
        pscs,
        twist_Kp = twist_Kp,
        twist_Kd = twist_Kd,
        wrench_Kp = wrench_Kp,
        wrench_Kd = wrench_Kd )
    
    return pscs, controller