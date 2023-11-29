                        
"""

sequence_calibration_toward_wall.py
Description:
    This scripts contains the command sequence and controller to calibrate the camera
    toward the atrium (facing the arm from the front side)
"""

""" Imports """

import numpy as np
from kinova_drake.kinova_station import (KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget)
from kinova_drake.controllers import (PSCommandSequenceController, PSCommandSequence, PartialStateCommand)

def calibration_toward_wall(initial_move, roll, pitch, yaw, width, depth, height):
    """ Command Sequence """
    pscs = PSCommandSequence([]) # create the command sequence
    if initial_move:
        pscs.append(PartialStateCommand(
            name="come front",
            target_type=EndEffectorTarget.kPose,
            target_value=np.array([0.5*np.pi, 0.0*np.pi, 0.5*np.pi, 0.9, 0.2, 0.3]),
            gripper_value=0.0,
            duration=10.0))
        pscs.append(PartialStateCommand(
            name="pause",
            target_type=EndEffectorTarget.kPose,
            target_value=np.array([0.5*np.pi, 0.0*np.pi, 0.5*np.pi, 0.9, 0.2, 0.3]),
            gripper_value=0.0,
            duration=5.0))
    
    pscs.append(PartialStateCommand(
        name="varient align",
        target_type=EndEffectorTarget.kPose,
        target_value=np.array([
                            0.5*np.pi + 0.1*np.pi*roll,
                            0.0*np.pi + 0.1*np.pi*pitch,
                            0.5*np.pi + 0.1*np.pi*yaw,
                            0.9 + 0.1*width,
                            0.2 + 0.1*depth,
                            0.3 + 0.1*height
                            ]),
        gripper_value=0.0,
        duration=10.0))
    

    """ Controller """
    twist_Kp = np.diag([5.0, 5.0, 5.0, 4.0, 4.0, 4.0])*0.1
    twist_Kd = np.sqrt(twist_Kp)*0.25 + np.diag([0, 0, 0, 0, 0, 0.01])
    wrench_Kp = np.diag([75.0, 75, 75, 1500, 1500, 1500])
    wrench_Kd = np.sqrt(wrench_Kp)*0.125 + np.diag([0, 0, 0, 0, 0, 0])

    controller = PSCommandSequenceController(
        pscs,
        twist_Kp = twist_Kp,
        twist_Kd = twist_Kd,
        wrench_Kp = wrench_Kp,
        wrench_Kd = wrench_Kd )
    
    return pscs, controller
