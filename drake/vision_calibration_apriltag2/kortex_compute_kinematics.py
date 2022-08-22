#! /usr/bin/env python3

"""_summary_
    This script is adapted from the Kortex SDK examples.
Returns:
    _type_: _description_
"""

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

from kortex_api.autogen.messages import Base_pb2
from kortex_api.Exceptions.KServerException import KServerException

import numpy as np

#
#
# Example core functions
#

def compute_forward_kinematics(base):
    # Current arm's joint angles (in home position)
    try:
        print("Getting Angles for every joint...")
        input_joint_angles = base.GetMeasuredJointAngles()
    except KServerException as ex:
        print("Unable to get joint angles")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return None
    
    # Computing Foward Kinematics (Angle -> cartesian convert) from arm's current joint angles
    try:
        print("Computing Foward Kinematics using joint angles...")
        pose = base.ComputeForwardKinematics(input_joint_angles)
    except KServerException as ex:
        print("Unable to compute forward kinematics")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return None

    R = np.deg2rad(np.array([pose.theta_x, pose.theta_y, pose.theta_z]))
    t = np.array([pose.x, pose.y, pose.z])

    return R, t

## Main function

# Import the utilities helper module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import utilities

# Parse arguments
args = utilities.parseConnectionArguments()

# Create connection to the device and get the router
with utilities.DeviceConnection.createTcpConnection(args) as router:

    # Create required services
    base = BaseClient(router)

    # Get the result
    rpy_WorldEndeffector, t_WorldEndeffector = compute_forward_kinematics(base)

print("Pose calculated : ")
print("Coordinate (x, y, z)  : ({})".format(t_WorldEndeffector))
print("Theta (theta_x, theta_y, theta_z)  : ({})".format(rpy_WorldEndeffector))
