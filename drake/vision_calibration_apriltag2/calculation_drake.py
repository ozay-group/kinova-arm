"""_summary_
CALCULATION
    Calculate the pose of the Intel RealSense camera relative to 
    the base of KINOVA arm (world frame).
Dependency:
    numpy
    pydrake.math
Input:
    (1) "forward_kinematics.npy": The pose of the end effector relative
        to the base of the KINOVA arm (world frame).
    (2) R_ColorsensorApriltag (numpy array (3,3)): The rotation matrix from
        the color sense on KINOVA arm to the Intel RealSense camera.
        Must be hard coded for now.
    (3) t_ColorsensorApriltag (numpy array (3,1)): The translation matrix from
        the color sense on KINOVA arm to the Intel RealSense camera.
        Must be hard coded for now.
    (4) R_RealsenseApriltag (numpy array (3,3)): The rotation matrix from
        the Intel RealSense camera to the Apriltag.
        Must be hard coded for now.
    (5) t_RealsenseApriltag (numpy array (3,1)): The translation matrix from
        the Intel RealSense camera to the Apriltag.
        Must be hard coded for now.
Output:
    "X_WorldRealsense.npy": The pose of the Intel RealSense camera relative
        to the base of the KINOVA arm (world frame).
"""

import numpy as np
from pydrake.math import ( 
    RigidTransform , RotationMatrix )

#################################
##      Part Zero: Import      ##
#################################
from kortex_compute_kinematics import R_WorldEndeffector, t_WorldEndeffector

from pose_estimator_realsense import R_RealsenseApriltag, t_RealsenseApriltag


################################
##      Part One: KINOVA      ##
################################

## Transform from the end effector to the color sensor, provided by the User Guide Gen3.
t_EndeffectorApriltag = np.array([0.0, 0.0, 0.01]) # meters
R_EndeffectorApriltag = RotationMatrix.MakeXRotation(np.pi)
X_EndeffectorApriltag = RigidTransform(R_EndeffectorApriltag, t_EndeffectorApriltag)

## Load the transform of the end effector.
X_WorldEndeffector = RigidTransform(RotationMatrix(R_WorldEndeffector), t_WorldEndeffector)

## Calculate the pose of the Apriltag in the world frame via KINOVA's vision.
X_WorldApriltag = X_WorldEndeffector.multiply(X_EndeffectorApriltag)

###################################
##      Part Two: Realsense      ##
###################################

## Hard code the rotation matrix and translation vector from realsense_result
X_RealsenseApriltag = RigidTransform(RotationMatrix(R_RealsenseApriltag), t_RealsenseApriltag)

##########################################################
##      Part Four: Calculate the pose using Drake.      ##
##########################################################

## Calculate the pose of the Intel RealSense camera in the world frame.
print("debug:")
print(R_RealsenseApriltag)
print(R_RealsenseApriltag.shape)
print(t_RealsenseApriltag)
print(t_RealsenseApriltag.shape)
print(t_RealsenseApriltag.reshape((3,)))
print(t_RealsenseApriltag.reshape((3,1)))
X_WorldRealsense = X_WorldApriltag.multiply( X_RealsenseApriltag.inverse() )

## Save the result.
#np.save("X_WorldRealsense_usingDrake.npy", X_KinovacamRealsense)
print("The pose of the Intel RealSense camera with respect to world is: ", X_WorldRealsense)
