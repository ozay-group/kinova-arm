"""_summary_
CALCULATION
    Calculate the pose of the Intel RealSense camera relative to 
    the base of KINOVA arm (world frame).
Dependency:
    numpy
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

def RigidTransform(R, t):
    """_summary_
        Calculate the transformation matrix (4,4) from the rotation matrix (3,3)
        and translation vector (3,1).
    Args:
        R (numpy array (3,3)): rotation matrix.
        t (numpy array (3,1)): translation vector.

    Returns:
        X (numpy array (4,4)): transformation matrix.
    """
    X = np.concatenate((R, t), axis=1)
    X = np.concatenate((X, np.array([[0, 0, 0, 1]])), axis=0)
    return X

################################
##      Part One: KINOVA      ##
################################

## Transform from the end effector to the color sensor, provided by the User Guide Gen3.
X_EndeffectorColorsensor = np.array([[1, 0, 0, 0],[0, 1, 0, 0.05639],[0, 0, 1, -0.00305],[0, 0, 0, 1]])

## Load the transform of the end effector.
X_WorldEndeffector = np.load("forward_kinematics.npy")

## Hard code the rotation matrix and translation vector from kinova_result
R_ColorsensorApriltag = np.load("pose_R_kinova.npy")
t_ColorsensorApriltag = np.load("pose_t_kinova.npy")
X_ColorsensorApriltag = RigidTransform(R_ColorsensorApriltag, t_ColorsensorApriltag)

## Calculate the pose of the Apriltag in the world frame via KINOVA's vision.
X_WorldApriltag = X_WorldEndeffector @ X_EndeffectorColorsensor @ X_ColorsensorApriltag

###################################
##      Part Two: Realsense      ##
###################################

## Hard code the rotation matrix and translation vector from realsense_result
R_RealsenseApriltag = np.load("pose_R_realsense.npy")
t_RealsenseApriltag = np.load("pose_t_realsense.npy")
X_RealsenseApriltag = RigidTransform(R_RealsenseApriltag, t_RealsenseApriltag)

###############################################
##      Part Three: Calculate the pose.      ##
###############################################

## Calculate the pose of the Intel RealSense camera in the world frame.
X_WorldRealsense = X_WorldApriltag @ np.linalg.inv(X_RealsenseApriltag)

## Save the result.
np.save("X_WorldRealsense.npy", X_WorldRealsense)
print("The pose of the Intel RealSense camera: ", X_WorldRealsense)