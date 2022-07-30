"""POSE_VERIFICATION
    Summary:
        This script is used to verify the pose of the Intel Realsense camera.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

## Load the transform of the end effector.
X_WorldEndeffector = np.load('forward_kinematics.npy')
#print(X_WorldEndeffector)

## According to the User Guide Gen3, if the Robotiq 2F-85 Gripper is installed,
## the transformation from interface module frame to tool frame is:
## x = 0, y = 0, z = 0.120m, theta_x = 0, theta_y = 0, theta_z = 0.
z = 0.120

## The size of the Apriltag cube is 3 inches, 0.0762m
a = 0.0762

## If the opposite face of the Apriltag matches the center of the gripper and
## it is kept parallel to the interface.
X_EndeffectorApriltag = np.identity(4)
X_EndeffectorApriltag[2,3] = 0#z + a

## Find the pose of the Apriltag in the world frame.
X_WorldApriltag = X_WorldEndeffector @ X_EndeffectorApriltag

## Split the transformation (4,4) into rotation (3,3) and translation (3,1)
t_WorldApriltag = X_WorldApriltag[:3,3]
R_WorldApriltag = X_WorldApriltag[:3,:3]
r_WorldApriltag = R.from_matrix(R_WorldApriltag).as_euler('xyz') # numpy array (1,3)

## Load the measured pose of the Intel Realsense camera.
X_WorldRealsense = np.load('/root/kinova-arm/drake/vision_calibration_apriltag/X_WorldRealsense.npy')

## Hard code the rotation matrix and translation vector from realsense_result
R_RealsenseApriltag = np.load("pose_R_realsense.npy")
t_RealsenseApriltag = np.load("pose_t_realsense.npy")
"""R_RealsenseApriltag = np.array([[ 0.59212563,  0.01627912, -0.80568122],
 [-0.28292787,  0.94034869, -0.18893427],
 [ 0.7545456,   0.3398225,   0.56141038]])
t_RealsenseApriltag = np.array([[-0.12280271],
 [ 0.02189387],
 [ 0.5466977 ]])"""
X_RealsenseApriltag = np.concatenate((R_RealsenseApriltag, t_RealsenseApriltag), axis=1)
X_RealsenseApriltag = np.concatenate((X_RealsenseApriltag, np.array([[0, 0, 0, 1]])), axis=0)

## Calculate the pose of the Apriltag based on the measured pose of the Intel Realsense camera.
measured_X_WorldApriltag = X_WorldRealsense @ X_RealsenseApriltag

## Split the transformation (4,4) into rotation (3,3) and translation (3,1)
measured_t_WorldApriltag = measured_X_WorldApriltag[:3,3]
measured_R_WorldApriltag = measured_X_WorldApriltag[:3,:3]
measured_r_WorldApriltag = R.from_matrix(measured_R_WorldApriltag).as_euler('xyz')

pose_err = 3.619507164899972e-08 # Hard coded from "realsense_result.txt"

## Calculate the error between the measured pose and the true pose.
t_err = (t_WorldApriltag - measured_t_WorldApriltag)
print("Translation error: ", t_err)
print("Translation error Inf norm: ", np.linalg.norm(t_err, np.inf))
print("Translation error 1-norm: ", np.linalg.norm(t_err,1))
print("Translation error 2-norm: ", np.linalg.norm(t_err))

r_tolerance = np.ones(3) * np.pi / 18
r_err = np.abs(r_WorldApriltag - measured_r_WorldApriltag)
print("Rotation error: ", r_err)
r_bool = r_err / r_tolerance
print("Rotation error is __ times of pi/18: ", r_bool)