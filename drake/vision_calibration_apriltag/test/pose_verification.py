import numpy as np
from cv2 import Rodrigues

X_WorldCube = np.load('forward_kinematics.npy')

X_WorldRealsense = np.load('kinova-arm/drake/vision_calibration_apriltag/X_WorldRealsense.npy')

R_RealsenseCube = np.array([[ 0.59212563,  0.01627912, -0.80568122],
 [-0.28292787,  0.94034869, -0.18893427],
 [ 0.7545456,   0.3398225,   0.56141038]])
t_RealsenseCube = np.array([[-0.12280271],
 [ 0.02189387],
 [ 0.5466977 ]])
X_RealsenseCube = np.concatenate((R_RealsenseCube, t_RealsenseCube), axis=1)
X_RealsenseCube = np.concatenate((X_RealsenseCube, np.array([[0, 0, 0, 1]])), axis=0)

measured_X_WorldCube = X_WorldRealsense @ X_RealsenseCube

pose_err = 1e-3 # Hard coded for now

# https://stackoverflow.com/a/42723571
v, _ = Rodrigues(X_WorldCube.dot(measured_X_WorldCube.T))
transformation_error_from_identity = np.linalg.norm(v)
print('Transformation error from identity:', transformation_error_from_identity)