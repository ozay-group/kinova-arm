import numpy as np
from cv2 import Rodrigues

X_WorldCube = np.load('forward_kinematics.npy')

X_WorldRealsense = np.load('kinova-arm/drake/vision_calibration_apriltag/X_WorldRealsense.npy')

X_RealsenseCube = np.array([[0.0, 0.0, 0.0, 0.0],]) # Hard coded for now

measured_X_WorldCube = X_WorldRealsense @ X_RealsenseCube

pose_err = 1e-3 # Hard coded for now

# https://stackoverflow.com/a/42723571
v, _ = Rodrigues(X_WorldCube.dot(measured_X_WorldCube.T))
transformation_error_from_identity = np.linalg.norm(v)
print('Transformation error from identity:', transformation_error_from_identity)