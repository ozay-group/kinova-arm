import numpy as np
from cv2 import Rodrigues

cube_pose = np.load('forward_kinematics.npy')

measured_pose = np.array([[0.0, 0.0, 0.0, 0.0],]) # Hard coded for now

pose_err = 1e-3 # Hard coded for now

# https://stackoverflow.com/a/42723571
r, _ = Rodrigues(cube_pose.dot(measured_pose.T))
transformation_error_from_identity = np.linalg.norm(r)