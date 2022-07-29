import numpy as np
from cv2 import Rodrigues

X_WorldEndeffector = np.load('forward_kinematics.npy')
print(X_WorldEndeffector)

# According to the User Guide Gen3, if the Robotiq 2F-85 Gripper is installed,
# the transformation from interface module frame to tool frame is:
# x = 0, y = 0, z = 0.120m, theta_x = 0, theta_y = 0, theta_z = 0.
z = 0.120
# The size of the Apriltag cube is 3 inches, 0.0762m
a = 0.0762
# If the opposite face of the Apriltag matches the center of the gripper and
# it is kept parallel to the interface.
z_offset = z + a
X_EndeffectorApriltag = np.zeros((4,4))
X_EndeffectorApriltag[2,3] = z_offset

X_WorldApriltag = X_WorldEndeffector + X_EndeffectorApriltag

print(X_WorldApriltag)

X_WorldRealsense = np.load('/root/kinova-arm/drake/vision_calibration_apriltag/X_WorldRealsense.npy')

R_RealsenseApriltag = np.array([[ 0.99559036,  0.08261408,  0.04443809],
 [-0.07829508,  0.99272777, -0.09144102],
 [-0.05166924,  0.08755852,  0.99481847]])
t_RealsenseApriltag = np.array([[0.07921371],
 [-0.12779749],
 [ 0.37730513]])
X_RealsenseApriltag = np.concatenate((R_RealsenseApriltag, t_RealsenseApriltag), axis=1)
X_RealsenseApriltag = np.concatenate((X_RealsenseApriltag, np.array([[0, 0, 0, 1]])), axis=0)

measured_X_WorldApriltag = X_WorldRealsense @ X_RealsenseApriltag

pose_err = 3.619507164899972e-08 # Hard coded for now

# https://stackoverflow.com/a/42723571
#v, _ = Rodrigues(X_WorldCube.dot(measured_X_WorldCube.T))
#
m = X_WorldApriltag @ measured_X_WorldApriltag.T
print(m)
transformation_error_from_identity = np.linalg.norm(m)
print('Transformation error from identity:', transformation_error_from_identity)

dm = X_WorldApriltag - measured_X_WorldApriltag
print(dm)

m = X_WorldApriltag.T @ measured_X_WorldApriltag
print(m)
transformation_error_from_identity = np.linalg.norm(m)
print('Transformation error from identity:', transformation_error_from_identity)