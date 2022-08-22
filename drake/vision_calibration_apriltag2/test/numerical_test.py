import numpy as np
from scipy.spatial.transform import Rotation as R

X1 = np.load("test_data3/X_WorldRealsense.npy")
X2 = np.load("test_data4/X_WorldRealsense.npy")

R1 = X1[0:3, 0:3]
r1= R.from_matrix(R1).as_euler('xyz') # numpy array (1,3)
t1 = X1[0:3, 3]

R2 = X2[0:3, 0:3]
r2 = R.from_matrix(R2).as_euler('xyz') # numpy array (1,3)
t2 = X2[0:3, 3]

################################################
##      Part Three: Calculate the error.      ##
################################################

## Translation error.
t_err = (t1 - t2)
print("Translation error: ", t_err)
print("Translation error Inf norm: ", np.linalg.norm(t_err, np.inf))
print("Translation error 1-norm: ", np.linalg.norm(t_err,1))
print("Translation error 2-norm: ", np.linalg.norm(t_err))

## Angular error.
r_tolerance = np.ones(3) * np.pi / 18
r_err = np.abs(r1 - r2)
print("Rotation error: ", r_err)
r_bool = r_err / r_tolerance
print("Rotation error is __ times of pi/18: ", r_bool)