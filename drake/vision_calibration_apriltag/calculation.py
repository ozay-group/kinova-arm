import numpy as np

X_EndeffectorColorsensor = np.array([[1, 0, 0, 0],[0, 1, 0, 0.05639],[0, 0, 1, -0.00305],[0, 0, 0, 1]])

X_WorldEndeffector = np.load("forward_kinematics.npy")

R_ColorsensorApriltag = np.array([[ 0.99816156,  0.05757816,  0.01892744],
 [-0.03989837,  0.85929397, -0.50992351],
 [-0.04562469,  0.50823088,  0.86001149]]) # Copy and paste from txt file. TODO: need to automate this process
t_ColorsensorApriltag = np.array([[-0.00310981],
 [ 0.0407879 ],
 [ 0.30438019]])
X_ColorsensorApriltag = np.concatenate((R_ColorsensorApriltag, t_ColorsensorApriltag), axis=1)
X_ColorsensorApriltag = np.concatenate((X_ColorsensorApriltag, np.array([[0, 0, 0, 1]])), axis=0)

X_WorldApriltag = X_WorldEndeffector @ X_EndeffectorColorsensor @ X_ColorsensorApriltag


R_RealsenseApriltag = np.array([[ 0.59212563,  0.01627912, -0.80568122],
 [-0.28292787,  0.94034869, -0.18893427],
 [ 0.7545456,   0.3398225,   0.56141038]])
t_RealsenseApriltag = np.array([[-0.12280271],
 [ 0.02189387],
 [ 0.5466977 ]])
X_RealsenseApriltag = np.concatenate((R_RealsenseApriltag, t_RealsenseApriltag), axis=1)
X_RealsenseApriltag = np.concatenate((X_RealsenseApriltag, np.array([[0, 0, 0, 1]])), axis=0)

X_WorldRealsense = X_WorldApriltag @ np.linalg.inv(X_RealsenseApriltag)

np.save("X_WorldRealsense.npy", X_WorldRealsense)
print(X_WorldRealsense)