# Calibration
This package contains a series of calibration for the Intel RealSense D435i camera.

# Purpose
Find the pose of an object in the camera frame and convert it into the base frame.

# Procedure
1. Correct distortion transformation [Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
2. Find transformation from pixel to 3D point
3. Find transformation from 3D point to base frame

# Spec of D435i
Ideal range:
.3 m to 3 m

Depth Field of View (FOV):
87° × 58° 

Depth output resolution:
Up to 1280 × 720 

Depth frame rate:
Up to 90 fps

Depth technology:
Stereoscopic 

Minimum depth distance
(Min‑Z) at max resolution:
~28 cm 

Depth Accuracy:
<2% at 2 m

RGB sensor FOV (H × V):
69° × 42° 

RGB sensor resolution:
2 MP

RGB frame resolution:
1920 × 1080 

RGB frame rate:
30 fps