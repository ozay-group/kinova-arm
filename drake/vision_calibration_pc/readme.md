# Calibration
This package contains a series of calibration for the static camera (Intel RealSense D435i).

# Purpose
Find the pose of an object in the camera frame and convert it into the base frame.

# Procedure
1. Correct distortion transformation [Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
    - No need for this step if you have the camera listed in the Appendix.
2. Find the first, exact cloud point
    - Position the Kinova arm properly so that an object is within the frame.
    - Capture a color and depth image with the camera and generate cloud points (.ply).
    - Find the transformation of the cloud points to the base frame (the pose of the kinova arm + the camera pose).
3. Find the second, unknown-pose cloud point.
    - Make the same object is included in the frame.
    - Capture a color and depth image with the static camera and generate cloud points (.ply).
4. Using the ICP algorithm, calculate the pose of the static camera (see the ICP assignment).

# Appendix
## Spec of the static camera (D435)
Ideal range: .3 m to 3 m
Depth Field of View (FOV): 87° × 58° 
Depth output resolution: Up to 1280 × 720 pixels
Depth frame rate: Up to 90 fps
Depth technology: Stereoscopic 
Minimum depth distance (Min‑Z) at max resolution: ~28 cm
Depth Accuracy: <2% at 2 m
RGB sensor FOV (H × V): 69° × 42° 
RGB sensor resolution: 2 MP
RGB frame resolution: 1920 × 1080 pixels
RGB frame rate: 30 fps

## Spec of the wrist camera (Omnivision OV5640 + Intel® RealSense Depth Module D410)
Color sensor:
• resolution, frame rates (fps), and fields of view (FOV):
    o 1920 x 1080 (16:9) @ 30, 15 fps; FOV 47 ± 3° (diagonal)
    o 1280 x 720 (16:9) @ 30, 15 fps; FOV 60 ± 3° (diagonal)
    o 640 x 480 (4:3) @ 30, 15 fps; FOV 65 ± 3° (diagonal)
    o 320 x 240 (4:3)@ 30, 15 fps; FOV 65 ± 3° (diagonal)
• focusing range - 30 cm to ∞
Depth sensor:
• resolution, framerates (fps), and fields of view (FOV):
    o 480 x 270 (16:9) @ 30, 15, 6 fps; FOV 72 ± 3° (diagonal)
    o 424 x 240 (16:9) @ 30, 15, 6 fps; FOV 72 ± 3° (diagonal)
• minimum depth distance (min-Z) - 18 cm