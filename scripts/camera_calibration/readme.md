
## **Camera Calibration Objective**

Determine the **Camera pose in Base frame** (Camera Extrinsics), to compute the locations of object in base frame based on the camera frames.


## Calibration **Procedure**

1. Attach an AprilTag to Kinova Arm’s End Effector
2. Correct the distortion of camera → Not needed for Intel RealSense D435
    
    https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
    
    “For Intel RealSense D435 camera, its two infrared streams have no distortion. Therefore, this specific function doesn't not necessarily need to be used.”
    
**Below steps can be performed using: camer_calibration_single_pose.py or camera_calibration_multi_pose.py**
**Multi Pose script loops through these steps for a number of different poses and takes average of them**

3. Move the arm to the position where AprilTag can be seen by the camera
4. Detect the AprilTag pose within the captured frames from camera
5. Average out the **AprilTag pose in Camera frame**
6. Use forward kinematics to compute the **End Effector pose in Base frame**
7. Compute the **Camera pose in Base frame** using “AprilTag pose in Camera frame” and “End Effector pose in Base frame”, assuming AprilTag pose = End Effector pose

$$
\text{Assume: } X^A = X^E
$$
$$
X^A = X^C\ ^CX^A
$$
$$
\text{ Then, } X^C = X^E (^CX^A)^{-1}
$$

9. Save the determined extrinsics to .npy file for reuse
8. Validate the determined extrinsics are reasonable by placing an apriltag at the known location and running **camera_detect_apriltag.py** (multiple captures averaging) or **camera_stream_apriltag.py** (stream every captures) script


## **Intel RealSense D435i camera Specification**

- Ideal range: 0.3 m to 3 m
- Depth Field of View (FOV): 87° × 58°
- Depth output resolution: Up to 1280 × 720
- Depth frame rate: Up to 90 fps
- Depth technology: Stereoscopic
- Minimum depth distance (Min‑Z) at max resolution: ~28 cm
- Depth Accuracy: <2% at 2 m
- RGB sensor FOV (H × V): 69° × 42°
- RGB sensor resolution: 2 MP
- RGB frame resolution: 1920 × 1080
- RGB frame rate: 30 fps