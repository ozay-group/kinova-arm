import cv2
import numpy as np

# The video stream from the depth Camera on the Kinova Gen3 is sent through rtsp.
# Here we capture the stream by opencv. Note: the color stream and depth stream are separate.
color_cap = cv2.VideoCapture("rtsp://192.168.1.10/color", cv2.CAP_FFMPEG)
depth_cap = cv2.VideoCapture("rtsp://192.168.1.10/depth")

# Get the camera intrinsics from "01-vision_intrinsics.py" by KINOVA (R) KORTEX (TM). 
# Make sure you have the repository "kortex"
import OzayGroupExploration.drake.vision_calibration.test_workspace.kortex_vision_intrinsics as kv_in
color_intrinsics, depth_intrinsics = kv_in.main()
#print(color_intrinsics)
#print(depth_intrinsics)

"""TODO: # Get the forward transformation of the kinova arm.
import kortex_compute_kinematics as kck
joint_angle, forward_kinematics = kck.main()
print(joint_angle)
print(forward_kinematics)"""

from datetime import datetime
import os
directory_path = "/root/OzayGroupExploration/vision_module/image_from_kinova"

num_frame = 0

try:
    while(1):
        # These frames are expressed in the camera frame. 
        # For locating, you need to transform them to the end effector frame first.
        _, color_frame = color_cap.read()
        _, depth_frame = depth_cap.read()

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame)
        depth_image = np.asanyarray(depth_frame)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=1), cv2.COLORMAP_JET)

        # If depth and color resolutions are different, resize color image to match depth image for display
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape
        
        if depth_colormap_dim != color_colormap_dim:
            color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)

        # Show images
        images = np.hstack((color_image, depth_colormap))
        cv2.namedWindow('Kinova Depth Camera', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Kinova Depth Camera', images)
        cv2.waitKey(1)

        if num_frame == 10:
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            color_file = current_time + "_color.png"
            color_path = os.path.join(directory_path,color_file)
            cv2.imwrite(color_path, color_image)
            depth_file = current_time + "_depth.png"
            depth_path = os.path.join(directory_path,depth_file)
            cv2.imwrite(depth_path, depth_image)
            break
        num_frame = num_frame + 1

finally:
    color_cap.release()
    depth_cap.release()
    print("Done")

"""
Knowledge base:
https://stackoverflow.com/questions/59590200/generate-point-cloud-from-depth-image
Inspiration: https://www.mathworks.com/help/supportpkg/robotmanipulator/ug/generate-colorized-point-cloud-gen3.html
https://stackoverflow.com/questions/40875846/capturing-rtsp-camera-using-opencv-python
https://github.com/Kinovarobotics/kortex/issues/112
"""