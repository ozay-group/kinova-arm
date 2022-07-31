"""
opencv_pose_estimator_example.py
Description:
    This script should open up a live feed and then perform pose estimation on the objects in the frame
    for the two tags that are currently pasted on the block:
        #3
        #5
    from the tagStandard41h12 directory of apriltags3.
"""
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import numpy as np
import cv2
from dt_apriltags import Detector

# The video stream from the depth Camera on the Kinova Gen3 is sent through rtsp.
# Here we capture the stream by opencv. Note: the color stream and depth stream are separate.
color_cap = cv2.VideoCapture("rtsp://192.168.1.10/color", cv2.CAP_FFMPEG)
depth_cap = cv2.VideoCapture("rtsp://192.168.1.10/depth")

# Configure april tag detector
# detector = apriltag.Detector("tagStandard41h12")
# AprilTag detector options
# options = apriltag.DetectorOptions(families='tag41h12',
#                                 border=1,
#                                 nthreads=4,
#                                 quad_decimate=1.0,
#                                 quad_blur=0.0,
#                                 refine_edges=True,
#                                 refine_decode=False,
#                                 refine_pose=True,
#                                 debug=False,
#                                 quad_contours=True)
# detector = apriltag.Detector(options)
at_detector = Detector(families='tagStandard41h12',
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)

# camera parameters [fx, fy, cx, cy]
intrinsic_array = np.load("depth_general_intrinsic_parameters.npy")
cam_params0 = intrinsic_array[2:].tolist()
print(cam_params0)
tag_size0 = 0.040084375

# counter
n = 0

try:
    while True:
        n = n + 1

        # These frames are expressed in the camera frame. 
        # For locating, you need to transform them to the end effector frame first.
        _, color_frame = color_cap.read()
        _, depth_frame = depth_cap.read()

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame)
        color_image = np.asanyarray(color_frame)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

        # Print whether or not detector detects anything.
        gray_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
        result = at_detector.detect(
                gray_image,
                estimate_tag_pose=True,
                camera_params=cam_params0,
                tag_size= tag_size0
                )
        print(str(result))
        
        # Log the rotation and translation matrix at the 100th frame.
        if n == 100:
            np.save("pose_R_kinova.npy", result[0].pose_R)
            np.save("pose_t_kinova.npy", result[0].pose_t)
            break


finally:

    # Stop streaming
    color_cap.release()
    depth_cap.release()
    cv2.destroyAllWindows()