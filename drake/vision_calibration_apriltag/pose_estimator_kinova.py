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

"""# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)"""

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
intrinsic_array = np.load("color_general_intrinsic_parameters.npy")
cam_params0 = [1297.6728515625, 1298.63134765625, 620.9140014648438, 238.2803192138672]#intrinsic_array[2:]
tag_size0 = 0.040084375

# Start streaming
# pipeline.start(config)

# counter
n = 0

try:
    while True:
        n = n + 1

        """# Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue"""

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

        #  Print whether or not detector detects anything.
        gray_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
        result = str(at_detector.detect(
                gray_image,
                estimate_tag_pose=True,
                camera_params=cam_params0,
                tag_size= tag_size0
                ))
        print(result)

        
        if n == 100:
            with open('kinova_result.txt','w') as f:
                f.write(result)
                f.close()
            break


        #print('intrinsics')
        #print(pipeline_profile)
        #print(depth_frame.profile.as_video_stream_profile().intrinsics)

finally:

    # Stop streaming
    #pipeline.stop()
    color_cap.release()
    depth_cap.release()
    cv2.destroyAllWindows()