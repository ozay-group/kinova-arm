## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

from multiprocessing import Condition
import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
print(device)
device_product_line = str(device.get_info(rs.camera_info.product_line))

# Whether the camera supports RGB videos
found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

# set the configuration for streaming
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


## --- Start streaming --- ##
pipeline.start(config)
# Initiate a stop sign
num_frame = 0
tot_frame = 500 # A maximum of 500 frames will be shot
cond = True

# TODO: Launch video saving
# result = cv2.VideoWriter('ball bouncing.avi', cv2.VideoWriter_fourcc(*'MJPG'), 10, (640, 480))

try:
    while cond:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        rgb_image = np.asanyarray(color_frame.get_data())

        # TODO: color thresholding
        '''Thresholding Information RGB --> HSV
            Bouncy Blue: 0 30 100       --> 220 100 39
            Bouncy Yellow: 160 140 40   --> 50 75 63
            Bouncy Orange: 210 10 0     --> 3 100 82
            Bouncy Red: 190 1 5         --> 358 100 75
            Bouncy Green: 0 140 50      --> 141 100 55
            Ping Pong: 220 100 0        --> 27 100 86
        '''
        hsv_frame = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        lower_bound = np.array([40, 50, 50])
        upper_bound = np.array([80, 255, 255]) # Green
        background_elimination_mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
        filtered_rgb_image = cv2.bitwise_and(rgb_image, rgb_image, mask= background_elimination_mask)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = filtered_rgb_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(filtered_rgb_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((filtered_rgb_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

        # TODO: output video
        # result.write(filtered_rgb_image)

        # Stop sign
        num_frame = num_frame + 1
        if num_frame > tot_frame: cond = False

finally:

    # Stop streaming
    pipeline.stop()

    # save the output video
    #result.release()