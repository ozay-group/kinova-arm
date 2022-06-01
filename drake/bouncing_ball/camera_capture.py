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
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# Initiate a stop sign
num_frame = 0
tot_frame = 30000 # A maximum of 500 frames will be shot
cond = True

# TODO: Launch video saving
# result = cv2.VideoWriter('ball bouncing.avi', cv2.VideoWriter_fourcc(*'MJPG'), 10, (640, 480))

try:
    while cond:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
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
        upper_bound = np.array([90, 255, 255]) # Green
        background_elimination_mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
        # Display filtered image
        filtered_rgb_image = cv2.bitwise_and(rgb_image, rgb_image, mask= background_elimination_mask)
        # HoughCircle to find the ball
        filtered_gray_image = cv2.cvtColor(filtered_rgb_image, cv2.COLOR_BGR2GRAY)
        
        filtered_blurred_gray_image = cv2.medianBlur(filtered_gray_image,15)
        #cv2.imshow("Temp",filtered_blurred_gray_image)

        rows = filtered_blurred_gray_image.shape[0]
        circles = cv2.HoughCircles(filtered_blurred_gray_image, cv2.HOUGH_GRADIENT, 0.5, rows/8, param1=120, param2=15, minRadius=0, maxRadius=-1)

        if circles is not None:
            circles= np.uint16(np.around(circles))
            major_circle = circles[0][0]
            center = (major_circle[0],major_circle[1])
            cv2.circle(filtered_rgb_image, center, 1, (0,255,255),3)
            try:
                if (num_frame%1 ==0):
                    print(center,depth_image[center[0],center[1]])
            except:
                continue

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_image = cv2.bitwise_and(depth_image, depth_image, mask= background_elimination_mask)
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

        ''' TODO: try to understand how the depth frame stores data
        if (num_frame == 100):
            with np.printoptions(threshold=np.inf):
                #print(depth_image[0:100,0:20])
                continue'''

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