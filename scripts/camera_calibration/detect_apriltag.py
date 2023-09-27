"""
detect_apriltag.py
Description:
    Simple apriltag detection test script using Intel RealSense
"""

## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

""" Imports """
# general python modules
import numpy as np
import cv2

# intel realsense
import pyrealsense2 as rs

# apriltag detector
from dt_apriltags import Detector

# drake functions
from pydrake.all import *
from pydrake.all import (
    DepthImageToPointCloud, PointCloud, Fields, BaseField, ResetIntegratorFromFlags,
    RollPitchYaw, RotationMatrix, RigidTransform, ConstantVectorSource,LogVectorOutput,
    Meshcat, StartMeshcat, MeshcatVisualizer, MeshcatPointCloudVisualizer,
    DiagramBuilder, Parser, Simulator, AddMultibodyPlantSceneGraph,
    Rgba, CameraInfo, PixelType
)

""" Apriltag Detector """
tag_size = 0.014
at_detector = Detector(families='tagStandard41h12', # Configure AprilTag detector
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
n_sample = 500 # number of images captured by camera

""" Start RealSense Pipeline """
pipeline = rs.pipeline() # Declare RealSense pipeline, encapsulating the actual device and sensors
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline) # Get device product line for setting a supporting resolution
pipeline_profile = config.resolve(pipeline_wrapper) 
device = pipeline_profile.get_device()
    
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Enable depth stream
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) # Enable color stream

cfg = pipeline.start(config) # Start streaming the pipeline and get the configuration

""" Camera Intrinsics """
""" Get camera parameters [fx, fy, cx, cy] from RealSense camera
cam_params = [ 386.738, 386.738, 321.281, 238.221 ]
https://github.com/IntelRealSense/librealsense/issues/869
https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.intrinsics.html
"""
profile = cfg.get_stream(rs.stream.depth)                       # Fetch stream profile for depth stream
intrinsics = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
cam_params = [intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy]


""" Pipeline Streaming """
R_cam_atag = np.zeros(3)
p_cam_atag = np.zeros((3,1))

for i in range(n_sample):
    frames = pipeline.wait_for_frames() # Wait for a coherent pair of frames: depth and color
    
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue
    
    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

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

    # Perform Apriltag detection
    gray_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
    atag = at_detector.detect(
            gray_image,
            estimate_tag_pose=True,
            camera_params=cam_params,
            tag_size= tag_size
            )
    
    """ Collect RealSense Data """
    if not atag:
        i = i - 1
    else:
        R_cam_atag = R_cam_atag + atag[0].pose_R
        p_cam_atag = p_cam_atag + atag[0].pose_t
        # print(atag)

print('\n RealSense Streamig & Apriltag Data Collecting... \n')

print('Camera Intrinsics:')
print(pipeline_profile)
print(depth_frame.profile.as_video_stream_profile().intrinsics)

pipeline.stop() # Stop streaming
R_cam_atag = R_cam_atag/n_sample
p_cam_atag = p_cam_atag/n_sample
print()
print(f"Apriltag Pose in Camera Frame: Rotation: \n {R_cam_atag} \n")
print(f"Apriltag Pose in Camera Frame: Translation: \n {p_cam_atag}")
print()

# Translate the Apriltag pose to the Base frame
R_cam_atag = RotationMatrix(R_cam_atag)
p_cam_atag = p_cam_atag
X_cam_atag = RigidTransform(R_cam_atag, p_cam_atag)

X_base_cam = RigidTransform(R=RotationMatrix([
                            [0.1366348257770575, 0.48609800872971815, 0.8619691386213646],
                            [0.9847153533655734, -0.15873137057852568, -0.06571224870574997],
                            [0.10542012819470732, 0.8582061742657858, -0.5002611021042845]]
                                             ),
                            p=[-0.038592930963458685, 0.7823496735507911, 0.7661446934495219]
                            )

X_base_atag = X_base_cam.multiply(X_cam_atag)
print(f"\n Apriltag Pose in Base Frame: \n {X_base_atag} \n")
