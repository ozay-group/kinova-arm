"""
realsense_image.py
    Connects to the depth camera and captures one color image and one depth image.
    Both images are saved in png format.
"""

import cv2
import numpy as np
import pyrealsense2 as rs

def main():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    # Start streaming
    cfg = pipeline.start(config)

    # Get camera parameters [fx, fy, cx, cy] from RealSense camera
    profile = cfg.get_stream(rs.stream.depth)
    intrinsics = profile.as_video_stream_profile().get_intrinsics()
    cam_params = [intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy]

    # The number of frames to capture
    num = 0

    try:
        while True:
            num = num + 1

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert depth and color images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=1), cv2.COLORMAP_JET)

            # If depth and color resolutions are different, resize color image to match depth image for display
            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape
            
            if depth_colormap_dim != color_colormap_dim:
                color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)

            # Show images
            images = np.hstack((color_image, depth_colormap))
            cv2.namedWindow('RealSense Depth Camera', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense Depth Camera', images)
            cv2.waitKey(0)

            # Save the 10th frame
            if num == 10:
                cv2.imwrite("color_image.png", color_image)
                cv2.imwrite("depth_image.png", depth_image)
                break

    finally:
        pipeline.stop()


if __name__ == "__main__":
    exit(main())
