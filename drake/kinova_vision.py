import cv2
import numpy as np


color_cap = cv2.VideoCapture("rtsp://192.168.1.10/color", cv2.CAP_FFMPEG)
depth_cap = cv2.VideoCapture("rtsp://192.168.1.10/depth")

try:
    while(1):
        _, color_frame = color_cap.read()
        _, depth_frame = depth_cap.read()

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame)
        depth_image = np.asanyarray(depth_frame)
        print(np.sum(depth_image))

        # images = np.hstack(color_image)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=1), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        print(depth_colormap_dim)
        color_colormap_dim = color_image.shape
        print(color_colormap_dim)

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[0], depth_colormap_dim[1]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', images)
        cv2.waitKey(1)

finally:
    color_cap.release()
    depth_cap.release()
    print("Done")


"""color_cap = cv2.VideoCapture("rtsp://192.168.1.10/color", cv2.CAP_FFMPEG)

try:
    while(1):
        _, color_frame = color_cap.read()

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame)
        print(color_image)

        # images = np.hstack(color_image)

        # Show images
        cv2.namedWindow('1', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('1', color_frame)
        cv2.waitKey(1)

finally:
    color_cap.release()
    print("Done")"""

"""
depth_cap = cv2.VideoCapture("rtsp://192.168.1.10/depth")

try:
    while(1):
        _, depth_frame = depth_cap.read()

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame)
        print(np.sum(depth_image))

        # images = np.hstack(depth_image)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=1), cv2.COLORMAP_JET)

        # Show images
        cv2.namedWindow('1', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('1', depth_colormap)
        cv2.waitKey(1)

finally:
    depth_cap.release()
    print("Done")
"""

"""
https://stackoverflow.com/questions/59590200/generate-point-cloud-from-depth-image
Inspiration: https://www.mathworks.com/help/supportpkg/robotmanipulator/ug/generate-colorized-point-cloud-gen3.html
https://stackoverflow.com/questions/40875846/capturing-rtsp-camera-using-opencv-python
https://github.com/Kinovarobotics/kortex/issues/112
https://www.mathworks.com/help/supportpkg/robotmanipulator/ug/generate-colorized-point-cloud-gen3.html
"""