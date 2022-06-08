import cv2
import numpy as np

# The video stream from the depth Camera on the Kinova Gen3 is sent through rtsp.
# Here we capture the stream by opencv. Note: the color stream and depth stream are separate.
color_cap = cv2.VideoCapture("rtsp://192.168.1.10/color", cv2.CAP_FFMPEG)
depth_cap = cv2.VideoCapture("rtsp://192.168.1.10/depth")

try:
    while(1):
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
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))

        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('Kinova Depth Camera', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Kinova Depth Camera', images)
        cv2.waitKey(1)

finally:
    color_cap.release()
    depth_cap.release()
    print("Done")

"""
https://stackoverflow.com/questions/59590200/generate-point-cloud-from-depth-image
Inspiration: https://www.mathworks.com/help/supportpkg/robotmanipulator/ug/generate-colorized-point-cloud-gen3.html
https://stackoverflow.com/questions/40875846/capturing-rtsp-camera-using-opencv-python
https://github.com/Kinovarobotics/kortex/issues/112
https://www.mathworks.com/help/supportpkg/robotmanipulator/ug/generate-colorized-point-cloud-gen3.html
"""