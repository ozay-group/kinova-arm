import cv2
import numpy as np

color = cv2.imread("color_image.png")
depth = cv2.imread("depth_image.png")

c_dim = color.shape
print(c_dim)
d_dim = depth.shape
print(d_dim)