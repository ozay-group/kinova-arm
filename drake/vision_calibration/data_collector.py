"""_summary_
    data_collector.py
    Collects necessary data to run camera calibration.
"""

import os
import sys

## Find the path of this level
script_path = os.path.dirname(os.path.realpath(sys.argv[0]))
os.chdir(script_path)

## Find all directories in this level
all_folders = os.listdir(script_path)
all_dataset = [f for f in all_folders if f.startswith('dataset_')]

all_dataset.sort()

## Make dataset directories
order = len(all_dataset)
new_directory_name = "dataset_" + str(order)
new_directory_path = os.path.join(script_path,new_directory_name)
os.mkdir(new_directory_path)

## Move to the new directory
os.chdir(new_directory_path)

## 1. Generate the point cloud model from the static camera (D435)
import vision_calibration.realsense_export_ply_example as realsense_export_ply_example
print("Step 1: Generate the point cloud model from the static camera...")
realsense_export_ply_example.main()

## 2. Capture one color image and one depth image by kinova camera
import vision_calibration.kinova_image as kinova_image
print("Step 2: Capture one color image and one depth image by kinova camera...")
kinova_image.main()

## 3. Find the intrinsic of the kinova camera
import vision_calibration.kortex_intrinsic as kortex_intrinsic
print("Step 3: Find the intrinsic of the kinova camera...")
kortex_intrinsic.main()

## 4. Find the pose of the kinova camera
import vision_calibration.kortex_compute_kinematics as kortex_compute_kinematics
print("Step 4: Find the pose of the kinova camera...")
kortex_compute_kinematics.main()

## 5. Generate the point cloud model from the kinova images
import vision_calibration.kinova_pointcloud as kinova_pointcloud
print("Step 5: Generate the point cloud model from the kinova images...")
kinova_pointcloud.main()

print("Data collection complete.")
os.chdir(script_path)