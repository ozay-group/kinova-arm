"""_summary_
    data_collector.py
    Collects necessary data to run camera calibration.
"""

import os
import sys

print("Data collection starts...")

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
print("Created and moved to the new working directory: " + str(os.getcwd()))

## 1. Generate the point cloud model from the static camera (D435)
import realsense_export_ply_example as realsense_export_ply_example
print("Step 1: Generate the point cloud model from the static camera...")
realsense_export_ply_example.main()

## 2. Capture one color image and one depth image by kinova camera
import kinova_image as kinova_image
print("Step 2: Capture one color image and one depth image by kinova camera...")
kinova_image.main()
print("Step 2: Done.")

## 3. Find the intrinsic of the kinova camera
import kortex_intrinsic as kortex_intrinsic
print("Step 3: Find the intrinsic of the kinova camera...")
kortex_intrinsic.main()
print("Step 3: Done.")

## 4. Find the pose of the kinova camera
#TODO: Due to confliction in argument flags, the following script can only be run individually.
#TODO: try to run it in this script but without moving to the new directory. To test whether it fails because of the address.
#       If it is because the address of the subdirectory, should try collect data in the main directory and then move all
#       results to the subdirectory (Top part of the code moved to the last.)
"""import kortex_compute_kinematics as kortex_compute_kinematics
print("Step 4: Find the pose of the kinova camera...")
kortex_compute_kinematics.main()
print("Step 4: Done.")"""

print("Data collection complete.")

# Go back to the previous level
os.chdir(script_path)