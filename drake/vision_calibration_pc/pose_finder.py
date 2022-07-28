"""
_summary_:
    This script is used to find the pose of the RealSense Camera.
"""

import os
import sys
import open3d as o3d

print("Data collection starts...")

## Find the path of this level
script_path = os.path.dirname(os.path.realpath(sys.argv[0]))
os.chdir(script_path)

## Find all directories in this level
all_folders = os.listdir(script_path)
all_dataset = [f for f in all_folders if f.startswith('dataset_')]

all_dataset.sort()

## Move to the new directory
#TODO: May be interested in running calibration over all datasets and finally calculate one value.
#       Still, it is the priority that the calibration should be done on a single dataset.
#       Here, the number of directory is hard-coded.
order = len(all_dataset)
dataset_name = "dataset_" + "0" #str(order)
dataset_path = os.path.join(script_path,dataset_name)
os.chdir(dataset_path)
print("Created and moved to the new working directory: " + str(os.getcwd()))

## 1. Generate the point cloud model from the kinova images
import vision_calibration.kinova_pointcloud as kinova_pointcloud
print("Step 1: Generate the point cloud model from the kinova images...")
pc_kinova = kinova_pointcloud.main()
print("Step 1: Point cloud model 'pc_kinova' is successfully generated.")

## 2. Load the point cloud model from the static camera
print("Step 2: Load the point cloud model from the static camera...")
pc_static = o3d.io.read_point_cloud("pc_static.ply")
print("Step 2: Point cloud model 'pc_static' is successfully loaded.")

## 3. ICP algorithm.
print("Step 3: ICP algorithm...")
print("...")
#TODO: Get the data and try different icp algorithm to determine which one is the best.
#      Candidates:
#      Colored ICP Registration: http://www.open3d.org/docs/release/tutorial/t_pipelines/t_icp_registration.html
#      (Fast) Global Registration: http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html
#      Multiway Registration: http://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html
print("Step 3: Done.")

