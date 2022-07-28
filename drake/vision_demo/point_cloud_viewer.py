"""
POINT CLOUD VIEWER
Summary:
        Visualizes a point cloud.
Args:
        point_cloud_path: path to the point cloud file
"""

import numpy as np
import open3d as o3d

def main(point_cloud_path):
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Point Cloud Viewer")

        pc = o3d.io.read_point_cloud(point_cloud_path)

        # flip the orientation, so it looks upright, not upside-down
        pc.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

        vis.add_geometry(pc)
        vis.run()

if '__name__' == '__main__':
        path = './point_cloud.ply' # Fill in your point cloud path here.
        exit(main(path))