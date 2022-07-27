"""_summary_
kinova_pointcloud.py
    Reads a color image and depth image and generate a point cloud. 
    The point cloud is saved in .ply format.
"""

import open3d as o3d
import numpy as np

def main():
    ## import color and depth image
    color_image = o3d.io.read_image("color_image.png")
    depth_image = o3d.io.read_image("depth_image.png")

    ## Generate an RGBD image (open3d)
    print("Generate an RGBD image (open3d)")
    rgbd_image = o3d.geometry.RGBDImage().create_from_color_and_depth(color_image, depth_image)


    ## Load wrist camera extrinsic properties
    print("Load wrist camera extrinsic properties")
    extrinsic = np.load('forward_kinematics.npy')

    ## Load camera intrinsic properties
    print("Load camera intrinsic properties")
    color_intrinsic = np.load('color_general_intrinsic_parameters.npy')
    depth_intrinsic = np.load('depth_general_intrinsic_parameters.npy')
    #TODO: which intrinsic properties are needed?
    intrinsic_param = color_intrinsic #TODO: temporary solution.
    
    ## Set the intrinsic properties
    print("Set the intrinsic properties")
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(width=int(intrinsic_param[0]), height=int(intrinsic_param[1]), 
                            fx=intrinsic_param[2], fy=intrinsic_param[3], 
                            cx=intrinsic_param[4], cy=intrinsic_param[5])

    ## Generate a point cloud model
    print("Generate a point cloud model")
    point_cloud = o3d.geometry.PointCloud().create_from_rgbd_image(rgbd_image, intrinsic, extrinsic)
    # Flip it, otherwise the pointcloud will be upside down.
    point_cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    return point_cloud

if __name__ == "__main__":
    pc = main()
    o3d.io.write_point_cloud("kinova_pointcloud.ply", pc)