"""_summary_
kinova_pointcloud.py
    Reads a color image and depth image and generate a point cloud. 
    The point cloud is saved in .ply format.
"""

import open3d as o3d
import json

def main():
    ## import color and depth image
    color_image = o3d.io.read_image("color_image.png")
    depth_image = o3d.io.read_image("depth_image.png")

    ## Generate an RGBD image (open3d)
    rgbd_image = o3d.geometry.RGBDImage()
    rgbd_image.create_from_color_and_depth(color_image, depth_image)


    ## Load camera extrinsic properties
    with open('kinematics.json', 'r') as pose_file:
        pose_data = json.load(pose_file)
    extrinsic = pose_data["Forward_kinematics"]

    ## Load camera intrinsic properties
    with open('intrinsics.json','r') as camera_intrinsics:
        intrinsic_data = json.load(camera_intrinsics)
    camera_resolution = intrinsic_data["resolution"]
    w, h = camera_resolution.split("x",1)
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(intrinsic, 
                            width=w, height=h, 
                            fx=intrinsic_data["fx"], fy=intrinsic_data["fy"], 
                            cx=intrinsic_data["cx"], cy=intrinsic_data["cy"])

    ## Generate a point cloud model
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.create_from_rgbd_image(rgbd_image, intrinsic, extrinsic)
    o3d.io.write_point_cloud("kinova_pointcloud.ply", point_cloud)

if __name__ == "__main__":
    exit(main())