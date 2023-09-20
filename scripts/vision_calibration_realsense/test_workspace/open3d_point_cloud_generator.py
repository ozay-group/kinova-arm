"""
This is a workspace. It makes sense if no line is executable.
"""

color_raw = o3d.io.read_image()

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image, convert_rgb_to_intensity = False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, color_intrinsics)

        # flip the orientation, so it looks upright, not upside-down
        pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

        o3d.draw_geometries([pcd])    # visualize the point cloud