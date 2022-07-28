import numpy as np
import open3d as o3d


vis = o3d.visualization.Visualizer()
vis.create_window(window_name="viewer")




np_points = np.array([[0, 0, 0],
                      [1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1],
                      [1, 1, 0],
                      [1, 0, 1],
                      [0, 1, 1],
                      [1, 1, 1]])


#move = np.tile(np.array([-1, -0.3, -2]), (8,1)),  # x,y,z
#scale = 2
#scale = 0.00004
#np_points = np.squeeze((np_points * scale + move), axis=0)
#offset_kinova = np.tile(np.array([-0.1671, 0.5135, 0.2879]), (8,1)),  # x,y,z
#np_points = np.squeeze(np_points * scale + offset_kinova, axis=0)


points = o3d.utility.Vector3dVector(np_points)
box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(points)
vis.add_geometry(box)


#geometry = o3d.io.read_point_cloud("pc_static.ply")
geometry = o3d.io.read_point_cloud("kinova_pointcloud.ply")
#vis.add_geometry(geometry)
print(geometry.get_max_bound())
print(geometry.get_min_bound())

#cropped_geometry = geometry.crop(box)
cropped_geometry = geometry
o3d.io.write_point_cloud("cropped_kinova_pointcloud.ply", cropped_geometry)
#o3d.io.write_point_cloud("cropped_pc_static.ply",cropped_geometry)
vis.add_geometry(cropped_geometry)
vis.run()