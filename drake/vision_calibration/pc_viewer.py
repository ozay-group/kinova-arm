import numpy as np
import open3d as o3d


vis = o3d.visualization.Visualizer()
vis.create_window(window_name="viewer")


pc_s = o3d.io.read_point_cloud("cropped_pc_static.ply")
min_bound_s = pc_s.get_min_bound()
print(min_bound_s)
print(pc_s.get_max_bound())
vis.add_geometry(pc_s)

pc_k = o3d.io.read_point_cloud("cropped_kinova_pointcloud.ply")
max_bound_k = pc_k.get_max_bound()
print(max_bound_k)
min_bound_k = pc_k.get_min_bound()
print(min_bound_k)
center = (min_bound_k + max_bound_k)/2
print(center)
#scale = np.sum(min_bound_s / max_bound_k)/3
scale = 30000
print(scale)
pc_k_scaled = pc_k.scale(scale, center)
vis.add_geometry(pc_k_scaled)

#o3d.io.write_point_cloud("cropped_kinova_pointcloud.ply", cropped_geometry)
#o3d.io.write_point_cloud("cropped_pc_static.ply",cropped_geometry)

vis.run()