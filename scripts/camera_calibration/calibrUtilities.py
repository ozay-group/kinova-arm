"""
Utility functions used for camera calibration

Author: Zexiang Liu
Date: May 2024
"""
import numpy as np
# average of SE3
from liegroups import SE3
from ransac import ransac

def plot_frame(ax, pose, width=0.5, alpha = 1):
    x, y, z = pose[:3, 3]
    ax.scatter(x, y, z, c='k', marker='o')
    colors = ['r', 'g', 'b']
    r = 1 # scaling factor
    for i in range(3):
        ax.plot([x, x+r*pose[0, i]], [y, y+r*pose[1, i]], [z, z+r*pose[2, i]], c=colors[i], linewidth= width, alpha=alpha)

def averageSE3(se3_list, thres = 0.05):
    # Input: a list of se3 elements
    #        threshold for ransac
    # Output: the exp of the average of the se3 elements
#     avg_se3 = np.mean(se3_list, axis=0) 
    if len(se3_list) == 0:
        return None, None
    if len(se3_list) == 1:
        return SE3.exp(se3_list[0]).as_matrix(), se3_list
    avg_se3, inliers = ransac(se3_list, 0.2, 200, thres)
    if avg_se3 is not None:
        avg_pose = SE3.exp(avg_se3).as_matrix()
        return avg_pose, inliers
    else:
        return None, None

# def visualize_frame(name:str, pose:RigidTransform):
#     AddMeshcatTriad(
#             meshcat, name, length=0.15, radius=0.006, X_PT=pose
#         )
# visualize_frame("/ee", convertRPY2RigidTransform(pose_log_data[:,-1]))