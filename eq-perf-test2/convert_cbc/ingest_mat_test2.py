import scipy.io as sio

import sys
sys.path.append('../')
from classes.consistentbeliefcontroller import matfile_data_to_cbc

import matplotlib.pyplot as plt
import numpy as np

import polytope as pc

"""
Algorithm
"""
mat_data_filename = "test_cbc2.mat"

temp2 = sio.loadmat(mat_data_filename)
# print(temp2)
# print(temp2['P_target'])
# print(temp2['K_set'])
# print(temp2['U_H'])

# print("temp2['K_set'][0,0] = ")
# print(temp2['K_set'][0,0])

# print("temp2['K_set'].shape = ",temp2['K_set'].shape)

cbc1 = matfile_data_to_cbc(mat_data_filename)

# Simulate trajectories
num_trajs = 10

x_trajectories = np.zeros(shape=(cbc1.system.dim_x(),cbc1.time_horizon()+1,num_trajs))
for traj_index in range(num_trajs):
    x_0_t, u_0_tm1 , y_0_t , sig = cbc1.simulate_1_run()

    print('x_0_t = ')
    print(x_0_t)

    cbc1.clear_histories()
    x_trajectories[:,:,traj_index] = x_0_t



# print('u_0_tm1 = ')
# print(u_0_tm1)

# print('y_0_t = ')
# print(y_0_t)

# print('sig = ')
# print(sig)

# Plot Trajectories
A_target = np.vstack(( -np.eye(cbc1.system.dim_x()),np.eye(cbc1.system.dim_x()) ))
b_target = np.array([[-1.3536],[1.3536],[4.0607],[1.3536]])
target_set = pc.Polytope(A_target,b_target)


fig1, ax = plt.subplots()
target_set.plot(ax)
for traj_index in range(num_trajs):
    temp_x_traj = x_trajectories[:,:,traj_index]
    plt.plot(temp_x_traj[0,:],temp_x_traj[1,:])
    #plt.plot(x_0_t[0,:],x_0_t[1,:])

ax.set_title('Trajectories of the Opposing Rotation System')
ax.set_xlim(-3,4.5)
ax.set_ylim(-2.5,2.5)
plt.show()