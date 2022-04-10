import scipy.io as sio

import sys
sys.path.append('../')
from classes.consistentbeliefcontroller import matfile_data_to_cbc

"""
Algorithm
"""
mat_data_filename = "test_cbc2.mat"

temp2 = sio.loadmat(mat_data_filename)
# print(temp2)
# print(temp2['K_set'])
# print(temp2['U_H'])

# print("temp2['K_set'][0,0] = ")
# print(temp2['K_set'][0,0])

# print("temp2['K_set'].shape = ",temp2['K_set'].shape)

cbc1 = matfile_data_to_cbc(mat_data_filename)

x_0_t, u_0_tm1 , y_0_t , sig = cbc1.simulate_1_run()

print('x_0_t = ')
print(x_0_t)

print('u_0_tm1 = ')
print(u_0_tm1)

print('y_0_t = ')
print(y_0_t)

print('sig = ')
print(sig)