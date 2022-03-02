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

matfile_data_to_cbc(mat_data_filename)