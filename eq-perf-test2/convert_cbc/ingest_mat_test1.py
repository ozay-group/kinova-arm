import scipy.io as sio

"""
Algorithm
"""
mat_data_filename = "test_cbc1.mat"

temp2 = sio.loadmat(mat_data_filename)
print(temp2)
print(temp2['K_set'])
print(temp2['U_H'])
