"""
drone_test5.py
Description:
    In this test, we will observe how the MPC-based controller works.
"""

# Import
import numpy as np
import affinedynamics as ad

# Define Simple Discrete-Time Linear System
dim_x = 2
A = np.eye(dim_x)
B = np.array([[0.0],[1.0]])

K = np.array([[1.0],[0.0]])

affine_system1 = ad.AffineDynamics(A=A,B=B,K=K)

print(affine_system1)

# Create MPC problem and try to solve with gurobi