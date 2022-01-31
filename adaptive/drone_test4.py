"""
drone_test4.py
Description:
    Using the python Quadrotor object to linearize the system about several different points along a given trajectory.
"""

import numpy as np
import scipy
from scipy.integrate import ode
import sympy as sp

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from affinedynamics import AffineDynamics

from quadrotor.quadrotor import Quadrotor

print("drone_test3.py\n\n")

# Constants

q0 = Quadrotor()
t = 1

t0 = 0
s0 = np.array([0.5,0.2,0.5,0.2,0.4,0.2,0,0,0,0,0,0])
u0 = np.array([0.5,0.05,0,0])

# Simulate Quadrotor for a certain amount of time
r = ode(q0.f).set_integrator('zvode', method='bdf')
r.set_initial_value(s0,t0).set_f_params(u0)
print("Was the differential equation setup successful?",r.successful())

# Simulate
t1 = 5 # seconds
dt = 0.2
s_trajectory = np.matrix(s0)
t_trajectory = np.array([0.0])
#print(s_trajectory.shape)

while r.successful() and r.t < t1:
    s_t = r.integrate(r.t+dt)
    s_trajectory = np.concatenate( (s_trajectory,np.matrix(s_t)),axis=0 )
    np.append(t_trajectory,r.t+dt)
    # print(r.t+dt, s_t)
    # print(s_trajectory)

# print(s_trajectory[0,:])
# print(s_trajectory[:,0])

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.plot( np.squeeze(np.asarray(s_trajectory[:,[0]])),np.squeeze(np.asarray(s_trajectory[:,[1]])) , np.squeeze(np.asarray(s_trajectory[:,[2]])) )
# ax.scatter(0,1,2)

# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('-z')
# plt.show()

# Pick a random point in the trajectory and linearize about it.
k_random = np.random.randint(0,s_trajectory.shape[0])

s_r = np.array(np.real(s_trajectory[k_random,:]))
s_r = s_r.flatten()
u_r = u0

print(q0.GetLinearizedMatricesAbout(s_r,u_r))

"""
Create the MPC Problem
"""
# Define Constants
k0 = 0
s0 = np.array(np.real(s_trajectory[k_random,:]))
s0 = s_r.flatten()
u0 = u0

T = 5
A, B = q0.GetLinearizedMatricesAbout(s0,u0)

n_x = A.shape[0]
n_u = B.shape[1]

P = np.diag( np.ones((12,)) )
P_T = np.diag( np.ones((12,)) )
R = np.diag( np.ones((4,)))

K = q0.f(t_trajectory[k0],s0,u0)

# Create the MPC Matrices (S_w)
S_w = np.zeros((T*n_x,T*n_x))
for j in range(T):
    for i in range(j,T):
        S_w[i*n_x:(i+1)*n_x, j*n_x:(j+1)*n_x]=np.linalg.matrix_power(A,i-j) 

# Create the MPC Matrices (S_u)
S_u = np.zeros((T*n_x,T*n_u))
for j in range(T):
    for i in range(j,T):
        S_u[i*n_x:(i+1)*n_x, j*n_u:(j+1)*n_u]=np.dot(np.linalg.matrix_power(A,i-j),B)

# Create the MPC Matrices (S_w)
S_x0 = M=np.zeros((T*n_x,n_x))
for j in range(T):
    S_x0[j*n_x:(j+1)*n_x,:]=np.linalg.matrix_power(A,j+1)

# Create the MPC Matrices (S_K)
S_K=np.array([K for i in range(T)])
S_K=S_K.reshape(np.shape(S_K)[0]*np.shape(S_K)[1],1)

# Create state quadratic cost (Q) and input quadratic cost R
Q_barre=scipy.linalg.block_diag(np.kron(np.eye(T-1,dtype=float),P),P_T)
R_barre=np.kron(np.eye(T,dtype=float),R)

# Create the lifted cost matrices in terms of u
print("S_w.shape = ",S_w.shape)
print("Q_barre.shape = ",Q_barre.shape)
print("R_barre.shape = ", R_barre.shape)
H = np.dot(S_u.T,np.dot(Q_barre,S_u))+R_barre

ad0 = AffineDynamics(A,B,np.eye(n_x),q0.f(t_trajectory[k0],s0,u0),np.eye(n_x))
print("ad0.get_mpc_matrices(T=5)",ad0.get_mpc_matrices(T=5))