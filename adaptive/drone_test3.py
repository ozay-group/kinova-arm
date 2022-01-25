"""
drone_test4.py
Description:
    Using the python Quadrotor object to linearize the system about several different points along a given trajectory.
"""

import numpy as np
from scipy.integrate import ode
import sympy as sp

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
#print(s_trajectory.shape)

while r.successful() and r.t < t1:
    s_t = r.integrate(r.t+dt)
    s_trajectory = np.concatenate( (s_trajectory,np.matrix(s_t)),axis=0 )
    # print(r.t+dt, s_t)
    # print(s_trajectory)

# print(s_trajectory[0,:])
# print(s_trajectory[:,0])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot( np.squeeze(np.asarray(s_trajectory[:,[0]])),np.squeeze(np.asarray(s_trajectory[:,[1]])) , np.squeeze(np.asarray(s_trajectory[:,[2]])) )
ax.scatter(0,1,2)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('-z')
plt.show()

# Pick a random point in the trajectory and linearize about it.
k_random = np.random.randint(0,s_trajectory.shape[0])

s_r = np.array(np.real(s_trajectory[k_random,:]))
s_r = s_r.flatten()
u_r = u0

print(q0.GetLinearizedMatricesAbout(s_r,u_r))