"""
drone_test1.py
Description:
    Using the python Quadrotor object to create a trajectory using the Quadrotor.
"""

import numpy as np
from scipy.integrate import ode

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from quadrotor.quadrotor import Quadrotor

print("Kehlani")

s0 = np.zeros((12,))
s0[3] = 1.0
t0 = 0.0
u0 = np.zeros((4,))
u0[0] = 11

q0 = Quadrotor()

r = ode(q0.f).set_integrator('zvode', method='bdf')
r.set_initial_value(s0,t0).set_f_params(u0)

print(r.successful())

t1 = 10
dt = 1
s_trajectory = np.matrix(s0)
print(s_trajectory.shape)

while r.successful() and r.t < t1:
    s_t = r.integrate(r.t+dt)
    s_trajectory = np.concatenate( (s_trajectory,np.matrix(s_t)),axis=0 )
    print(r.t+dt, s_t)
    print(s_trajectory)

# print(s_trajectory[0,:])
# print(s_trajectory[:,0])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot( np.squeeze(np.asarray(s_trajectory[:,[0]])),np.squeeze(np.asarray(s_trajectory[:,[1]])) , np.squeeze(np.asarray(s_trajectory[:,[2]])) )
ax.scatter(0,1,2)
plt.show()

# # Plot Second Figure for second motion primitive

# u0 = np.zeros((4,))
# u0[0] = 11
# u0[1] = 0.1

# r2 = ode(quadrotor_dynamics).set_integrator('zvode', method='bdf')
# r2.set_initial_value(s0,t0).set_f_params(u0)

# s_trajectory2 = np.matrix(s0)
# print(s_trajectory2.shape)

# while r2.successful() and r2.t < t1:
#     s_t = r2.integrate(r2.t+dt)
#     s_trajectory2 = np.concatenate( (s_trajectory2,np.matrix(s_t)),axis=0 )
#     print(r2.t+dt, s_t)
#     print(s_trajectory2)

# fig2 = plt.figure()
# ax2 = fig2.add_subplot(111, projection='3d')
# ax2.plot( np.squeeze(np.asarray(s_trajectory2[:,[0]])),np.squeeze(np.asarray(s_trajectory2[:,[1]])) , np.squeeze(np.asarray(s_trajectory2[:,[2]])) )
# ax2.scatter(0,1,2)
# ax2.set_xlabel('x')
# ax2.set_ylabel('y')
# ax2.set_zlabel('-z')
# plt.show()
