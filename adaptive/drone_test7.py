"""
drone_test3.py
Description:
    Using the python Quadrotor object to linearize the system about several different points along a given trajectory.
"""

import numpy as np
from scipy.integrate import ode
import sympy as sp

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from classes.affinedynamics import AffineDynamics
from classes.switchedaffinedynamics import SwitchedAffineDynamics
from classes.language import Language

from quadrotor.quadrotor import Quadrotor

from argparse import ArgumentError

def mpc_matrix_comparison(ad_in:AffineDynamics,sad_in:SwitchedAffineDynamics,word_i):
    """
    mpc_matrix_comparison
    Description:
        This compares the mpc matrices created by ad with time horizon
        T (determined by the words in sad) and the mpc matrix of sad word
        word index word_i.
    """

    # input processing
    if (word_i >= sad_in.n_modes()) or (word_i < 0):
        raise ArgumentError(None,'The mode ' + str(word_i) + ' was not recognized in SwitchedAffineDynamics with n_modes = ' + str( sad.n_modes() ) + '.')

    # Constants
    L = sad_in.L
    T = len(L.words[0])

    # Algorithm
    S_w_ad,  S_u_ad,  S_x0_ad,  S_K_ad  = ad_in.get_mpc_matrices(T)
    S_w_sad, S_u_sad, S_x0_sad, S_K_sad = sad_in.get_mpc_matrices(L.words[word_i])

    # Return whether or not all matrices match
    return np.all( S_w_ad == S_w_sad) and np.all( S_u_ad == S_u_sad ) and np.all( S_x0_ad == S_x0_ad ) and np.all( S_K_ad == S_K_sad )

print("drone_test3.py\n\n")

###########
# Constants
###########

q0 = Quadrotor()
t = 1

t0 = 0
s0 = np.array([0.5,0.2,0.5,0.2,0.4,0.2,0,0,0,0,0,0])
u0 = np.array([0.5,0.05,0,0])

plotting_target_trajectory = False

# Simulate Quadrotor for a certain amount of time
r = ode(q0.f).set_integrator('zvode', method='bdf')
r.set_initial_value(s0,t0).set_f_params(u0)
print("Was the differential equation setup successful?",r.successful())

# Simulate
t1 = 5 # seconds
dt = 0.2
s_trajectory = np.matrix(s0)
t_trajectory = np.matrix([[0.0]])
#print(s_trajectory.shape)

while r.successful() and r.t < t1:
    s_t = r.integrate(r.t+dt)
    s_trajectory = np.concatenate( (s_trajectory,np.matrix(s_t)),axis=0 )
    t_trajectory = np.concatenate( (t_trajectory,np.matrix([[r.t+dt]])),axis=0)
    # print(r.t+dt, s_t)
    # print(s_trajectory)

#print(t_trajectory)

# Pick a random point in the trajectory and linearize about it.
k_random = np.random.randint(0,s_trajectory.shape[0])

s_r = np.array(np.real(s_trajectory[k_random,:]))
s_r = s_r.flatten()
u_r = u0

print(q0.GetLinearizedMatricesAbout(s_r,u_r))

# Create first two systems
print(np.array(np.real(s_trajectory[0,:])).flatten())
print(np.matrix(q0.f( t_trajectory[0] , s0 , u0 )).T)
s0 = np.array(np.real(s_trajectory[0,:])).flatten()
A0, B0 = q0.GetLinearizedMatricesAbout(s0,u0)
K0 = np.matrix(q0.f( t_trajectory[0] , s0 , u0 )).T
ad1 = AffineDynamics(A=A0,B=B0,K=K0)

s2 = np.array(np.real(s_trajectory[1,:])).flatten()
A2, B2 = q0.GetLinearizedMatricesAbout(s_trajectory[1,:].flatten(),u0)
K2 = np.matrix(q0.f( t_trajectory[0] , s2 , u0 )).T
ad2 = AffineDynamics(A=A2,B=B2,K=K2)

# Create Switched System
sas1 = SwitchedAffineDynamics([ad1,ad2])
print(sas1)

# Create a switched system based on a time horizon and initial point
k_i = 3
T = 4
ad_list = []
for k in range(k_i,k_i+T):
    # Create system based on the trajectory at the kth index/point
    s_k = np.array(np.real(s_trajectory[k,:])).flatten()
    A_k, B_k = q0.GetLinearizedMatricesAbout(s_k,u0)
    K_k = np.matrix(q0.f( t_trajectory[k] , s_k , u0 )).T
    ad_k = AffineDynamics(A=A_k,B=B_k,K=K_k)
    ad_list.append(ad_k)

sas2 = SwitchedAffineDynamics(ad_list,Language([i for i in range(T)]))

L3 = Language( (np.zeros(shape=(1,T),dtype=int).flatten(),np.ones(shape=(1,T),dtype=int).flatten()) )
sas3 = SwitchedAffineDynamics([ad1,ad2],L3)

print(sas3.L.words[0])
print(sas3.L.words[0][0])

print("mpc_matrix_comparison() = " + str(mpc_matrix_comparison( ad1 , sas3, 0 )))

if plotting_target_trajectory:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot( np.squeeze(np.asarray(s_trajectory[:,[0]])),np.squeeze(np.asarray(s_trajectory[:,[1]])) , np.squeeze(np.asarray(s_trajectory[:,[2]])) )
    ax.scatter(0,1,2)

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('-z')
    plt.show()
