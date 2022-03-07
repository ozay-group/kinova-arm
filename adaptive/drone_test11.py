"""
drone_test11.py
Description:
    Using the python SmallOscillationQuadrotor object to linearize the system about several different points along a given trajectory.
"""

import numpy as np
import scipy
from scipy.integrate import ode
import sympy as sp

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from classes.affinedynamics import AffineDynamics
from classes.switchedaffinedynamics import SwitchedAffineDynamics
from classes.language import Language

from quadrotor.small_oscillations_model import SmallOscillationsQuadrotor

from argparse import ArgumentError

import control
import polytope as pc
import gurobipy as gp
from gurobipy import GRB

print("drone_test11.py\n\n")

###########
# Constants
###########

q0 = SmallOscillationsQuadrotor()
t = 5

# Simulation Limits
t0 = 0
t1 = 5 # seconds

s_init = np.array([0,0.0,0.0,0,0,0,0,0,0,23,23,-23]) # Create initial state
s_target = np.array([0,0.0,0.0,0,0,0,0,0,0,25,25,-25]) # Create target state

u_stable = np.array([q0.m*9.81,0.0,0.0,0.0])

dt = 0.01

######################################################
# Create Linearized System About the Target Location #
######################################################

Ac_k, Bc_k = q0.GetLinearizedMatricesAbout(s_target,u_stable)
Kc_k = np.matrix(q0.f(-1,s_target,u_stable))
Kc_k = Kc_k.T

# Discretize System
cont_sys1 = control.StateSpace(Ac_k,Bc_k,np.eye(Ac_k.shape[0]),np.zeros(shape=(Ac_k.shape[0],Bc_k.shape[1])))
cont_sys2 = control.StateSpace(Ac_k,Kc_k,np.eye(Ac_k.shape[0]),np.zeros(shape=(Ac_k.shape[0],Kc_k.shape[1])))

# Create Discretized Version of System
disc_sys1 = control.c2d( cont_sys1 , dt )
disc_sys2 = control.c2d( cont_sys2 , dt )

Ad = disc_sys1.A
Bd = disc_sys1.B
Kd = disc_sys2.B

# Q = np.zeros(shape=(12,12))
# Q[9,9]   = 1
# Q[10,10] = 1
# Q[11,11] = 1
Q = np.eye(12)
Q[9,9]   = 10
Q[10,10] = 10
Q[11,11] = 10

R = np.eye(4)
R[1,1] = 10
R[2,2] = 10
R[3,3] = 10

print('controllability matrix rank = ',np.linalg.matrix_rank(control.ctrb(Ad,Bd)))

K , S , E = control.lqr( disc_sys1 , Q , R  )
print(K)

plotting_target_trajectory = True

############################
# Create Target Trajectory #
############################

u0 = np.dot(K,s_init - s_target)
t = 0.0

# Simulate Quadrotor for a certain amount of time
r_t = ode(q0.f).set_integrator('zvode', method='bdf')
r_t.set_initial_value(s_init,t0).set_f_params(u0)
print("Was the differential equation setup successful?",r_t.successful())

# Simulate
s_trajectory = np.matrix(s_init)
u_trajectory = np.matrix(u0)
t_trajectory = np.matrix([[0.0]])
#print(s_trajectory.shape)
while r_t.successful() and r_t.t < t1:
    s_t = r_t.integrate(r_t.t+dt)
    t   = r_t.t+dt
    u_t = np.dot(K,s_t - s_target)
    print(u_t)

    s_trajectory = np.concatenate( (s_trajectory,np.matrix(s_t)),axis=0 )
    u_trajectory = np.concatenate( (u_trajectory,np.matrix(u0)),axis=0 )
    t_trajectory = np.concatenate( (t_trajectory,np.matrix([[r_t.t+dt]])),axis=0)
    #print(r_t.t+dt, s_t)

    #print(r_t.successful())
    # print(s_trajectory)

    # Create new simulator with these initial values
    r_t = ode(q0.f).set_integrator('zvode', method='bdf')
    r_t.set_initial_value(s_t,t).set_f_params(u_t)

print(u_trajectory)

if plotting_target_trajectory:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot( np.squeeze(np.asarray(s_trajectory[:,[9]])),np.squeeze(np.asarray(s_trajectory[:,[10]])) , -np.squeeze(np.asarray(s_trajectory[:,[11]])) )
    # ax.scatter(0,1,2)

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('-z')
    plt.show()