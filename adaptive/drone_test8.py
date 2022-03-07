"""
drone_test3.py
Description:
    Using the python Quadrotor object to linearize the system about several different points along a given trajectory.
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

from quadrotor.quadrotor import Quadrotor

from argparse import ArgumentError

import control
import polytope as pc
import gurobipy as gp
from gurobipy import GRB

def QuadrotorTrackingMPC(s_trajectory_in,u_trajectory_in,dt,P=None,P_T=None,R=None,U:pc.Polytope=None):
    """
    NonlinearTrackingMPC
    Description:

    Usage:
        u, u_star, opt_val, err_messages = QuadrotorTrackingMPC(s_trajectory_in,u_trajectory_in,dt)

    Inputs:

    Outputs:
        @output u_k: The input that has been chosen for the current time step.
        @output u_star: The vector containing the SEQUENCE of inputs selected for the next TimeHorizon steps of the approximation to the quadrotor.
        @output obj_val: The optimal value of the Quadratic Program's objective.
        @output error_string: A notification of what errors might have occurred in the function.
    """

    # Constants
    quad_model = Quadrotor()

    s0 = s_trajectory_in[:,0] # First column is the initial state
    TimeHorizon = s_trajectory_in.shape[1]-1

    # Get System Dimensions
    n_s = 12
    n_u = 4

    # Input Checking
    if P is not None:
        if np.any(P.shape != (n_s,n_s)):
            raise Exception('The shape of P (' + str(P.shape) + ') did not match n_s (=' + str(n_s) + ').')

    # Create Default Values
    if P is None:
        P = np.eye(n_s)

    if P_T is None:
        P_T = np.eye(n_s)

    if R is None:
        R = np.eye(n_u)

    if U is None:
        thrust_bound = 10
        angle_bound = 100
        U = pc.box2poly([ [-thrust_bound,thrust_bound], [-angle_bound,angle_bound], [-angle_bound,angle_bound], [-angle_bound,angle_bound] ])

    # Linearizing Drone About the given Trajectory
    ad_list = []
    for k in range(TimeHorizon):
        # Create continuous-time system based on the trajectory at the kth index/point
        s_k = np.array(np.real(s_trajectory_in[:,k+1])).flatten()
        u_k = np.array(np.real(u_trajectory_in[:,k])).flatten()
        Ac_k, Bc_k = quad_model.GetLinearizedMatricesAbout(s_k,u_k)
        Kc_k = np.matrix(quad_model.f( -1 , s_k , u_k )) - np.dot(Ac_k,s_k) - np.dot(Bc_k,u_k)
        Kc_k = Kc_k.T
        cont_sys1 = control.StateSpace(Ac_k,Bc_k,np.eye(Ac_k.shape[0]),np.zeros(shape=(Ac_k.shape[0],Bc_k.shape[1])))
        cont_sys2 = control.StateSpace(Ac_k,Kc_k,np.eye(Ac_k.shape[0]),np.zeros(shape=(Ac_k.shape[0],Kc_k.shape[1])))

        # Create Discretized Version of System
        disc_sys1 = control.c2d( cont_sys1 , dt )
        disc_sys2 = control.c2d( cont_sys2 , dt )

        Ad_k = disc_sys1.A
        Bd_k = disc_sys1.B
        Kd_k = disc_sys2.B

        # Create Affine Dynamics Object with Discrete-Time representation
        ad_k = AffineDynamics(A=Ad_k,B=Bd_k,K=Kd_k)
        ad_list.append(ad_k)

    # Create SwitchedAffine System Model from linearized system matrices
    words = ( np.arange(TimeHorizon), )
    L_seq = Language(words)
    sad_model = SwitchedAffineDynamics(ad_list,L_seq)

    # Create target sequence of states
    x_target_barre = np.reshape(s_trajectory_in[:,1:],(np.prod(s_trajectory_in[:,1:].shape),))

    # Create target sequence of inputs
    if len(u_trajectory_in.shape) == 1:
        tempOneHotArray = np.ones(shape=(TimeHorizon,))
        u_target_barre = np.kron( tempOneHotArray , u_trajectory_in )
    elif u_trajectory_in.shape[1] == 1:
        tempOneHotArray = np.ones(shape=(TimeHorizon,))
        u_target_barre = np.kron( tempOneHotArray , u_trajectory_in )
    elif u_trajectory_in.shape[1] == TimeHorizon:
        u_target_barre = np.reshape(u_trajectory_in,(np.prod(u_trajectory_in.shape),))

    # Create MPC Matrices for sad_model
    S_w, S_u, S_s0, S_K = sad_model.get_mpc_matrices(L_seq.words[0])

    # Create Extended Cost Matrices
    P_barre = scipy.linalg.block_diag( np.kron(np.eye(TimeHorizon-1,dtype=float),P) , P_T )
    R_barre = np.kron( np.eye(TimeHorizon,dtype=float) , R )

    Q = np.dot(S_u.T,np.dot(P_barre , S_u)) + R_barre
    q = 2.0 * np.dot( (np.dot(S_s0,s0) + S_K - x_target_barre.T).T , np.dot(P_barre,S_u) ) + 2.0 * np.dot( -u_target_barre , R_barre )

    # print('Q = ',Q)
    # print('q = ',q)

    # Input Constraints

    try:
        m = gp.Model("Nonlinear MPC1")
        u = m.addMVar(shape=(n_u*TimeHorizon,),lb=-float('inf'),vtype=GRB.CONTINUOUS)

        # Create Objective
        m.setObjective( u @ Q @ u + q @ u , GRB.MINIMIZE )

        # Add constraints
        m.addConstr(np.kron(np.eye(TimeHorizon),U.A) @ u <= np.kron(np.ones((TimeHorizon,)),U.b) )

        # Optimize model
        m.optimize()

        return u.X[:n_u], u.X, m.ObjVal , None

    except gp.GurobiError as e:

        return None, None, float('inf'), "Gurobi Error! Error code " + str(e.errno) + ": " + str(e)

    except AttributeError:
        return None, None, float('inf'), "Encountered an attribute error!"

print("drone_test8.py\n\n")

###########
# Constants
###########

q0 = Quadrotor()
t = 5

t0 = 0
# s_init = np.array([0.5,0.2,0.5,0.0,0.0,0.0,0,0,0,0,0,0])
s_init = np.array([23,23,-23,0.1,0.05,0.2,0,0,0,0,0,0]) # Create initial state

u0 = np.array([0.5,0.0,0,0])

plotting_target_trajectory = False

############################
# Create Target Trajectory #
############################

# Simulate Quadrotor for a certain amount of time
r = ode(q0.f).set_integrator('zvode', method='bdf')
r.set_initial_value(s_init,t0).set_f_params(u0)
print("Was the differential equation setup successful?",r.successful())

# Simulate
t1 = 5 # seconds
delta_t = 0.01
s_trajectory = np.matrix(s_init)
u_trajectory = np.matrix(u0)
t_trajectory = np.matrix([[0.0]])
#print(s_trajectory.shape)

while r.successful() and r.t < t1:
    s_t = r.integrate(r.t+delta_t)
    s_trajectory = np.concatenate( (s_trajectory,np.matrix(s_t)),axis=0 )
    u_trajectory = np.concatenate( (u_trajectory,np.matrix(u0)),axis=0 )
    t_trajectory = np.concatenate( (t_trajectory,np.matrix([[r.t+delta_t]])),axis=0)
    # print(r.t+delta_t, s_t)
    # print(s_trajectory)

if plotting_target_trajectory:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot( np.squeeze(np.asarray(s_trajectory[:,[0]])),np.squeeze(np.asarray(s_trajectory[:,[1]])) , np.squeeze(np.asarray(s_trajectory[:,[2]])) )
    ax.scatter(0,1,2)

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('-z')
    plt.show()

#######################################################
## Use MPC Controller to Attempt to Track Trajectory ##
#######################################################

T = 5
#k_random = np.random.randint(0,s_trajectory.shape[0]-T-1)
k_random = 4

s_segment = np.real(s_trajectory[k_random:k_random+T+1,:]).T
u_segment = np.kron(np.ones(shape=(1,T)),np.matrix(u0).T)

# print(s_segment)
print('u_segment = ')
print(u_segment)

P0 = np.eye(12)
P0[1,1] = 10
P0[2,2] = 10
P0[3,3] = 10

u_mpc, u_star, opt_val, err = QuadrotorTrackingMPC( s_segment , u_segment , delta_t , P=P0)
print('QuadrotorTrackingMPC:')
print('u_mpc = ', u_mpc)
print('u* = ',u_star)
print('opt_val = ',opt_val)
print('err = ',err)

s_kp1 = q0.f(-1, np.array(s_segment[:,0]).flatten() , u_mpc )

print('||s_segment[:,k+1] - s_simulated || = ', np.linalg.norm( s_segment[:,1] - s_kp1 ))