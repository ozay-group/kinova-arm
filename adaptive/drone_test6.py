"""
drone_test5.py
Description:
    In this test, we will observe how the MPC-based controller works.
"""

# Import
from os import system
from sqlite3 import Time
import numpy as np
from classes.affinedynamics import AffineDynamics
import classes.affinedynamics as ad
import gurobipy as gp
from gurobipy import GRB
import scipy

import polytope as pc
import qpsolvers

# Define MPC Function
def AffineMPC( system_in:AffineDynamics , x0:np.ndarray=None, TimeHorizon=-1, P=None,P_T=None,R=None,x_target=None,u_target=None,U:pc.Polytope=None):
    """
    AffineMPC
    Description:
        This function computes the output of an affine MPC Problem with quadratic cost in the structure of:

            min \sum_{k=1}^{T-1} x_k^T P x_k + u_k^T R u_k + x_T^T P_T x_T + u_T^T R u_T

    Inputs:
        @param system_in: An affine dynamics object that is properly defined.
        @param TimeHorizon: The time horizon that this MPC problem is considered on. T should be an integer.
        @param P (optional): The running cost of each state in the time horizon T, except for the last state. P is a square matrix with the same dimension as the state of system_in.
        @param P_T (optional): The terminal cost of the state (i.e. quadratic cost associated with the final state). P_T is a square matrix with the same dimension as the state of system_in.
        @param R (optional): The running cost of each input in the time horizon T. R is a square matrix with the same dimension as the input of system_in.
        @param x_target (optional): The target state of the MPC problem. If a single state is given, then this target is included in the cost for just the terminal state. If this is given as a sequence
        of states, then this is provided as a sequence of targets (i.e., each element x_target[:,k] is the target for time k).
        @param u_target (optional): The target input of the MPC problem. If a single input is given, then this target is included in the input for just the terminal state. If this is given as a sequence
        of inputs, then this is provided as a sequence of targets (i.e., each element u_target[:,k] is the target for time k).
        @param U (optional): The set of states that the input must be selected from. U should be a polytope object.

         
    """

    # Get System Dimensions
    n_x = system_in.A.shape[0]
    n_u = system_in.B.shape[1]

    # Create Default Values for x_target, u_target
    if P is None:
        P = np.eye(n_x)

    if P_T is None:
        P_T = np.eye(n_x)

    if R is None:
        R = np.eye(n_u)

    if x_target is None:
        x_target = np.zeros((n_x,))

    if u_target is None:
        u_target = np.zeros((n_u,))

    if x0 is None:
        x0 = np.zeros((n_x,))

    # print('P = ',P)
    # print('P_T = ',P_T)
    # print('R = ',R)
    # print('x_target = ',x_target)
    # print('u_target = ',u_target)
    # print('x0 = ',x0)

    # Create Target Vectors, if they exist
    if len(x_target.shape) == 1:
        tempOneHotArray = np.zeros((TimeHorizon,1))
        tempOneHotArray[-1] = 1
        x_target_barre = np.kron( tempOneHotArray , x_target )
    elif x_target.shape[1] == 1:
        tempOneHotArray = np.zeros((TimeHorizon,1))
        tempOneHotArray[-1] = 1
        x_target_barre = np.kron( tempOneHotArray , x_target )
    elif x_target.shape[1] == TimeHorizon:
        x_target_barre = np.reshape(x_target,(np.prod(x_target.shape),))

    if len(u_target.shape) == 1:
        tempOneHotArray = np.zeros((TimeHorizon,))
        tempOneHotArray[-1] = 1
        u_target_barre = np.kron( tempOneHotArray , u_target )
    elif u_target.shape[1] == 1:
        tempOneHotArray = np.zeros((TimeHorizon,))
        tempOneHotArray[-1] = 1
        u_target_barre = np.kron( tempOneHotArray , u_target )
    elif u_target.shape[1] == TimeHorizon:
        u_target_barre = np.reshape(u_target,(np.prod(u_target.shape),))

    # print('x_target_barre = ', x_target_barre)
    # print('u_target_barre = ', u_target_barre)

    # Get MPC Matrices
    S_w, S_u, S_x0, S_K = system_in.get_mpc_matrices(TimeHorizon)

    # Create Extended Cost Matrices
    P_barre = scipy.linalg.block_diag( np.kron(np.eye(TimeHorizon-1,dtype=float),P) , P_T )
    R_barre = np.kron( np.eye(TimeHorizon,dtype=float) , R )

    Q = np.dot(S_u.T,np.dot(P_barre , S_u)) + R_barre
    q = 2.0 * np.dot( (np.dot(S_x0,x0) + S_K.T - x_target_barre) , np.dot(P_barre,S_u) ) + 2.0 * np.dot( -u_target_barre , R_barre )

    # print('Q = ',Q)
    # print('q = ',q)

    # Input Constraints

    try:
        m = gp.Model("MPC1")
        u = m.addMVar(shape=(dim_u*TimeHorizon,),lb=-float('inf'),vtype=GRB.CONTINUOUS)

        # Create Objective
        m.setObjective( u @ Q @ u + q @ u , GRB.MINIMIZE )

        # Add constraints
        m.addConstr(np.kron(np.eye(TimeHorizon),U.A) @ u <= np.kron(np.ones((TimeHorizon,)),U.b) )

        # Optimize model
        m.optimize()

        return u.X, m.ObjVal , None

    except gp.GurobiError as e:

        return None, float('inf'), "Error code " + str(e.errno) + ": " + str(e)

    except AttributeError:
        return None, float('inf'), "Encountered an attribute error!"

# Define Simple Discrete-Time Linear System
dim_x = 2
A = 1*np.eye(dim_x)
#A[0,1] = 0.2

# dim_u = 1
# B = np.array([[0.0],[1.0]])
dim_u = 2
B = np.eye(dim_u)

K = np.array([[1.0],[0.0]])

x0_0 = np.zeros((2,1))

affine_system2 = ad.AffineDynamics(A=A,B=B,K=K)

print(affine_system2)
affine_system2.print_matrices()

"""
Create MPC problem and try to solve with gurobi
"""

# Create state quadratic cost (Q) and input quadratic cost R
P0 = np.eye(dim_x)
P_T0 = P0
R0 = 0.01*np.eye(dim_u)
#R = np.zeros((dim_u,dim_u))

TimeHorizon_prime = 5
x_target = np.array([[1.0],[1.0]])

# Input Constraints
umin = -10
umax = 10
U0 = pc.box2poly([ [umin,umax] for i in range(0,dim_u) ])

u_star, cost_value, flags = AffineMPC( affine_system2 , TimeHorizon=TimeHorizon_prime , P=P0, P_T=P_T0, R=R0, x_target=np.kron(np.ones((1,TimeHorizon_prime)),x_target), U=U0 )

print('Obj: %g' % cost_value)
print('u*: ',u_star)

if u_star is not None:
    # Use Identified u to see if system reaches target
    x = np.zeros((dim_x,TimeHorizon_prime+1))
    x[:,0] = x0_0.flatten()
    for t in range(TimeHorizon_prime):
        #print(affine_system1.f(x[:,t].flatten(),u_star[t*dim_u:(t+1)*dim_u],flags="no_w"))
        x[:,t+1] = affine_system2.f(x[:,t].flatten(),u_star[t*dim_u:(t+1)*dim_u],flags="no_w").flatten()

    print( "x[:,-1] = " , x[:,-1] )

    S_w2, S_u2, S_x02, S_K2 = affine_system2.get_mpc_matrices(TimeHorizon_prime)
    print( "x_traj (according to mpc matrices) = " , np.dot(S_u2,np.matrix(u_star).T) + np.dot(S_x02,x0_0) + S_K2  )


