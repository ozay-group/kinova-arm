from random import sample
import sys
sys.path.append('../')

import polytope as pc
import cvxpy as cp
import numpy as np

from scalaruncertainaffinedynamics import ScalarUncertainAffineDynamics, sample_from_polytope

#####################
## Extra Functions ##
#####################

def estimate_theta_lb(uad_in,x_hist,u_hist):
    """
    estimate_theta
    Description:

    """

    # Input Processing
    if x_hist.shape[1] != u_hist.shape[1] + 1:
        raise ValueError("The history of the state (length =" + str(x_hist.shape[1]) + ") and inputs (length =" + str(u_hist.shape[1]) + ") should have dimensions such that state history has one more value than the input history. ")

    # Constants
    t = x_hist.shape[1]-1

    n_x = uad_in.dim_x()
    n_u = uad_in.dim_u()

    W = uad_in.W
    A = uad_in.A
    dA = uad_in.dA
    B = uad_in.B


    # Define and solve the optimization problem with CVXPY.
    # Create a symmetric matrix variable.
    theta = cp.Variable((1,))
    theta_mat = cp.Variable((4,4))
    # The operator >> denotes matrix inequality.
    constraints = [ uad_in.Theta.A @ theta <= uad_in.Theta.b ]
    constraints += [ theta_mat == np.eye(4) * theta  ]
    for k in range(t):
        x_k   = np.reshape(x_hist[:,k],newshape=(n_x,))
        u_k   = np.reshape(u_hist[:,k],newshape=(n_u,))
        x_kp1 = np.reshape(x_hist[:,k+1],newshape=(n_x,))

        # print(x_k)
        # print(u_k)
        # print(x_kp1)

        # print("np.dot(W.A, -np.dot(dA,x_k)) = ")
        # print(np.dot(W.A, -np.dot(dA,x_k)))
        # print("np.dot(W.A, -np.dot(dA,x_k)).shape = ", np.dot(W.A, -np.dot(dA,x_k)).shape)

        constraints += [ 
            theta_mat @ np.dot(W.A, -np.dot(dA,x_k)) <= \
            W.b - np.dot(W.A,x_kp1) + np.dot(W.A, np.dot(A,x_k) + np.dot(B,u_k) )
        ]

    prob = cp.Problem(cp.Minimize(theta),constraints)
    prob.solve()

    # print(prob.status)
    # print(prob.value)
    # print("theta_mat.value = ")
    # print(theta_mat.value)
    return theta.value

def estimate_theta_ub(uad_in,x_hist,u_hist):
    """
    estimate_theta
    Description:

    """

    # Input Processing
    if x_hist.shape[1] != u_hist.shape[1] + 1:
        raise ValueError("The history of the state (length =" + str(x_hist.shape[1]) + ") and inputs (length =" + str(u_hist.shape[1]) + ") should have dimensions such that state history has one more value than the input history. ")

    # Constants
    t = x_hist.shape[1]-1

    n_x = uad_in.dim_x()
    n_u = uad_in.dim_u()

    W = uad_in.W
    A = uad_in.A
    dA = uad_in.dA
    B = uad_in.B

    # Define and solve the optimization problem with CVXPY.
    # Create a symmetric matrix variable.
    theta = cp.Variable((1,))
    theta_mat = cp.Variable((4,4))
    # The operator >> denotes matrix inequality.
    constraints = [ uad_in.Theta.A @ theta <= uad_in.Theta.b ]
    constraints += [ theta_mat == np.eye(4) * theta  ]
    for k in range(t):
        x_k   = np.reshape(x_hist[:,k],newshape=(n_x,))
        u_k   = np.reshape(u_hist[:,k],newshape=(n_u,))
        x_kp1 = np.reshape(x_hist[:,k+1],newshape=(n_x,))

        constraints += [ 
            theta_mat @ np.dot(W.A, -np.dot(dA,x_k)) <= \
            W.b - np.dot(W.A,x_kp1) + np.dot(W.A, np.dot(A,x_k) + np.dot(B,u_k) )
        ]

    prob = cp.Problem(cp.Maximize(theta),constraints)
    prob.solve()

    return theta.value

###############
## Constants ##
###############

T = 5

# Create System
n_x = 2
A1  = np.eye(2)
dA1 = np.zeros(shape=(2,2))
dA1[0,1] = 0.1
B1 = np.array([[0.0],[1.0]])
W1 = pc.box2poly( [ [ -0.1, 0.1 ] , [-0.5, 0.5] ] )
Theta1 = pc.box2poly([ [1.0, 2.0] ])

suad1 = ScalarUncertainAffineDynamics(A1,dA1,B1,W1,Theta1)

x0 = np.array([[1.0],[0.2]])

# Simulate T steps
#theta_star = sample_from_polytope(suad1.Theta)[0][0]
theta_star = 1.5
print('theta_star = ' + str(theta_star) )

x_hist_0_T = np.zeros(shape=( suad1.dim_x() , T+1 ))
u_hist_0_T = np.ones(shape=(suad1.dim_u(),T))

x_hist_0_T[:,0] = x0.flatten()
for tau in range(T):
    x_hist_0_T[:,tau+1] = suad1.f( x_hist_0_T[:,tau] , theta_star , u_hist_0_T[:,tau] )

print('x_hist_0_T =')
print(x_hist_0_T)

# Run Estimator
for tau in range(1,T):
    print('tau = ',tau)
    print('- bar(mu)_' + str(tau) + ' = ')
    print('- ', estimate_theta_lb( suad1 , x_hist_0_T[:,:tau+1] , u_hist_0_T[:,:tau] ) )
    print('- underline(mu)_' + str(tau) + ' = ')
    print('- ', estimate_theta_ub( suad1 , x_hist_0_T[:,:tau+1] , u_hist_0_T[:,:tau] ) )

