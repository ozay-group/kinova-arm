from random import sample
import sys
sys.path.append('../')

import polytope as pc
import cvxpy as cp
import numpy as np

import scipy

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
    K = uad_in.K

    n_WA = W.A.shape[0]

    # Define and solve the optimization problem with CVXPY.
    # Create a symmetric matrix variable.
    theta = cp.Variable((1,))
    theta_mat = cp.Variable((n_WA,n_WA))
    # The operator >> denotes matrix inequality.
    constraints = [ uad_in.Theta.A @ theta <= uad_in.Theta.b ]
    constraints += [ theta_mat == np.eye(n_WA) * theta  ]
    for k in range(t):
        x_k   = np.reshape(x_hist[:,k],newshape=(n_x,1))
        u_k   = np.reshape(u_hist[:,k],newshape=(n_u,1))
        x_kp1 = np.reshape(x_hist[:,k+1],newshape=(n_x,1))

        # print(x_k)
        # print(u_k)
        # print(x_kp1)

        # print("np.dot(W.A, -np.dot(dA,x_k)) = ")
        # print(np.dot(W.A, -np.dot(dA,x_k)))
        # print("np.dot(W.A, -np.dot(dA,x_k)).shape = ", np.dot(W.A, -np.dot(dA,x_k)).shape)

        # print(W.b - np.dot(W.A,x_kp1) + np.dot(W.A, np.dot(A,x_k) + np.dot(B,u_k) ))

        constraints += [ 
            theta_mat @ np.dot(W.A, -np.dot(dA,x_k)) <= \
            W.b - np.dot(W.A,x_kp1) + np.dot(W.A, np.dot(A,x_k) + np.dot(B,u_k) + K )
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
    K = uad_in.K

    n_WA = W.A.shape[0]

    # Define and solve the optimization problem with CVXPY.
    # Create a symmetric matrix variable.
    theta = cp.Variable((1,))
    theta_mat = cp.Variable((n_WA,n_WA))
    # The operator >> denotes matrix inequality.
    constraints = [ uad_in.Theta.A @ theta <= uad_in.Theta.b ]
    constraints += [ theta_mat == np.eye(n_WA) * theta  ]
    for k in range(t):
        x_k   = np.reshape(x_hist[:,k],newshape=(n_x,1))
        u_k   = np.reshape(u_hist[:,k],newshape=(n_u,1))
        x_kp1 = np.reshape(x_hist[:,k+1],newshape=(n_x,1))

        constraints += [ 
            theta_mat @ np.dot(W.A, -np.dot(dA,x_k)) <= \
            W.b - np.dot(W.A,x_kp1) + np.dot(W.A, np.dot(A,x_k) + np.dot(B,u_k) + K )
        ]

    prob = cp.Problem(cp.Maximize(theta),constraints)
    prob.solve()

    return theta.value

def system_to_ls_matrices1(uad_in,t,x_seq,u_seq)->(np.array,np.array,np.array):
    """
    system_to_ls_matrices1
    Description:
        Performs the least squares estimate on the trajectory data in x_seq and u_seq.
        In this case, this estimator assumes that there is no disturbance and the algorithm seeks
        to find the parameter in the linear system which leads to the least error in the state trajectory.

    """

    # Constants
    A = uad_in.A
    dA = uad_in.dA
    B = uad_in.B

    n_x = uad_in.dim_x()

    # Algorithm

    A_prime = np.zeros(shape=(n_x*t,1))
    for tau in range(0,t):
        A_prime[n_x*tau:n_x*(tau+1),0] = np.dot(dA,x_seq[:,tau])

    #A_tilde = np.hstack( ( A_prime , np.kron( np.eye(n_x) , np.eye(t) ) ) )
    A_tilde = A_prime

    b_tilde = np.zeros(shape=(n_x*t,1))
    for tau in range(0,t):
        x_tau = x_seq[:,tau]
        u_tau = u_seq[:,tau]
        x_tauPOne = x_seq[:,tau+1]
        b_tilde[n_x*tau:n_x*(tau+1),0] = x_tauPOne - np.dot(A,x_tau) - np.dot(B,u_tau)

    return A_tilde, b_tilde

class rls_estimator0:
    """
    rls_estimator0
    Description:

    Usage:
        estim0 = rls_estimator0(theta_hat)
    """

    def __init__(self,system_in, theta_hat_in, x_tm1_in , u_tm1_in , Phi_t0 ):
        """
        __init__
        Description:
            Initializing the estimator.
        Usage:
            estim0 = rls_estimator0(system_in, theta_hat_in, x_tm1_in , u_tm1_in , Phi_t0)
        """

        # Constants

        # Algorithm
        self.theta_hat = theta_hat_in
        
        self.x_tm1 = x_tm1_in
        self.u_tm1 = u_tm1_in

        self.system = system_in

        # Estimator Variables
        self.P = np.linalg.inv( np.dot( Phi_t0.T , Phi_t0 ) )
        self.t = 0

    def update(self,x_t):
        """
        update
        Description:
        
        Usage:
            estim0.update( phi_t )
        """

        # Constants
        x_tm1 = self.x_tm1
        u_tm1 = self.u_tm1

        theta_hat_tm1 = self.theta_hat

        system = self.system
        A = system.A
        dA = system.dA
        B = system.B

        # Update K
        y_t = np.reshape( x_t - np.dot(A,x_tm1) - np.dot(B,u_tm1)  , newshape=(system.dim_x(),1))
        phi_t = np.reshape( np.dot(dA,x_tm1), newshape=(1,system.dim_x()))

        P_tm1 = self.P

        print(np.dot(P_tm1,phi_t))
        temp_inv = np.eye(1) + np.dot(phi_t.T,np.dot(P_tm1,phi_t))
        temp_inv = np.linalg.inv( temp_inv )
        K = np.dot( P_tm1 , np.dot(phi_t , temp_inv ) )

        P_t = np.dot(np.eye(K.shape[0]) - np.dot(K,phi_t.T), P_tm1 )

        print(K)
        print(P_t)

        theta_hat_t = theta_hat_tm1 + np.dot(K,y_t - np.dot(phi_t.T,theta_hat_tm1))

        self.theta_hat = theta_hat_t

        # Update time
        self.t += 1


###############
## Constants ##
###############

T = 5

# Create System
n_x = 3
A1  = np.eye(n_x)
dA1 = np.eye(n_x)
# dA1[0,1] = 0.5
# B1 = np.array([[0.0],[1.0]])
B1 = np.ones(shape=(n_x,1))
# W1 = pc.box2poly( [ [ 0, 1 ] , [0, 1], [0.5,1.5] ] )
W1 = pc.box2poly( [ [ -1, 1 ] , [-1, 1], [-1,1] ] )
W2 = pc.box2poly( [ [ -0.2, 0.2 ] , [-0.2, 0.2], [-0.2,0.2] ] )
# Theta1 = pc.box2poly([ [1.0, 2.0] ])
Theta1 = pc.box2poly([ [-0.7,0.3] ])

suad1 = ScalarUncertainAffineDynamics(A1,dA1,B1,W1,Theta1)
suad2 = ScalarUncertainAffineDynamics(A1,dA1,B1,W2,Theta1)

temp_system = suad2

# x0 = np.array([[1.0],[0.2]])
x0 = np.zeros(shape=(3,1))
x0[0] = 1.0

# Simulate T steps
#theta_star = sample_from_polytope(suad1.Theta)[0][0]
theta_star = 0.1
print('theta_star = ' + str(theta_star) )

x_hist_0_T = np.zeros(shape=( temp_system.dim_x() , T+1 ))
u_hist_0_T = np.ones(shape=(temp_system.dim_u(),T))

x_hist_0_T[:,0] = x0.flatten()
for tau in range(T):
    x_tau = np.reshape( x_hist_0_T[:,tau] , newshape=(temp_system.dim_x(),1) )
    u_tau = np.reshape( u_hist_0_T[:,tau] , newshape=(temp_system.dim_u(),1) )
    x_tauPOne = temp_system.f( x_tau , theta_star , u_tau )
    x_hist_0_T[:,tau+1] = np.reshape(x_tauPOne,newshape=(temp_system.dim_x(),))

print('x_hist_0_T =')
print(x_hist_0_T)

# Run Estimators
estim0 = rls_estimator0(temp_system, -0.2, \
    x_tm1_in=x_hist_0_T[:,0] , u_tm1_in=u_hist_0_T[:,0] , \
    Phi_t0=np.eye( temp_system.dim_x() ) )
    # Phi_t0=np.reshape(np.dot(temp_system.dA,x0),newshape=(1,n_x)) )

for tau in range(1,T):
    print('tau = ',tau)
    print('- bar(mu)_' + str(tau) + ' = ')
    print('- ', estimate_theta_lb( temp_system , x_hist_0_T[:,:tau+1] , u_hist_0_T[:,:tau] ) )
    print('- underline(mu)_' + str(tau) + ' = ')
    print('- ', estimate_theta_ub( temp_system , x_hist_0_T[:,:tau+1] , u_hist_0_T[:,:tau] ) )
    H_t, h_t = system_to_ls_matrices1(temp_system,tau,x_hist_0_T[:,:tau+1],u_hist_0_T[:,:tau])
    print('- mu_LS = ' + str( scipy.linalg.lstsq(H_t,h_t) ))
    estim0.update(x_hist_0_T[:,tau])
    print(estim0.theta_hat)

