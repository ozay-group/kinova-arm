"""
drone_test5.py
Description:
    In this test, we will observe how the MPC-based controller works.
"""

# Import
import numpy as np
import affinedynamics as ad
import gurobipy as gp
from gurobipy import GRB
import scipy

import polytope as pc
import qpsolvers
# Define Simple Discrete-Time Linear System
dim_x = 2
A = 1*np.eye(dim_x)
#A[0,1] = 0.2

# dim_u = 1
# B = np.array([[0.0],[1.0]])
dim_u = 2
B = np.eye(dim_u)

K = np.array([[1.0],[0.0]])

affine_system1 = ad.AffineDynamics(A=A,B=B,K=K)

x0 = np.zeros((2,1))

print(affine_system1)
affine_system1.print_matrices()

# Define Time Horizon 
T = 5

"""
Create MPC problem and try to solve with gurobi
"""

# Create state quadratic cost (Q) and input quadratic cost R
P = np.eye(dim_x)
P_T = P
R = 0.001*np.eye(dim_u)
#R = np.zeros((dim_u,dim_u))

# Create MPC Matrices
S_w, S_u, S_x0, S_K = affine_system1.get_mpc_matrices(T)

print('S_K = ', S_K)
print('S_x0 = ', S_x0)
print('S_u = ',S_u) 

# Create Extended Cost Matrices
P_barre = scipy.linalg.block_diag( np.kron(np.eye(T-1,dtype=float),P) , P_T )
R_barre = np.kron( np.eye(T,dtype=float) , R )

Q = np.dot(S_u.T,np.dot(P_barre , S_u)) + R_barre
# q = np.zeros((dim_u*T,))
x_target = np.matrix([[1.0],[2.0]])
target_barre = np.kron( np.ones((T,1)) , x_target )
q = 2.0 * np.dot( (np.dot(S_x0,x0) + S_K - target_barre).T , np.dot(P_barre,S_u) )

# Input Constraints
umin = -10
umax = 10
U = pc.box2poly([ [umin,umax] for i in range(0,dim_u*T) ])

try:
    m = gp.Model("MPC1")
    u = m.addMVar(shape=(dim_u*T,),lb=-float('inf'),vtype=GRB.CONTINUOUS)

    # Create Objective
    m.setObjective( u @ Q @ u + q @ u , GRB.MINIMIZE )
    # m.setObjective( u @ Q @ u + q @ u + np.dot( (np.dot(S_x0,x0) + S_K - target_barre).T , np.dot(P_barre,(np.dot(S_x0,x0) + S_K - target_barre)) ) , GRB.MINIMIZE )

    # Add constraints
    m.addConstr(U.A @ u <= U.b)

    # Optimize model
    m.optimize()

    print(u.X)
    print('Obj: %g' % m.ObjVal)

    # Use Identified u to see if system reaches target
    u_star = u.X
    x = np.zeros((dim_x,T+1))
    x[:,0] = x0.flatten()
    for t in range(T):
        #print(affine_system1.f(x[:,t].flatten(),u_star[t*dim_u:(t+1)*dim_u],flags="no_w"))
        x[:,t+1] = affine_system1.f(x[:,t].flatten(),u_star[t*dim_u:(t+1)*dim_u],flags="no_w").flatten()

    print( "x[:,-1] = " , x[:,-1] )
    
    print( "x_traj (according to mpc matrices) = " , np.dot(S_u,np.matrix(u_star).T) + np.dot(S_x0,x0) + S_K  )

    print(Q.shape)
    print(q.shape)
    print(U.A.shape)
    print(U.b.shape)

    res = qpsolvers.solve_qp( Q , q.T , U.A , U.b , verbose=True , sym_proj=True )
    print(res)

    u2 = res[0]
    print("x_traj (according to solve_qp) = ", np.dot( S_u , np.matrix(u2).T) + np.dot(S_x0,x0) + S_K )

except gp.GurobiError as e:
    print("Error code " + str(e.errno) + ": " + str(e) ) 

except AttributeError:
    print('Encountered an attribute error!')