import numpy as np
import polytope as pc

import cvxpy as cp

import unittest

# from sympy import DomainError

"""
AffineDynamics
Description:
    A simple Linear Dynamical system stored in an object.
    Defines the linear system:
        x^+ = A x + B u + B_w w + K , w \in W
        y   = C x + C_v v           , v \in V

"""
class AffineDynamics:

    def __init__(self,A,B,W:pc.Polytope,B_w=None, K=None,C=None,C_v=None, V:pc.Polytope=None) -> None:
        # Input Checking

        # Mandatory Matrices
        self.A=A
        self.B=B
        self.W=W

        n_x = A.shape[0]

        # Optional Matrices
        if B_w is None:
            self.B_w = np.eye(n_x) 
        else:   
            self.B_w = B_w

        if K is None:
            self.K = np.zeros((n_x,))
        else:
            self.K=K

        if C is None:
            self.C = np.eye(n_x)
        else:
            self.C = C

        if C_v is None:
            self.C_v = np.eye( self.C.shape[0] )
        else:
            self.C_v = C_v

        if V is None:
            self.V = pc.box2poly([ [-1.0,1.0] for i in range(self.C_v.shape[1]) ])
        else:
            self.V = V

        # Check Matrices
        self.checkA()
        
        if self.A.shape[0] != self.B.shape[0]:
            raise Exception("The dimension of the state according to A was " + str(self.A.shape[0]) + ", but according to B it is " + str(self.B.shape[0]))

        
    def __str__(self) -> str:
        """
        __str__
        Description:
            This is what is produced when asked to convert the object to a string.
        """

        # Constants
        n_x = self.A.shape[0]

        temp_output = str(n_x) + '-Dimensional Affine System'
        return temp_output

    def dim_x(self) -> int:
        """
        dim_x
        Description:
            Returns the dimension of the state.
        """
        return self.A.shape[0]

    def dim_u(self) -> int:
        """
        n_u
        Description:
            Returns the dimension of the input to the affine dynamics.
        """
        return self.B.shape[1]

    def dim_w(self) -> int:
        """
        n_w
        Description:
            Returns the dimension of the process disturbance to the affine dynamics.
        """
        return self.B_w.shape[1]

    def dim_y(self) -> int:
        """
        dim_y
        Description:
            Returns the dimension of the output of the affine dynamics.
        """

        return self.C.shape[0]

    def dim_v(self) -> int:
        """
        dim_v
        Description:
            Returns the dimension of the measurement/output noise in the affine dynamics.
        """

        return self.C_v.shape[1]

    def dimensions(self) -> (int,int,int,int,int):
        """
        dimensions
        Description:
            Returns all of the relevant dimensions for the dynamical system.
        Usage:
            n_x, n_u, n_y, n_w, n_v = ad0.dimensions()
        """

        return self.dim_x(), self.dim_u(), self.dim_y(), self.dim_w(), self.dim_v()

    def print_matrices(self):
        print('The dynamical matrices are', '\n')
        print('A_sig = ', self.A, '\n')
        print('B_sig = ', self.B, '\n')
        print('E_sig = ', self.E, '\n')
        print('K_sig = ', self.K, '\n')

    def checkA(self):
        """
        checkA
        Description:
            Checks to see if the dimensions of A match what we expect.
        Usage:
            ad0.checkA()
        """

        A = self.A

        if A.ndim != 2:
            raise ValueError("Expected for A to be square matrix, but received a matrix with " + str(A.ndims) + " ndims." )

        if np.size(A,axis=0) != np.size(A,axis=1):
            raise ValueError("Input matrix A is not square! size = " + str(np.size(A)) )

    def get_mpc_matrices(self,T=-1):
        """
        get_mpc_matrices
        Description:
            Get the mpc_matrices for the discrete-time dynamical system described by self.
        Assumes:
            Assumes T is an integer input
        Usage:
            S_w, S_u, S_x0, S_K = ad0.get_mpc_matrices(T)
        """

        # Input Processing
        if T < 0:
            raise DomainError("T should be a positive integer; received " + str(T))

        # Constants
        A = self.A
        B = self.B
        B_w = self.B_w

        n_x, n_u, n_y, n_w, n_v = self.dimensions()

        K = self.K

        # Create the MPC Matrices (S_w)
        S_w = np.zeros((T*n_x,T*n_w))
        Bw_prefactor = np.zeros((T*n_x,T*n_x))
        for j in range(T):
            for i in range(j,T):
                Bw_prefactor[i*n_x:(i+1)*n_x, j*n_x:(j+1)*n_x]=np.linalg.matrix_power(A,i-j)

        S_w = np.dot(Bw_prefactor , np.kron(np.eye(T),E))

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
        S_K = np.kron(np.ones((T,1)),K)
        S_K = np.dot(Bw_prefactor,S_K)

        return S_w, S_u, S_x0, S_K

    def f(self,x,u,flags=[]):
        """
        f
        Description:
            This function computes the linear update of the system from the current state.
        """
        if flags == []:
            w = np.reshape(sample_from_polytope(self.W),newshape=(self.dim_w(),))
            return np.dot(self.A,x) + np.dot(self.B, u) + self.K + w
        if 'no_w' == flags:
            # Simulate System with No disturbance w
            return (np.dot(self.A,x) + np.dot(self.B, u) + self.K.T).T
        else:
            raise NotImplementedError("Warning this part of f() has not been implemented yet!")

    def reconstruct_w(self,x_t,x_tp1,u_t):
        """
        reconstruct_w
        Description:
            Finds a disturbance which could have been used to create the state x_tp1 given the previous state x_t and input u_t.
        """

        # Constants
        A = self.A
        B = self.B
        B_w = self.B_w
        K , W = self.K, self.W

        n_x, n_u, n_y, n_w, n_v = self.dimensions()

        # Algorithm
        
        # Define and solve the CVXPY problem.
        # Create a symmetric matrix variable.
        w = cp.Variable((n_w,))
        # The operator >> denotes matrix inequality.
        constraints = [ W.A @ w <= W.b]
        constraints += [
            x_tp1 == np.dot(A,x_t) + np.dot(B,u_t) + B_w @ w + K
        ]
        prob = cp.Problem(cp.Minimize(1),constraints)
        prob.solve()

        return w.value, prob.status


def sample_from_polytope(P:pc.Polytope):
    """
    sample_from_polytope
    Description:
        Produces a single sample from the polytope defined by P.
    """
    return get_N_samples_from_polytope(P,1)

def get_N_samples_from_polytope(P:pc.Polytope,N_samples):
    """
    get_N_samples_from_polytope
    Description:
        This function retrieves N samples from the polytope P.
        Used to more efficiently produce samples (only have to compute extremes once.)
    """

    # Compute V Representation
    V = pc.extreme(P)
    
    # print(V is None)
    if V is None:
        # I suspect this means that the polytope contains a singleton.
        # Select the element at the boundary to get the correct sample.
        return P.b[:P.dim].reshape((P.dim,1))

    n_V = V.shape[0]

    # Random Variable
    comb_rand_var = np.random.exponential( size=(n_V,N_samples) )
    for sample_index in range(N_samples):
        comb_rand_var[:,sample_index] = comb_rand_var[:,sample_index] / np.sum(comb_rand_var[:,sample_index])

    return np.dot(V.T , comb_rand_var )

class TestAffineDynamics(unittest.TestCase):
    """
    Description:
        Tests the AffineDynamics object.
    """
    def test_construct1(self):
        try:
            W0 = pc.box2poly([ [-1,1], [-1,1] ])
            ts0 = AffineDynamics(np.zeros((3,2)),np.eye(3),W0)
            self.assertTrue(False)
        except ValueError:
            self.assertTrue(True)
        else:
            self.assertTrue(False)


    def test_dim_y1(self):
        """
        test_dim_y1
        Description:
            This test verifies that the system correctly identifies when a two-dimensional system
            has a one-dimensional output.
        """
        W0 = pc.box2poly([ [-1,1], [-1,1] ])
        ad0 = AffineDynamics(np.eye(2),np.ones(shape=(2,1)),W0,B_w=np.eye(2),C=np.ones(shape=(1,2)))

        self.assertTrue( ad0.dim_y() == 1 )

    def test_dim_y2(self):
        """
        test_dim_y2
        Description:
            This test verifies that the system correctly identifies when a two-dimensional system
            has a three-dimensional output.
        """
        W0 = pc.box2poly([ [-1,1], [-1,1] ])
        ad0 = AffineDynamics(np.eye(2),np.ones(shape=(2,1)),W0,B_w=np.eye(2),C=np.ones(shape=(3,2)))

        self.assertTrue( ad0.dim_y() == 3 )

    def test_dim_v1(self):
        """
        test_dim_v1
        Description:
            This test verifies that the function which returns the dimension of the output noise
            works for a 2-D system with 1-D output and unspecified C_v matrix. (Should be 1)
        """
        W0 = pc.box2poly([ [-1,1], [-1,1] ])
        ad0 = AffineDynamics(np.eye(2),np.ones(shape=(2,1)),W0,B_w=np.eye(2),C=np.ones(shape=(1,2)))

        self.assertTrue( ad0.dim_v() == 1 )

    def test_dim_v2(self):
        """
        test_dim_v2
        Description:
            This test verifies that the function which returns the dimension of the output noise
            works for a 2-D system with 3-D output and SPECIFIED C_v matrix. (Should be 2)
        """
        W0 = pc.box2poly([ [-1,1], [-1,1] ])
        ad0 = AffineDynamics(np.eye(2),np.ones(shape=(2,1)),W0,B_w=np.eye(2),C=np.ones(shape=(3,2)),C_v=np.ones(shape=(3,2)))

        self.assertTrue( ad0.dim_v() == 2 )

    def test_dimensions1(self):
        """
        test_dimensions1
        Description:
            This test verifies that the function which returns the correct dimension of the output noise
            works for a 2-D system with 3-D output and SPECIFIED C_v matrix. (Should be 2)
        """
        W0 = pc.box2poly([ [-1,1], [-1,1] ])
        ad0 = AffineDynamics(np.eye(2),np.ones(shape=(2,1)),W0,B_w=np.eye(2),C=np.ones(shape=(3,2)),C_v=np.ones(shape=(3,2)))
        n_x, n_u, n_y, n_w, n_v = ad0.dimensions()

        self.assertTrue( n_v == 2 )
        self.assertTrue( n_x == 2 )
        self.assertTrue( n_u == 1 )
        self.assertTrue( n_y == 3 )
        self.assertTrue( n_w == 2 )

    def test_reconstruct_w1(self):
        """
        test_reconstruct_w1
        Description:
            Attempts to reconstruct a very simple disturbance
        """
        W0 = pc.box2poly([ [-1,1], [-1,1] ])
        ad0 = AffineDynamics(np.eye(2),np.ones(shape=(2,1)),W0,B_w=np.eye(2),C=np.ones(shape=(3,2)),C_v=np.ones(shape=(3,2)))
        x0 = np.zeros((2,)) #np.array([[1.0],[2.0]])
        u0 = np.zeros((1,))
        x1 = ad0.f(x0,u0)

        w_reconstr, good = ad0.reconstruct_w(x0,x1,u0)
        self.assertTrue( np.allclose(w_reconstr , x1) )
        
        

