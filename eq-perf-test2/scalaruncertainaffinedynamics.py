from multiprocessing.sharedctypes import Value
from random import sample
import numpy as np
import polytope as pc

import unittest

"""
ScalarUncertainAffineDynamics
Description:
    A simple Linear Dynamical system stored in an object.
    Defines the linear system:
        x^+ = (A + \theta^* dA) x + B u + w , w \in W
        y   = C x + v                       , v \in V
    where:
    - x is the state of the system
    - u is the input to the system
    - \theta is the unknown SCALAR parameter in the system.
Members:
    A
    dA

"""
class ScalarUncertainAffineDynamics:

    def __init__(self,A,dA,B,W:pc.Polytope,Theta:pc.Polytope, E=None, K=None,C=None) -> None:
        # Mandatory Matrices
        self.A      = A
        self.dA     = dA
        self.B      = B
        self.W      = W
        self.Theta  = Theta

        # Input Checking
        self.checkA(A)
        self.checkA(dA)
        self.checkB(B)

        if self.Theta.dim != 1:
            raise ValueError("The parameter theta is supposed to be a scalar, but Theta has dimension " + str(self.Theta.dim) )

        n_x = A.shape[0]

        # Optional Matrices
        if E is None:
            self.E = np.eye(n_x) 
        else:   
            self.E=E

        if K is None:
            self.K = np.zeros((n_x,1))
        else:
            self.K=K

        if C is None:
            self.C = np.eye(n_x)
        else:
            self.C = C
        
    def __str__(self) -> str:
        """
        __str__
        Description:
            This is what is produced when asked to convert the object to a string.
        """

        # Constants
        
        # Create String
        temp_output = str(self.dim_x()) + '-Dimensional Affine System with Scalar Uncertain Parameter and ' + str(self.dim_u()) + '-Dimensional Input'
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
        return self.E.shape[1]

    def print_matrices(self):
        print('The dynamical matrices are', '\n')
        print('A_sig = ', self.A, '\n')
        print('B_sig = ', self.B, '\n')
        print('E_sig = ', self.E, '\n')
        print('K_sig = ', self.K, '\n')

    def checkA(self,A):

        if A.ndim != 2:
            raise ValueError("Expected for A to be square matrix, but received a matrix with " + str(A.ndims) + " ndims." )

        if np.size(A,axis=0) != np.size(A,axis=1):
            raise ValueError("Input matrix A is not square! size = " + str(np.size(A)) )

    def checkB(self,B):
        """
        checkB
        Description:
            Checks to make sure that the dimensions of the B matrix are correct.
        """
        if len(B.shape) > 2:
            raise ValueError("Expected for B to be a matrix or vector, not a tensor.")

        if len(B.shape) == 1:
            if B.shape[0] != self.dim_x():
                raise ValueError("Input vector B is supposed to have the same dimension as the state (" + str(self.dim_x()) + "), but received vector of length " + str(len(self.B)) + "." )
            
        if len(B.shape) == 2:
            if B.shape[0] != self.dim_x():
                raise ValueError("Input matrix B is supposed to have the same dimension as the state (" + str(self.dim_x()) + "), but received vector of length " + str(len(self.B)) + "." )


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
        n_x = A.shape[0]

        B = self.B
        n_u = B.shape[1]

        E = self.E
        n_w = E.shape[1]

        K = self.K

        # Create the MPC Matrices (S_w)
        S_w = np.zeros((T*n_x,T*n_w))
        E_prefactor = np.zeros((T*n_x,T*n_x))
        for j in range(T):
            for i in range(j,T):
                E_prefactor[i*n_x:(i+1)*n_x, j*n_x:(j+1)*n_x]=np.linalg.matrix_power(A,i-j)

        S_w = np.dot(E_prefactor , np.kron(np.eye(T),E))

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
        S_K = np.dot(E_prefactor,S_K)

        return S_w, S_u, S_x0, S_K

    def f(self,x,theta,u,w=np.array([])):
        """
        f
        Description:
            This function computes the linear update of the system from the current state.
        """

        # Constants
        A = self.A
        dA = self.dA
        B = self.B
        K = self.K
        E = self.E

        w_is_empty = (w.shape[0] == 0)

        # Input Processing
        if w_is_empty:
            w = np.reshape(sample_from_polytope(self.W),newshape=(self.dim_w(),1))

        if (len(w.shape) > 1) and (len(w.shape) == 0):
            raise Exception('Expected w to be an array of one dimension only')

        if w.shape[0] != self.dim_w():
                raise Exception('The provided disturbance was of dimension ' + str(w.shape[0]) + ' but the dimension of disturbances in this affine dynamics is ' + str(self.dim_w()) + '.' )

        print(w)

        print( np.dot(A + dA*theta,x).shape )
        print( np.dot(A + dA*theta,x) )
        print( np.dot(B,u).shape )
        print(E.shape)
        print( np.dot(E,w).shape )

        return np.dot(A + dA*theta,x) + np.dot(B,u) + np.dot(E,w) + K


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
    n_V = V.shape[0]

    # Random Variable
    comb_rand_var = np.random.exponential( size=(n_V,N_samples) )
    for sample_index in range(N_samples):
        comb_rand_var[:,sample_index] = comb_rand_var[:,sample_index] / np.sum(comb_rand_var[:,sample_index])

    return np.dot(V.T , comb_rand_var )

    


class TestScalarUncertainAffineDynamics(unittest.TestCase):
    """
    Description:
        Tests the ScalarUncertainAffineDynamics object.
    """
    def test_construct1(self):
        try:
            A  = np.zeros(shape=(3,2))
            dA = np.eye(3)
            B  = dA
            W = pc.box2poly( [ [ 1.0, 2.0 ] , [3.0, 4.0] ] )
            ts0 = ScalarUncertainAffineDynamics(A,dA,B,W)
            self.assertTrue(False)
        except ValueError:
            self.assertTrue(True)
        else:
            self.assertTrue(False)

    def test_sample_polytope1(self):
        """
        test_sample_polytope1
        Description:
            Samples one point from a simple box polytope.
        """
        P1 = pc.box2poly( [ [ 1.0, 2.0 ] , [3.0, 4.0] ] )

        s = get_N_samples_from_polytope(P1,1)

        self.assertTrue( s in P1 ) #Verify that the point is in the polytope

    def test_sample_polytope2(self):
        """
        test_sample_polytope2
        Description:
            Samples MANY points from a simple box polytope.
        """

        P2 = pc.box2poly( [ [ 1.0, 2.0 ] , [3.0, 4.0] ] )

        samples = get_N_samples_from_polytope( P2 , 20 )

        for sample_index in range(samples.shape[1]):
            sample = samples[:,sample_index]
            self.assertTrue( sample in P2 )

    def test_f1(self):
        """
        test_f1
        Description:
            Tests that the dynamics satisfy
        """

        # Constants
        n_x = 2
        A1  = np.eye(2)
        dA1 = np.zeros(shape=(2,2))
        dA1[0,1] = 1
        B1 = np.array([[0.0],[1.0]])
        W1 = pc.box2poly( [ [ 1.0, 2.0 ] , [3.0, 4.0] ] )

        # Algorithm
        uad1 = ScalarUncertainAffineDynamics(A1,dA1,B1,W1)
        x0 = np.array([[1.0],[-1.0]])
        u0 = np.array([[0.25]])
        theta_star = 1.5

        x1 = uad1.f( x0 , theta_star , u0 )

        self.assertTrue( (x1 - np.dot(uad1.A + theta_star * uad1.dA,x0) - np.dot(uad1.B,u0) ) in uad1.W )
