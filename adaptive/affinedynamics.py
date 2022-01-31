import numpy as np
import polytope as pc

import unittest

"""
AffineDynamics
Description:
    A simple Linear Dynamical system stored in an object.
    Defines the linear system:
        x^+ = A x + B u + w , w \in W
            y = C x + v       , v \in V

"""
class AffineDynamics:

    def __init__(self,A=np.zeros((0,0)),B=np.zeros((0,0)),E=None, K=None,C=None) -> None:
        # Input Checking
        self.checkA(A)

        # Mandatory Matrices
        self.A=A
        self.B=B

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
        n_x = self.A.shape[0]

        temp_output = str(n_x) + '-Dimensional Affine System'
        return temp_output

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

    def get_mpc_matrices(self,T=-1):
        """
        get_mpc_matrices
        Description:
            Get the mpc_matrices for the discrete-time dynamical system described by self.
        Assumes:
            Assumes T is an integer input
        """

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
        for j in range(T):
            for i in range(j,T):
                S_w[i*n_x:(i+1)*n_x, j*n_x:(j+1)*n_x]=np.dot(np.linalg.matrix_power(A,i-j),E)

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
        S_K=np.array([K for i in range(T)])
        S_K=S_K.reshape(np.shape(S_K)[0]*np.shape(S_K)[1],1)

        return S_w, S_u, S_x0, S_K


class TestAffineDynamics(unittest.TestCase):
    """
    Description:
        Tests the AffineDynamics object.
    """
    def test_construct1(self):
        try:
            ts0 = AffineDynamics(np.zeros((3,2)),np.eye(3))
            self.assertTrue(False)
        except ValueError:
            self.assertTrue(True)
        else:
            self.assertTrue(False)