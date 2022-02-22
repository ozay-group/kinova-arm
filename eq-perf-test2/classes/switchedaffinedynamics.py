from argparse import ArgumentError
from classes.language import Language
from classes.affinedynamics import AffineDynamics

import numpy as np
import scipy

"""
SwitchedAffineDynamics
Description:
    A dynamical system whose state evolves according to a different linear
    system
        x^+ = A_m x + B_m u + w + K_m   , w \in W_m
        y   = C_m x + v                 , v \in V_m
    at each time dependent on the current mode m.
"""
class SwitchedAffineDynamics:

    def __init__(self,affine_dynamics_list,L:Language=None) -> None:
        # Input Checking
        self.check_dynamics(affine_dynamics_list)

        # Mandatory Values
        self.Dynamics=affine_dynamics_list
        self.L = L
        
    def __str__(self) -> str:
        """
        __str__
        Description:
            This is what is produced when asked to convert the object to a string.
        """

        # Constants
        n_x = self.dim_x()

        temp_output = str(n_x) + '-Dimensional Switched Affine System with ' + str( self.n_modes() ) + ' Modes'
        return temp_output

    def dim_x(self) -> int:
        """
        dim_x
        Description:
            Returns the dimension of the state x.
        """
        return self.Dynamics[0].dim_x()

    def dim_u(self) -> int:
        """
        dim_u
        Description:
            Returns the dimension of the input u.
        Assumption:
            Assumes that all of the systems in the Dynamics list have the same input dimension.
        """
        return self.Dynamics[0].dim_u()

    def dim_w(self) -> int:
        """
        dim_w
        Description:
            Returns the dimension of the disturbance w.
        """
        return self.Dynamics[0].dim_w()

    def dim_y(self) -> int:
        """
        dim_y
        Description:
            Returns the dimension of the output y.
        """
        return self.Dynamics[0].dim_y()

    def dimensions(self) -> (int,int,int,int,int):
        """
        dimensions()
        Description:
            Returns the dimensions of all of the relevant variables influencing the system's dynamics.
        """
        return self.Dynamics[0].dimensions()

    def n_modes(self) -> int:
        return len(self.Dynamics)

    def check_dynamics(self,affine_dynamics_list):
        """
        check_dynamics
        Description:
            Creating
        """

        # Check that the x dimension has the same size
        n_x0 = affine_dynamics_list[0].A.shape[0]
        for aff_dyn0 in affine_dynamics_list:
            if n_x0 != aff_dyn0.A.shape[0]:
                raise ArgumentError('There was an issue with affine dynamics with dimension ' + str(aff_dyn0.A.shape[0]) + '; expected ' + str(n_x0))

    def get_mpc_matrices(self,word=None):
        """
        get_mpc_matrices
        Description:
            Get the mpc_matrices for the discrete-time dynamical system described by self.
        Assumes:
            Assumes word is list of integers where each integer is a mode of the switched affine dynamics.
        Usage:
            S_w, S_u, S_x0, S_K = sad.get_mpc_matrices(word_in)
        """

        # Input Processing
        if len(word)==0: #if word is empty array then there should be an issue
            raise ArgumentError('word should be an array of integers; received ' + str(word))

        if any(word < 0) or any(word >= self.n_modes()):
            raise ArgumentError('There is a mode in word ' + str(word) + ' that does not fit into the expected range for this LCSAS [0,' + str(self.n_modes) + ').' )

        # Constants
        n_x = self.dim_x()
        n_u = self.dim_u()
        n_w = self.dim_w()

        T = len(word)

        # Create the MPC Matrices (S_w)
        S_w = np.zeros((T*n_x,T*n_w))
        E_prefactor = np.zeros((T*n_x,T*n_x))
        for i in range(T):
            if i == 0:
                nonzero_part = np.eye(n_x)
            else:
                nonzero_part = np.block([ np.dot( self.Dynamics[ word[i] ].A, nonzero_part ) ,np.eye(n_x) ])
            
            E_prefactor[i*n_x:(i+1)*n_x,:(i+1)*n_x]=nonzero_part

        E_tuple = ()
        for i in range(T):
            E_tuple += ( self.Dynamics[ word[i] ].E,)

        blockE = scipy.linalg.block_diag(*(E_tuple))
        S_w = np.dot( E_prefactor , blockE )

        # Create the MPC Matrices (S_u)
        S_u = np.zeros((T*n_x,T*n_u))

        B_tuple = ()
        for i in range(T):
            B_tuple += ( self.Dynamics[ word[i] ].B, )
        blockB = scipy.linalg.block_diag(*(B_tuple))

        S_u = np.dot( E_prefactor , blockB )

        # Create the MPC Matrices (S_x0)
        S_x0 = np.zeros((T*n_x,n_x))
        for i in range(T):
            if i == 0:
                S_x0[i*n_x:(i+1)*n_x,:] = self.Dynamics[ word[i] ].A
            else:
                S_x0[i*n_x:(i+1)*n_x,:]= np.dot( self.Dynamics[ word[i] ].A , S_x0[(i-1)*n_x:i*n_x,:] )

        # Create the MPC Matrices (S_K)
        S_K = np.zeros(shape=(n_x*T,1))
        for i in range(T):
            S_K[i*n_x:(i+1)*n_x,:] = self.Dynamics[ word[i] ].K
        S_K = np.dot(E_prefactor,S_K)

        return S_w, S_u, S_x0, S_K

    def f(self,x,u,m,flags=[]):
        """
        f
        Description:
            This function computes the linear update of the system from the current state.
        """

        if 'no_w' == flags:
            # Simulate System with No disturbance w
            return (np.dot(self.A,x) + np.dot(self.B, u) + self.K.T).T
        else:
            raise NotImplementedError("Warning this part of f() has not been implemented yet!")

class TestSwitchedAffineDynamics(unittest.TestCase):
    """
    TestSwitchedAffineDynamics
    Description:
        Tests the SwitchedAffineDynamics object.
    """
    def test_construct1(self):
        try:
            ts0 = AffineDynamics(np.zeros((3,2)),np.eye(3))
            self.assertTrue(False)
        except ValueError:
            self.assertTrue(True)
        else:
            self.assertTrue(False)