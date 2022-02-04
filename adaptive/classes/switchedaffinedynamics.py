from argparse import ArgumentError
from classes.language import Language
from classes.affinedynamics import AffineDynamics

import numpy as np

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
        return self.Dynamics[0].A.shape[0]

    def dim_w(self) -> int:
        """
        dim_w
        Description:
            Returns the dimension of the disturbance w.
        """
        return self.Dynamics[0].E.shape[1]

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
            Assumes T is an integer input
        Usage:
            S_w, S_u, S_x0, S_K = ad0.get_mpc_matrices(T)
        """

        # Input Processing
        if not word:
            raise DomainError("word should be an array of integers; received " + str(word))

        # Constants
        n_x = self.n_x()
        n_w = self.n_w()

        T = len(word)

        # Create the MPC Matrices (S_w)
        S_w = np.zeros((T*n_x,T*n_w))
        E_prefactor = np.zeros((T*n_x,T*n_x))
        for i in range(T):
            if i == 0:
                nonzero_part = np.eye(n_x)
            else:
                nonzero_part = np.block([ np.dot( self.Dynamics[word[i]], nonzero_part ) ,np.eye(n_x) ])
            
            E_prefactor[i*n_x:(i+1)*n_x,:(i+1)*n_x]=nonzero_part

        E_stacked = np.zeros(shape=(n_w*T))
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