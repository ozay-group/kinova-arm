from classes.switchedaffinedynamics import SwitchedAffineDynamics, get_test_sad1
from classes.knowledgesequence import KnowledgeSequence

import unittest
import numpy as np
import polytope as pc

class InternalBehaviorSet:
    """
    InternalBehaviorSet
    Description:
        This object
    """
    def __init__(self,system:SwitchedAffineDynamics,m:KnowledgeSequence,A,Ae,b,be,flags={},K=None,k=None):
        """
        __init__
        Description:
            Creates the reachable behavior set object using a construction of the polytope matrices
            which is based on the MPC-like construction coming from the system.
        """
        
        # Input Processing

        self.System = system
        self.KnowledgeSequence = m
        self.K = K
        self.k = k

        self.settings = {
            'is_closed_loop': True
        }
        self.settings['is_closed_loop'] = (K is None) or (k is None)

        self.check_system()
        self.check_knowledge_sequence()

        print('ibs shape of A = ' + str(A.shape))
        print('ibs shape of Ae = ' + str(Ae.shape))
        self.as_polytope = pc.Polytope(
            np.vstack( (A,Ae,-Ae) ), np.vstack( (b,be,-be) )
        )

        # Extract Time Horizon from the value of m.
        t = m.time_horizon() - 1
        LargestLangInSequence = m.sequence[0]
        T = len(system.L.words[0])

    def check_system(self):
        """
        check_system
        Description:
            Checks the SwitchedAffineDynamics object which is stored in self.System
        """
        self.System.check_dynamics()

    def check_knowledge_sequence(self):
        """
        check_knowledge_sequence
        Description:
            This function verifies that the knowledge sequence:
            - Is non-increasing in magnitude.
        """
        
        # Constants
        ks = self.KnowledgeSequence
        T  = ks.time_horizon()

        # Check to see if 
        for tau in range(T-1):
            L_tau = ks.sequence[tau]
            L_tau_p_one = ks.sequence[tau+1]

            if not( L_tau >= L_tau_p_one ):
                raise Exception("The input sequence has an INCREASING step. (It should be strictly non-increasing in terms of set inclusion.)")


    def has_associated_external_behavior(self,eb_in) -> (bool,np.array):
        """
        has_associated_external_behavior
        Description:
            This function receives an input external behavior and returns true if there exists an internal behavior (remember self only contains internal
            behaviors) that matches the entries of eb_in.
        """

        # Constants
        system = self.System
        L = system.L
        n_x, n_u, n_y, n_w, n_v = system.dimensions()

        ks = self.KnowledgeSequence
        t  = ks.time_horizon() - 1

        x_vec = eb_in[:(t+1)*n_x,:]
        u_vec = eb_in[(t+1)*n_x:,:]

        #Algorithm
        w_mat = np.zeros(shape=(n_w,t*L.cardinality()))
        for word_index in range(L.cardinality()):
            temp_word = L.words[word_index]

            print(temp_word)

            for tau in range(t):
                L_t = ks.sequence[tau+1]

                if L_t.contains(temp_word):

                    # Reconstruct the disturbance at time tau
                    x_tau = np.reshape(x_vec[tau*n_x:(tau+1)*n_x,:],newshape=(n_x,),order='F')
                    x_tau_p_one = np.reshape(x_vec[(tau+1)*n_x:(tau+2)*n_x,:],newshape=(n_x,),order='F')
                    u_tau = np.reshape(u_vec[tau*n_u:(tau+1)*n_u],newshape=(n_u,),order='F')

                    mode_tau_index = temp_word[tau]
                    mode_tau = system.Dynamics[mode_tau_index]

                    w_tau, status_tau = mode_tau.reconstruct_w(x_tau,x_tau_p_one,u_tau)
                    if status_tau != 'optimal':
                        return False, np.array([])

                    w_mat[:,tau+word_index*t] = w_tau

                # else:
                #     w_mat[:,tau+word_index*n_w*t] = np.zeros(shape=(n_w,))

        # If we can reconstruct all w's then return True, and return a valid ib
        w_vec = np.reshape(w_mat,newshape=(n_w*t*L.cardinality(),1),order='F')
        print('internal_behavior is:')
        # print(x_vec)
        # print(u_vec)
        # print(w_mat)
        temp_ib = np.vstack( (x_vec, u_vec, w_vec , x_vec[:n_x] ) )
        print(temp_ib)

        return True, np.vstack( \
            (x_vec, u_vec, w_vec , x_vec[:n_x] ) \
        )


class TestInternalBehaviorSet(unittest.TestCase):
    """
    TestInternalBehaviorSet
    Description:
        A class containing all tests for the InternalBehaviorSet object.
    """

    def test_constructor1(self):
        """
        test_constructor1
        Description:
            Tests the basic version of the constructor.
        """
        try:
            sad1 = get_test_sad1()
            m = KnowledgeSequence([ sad1.L, sad1.L, sad1.L ])

            ibs1 = InternalBehaviorSet(sad1,m)

            self.assertTrue(True)
        except:
            self.assertFalse(True)
    

class ReachableBehaviorSet:
    """
    """


