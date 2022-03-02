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


