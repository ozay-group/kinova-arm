"""
consistentbeliefcontroller
"""
import unittest
import polytope as pc
import numpy as np

from classes.switchedaffinedynamics import SwitchedAffineDynamics

class ConsistentBeliefController:
    """
    ConsistentBeliefController
    Description:
        Controller which executes the control from the Correct-By-Construction 
    """

    """
    __init__
    Description:
        
    """
    def __init__(self , system, knowledge_sequences, K_set, k_set, A_set, b_set, Ae_set, be_set, options_in=None) -> None:

        # Input Processing

        # verify that system is of type dyn
        if isinstance(system,SwitchedAffineDynamics):
            raise TypeError("The input 'system' to the controller must be a Dyn() object.")
        
        # Algorithm

        self.system = system
        self.knowledge_sequences = knowledge_sequences

        if options_in is None:
            self.settings = CBCSettings()
        else:
            if isinstance(options_in,CBCSettings):
                raise TypeError("The input 'options_in' must be of class CBCSettings().")

            self.settings = options_in

        self.external_behavior_sets = []
        for row_index in range(len(self.knowledge_sequences)):
            temp_row_of_sets = []
            for col_index in range(len(self.knowledge_sequences[0])):
                temp_row_of_sets = append(temp_row_of_sets,pc.polytope.Polytope(
                    np.vstack((A_set[row_index][col_index],Ae_set[row_index][col_index],-Ae_set[row_index][col_index])),
                    np.vstack((b_set[row_index][col_index],be_set[row_index][col_index],-be_set[row_index][col_index]))
                    ))
            self.external_behavior_sets = append(self.external_behavior_sets,temp_row_of_sets)

        # Assign values of the memory variables
        self.u_hist = []
        self.x_hist = []


    """
    num_sequeences
    Description:
        Returns the number of Knowledge Sequences used to define this controller.
    """
    def num_sequences(self)->int:
        return len(self.knowledge_sequences)

    """
    time_horizon
    Description:
        Returns the number of time steps into the future that this controller attempts to make guarantees for.
    """
    def time_horizon(self)->int:
        return len(self.knowledge_sequences[0][0].words[0])

class CBCSettings:
    """
    __init__
    Description:
        The constructor for this class.
    """
    def __init__(self,feedback_method='Disturbance (State)') -> None:
        self.feedback_method = feedback_method

        

class TestConsistentBeliefController(unittest.TestCase):
    """
    test_construct1
    Description:
        Test the empty language construction.
    """
    def test_construct1(self):
        ts0 = CBCSettings()
        self.assertEqual(ts0.feedback_method,'Disturbance (State)')


    

if __name__ == '__main__':
    unittest.main()
