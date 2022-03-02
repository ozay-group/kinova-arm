"""
consistentbeliefcontroller
"""
import unittest
import polytope as pc
import numpy as np
import scipy.io as sio

from classes.affinedynamics import AffineDynamics
from classes.switchedaffinedynamics import SwitchedAffineDynamics
from classes.knowledgesequence import KnowledgeSequence
from classes.language import Language
from classes.reachablebehaviorset import InternalBehaviorSet

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

def matfile_data_to_cbc( matfile_name:str ):
    """
    matfile_data_to_cbc
    Description:
        This function loads data from matfile_name and then extracts the static data into proper 
    """

    # Constants
    
    ## Algorithm  
       
    matlab_data = sio.loadmat(matfile_name)

    # Extract Data for The Switched Affine System
    num_dynamic_modes = np.prod(matlab_data['lcsas_A_matrices'].shape)
    mode_list = []
    for mode_index in range(num_dynamic_modes):
        temp_A = matlab_data['lcsas_A_matrices'][0,mode_index]
        temp_B = matlab_data['lcsas_B_matrices'][0,mode_index]
        temp_K = matlab_data['lcsas_K_matrices'][0,mode_index]

        temp_W_A = np.vstack( (matlab_data['lcsas_W_A_matrices'][0,mode_index],matlab_data['lcsas_W_Ae_matrices'][0,mode_index],-matlab_data['lcsas_W_Ae_matrices'][0,mode_index]) )
        temp_W_b = np.vstack( (matlab_data['lcsas_W_b_matrices'][0,mode_index],matlab_data['lcsas_W_be_matrices'][0,mode_index],-matlab_data['lcsas_W_be_matrices'][0,mode_index]) )
        temp_W = pc.Polytope(temp_W_A,temp_W_b)

        temp_mode = AffineDynamics(temp_A,temp_B,temp_W,K=temp_K)
        print(temp_mode)

        mode_list.append(temp_mode)

    word_tuple = ()
    L_cardinality = matlab_data['LAsCellArray'].shape[1]
    for word_index in range(L_cardinality):
        word_tuple = word_tuple + (matlab_data['LAsCellArray'][0,word_index][0],)
    L = Language(word_tuple)
    
    x0 = matlab_data['x0']
    n_x = x0.shape[1]
    X0 = pc.Polytope( np.vstack( (np.eye(n_x),-np.eye(n_x)) ) , np.kron(np.ones(shape=(2,1)),x0) )

    U = pc.Polytope( matlab_data['U_A'] , matlab_data['U_b'] )

    sas = SwitchedAffineDynamics(mode_list,L,X0,U)

    ## Build the Exploration-Exploitation Profile from the data
    M = []
    T, num_sequences = matlab_data['knowl_matrix'].shape
    for sequence_index in range(num_sequences):
        temp_sequence = []
        for tau in range(T):
            # Construct the words for the current language
            L_as_matrix = matlab_data['knowl_matrix'][tau,sequence_index]

            word_tuple = ()
            for word_index in range(L_as_matrix.shape[0]):
                word_tuple += ( L_as_matrix[word_index,:] , )
            
            temp_L = Language(word_tuple)
            temp_sequence.append(temp_L)

        m = KnowledgeSequence(temp_sequence)
        M.append(m)

    ## Build Internal Behavior Sets from data
    internal_behavior_sets = []
    for sequence_index in range(num_sequences):
        temp_behavior_set_sequence = []
        for tau in range(T):
            # Construct the internal behavior set for each system.
            temp_full_ks = M[sequence_index]
            temp_truncated_ks = KnowledgeSequence(temp_full_ks.sequence[:tau+1])
            temp_ibs = InternalBehaviorSet(sas,temp_truncated_ks,
                matlab_data['ibs_matrix_A'][tau,sequence_index],
                matlab_data['ibs_matrix_Ae'][tau,sequence_index],
                matlab_data['ibs_matrix_b'][tau,sequence_index],
                matlab_data['ibs_matrix_be'][tau,sequence_index]
            )
            temp_behavior_set_sequence.append(temp_ibs)
        # Append the sequence to the variable internal_behavior_sets
        internal_behavior_sets.append(temp_behavior_set_sequence)

    ## Return final results
    return sas


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
