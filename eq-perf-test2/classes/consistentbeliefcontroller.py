"""
consistentbeliefcontroller
"""
from nis import match
from random import sample
import unittest
import polytope as pc
import numpy as np
import scipy.io as sio

from classes.affinedynamics import AffineDynamics, sample_from_polytope , get_N_samples_from_polytope
from classes.switchedaffinedynamics import SwitchedAffineDynamics, get_test_sad1
from classes.knowledgesequence import KnowledgeSequence
from classes.language import Language
from classes.reachablebehaviorset import InternalBehaviorSet

class ConsistentBeliefController:
    """
    ConsistentBeliefController
    Description:
        Controller which executes the control from the Correct-By-Construction 
    """

    def __init__(self, system, ee_profile, K_set, k_set, internal_behavior_sets, options_in=None):
        """
        __init__
        Description:
        
        Usage:
            cbc_out = ConsistentBeliefController( sad , M , K_set , k_set , internal_behavior_sets  )
        """

        # Constants
        

        # Input Processing

        # verify that system is of type dyn
        if not isinstance(system,SwitchedAffineDynamics):
            raise TypeError("The input 'system' to the controller must be a SwitchedAffineDynamics() object.")
        
        # Algorithm

        self.system = system
        self.profile = ee_profile
        self.K_set = K_set
        self.k_set = k_set
        self.internal_behavior_sets = internal_behavior_sets

        if options_in is None:
            self.settings = CBCSettings()
        else:
            if isinstance(options_in,CBCSettings):
                raise TypeError("The input 'options_in' must be of class CBCSettings().")

            self.settings = options_in

        # Create Memory Variables
        n_x, n_u, n_y, n_w, n_v = system.dimensions()

        self.clear_histories() # Clear histories

    def num_sequences(self)->int:
        """
        num_sequences
        Description:
            Returns the number of Knowledge Sequences used to define this controller.
        """
        return len(self.profile)

    def time_horizon(self)->int:
        """
        time_horizon
        Description:
            Returns the number of time steps into the future that this controller attempts to make guarantees for.
        """
        return self.profile[0].time_horizon()

    def compute_control(self):
        """
        compute_control
        Description:
            Assuming that you've saved the x_history and u_history variables into the controller memory, then this
            computes the proper control input for the current time step.
        """

        # Constant
        system = self.system
        L = system.L
        T = len(L.words[0])
        n_x, n_u, n_y, n_w, n_v = system.dimensions()
        
        t = self.u_history.shape[1]
        M = self.profile

        # Algorithm
        if self.settings.feedback_method == 'Disturbance (State)':
            gain_index = self.prefix_detection()
            self.b_history = M[gain_index].subsequence(0,t)
            #print(self.b_history)
            print("gain_index = " + str(gain_index) )
        else:
            raise Exception('The only feedback method that ConsistentBeliefController supports is \'Disturbance (State)\'. Received ' + self.settings.feedback_method )

        # Create control input u
        if t == 0:
            u_t = self.k_set[0][t*n_u:(t+1)*n_u]
        else:
            w_vec = self.history_to_w_vec()
            K_i = self.K_set[gain_index]
            k_i = self.k_set[gain_index]

            print(k_i)

            u_t = np.dot(K_i[t*n_u:(t+1)*n_u,:(t*n_w)],w_vec) + np.reshape(k_i[t*n_u:(t+1)*n_u],newshape=(n_u,))

        self.u_history = np.hstack( (self.u_history, np.reshape(u_t,newshape=(n_u,1)) ) )

        return u_t

    def simulate_1_run(self):
        """
        simulate_1_run
        Description:
            Simulates one run of the finite time system described by the SwitchedAffineDynamics.
        Usage:
            x_0_t, u_0_tm1 , y_0_t , sig = cbc.simulation_1_run()
        """

        # Constants
        system = self.system
        L = system.L
        T = len(L.words[0])
        n_x, n_u, n_y, n_w, n_v = system.dimensions()

        ## Algorithm
        x_0_t, y_0_t, u_0_tm1 = np.array([]).reshape((n_x,0)), np.array([]).reshape((n_y,0)), np.array([]).reshape((n_u,0))

        w_seq, v_seq, sig , x0 = self.generate_uncontrolled_variables()

        # Initialize Controller Histories
        x_t = x0
        x_0_t = np.hstack( (x_0_t , x_t) )
        self.x_history = x_0_t

        sigma0, v0 = sig[0], v_seq[:,0].reshape((n_v,1))
        y_t = np.dot(system.Dynamics[sigma0].C,x0) + np.dot(system.Dynamics[sigma0].C_v,v0)
        y_0_t = np.hstack( (y_0_t, y_t) )
        self.y_history = y_0_t

        # Simulate
        print("sig = " + str(sig))
        for t in range(T):
            print("t = " + str(t))

            print(self.x_history)
            print(self.u_history)
            print(self.b_history)

            sigma_t = sig[t]
            w_t = w_seq[:,t].reshape((n_w,1))

            #Use Affine Dynamics with proper control law.
            print("x_t.shape = " + str(x_t.shape))
            u_t = self.compute_control()
            print("u_t.shape = " + str(u_t.shape))
            print("sigma_t = " + str(sigma_t) )
            print("w_t = " + str(w_t))
            x_tp1 = system.f( x_t, np.reshape(u_t,newshape=(n_u,1)), sigma_t , w=w_t )

            print("x_tp1 = ")
            print(x_tp1)

            #Update other variables in system
            x_t = x_tp1
            x_0_t = np.hstack( (x_0_t , x_t) )
            self.x_history = x_0_t

            if t == T-1:
                continue

            sigma_tp1 = sig[t+1]

            y_t = np.dot(system.Dynamics[sigma_tp1].C,x0) + np.dot(system.Dynamics[sigma_tp1].C_v,v0)
            y_0_t = np.hstack( (y_0_t, y_t) )
            self.y_history = y_0_t


        # Return
        return x_0_t, u_0_tm1 , y_0_t , sig
    
    def generate_uncontrolled_variables(self):
        """
        generate_uncontrolled_variables
        Description:
            This function produces the values for each uncontrolled variable in the SwitchedAffineDynamics
            for use in simulation.
        """

        # Constants
        system = self.system
        L = system.L
        T = len(L.words[0])
        n_x, n_u, n_y, n_w, n_v = system.dimensions()

        # Produce T process noise vectors.
        sig   = L.words[ np.random.randint(0,system.n_modes()) ]

        w_seq, v_seq = np.array([]).reshape((n_w,0)), np.array([[]]).reshape((n_v,0))
        for sig_t in sig:
            mode_t = system.Dynamics[sig_t]
            w_seq = np.hstack( (w_seq, sample_from_polytope( mode_t.W )) )
            v_seq = np.hstack( (v_seq, sample_from_polytope( mode_t.V )) )

        x0 = sample_from_polytope(system.X0)

        return w_seq, v_seq, sig, x0


    def prefix_detection(self):
        """
        prefix_detection
        Description:
            Uses set containment of the current external behavior in order to make an estimate of the current
            mode.
        Usage:
            gain_index = self.prefix_detection()
        """

        # Constants
        M = self.profile
        u_history = self.u_history
        b_history = self.b_history

        eb0 = self.history_to_external_behavior()
        t   = u_history.shape[1]

        num_sequences = len(M)

        ## Algorithm
        candidate_indices = []

        # Only consider indices which have the right prefix.
        if t != 0:
            for knowl_seq_index in range(num_sequences):
                temp_m = M[knowl_seq_index]
                temp_prefix = temp_m.subsequence(0,t-1)
                if temp_prefix == self.b_history:
                    candidate_indices.append(knowl_seq_index)
        else:
            candidate_indices = range(num_sequences)
            return 0

        
        # Find all indices whose behavior set MATCHES eb0
        matching_indices = []
        for knowl_seq_index in candidate_indices:
            temp_ibs = self.internal_behavior_sets[knowl_seq_index][t]
            eb0_in_tibs, temp_ibs = temp_ibs.has_associated_external_behavior(eb0)
            if eb0_in_tibs:
                matching_indices.append(knowl_seq_index)

        print("matching_indices = " + str(matching_indices) )

        # If no behaviors match, then throw an error!
        if len(matching_indices) == 0:
            print('There was an issue identifying the external behavior!')
            print('eb0 = ' + str(eb0.T))
            if t > 0:
                print('belief_history = ')
                print(b_history)
        
        # Search through all matching indices for the one with maximum cardinality
        detected_index = matching_indices[0]
        detected_prefix = M[detected_index].subsequence(0,t)
        
        mi = detected_index

        for mi_index in range(1,len(matching_indices)):
            mi = matching_indices[mi_index]
            temp_prefix = M[mi].subsequence(0,t)
            if temp_prefix >= detected_prefix:
                # Update the matching prefix to be temp_prefix
                detected_prefix = temp_prefix
                detected_index  = mi

        return mi

    def history_to_external_behavior(self):
        """
        history_to_external_behavior
        Description:
            Converts the multiple history objects into the external behavior observed by the controller so far.
        """

        # Constants

        # Algorithm
        if self.settings.feedback_method == 'Disturbance (State)':
            return np.vstack( ( self.history_to_x_vec() , self.history_to_u_vec() ) )
        else:
            raise Exception('The only feedback method that history_to_external_behavior() supports is \'Disturbance (State)\'. Received ' + self.settings.feedback_method )

    def history_to_x_vec(self):
        """
        history_to_x_vec
        Description:
            Converts the history matrix for x into a vector.
        """

        # Constants
        t = self.x_history.shape[1]-1
        system = self.system
        n_x = system.dim_x()

        # Algorithm
        return np.reshape(self.x_history, newshape=((t+1)*n_x,1), order='F')

    def history_to_u_vec(self):
        """
        history_to_u_vec
        Description:
            Converts the history matrix for u into a vector.
        """

        # Constants
        t = self.u_history.shape[1]
        system = self.system
        n_u = system.dim_u()

        # Algorithm
        return np.reshape(self.u_history, newshape=(t*n_u,1), order='F')

    def history_to_w_vec(self):
        """
        history_to_w_vec
        Description:
            This algorithm uses the history matrices to reconstruct which process disturbances
            w could have generated the given behavior.
        """
        # Constants
        t = self.u_history.shape[1]
        system = self.system
        n_x, n_u, n_y, n_w, n_v = system.dimensions()

        x_history = self.x_history
        u_history = self.u_history
        b_history = self.b_history

        #Algorithm
        w_mat = np.zeros(shape=(n_w,t))
        for tau in range(t):
            # Reconstruct the disturbance at time tau
            x_tau = np.reshape(x_history[ :,tau ],newshape=(n_x,),order='F')
            x_tau_p_one = np.reshape(x_history[:,tau+1],newshape=(n_x,),order='F')
            u_tau = np.reshape(u_history[:,tau],newshape=(n_u,),order='F')

            mode_tau_index = b_history.sequence[tau+1].words[0][0]
            mode_tau = system.Dynamics[mode_tau_index]

            w_tau, status_tau = mode_tau.reconstruct_w(x_tau,x_tau_p_one,u_tau)
            if status_tau != 'optimal':
                raise Exception('Error: The reconstruction of disturbance w returned a non-optimal status: ' + status_tau )

            w_mat[:,tau] = w_tau

        return np.reshape(w_mat,newshape=(n_w*t,),order='F')

    def history_to_internal_behavior(self):
        """
        history_to_internal_behavior
        Description:
            Converts the multiple history objects into the internal behavior observed by the controller so far.
        """

        # Constants
        system = self.system
        n_x = system.dim_x()

        t = self.x_history.shape[1]-1

        # Algorithm
        if self.settings.feedback_method == 'Disturbance (State)':
            if t == 0:
                return np.reshape(self.history_to_x_vec(),newshape=(n_x,),order='F')
            if t > 0:
                return np.vstack( ( self.history_to_x_vec() , self.history_to_u_vec() , self.history_to_w_vec(), np.reshape(self.x_history[:,0],newshape=(n_x,),order='F') ) )
        else:
            raise Exception('The only feedback method that history_to_external_behavior() supports is \'Disturbance (State)\'. Received ' + self.settings.feedback_method )

    def clear_histories(self):
        """
        clear_histories
        Description:
            Clears all of the memory elements of the controller.
        Usage:
            self.clear_histories()
        """

        # Constants
        system = self.system
        n_x, n_u, n_y, n_w, n_v = system.dimensions()

        # Algorithm

        self.x_history = np.zeros(shape=(n_x,0))
        self.y_history = np.zeros(shape=(n_y,0))
        self.u_history = np.zeros(shape=(n_u,0))
        self.b_history = KnowledgeSequence([])


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
        temp_B_w = matlab_data['lcsas_B_w_matrices'][0,mode_index]
        temp_K = matlab_data['lcsas_K_matrices'][0,mode_index]

        temp_W_A = np.vstack( (matlab_data['lcsas_W_A_matrices'][0,mode_index],matlab_data['lcsas_W_Ae_matrices'][0,mode_index],-matlab_data['lcsas_W_Ae_matrices'][0,mode_index]) )
        temp_W_b = np.vstack( (matlab_data['lcsas_W_b_matrices'][0,mode_index],matlab_data['lcsas_W_be_matrices'][0,mode_index],-matlab_data['lcsas_W_be_matrices'][0,mode_index]) )
        temp_W = pc.Polytope(temp_W_A,temp_W_b)

        temp_mode = AffineDynamics(temp_A,temp_B,temp_W,K=temp_K,B_w=temp_B_w)

        mode_list.append(temp_mode)

    word_tuple = ()
    L_cardinality = matlab_data['LAsCellArray'].shape[1]
    T = len(matlab_data['LAsCellArray'][0,0][0])
    for word_index in range(L_cardinality):
        word_as_np_array = matlab_data['LAsCellArray'][0,word_index][0] - 1
        print(word_as_np_array)
        word_tuple = word_tuple + ( word_as_np_array.tolist() ,)
    L = Language(word_tuple)

    x0 = matlab_data['x0']
    n_x = mode_list[0].dim_x()
    X0 = pc.Polytope( np.vstack( (np.eye(n_x),-np.eye(n_x)) ) , np.kron(np.array([[1],[-1]]),x0) )

    U = pc.Polytope( matlab_data['U_A'] , matlab_data['U_b'] )

    sas = SwitchedAffineDynamics(mode_list,L,X0,U)

    ## Build the Exploration-Exploitation Profile from the data
    M = []
    T, num_sequences = matlab_data['knowl_matrix'].shape
    for sequence_index in range(num_sequences):
        temp_sequence = []
        for tau in range(T):
            # Construct the words for the current language
            L_as_matrix = matlab_data['knowl_matrix'][tau,sequence_index] - 1

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
            print(temp_ibs.as_polytope.dim)
            temp_behavior_set_sequence.append(temp_ibs)
        # Append the sequence to the variable internal_behavior_sets
        internal_behavior_sets.append(temp_behavior_set_sequence)

    K_set = matlab_data['controller_K_matrices'].flatten()
    k_set = matlab_data['controller_k_matrices'].flatten()

    ## Return final results
    cbc_out = ConsistentBeliefController( sas , M , K_set , k_set , internal_behavior_sets )

    return cbc_out


class TestCBCSettings(unittest.TestCase):
    """
    test_construct1
    Description:
        Test the empty language construction.
    """
    def test_construct1(self):
        ts0 = CBCSettings()
        self.assertEqual(ts0.feedback_method,'Disturbance (State)')

class TestConsistentBeliefController(unittest.TestCase):
    """
    test_construct1
    Description:
        Test the empty language construction.
    """
    def test_construct1(self):
        ts0 = CBCSettings()
        self.assertEqual(ts0.feedback_method,'Disturbance (State)')

    def test_history_to_x_vec1(self):
        """
        test_history_to_x_vec1
        Description:
            Verifies for a simple x_history, u_history combination that the algorithm identifies
            the correct sequence of disturbances.
        """

        # Constants
        sad1 = get_test_sad1()
        L = sad1.L
        n_x, n_u, n_y, n_w, n_v = sad1.dimensions()

        M = [ KnowledgeSequence([L,L,L,L]) ]
        T = 4

        K_set = [np.eye(4*n_u,4*n_w)]
        k_set = [np.ones(shape=(T*n_u,1))]

        internal_behavior_sets = [ pc.Polytope() ]

        cbc_out = ConsistentBeliefController( sad1 , M , K_set , k_set , internal_behavior_sets  )

        cbc_out.x_history = np.array([[1.0,2.0],[3.0,4.0]])

        self.assertTrue( np.all(cbc_out.history_to_x_vec() == np.array([[1.0],[3.0],[2.0],[4.0]]) ) )
        
    def test_history_to_u_vec1(self):
        """
        test_history_to_u_vec1
        Description:
            Verifies for a simple u_history that the algorithm identifies
            the correct u history as a vector.
        """

        # Constants
        sad1 = get_test_sad1()
        L = sad1.L
        n_x, n_u, n_y, n_w, n_v = sad1.dimensions()

        M = [ KnowledgeSequence([L,L,L,L]) ]
        T = 4

        K_set = [np.eye(4*n_u,4*n_w)]
        k_set = [np.ones(shape=(T*n_u,1))]

        internal_behavior_sets = [ pc.Polytope() ]

        cbc_out = ConsistentBeliefController( sad1 , M , K_set , k_set , internal_behavior_sets  )

        cbc_out.u_history = np.array([[1.0,2.0]])

        self.assertTrue( np.all(cbc_out.history_to_u_vec() == np.array([[1.0],[2.0]]) ) )

    def test_history_to_u_vec2(self):
        """
        test_history_to_u_vec1
        Description:
            Verifies for a simple u_history that the algorithm identifies
            the correct u history as a vector.
        """

        # Constants
        sad1 = get_test_sad1()
        sad1.Dynamics[0].B = np.eye(2)
        sad1.Dynamics[1].B = np.eye(2)

        L = sad1.L
        n_x, n_u, n_y, n_w, n_v = sad1.dimensions()

        M = [ KnowledgeSequence([L,L,L,L]) ]
        T = 4

        K_set = [np.eye(4*n_u,4*n_w)]
        k_set = [np.ones(shape=(T*n_u,1))]

        internal_behavior_sets = [ pc.Polytope() ]

        cbc_out = ConsistentBeliefController( sad1 , M , K_set , k_set , internal_behavior_sets  )

        cbc_out.u_history = np.array([[1.0,2.0],[2.0,1.0]])
        self.assertTrue( np.all(cbc_out.history_to_u_vec() == np.array([[1.0],[2.0],[2.0],[1.0]]) ) )

    def test_history_to_w_vec1(self):
        """
        test_history_to_w_vec1
        Description:
            Verifies for a simple u_history that the algorithm identifies
            the correct w history as a vector.
        """

        # Constants
        sad1 = get_test_sad1()
        L = sad1.L
        n_x, n_u, n_y, n_w, n_v = sad1.dimensions()

        M = [ KnowledgeSequence([L,L,L,L]) ]
        T = 4

        K_set = [np.eye(T*n_u,T*n_w)]
        k_set = [np.ones(shape=(T*n_u,1))]

        internal_behavior_sets = [ pc.Polytope() ]

        cbc_out = ConsistentBeliefController( sad1 , M , K_set , k_set , internal_behavior_sets  )

        # Simulate System with random inputs.
        cbc_out.u_history = np.array([[1.0,2.0]])
        cbc_out.b_history = KnowledgeSequence([L,L,L])
        w_seq = np.zeros(shape=( n_w , 2 ))
        cbc_out.x_history = np.zeros(shape=(n_x,3))

        # cbc_out.x_history[:,0] = np.zeros(shape=(n_x,1))
        for tau in range(1,3):
            #Compute next step with correct input
            u_tau = np.reshape(cbc_out.u_history[:,tau-1],newshape=(n_u,),order='F')
            w_tau = np.reshape(w_seq[:,tau-1],newshape=(n_w,),order='F')
            x_tau = np.reshape(cbc_out.x_history[:,tau-1],newshape=(n_x,),order='F')

            cbc_out.x_history[:,tau] = sad1.f(x_tau,u_tau,0,w=w_tau)

        # Verify that the w_seq is correctly reconstructed
        w_seq_as_vec = np.reshape(w_seq,newshape=(2*n_w,),order='F')

        self.assertTrue( np.allclose( w_seq_as_vec , cbc_out.history_to_w_vec()) )

    

if __name__ == '__main__':
    unittest.main()
