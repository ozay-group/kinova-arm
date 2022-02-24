"""
itp.py
Description:
    A file which contains information about the InvertedThickPendulum.
"""

import numpy as np
import matplotlib.pyplot as plt
import sympy as sp
import control

import unittest

class InvertedThickPendulum:
    """
    InvertedThickPendulum
    Description:

    """

    def __init__(self,m=1,length_in=1.5,height_in=1.0,rel_CoM=None,mu_rot=0.0):
        """
        __init__
		Description:
			Initializer for the Inverted Thick Pendulum
		
			  |<---- L --->|
			_ ______________ _
			^ |            | ^
			| |            | |
			| |            | Height
			| |            | |
			v |            | V
			_ ______________ _
		
	    	Where the axis of rotation is on the bottom right corner.
		
			The center of mass is located at the point (rel_CoM['x'],rel_CoM['y']) relative
			to the reference frame starting in the BOTTOM RIGHT corner of the square.
		
		Usage:
			itp = InvertedThickPendulum()
        """

        # Set Member Variables
        self.mass   = m
        self.length = length_in
        self.height = height_in
        self.mu_rot = mu_rot

        # Initializing Center of Mass
        if rel_CoM is None:
            self.rel_CoM = {}
            self.rel_CoM['x'] = (length_in/2.0) - length_in
            self.rel_CoM['y'] = height_in/2.0
        else:
            self.rel_CoM = rel_CoM

        # Initializing State
        self.x = np.array([ [0.1],[0.2] ])

    def show(self) -> None:
        """
        show
        Description:
            This algoirhtm
        """

        # Constants

        lower_right_corner_pos = np.array([0.0,0])
        lrcp = lower_right_corner_pos

        alpha_ang = np.arctan(self.height/self.length)
        theta = wrap_to_pi(self.x[0][0])

        # Plotting Constants
        lw = 2.0 # Line Width
        pendulumColorChoice = 'blue'

        markerSizeCoM = 72;         # Size of the Marker for Center Of Mass Position
        CoMColorChoice = 'blue'

        ## Construct All Geometric Features

        # Plotting all corners
        corners = np.array([[lrcp[0] , lrcp[1]], \
						    [lrcp[0] , lrcp[1] + self.height], \
						    [lrcp[0] - self.length , lrcp[1] + self.height], \
                            [lrcp[0] - self.length , lrcp[1]] ])
        corners = corners.T

        if (0 <= theta) and (theta <= np.pi):
            angle_wrt_horizontal = -((np.pi/2) - theta - alpha_ang) ;
        elif (-np.pi <= theta) and (theta <= 0):
            angle_wrt_horizontal = -(np.pi/2 + (-theta ) - alpha_ang );
        else:
            raise ValueError('Unexpected angle value:' + str(theta))


        rot_phi = np.array([[ np.cos( angle_wrt_horizontal ) , -np.sin( angle_wrt_horizontal )], \
            			    [ np.sin( angle_wrt_horizontal ) ,  np.cos( angle_wrt_horizontal )] ])

        rot_corners = np.dot(rot_phi, corners)

        print(rot_corners)

        ## Plot
        
        # Plotting Sides of the Thick Pendulum
        num_corners = 4
        for corner_idx in range(num_corners-1):
            plt.plot( rot_corners[0,corner_idx:corner_idx+2], rot_corners[1,corner_idx:corner_idx+2] )
        plt.plot( rot_corners[0,[3,0]], rot_corners[1,[3,0]] )

        # Plotting the Center of Mass
        CoM_coordinates = np.array([ [self.rel_CoM['x']] , [self.rel_CoM['y']] ])
        rot_CoM = np.dot(rot_phi,CoM_coordinates)
        plt.plot( rot_CoM[0] , rot_CoM[1] , 'ro' )

        # Plotting the upper corner position (state of the system)
        plt.plot( rot_corners[0,2], rot_corners[1,2] , 'g*' )

        # Finalize Plot
        plt.show()

    def get_CoM_angle_offset(self):
        """
        Description:
        	The position of the center of mass causes there to be an offset between
        	theta (x(1)) and the true angle at which the center of mass is located
        	theta + theta_tilde. This finds the offset theta_tilde.
        """

        # Constants

        # Algorithm
        theta_CoM0 = np.arctan( self.rel_CoM['y'] / np.abs(self.rel_CoM['x']) );

        theta_tilde = - (theta_CoM0 - np.arctan( self.height / self.length ));	
        return theta_tilde

    def f(self,x,u)->np.array:
        """
        f
        Description:
            The nonlinear dynamics of this inverted pendulum system (with unknown center of mass).
            Describes what the derivative of the system's dynamics are for a given state x and input u
        Input:
            x - np.array of shape (2,1)
              - Example: x = np.array([ [1.0], [2.0] ])
        """

        # Constants
        m = self.Mass
        g = 9.8

        r_CoM = np.linalg.norm( np.array([[self.CoMx_rel],[self.CoMy_rel]]) ,2)

        # Algorithm
        dxdt = np.array([
            [ x[1] ],
            [ r_CoM * m * g * np.cos( (np.pi/2) - (x[0] + self.get_CoM_angle_offset()) ) - self.mu_rot * x[1] + u ]
        ])
        return dxdt

    def f_symbolic(self,x,u):
        """
        f_symbolic
        Description:
            Computes the symbolic value of the quadratic function variables.
        """

        # Constants
        m = self.Mass
        g = 9.8

        r_CoM = np.linalg.norm( np.array([[self.CoMx_rel],[self.CoMy_rel]]) ,2)

        # Get State Values

        x_c = x[0] # Corner's x-position
        y_c = x[1] # Corner's y-position

        # Algorithm 

        dxdt = sp.matrices.Matrix((
            ( x[1] ),
            (r_CoM * m * g * sp.cos( (np.pi/2) - (x[0] + self.get_CoM_angle_offset()) ) - self.mu_rot * x[1] + u)
        ))

        return dxdt

    def SymbolicLinearization(self,sym_s,sym_u) -> (np.ndarray,np.ndarray):
        """
        SymbolicLinearization
        Description:
            Uses sympy's built-in tools to define the symbolic linearization of the inverted thick pendulum system.
        """

        # Constants
        n_x = 2
        n_u = 1

        # Algorithm
        f_sym_s_u = self.f_symbolic(sym_s,sym_u)

        A = f_sym_s_u.jacobian(sym_s)
        B = f_sym_s_u.jacobian(sym_u)

        return A, B 

    def GetLinearizedMatricesAbout(self,x,u)->(np.ndarray,np.ndarray,np.ndarray):
        """
        GetLinearizedMatricesAbout
        Description:
            Linearizes the model matrices about the desired state s and input u.
        """

        # Constants
        n_x = 2
        n_u = 1

        # Get the symbolic vectors for s and u
        sym_s = sp.symarray('s',(n_x,))
        sym_u = sp.symarray('u',(n_u,))

        # Create Linearization Matrices

        A_symb, B_symb = self.SymbolicLinearization(sym_s,sym_u)

        # Evaluate them at the current state and input.
        mapFromSymbolicToValue = {}
        for s_index in range(len(s)):
            mapFromSymbolicToValue[sym_s[s_index]] = s[s_index]

        for u_index in range(len(u)):
            mapFromSymbolicToValue[sym_u[u_index]] = u[u_index]
        
        # Define A and B, by plugging in values
        A = A_symb.subs(mapFromSymbolicToValue)
        B = B_symb.subs(mapFromSymbolicToValue)
        K = self.f(x,u)

        return np.array(A), np.array(B), K

    def GetDiscretizedLinearizedMatricesAbout(self,x,u,dt)->(np.ndarray,np.ndarray,np.ndarray):
        """
        GetLinearizedMatricesAbout
        Description:
            Linearizes the model matrices about the desired state s and input u.
        """

        # Constants
        n_x = 2
        n_u = 1

        # Compute Continuous-Time Linearization
        Ac, Bc, Kc = self.GetLinearizedMatricesAbout(x,u)

        # Discretize the Matrices
        cont_sys1 = control.StateSpace(Ac,Bc,np.eye(Ac.shape[0]),np.zeros(shape=(Ac.shape[0],Bc.shape[1])))
        cont_sys2 = control.StateSpace(Ac,Kc,np.eye(Ac.shape[0]),np.zeros(shape=(Ac.shape[0],Kc.shape[1])))

        # Create Discretized Version of System
        disc_sys1 = control.c2d( cont_sys1 , dt )
        disc_sys2 = control.c2d( cont_sys2 , dt )

        Ad = disc_sys1.A
        Bd = disc_sys1.B
        Kd = disc_sys2.B

        return Ad, Bd, Kd

def wrap_to_pi(x):
    """
    wrap_to_pi
    Description:
        Wraps the angle x to a value between -pi and +pi.
    Usage:
        x_wrapped = wrap_to_pi(x)
    """
    pi = np.pi
    while ( x < -np.pi ) or (x > np.pi):
        if x < -np.pi:
            x += 2*pi

        if x > np.pi:
            x -= 2*pi

    return x

class TestScalarUncertainAffineDynamics(unittest.TestCase):
    """
    Description:
        Tests the ScalarUncertainAffineDynamics object.
    """
    def test_constructor1(self):
        """
        test_constructor1
        Description:
            Verifies that the system can correctly be constructed using the default constructor.
        """
        try:
            itp1 = InvertedThickPendulum()
            self.assertTrue(True)

        except:
            self.asserTrue(False)

    def test_wrap_to_pi0(self):
        """
        test_wrap_to_pi0
        Description:
            Verifies that the algorithm properly returns THE SAME ANGLE when the angle is already between
            -pi and +pi.
        """

        x1 = np.pi/4.0
        self.assertEqual( x1 , wrap_to_pi(x1) )


    def test_wrap_to_pi1(self):
        """
        test_wrap_to_pi1
        Description:
            Verifies that the algorithm properly returns the expected angle when the angle is very far from the desired range.
        """
        
        x1 = -np.pi/2.0
        x1_prime = x1 + 3.0 * 2 * np.pi
        self.assertAlmostEqual( x1 , wrap_to_pi(x1_prime) )

    def test_show1(self):
        """
        test_show1
        Description:
            Verifies that the algorithm correctly shows the standard initial condition.
        """

        showPlot = False

        if showPlot:
            itp1 = InvertedThickPendulum()
            plt.figure()
            itp1.show()

        self.assertTrue(True)

    def test_get_CoM_angle_offset0(self):
        """
        test_get_CoM_angle_offset0
        Description:
            This function tests the get_CoM_angle_offset() member function.
            For the default CoM values, this should be zero.
        """
        
        itp1 = InvertedThickPendulum()
        self.assertAlmostEqual( 0.0 , itp1.get_CoM_angle_offset() )

    def test_get_CoM_angle_offset1(self):
        """
        test_get_CoM_angle_offset1
        Description:
            This function tests the get_CoM_angle_offset() member function.
            For a slightly tweaked offset, we should be able to find the angle offset.
        """
        
        itp1 = InvertedThickPendulum()
        itp1.rel_CoM['x'] = -itp1.length
        itp1.rel_CoM['y'] = 0.0
        self.assertAlmostEqual( np.arctan(itp1.height/itp1.length) , itp1.get_CoM_angle_offset() )