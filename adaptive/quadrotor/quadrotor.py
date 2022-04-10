"""
quadcopter.py
Description:
    Defines the Quadrotor() object, an object which makes it simple to construct twelve-dimensional quadrotor dynamics for
    simulation or other experiments.
References:
    Based on the work in this thesis:
        https://www.kth.se/polopoly_fs/1.588039.1600688317!/Thesis%20KTH%20-%20Francesco%20Sabatino.pdf
"""
import numpy as np
from scipy.integrate import ode

import sympy as sp

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Quadrotor:
    """
    Quadrotor class
        This object can be used to quickly construct the continuous time, nonlinear dynamics of a quadrotor type system.
        This has been tested on some basic dynamical systems simulation libraries, and can be used to calculate linearizations
        of the dynamics as well.
    """

    def __init__(self,I_x=0.5,I_y=0.1, I_z=0.3, m=1) -> None:
        """
        __init__
        Description:
            Initializes the quadrotor object. Can use default arguments if necessary.
        Usage:
            q1 = Quadrotor()
            q1 = Quadrotor(0.7) # Sets the I_x value to 0.7
        """
        self.m   = m 
        self.I_x = I_x 
        self.I_y = I_y
        self.I_z = I_z
    
    def f(self,t,s,u) -> np.array:
        """
        f
        Description:
            Computes the value of the differential equation governing the quadrotor's dynamics at state x with input u.
        """

        # Constants
        g = 9.81 # m/s^2

        I_x = self.I_x
        I_y = self.I_y
        I_z = self.I_z # kg . m^2

        m = self.m # kg

        # Get State Values

        x = s[0]
        y = s[1]
        z = s[2]
        alpha = s[3]
        beta = s[4]
        gamma = s[5]
        x_dot = s[6]
        y_dot = s[7]
        z_dot = s[8]
        alpha_dot = s[9]
        beta_dot = s[10]
        gamma_dot = s[11]

        # Algorithm 

        f_s = [ 
            x_dot ,
            y_dot ,
            z_dot ,
            beta_dot * (np.sin(gamma)/np.cos(beta)) + gamma_dot * (np.cos(gamma)/np.cos(beta)) ,
            beta_dot * np.cos(gamma) - gamma_dot * np.sin(gamma) ,
            alpha_dot + beta_dot * np.sin(gamma) * np.tan(beta) + gamma_dot * np.cos(gamma) * np.tan(beta) ,
            -(1/m) * ( np.sin(gamma) * np.sin(alpha) + np.cos(gamma) * np.cos(alpha) * np.sin(beta) ) * u[0],
            -(1/m) * ( np.sin(gamma) * np.cos(alpha) - np.cos(gamma) * np.sin(alpha) * np.sin(beta) ) * u[0] ,
            g - (1/m) * np.cos(gamma) * np.cos(beta) * u[0],
            ((I_y - I_z)/I_x) * beta_dot * gamma_dot + (1/I_x) * u[1],
            ((I_z - I_x)/I_y) * alpha_dot * gamma_dot + (1/I_y) * u[2],
            ((I_x - I_y)/I_z) * alpha_dot * beta_dot + (1/I_z) * u[3]
        ]

        return f_s

    def f_symbolic(self,t,s,u):
        """
        f_symbolic
        Description:
            Computes the symbolic value of the quadratic function variables.
        """

        # Constants
        g = 9.81 # m/s^2

        I_x = self.I_x
        I_y = self.I_y
        I_z = self.I_z # kg . m^2

        m = self.m # kg

        # Get State Values

        x = s[0]
        y = s[1]
        z = s[2]
        alpha = s[3]
        beta = s[4]
        gamma = s[5]
        x_dot = s[6]
        y_dot = s[7]
        z_dot = s[8]
        alpha_dot = s[9]
        beta_dot = s[10]
        gamma_dot = s[11]

        # Algorithm 

        f_s = sp.matrices.Matrix(( 
            (x_dot) ,
            (y_dot) ,
            (z_dot) ,
            (beta_dot * (sp.sin(gamma)/sp.cos(beta)) + gamma_dot * (sp.cos(gamma)/sp.cos(beta))) ,
            (beta_dot * sp.cos(gamma) - gamma_dot * sp.sin(gamma)) ,
            (alpha_dot + beta_dot * sp.sin(gamma) * sp.tan(beta) + gamma_dot * sp.cos(gamma) * sp.tan(beta)) ,
            (-(1/m) * ( sp.sin(gamma) * sp.sin(alpha) + sp.cos(gamma) * sp.cos(alpha) * sp.sin(beta) ) * u[0]),
            (-(1/m) * ( sp.sin(gamma) * sp.cos(alpha) - sp.cos(gamma) * sp.sin(alpha) * sp.sin(beta) ) * u[0]) ,
            (g - (1/m) * sp.cos(gamma) * sp.cos(beta) * u[0]),
            (((I_y - I_z)/I_x) * beta_dot * gamma_dot + (1/I_x) * u[1]),
            (((I_z - I_x)/I_y) * alpha_dot * gamma_dot + (1/I_y) * u[2]),
            (((I_x - I_y)/I_z) * alpha_dot * beta_dot + (1/I_z) * u[3])
        ))

        return f_s

    def SymbolicLinearization(self,sym_t,sym_s,sym_u) -> (np.ndarray,np.ndarray):
        """
        SymbolicLinearization
        Description:

        """

        # Constants
        n_x = 12
        n_u = 4

        # Algorithm
        f_sym_s_u = self.f_symbolic(sym_t,sym_s,sym_u)

        A = f_sym_s_u.jacobian(sym_s)
        B = f_sym_s_u.jacobian(sym_u)

        return A, B 

    def GetLinearizedMatricesAbout(self,s,u)->(np.ndarray,np.ndarray):
        """
        GetLinearizedMatricesAbout
        Description:
            Linearizes the model matrices about the desired state s and input u.
        """

        # Constants
        n_x = 12
        n_u = 4

        # Get the symbolic vectors for s and u
        sym_s = sp.symarray('s',(12,))
        sym_u = sp.symarray('u',(4,))
        sym_t = sp.symbols('t')

        # Create Linearization Matrices

        A_symb, B_symb = self.SymbolicLinearization(sym_t,sym_s,sym_u)

        # Evaluate them at the current state and input.
        mapFromSymbolicToValue = {}
        for s_index in range(len(s)):
            mapFromSymbolicToValue[sym_s[s_index]] = s[s_index]

        for u_index in range(len(u)):
            mapFromSymbolicToValue[sym_u[u_index]] = u[u_index]
        
        # Define A and B, by plugging in values
        A = A_symb.subs(mapFromSymbolicToValue)

        B = B_symb.subs(mapFromSymbolicToValue)

        return np.array(A), np.array(B)