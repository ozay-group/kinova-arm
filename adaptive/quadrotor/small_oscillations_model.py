"""
small_oscillations_model.py
Description:
    Defines the SmallOscillationsQuadrotor() object, an object which makes it simple to construct twelve-dimensional quadrotor dynamics for
    simulation or other experiments.
    This comes from (2.27) in the thesis referenced below.
References:
    Based on the work in this thesis:
        https://www.kth.se/polopoly_fs/1.588039.1600688317!/Thesis%20KTH%20-%20Francesco%20Sabatino.pdf
"""
import numpy as np
from scipy.integrate import ode

import sympy as sp

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class SmallOscillationsQuadrotor:
    """
    SmallOscillationsQuadrotor class
        This object can be used to quickly construct the continuous time, nonlinear dynamics of a quadrotor type system.
        This has been tested on some basic dynamical systems simulation libraries, and can be used to calculate linearizations
        of the dynamics as well.
    """

    def __init__(self,I_x=0.5,I_y=0.1, I_z=0.3, m=1.0) -> None:
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
        Notes:
            This is the model without disturbances. (2.27) without tau_wx, tau_wy, tau_wz, f_wx, f_wy, f_wz.
        """

        # Constants
        g = 9.81 # m/s^2

        I_x = self.I_x
        I_y = self.I_y
        I_z = self.I_z # kg . m^2

        m = self.m # kg

        # Get Input Values

        f_t   = u[0]
        tau_x = u[1]
        tau_y = u[2]
        tau_z = u[3]

        # Get State Values

        phi     = s[0]
        theta   = s[1]
        psi     = s[2]
        p       = s[3]
        q       = s[4]
        r       = s[5]
        u2      = s[6]
        v       = s[7]
        w       = s[8]
        x       = s[9]
        y       = s[10]
        z       = s[11]

        # Algorithm 

        f_s = [ 
            p + r * theta + q * phi * theta ,
            q - r * phi ,
            r + q * phi ,
            ((I_y - I_z)/I_x) * r * q + (1.0/I_x)*tau_x,
            ((I_z - I_x)/I_y) * p * r + (1.0/I_y)*tau_y,
            ((I_x - I_y)/I_z) * p * q + (1.0/I_z)*tau_z,
            r * v - q * w - g * theta,
            p * w - r * u2 + g * phi,
            q * u2 - p * v + g - (1.0/m)*f_t,
            w*(phi*psi + theta) - v*(psi - phi * theta) + u2,
            v*(1 + phi * psi * theta) - w * (phi - psi * theta) + u2 * psi,
            w - u2 * theta + v * phi
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

        phi     = s[0]
        theta   = s[1]
        psi     = s[2]
        p       = s[3]
        q       = s[4]
        r       = s[5]
        u2      = s[6]
        v       = s[7]
        w       = s[8]
        x       = s[9]
        y       = s[10]
        z       = s[11]

        # Algorithm 

        f_s = sp.matrices.Matrix(( 
            (p + r * theta + q * phi * theta) ,
            (q - r * phi) ,
            (r + q * phi) ,
            ( ((I_y - I_z)/I_x) * r * q + (1/I_x)*u[1] ) ,
            ( ((I_z - I_x)/I_y) * p * r + (1/I_y)*u[2] ) ,
            ( ((I_x - I_y)/I_z) * p * q + (1/I_z)*u[3] ) ,
            (r * v - q * w - g * theta),
            (p * w - r * u2 + g * phi),
            (q * u2 - p * v + g - (1.0/m)*u[0]),
            (w*(phi*psi + theta) - v*(psi - phi * theta) + u2),
            (v*(1 + phi * psi * theta) - w * (phi - psi * theta) + u2 * psi),
            (w - u2 * theta + v * phi)
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
        Usage:
            Ac_k, Bc_k = quad_model.GetLinearizedMatricesAbout(s_k,u_k)
        """

        # Constants
        n_x = 12
        n_u = 4

        # Get the symbolic vectors for s and u
        sym_s = sp.symarray('s',(n_x,))
        sym_u = sp.symarray('u',(n_u,))
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