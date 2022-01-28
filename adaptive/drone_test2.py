"""
drone_test2.py
Description:
    Using the python Quadrotor object to create symbolic derivatives.
"""

import numpy as np
from scipy.integrate import ode
import sympy as sp

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from quadrotor.quadrotor import Quadrotor

# Constants

q0 = Quadrotor()
t = 1

# Create Symbolic Vectors

x, y, z = sp.symbols('x y z')

x , y , z , alpha , beta , gamma , x_dot , y_dot , z_dot , alpha_dot , beta_dot , gamma_dot = sp.symbols('x , y , z , alpha , beta , gamma , x_dot , y_dot , z_dot , alpha_dot , beta_dot , gamma_dot')

s0 = [ x , y , z , alpha , beta , gamma , x_dot , y_dot , z_dot , alpha_dot , beta_dot , gamma_dot ]
print(s0)

s1 = sp.symarray('s1',(12,))
print(s1)

u = sp.symarray('u',(4,))
print(u)

t1 = sp.symbols('t1')

# Compute f
# print(q0.f(t,s0,u))
# print(q0.f(t,s1,u))
print(q0.f_symbolic(t,s1,u))
print(sp.diff(q0.f_symbolic(t,s1,u)[0],s1))

print( q0.f_symbolic(t,s1,u).jacobian(s1) )

A_container = np.zeros((12,12)) #np.ndarray((12,12))
A_container[1,:] = sp.diff(q0.f_symbolic(t,s1,u)[1],s1)

print(A_container)

print(q0.SymbolicLinearization(t1, s1, u))

s2 = np.zeros((12,))
u2 = np.zeros((4,))
print(q0.GetLinearizedMatricesAbout(s2,u2))

#s3 = np.array([[0.1],[0.2],[0.1],[0.2],[0.1],[0.2],[0],[0],[0],[0],[0],[0]])
s3 = np.array([0.1,0.2,0.1,0.2,0.1,0.2,0,0,0,0,0,0])
u3 = np.array([5.0,0,0,0])
print(q0.GetLinearizedMatricesAbout(s3,u3))