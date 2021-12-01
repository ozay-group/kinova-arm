"""
test2.py
Description:
    Testing how well we can fit a nonlinear function with a linear system representation.
"""

import control
import numpy as np
from matplotlib import pyplot as plt

""" Functions """

def example5_15(t,x,u,params):
    # Constants

    # Algorithm

    error = u[0] - x[0]

    return np.array( [ x[1] + np.power(x[0],2) , u[0] ] )



""" Algorithm """

# Constants

sys0 = control.NonlinearIOSystem(example5_15)

T0 = np.arange(0.0,3.0,0.1)
t, yout = control.input_output_response(sys0,T0,X0=np.array([0.0,0.0]),U=0.1*np.ones((1,len(T0))))

fig = plt.figure()
plt.plot(t,yout[0,:],label='x_0')
plt.plot(t,yout[1,:],label='x_1')

plt.legend()

# Create New System with Crazy Transfer Function
G0 = control.TransferFunction([0.5],[1/800,14/800,150/800,1])
print(G0.pole())

T1 = np.arange(0.0,3.0,0.1)
t, step1 = control.step_response(G0,T=T1)

fig = plt.figure()
plt.plot(T1,step1)

# Fit step response with a polynomial
p = np.polyfit(T1,step1,10)
print(p)

fit1 = np.poly1d(p)
fig = plt.figure()
plt.plot(T1,fit1(T1),label='fit')
plt.plot(T1,step1,label='step')
plt.legend()
plt.show()
#print(res)

# Can then differentiate with numpy.polynomial.polynomial.polyder¶
