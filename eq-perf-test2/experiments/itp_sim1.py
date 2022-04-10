"""
itp_sim1.py
Description:
    Attempting to simulate the inverted thick pendulum.
    Make sure to runthis from eq-perf-test2.
"""

import numpy as np
from scipy.integrate import ode
import matplotlib.pyplot as plt

import sys
sys.path.append('../')
from systems.itp import InvertedThickPendulum

print('itp_sim1.py\n\n')

def linearized_itp_f(t,x,u):
    """
    linearized_itp_f
    Description:
        
    """
    
    # Constants
    itp_temp = InvertedThickPendulum()
    A,B,K = itp_temp.GetLinearizedMatricesAbout(x,u)
    return np.dot(A,x) + np.dot(B,u) + K


#############
# Constants #
#############

itp2 = InvertedThickPendulum()
u0 = 0.0
x0 = itp2.x
t0 = 0.0

plot_nl_trajectory = True
plot_l_ct_trajectory = True

########################################
# Create Simple Simulation (nonlinear) #
########################################

# Simulate Quadrotor for a certain amount of time
r = ode(itp2.f).set_integrator('zvode', method='bdf')
r.set_initial_value(x0,t0).set_f_params(u0)
print("Was the differential equation setup successful?",r.successful())

# Simulate
t1 = 5 # seconds
delta_t = 0.01
s_trajectory = np.matrix(itp2.x.T)
u_trajectory = np.matrix([[u0]])
t_trajectory = np.matrix([[0.0]])
print('s_trajectory=',s_trajectory)
print('u_trajectory = ', u_trajectory)
#print(s_trajectory.shape)

print(itp2.GetLinearizedMatricesAbout(x0,1.2))

while r.successful() and r.t < t1:
    s_t = np.real(r.integrate(r.t+delta_t))
    print(np.real(s_t))
    s_trajectory = np.concatenate( (s_trajectory,np.matrix(s_t.T)),axis=0 )
    u_trajectory = np.concatenate( (u_trajectory,np.matrix(u0)),axis=0 )
    t_trajectory = np.concatenate( (t_trajectory,np.matrix([[r.t+delta_t]])),axis=0)
    # print(r.t+delta_t, s_t)
    # print(s_trajectory)

if plot_nl_trajectory:
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.plot( np.squeeze(np.asarray(s_trajectory[:,[0]])),np.squeeze(np.asarray(s_trajectory[:,[1]])) )

    ax.set_xlabel(r'$\theta$')
    ax.set_ylabel(r'$\dot{\theta}$')
    plt.show()

##########################################################
# Create Simple Simulation (linearized, continuous-time) #
##########################################################

# Simulate Quadrotor for a certain amount of time
r2 = ode(linearized_itp_f).set_integrator('zvode', method='bdf')
r2.set_initial_value(x0,t0).set_f_params(u0)
print("Was the differential equation setup successful?",r.successful())

# Simulate
t1 = 5 # seconds
delta_t = 0.01
s_trajectory2 = np.matrix(itp2.x.T)
u_trajectory2 = np.matrix([[u0]])
t_trajectory2 = np.matrix([[0.0]])
print('s_trajectory2 = ', s_trajectory2)
print('u_trajectory2 = ', u_trajectory2)
#print(s_trajectory2.shape)

while r.successful() and r2.t < t1:
    s_t = np.real(r2.integrate(r2.t+delta_t))
    print(np.real(s_t))
    s_trajectory2 = np.concatenate( (s_trajectory2,np.matrix(s_t.T)),axis=0 )
    u_trajectory2 = np.concatenate( (u_trajectory2,np.matrix(u0)),axis=0 )
    t_trajectory2 = np.concatenate( (t_trajectory2,np.matrix([[r.t+delta_t]])),axis=0)
    # print(r.t+delta_t, s_t)
    # print(s_trajectory2)

if plot_l_ct_trajectory:
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.plot( np.squeeze(np.asarray(s_trajectory2[:,[0]])),np.squeeze(np.asarray(s_trajectory2[:,[1]])) )

    ax.set_xlabel(r'$\theta$')
    ax.set_ylabel(r'$\dot{\theta}$')
    plt.show()
