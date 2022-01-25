"""
quadcopter_dynamics.py
Description:

"""
import numpy as np
from scipy.integrate import ode

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def quadrotor_dynamics(t,s,u):
    # Constants
    g = 9.81 # m/s^2

    # Glen's Constants
    I_x = 0.5 # kg . m^2
    I_y = 0.1 # kg . m^2
    I_z = 0.3 # kg . m^2

    m = 1 # kg

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

def quadrotor_dynamics_with_constant_z_input( t , s ):
    # Constant
    u0 = np.zeros((4,))
    u0[0] = 10

    # Return Dynamics
    return quadrotor_dynamics(t,s,u0)

if __name__ == "__main__":
    print("Kehlani")

    s0 = np.zeros((12,))
    s0[3] = 1.0
    t0 = 0.0
    u0 = np.zeros((4,))
    u0[0] = 11

    r = ode(quadrotor_dynamics).set_integrator('zvode', method='bdf')
    r.set_initial_value(s0,t0).set_f_params(u0)

    print(r.successful())

    t1 = 10
    dt = 1
    s_trajectory = np.matrix(s0)
    print(s_trajectory.shape)

    while r.successful() and r.t < t1:
        s_t = r.integrate(r.t+dt)
        s_trajectory = np.concatenate( (s_trajectory,np.matrix(s_t)),axis=0 )
        print(r.t+dt, s_t)
        print(s_trajectory)

    # print(s_trajectory[0,:])
    # print(s_trajectory[:,0])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot( np.squeeze(np.asarray(s_trajectory[:,[0]])),np.squeeze(np.asarray(s_trajectory[:,[1]])) , np.squeeze(np.asarray(s_trajectory[:,[2]])) )
    ax.scatter(0,1,2)
    plt.show()

    # Plot Second Figure for second motion primitive

    u0 = np.zeros((4,))
    u0[0] = 11
    u0[1] = 0.1

    r2 = ode(quadrotor_dynamics).set_integrator('zvode', method='bdf')
    r2.set_initial_value(s0,t0).set_f_params(u0)

    s_trajectory2 = np.matrix(s0)
    print(s_trajectory2.shape)

    while r2.successful() and r2.t < t1:
        s_t = r2.integrate(r2.t+dt)
        s_trajectory2 = np.concatenate( (s_trajectory2,np.matrix(s_t)),axis=0 )
        print(r2.t+dt, s_t)
        print(s_trajectory2)

    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, projection='3d')
    ax2.plot( np.squeeze(np.asarray(s_trajectory2[:,[0]])),np.squeeze(np.asarray(s_trajectory2[:,[1]])) , np.squeeze(np.asarray(s_trajectory2[:,[2]])) )
    ax2.scatter(0,1,2)
    ax2.set_xlabel('x')
    ax2.set_ylabel('y')
    ax2.set_zlabel('-z')
    plt.show()

    