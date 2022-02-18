"""
drone_test9.py
Description:
    Attempting to use the nonlinear model predictive control toolbox do-mpc in order to
    control the quadrotor's twelve-dimensional model.
"""

import do_mpc
import casadi
import numpy as np
import matplotlib.pyplot as plt

# ================
# Helper Functions
# ================

def quadrotor_template_model(I_x=0.5,I_y=0.1, I_z=0.3, m=1):

    # Constants
    g = 10 # m/s^2

    # Obtain an instance of the do-mpc model class
    # and select time discretization:
    model_type = 'continuous' # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)

    # Introduce new states, inputs and other variables to the model, e.g.:
    r_x = model.set_variable(var_type='_x', var_name='r_x', shape=(1,1))
    r_y = model.set_variable(var_type='_x', var_name='r_y', shape=(1,1))
    r_z = model.set_variable(var_type='_x', var_name='r_z', shape=(1,1))

    psi   = model.set_variable(var_type='_x', var_name='psi', shape=(1,1))
    theta = model.set_variable(var_type='_x', var_name='theta', shape=(1,1))
    phi   = model.set_variable(var_type='_x', var_name='phi', shape=(1,1))
    
    v_x = model.set_variable(var_type='_x', var_name='v_x', shape=(1,1))
    v_y = model.set_variable(var_type='_x', var_name='v_y', shape=(1,1))
    v_z = model.set_variable(var_type='_x', var_name='v_z', shape=(1,1))

    p = model.set_variable(var_type='_x', var_name='p', shape=(1,1))
    q = model.set_variable(var_type='_x', var_name='q', shape=(1,1))
    r = model.set_variable(var_type='_x', var_name='r', shape=(1,1))

    # Introduce Inputs
    f_t   = model.set_variable(var_type='_u', var_name='f_t', shape=(1,1))
    tau_x = model.set_variable(var_type='_u', var_name='tau_x', shape=(1,1))
    tau_y = model.set_variable(var_type='_u', var_name='tau_y', shape=(1,1))
    tau_z = model.set_variable(var_type='_u', var_name='tau_z', shape=(1,1))

    # Set right-hand-side of ODE for all introduced states (_x).
    # Names are inherited from the state definition.
    model.set_rhs('r_x', v_x )
    model.set_rhs('r_y', v_y )
    model.set_rhs('r_z', v_z )

    model.set_rhs('psi', q * (casadi.sin(phi)/casadi.cos(theta)) + r * (casadi.cos(phi) / casadi.sin(phi) ) )
    model.set_rhs('theta', q * (casadi.cos(phi) - r * casadi.sin(phi) ) )
    model.set_rhs('phi', p + q*( casadi.sin(phi) * casadi.tan(theta) ) + r * ( casadi.cos(phi) * casadi.tan(theta) ))

    model.set_rhs('v_x', - (f_t/m) * ( casadi.sin(phi) * casadi.sin(psi) + casadi.cos(phi) * casadi.cos(psi) * casadi.sin(theta) ) )
    model.set_rhs('v_y', - (f_t/m) * ( casadi.cos(phi) * casadi.sin(psi) * casadi.sin(theta) - casadi.cos(psi) * casadi.sin(phi) ) )
    model.set_rhs('v_z', g - (f_t/m) * casadi.cos(phi) * casadi.cos(theta))

    model.set_rhs('p', ((I_y - I_z)/I_x) * q * r + (tau_x/I_x)  )
    model.set_rhs('q', ((I_z - I_x)/I_y) * p * r + (tau_y/I_y)  )
    model.set_rhs('r', ((I_x - I_y)/I_z) * p * q + (tau_z/I_z)  ) 

    # Setup model:
    model.setup()

    return model

def template_mpc(model,m=1):
    # Constants
    g = 9.8 # m / s^2

    # Obtain an instance of the do-mpc MPC class
    # and initiate it with the model:
    mpc = do_mpc.controller.MPC(model)

    # Set parameters:
    setup_mpc = {
        'n_horizon': 20, # n_horizon = 20 also works.
        'n_robust': 0,
        't_step': 0.01,
    }
    mpc.set_param(**setup_mpc)

    # Configure objective function:
    mterm = (model._x['r_x'] - 25.0 )**2 + (model._x['r_y'] - 25.0 )**2 + (model._x['r_z'] + 25.0) ** 2   #\
            # + (model._x['psi'] - np.pi/10)**2 + (model._x['phi']-np.pi/10)**2 # Setpoint tracking
    #lterm = (_x['r_x'] - 0.6)**2    # Setpoint tracking
    lterm = mterm

    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm(f_t=1, tau_x=10, tau_y=10, tau_z=10) # Scaling for quad. cost.

    # State and input bounds:
    # mpc.bounds['lower', '_x', 'C_b'] = 0.1
    # Lower bounds on inputs:
    mpc.bounds['lower','_u', 'f_t'] = 0.0
    mpc.bounds['upper','_u', 'f_t'] = 10.0*m*g
    # mpc.bounds['lower','_u', 'phi_m_2_set'] = -2*np.pi

    mpc.setup()

    return mpc

def template_simulator(model):
    # Obtain an instance of the do-mpc simulator class
    # and initiate it with the model:
    simulator = do_mpc.simulator.Simulator(model)

    # Set parameter(s):
    simulator.set_param(t_step = 0.005)

    # Optional: Set function for parameters and time-varying parameters.

    # Setup simulator:
    simulator.setup()

    return simulator

# =====
# Main
# =====

print('Hello world!')

quad_model_dompc = quadrotor_template_model()
quad_mpc_controller = template_mpc(quad_model_dompc)
quad_sim = template_simulator(quad_model_dompc)

# Create Constants for Simulation
# s_init = np.array([20,20,20,0.0,0.05,0.2,0,0,0,0,0,0]) # Create initial state
s_init = np.array([23,23,-23,0.0,0.05,0.2,0,0,0,0,0,0]) # Create initial state

quad_sim.x0 = s_init
quad_mpc_controller.x0 = s_init
quad_mpc_controller.set_initial_guess()

# Create figure for plots

# Initialize graphic:
graphics = do_mpc.graphics.Graphics(quad_sim.data)

fig, ax = plt.subplots(3, sharex=True) # Create State Trajectories
input_fig, ax2 = plt.subplots(4, sharex=True) # Create Input Trajectories

graphics.add_line(var_type='_x', var_name='r_x', axis=ax[0])
graphics.add_line(var_type='_x', var_name='r_y', axis=ax[1])
graphics.add_line(var_type='_x', var_name='r_z', axis=ax[2])
# graphics.add_line(var_type='_u', var_name='f_t', axis=ax[3])
# graphics.add_line(var_type='_u', var_name='tau_x', axis=ax[4])

graphics.add_line(var_type='_u', var_name='f_t', axis=ax2[0])
graphics.add_line(var_type='_u', var_name='tau_x', axis=ax2[1])
graphics.add_line(var_type='_u', var_name='tau_y', axis=ax2[2])
graphics.add_line(var_type='_u', var_name='tau_z', axis=ax2[3])

# Simulate
x0 = s_init
num_ticks = 300
for k in range(num_ticks):
    u0 = quad_mpc_controller.make_step(x0)
    y_next = quad_sim.make_step(u0)
    x0 = y_next

graphics.plot_results()
graphics.reset_axes()

ax[0].set_ylabel(r'$r_x$ [pos]')
ax[1].set_ylabel(r'$r_y$ [pos]')
ax[2].set_ylabel(r'$r_z$ [pos]')

ax2[0].set_ylabel(r'$f_t$ [N]')
ax2[1].set_ylabel(r'$\tau_x$ [N$\cdot$m]')
ax2[2].set_ylabel(r'$\tau_y$ [N$\cdot$m]')
ax2[3].set_ylabel(r'$\tau_z$ [N$\cdot$m]')

plt.show()

# input('Press any button to continue!')

# Fully customizable:
# ax[0].set_ylim(...)