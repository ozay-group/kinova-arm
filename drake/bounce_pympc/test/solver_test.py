# Drake imports
# External imports
import numpy as np
from pydrake.all import *

import os.path
import sys
sys.path.append('/root/kinova-arm/drake/bounce_pympc/')

# Internal imports
import default_params as params
from pympc.control.hscc.controllers import HybridModelPredictiveController

# Logging
# Lines printed in the console will be automatically logged.
import logging
from datetime import datetime

from bounce_dynamics import symbolic_bounce_dynamics_restitution
# Generate two ball dynamic models. One for motion simulation and one for MPC.
S_mpc = symbolic_bounce_dynamics_restitution(0.1)

logging.basicConfig(filename='runtimeLog.log', level=logging.DEBUG)
logging.info('======================= Started at {} ======================='.format(datetime.now()))

simulation_duration = float(0.5)
num = int(simulation_duration/params.h + 1)
x_hist = np.zeros((num,10))

class Solver(LeafSystem):
    """_summary_
    SOLVER() is a computational system based on Model Predictive Controller. 
    It takes the state of the ball and the state of the paddle and renders the 
    control input (paddle acceleration) to the paddle. Because it is a mpc solver,
    the rendered control input is the first element of the control input sequence.

    Args:
        LeafSystem (LeafSystem class): A basic model unit in drake. Solver is inherited from LeafSystem.

    Diagram:
                        -------------------------
                        |                       |
    ball_state   -----> |                       | ----> paddle_acc_adv
                        |         Solver        |
    paddle_state -----> |                       |
                        |                       |
                        -------------------------        
    """

    def __init__(self, params):
        LeafSystem.__init__(self)

        # global counter: how many times the controller is being called.
        self.cntr = 0
        
        # Declare input ports for ball and paddle states
        # self.ball_input_port = self.DeclareVectorInputPort("ball_state", 6)
        # self.paddle_input_port = self.DeclareVectorInputPort("paddle_state", 4)
        
        # Declare output ports for control input
        self.state_index = self.DeclareDiscreteState(2) # [xddf, yddf]
        self.acc_adv_output_port = self.DeclareStateOutputPort("paddle_acc_adv", self.state_index)

        # Update paddle acceleration with discrete period according to function DesignController()
        self.period = params.h
        self.DeclarePeriodicDiscreteUpdateEvent(self.period, 0, self.DesignController)
    
    def DesignController(self, context, output):
        """_summary_
        DesignController() is an optimization solver that calculate the optimal control input sequence 
        and returns the first control input in the sequence.

        Args:
            context (drake class): The context of this LeafSystem.
            output (numpy array (1,2)): The first element of the optimal control input sequence. This is
                                        the calculation result and is a referred variable.

        Returns:
            EventStatus: The status of the updating event. If it is not success, the system will not update.
        """

        # Log time in drake
        drake_time_msg = "----------------------- Drake time: %f s -----------------------" % context.get_time()
        print(drake_time_msg)
        logging.debug(drake_time_msg)

        # Count the number of times the controller has been called
        # self.cntr += 1
        msg = "Solver being activated: %d" % self.cntr
        print(msg)
        logging.debug(msg)

        # Load states from context
        # ball_state = self.ball_input_port.Eval(context)
        # ball_msg = "Ball states: %s" % str(ball_state)
        # logging.debug(ball_msg)
        # print(ball_msg)

        # TODO: a hack to check if the ball's position has changed since last calculation.
        #       If not, the solver will be skipped and the previous acceleration will be held.
        #       (Zero-order holding). This is to avoid the solver being called unnecessarily.
        # if ball_state[4] > 0: # ball is falling
        #     output.set_value([0, 0])
        #     return

        # paddle_state = self.paddle_input_port.Eval(context)
        # paddle_msg = "Paddle states: %s" % str(paddle_state)
        # print(paddle_msg)
        # logging.debug(paddle_msg)

        
        # mixed-integer formulations
        methods = ['pf', 'ch', 'bm', 'mld']

        # norms of the objective
        norms = ['inf', 'one', 'two']

        # initial condition
        # [x1, x2, x3, x4, x5, x6,  x7,  x8,  x9,  x10]
        # [xb, yb, tb, xf, yf, xdb, ydb, tdb, xdf, ydf]
        # x0 = np.concatenate((ball_state[:3], paddle_state[:2], ball_state[3:], paddle_state[2:]))
        x0 = params.x0
        # with open('x_hist.csv', 'w', encoding='utf-8', newline='') as f:
        #     writer = csv.writer(f)
        #     writer.writerow(x0)
        global x_hist
        x_hist[self.cntr, :] = x0

        self.cntr += 1

        # solves of the MICP
        solves = {}
        gurobi_options = {'OutputFlag': 1, 'LogToConsole': 0, 'LogFile': ""} # set OutputFlag to 0 to turn off gurobi log

        # Choose the norm-2 to solve the optimization instead of looping all
        norm = norms[2]
        solves[norm] = {}

        # Choose 'ch' mixed-integer formulations to solve instead of looping all
        method = methods[1]
            
        # Build the controller
        controller = HybridModelPredictiveController(
                S_mpc,
                params.N,
                params.Q,
                params.R,
                params.P,
                params.X_N,
                method,
                norm
            )
            
        # kill solution if longer than 1h
        # controller.prog.setParam('TimeLimit', 3600)
            
        # Solve and store result
        u_mip, x_mip, ms_mip, cost_mip = controller.feedforward(x0, gurobi_options)

        # Check if the solution is feasible
        if u_mip is None:
            print('No solution found. u_mip: %s' % str(u_mip))
            logging.error('No solution found. u_mip: %s' % str(u_mip))
            exit(1)

        # Organize the solution
        solves[norm][method] = {
                'time': controller.prog.Runtime,
                'nodes': controller.prog.NodeCount,
                'mip_gap': controller.prog.MIPGap,
                'u': u_mip,
                'x': x_mip,
                'ms': ms_mip,
                'cost': cost_mip
            }
    
        # log results
        logging.debug("-> norm: %s" % norm)
        logging.debug("-> method: %s" % method)
        logging.debug("-> time: %f" % solves[norm][method]['time'])
        logging.debug("-> mip gap: %f" % solves[norm][method]['mip_gap'])
        logging.debug("-> nodes: %d" % solves[norm][method]['nodes'])
        logging.debug("-> u: %s" % str(solves[norm][method]['u']))
        print("Next input acc: ", u_mip[0])
        
        # Update the output port with the first element of the optimal control input sequence
        output.set_value(u_mip[0])

        # # TODO: update the zero-order holding acceleration in memory
        # p_acc = u_mip[0]
        
        # Return the status of the updating event
        # The following line is critical. Odd errors result if removed
        return EventStatus.Succeeded()
    
    def AddToBuilder(self, builder, scene_graph):
        builder.AddSystem(self)
        return self

def plot(x_hist):
    """
    Plot the trajectory of the ball.
    """
    import matplotlib.pyplot as plt
    time_ticks = np.linspace(0, simulation_duration, num, endpoint=True)
    plt.figure()
    plt.plot(time_ticks, x_hist[:,1], 'b-', label='Ball')
    plt.plot(time_ticks, x_hist[:,4], 'r-', label='Paddle')
    plt.xlabel('Time (s)')
    plt.ylabel('Height (m)')
    plt.legend()
    plt.savefig('sim_trajectory.png') #plt.show()

def demo():
    # Whether or not to plot the safety/target regions in the meshcat visualizer
    plot_regions = True

    # Create a diagram
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())

    # Setup visualization
    meshcat = StartMeshcat()
    MeshcatVisualizer(meshcat).AddToBuilder(builder, scene_graph, meshcat)

    
    # Add modules to the diagram
    sol = Solver(params).AddToBuilder(builder,scene_graph)
    
    # Build the diagram
    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # Set the initial conditions
    context.SetDiscreteState(params.xd2f0) # initial context for the solver (paddle acceleration)
    # context.SetDiscreteState(params.xb0)
    # if plot_regions: context.SetDiscreteState(region.def_state_v) # initial context for the region plotting system if Region() is constructed.
    
    # Try to run simulation 4 times slower than real time
    simulator.set_target_realtime_rate(0.25)
    try:
        simulator_status = simulator.Initialize()
        simulator.AdvanceTo(simulation_duration)
    except RuntimeError:
        print(simulator_status.message())
        return
    

if __name__ == "__main__":
    try:
        demo()
    except KeyboardInterrupt:
        exit(1)
    finally:
        plot(x_hist)
        np.save('x_hist.npy', x_hist)