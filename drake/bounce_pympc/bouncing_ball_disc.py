from pydrake.all import *

import numpy as np

import default_params as params
from bounce_dynamics import symbolic_bounce_dynamics_restitution
S = symbolic_bounce_dynamics_restitution(params)

from pympc.control.hscc.controllers import HybridModelPredictiveController

# Logging
# Lines printed in the console will be automatically logged.
import logging
from datetime import datetime
logging.basicConfig(filename='runtimeLog.log', level=logging.DEBUG)
logging.info('======================= Started at {} ======================='.format(datetime.now()))

# global counter: how many times the controller is being called.
glbl_cntr = 0

# TODO: Outside memory - this is a hack to skip some optimization steps by checking whether
# the situation is different from the previous one (in the memory). If it is, then we skip the
# optimization with a constant paddle acceleration from previous valid solution (Zero-order hold).
b_state = np.array([])
p_acc = [0,0]

# TODO - adjust feasible region automatically for pwa bounce dynamics

# This is a little hack for the geometry output port
# Drake needs a function that allocates a new FramePoseVector wrapped in an AbstractValue
# Maybe there is a built in way to do this?
def alloc_FramePoseVector():
    return AbstractValue.Make(FramePoseVector())

class BouncingBallPlant(LeafSystem):
    """
    A system representing the movement of a ball in 2D with elastic collision with a paddle.
    The position and velocity of the paddle are treated as an input
    Dynamics are given in discrete time (approximate discretization of continuous dynamics)
    Dynamics are taken from ``bounce_dynamics.py``.
    
                        -------------------------
                        |                       |
    paddle_state -----> |                       | ----> ball_state
                        |   BouncingBallPlant   |
                        |                       | ----> ball_geom_pose
                        |                       |
                        -------------------------
    
    paddle_state: [xp, yp, xpd, ypd]
    ball_state: [xb, yb, tb, xbd, ybd, tbd]
    ball_geom_pose: [xf, yf, xfd, yfd]
    """
    
    def __init__(self, params):
        LeafSystem.__init__(self)
        
        # set ball parameters
        self.radius = params.r
        self.period = params.h
        self.pwa_sys = symbolic_bounce_dynamics_restitution(params)
        
        # [xf, yf, xfd, yfd]
        self.paddle_input_port = self.DeclareVectorInputPort("paddle_state", 4)
        
        # [xb, yb, tb, xbd, ybd, tbd]
        self.state_index = self.DeclareDiscreteState(6)
        # Update ball position with discrete period according to function DoUpdate
        self.DeclarePeriodicDiscreteUpdateEvent(self.period, 0, self.DoUpdate)
        self.state_output_port = self.DeclareStateOutputPort("ball_state", self.state_index)
        
        # Output ball geometry for visualization
        self.geom_output_port = self.DeclareAbstractOutputPort("ball_geom_pose",
                                                               alloc_FramePoseVector,
                                                               self.CalcFramePoseOutput)
        
    def DoUpdate(self, context, xd):
        """
        Discrete update of ball state. Set new state in xd
        """
        state = context.get_discrete_state_vector()
        paddle_state = self.paddle_input_port.Eval(context)

        # Log time in drake
        drake_time_msg = "----------------------- Drake time: %f s -----------------------" % context.get_time()
        print(drake_time_msg)
        logging.debug(drake_time_msg)
        
        # PWA dynamics are formulated in terms of Ball + Paddle state
        # [xb, yb, tb, xp, yp, xbd, ybd, tbd, xpd, ypd]
        x = np.array([state[0], state[1],state[2], paddle_state[0], paddle_state[1],
                       state[3], state[4],state[5], paddle_state[2], paddle_state[3]])
        # Hack - approximate paddle as not accelerating (zero input)
        u = [np.array([0,0])]
        
        xp = self.pwa_sys.simulate(x, u)[0][1]
        
        xd.set_value([xp[i] for i in [0, 1, 2, 5, 6, 7]])
        
        # The following line is critical. Odd errors result if removed
        return EventStatus.Succeeded()
       
    def CalcFramePoseOutput(self, context, poses):
        """
        Set the pose of the geometry based on system state
        """
        poses = poses.get_value()
        ball_state = context.get_discrete_state_vector()
        pose = RigidTransform(np.array([ball_state[0], 0, ball_state[1] + self.radius]))
        poses.set_value(self.f_id, pose)
    
    def AddToBuilder(self, builder, scene_graph):
        """
        Add the ball to the `builder` setup geometry
        """
        s_id = scene_graph.RegisterSource()
        f_id = scene_graph.RegisterFrame(s_id, GeometryFrame("ball"))
        # Set the ball geometry to be a sphere with phong lighting model for visualization
        g_id = scene_graph.RegisterGeometry(s_id, f_id, GeometryInstance(RigidTransform.Identity(),
                                            Sphere(self.radius), "ball_geom"))
        green = [0, 1, 0, 1]
        scene_graph.AssignRole(s_id, g_id, MakePhongIllustrationProperties(green))
        
        self.f_id = f_id
        builder.AddSystem(self)
        builder.Connect(self.geom_output_port,
                        scene_graph.get_source_pose_port(s_id))
        return self
                 
                 
class PaddlePlant(LeafSystem):
    """
    A system representing the movement of a paddle in 2D with horizontal orientation.
    Dynamics are given double integrator of accleration input.
    
                        -------------------------
                        |                       |
    paddle_acc   -----> |                       | ----> paddle_state
                        |      PaddlePlant      |
                        |                       | ----> paddle_geom_pose
                        |                       |
                        -------------------------
    
    paddle_acc: [xpdd, ypdd]
    paddle_state: [xp, yp, xpd, ypd]
    paddle_geom_pose: [xp, 0, yp]
    """
    
    def __init__(self, params):
        LeafSystem.__init__(self)
        
        # Set paddle params
        self.width = params.l
        
        # [xpdd, ypdd]
        self.acc_input_port = self.DeclareVectorInputPort("paddle_acc", 2)
        
        # Declare state with 2 positions, 2 velocities
        # [xp, yp, xpd, ypd]
        self.state_index = self.DeclareContinuousState(2, 2, 0)
        self.state_output_port = self.DeclareStateOutputPort("paddle_state", self.state_index)
        # self.state_output_port = self.DeclareStateOutputPort("paddle_state", 4, self.DoCalcTimeDerivatives)

        # Output paddle geometry for visualization
        self.geom_output_port = self.DeclareAbstractOutputPort("paddle_geom_pose",
                                                               alloc_FramePoseVector,
                                                               self.CalcFramePoseOutput)
        
    def DoCalcTimeDerivatives(self, context, derivatives):
        state = context.get_continuous_state_vector()
        acc = self.acc_input_port.Eval(context)
        derivatives.SetFromVector(np.array([state[2], state[3], acc[0], acc[1]]))
    
    def CalcFramePoseOutput(self, context, poses):
        poses = poses.get_value()
        paddle_state = context.get_continuous_state_vector()
        pose = RigidTransform(np.array([paddle_state[0], 0, paddle_state[1]]))
        poses.set_value(self.f_id, pose)
        
    def AddToBuilder(self, builder, scene_graph):
        """
        Add the ball to the `builder` setup geometry
        """
        s_id = scene_graph.RegisterSource()
        f_id = scene_graph.RegisterFrame(s_id, GeometryFrame("paddle"))
        # Set the ball geometry to be a thin box with phong lighting model for visualization
        g_id = scene_graph.RegisterGeometry(s_id, f_id, GeometryInstance(RigidTransform.Identity(),
                                            Box(self.width, self.width, 0.01), "paddle_geom"))
        grey = [0.3, 0.3, 0.3, 1]
        scene_graph.AssignRole(s_id, g_id, MakePhongIllustrationProperties(grey))
        self.f_id = f_id
        builder.AddSystem(self)
        builder.Connect(self.geom_output_port,
                        scene_graph.get_source_pose_port(s_id))
        return self
        
        
class PaddleController(LeafSystem):
    """
    Example controller for paddle
    
                        -------------------------
                        |                       |
    ball_state   -----> |                       | ----> paddle_acc
                        |   PaddleController    |
                        |                       |
                        |                       |
                        -------------------------

    ball_state: [xb, yb, tb, xbd, ybd, tbd]
    paddle_acc: [xpdd, ypdd]
    """
    
    def __init__(self, params):
        LeafSystem.__init__(self)
        self.ball_input_port = self.DeclareVectorInputPort("ball_state", 6)
        self.acc_output_port = self.DeclareVectorOutputPort("paddle_acc", 2,
                                                            self.CalcOutput)
        
    def CalcOutput(self, context, output):
        ball_state = self.ball_input_port.Eval(context)
        # proportional to ball position, results in oscillations
        u1 = 0.0
        u2 = -20 * (ball_state[1])
        output.SetFromVector([u1, u2])
        
    def AddToBuilder(self, builder, scene_graph):
        builder.AddSystem(self)
        return self

class solver(LeafSystem):
    """_summary_

    Args:
        LeafSystem (_type_): _description_

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
        
        self.ball_input_port = self.DeclareVectorInputPort("ball_state", 6)
        self.paddle_input_port = self.DeclareVectorInputPort("paddle_state", 4)

        self.acc_adv_output_port = self.DeclareVectorOutputPort("paddle_acc_adv", 2,
                                                            self.DesignController)
    
    def DesignController(self, context, output):
        # Count the number of times the controller has been called
        global glbl_cntr
        msg = "Solver being activated: %d" % glbl_cntr
        print(msg)
        logging.debug(msg)
        glbl_cntr += 1

        # Load states from context
        ball_state = self.ball_input_port.Eval(context)
        ball_msg = "Ball states: %s" % str(ball_state)
        logging.debug(ball_msg)

        # TODO: a hack to check if the ball's position has changed since last calculation.
        #       If not, the solver will be skipped and the previous acceleration will be held.
        #       (Zero-order holding). This is to avoid the solver being called unnecessarily.
        global b_state
        global p_acc
        if np.array_equal(b_state, ball_state):
            output.SetFromVector(p_acc)
            return
        else:
            b_state = ball_state


        print(ball_msg)

        paddle_state = self.paddle_input_port.Eval(context)
        paddle_msg = "Paddle states: %s" % str(paddle_state)
        print(paddle_msg)
        logging.debug(paddle_msg)

        
        # mixed-integer formulations
        methods = ['pf', 'ch', 'bm', 'mld']

        # norms of the objective
        norms = ['inf', 'one', 'two']

        # initial condition
        # [x1, x2, x3, x4, x5, x6,  x7,  x8,  x9,  x10]
        # [xb, yb, tb, xf, yf, xdb, ydb, tdb, xdf, ydf]
        x0 = np.concatenate((ball_state[:3], paddle_state[:2], ball_state[3:], paddle_state[2:]))
        # x0 = np.array([
        #     0., 0., np.pi,
        #     0., 0.,
        #     0., 0., 0.,
        #     0., 0.
        # ])

        # solves of the MICP with all the methods and the norms (takes hours!)
        solves = {}
        gurobi_options = {'OutputFlag': 1, 'LogToConsole': 0, 'LogFile': ""} # set OutputFlag to 0 to turn off gurobi log

        # for all the norms of the objective
        # for norm in norms[2]:
        norm = norms[2]  # Choose the norm-2 to solve the optimization instead of looping all
        solves[norm] = {}
        
        # for all the mixed-integer formulations
        # for method in methods:
        method = methods[1] # Choose pf method to solve the optimization instead of looping all
        # print('\n-> norm:', norm)
        # print('-> method:', method, '\n')
            
        # build the copntroller
        controller = HybridModelPredictiveController(
                S,
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
            
        # solve and store result
        u_mip, x_mip, ms_mip, cost_mip = controller.feedforward(x0, gurobi_options)

        # Check if the solution is feasible
        if u_mip is None:
            print('No solution found. u_mip: %s' % str(u_mip))
            logging.error('No solution found. u_mip: %s' % str(u_mip))
            exit(1)

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
        # for norm in norms[2]: # Choose the 2-norm to solve the optimization instead of looping all
        logging.debug("-> norm: %s" % norm)
        logging.debug("-> method: %s" % method)
        logging.debug("-> time: %f" % solves[norm][method]['time'])
        logging.debug("-> mip gap: %f" % solves[norm][method]['mip_gap'])
        logging.debug("-> nodes: %d" % solves[norm][method]['nodes'])
        logging.debug("-> u: %s" % str(solves[norm][method]['u']))
        print("Next input acc: ", u_mip[0])
        # TODO: update the zero-order holding acceleration in memory
        p_acc = u_mip[0]

        # output the controller
        output.SetFromVector(u_mip[0])
    
    def AddToBuilder(self, builder, scene_graph):
        builder.AddSystem(self)
        return self

    
        
def balldemo(init_ball, init_paddle):
    builder = DiagramBuilder()
    # Setup visualization
    meshcat = StartMeshcat()
    scene_graph = builder.AddSystem(SceneGraph())
    MeshcatVisualizer(meshcat).AddToBuilder(builder, scene_graph, meshcat)

    
    # Add modules to the diagram
    ball = BouncingBallPlant(params).AddToBuilder(builder, scene_graph)
    paddle = PaddlePlant(params).AddToBuilder(builder, scene_graph)
    cont = PaddleController(params).AddToBuilder(builder, scene_graph)
    sol = solver(params).AddToBuilder(builder,scene_graph)
    
    # Connect i/o ports
    builder.Connect(paddle.state_output_port,   ball.paddle_input_port)
    # builder.Connect(ball.state_output_port,     cont.ball_input_port)
    # builder.Connect(cont.acc_output_port,       paddle.acc_input_port)

    # Connect the solver
    builder.Connect(ball.state_output_port,     sol.ball_input_port)
    builder.Connect(paddle.state_output_port,   sol.paddle_input_port)
    builder.Connect(sol.acc_adv_output_port,    paddle.acc_input_port)
    
    # Build the diagram
    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # Set the initial conditions 
    context.SetDiscreteState(init_ball)
    context.SetContinuousState(init_paddle)
    
    # Try to run simulation 4 times slower than real time
    simulator.set_target_realtime_rate(0.25)
    simulator.AdvanceTo(10.0)
    

if __name__ == "__main__":
    try:
        balldemo([0,.1,0,0,0,0],
                [0,0,0,0])
    except KeyboardInterrupt:
        exit(1)