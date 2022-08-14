from pydrake.all import *

import numpy as np

import default_params as params
from bounce_dynamics import symbolic_bounce_dynamics_restitution
S_sim = symbolic_bounce_dynamics_restitution()
S_mpc = symbolic_bounce_dynamics_restitution(stp=2*params.h)

from pympc.control.hscc.controllers import HybridModelPredictiveController

# Logging
# Lines printed in the console will be automatically logged.
import logging
from datetime import datetime
logging.basicConfig(filename='runtimeLog.log', level=logging.DEBUG)
logging.info('======================= Started at {} ======================='.format(datetime.now()))

# global counter: how many times the controller is being called.
glbl_cntr = 0

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
        self.pwa_sys = S_sim
        
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

        # Compare the relative process to the terminal set
        terminal_center = (params.xn_min + params.xn_max) / 2
        width = params.xn_max - params.xn_min
        r = (x - terminal_center) / width
        goal_left_msg = "Relative error to the terminal set: %s" % str(r)
        print(goal_left_msg)
        logging.debug(goal_left_msg)
        
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

class Solver(LeafSystem):
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

        self.period = params.h
        # [xddf, yddf]
        self.state_index = self.DeclareDiscreteState(2)
        # Update paddle acceleration with discrete period according to function DesignController()
        self.DeclarePeriodicDiscreteUpdateEvent(self.period, 0, self.DesignController)
        self.acc_adv_output_port = self.DeclareStateOutputPort("paddle_acc_adv", self.state_index)
    
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
        
        output.set_value(u_mip[0])
        
        # The following line is critical. Odd errors result if removed
        return EventStatus.Succeeded()
    
    def AddToBuilder(self, builder, scene_graph):
        builder.AddSystem(self)
        return self

class Region(LeafSystem):
    """_summary_
    https://drake.mit.edu/pydrake/pydrake.multibody.plant.html?highlight=multibody.plant.multibodyplant#pydrake.multibody.plant.MultibodyPlant
    Args:
        LeafSystem (_type_): _description_
    """
    def __init__(self,params,scene_graph):
        LeafSystem.__init__(self)

        # Constants
        self.block_name = 'Region'

        # Create a multibody plant with discrete time step 1e-3.
        self.plant = MultibodyPlant(time_step=1e-3)

        # Specify the plant as a source for the scene graph.
        self.scene_graph = scene_graph
        self.name = self.plant.RegisterAsSourceForSceneGraph(self.scene_graph)

        # Retrieve the bounds from default_params.py
        sr = params.safety_region
        tr = params.target_region

        ## 2D to 3D: symmetric between x and y
        sc = params.safety_center
        b_sc = [sc[0], 0.0, sc[1]]
        f_sc = [sc[2], 0.0, sc[3]]

        tc = params.target_center
        b_tc = [tc[0], 0.0, tc[1]]
        f_tc = [tc[2], 0.0, tc[3]]

        # Construct the safety region for the ball
        self.s_B = self.plant.AddRigidBody("safety_region_b", SpatialInertia(mass=0.0, p_PScm_E=np.array([0., 0., 0.]), G_SP_E=UnitInertia(1.0, 1.0, 1.0)))
        self.plant.RegisterVisualGeometry(self.s_B, RigidTransform(RotationMatrix.MakeXRotation(0.0), b_sc), Box(sr[0], sr[0], sr[1]), \
            "safety_region_b", diffuse_color=[1.0, 0.0, 1.0, 0.1]) # magenta
        self.plant.WeldFrames(self.plant.world_frame(), self.plant.GetFrameByName("safety_region_b"), RigidTransform())

        # Construct the safety region for the floor (paddle)
        self.s_P = self.plant.AddRigidBody("safety_region_f", SpatialInertia(mass=0.0, p_PScm_E=np.array([0., 0., 0.]), G_SP_E=UnitInertia(1.0, 1.0, 1.0)))
        self.plant.RegisterVisualGeometry(self.s_P, RigidTransform(RotationMatrix.MakeXRotation(0.0), f_sc), Box(sr[2], sr[2], sr[3]), \
            "safety_region_f", diffuse_color=[0.0, 1.0, 0.0, 0.1]) # green
        self.plant.WeldFrames(self.plant.world_frame(), self.plant.GetFrameByName("safety_region_f"), RigidTransform())

        # Construct the target region for the ball
        self.t_B = self.plant.AddRigidBody("target_region_b", SpatialInertia(mass=0.0, p_PScm_E=np.array([0., 0., 0.]), G_SP_E=UnitInertia(1.0, 1.0, 1.0)))
        self.plant.RegisterVisualGeometry(self.t_B, RigidTransform(RotationMatrix.MakeXRotation(0.0), b_tc), Box(tr[0], tr[0], tr[1]), \
            "target_region_b", diffuse_color=[0.0, 0.0, 1.0, 0.4]) # blue
        self.plant.WeldFrames(self.plant.world_frame(), self.plant.GetFrameByName("target_region_b"), RigidTransform())

        # Construct the target region for the floor (paddle)
        self.t_P = self.plant.AddRigidBody("target_region_f", SpatialInertia(mass=0.0, p_PScm_E=np.array([0., 0., 0.]), G_SP_E=UnitInertia(1.0, 1.0, 1.0)))
        self.plant.RegisterVisualGeometry(self.t_P, RigidTransform(RotationMatrix.MakeXRotation(0.0), f_tc), Box(tr[2], tr[2], tr[3]), \
            "target_region_f", diffuse_color=[1.0, 0.0, 0.0, 0.4]) # red
        self.plant.WeldFrames(self.plant.world_frame(), self.plant.GetFrameByName("target_region_f"), RigidTransform())

        # Conclude up the plant
        self.plant.Finalize()

        # Create a default context for this system.
        self.def_state_v = self.plant.CreateDefaultContext().get_discrete_state_vector().value()

    def AddToBuilder(self, builder):
        # Add the plant to the diagram.
        builder.AddSystem(self.plant)

        # Connect the pose i/o ports and scene graph ports.
        builder.Connect(self.plant.get_geometry_poses_output_port(), self.scene_graph.get_source_pose_port(self.name))
        builder.Connect(self.scene_graph.get_query_output_port(),self.plant.get_geometry_query_input_port())
        return self

    
        
def balldemo():
    # Whether or not to plot the safety/target regions in the meshcat visualizer
    plot_regions = True

    # Create a diagram
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())

    # Setup visualization
    meshcat = StartMeshcat()
    MeshcatVisualizer(meshcat).AddToBuilder(builder, scene_graph, meshcat)

    
    # Add modules to the diagram
    ball = BouncingBallPlant(params).AddToBuilder(builder, scene_graph)
    paddle = PaddlePlant(params).AddToBuilder(builder, scene_graph)
    cont = PaddleController(params).AddToBuilder(builder, scene_graph)
    sol = Solver(params).AddToBuilder(builder,scene_graph)
    if plot_regions: region = Region(params, scene_graph).AddToBuilder(builder)
    
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
    context.SetDiscreteState(0, params.xb0) # initial context for the ball system.
    context.SetDiscreteState(1, [0, 0]) # initial context for the solver (paddle acceleration)
    if plot_regions: context.SetDiscreteState(2, region.def_state_v) # initial context for the region plotting system if Region() is constructed.
    context.SetContinuousState(params.xf0)
    
    # Try to run simulation 4 times slower than real time
    simulator.set_target_realtime_rate(0.25)
    simulator.AdvanceTo(0.5)
    

if __name__ == "__main__":
    try:
        balldemo()
    except KeyboardInterrupt:
        exit(1)