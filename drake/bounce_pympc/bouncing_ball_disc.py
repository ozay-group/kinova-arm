# Drake imports
# External imports
import numpy as np
from pydrake.all import *

# Internal imports
import default_params as params
from bounce_dynamics import symbolic_bounce_dynamics_restitution
from pympc.control.hscc.controllers import HybridModelPredictiveController

# Generate two ball dynamic models. One for motion simulation and one for MPC.
S_sim = symbolic_bounce_dynamics_restitution()
S_mpc = symbolic_bounce_dynamics_restitution(stp=10*params.h)

# Logging
# Lines printed in the console will be automatically logged.
import logging
from datetime import datetime

logging.basicConfig(filename='runtimeLog.log', level=logging.DEBUG)
logging.info('======================= Started at {} ======================='.format(datetime.now()))

# Plotting database
simulation_duration = float(10.0)
class TrajectoryPlotting():
    def __init__(self, simulation_duration, params):
        self.duration = simulation_duration
        self.plot_option = True
        self.x_hist_len = int(simulation_duration/params.h + 1) # Append one more timestamp to prevent index overflow
        self.x_hist = np.zeros((self.x_hist_len,10)) # 10 states: [xb, yb, tb, xf, yf, xdb, ydb, tdb, xdf, ydf]
    
    def log_state(self, index, state):
        self.x_hist[index, :] = state
        return self

    def plot(self, to_plot=True):
        """
        Plot the trajectory of the ball.
        """
        if not to_plot: return self # skip plotting

        import matplotlib.pyplot as plt
        time_ticks = np.linspace(0, self.duration, self.x_hist_len, endpoint=True)
        plt.figure()
        plt.plot(time_ticks, self.x_hist[:,1], 'b-', label='Ball')      # yb
        plt.plot(time_ticks, self.x_hist[:,4], 'r-', label='Paddle')    # yf
        plt.xlabel('Time (s)')
        plt.ylabel('Height (m)')
        plt.legend()
        plt.savefig('sim_trajectory.png') #plt.show()
        return self

    def save(self):
        np.save('x_hist.npy', self.x_hist)
        return self

TrajectoryPlotter = TrajectoryPlotting(simulation_duration, params)

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
        self.actv_cntr = 0

        # Optimal control (xddf, yddf) from last computation
        self.control_index = 0
        self.control_sequence = [np.zeros(2)] * params.N
        
        # Declare input ports for ball and paddle states
        self.ball_input_port = self.DeclareVectorInputPort("ball_state", 6)
        self.paddle_input_port = self.DeclareVectorInputPort("paddle_state", 4)
        
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

        # Count the number of times the controller has been called
        # self.cntr += 1
        msg = "Solver being activated: %d" % self.cntr
        print(msg)
        logging.debug(msg)

        # Load states from context
        ball_state = self.ball_input_port.Eval(context)
        ball_msg = "Ball states: %s" % str(ball_state)
        logging.debug(ball_msg)
        print(ball_msg)

        paddle_state = self.paddle_input_port.Eval(context)
        paddle_msg = "Paddle states: %s" % str(paddle_state)
        print(paddle_msg)
        logging.debug(paddle_msg)

        

        # initial condition
        # [x1, x2, x3, x4, x5, x6,  x7,  x8,  x9,  x10]
        # [xb, yb, tb, xf, yf, xdb, ydb, tdb, xdf, ydf]
        x0 = np.concatenate((ball_state[:3], paddle_state[:2], ball_state[3:], paddle_state[2:]))
        
        # Log the state into database
        TrajectoryPlotter.log_state(self.cntr, x0)
        
        self.actv_cntr += 1
        self.cntr += 1

        # TODO: Skip re-optimization and use the planned control if 1) the ball is rising, or 2) the ball is far from the paddle.
        if ball_state[1] - paddle_state[1] > 0.4 or ball_state[4] > 0: # ball is falling
            try:
                if self.actv_cntr % 10 == 0:
                    self.control_index += 1
                    output.set_value(self.control_sequence[self.control_index])
            except IndexError:
                output.set_value([0, 0])
            
            return EventStatus.Succeeded()

        # mixed-integer formulations
        methods = ['pf', 'ch', 'bm', 'mld']

        # norms of the objective
        norms = ['inf', 'one', 'two']

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
        
        # Log the optimal control sequence and update the output port with its first element
        self.control_sequence = u_mip
        self.control_index = 0
        self.actv_cntr = 0
        output.set_value(self.control_sequence[self.control_index])
        
        # Return the status of the updating event
        # The following line is critical. Odd errors result if removed
        return EventStatus.Succeeded()
    
    def AddToBuilder(self, builder, scene_graph):
        builder.AddSystem(self)
        return self

class Region(LeafSystem):
    """_summary_
    Region() is a visualization module. It loads boundaries in default_params.py and visualizes them in
    meshcat. It is an optional module to be executed in simulation, controlled by the boolean variable 
    'plot_regions'. It is not connected to any function module but connected to SceneGraph LeafSystem.
    Useful references: [MultibodyPlant](https://drake.mit.edu/pydrake/pydrake.multibody.plant.html?highlight=multibody.plant.multibodyplant#pydrake.multibody.plant.MultibodyPlant)

    Args:
        LeafSystem (LeafSystem class): A basic model unit in drake. Solver is inherited from LeafSystem.

    Diagram:
                -------------------------               ----------------------
                |              (params) |               |                    |
                |                       | ---(pose)---> |                    |
                |        Region         |               |     SceneGraph     |
                |                       | ---(query)--> |                    |
                |                       |               |                    |
                -------------------------               ----------------------
    """

    def __init__(self,params,scene_graph):
        LeafSystem.__init__(self)

        # Constants
        self.block_name = 'Region'
        self.choice = False # True for visualizing safety region, False for no visualization

        # Create a multibody plant with discrete time step 1e-3.
        self.plant = MultibodyPlant(time_step=1e-3)

        # Specify the plant as a source for the scene graph.
        self.scene_graph = scene_graph
        self.name = self.plant.RegisterAsSourceForSceneGraph(self.scene_graph)

        # Target region.
        # Retrieve the bounds from default_params.py
        tr = params.target_region

        ## 2D to 3D: symmetric between x and y
        tc = params.target_center
        b_tc = [tc[0], 0.0, tc[1]]
        f_tc = [tc[2], 0.0, tc[3]]

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

        if self.choice:
            # Safety region.
            # Retrieve the bounds from default_params.py
            sr = params.safety_region
            ## 2D to 3D: symmetric between x and y
            sc = params.safety_center
            b_sc = [sc[0], 0.0, sc[1]]
            f_sc = [sc[2], 0.0, sc[3]]

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

        # Conclude up the plant
        self.plant.Finalize()

        # Create a default context for this plant system.
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
    context.SetDiscreteState(1, params.xd2f0) # initial context for the solver (paddle acceleration)
    if plot_regions: context.SetDiscreteState(2, region.def_state_v) # initial context for the region plotting system if Region() is constructed.
    context.SetContinuousState(params.xf0)
    
    # Try to run simulation 4 times slower than real time
    simulator.set_target_realtime_rate(1.0)
    try:
        simulator_status = simulator.Initialize()
        simulator.AdvanceTo(simulation_duration)
    except RuntimeError:
        print(simulator_status.message())
        return
    

if __name__ == "__main__":
    try:
        balldemo()
    except KeyboardInterrupt:
        exit(1)
    finally:
        TrajectoryPlotter.plot()
        TrajectoryPlotter.save()
