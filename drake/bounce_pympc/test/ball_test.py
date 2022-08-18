# Drake imports
# External imports
import numpy as np
from pydrake.all import *

# Internal imports
import default_params as params

# Logging
# Lines printed in the console will be automatically logged.
import logging
from datetime import datetime

from bounce_dynamics import symbolic_bounce_dynamics_restitution
# Generate two ball dynamic models. One for motion simulation and one for MPC.
S_sim = symbolic_bounce_dynamics_restitution()

logging.basicConfig(filename='runtimeLog.log', level=logging.DEBUG)
logging.info('======================= Started at {} ======================='.format(datetime.now()))

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
        # self.paddle_input_port = self.DeclareVectorInputPort("paddle_state", 4)
        
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
        paddle_state = np.array([0, 0, 0, 0])
        # paddle_state = self.paddle_input_port.Eval(context)

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
    ball = BouncingBallPlant(params).AddToBuilder(builder, scene_graph)
    
    # Build the diagram
    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # Set the initial conditions
    context.SetDiscreteState(params.xb0)
    # if plot_regions: context.SetDiscreteState(region.def_state_v) # initial context for the region plotting system if Region() is constructed.
    
    # Try to run simulation 4 times slower than real time
    simulator.set_target_realtime_rate(0.25)
    try:
        simulator_status = simulator.Initialize()
        simulator.AdvanceTo(float(0.5))
    except RuntimeError:
        print(simulator_status.message())
        return
    

if __name__ == "__main__":
    try:
        demo()
    except KeyboardInterrupt:
        exit(1)