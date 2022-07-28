from pydrake.all import *

import numpy as np

from bounce_dynamics import symbolic_bounce_dynamics_restitution

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
    """
    
    def __init__(self, params):
        LeafSystem.__init__(self)
        self.ball_input_port = self.DeclareVectorInputPort("ball_state", 6)
        self.acc_output_port = self.DeclareVectorOutputPort("paddle_acc", 2,
                                                            self.CalcOutput)
        
    def CalcOutput(self, context, output):
        ball_state = self.ball_input_port.Eval(context)
        # proportional to ball position, results in oscillations
        output.SetFromVector([0.0, -20 * ball_state[1]])
        
    def AddToBuilder(self, builder, scene_graph):
        builder.AddSystem(self)
        return self
    
        
def balldemo(init_ball, init_paddle):
    builder = DiagramBuilder()
    # Setup visualization
    meshcat = StartMeshcat()
    scene_graph = builder.AddSystem(SceneGraph())
    MeshcatVisualizer(meshcat).AddToBuilder(builder, scene_graph, meshcat)

    import default_params as params
    ball = BouncingBallPlant(params).AddToBuilder(builder, scene_graph)
    paddle = PaddlePlant(params).AddToBuilder(builder, scene_graph)
    cont = PaddleController(params).AddToBuilder(builder, scene_graph)
    
    builder.Connect(paddle.state_output_port,
                    ball.paddle_input_port)
    builder.Connect(ball.state_output_port,
                    cont.ball_input_port)
    builder.Connect(cont.acc_output_port,
                    paddle.acc_input_port)
    
    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # Set the initial conditions 
    context.SetDiscreteState(init_ball)
    context.SetContinuousState(init_paddle)
    
    # Try to run simulation 4 times slower than real time
    simulator.set_target_realtime_rate(0.25)
    simulator.AdvanceTo(200.0)
    

if __name__ == "__main__":
    balldemo([0,0.1,0,0,0,0],
             [0,0,0,0])