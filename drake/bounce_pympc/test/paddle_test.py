# Drake imports
# External imports
import numpy as np
from pydrake.all import *

import os.path
import sys
sys.path.append('/root/kinova-arm/drake/bounce_pympc/')

# Internal imports
import default_params as params

# Logging
# Lines printed in the console will be automatically logged.
import logging
from datetime import datetime

logging.basicConfig(filename='runtimeLog.log', level=logging.DEBUG)
logging.info('======================= Started at {} ======================='.format(datetime.now()))

def alloc_FramePoseVector():
    return AbstractValue.Make(FramePoseVector())

cntr = 0
u_osc = [np.array([0., np.cos(0.01 * i)]) for i in range(2000)]

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
        # self.acc_input_port = self.DeclareVectorInputPort("paddle_acc", 2)
        
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

        # Log time in drake
        drake_time_msg = "----------------------- Drake time: %f s -----------------------" % context.get_time()
        print(drake_time_msg)
        logging.debug(drake_time_msg)

        # acc = self.acc_input_port.Eval(context)
        # acc = np.random.rand(2)
        acc = [0,0]
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
    paddle = PaddlePlant(params).AddToBuilder(builder, scene_graph)
    
    # Build the diagram
    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # Set the initial conditions
    context.SetContinuousState(params.xf0)
    # context.SetDiscreteState(params.xb0)
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