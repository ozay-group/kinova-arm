# Drake imports
# External imports
from email import feedparser
import numpy as np
from pydrake.all import *

# Internal imports
import os.path
import sys
sys.path.append('/root/kinova-arm/drake/bounce_pympc/')
import default_params as params
from bouncing_ball_disc import BouncingBallPlant
from test_modules import Feeder, Sink

# Logging
# Lines printed in the console will be automatically logged.
import logging
from datetime import datetime

logging.basicConfig(filename='runtimeLog.log', level=logging.DEBUG)
logging.info('======================= Started at {} ======================='.format(datetime.now()))

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
    feeder = Feeder(params).AddToBuilder(builder, scene_graph)
    sink = Sink(params).AddToBuilder(builder, scene_graph)

    builder.Connect(feeder.disc_state_output_port, ball.paddle_input_port)
    builder.Connect(ball.state_output_port, sink.input_por)
    
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