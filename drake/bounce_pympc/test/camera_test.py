# Drake imports
# External imports
import numpy as np
from pydrake.all import *
import pyrealsense2 as rs
import cv2

# Internal imports
import os.path
import sys
sys.path.append('/root/kinova-arm/drake/bounce_pympc/')
import default_params as params
from test_modules import Sink
from bouncing_ball_hw import Camera

# Logging
# Lines printed in the console will be automatically logged.
import logging
from datetime import datetime

logging.basicConfig(filename='runtimeLog.log', level=logging.DEBUG)
logging.info('======================= Started at {} ======================='.format(datetime.now()))

def demo():
    # with Camera().__enter__() as Camera:
        # Whether or not to plot the safety/target regions in the meshcat visualizer
        plot_regions = True

        # Create a diagram
        builder = DiagramBuilder()
        scene_graph = builder.AddSystem(SceneGraph())

        # Setup visualization
        # meshcat = StartMeshcat()
        # MeshcatVisualizer(meshcat).AddToBuilder(builder, scene_graph, meshcat)

        
        # Add modules to the diagram
        cam = Camera(params).AddToBuilder(builder, scene_graph)
        sink = Sink().AddToBuilder(builder, scene_graph)

        builder.Connect(cam.state_output_port, sink.input_port)
        
        # Build the diagram
        diagram = builder.Build()

        # Set up a simulator to run this diagram
        simulator = Simulator(diagram)
        context = simulator.get_mutable_context()
        context.SetDiscreteState(params.xb0) # initial context for the ball system.
        context.SetContinuousState([]) # initial context for the solver (paddle acceleration)

        # Set the initial conditions
        # context.SetDiscreteState(params.xb0)
        # if plot_regions: context.SetDiscreteState(region.def_state_v) # initial context for the region plotting system if Region() is constructed.
        
        # Try to run simulation 4 times slower than real time
        simulator.set_target_realtime_rate(0.25)
        simulator.set_publish_every_time_step(True)
        simulator.AdvanceTo(0.5)
    

if __name__ == "__main__":
    try:
        demo()
    except KeyboardInterrupt:
        exit(1)