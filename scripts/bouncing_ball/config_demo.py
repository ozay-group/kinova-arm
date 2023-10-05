"""
config_demo.py
Description:
    Trying to support the basic meshcat visualizer from within a Drake container.
    Using this to visualize Kinova Gen3 6DoF
"""

import importlib
import sys
from urllib.request import urlretrieve

"""# Start a single meshcat server instance to use for the remainder of this notebook.
server_args = []
from meshcat.servers.zmqserver import start_zmq_server_as_subprocess
proc, zmq_url, web_url = start_zmq_server_as_subprocess(server_args=server_args)"""

# from manipulation import running_as_notebook

# Imports
import numpy as np
import pydot
from ipywidgets import Dropdown, Layout
from IPython.display import display, HTML, SVG

from pydrake.all import (
    AddMultibodyPlantSceneGraph, Meshcat, MeshcatVisualizer, DiagramBuilder, 
    FindResourceOrThrow, GenerateHtml, InverseDynamicsController, 
    MultibodyPlant, Parser, Simulator, RigidTransform , RotationMatrix )
from pydrake.multibody.jupyter_widgets import MakeJointSlidersThatPublishOnCallback

from pydrake.geometry import (Cylinder, GeometryInstance,
                                MakePhongIllustrationProperties)

from manipulation.scenarios import AddMultibodyTriad

##########################
## Function Definitions ##
##########################

# Building Diagram

builder = DiagramBuilder()

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)

# Parse CAD models and add them to the plant.
# Kinova Gen3 6DoF
Parser(plant, scene_graph).AddModelFromFile(
    "../../data/models/gen3_6dof/urdf/GEN3-6DOF.urdf",
)
Parser(plant, scene_graph).AddModelFromFile("../../data/models/simpleDesk/simpleDesk.urdf") # Desk
Parser(plant, scene_graph).AddModelFromFile("../../data/models/ball_model/ball.urdf") # Bouncy Ball
Parser(plant, scene_graph).AddModelFromFile("../../data/models/paddle_model/paddle.urdf") # Paddle

"""# Ball location
pusher_position = [0.8,0.5,0.25]
# pusher_rotation=[0,np.pi/2,0]
pusher_rotation=[0,0,0]
X_pusher = RigidTransform()
X_pusher.set_translation(pusher_position)
X_pusher.set_rotation(RotationMatrix(RollPitchYaw(pusher_rotation)))
station.AddManipulandFromFile("pusher/pusher1_urdf.urdf",X_pusher)"""

ball_position = [0.8, 0.5, 0.25]
X_ball = RigidTransform()
X_ball.set_translation(ball_position)
# station.AddManipulandFromFile("ball_model/ball.urdf",X_ball)

# Weld table to world frame, with rotation about x
p_RightTableO = [0, 0, 0]
R_RightTableO = RotationMatrix.MakeXRotation(np.pi/2.0)
X_WorldTable = RigidTransform(R_RightTableO, p_RightTableO)
plant.WeldFrames(
    plant.world_frame(), plant.GetFrameByName("simpleDesk"),X_WorldTable)

# Weld robot to table, with translation in x, y and z
p_PlaceOnTableO = [0.15, 0.75, -0.20]
R_PlaceOnTableO = RotationMatrix.MakeXRotation(-np.pi/2.0)
X_TableRobot = RigidTransform(R_PlaceOnTableO,p_PlaceOnTableO)
plant.WeldFrames(
    plant.GetFrameByName("simpleDesk"),plant.GetFrameByName("base_link"),X_TableRobot)

# Weld paddle to bracelet frame
p_PlaceOnBraceletO = [0.1, -0.0375, 0.23] # compensate for the shape factor of the paddle (already scaled by the factor)
R_PlaceOnBraceletO = RotationMatrix.MakeYRotation(-np.pi/1.0)
X_BraceletPaddle = RigidTransform(R_PlaceOnBraceletO,p_PlaceOnBraceletO)
plant.WeldFrames(
    plant.GetFrameByName("end_effector_link"),plant.GetFrameByName("paddle"), X_BraceletPaddle)

plant.Finalize()
# Draw the frames
for body_name in ["base_link", "shoulder_link", "bicep_link", "forearm_link", "spherical_wrist_1_link", "spherical_wrist_2_link", "bracelet_with_vision_link", "end_effector_link"]:
    AddMultibodyTriad(plant.GetFrameByName(body_name), scene_graph)

# Inspecting the kinematic tree
if True: 
    connection_tree = pydot.graph_from_dot_data(plant.GetTopologyGraphvizString())[0]
    connection_tree.write_png("connection_tree.png")

# Connect to Meshcat
meshcat0 = Meshcat() # Object provides an interface to Meshcat
mCpp = MeshcatVisualizer(meshcat0)
mCpp.AddToBuilder(builder,scene_graph,meshcat0)

diagram = builder.Build()

context = diagram.CreateDefaultContext()
# meshcat.load()
diagram.ForcedPublish(context)

# Keep connection to meshcat server
while True:
    1   
