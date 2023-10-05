"""
basictest4.py
Description:
    Trying to support the basic meshcat visualizer from within a Drake container.
    Using this to visualize Kinova Gen3 6DoF
"""

import importlib
import sys
from urllib.request import urlretrieve

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
Parser(plant, scene_graph).AddModelFromFile(
    "../../../data/models/gen3_6dof/urdf/GEN3-6DOF.urdf",
)
Parser(plant, scene_graph).AddModelFromFile(
    "../../../data/models/simpleDesk/simpleDesk.urdf",
)
#Weld table to world frame, with rotation about x
p_RightTableO = [0, 0, 0]
R_RightTableO = RotationMatrix.MakeXRotation(np.pi/2.0)
X_WorldTable = RigidTransform(R_RightTableO, p_RightTableO)
plant.WeldFrames(
    plant.world_frame(), plant.GetFrameByName("simpleDesk"),X_WorldTable)
#Weld robot to table, with translation in x, y and z
p_PlaceOnTable0 = [0.15,0.75,-0.20]
R_PlaceOnTableO = RotationMatrix.MakeXRotation(-np.pi/2.0)
X_TableRobot = RigidTransform(R_PlaceOnTableO, p_PlaceOnTable0)
plant.WeldFrames(
    plant.GetFrameByName("simpleDesk"),plant.GetFrameByName("base_link"),X_TableRobot)

plant.Finalize()
# Draw the frames
for body_name in ["base_link", "shoulder_link", "bicep_link", "forearm_link", "spherical_wrist_1_link", "spherical_wrist_2_link", "bracelet_with_vision_link", "end_effector_link"]:
    AddMultibodyTriad(plant.GetFrameByName(body_name), scene_graph)

# Connect to Meshcat
meshcat0 = Meshcat() # Object provides an interface to Meshcat
mVisualizer = MeshcatVisualizer(meshcat0)
mVisualizer.AddToBuilder(builder, scene_graph, meshcat0)

diagram = builder.Build()

context = diagram.CreateDefaultContext()
diagram.ForcedPublish(context)

while True:
    1   
