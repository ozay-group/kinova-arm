"""
move_arm_to_slider.py
Description:
    Trying to build a basic simulation where we move the gripper of the Kinova Gen3 6DoF
    to the target location and grip an object.
"""

import importlib
import sys
from urllib.request import urlretrieve

# Start a single meshcat server instance to use for the remainder of this notebook.
server_args = []
from meshcat.servers.zmqserver import start_zmq_server_as_subprocess
proc, zmq_url, web_url = start_zmq_server_as_subprocess(server_args=server_args)

# from manipulation import running_as_notebook

# Imports
import numpy as np
import pydot
from ipywidgets import Dropdown, Layout
from IPython.display import display, HTML, SVG

from pydrake.all import (
    AddMultibodyPlantSceneGraph, ConnectMeshcatVisualizer, DiagramBuilder, 
    FindResourceOrThrow, GenerateHtml, InverseDynamicsController, 
    MultibodyPlant, Parser, Simulator, RigidTransform , RotationMatrix,
    ConstantValueSource, ConstantVectorSource, AbstractValue )
from pydrake.multibody.jupyter_widgets import MakeJointSlidersThatPublishOnCallback
  
# setting path
sys.path.append('/root/kinova_drake/')

from kinova_station import KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget, KinovaStation
from observers.camera_viewer import CameraViewer

###############################################
# Important Flags

run = True
###############################################

# Building Diagram

builder = DiagramBuilder()

station = KinovaStation(time_step=0.002)

station.gripper_type = "2f_85"

# station.AddArm()
# station.Add2f85Gripper()
station.AddArmWith2f85Gripper()

station.AddGround()
station.AddCamera()

station.ConnectToMeshcatVisualizer(zmq_url="tcp://127.0.0.1:6000")

station.Finalize() # finalize station (a diagram in and of itself)

# Start assembling the overall system diagram
builder = DiagramBuilder()
station = builder.AddSystem(station)

# Add Target Object To Diagram to Allow Us to Feed in Target End Effector Positions
# If we're running a simulation, choose which sort of commands are
# sent to the arm and the gripper
ee_command_type = EndEffectorTarget.kTwist      # kPose, kTwist, or kWrench

# Send end-effector command and type
target_type_source = builder.AddSystem(ConstantValueSource(AbstractValue.Make(ee_command_type)))
builder.Connect(
        target_type_source.get_output_port(),
        station.GetInputPort("ee_target_type"))
target_source = builder.AddSystem(ConstantVectorSource(pose_des))
builder.Connect(
        target_source.get_output_port(),
        station.GetInputPort("ee_target"))

target_source.set_name("ee_command_source")
target_type_source.set_name("ee_type_source")

# Parser(plant, scene_graph).AddModelFromFile(FindResourceOrThrow("drake/kinova_drake/models/gen3_6dof/urdf/GEN3-6DOF.urdf"))
# Parser(plant, scene_graph).AddModelFromFile("simpleDesk2/simpleDesk2.urdf")

# #Weld table to world frame, with rotation about x
# p_RightTableO = [0, 0, 0]
# R_RightTableO = RotationMatrix.MakeXRotation(np.pi/2.0)
# X_WorldTable = RigidTransform(R_RightTableO,p_RightTableO)
# plant.WeldFrames(
#     plant.world_frame(), plant.GetFrameByName("simpleDesk"),X_WorldTable)
# #Weld robot to table, with translation in x, y and z
# p_PlaceOnTable0 = [0.15,0.75,-0.20]
# R_PlaceOnTableO = RotationMatrix.MakeXRotation(-np.pi/2.0)
# X_TableRobot = RigidTransform(R_PlaceOnTableO,p_PlaceOnTable0)
# plant.WeldFrames(
#     plant.GetFrameByName("simpleDesk"),station.GetFrameByName,X_TableRobot)
# plant.Finalize()


# Build the system diagram
diagram = builder.Build()
diagram.set_name("system_diagram")
diagram_context = diagram.CreateDefaultContext()

# context = diagram.CreateDefaultContext()
station.meshcat.load()
diagram.Publish(diagram_context)

if run:
    # First thing: send to home position
    station.go_home(diagram,diagram_context)

    # We use a simulator instance to run the example, but no actual simulation 
    # is being done: it's all on the hardware. 
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(True)  # not sure if this is correct

    # We'll use a super simple integration scheme (since we only update a dummy state)
    # and set the maximum timestep to correspond to roughly 40Hz 
    integration_scheme = "explicit_euler"
    time_step = 0.025
    #ResetIntegratorFromFlags(simulator, integration_scheme, time_step)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(5.0)

#Wait at end
while True:
    1