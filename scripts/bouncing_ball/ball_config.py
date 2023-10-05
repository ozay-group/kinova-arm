import importlib
import sys
from urllib.request import urlretrieve

# from manipulation import running_as_notebook

# Imports
import numpy as np
import pydot
from ipywidgets import Dropdown, Layout
from IPython.display import display, HTML, SVG, clear_output

# Prevent ModuleNotFoundError: No module named 'tkinter'
# https://stackoverflow.com/a/49988926
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt

from pydrake.all import (
    AddMultibodyPlantSceneGraph, Meshcat, MeshcatVisualizer, DiagramBuilder,
    FindResourceOrThrow, GenerateHtml, InverseDynamicsController, 
    MultibodyPlant, Parser, Simulator, RigidTransform , SpatialVelocity, RotationMatrix,
    AffineSystem, Diagram, LeafSystem, LogVectorOutput, CoulombFriction, HalfSpace )
from pydrake.multibody.jupyter_widgets import MakeJointSlidersThatPublishOnCallback

from pydrake.geometry import (Cylinder, GeometryInstance,
                                MakePhongIllustrationProperties)

from manipulation.scenarios import AddMultibodyTriad, AddTriad

def AddGround(plant):
    """
    Add a flat ground with friction
    """

    # Constants
    transparent_color = np.array([0.5,0.5,0.5,0])
    nontransparent_color = np.array([0.5,0.5,0.5,0.1])

    p_GroundOrigin = [0, 0.0, 0.0]
    R_GroundOrigin = RotationMatrix.MakeXRotation(0.0)
    X_GroundOrigin = RigidTransform(R_GroundOrigin,p_GroundOrigin)

    # Set Up Ground on Plant

    surface_friction = CoulombFriction(
            static_friction = 0.7,
            dynamic_friction = 0.5)
    plant.RegisterCollisionGeometry(
            plant.world_body(),
            X_GroundOrigin,
            HalfSpace(),
            "ground_collision",
            surface_friction)
    plant.RegisterVisualGeometry(
            plant.world_body(),
            X_GroundOrigin,
            HalfSpace(),
            "ground_visual",
            transparent_color)  # transparent

builder = DiagramBuilder()

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)

ball_as_model = Parser(plant=plant).AddModelFromFile(
            "../../data/models/ball_model/ball.urdf",
            'ball')
# plant.RegisterCollisionGeometry(ball_as_model, RigidTransform(), ball_as_model.shape, "A", CoulombFriction(mu, mu))
AddGround(plant)

plant.Finalize()
# Draw the frames
ball_frame = plant.GetFrameByName("ball")
parent_plant = ball_frame.GetParentPlant()
AddTriad(parent_plant.get_source_id(),
             parent_plant.GetBodyFrameIdOrThrow(ball_frame.body().index()), scene_graph,
             0.25, 0.01, 1.0, ball_frame.GetFixedPoseInBodyFrame())

state_logger = LogVectorOutput(plant.get_state_output_port(ball_as_model), builder)
state_logger.set_name("state_logger")

# Draw Triad system to the ball (helpful for debugging)
ball_frame = plant.GetFrameByName("ball")
parent_plant = ball_frame.GetParentPlant()
AddTriad(parent_plant.get_source_id(),
        parent_plant.GetBodyFrameIdOrThrow(ball_frame.body().index()), scene_graph,
        length=0.25, radius=0.01, opacity=1., name="triad2")

# Connect to Meshcat
meshcat0 = Meshcat() # Object provides an interface to Meshcat
meshcat0.SetAnimation #TODO: Action buttons for simulation.
mCpp = MeshcatVisualizer(meshcat0)
mCpp.AddToBuilder(builder,scene_graph,meshcat0)

diagram = builder.Build()

diagram_context = diagram.CreateDefaultContext()
p_WBlock = [0.0, 0.0, 0.1]
R_WBlock = RotationMatrix.MakeXRotation(0.0/2.0)
X_WBlock = RigidTransform(R_WBlock,p_WBlock)
plant.SetFreeBodyPose(
            plant.GetMyContextFromRoot(diagram_context),
            plant.GetBodyByName("ball", ball_as_model),
            X_WBlock)
plant.SetFreeBodySpatialVelocity(
            plant.GetBodyByName("ball", ball_as_model),
            SpatialVelocity(np.array([0.0, 0.0, 0.0]),np.array([0.0,0.0,0.0])),
            plant.GetMyContextFromRoot(diagram_context))

diagram.ForcedPublish(diagram_context)

"""
formatter = {'float': lambda x: '{:5.2f}'.format(x)}
def my_callback(plant_context):
        results = plant.get_contact_results_output_port().Eval(plant_context)
        meshcat0.Delete("contact")

        clear_output(wait=True)
        if results.num_point_pair_contacts()==0:
            print("no contact")
        for i in range(results.num_point_pair_contacts()):
            info = results.point_pair_contact_info(i)
            pair = info.point_pair()

            point_string = np.array2string(
                info.contact_point(), formatter=formatter)
            normal_string = np.array2string(
                -pair.nhat_BA_W, formatter=formatter)
            force_string = np.array2string(
                info.contact_force(), formatter=formatter)
            print(
              f"slip speed:{info.slip_speed():.4f}, "
              f"separation speed:{info.separation_speed():.4f}, "
              f"depth:{pair.depth:.4f},\n"
              f"point:{point_string},\n"
              f"normal:{normal_string},\n"
              f"force:{force_string}\n"
            )

        diagram.Publish(context)
"""

# Set up simulation
simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

# Run simulation
simulator.Initialize()
simulator.AdvanceTo(10.0)

# Collect Data
state_log = state_logger.FindLog(diagram_context)
log_times  = state_log.sample_times()
state_data = state_log.data()
print(state_data.shape)