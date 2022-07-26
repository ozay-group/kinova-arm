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
    AddMultibodyPlantSceneGraph, Meshcat, MeshcatVisualizerCpp, DiagramBuilder, 
    FindResourceOrThrow, GenerateHtml, InverseDynamicsController, 
    MultibodyPlant, Parser, Simulator, RigidTransform , SpatialVelocity, RotationMatrix,
    AffineSystem, Diagram, LeafSystem, LogVectorOutput, CoulombFriction, HalfSpace )
from pydrake.multibody.jupyter_widgets import MakeJointSlidersThatPublishOnCallback

from pydrake.geometry import (Cylinder, GeometryInstance,
                                MakePhongIllustrationProperties)

def AddTriad(source_id,
             frame_id,
             scene_graph,
             length=.25,
             radius=0.01,
             opacity=1.,
             X_FT=RigidTransform(),
             name="frame"):
    """
    Adds illustration geometry representing the coordinate frame, with the
    x-axis drawn in red, the y-axis in green and the z-axis in blue. The axes
    point in +x, +y and +z directions, respectively.
    Args:
      source_id: The source registered with SceneGraph.
      frame_id: A geometry::frame_id registered with scene_graph.
      scene_graph: The SceneGraph with which we will register the geometry.
      length: the length of each axis in meters.
      radius: the radius of each axis in meters.
      opacity: the opacity of the coordinate axes, between 0 and 1.
      X_FT: a RigidTransform from the triad frame T to the frame_id frame F
      name: the added geometry will have names name + " x-axis", etc.
    """
    # x-axis
    X_TG = RigidTransform(RotationMatrix.MakeYRotation(np.pi / 2),
                          [length / 2., 0, 0])
    geom = GeometryInstance(X_FT.multiply(X_TG), Cylinder(radius, length),
                            name + " x-axis")
    geom.set_illustration_properties(
        MakePhongIllustrationProperties([1, 0, 0, opacity]))
    scene_graph.RegisterGeometry(source_id, frame_id, geom)

    # y-axis
    Y_TG = RigidTransform(RotationMatrix.MakeXRotation(np.pi / 2),
                          [0, length / 2., 0])
    geom = GeometryInstance(X_FT.multiply(Y_TG), Cylinder(radius, length),
                            name + " y-axis")
    geom.set_illustration_properties(
        MakePhongIllustrationProperties([0, 1, 0, opacity]))
    scene_graph.RegisterGeometry(source_id, frame_id, geom)

    # z-axis
    Z_TG = RigidTransform([0, 0, length / 2.])
    geom = GeometryInstance(X_FT.multiply(Z_TG), Cylinder(radius, length),
                            name + " z-axis")
    geom.set_illustration_properties(
        MakePhongIllustrationProperties([0, 0, 1, opacity]))
    scene_graph.RegisterGeometry(source_id, frame_id, geom)

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
            "/root/kinova-arm/drake/bouncing_ball/ball_model/ball.urdf",
            'ball')
# plant.RegisterCollisionGeometry(ball_as_model, RigidTransform(), ball_as_model.shape, "A", CoulombFriction(mu, mu))
AddGround(plant)

plant.Finalize()
# Draw the frames
ball_frame = plant.GetFrameByName("ball")
parent_plant = ball_frame.GetParentPlant()
AddTriad(parent_plant.get_source_id(),
             parent_plant.GetBodyFrameIdOrThrow(ball_frame.body().index()), scene_graph,
             0.25, 0.01, 1.0, ball_frame.GetFixedPoseInBodyFrame(), name="ball_frame")

# Setup data loggers
state_logger = LogVectorOutput(plant.get_state_output_port(ball_as_model), builder)
state_logger.set_name("state_logger")

"""
force_logger = LogVectorOutput(plant.get_reaction_forces_output_port(), builder)
force_logger.set_name("force_logger")

contact_logger = LogVectorOutput(plant.get_contact_results_output_port(), builder)
contact_logger.set_name("contact_logger")
"""

# Draw Triad system to the ball (helpful for debugging)
ball_frame = plant.GetFrameByName("ball")
parent_plant = ball_frame.GetParentPlant()
AddTriad(parent_plant.get_source_id(),
        parent_plant.GetBodyFrameIdOrThrow(ball_frame.body().index()), scene_graph,
        length=0.25, radius=0.01, opacity=1.)

# Connect to Meshcat
meshcat0 = Meshcat(port=7001) # Object provides an interface to Meshcat
meshcat0.SetAnimation #TODO: Action buttons for simulation.
mCpp = MeshcatVisualizerCpp(meshcat0)
mCpp.AddToBuilder(builder,scene_graph,meshcat0)

diagram = builder.Build()

diagram_context = diagram.CreateDefaultContext()
p_WBlock = [0.0, 0.0, 3.0]
R_WBlock = RotationMatrix.MakeXRotation(0.0/2.0)
X_WBlock = RigidTransform(R_WBlock,p_WBlock)
plant.SetFreeBodyPose(
            plant.GetMyContextFromRoot(diagram_context),
            plant.GetBodyByName("ball", ball_as_model),
            X_WBlock)
plant.SetFreeBodySpatialVelocity(
            plant.GetBodyByName("ball", ball_as_model),
            SpatialVelocity(np.array([0.0, 0.0, 0.0]),np.array([2.0,0.0,0.0])),
            plant.GetMyContextFromRoot(diagram_context))

diagram.Publish(diagram_context)

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
simulator.AdvanceTo(20.0)

# Collect state data
state_log = state_logger.FindLog(diagram_context)
log_times  = state_log.sample_times()
state_data = state_log.data()
print(state_data.shape)

"""
# Collect force data
force_log = force_logger.FindLog(diagram_context)
force_data = force_log.data()
print(force_data.shape)
# Collect contact data
contact_log = contact_logger.FindLog(diagram_context)
contact_data = contact_log.data()
print(contact_data.shape)
"""

# Plot state data
if True:

    # Plot Data - First Half (position)
    fig = plt.figure()
    ax_list1 = []

    for plt_index1 in range(6):
        ax_list1.append( fig.add_subplot(231+plt_index1) )
        plt.plot(log_times,state_data[plt_index1,:])
        plt.title('State #' + str(plt_index1))
    plt.savefig('trace.png')

    # Plot Data - Second Half (velocity)
    fig = plt.figure()
    ax_list2 = []

    for plt_index2 in range(7,13):
        ax_list2.append( fig.add_subplot(231+plt_index2 - 7) )
        plt.plot(log_times,state_data[plt_index2,:])
        plt.title('State #' + str(plt_index2))

    plt.savefig('velocity.png')