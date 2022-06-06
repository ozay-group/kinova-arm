"""_summary_
    This function is meant to provide a connection module to the meshcat simulator for our Kinova arm.
    This is built based on infinity_simulation.py
    It is worth considering that in "infinity_simulation.py", there are lines of codes being commented out.
    They are extensional functions. In this script, only necessary functions will be remained for the
    purposes of clarity and minimizing.

    In bouncing ball controlling, the end effector is replaced by a paddle. No actuator or sensor maneuvers the paddle.
    Therefore, any function concerning with gripper controllers is deleted.

Raises:
    RuntimeError: _description_
    ConnectionError: The script failed to connect to meshcat

Returns:
    _type_: _description_
    image:
"""

###########
# Imports #
###########

from pydrake.all import *
import numpy as np
import matplotlib.pyplot as plt

# Start a single meshcat server instance to use for the remainder of this notebook.
server_args = []
from meshcat.servers.zmqserver import start_zmq_server_as_subprocess
proc, zmq_url, web_url = start_zmq_server_as_subprocess(server_args=server_args)


# Import kinova drake which is not in the subdirectories here
import sys
sys.path.append('/root/kinova_drake/')
from kinova_station import KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget, KinovaStation
from observers.camera_viewer import CameraViewer

# Import the controller you designed
import control_module

####################
# Helper Functions #
####################

def add_loggers_to_system(builder,station):
    """_summary_
        Loggers force certain outputs to be computed.
    Args:
        builder (_type_): _description_
        station (_type_): _description_
    """
    wrench_logger = LogVectorOutput(station.GetOutputPort("measured_ee_wrench"),builder)
    wrench_logger.set_name("wrench_logger")

    pose_logger = LogVectorOutput(station.GetOutputPort("measured_ee_pose"), builder)
    pose_logger.set_name("pose_logger")

    twist_logger = LogVectorOutput(station.GetOutputPort("measured_ee_twist"), builder)
    twist_logger.set_name("twist_logger")

    gripper_logger = LogVectorOutput(station.GetOutputPort("measured_gripper_velocity"), builder)
    gripper_logger.set_name("gripper_logger")

def create_scenario():
    """_summary_
        Creates the 6 degree of freedom Kinova system in simulation. Anchors it to a "ground plane" and gives it the
        RobotiQ 2f_85 gripper.
        This should also initialize the meshcat visualizer so that we can easily view the robot.
    Returns:
        builder (_type_): _description_
        controller ():
        station ():
        diagram ():
        diagram_context ():
    """

    # Determine the type of the end effector (gripper)
    gripper_type = '2f_85'

    ## Build the environment
    # Start with the Kinova Station object
    station = KinovaStation(time_step=0.001,n_dof=6)
    station.AddArmWith2f85Gripper()
    station.AddGround()
    station.AddCamera()
    #station.SetupSinglePegScenario(gripper_type=gripper_type, arm_damping=False)

    # Create environment in MeshCat
    station.ConnectToMeshcatVisualizer(zmq_url="tcp://127.0.0.1:6000")

    station.Finalize() # finalize station (a diagram in and of itself)

    ## Build the Kinova
    # Start assembling the overall system diagram
    builder = DiagramBuilder()
    station = builder.AddSystem(station)

    # Setup loggers
    add_loggers_to_system(builder,station)

    # Setup Controller
    controller = control_module.setup_example_controller_and_connect_to_station(builder,station)

    # Build the system diagram
    diagram = builder.Build()
    diagram.set_name("system_diagram")
    diagram_context = diagram.CreateDefaultContext()

    # context = diagram.CreateDefaultContext()
    station.meshcat.load()
    diagram.Publish(diagram_context)

    # Return station
    return builder, controller, station, diagram, diagram_context

#########################
# Simulation Parameters #
#########################

# Make a plot of the inner workings of the station
show_station_diagram = True

# Make a plot of the diagram for this example, where only the inputs
# and outputs of the station are shown
show_toplevel_diagram = True

# Which gripper to use (hande or 2f_85)
gripper_type = "2f_85"

# Option of simulation
run = True

##########################
# Setting Up the Station #
##########################

# Set up the kinova station
builder, controller, station, diagram, diagram_context = create_scenario()
#station.SetArmPositions(diagram,diagram_context,np.array([0.1,0.0,0.0,2.0,0.0,0.2]))

if run:

    # First thing: send to home position
    station.go_home(diagram,diagram_context)

    # We use a simulator instance to run the example, but no actual simulation 
    # is being done: it's all on the hardware. 
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)  # not sure if this is correct

    # We'll use a super simple integration scheme (since we only update a dummy state)
    # and set the maximum timestep to correspond to roughly 40Hz 
    integration_scheme = "explicit_euler"
    time_step = 0.025
    #ResetIntegratorFromFlags(simulator, integration_scheme, time_step)

    try:
        # Run simulation
        simulator.Initialize()
        simulator.AdvanceTo(40.0)
    except Exception as e:
        print(str(e))

while True:
    1

