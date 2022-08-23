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
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt

from pydrake.all import (
    AddMultibodyPlantSceneGraph, DiagramBuilder, 
    FindResourceOrThrow, GenerateHtml, InverseDynamicsController, 
    MultibodyPlant, Parser, Simulator, RigidTransform , RotationMatrix,
    ConstantValueSource, ConstantVectorSource, AbstractValue, 
    RollPitchYaw, LogVectorOutput, plot_system_graphviz,
    LeafSystem, BasicVector, DiscreteTimeDelay, SceneGraph )
from pydrake.multibody.jupyter_widgets import MakeJointSlidersThatPublishOnCallback
  
# setting path
sys.path.append('/root/kinova_drake/')

from kinova_station import KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget, KinovaStation

sys.path.append('/root/kinova-arm/drake/')
# from controllers.velocity import VelocityCommand, VelocityCommandSequence, VelocityCommandSequenceController
from twist_sequence_controller.controller import Controller

sys.path.append('/root/kinova-arm/drake/bounce_pympc')
import default_params as params

################################################################
class TwistController(Controller):
    """
    Description:
        A controller that attempts to execute each of the commands in the CommandSequence
        object given to it.

        Sends gripper position commands as well as end-effector twist/wrench commands.
    """

    def __init__(self,
                        Kp = np.diag([10,10,10,2,2,2])*10, Kd = np.eye(6)*np.power(10.0,-2)):
        """
        __init__
        Description:
            Constructor for CommandSequenceController objects.
        """
        Controller.__init__(self,command_type=EndEffectorTarget.kWrench)

        # self.cs = command_sequence
        self.gripper_target_value = [0] # Not interested but for the sake of completeness

        # [xpdd, ypdd]
        self.acc_input_port = self.DeclareVectorInputPort("paddle_acc", 2)

        #PD gains for Twist Controller
        self.Kp = Kp
        self.Kd = Kd

    def CalcGripperCommand(self,context,output):
        """
        Description:
            Computes the gripper command (position to consider).
        """
        # t = context.get_time()

        cmd_pos = np.array([0])
        output.SetFromVector(cmd_pos)
    def SetGripperCommandType(self, context, output):
        command_type = GripperTarget.kPosition
        output.SetFrom(AbstractValue.Make(command_type))

    def CalcEndEffectorCommand(self,context,output):
        """
        CalcEndEffectorCommand
        Description:
            Computes the output command to send to the controller. 
        """
        # t = context.get_time()
        # print("t = %s" % t)

        # Get Target End-Effector Target Type
        # command_t = self.cs.current_command(t)
        #print(command_t)

        # For Twist Control
        self.command_type = EndEffectorTarget.kTwist

        # Get acceleration input
        acc = self.acc_input_port.Eval(context) # [xd2f, yd2f]
        print("------------------------------",acc,"-------------------------------")
        print("------------------------------", acc.shape, "------------------------------")

        # Get target end-effector twist and wrench
        # target_twist = command_t.ee_target_twist
        # [roll, pitch, yaw, x, y, z]
        period = params.h
        target_twist = np.array([0.0, 0.0, 0.0, period*acc[0], 0, period*acc[1]])
        target_wrench = np.zeros(6)

        # Get current end-effector pose and twist
        current_twist = self.ee_twist_port.Eval(context)
        current_wrench = self.ee_wrench_port.Eval(context)

        # Compute pose and twist errors
        twist_err = target_twist - current_twist
        wrench_err = target_wrench - current_wrench

        # Set command (i.e. end-effector twist or wrench) using a PD controller
        Kp = self.Kp
        Kd = self.Kd
        cmd = Kp@twist_err + Kd@wrench_err

        #print(cmd)

        # Return Output
        output.SetFromVector(cmd)

    def ConnectToStation(self, builder, station, time_step=-1.0):
        """
        Connect inputs and outputs of this controller to the given kinova station (either
        hardware or simulation). 
        """

        # Construct Default Value for time_step
        if time_step < 0.0:
            if isinstance(station,KinovaStation):
                time_step = station.plant.time_step()
            else:
                raise Exception("Time step should be given when running ConnectToStation() on the HarwareKinovaStation.")

        # Create a simple delay block
        delay_block = builder.AddSystem(DiscreteTimeDelay(
            time_step, # Setting the update_sec (width of each discrete step)
            1, # Setting the number of discrete steps to wait
            6  # Size of the input to the delay block
        ))

        #Connect: ee_command output port -> delay -> the station target
        builder.Connect(
            self.GetOutputPort("ee_command"),
            delay_block.get_input_port()
        )
        builder.Connect(                                  # Send commands to the station
                delay_block.get_output_port(),
                station.GetInputPort("ee_target"))
        
        # Connect the command type port to the station
        builder.Connect(
                self.GetOutputPort("ee_command_type"),
                station.GetInputPort("ee_target_type"))

        # Connect Gripper Commands to the station
        builder.Connect(
                self.GetOutputPort("gripper_command"),
                station.GetInputPort("gripper_target"))
        builder.Connect(
                self.GetOutputPort("gripper_command_type"),
                station.GetInputPort("gripper_target_type"))

        # builder.Connect(                                     # Send state information
        #         station.GetOutputPort("measured_ee_twist"),  # to the controller
        #         self.GetInputPort("ee_twist"))
        builder.Connect(
                station.GetOutputPort("measured_ee_wrench"),
                self.GetInputPort("ee_wrench"))




################################################################

class VelocityCalculator(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        # Constants
        self.last_ee_pose = np.zeros(6)
        self.last_time = 0.0
        self.last_ee_velocity = np.zeros(6)
        
        # Create Input Port for The Current State
        self.pose_port = self.DeclareVectorInputPort("current_ee_pose",
                                                                BasicVector(6))

        # Create Output Port which should share the pose of the block
        self.DeclareVectorOutputPort(
                "estimated_ee_velocity",
                BasicVector(6),
                self.ComputeVelocity,
                {self.time_ticket()}   # indicate that this doesn't depend on any inputs,
                )                      # but should still be updated each timestep

        # Build the diagram

    def ComputeVelocity(self, context, output):
        """
        Description:
            This function computes the velocity in pose-space of the signal connected to the input port.
        """
        # Constants
        t = context.get_time()
        eps0 = 0.002

        if (t - self.last_time) < eps0:
            output.SetFromVector(self.last_ee_velocity)
        else:
            # Get Current Pose from Port
            current_ee_pose = self.pose_port.Eval(context)

            ee_estimated_velocity = (current_ee_pose - self.last_ee_pose)/(t-self.last_time)
            
            # Save last variables
            self.last_time = t
            self.last_ee_velocity = ee_estimated_velocity
            self.last_ee_pose = current_ee_pose
            
            print("ee_velocity")
            print(ee_estimated_velocity)

            # Create Output
            output.SetFromVector(ee_estimated_velocity)

def add_loggers_to_system(builder,station):
    # Loggers force certain outputs to be computed
    wrench_logger = LogVectorOutput(station.GetOutputPort("measured_ee_wrench"),builder)
    wrench_logger.set_name("wrench_logger")

    pose_logger = LogVectorOutput(station.GetOutputPort("measured_ee_pose"), builder)
    pose_logger.set_name("pose_logger")

    twist_logger = LogVectorOutput(station.GetOutputPort("measured_ee_twist"), builder)
    twist_logger.set_name("twist_logger")

    gripper_logger = LogVectorOutput(station.GetOutputPort("measured_gripper_velocity"), builder)
    gripper_logger.set_name("gripper_logger")

    return pose_logger

# def setup_triangle_command_sequence():
#     """
#     Description:
#         Creates the command sequence that we need to achieve the infinity sequence.
#     Notes:
#         Each velocity is a six-dimensional vector where each dimension represents the following rates:
#         - [roll, pitch, yaw, x, y, z]
#     """
#     # Constants
#     triangle_side_duration = 10.0

#     # Create the command sequence object
#     # vcs = VelocityCommandSequence([])
#     vcs = PSCSequence([])

#     # 1. Initial Command (Pause for 5s)
#     init_velocity = np.zeros((6,))
#     vcs.append(PartialStateCommand(
#         name="pause1",
#         target_type=EndEffectorTarget.kTwist,
#         target_value=init_velocity,
#         duration=2,
#         gripper_value=False))

#     # 2. Upper Right
#     deltap1 = np.zeros((6,))
#     deltap1[3:] = np.array([0.2,0.2,0])
#     vcs.append(PartialStateCommand(
#         name="upper_right",
#         target_type=EndEffectorTarget.kTwist,
#         target_value=deltap1/triangle_side_duration,
#         duration=triangle_side_duration,
#         gripper_value=False))

#     # 3. Lower Right
#     deltap2 = np.zeros((6,))
#     deltap2[3:] = np.array([0.2,-0.2,0])
#     vcs.append(PartialStateCommand(
#         name="upper_right",
#         target_type=EndEffectorTarget.kTwist,
#         target_value=deltap2/triangle_side_duration,
#         duration=triangle_side_duration,
#         gripper_value=False))    

#     # 4. Return to STart
#     deltap3 = np.zeros((6,))
#     deltap3[3:] = np.array([-0.4,0,0])
#     vcs.append(PartialStateCommand(
#         name="return_home",
#         target_type=EndEffectorTarget.kTwist,
#         target_value=deltap3/triangle_side_duration,
#         duration=triangle_side_duration,
#         gripper_value=False))   

#     # 5. Pause
#     vcs.append(PartialStateCommand(
#         name="pause2",
#         target_type=EndEffectorTarget.kTwist,
#         target_value=init_velocity,
#         duration=2,
#         gripper_value=False))

#     return vcs

###############################################
# Important Flags

run = True
show_station_diagram = False
show_plots = True

n_dof = 6
###############################################

time_step = 0.025

# Building Diagram
with KinovaStationHardwareInterface(n_dof) as station:
    # Constants
    gripper_type = '2f_85'
    dt = 0.001

    # pusher_position = [0.8,0.5,0.25]
    # pusher_rotation=[0,np.pi/2,0]
    # pusher_rotation=[0,0,0]

    # Start with the Kinova Station object
    # station = KinovaStationHardwareInterface(time_step=dt,n_dof=6)

    # Start assembling the overall system diagram
    builder = DiagramBuilder()
    builder.AddSystem(station)

    # Setup loggers
    pose_logger = add_loggers_to_system(builder,station)

    # Setup Controller
    # cs = setup_triangle_command_sequence()

    #########
    # controller, v_estimator = setup_controller_and_connect_to_station(cs,builder,station)
    """
    Description:
        Defines the controller (PID) which is a CommandSequenceController as defined in
        kinova_drake.
    Inputs:
        cs = A CommandSequence object which helps define the CommandSequenceController.
    """

    # Create the velocity estimator
    v_estimator = builder.AddSystem(VelocityCalculator())

    # Create the controller and connect inputs and outputs appropriately
    #Kp = 10*np.eye(6)
    # Kp = np.diag([0.2,0.2,0.2,2,2,2])
    # Kd = 2*np.sqrt(Kp)

    Kp = 10*np.eye(6)
    Kd = 2*np.sqrt(Kp)

    # controller = HardwarePSCSequenceController(
    #     cs,  # wrench commands work best on hardware
    #     twist_Kp=Kp,
    #     twist_Kd=Kd)
    # builder.AddSystem(controller)
    controller = TwistController(
        Kp=Kp,
        Kd=Kd
    )
    builder.AddSystem(controller)
    # controller.ConnectToStation(builder, station, time_step)
    controller.set_name("controller")

    # # Connect the Controller to the station
    # builder.Connect(                                  # Send commands to the station
    #             controller.GetOutputPort("ee_command"),
    #             station.GetInputPort("ee_target"))
    # builder.Connect(
    #         controller.GetOutputPort("ee_command_type"),
    #         station.GetInputPort("ee_target_type"))
    # builder.Connect(
    #         controller.GetOutputPort("gripper_command"),
    #         station.GetInputPort("gripper_target"))
    # builder.Connect(
    #         controller.GetOutputPort("gripper_command_type"),
    #         station.GetInputPort("gripper_target_type"))

    # Connect the Station to the Estimator
    builder.Connect(
        station.GetOutputPort("measured_ee_pose"),
        v_estimator.GetInputPort("current_ee_pose")
    )

    # Connect the Estimator to the Controller
    builder.Connect(                                        # Send estimated state information
            v_estimator.GetOutputPort("estimated_ee_velocity"), # to the controller
            controller.GetInputPort("ee_twist"))
    # builder.Connect(
    #         station.GetOutputPort("measured_ee_twist"),
    #         controller.GetInputPort("ee_twist"))
    controller.ConnectToStation(builder, station, time_step)
    #########

    # Log Velocity Estimator
    vel_estimate_logger = LogVectorOutput(v_estimator.GetOutputPort("estimated_ee_velocity"), builder)
    vel_estimate_logger.set_name("velocity_estimate_logger")

    from test_modules import Feeder
    scene_graph = builder.AddSystem(SceneGraph())
    F = Feeder(params).AddToBuilder(builder, scene_graph)
    builder.Connect(F.disc_state_output_port, controller.acc_input_port)

    # Build the system diagram
    diagram = builder.Build()
    diagram.set_name("system_diagram")
    diagram_context = diagram.CreateDefaultContext()

    # context = diagram.CreateDefaultContext()
    # station.meshcat.load()
    diagram.Publish(diagram_context)

    if show_station_diagram:
        # Show the station's system diagram
        plt.figure()
        plot_system_graphviz(diagram,max_depth=1)
        plt.show()

    if run:
        # # First thing: send to home position
        station.go_home()

        # We use a simulator instance to run the example, but no actual simulation 
        # is being done: it's all on the hardware. 
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)
        simulator.set_publish_every_time_step(False)  # Usually, this should be set to False. Otherwise, simulation will be very slow and won't look like real time.

        # We'll use a super simple integration scheme (since we only update a dummy state)
        # and set the maximum timestep to correspond to roughly 40Hz 
        integration_scheme = "explicit_euler"
        time_step = 0.025
        #ResetIntegratorFromFlags(simulator, integration_scheme, time_step)

        # Run simulation
        simulator.Initialize()
        simulator.AdvanceTo(10.0)

        # Collect Data
        pose_log = pose_logger.FindLog(diagram_context)
        log_times  = pose_log.sample_times()
        pose_data = pose_log.data()
        print(pose_data.shape)

        vel_log = vel_estimate_logger.FindLog(diagram_context)
        vel_log_times = vel_log.sample_times()
        vel_data = vel_log.data()
        print(vel_data.shape)

        if show_plots:

            # Plot Data - First Half
            fig = plt.figure()
            ax_list1 = []

            for plt_index1 in range(6):
                ax_list1.append( fig.add_subplot(231+plt_index1) )
                plt.plot(log_times,pose_data[plt_index1,:])
                plt.title('Pose #' + str(plt_index1))

            fig2 = plt.figure()
            ax_list2 = []
            for plt_index2 in range(6):
                ax_list2.append( fig2.add_subplot(231+plt_index2) )
                plt.plot(vel_log_times[10:],vel_data[plt_index2,10:])
                plt.title('Vel #' + str(plt_index2))

            # plt.show()
            plt.savefig('command.png')

    #Wait at end
    input('Press ENTER to end python program.')