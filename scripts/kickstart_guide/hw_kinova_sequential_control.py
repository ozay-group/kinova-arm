"""

hw_kinova_sequential_control.py
Description:
    This scripts controls the kinova arm using the predetermined command sequence
    along with specifically tuned controller for the sequence
    
    sequence script should be imported (e.g., sequence_sliding_object.py,..)
    
"""

""" Imports """
import sys
sys.path.append('../') # setting path for imports

import numpy as np
import matplotlib.pyplot as plt
import cv2

from pydrake.all import *
from pydrake.all import (AddMultibodyPlantSceneGraph, Meshcat, MeshcatVisualizer,
                            DiagramBuilder, Simulator, MeshcatPointCloudVisualizer, LogVectorOutput)

from kinova_drake.kinova_station import (KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget)
from kinova_drake.controllers import (PSCommandSequenceController, PSCommandSequence, PartialStateCommand)
from kinova_drake.observers import CameraViewer

import sequence_sliding_object

""" Parameters """
show_toplevel_system_diagram = False    # Make a plot of the diagram for inner workings of the stationn
show_state_plots = True

n_dof = 6                               # number of degrees of freedom of the arm
gripper_type = "2f_85"                  # which gripper to use (hande or 2f_85)
time_step = 0.1                         # time step size (seconds)


""" Main Script """
with KinovaStationHardwareInterface(n_dof) as station:
# Note that unlike the simulation station, the hardware station needs to be used within a 'with' block.
# This is to allow for cleaner error handling, since the connection with the hardware needs to be
# closed properly even if there is an error (e.g. KeyboardInterrupt) during execution.

    ''' Connect Station '''
    builder = DiagramBuilder()
    station = builder.AddSystem(station)
    

    ''' Command Sequence & Control '''
    # pscs, controller = sequence_sliding_object.command_sequence()
    # pscs, controller = sequence_holding_object.command_sequence()
    pscs, controller = sequence_sliding_object.command_sequence()
    
    controller = builder.AddSystem(controller)
    controller.set_name("controller")
    controller.ConnectToStation(builder, station, time_step=time_step)
    
    
    ''' Connect Loggers '''
    q_logger = LogVectorOutput(station.GetOutputPort("measured_arm_position"), builder)
    q_logger.set_name("arm_position_logger")
    qd_logger = LogVectorOutput(station.GetOutputPort("measured_arm_velocity"), builder)
    qd_logger.set_name("arm_velocity_logger")
    tau_logger = LogVectorOutput(station.GetOutputPort("measured_arm_torque"), builder)
    tau_logger.set_name("arm_torque_logger")

    pose_logger = LogVectorOutput(station.GetOutputPort("measured_ee_pose"), builder)
    pose_logger.set_name("pose_logger")
    twist_logger = LogVectorOutput(station.GetOutputPort("measured_ee_twist"), builder)
    twist_logger.set_name("twist_logger")
    wrench_logger = LogVectorOutput(station.GetOutputPort("measured_ee_wrench"), builder)
    wrench_logger.set_name("wrench_logger")

    #gp_logger = LogVectorOutput(station.GetOutputPort("measured_gripper_position"), builder)
    #gp_logger.set_name("gripper_position_logger")
    #gv_logger = LogVectorOutput(station.GetOutputPort("measured_gripper_velocity"), builder)
    #gv_logger.set_name("gripper_velocity_logger")
    
        
    ''' Build Diagram '''
    diagram = builder.Build() # build system diagram
    diagram.set_name("toplevel_system_diagram")
    diagram_context = diagram.CreateDefaultContext()

    if show_toplevel_system_diagram: # Show the overall system diagram
        plt.figure()
        plot_system_graphviz(diagram,max_depth=1)
        plt.show()


    ''' Simulation Sequence '''
    station.go_home(name="Home") # Set default arm positions
    
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    integration_scheme = "explicit_euler"
    ResetIntegratorFromFlags(simulator, integration_scheme, time_step)
    
    simulator.Initialize()
    simulator.AdvanceTo(controller.cs.total_duration())


    ''' Collect Data '''
    q_log = q_logger.FindLog(diagram_context)
    q_log_times = q_log.sample_times()
    q_log_data = q_log.data()
    print(q_log_data.shape)
    
    qd_log = qd_logger.FindLog(diagram_context)
    qd_log_times = qd_log.sample_times()
    qd_log_data = qd_log.data()
    print(qd_log_data.shape)
    
    tau_log = tau_logger.FindLog(diagram_context)
    tau_log_times = tau_log.sample_times()
    tau_log_data = tau_log.data()
    print(tau_log_data.shape)
    
    pose_log = pose_logger.FindLog(diagram_context)
    pose_log_times = pose_log.sample_times()
    pose_log_data = pose_log.data()
    print(pose_log_data.shape)
    
    twist_log = twist_logger.FindLog(diagram_context)
    twist_log_times = twist_log.sample_times()
    twist_log_data = twist_log.data()
    print(twist_log_data.shape)
    
    wrench_log = wrench_logger.FindLog(diagram_context)
    wrench_log_times = wrench_log.sample_times()
    wrench_log_data = wrench_log.data()
    print(wrench_log_data.shape)
    
    if show_state_plots:
        q_fig = plt.figure(figsize=(14,8))
        q_ax_list = []
        for i in range(6):
            q_ax_list.append(q_fig.add_subplot(231+i) )
            plt.plot(q_log_times,q_log_data[i,:])
            plt.title('q #' + str(i))
            
        qd_fig = plt.figure(figsize=(14,8))
        qd_ax_list = []
        for i in range(6):
            qd_ax_list.append(qd_fig.add_subplot(231+i) )
            plt.plot(qd_log_times,qd_log_data[i,:])
            plt.title('qd #' + str(i))
            
        tau_fig = plt.figure(figsize=(14,8))
        tau_ax_list = []
        for i in range(6):
            tau_ax_list.append(tau_fig.add_subplot(231+i) )
            plt.plot(tau_log_times,tau_log_data[i,:])
            plt.title('tau #' + str(i))
        
        pose_fig = plt.figure(figsize=(14,8))
        pose_ax_list = []
        for i in range(6):
            pose_ax_list.append(pose_fig.add_subplot(231+i) )
            plt.plot(pose_log_times,pose_log_data[i,:])
            plt.title('Pose #' + str(i))

        twist_fig = plt.figure(figsize=(14,8))
        twist_ax_list = []
        for i in range(6):
            twist_ax_list.append(twist_fig.add_subplot(231+i) )
            plt.plot(twist_log_times,twist_log_data[i,:])
            plt.title('Twist #' + str(i))
            
        wrench_fig = plt.figure(figsize=(14,8))
        wrench_ax_list = []
        for i in range(6):
            wrench_ax_list.append(wrench_fig.add_subplot(231+i) )
            plt.plot(wrench_log_times,wrench_log_data[i,:])
            plt.title('Wrench #' + str(i))
            
        plt.show()
    
    print("")
    print("Target control frequency: %s Hz" % (1/time_step))
    print("Actual control frequency: %s Hz" % (1/time_step * simulator.get_actual_realtime_rate()))

