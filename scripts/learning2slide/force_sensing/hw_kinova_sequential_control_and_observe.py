"""

hw_kinova_sequential_control_with_apriltag.py
Description:
    This scripts controls the kinova arm using the predetermined command sequence
    along with specifically tuned controller for the sequence
    
    sequence script should be imported (e.g., sequence_sliding_object.py,..)
    
    This script also detect apriltag simultaneously, tracking the location and orientation
    of the object with apriltag
    
"""

""" Imports """
import sys
sys.path.append('../') 

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2

from pydrake.all import *
from pydrake.all import (AddMultibodyPlantSceneGraph, Meshcat, MeshcatVisualizer,
                            DiagramBuilder, Simulator, MeshcatPointCloudVisualizer, LogVectorOutput)

from kinova_drake.kinova_station import (KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget)
from kinova_drake.controllers import (PSCommandSequenceController, PSCommandSequence, PartialStateCommand)
from kinova_drake.observers import CameraViewer

from object_tracker_system import ObjectTrackerSystem

import sequence_speed_limit_test
import sequence_sliding_object
import sequence_holding_object
import sequence_pause


""" Parameters """
show_toplevel_system_diagram = False    # Make a plot of the diagram for inner workings of the stationn
show_state_plots = True
save_state_plots = True
save_state_logs = True

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
    obj_tracker_system = builder.AddSystem(ObjectTrackerSystem(time_step))

    builder.Connect(station.GetOutputPort("measured_ee_twist"),
                    obj_tracker_system.GetInputPort("ee_twist"))
    builder.Connect(station.GetOutputPort("measured_ee_wrench"),
                    obj_tracker_system.GetInputPort("ee_wrench"))
    builder.Connect(station.GetOutputPort("measured_gripper_position"),
                    obj_tracker_system.GetInputPort("gripper_position"))
    
    ''' Command Sequence & Control '''
    # pscs, controller = sequence_speed_limit_test.command_sequence()
    pscs, controller = sequence_sliding_object.command_sequence()
    # pscs, controller = sequence_holding_object.command_sequence()
    # pscs, controller = sequence_pause.command_sequence()
    
    controller = builder.AddSystem(controller)
    controller.set_name("controller")
    controller.ConnectToStation(builder, station, time_step=time_step)
    
    
    ''' Connect Loggers '''
    ee_twist_logger = LogVectorOutput(station.GetOutputPort("measured_ee_twist"), builder)
    ee_twist_logger.set_name("ee_twist_logger")
    ee_wrench_logger = LogVectorOutput(station.GetOutputPort("measured_ee_wrench"), builder)
    ee_wrench_logger.set_name("ee_wrench_logger")
    ee_command_logger = LogVectorOutput(controller.GetOutputPort("ee_command"), builder)
    ee_command_logger.set_name("ee_command_logger")
    object_pose_logger = LogVectorOutput(obj_tracker_system.GetOutputPort("measured_object_pose"), builder)
    object_pose_logger.set_name("object_pose_logger")
    friction_coefficient_logger = LogVectorOutput(obj_tracker_system.GetOutputPort("estimated_friction_coefficient"), builder)
    friction_coefficient_logger.set_name("friction_coefficient_logger")

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
    
    simulator.set_publish_every_time_step(False)

    integration_scheme = "explicit_euler"
    ResetIntegratorFromFlags(simulator, integration_scheme, time_step)
    
    simulator.Initialize()
    simulator.AdvanceTo(controller.cs.total_duration())

    print("")
    print("Target control frequency: %s Hz" % (1/time_step))
    print("Actual control frequency: %s Hz" % (1/time_step * simulator.get_actual_realtime_rate()))


    ''' Collect Data '''
    ee_twist_log = ee_twist_logger.FindLog(diagram_context)
    ee_wrench_log = ee_wrench_logger.FindLog(diagram_context)
    ee_command_log = ee_command_logger.FindLog(diagram_context)
    object_pose_log = object_pose_logger.FindLog(diagram_context)
    friction_coefficient_log = friction_coefficient_logger.FindLog(diagram_context)
    
    if save_state_logs:
        df = pd.DataFrame( np.vstack((object_pose_log.sample_times(), object_pose_log.data()[4,:])) )
        df.to_csv('slide_data/logdata_object_pose.csv', index=False)
        df = pd.DataFrame( np.vstack((ee_twist_log.sample_times(), ee_twist_log.data()[4,:])) )
        df.to_csv('slide_data/logdata_ee_twist.csv', index=False)
        df = pd.DataFrame( np.vstack((ee_wrench_log.sample_times(), ee_wrench_log.data()[4,:])) )
        df.to_csv('slide_data/logdata_ee_wrench.csv', index=False)
        df = pd.DataFrame( np.vstack((friction_coefficient_log.sample_times(), friction_coefficient_log.data())) )
        df.to_csv('slide_data/logdata_friction_coefficient.csv', index=False)
    
        # with open('slide_data/logdata_object_pose.npy', 'wb') as f:
        #     np.save(f, (object_pose_log.sample_times(), object_pose_log.data()[4,:]))
        # with open('slide_data/logdata_arm_twist.npy', 'wb') as f:
        #     np.save(f, (ee_twist_log.sample_times(), ee_twist_log.data()[4,:]))
        # with open('slide_data/logdata_arm_wrench.npy', 'wb') as f:
        #     np.save(f, (ee_wrench_log.sample_times(), ee_wrench_log.data()[4,:]))
        
    if show_state_plots:
        xmin = 32
        xmax = 40
    
        ee_twist_fig = plt.figure(figsize=(14,8))
        ee_twist_ax_list = []
        for i in range(6):
            ee_twist_ax_list.append(ee_twist_fig.add_subplot(231+i) )
            # plt.plot(ee_command_log.sample_times(),ee_command_log.data()[i,:])
            ax = plt.gca()
            ax.set_xlim([xmin, xmax])
            ax.grid(which = "both")
            ax.minorticks_on()
            ax.tick_params(which = "minor", bottom = False, left = False)
            plt.plot(ee_twist_log.sample_times(),ee_twist_log.data()[i,:])
            plt.title('Twist #'+ str(i))
            
        if save_state_plots:
            plt.savefig('slide_data/ee_twist_data_plot.png', bbox_inches='tight')
                
        ee_wrench_fig = plt.figure(figsize=(14,8))
        ee_wrench_ax_list = []
        for i in range(6):
            ee_wrench_ax_list.append(ee_wrench_fig.add_subplot(231+i) )
            ax = plt.gca()
            ax.set_xlim([xmin, xmax])
            ax.grid(which = "both")
            ax.minorticks_on()
            ax.tick_params(which = "minor", bottom = False, left = False)
            plt.plot(ee_wrench_log.sample_times(),ee_wrench_log.data()[i,:])
            plt.title('Wrench #' + str(i))
            
        if save_state_plots:
            plt.savefig('slide_data/ee_wrench_data_plot.png', bbox_inches='tight')
            
        object_pose_fig = plt.figure(figsize=(14,8))
        object_pose_ax_list = []
        for i in range(6):
            object_pose_ax_list.append(object_pose_fig.add_subplot(231+i) )
            ax = plt.gca()
            ax.set_xlim([xmin, xmax])
            ax.grid(which = "both")
            ax.minorticks_on()
            ax.tick_params(which = "minor", bottom = False, left = False)
            plt.plot(object_pose_log.sample_times(),object_pose_log.data()[i,:])
            plt.title('Object Pose #' + str(i))
        
        if save_state_plots:
            plt.savefig('slide_data/object_pose_data_plot.png', bbox_inches='tight')
            
        friction_coefficient_fig = plt.figure()
        ax = plt.gca()
        ax.set_xlim([xmin, xmax])
        ax.grid(which = "both")
        ax.minorticks_on()
        ax.tick_params(which = "minor", bottom = False, left = False)
        plt.plot(friction_coefficient_log.sample_times(),friction_coefficient_log.data()[0,:])
        plt.title('Estimated Friction Coefficient')
        if save_state_plots:
            plt.savefig('slide_data/friction_coefficient_data_plot.png', bbox_inches='tight')
            
        plt.show()
    


