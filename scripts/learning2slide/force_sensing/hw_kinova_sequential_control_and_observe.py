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

from object_tracker_system import ObjectTrackerSystem

import sequence_sliding_object
import sequence_holding_object
import sequence_pause


""" Parameters """
show_toplevel_system_diagram = False    # Make a plot of the diagram for inner workings of the stationn
show_state_plots = True
save_state_plots = True

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
    
    # plant = builder.AddSystem(MultibodyPlant(time_step=time_step)) #Add plant to diagram builder
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    obj_tracker_system = builder.AddSystem(ObjectTrackerSystem(plant,scene_graph))


    ''' Command Sequence & Control '''
    pscs, controller = sequence_sliding_object.sliding_object()
    # pscs, controller = sequence_holding_object.holding_object()
    # pscs, controller = sequence_pause.pause()
    
    controller = builder.AddSystem(controller)
    controller.set_name("controller")
    controller.ConnectToStation(builder, station, time_step=time_step)
    
    
    ''' Connect Loggers '''
    arm_pose_logger = LogVectorOutput(station.GetOutputPort("measured_ee_pose"), builder)
    arm_pose_logger.set_name("arm_pose_logger")
    arm_twist_logger = LogVectorOutput(station.GetOutputPort("measured_ee_twist"), builder)
    arm_twist_logger.set_name("arm_twist_logger")
    arm_wrench_logger = LogVectorOutput(station.GetOutputPort("measured_ee_wrench"), builder)
    arm_wrench_logger.set_name("arm_wrench_logger")
    ee_command_logger = LogVectorOutput(controller.GetOutputPort("ee_command"), builder)
    ee_command_logger.set_name("ee_command_logger")
    object_pose_logger = LogVectorOutput(obj_tracker_system.GetOutputPort("measured_object_pose"), builder)
    object_pose_logger.set_name("object_pose_logger")
        
    ''' Build Diagram '''
    diagram = builder.Build() # build system diagram
    diagram.set_name("toplevel_system_diagram")
    diagram_context = diagram.CreateDefaultContext()

    # Set initial pose and vectors
    obj_tracker_system.SetInitialObjectState(diagram_context)

    if show_toplevel_system_diagram: # Show the overall system diagram
        plt.figure()
        plot_system_graphviz(diagram,max_depth=1)
        plt.show()


    ''' Simulation Sequence '''
    station.go_home(name="Home") # Set default arm positions
    
    simulator = Simulator(diagram, diagram_context)
    obj_tracker_system.context = obj_tracker_system.plant.GetMyMutableContextFromRoot(diagram_context)
    
    simulator.set_publish_every_time_step(False)

    integration_scheme = "explicit_euler"
    ResetIntegratorFromFlags(simulator, integration_scheme, time_step)
    
    simulator.Initialize()
    simulator.AdvanceTo(controller.cs.total_duration())


    ''' Collect Data '''
    pose_log = arm_pose_logger.FindLog(diagram_context)
    twist_log = arm_twist_logger.FindLog(diagram_context)
    wrench_log = arm_wrench_logger.FindLog(diagram_context)
    ee_command_log = ee_command_logger.FindLog(diagram_context)
    object_log = object_pose_logger.FindLog(diagram_context)
    
    if show_state_plots:
        xmin = 35
        xmax = 50
        # pose_fig = plt.figure(figsize=(14,8))
        # pose_ax_list = []
        # for i in range(6):
        #     pose_ax_list.append(pose_fig.add_subplot(231+i) )
        #     plt.plot(pose_log.sample_times(),pose_log.data()[i,:])
        #     plt.title('Pose #' + str(i))
            
        # if save_state_plots:
        #     plt.savefig('slide_data/pose_data.png', bbox_inches='tight')

        twist_fig = plt.figure(figsize=(14,8))
        twist_ax_list = []
        for i in range(6):
            twist_ax_list.append(twist_fig.add_subplot(231+i) )
            # plt.plot(ee_command_log.sample_times(),ee_command_log.data()[i,:])
            ax = plt.gca()
            ax.set_xlim([xmin, xmax])
            ax.grid(which = "both")
            ax.minorticks_on()
            ax.tick_params(which = "minor", bottom = False, left = False)
            plt.plot(twist_log.sample_times(),twist_log.data()[i,:])
            plt.title('Twist #'+ str(i))
            
        if save_state_plots:
            plt.savefig('slide_data/twist_data.png', bbox_inches='tight')
                
        wrench_fig = plt.figure(figsize=(14,8))
        wrench_ax_list = []
        for i in range(6):
            wrench_ax_list.append(wrench_fig.add_subplot(231+i) )
            ax = plt.gca()
            ax.set_xlim([xmin, xmax])
            ax.grid(which = "both")
            ax.minorticks_on()
            ax.tick_params(which = "minor", bottom = False, left = False)
            plt.plot(wrench_log.sample_times(),wrench_log.data()[i,:])
            plt.title('Wrench #' + str(i))
            
        if save_state_plots:
            plt.savefig('slide_data/wrench_data.png', bbox_inches='tight')
            
        object_fig = plt.figure(figsize=(14,8))
        object_ax_list = []
        for i in range(6):
            object_ax_list.append(object_fig.add_subplot(231+i) )
            ax = plt.gca()
            ax.set_xlim([xmin, xmax])
            ax.grid(which = "both")
            ax.minorticks_on()
            ax.tick_params(which = "minor", bottom = False, left = False)
            plt.plot(object_log.sample_times(),object_log.data()[i,:])
            plt.title('Object Pose #' + str(i))
            
        if save_state_plots:
            plt.savefig('slide_data/object_data.png', bbox_inches='tight')
            
        plt.show()
    
    print("")
    print("Target control frequency: %s Hz" % (1/time_step))
    print("Actual control frequency: %s Hz" % (1/time_step * simulator.get_actual_realtime_rate()))

