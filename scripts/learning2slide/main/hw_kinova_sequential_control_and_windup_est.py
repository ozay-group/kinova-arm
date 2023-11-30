"""

hw_kinova_sequential_control_and_windup_est.py
Description:
    This scripts controls the kinova arm using the predetermined command sequence
    along with specifically tuned controller for the sequence
    
    sequence script should be imported (e.g., sequence_sliding_object.py,..)
    
    This script also detect apriltag simultaneously, tracking the location and orientation
    of the object with apriltag
    
    This script also attempts to estimate the windup coefficient of the pullback minicar
    
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

from leaf_systems.object_tracker_system_windup_estimation import ObjectTrackerSystem

import command_sequences.sequence_pull_back_and_release
import command_sequences.sequence_variable_pull_back

""" Function Definitions """
def six_dof_plots(data_log, title, xmin, xmax, save_state_plots=False, output_filename=None):
    """
    six_dof_plots
    Description:
        Plot and Save 6D vector data logs (e.g., Pose, Twist, Wrench)
    """
    fig = plt.figure(figsize=(14,8))
    ax_list = []
    for i in range(6):
        ax_list.append(fig.add_subplot(231+i) )
        ax = plt.gca()
        ax.set_xlim([xmin, xmax])
        ax.grid(which = "both")
        ax.minorticks_on()
        ax.tick_params(which = "minor", bottom = False, left = False)
        plt.plot(data_log.sample_times(),data_log.data()[i,:])
        plt.title(title + ' # ' + str(i))
        
    if save_state_plots:
        plt.savefig(output_filename, bbox_inches='tight')
        

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

    builder.Connect(station.GetOutputPort("measured_ee_pose"),
                    obj_tracker_system.GetInputPort("ee_pose"))
    builder.Connect(station.GetOutputPort("measured_ee_wrench"),
                    obj_tracker_system.GetInputPort("ee_wrench"))
    builder.Connect(station.GetOutputPort("measured_gripper_position"),
                    obj_tracker_system.GetInputPort("gripper_position"))

    ''' Command Sequence & Control '''
    pscs, controller = sequence_pull_back_and_release.command_sequence()
    # pscs, controller = sequence_variable_pull_back.command_sequence()
    
    controller = builder.AddSystem(controller)
    controller.set_name("controller")
    controller.ConnectToStation(builder, station, time_step=time_step)
    
    
    ''' Connect Loggers '''
    ee_pose_logger = LogVectorOutput(station.GetOutputPort("measured_ee_pose"), builder)
    ee_pose_logger.set_name("ee_pose_logger")
    ee_twist_logger = LogVectorOutput(station.GetOutputPort("measured_ee_twist"), builder)
    ee_twist_logger.set_name("ee_twist_logger")
    ee_wrench_logger = LogVectorOutput(station.GetOutputPort("measured_ee_wrench"), builder)
    ee_wrench_logger.set_name("ee_wrench_logger")
    ee_command_logger = LogVectorOutput(controller.GetOutputPort("ee_command"), builder)
    ee_command_logger.set_name("ee_command_logger")
    object_pose_logger = LogVectorOutput(obj_tracker_system.GetOutputPort("measured_object_pose"), builder)
    object_pose_logger.set_name("object_pose_logger")
    spring_coefficient_logger = LogVectorOutput(obj_tracker_system.GetOutputPort("estimated_spring_coefficient"), builder)
    spring_coefficient_logger.set_name("spring_coefficient_logger")

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

    print("")
    print("Target control frequency: %s Hz" % (1/time_step))
    print("Actual control frequency: %s Hz" % (1/time_step * simulator.get_actual_realtime_rate()))

    print(f"Reference Force: {obj_tracker_system.reference_force}")
    print(f"Object Origin: {obj_tracker_system.origin}")
    print(f"Spring Coefficient: {obj_tracker_system.spring_coefficient}")
    

    ''' Collect Data '''
    ee_pose_log = ee_pose_logger.FindLog(diagram_context)
    ee_twist_log = ee_twist_logger.FindLog(diagram_context)
    ee_wrench_log = ee_wrench_logger.FindLog(diagram_context)
    ee_command_log = ee_command_logger.FindLog(diagram_context)
    object_pose_log = object_pose_logger.FindLog(diagram_context)
    spring_coefficient_log = spring_coefficient_logger.FindLog(diagram_context)
    
    if save_state_logs:
        df = pd.DataFrame( np.vstack((object_pose_log.sample_times(), object_pose_log.data()[4,:])) )
        df.to_csv('slide_data/logdata_object_pose.csv', index=False)
        df = pd.DataFrame( np.vstack((ee_pose_log.sample_times(), ee_pose_log.data()[4,:])) )
        df.to_csv('slide_data/logdata_ee_pose.csv', index=False)
        df = pd.DataFrame( np.vstack((ee_twist_log.sample_times(), ee_twist_log.data()[4,:])) )
        df.to_csv('slide_data/logdata_ee_twist.csv', index=False)
        df = pd.DataFrame( np.vstack((ee_wrench_log.sample_times(), ee_wrench_log.data()[4,:])) )
        df.to_csv('slide_data/logdata_ee_wrench.csv', index=False)
        df = pd.DataFrame( np.vstack((spring_coefficient_log.sample_times(), spring_coefficient_log.data())) )
        df.to_csv('slide_data/logdata_spring_coefficient.csv', index=False)
    
        # with open('slide_data/logdata_object_pose.npy', 'wb') as f:
        #     np.save(f, (object_pose_log.sample_times(), object_pose_log.data()[4,:]))
        # with open('slide_data/logdata_arm_twist.npy', 'wb') as f:
        #     np.save(f, (ee_twist_log.sample_times(), ee_twist_log.data()[4,:]))
        # with open('slide_data/logdata_arm_wrench.npy', 'wb') as f:
        #     np.save(f, (ee_wrench_log.sample_times(), ee_wrench_log.data()[4,:]))

    if show_state_plots:
        xmin = 0
        xmax = 100
        
        six_dof_plots(ee_pose_log, "EE Pose", xmin, xmax,
                      save_state_plots, 'slide_data/ee_pose_data_plot.png')
        six_dof_plots(ee_twist_log, "EE Twist", xmin, xmax,
                      save_state_plots, 'slide_data/ee_twist_data_plot.png')
        six_dof_plots(ee_wrench_log, "EE Wrench", xmin, xmax,
                      save_state_plots, 'slide_data/ee_wrench_data_plot.png')
        six_dof_plots(object_pose_log, "Object Pose", xmin, xmax,
                      save_state_plots, 'slide_data/object_pose_data_plot.png')


        spring_coefficient_fig = plt.figure()
        ax = plt.gca()
        ax.set_xlim([xmin, xmax])
        ax.grid(which = "both")
        ax.minorticks_on()
        ax.tick_params(which = "minor", bottom = False, left = False)
        plt.plot(spring_coefficient_log.sample_times(),spring_coefficient_log.data()[0,:])
        plt.title('Estimated windup Coefficient')
        if save_state_plots:
            plt.savefig('slide_data/spring_coefficient_data_plot.png', bbox_inches='tight')
            
        plt.show()
    


