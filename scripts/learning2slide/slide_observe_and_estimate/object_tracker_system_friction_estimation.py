"""

object_tracker_system.py
Description:
    This script contains the class definition of object tracker system (Leaf System)
    and the functions associated with the system
    Detects object pose via apriltag detector, and estimates friction coefficient each
    timesteps

"""

""" Imports """
import sys
sys.path.append('../')

import numpy as np
import cv2

import pyrealsense2 as rs

from dt_apriltags import Detector

from pydrake.all import *
from pydrake.all import (LeafSystem, RigidTransform, RotationMatrix, RollPitchYaw)


class ObjectTrackerSystem(LeafSystem):
    def __init__(self,time_step=0.1):
        """
        ObjectTrackerSystem
        Usage:
            ObjectTrackerSystem(time_step)
        """
        LeafSystem.__init__(self)
        self.set_name("object_tracker_system")
        self.time_step = time_step

        self.SetupInputPorts()
        self.SetupOutputPorts()
        
        self.SetupEnvironment()
        self.SetupObject()
        self.SetupCamera()
        self.SetupAprilTagTracker()
        
        self.object_pose_log = [np.zeros(6)]
        
        self.ee_twist_log = [0]
        
        self.ee_wrench_x_log = [0]
        self.ee_wrench_y_log = [0]
        
        self.ee_wrench_x_ref = []
        self.ee_wrench_y_ref = []
        
        self.acceleration_log = [0]
        
        self.friction_coefficient_log = [0]
        self.friction_coefficient_est = []


    def SetupInputPorts(self):
        """
        SetupOutputPorts
        Description:
            SetupInputPorts, should be connected to kinova station
        """
        self.ee_twist_port = self.DeclareVectorInputPort( # <- from station
            "ee_twist", # to compute the prev_acceleration
            BasicVector(6))
        
        self.ee_wrench_port = self.DeclareVectorInputPort( # <- from station
            "ee_wrench", # to compute the friction coefficients
            BasicVector(6))
        
        self.gripper_position_port = self.DeclareVectorInputPort( # <- from station
            "gripper_position", # to determine before/after release
            BasicVector(1))


    def SetupOutputPorts(self):
        """
        SetupOutputPorts
        Description:
            SetupOutputPorts
        """
        self.DeclareVectorOutputPort( # -> detected object pose -> plot/data
            "measured_object_pose",
            BasicVector(6),
            self.DetectObjectPose,
            {self.time_ticket()}) # update each timestep
        
        self.DeclareVectorOutputPort( # -> estimated friction coefficient -> control?
            "estimated_friction_coefficient",
            BasicVector(1),
            self.EstimateFrictionCoefficient_AA,
            {self.time_ticket()}) # update each timestep


    def SetupEnvironment(self):
        """
        SetupEnvironment
        Description:
            Defines some useful parameters
        """
        self.gravity = 9.8067
        self.arm_mass = 7.2 # - 1.697
        self.gripper_mass = 0.9


    def SetupObject(self):
        """
        SetupObject
        Description:
            Defines an object-related parameters
        """
        self.wheel_radius = 0.01675
        self.object_mass = 0.56237 # Empty Eisco Minicar, 2.2oz + 500g weight


    def SetupCamera(self):
        """
        SetupCamera
        Description:
            Defines a camera pose in world frame using the known intrinsics/extrinsics
            which can be found by running calibration scripts
        """
        self.camera_params = [ 386.738, 386.738, 321.281, 238.221 ]
        # These are the camera's focal length and focal center.
        # Received from running aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        
        with open('/home/krutledg/kinova/kinova-arm/scripts/camera_calibration/camera_extrinsics.npy', 'rb') as f:
            R_world_cam = np.load(f)
            p_world_cam = np.load(f)
        R_world_cam = RotationMatrix(R_world_cam)
        self.X_world_cam = RigidTransform(R_world_cam, p_world_cam.transpose())


    def SetupAprilTagTracker(self):
        """
        SetupAprilTagTracker
        Description:
            Defines an apriltag tracker which will attempt to find one of the tags on the object
        """
        self.rs_pipeline = rs.pipeline()
        self.rs_config = rs.config()

        self.rs_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Enable depth stream
        self.rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) # Enable color stream

        # Define April Tag Detector
        self.at_detector = Detector(
            families='tagStandard41h12',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0)

        self.tag_size = 0.016 # Measured on the tag.
        # (See documentation on how to measure april tag sizes. A link that can help explain:
        # https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide)
        
        # Start streaming
        self.rs_pipeline.start(self.rs_config)


    def DetectObjectPose(self,diagram_context,output):
        """
        DetectObjectPose
        Description:
            This function attempts to estimate the pose of the block
            using the pose of one of the detected tags.
        """

        # Wait for a coherent pair of frames: depth and color
        frames = self.rs_pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            # Not enough frame data was received, output the last pose
            output.SetFromVector(self.object_pose_log[-1])
            return

        # Print whether or not detector detects anything.
        color_image = np.asanyarray(color_frame.get_data())
        gray_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
        atag = self.at_detector.detect(gray_image,
                                       estimate_tag_pose=True,
                                       camera_params=self.camera_params,
                                       tag_size=self.tag_size)

        # Make the default be the last pose we output
        current_pose = self.object_pose_log[-1] 
        
        # If an apriltag is detected, process detection information
        if atag:
            R_cam_atag = RotationMatrix(atag[0].pose_R)
            p_cam_atag = atag[0].pose_t
            X_cam_atag = RigidTransform(R_cam_atag, p_cam_atag)

            X_world_object = self.X_world_cam.multiply(X_cam_atag)

            current_pose = np.hstack([RollPitchYaw(X_world_object.rotation()).vector(),
                                      X_world_object.translation().reshape((3,))])
            
            output.SetFromVector(current_pose)
            self.object_pose_log.append(current_pose)

            # print(f"current_pose: {current_pose}")

        output.SetFromVector(current_pose) # Set The Output of the block to be the current pose


    def EstimateFrictionCoefficient_AA(self,diagram_context,output):
        """
        EstimateFrictionCoefficient_AA (Acceleration Approximation method)
        Description:
            Estimate the friction coefficient each timestep
        """
        # Obtain the current timestep in the sequence
        t = diagram_context.get_time()
        
        # Parameter to be updated and returned
        mvavg_friction_coefficient = 0
        
        # Load Wrench X, Wrench Y values
        current_ee_wrench_x, current_ee_wrench_y = -self.ee_wrench_port.Eval(diagram_context)[4:]
        self.ee_wrench_x_log.append(0.5*current_ee_wrench_x + 0.5*self.ee_wrench_x_log[-1])
        self.ee_wrench_y_log.append(0.5*current_ee_wrench_y + 0.5*self.ee_wrench_y_log[-1])

        # Load Twist values
        current_ee_twist = self.ee_twist_port.Eval(diagram_context)[4]
        prev_ee_twist = self.ee_twist_log[-1]
        self.ee_twist_log.append(current_ee_twist)

        # Determine Total Mass based on the Gripper Position
        current_gripper_pos = self.gripper_position_port.Eval(diagram_context)
        if current_gripper_pos > 0.1:
            total_mass = self.object_mass + self.arm_mass + self.gripper_mass
        else: 
            total_mass = self.arm_mass + self.gripper_mass
        
        # Generate Reference Characteristic
        if not self.ee_wrench_x_ref and t > 131.0: 
            self.ee_wrench_x_ref = [0.1*sum(x) for x in 
                zip(self.ee_wrench_x_log[300:399], self.ee_wrench_x_log[400:499], self.ee_wrench_x_log[500:599],
                    self.ee_wrench_x_log[600:699], self.ee_wrench_x_log[700:799], self.ee_wrench_x_log[800:899],
                    self.ee_wrench_x_log[900:999], self.ee_wrench_x_log[1000:1099], self.ee_wrench_x_log[1100:1199],
                    self.ee_wrench_x_log[1200:1299])]
        if not self.ee_wrench_y_ref and t > 132.0:
            self.ee_wrench_y_ref = [0.1*sum(y) for y in 
                zip(self.ee_wrench_y_log[300:399], self.ee_wrench_y_log[400:499], self.ee_wrench_y_log[500:599],
                    self.ee_wrench_y_log[600:699], self.ee_wrench_y_log[700:799], self.ee_wrench_y_log[800:899],
                    self.ee_wrench_y_log[900:999], self.ee_wrench_y_log[1000:1099], self.ee_wrench_y_log[1100:1199],
                    self.ee_wrench_y_log[1200:1299])]
        
        # Compute Friction Coefficient while Dragging
        if t > 135.20 and t< 175.20:
            index = (10*round(t - 135.0)) % 100
            
            # Compute the Force Exerted by End Effector
            prev_ee_wrench_x = (0.5*(self.ee_wrench_x_log[-1] - self.ee_wrench_x_ref[index-1])
                                + (0.5*self.ee_wrench_x_log[-2] - self.ee_wrench_x_ref[index-2]))
            prev_ee_wrench_y = (0.5*(self.ee_wrench_y_log[-1] - self.ee_wrench_y_ref[index-1])
                                + (0.5*self.ee_wrench_y_log[-2] - self.ee_wrench_y_ref[index-2]))
            
            # Acceleration Approximation
            prev_acceleration = (current_ee_twist - prev_ee_twist)/self.time_step
            mvavg_prev_acceleration = 0.5*prev_acceleration + 0.5*self.acceleration_log[-1]
            self.acceleration_log.append(prev_acceleration)

            # Friction Coefficient Estimation
            friction_coefficient = ((prev_ee_wrench_x/total_mass - mvavg_prev_acceleration)/
                                    (self.gravity - prev_ee_wrench_y/total_mass))
            mvavg_friction_coefficient = 0.5*friction_coefficient + 0.5*self.friction_coefficient_log[-1]

        # Generate Initial Estimation
        if not self.friction_coefficient_est and t > 186.0:
            self.friction_coefficient_est = [0.2*sum(f) for f in 
                zip(self.friction_coefficient_log[1350:1449], self.friction_coefficient_log[1450:1549], self.friction_coefficient_log[1550:1649],
                    self.friction_coefficient_log[1650:1749], self.friction_coefficient_log[1750:1849])]
        
        # During the Actual Sliding Sequence
        if t > 190.20 and t < 195.20:
            index = 10*round(t - 190.0)
            
            # Compute the Force Exerted by End Effector
            prev_ee_wrench_x = (0.5*(self.ee_wrench_x_log[-1] - self.ee_wrench_x_ref[index-1])
                                + (0.5*self.ee_wrench_x_log[-2] - self.ee_wrench_x_ref[index-2]))
            prev_ee_wrench_y = (0.5*(self.ee_wrench_y_log[-1] - self.ee_wrench_y_ref[index-1])
                                + (0.5*self.ee_wrench_y_log[-2] - self.ee_wrench_y_ref[index-2]))
            
            # Acceleration Approximation
            prev_acceleration = (current_ee_twist - prev_ee_twist)/self.time_step
            mvavg_prev_acceleration = 0.5*prev_acceleration + 0.5*self.acceleration_log[-1]
            self.acceleration_log.append(prev_acceleration)

            # Friction Coefficient Estimation
            friction_coefficient = ((prev_ee_wrench_x/total_mass - mvavg_prev_acceleration)/
                                    (self.gravity - prev_ee_wrench_y/total_mass))
            mvavg_friction_coefficient = (0.1*friction_coefficient + 0.1*self.friction_coefficient_log[-1]
                                            + 0.8*self.friction_coefficient_est[index])
        
        if t > 186.0 and t < 186.2:
            print(self.ee_wrench_x_ref)
            print(self.ee_wrench_y_ref)
            print(self.acceleration_log)
            
        self.friction_coefficient_log.append(mvavg_friction_coefficient)
        output.SetFromVector([mvavg_friction_coefficient])

    # def EstimateFrictionCoefficient_AA(self,diagram_context,output):
    #     """
    #     EstimateFrictionCoefficient
    #     Description:
    #         Estimate the friction coefficient each timestep
    #     """
    #     current_ee_twist = self.ee_twist_port.Eval(diagram_context)[4]
    #     prev_ee_twist = self.ee_twist_log[-1]
    #     self.ee_twist_log.append(current_ee_twist)
        
    #     prev_acceleration = (current_ee_twist - prev_ee_twist)/self.time_step
    #     mvavg_prev_acceleration = 0.5*prev_acceleration + 0.5*self.acceleration_log[-1]
        
    #     current_gripper_pos = self.gripper_position_port.Eval(diagram_context)
    #     if current_gripper_pos > 0.1:
    #         total_mass = self.object_mass + self.arm_mass + self.gripper_mass
    #     else: 
    #         total_mass = self.arm_mass + self.gripper_mass

    #     current_ee_wrench_x, current_ee_wrench_y = -self.ee_wrench_port.Eval(diagram_context)[4:]
    #     prev_ee_wrench_x = 0.5*self.ee_wrench_x_log[-1] + 0.5*self.ee_wrench_x_log[-1]
    #     prev_ee_wrench_y = 0.5*self.ee_wrench_y_log[-1] + 0.5*self.ee_wrench_y_log[-1]
    #     self.ee_wrench_x_log.append(current_ee_wrench_x)
    #     self.ee_wrench_y_log.append(current_ee_wrench_y)
        
    #     friction_coefficient = ((prev_ee_wrench_x/total_mass - mvavg_prev_acceleration)/
    #                             (self.gravity - prev_ee_wrench_y/total_mass))
    #     mvavg_friction_coefficient = 1.0*friction_coefficient + 0.0*self.friction_coefficient_log[-1]

    #     self.acceleration_log.append(prev_acceleration)
    #     self.friction_coefficient_log.append(mvavg_friction_coefficient)
        
    #     output.SetFromVector([mvavg_friction_coefficient])
        
        
    # def EstimateFrictionCoefficient_LS(self,diagram_context,output):
    #     """
    #     EstimateFrictionCoefficient_LS (Least Square Method)
    #     Description:
    #         Estimate the friction coefficient each timestep
    #     """
    #     current_ee_wrench_x, current_ee_wrench_y = -self.ee_wrench_port.Eval(diagram_context)[4:]
    #     current_gripper_pos = self.gripper_position_port.Eval(diagram_context)

    #     if current_gripper_pos > 0.1:
    #         total_mass = self.object_mass + self.arm_mass + self.gripper_mass
    #     else: 
    #         total_mass = self.arm_mass + self.gripper_mass
        
    #     A = np.zeros([1,2])
    #     A[0,0] = 1
    #     A[0,1] = self.gravity - current_ee_wrench_y/total_mass
        
    #     b = np.zeros([1,])
    #     b[0,] = current_ee_wrench_x/total_mass
        
    #     acceleration, friction_coefficient = np.linalg.lstsq(A, b, rcond=-1)[0]
        
    #     self.ee_wrench_x_log.append(current_ee_wrench_x)
    #     self.ee_wrench_y_log.append(current_ee_wrench_y)
    #     self.acceleration_log.append(acceleration)
    #     self.friction_coefficient_log.append(friction_coefficient)

    #     output.SetFromVector([friction_coefficient])
