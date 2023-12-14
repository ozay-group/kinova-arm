"""

object_tracker_system_windup_estimation.py
Description:
    This script contains the class definition of object tracker system (Leaf System)
    and the functions associated with the system
    Detects object pose via apriltag detector, and estimates spring coefficient each
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
        self.ee_force_log = [0]


    def SetupInputPorts(self):
        """
        SetupOutputPorts
        Description:
            SetupInputPorts, should be connected to kinova station
        """
        self.ee_pose_port = self.DeclareVectorInputPort( # <- from station
            "ee_pose",
            BasicVector(6))
        
        self.ee_wrench_port = self.DeclareVectorInputPort( # <- from station
            "ee_wrench",
            BasicVector(6))
        
        self.gripper_position_port = self.DeclareVectorInputPort( # <- from station
            "gripper_position",
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
        
        self.DeclareVectorOutputPort( # -> estimated spring coefficient -> control?
            "estimated_spring_coefficient",
            BasicVector(1),
            self.EstimateSpringCoefficient,
            {self.time_ticket()}) # update each timestep


    def SetupEnvironment(self):
        """
        SetupEnvironment
        Description:
            Defines some useful parameters
        """
        self.gravity = 9.8067
        self.arm_mass = 7.2
        self.gripper_mass = 0.9
        self.origin = 0
        self.origin_log = []
        self.reference_force = 0
        self.reference_force_log = []


    def SetupObject(self):
        """
        SetupObject
        Description:
            Defines an object-related parameters
        """
        self.wheel_radius = 0.01
        self.object_mass = 0.226 # Red Pullback Car, 8oz
        self.rolling_friction_coefficient = 0.0123
        self.spring_coefficient = 0
        self.spring_coefficient_log = []


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

        self.tag_size = 0.025 # Measured on the tag.
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


    def EstimateSpringCoefficient(self,diagram_context,output):
        """
        EstimateSpringCoefficient
        Description:
            Estimate the spring coefficient each timestep
        """
        current_ee_force = -self.ee_wrench_port.Eval(diagram_context)[4]
        self.ee_force_log.append(current_ee_force)
        
        current_object_pose = self.object_pose_log[-1][4]
        
        spring_coefficient = 0
        
        t = diagram_context.get_time()
        
        # Measure the object origin
        if 15 < t and t < 25:
            self.origin_log.append(current_object_pose)
        if 25 < t and t < 26:
            self.origin = sum(self.origin_log)/len(self.origin_log)
        
        # Measure the reference force
        if 40 < t and t < 50:
            self.reference_force_log.append(current_ee_force)
        if 50 < t and t < 50.5:
            self.reference_force = sum(self.reference_force_log)/len(self.reference_force_log)
        
        # Measure the spring coefficient
        if 70 < t and t < 100:
            pullback_dist = self.origin - current_object_pose
            exerted_contact_force = current_ee_force - self.reference_force
            
            spring_coefficient = (1/pullback_dist)*(exerted_contact_force + 
                                    self.rolling_friction_coefficient*self.object_mass*self.gravity/self.wheel_radius)
        if 100 < t and t < 100.5:
            self.spring_coefficient = sum(self.spring_coefficient_log[901:999])/len(self.spring_coefficient_log[901:999])

        self.spring_coefficient_log.append(spring_coefficient)
        if t < 100:
            output.SetFromVector([spring_coefficient])
        else:
            output.SetFromVector([self.spring_coefficient])
