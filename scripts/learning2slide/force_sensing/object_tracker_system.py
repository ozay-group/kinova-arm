"""

object_tracker_system.py
Description:
    This script contains the class definition of object tracker system and the functions
    associated with the system

"""

""" Imports """
# setting path for imports
import sys
sys.path.append('../')

# general python modules
import numpy as np
import cv2

# intel realsense
import pyrealsense2 as rs

# apriltag detector
from dt_apriltags import Detector

# drake functions
from pydrake.all import *
from pydrake.all import (LeafSystem, RigidTransform, RotationMatrix,
                            Parser, FixedOffsetFrame, RollPitchYaw, SpatialVelocity)

# multibody triad
from manipulation.scenarios import AddMultibodyTriad, AddTriad


class ObjectTrackerSystem(LeafSystem):
    def __init__(self,plant,scene_graph,target_serial_number=145422070360):
        """
        Usage:
            ObjectTrackerSystem(plant,scene_graph)
            ObjectTrackerSystem(plant,scene_graph,serial_number=-1)
        """
        LeafSystem.__init__(self)

        # Constants
        self.object_name = 'object_to_track'

        # Add the Block to the given plant
        self.plant = plant
        self.object_as_model = Parser(plant=self.plant).AddModelFromFile(
            "../../../data/models/slider/slider-block.urdf",
            self.object_name,
        ) # Save the model

        # Add the Camera's frame to the image
        self.scene_graph = scene_graph
        with open('../../camera_calibration/camera_extrinsics.npy', 'rb') as f:
            R_world_cam = np.load(f)
            p_world_cam = np.load(f)
        R_world_cam = RotationMatrix(R_world_cam)
        self.X_world_cam = RigidTransform(R_world_cam, p_world_cam.transpose())

        self.camera_frame = FixedOffsetFrame("camera",plant.world_frame(),self.X_world_cam)
        self.plant.AddFrame(self.camera_frame)
        AddMultibodyTriad(plant.GetFrameByName("camera"), self.scene_graph)

        # Add Object's Frame to the Scene
        AddMultibodyTriad( plant.GetFrameByName("body"), self.scene_graph)

        # Create Output Port which should share the pose of the block
        self.DeclareVectorOutputPort(
                "measured_block_pose",
                BasicVector(6),
                self.DetectObjectPose,
                {self.time_ticket()}   # indicate that this doesn't depend on any inputs,
                )                      # but should still be updated each timestep

        # Setup Realsense Camera Tracking
        self.SetupAprilTagTracker(target_serial_number)
        self.last_pose = np.zeros(6)

        # Finalize Plant
        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()

    def SetupAprilTagTracker(self, serial_number):
        """
        SetupAprilTagTracker
        Description:
            Defines a serial tag tracker which will attempt to find one of the april tags on the
            3d printed block.
        """
        self.realsense_pipeline = rs.pipeline()
        self.realsense_config = rs.config()

        # Get device product line for setting a supporting resolution
        self.realsense_pipeline_wrapper = rs.pipeline_wrapper(self.realsense_pipeline)
        self.realsense_pipeline_profile = self.realsense_config.resolve(self.realsense_pipeline_wrapper)
        self.realsense_device = self.realsense_pipeline_profile.get_device()

        if (serial_number > 0) and (serial_number != int(self.realsense_device.get_info(rs.camera_info.serial_number))):
            raise Exception("Expected camera with serial number "+ str(serial_number)+", but received camera with serial number "+str(self.realsense_device.get_info(rs.camera_info.serial_number)) + ".")

        self.realsense_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Enable depth stream
        self.realsense_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) # Enable color stream

        # Define April Tag Detector
        self.at_detector = Detector(families='tagStandard41h12',
                        nthreads=4,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)

        # camera parameters
        self.camera_params = [ 386.738, 386.738, 321.281, 238.221 ] # These are the camera's focal length and focal center. Received from running aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        self.tag_size = 0.016 # Measured on the tag. (See documentation on how to measure april tag sizes. A link that can help explain: https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide)
        
        # Start streaming
        self.realsense_pipeline.start(self.realsense_config)

    def DetectObjectPose(self,context,output):
        """
        DetectObjectPose
        Description:
            This function attempts to estimate the pose of the block
            using the pose of one of the detected tags.
        """

        # Constants
        at_detector     = self.at_detector
        cam_params      = self.camera_params
        tag_size        = self.tag_size
        pipeline        = self.realsense_pipeline
        plant_context   = self.context

        # Compute Camera Frame's Pose w.r.t. World
        X_world_cam = self.X_world_cam

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            # Not enough frame data was received, output the last pose
            output.SetFromVector(self.last_pose)
            return

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())


        # Print whether or not detector detects anything.
        gray_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
        atag = self.at_detector.detect(gray_image,
                                       estimate_tag_pose=True,
                                       camera_params=cam_params,
                                       tag_size= tag_size)

        # Process detection info
        current_pose = self.last_pose # Make the default be the last pose we output
        if atag:
            R_cam_atag = atag[0].pose_R
            p_cam_atag = atag[0].pose_t
 
            R_cam_atag = RotationMatrix(R_cam_atag)
            p_cam_atag = p_cam_atag
            X_cam_atag = RigidTransform(R_cam_atag, p_cam_atag)

            X_world_object = X_world_cam.multiply(X_cam_atag)

            current_pose = np.hstack([RollPitchYaw(X_world_object.rotation()).vector(),
                                      X_world_object.translation().reshape((3,))])
            
            output.SetFromVector(current_pose)
            self.last_pose = current_pose

            print(current_pose)
            print(context.get_time())

        # Force the current free body to have the target pose/rigid transform
        self.plant.SetFreeBodyPose(
            plant_context,
            self.plant.GetBodyByName("body", self.object_as_model),
            RigidTransform(RollPitchYaw(current_pose[:3]),current_pose[3:])
        )

        self.plant.SetFreeBodySpatialVelocity(
            self.plant.GetBodyByName("body", self.object_as_model),
            SpatialVelocity(np.zeros(3),np.array([0.0,0.0,0.0])),
            plant_context
            )

        # Set The Output of the block to be the current pose
        output.SetFromVector(current_pose)


    def SetInitialObjectState(self,diagram_context):
        """
        Description:
            Sets the initial position to be slightly above the ground (small, positive z value)
            to be .
        """

        # Set Pose
        p_object = [0.0, 0.0, 0.2]
        R_object = RotationMatrix.MakeXRotation(np.pi/2.0) # RotationMatrix.MakeXRotation(-np.pi/2.0)
        X_object = RigidTransform(R_object,p_object)
        self.plant.SetFreeBodyPose(
            self.plant.GetMyContextFromRoot(diagram_context),
            self.plant.GetBodyByName("body", self.object_as_model),
            X_object)

        # Set Velocities
        self.plant.SetFreeBodySpatialVelocity(
            self.plant.GetBodyByName("body", self.object_as_model),
            SpatialVelocity(np.zeros(3),np.array([0.0,0.0,0.0])),
            self.plant.GetMyContextFromRoot(diagram_context))