"""
hw_test2.py
Description:
    This script will verify whether or not the calibration transform that we got from our previous activities works.
    It will:
        1. Collect an Image from the Kinova Gen3 Wrist Cam
        2. Collect an Image from the Realsense D435 external camera
        3. Extract the pose of the april tag in each image
        4. Compare the poses using our calibration constant.
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from dt_apriltags import Detector
from pydrake.all import ( 
    RigidTransform , RotationMatrix )
import sys, os
from kortex_api.autogen.client_stubs.VisionConfigClientRpc import VisionConfigClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient

from kortex_api.autogen.messages import DeviceConfig_pb2, Session_pb2, DeviceManager_pb2, VisionConfig_pb2

###############
## Functions ##
###############

"""
collect_kinova_image
Description:
    This function will collect the color image and the depth image associated with a single, static scene.
    This function is based on the file kinova_image.py from kinova-arm > drake > vision_calibration_pc.
"""
def collect_kinova_image() -> (np.ndarray,np.ndarray):
    # The video stream from the depth Camera on the Kinova Gen3 is sent through rtsp.
    # Here we capture the stream by opencv. Note: the color stream and depth stream are separate.
    color_cap = cv2.VideoCapture("rtsp://192.168.1.10/color", cv2.CAP_FFMPEG)
    depth_cap = cv2.VideoCapture("rtsp://192.168.1.10/depth")

    # The number of frames to capture
    max_frames_to_capture = 100

    for num in range(max_frames_to_capture):
        num = num + 1

        # These frames are expressed in the camera frame. 
        # For locating, you need to transform them to the end effector frame first.
        _, color_frame = color_cap.read()
        _, depth_frame = depth_cap.read()

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame)
        depth_image = np.asanyarray(depth_frame)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=1), cv2.COLORMAP_JET)

        # If depth and color resolutions are different, resize color image to match depth image for display
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape
        
        if depth_colormap_dim != color_colormap_dim:
            color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)

        # # Show images
        # images = np.hstack((color_image, depth_colormap))
        # cv2.namedWindow('Kinova Depth Camera', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('Kinova Depth Camera', images)
        # cv2.waitKey(1) # 1ms delay

        # Save the 100th frame (Magic number, long enough to focus and stablize the stream.)
        if num == max_frames_to_capture - 1:
            cv2.imwrite("data/kinova-color_image.png", color_image)
            cv2.imwrite("data/kinova-depth_image.png", depth_image)
            break

    return color_image, depth_image

"""
collect_external_realsense_image()
Description:
    This function will collect the color image and depth image associated with a single, static scene
    FROM THE EXTERNAL REALSENSE CAMERA in the lab.
    This function is based on the files:
    - realsense_export_ply_example.py from kinova-arm > drake > vision_calibration_pc and
    - opencv_pose_estimator_example.py from kinova-arm > drake > pusher_slider_sim.
    The last file was useful for fixing some errors in the first file.
"""
def collect_external_realsense_image() -> (np.ndarray,np.ndarray,list):

    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipe)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break

    # Enable color and depth stream
    config.enable_stream(rs.stream.depth, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, rs.format.bgr8, 30)

    # Start streaming with chosen configuration
    profile = pipe.start(config)

    # Wait for the next set of frames from the camera
    frames = pipe.wait_for_frames()

    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    cv2.imwrite("data/realsense-color_image.png", color_image)
    cv2.imwrite("data/realsense-depth_image.png", depth_image.copy())
    

    # print('intrinsics')
    # print(pipeline_profile)
    # print(depth_frame.profile.as_video_stream_profile().intrinsics)

    return color_image, depth_image, rs_intrinsics_to_list( depth_frame.profile.as_video_stream_profile().intrinsics )

"""
rs_intrinsics_to_list
Description:
    This function converts the intrinsics output of a realsense command to a
    list that you can use in something like april tag detection.
"""
def rs_intrinsics_to_list( intrinsics_in ):
    # Extract the relevant fields from the intrinsics struct
    return [ intrinsics_in.ppx, intrinsics_in.ppy, intrinsics_in.fx, intrinsics_in.fy ]

"""
get_kinova_intrinsics
Description:
    This function uses the kortex api to get the current intrinsics of the Kinova Gen3
    wrist camera.
    This function is based on kortex_intrinsic.py from kinova-arm > drake > vision_calibration_apriltag.
"""
def get_kinova_intrinsics()->(list):

    # Import the utilities helper module
    # sys.path.insert(0, os.path.join(os.path.dirname(__file__), "."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        device_manager = DeviceManagerClient(router)
        vision_config = VisionConfigClient(router)

        # Create empty struct for holding the parameters
        intrinsics_new = VisionConfig_pb2.IntrinsicParameters()

        profile_id = VisionConfig_pb2.IntrinsicProfileIdentifier()
        vision_device_id = utilities.example_vision_get_device_id(device_manager)

        if vision_device_id != 0:
            # Collect the intrinsics parameters for the color sensor
            sensor_id = VisionConfig_pb2.SensorIdentifier()

            print("\n-- Using Vision Config Service to get current intrinsic parameters for color resolution 640x480 --")
            sensor_id.sensor = VisionConfig_pb2.SENSOR_COLOR
            color_intrinsics = vision_config.GetIntrinsicParameters(sensor_id, vision_device_id)
            utilities.print_intrinsic_parameters(color_intrinsics)

            return [color_intrinsics.principal_point_x, color_intrinsics.principal_point_y, color_intrinsics.focal_length_x, color_intrinsics.focal_length_y]
        else:
            raise "There was an issue getting the vision device id! id = " + str(vision_device_id)

"""
set_kinova_intrinsics()
Description:
    Forces the kinova camera's color camera to obey a different set of intrinsics than it usually would!
"""
def set_kinova_intrinsics() -> (list):

    # Import the utilities helper module
    # sys.path.insert(0, os.path.join(os.path.dirname(__file__), "."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        device_manager = DeviceManagerClient(router)
        vision_config = VisionConfigClient(router)

        # Create empty struct for holding the parameters
        intrinsics_new = VisionConfig_pb2.IntrinsicParameters()

        profile_id = VisionConfig_pb2.IntrinsicProfileIdentifier()
        vision_device_id = utilities.example_vision_get_device_id(device_manager)

        if vision_device_id != 0:
            # Define the proper profile in order to be able to set the parameters
            print("\n-- Using Vision Config Service to get current intrinsic parameters for color resolution 640x480 --")
            profile_id.sensor = VisionConfig_pb2.SENSOR_COLOR
            profile_id.resolution = VisionConfig_pb2.RESOLUTION_640x480

            intrinsics_old = vision_config.GetIntrinsicParametersProfile(profile_id, vision_device_id)
            utilities.print_intrinsic_parameters(intrinsics_old)

            print("\n-- Using Vision Config Service to set new intrinsic parameters for color resolution 640x480 --")
            intrinsics_new.sensor = profile_id.sensor
            intrinsics_new.resolution = profile_id.resolution
            intrinsics_new.principal_point_x = 640 / 2 + 0.123456
            intrinsics_new.principal_point_y = 480 / 2 + 1.789012
            intrinsics_new.focal_length_x = 650.567890
            intrinsics_new.focal_length_y = 651.112233
            intrinsics_new.distortion_coeffs.k1 = 0.2
            intrinsics_new.distortion_coeffs.k2 = 0.05
            intrinsics_new.distortion_coeffs.p1 = 1.2
            intrinsics_new.distortion_coeffs.p2 = 0.999999
            intrinsics_new.distortion_coeffs.k3 = 0.001
            vision_config.SetIntrinsicParameters(intrinsics_new, vision_device_id)

            print("\n-- Using Vision Config Service to get new intrinsic parameters for color resolution 640x480 --")
            intrinsics_reply = vision_config.GetIntrinsicParametersProfile(profile_id, vision_device_id)
            utilities.print_intrinsic_parameters(intrinsics_reply)

            return [ intrinsics_new.principal_point_x, intrinsics_new.principal_point_y, intrinsics_new.focal_length_x, intrinsics_new.focal_length_y ]
        else:
            raise "There was an issue getting the vision device id! id = " + str(vision_device_id)

"""
extract_tag_pose_from_both_images()
Description:
    Returns the poses (as Drake RigidTransform objects) of the tag in the two images.
    First output is for the kinova_image and second output is for the realsense image.
"""
def extract_tag_pose_from_both_images(kinova_color_image,external_realsense_color_image, realsense_intrinsics, kinova_intrinsics)->(RigidTransform,RigidTransform):
    # Define April Tag Detector
    at_detector = Detector(families='tagStandard41h12',
                            nthreads=1,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)

    # camera parameters
    #cam_params0 = [ 386.738, 386.738, 321.281, 238.221 ]
    tag_size0 = 0.040084375

    realsense_gray_image = cv2.cvtColor(external_realsense_color_image,cv2.COLOR_BGR2GRAY)
    rs_detections = at_detector.detect(
        realsense_gray_image,
        estimate_tag_pose=True,
        camera_params=realsense_intrinsics,
        tag_size= tag_size0
        )

    X_RealsenseTag = RigidTransform(RotationMatrix(rs_detections[0].pose_R),rs_detections[0].pose_t)

    # Get Pose of the Tag w.r.t. Kinova Wrist Camera
    # kinova_intrinsics = get_kinova_intrinsics()
    kinova_camera_matrix = np.array([[kinova_intrinsics[2],0.0,kinova_intrinsics[0]],
                                    [0.0,kinova_intrinsics[3],kinova_intrinsics[1]],
                                    [0.0,0.0,1.0]])

    kinova_color_image_undistorted = cv2.undistort(kinova_color_image,kinova_camera_matrix,np.array([0.2,0.05,1.2,0.999999,0.001]))

    
    # cv2.namedWindow('Kinova Depth Camera', cv2.WINDOW_AUTOSIZE)
    # # cv2.imshow('Kinova Depth Camera', np.hstack((kinova_color_image,kinova_color_image_undistorted)))
    # cv2.imshow('Kinova Depth Camera', kinova_color_image_undistorted)
    # cv2.waitKey(0) # Wait until key is pressed

    kinova_gray_image = cv2.cvtColor(kinova_color_image, cv2.COLOR_BGR2GRAY)
    kinova_detections = at_detector.detect(
        kinova_gray_image,
        estimate_tag_pose=True,
        camera_params=kinova_intrinsics,
        tag_size= tag_size0
        )

    X_KinovacamTag = RigidTransform(RotationMatrix(kinova_detections[0].pose_R),kinova_detections[0].pose_t)

    return X_KinovacamTag, X_RealsenseTag

#################
## Script Body ##
#################

# Collect Images from Kinova Wrist Camera
new_kinova_intrinsics = set_kinova_intrinsics()
kinova_color_img, kinova_depth_img = collect_kinova_image()

#Collect Images from External Realsense Camera
extern_color_img, extern_depth_img, extern_intrinsics_list = collect_external_realsense_image()

# Collect pose of each tag
X_KinovacamTag, X_RealsenseTag = extract_tag_pose_from_both_images(kinova_color_img,extern_color_img, extern_intrinsics_list, new_kinova_intrinsics)

print("Kinova:")
print(X_KinovacamTag)
print("Realsense:")
print(X_RealsenseTag)