# Drake imports
# External imports
import numpy as np
from pydrake.all import *
import pyrealsense2 as rs
import cv2

# Internal imports
import os.path
import sys
sys.path.append('/root/kinova-arm/drake/bounce_pympc/')
import default_params as params
from test_modules import Sink

# Logging
# Lines printed in the console will be automatically logged.
import logging
from datetime import datetime

logging.basicConfig(filename='runtimeLog.log', level=logging.DEBUG)
logging.info('======================= Started at {} ======================='.format(datetime.now()))

class Camera(LeafSystem):
    def __init__(self, params):
        super().__init__()

        self.ball_radius = 0.02 # in meters

        # Get intrinsics of the depth stream
        depth_profile = self.profile.get_stream(rs.stream.depth)
        intr = depth_profile.as_video_stream_profile().get_intrinsics()
        self.intr_m = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        self.extr_m = np.load('X_WorldRealsense.npy')

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        # print("Depth Scale is: " , depth_scale)

        # We will be removing the background of objects more than
        #  clipping_distance_in_meters meters away
        # clipping_distance_in_meters = 1 #1 meter
        # clipping_distance = clipping_distance_in_meters / depth_scale

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        self.align = rs.align(rs.stream.color)

        # Parameters to calculate ball position
        self.last_position = None
        self.period = 1 / 30 # frame rate 30 fps

        # [xb, yb, tb, xbd, ybd, tbd]
        self.state_index = self.DeclareDiscreteState(6)
        # Update ball position with discrete period according to function DoUpdate
        self.DeclarePeriodicDiscreteUpdateEvent(self.period, 0, self.DoUpdate)
        self.state_output_port = self.DeclareStateOutputPort("ball_state", self.state_index)
    
    @staticmethod
    def __enter__(self):
        # Setup camera connection
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        ## --- Start streaming --- ##
        self.profile = self.pipeline.start(self.config)

        return self
    
    @staticmethod
    def __exit__(self, exc_type, exc_value, traceback):
        if traceback is not None:
            print(exc_type, exc_value, traceback)
        self.pipeline.stop()
        return self

    def _find_location(self, f_m, intrinsic_m, extrinsic_m, depth_scale):
            frame_coordinate = np.array([f_m[0], f_m[1], f_m[2]*depth_scale + self.ball_radius, 1])
            X_CameraBall = np.eye(4)
            X_CameraBall[0:3, 0:3] = intrinsic_m
            X_B = extrinsic_m @ X_CameraBall @ frame_coordinate
            return X_B

    def DoUpdate(self, context, xb):
        # Log time in drake
        drake_time_msg = "----------------------- Drake time: %f s -----------------------" % context.get_time()
        print(drake_time_msg)
        logging.debug(drake_time_msg)

        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        rgb_image = np.asanyarray(color_frame.get_data())

        # TODO: color thresholding
        '''Thresholding Information RGB --> HSV
            Bouncy Blue: 0 30 100       --> 220 100 39
            Bouncy Yellow: 160 140 40   --> 50 75 63
            Bouncy Orange: 210 10 0     --> 3 100 82
            Bouncy Red: 190 1 5         --> 358 100 75
            Bouncy Green: 0 140 50      --> 141 100 55
            Ping Pong: 220 100 0        --> 27 100 86
        '''
        hsv_frame = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        lower_bound = np.array([70, 50, 50])
        upper_bound = np.array([100, 255, 255]) # Green
        background_elimination_mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
        # Display filtered image
        filtered_rgb_image = cv2.bitwise_and(rgb_image, rgb_image, mask= background_elimination_mask)
        # HoughCircle to find the ball
        filtered_gray_image = cv2.cvtColor(filtered_rgb_image, cv2.COLOR_BGR2GRAY)
        
        filtered_blurred_gray_image = cv2.medianBlur(filtered_gray_image,15)
        #cv2.imshow("Temp",filtered_blurred_gray_image)

        rows = filtered_blurred_gray_image.shape[0]
        circles = cv2.HoughCircles(filtered_blurred_gray_image, cv2.HOUGH_GRADIENT, 0.5, rows/8, param1=120, param2=15, minRadius=0, maxRadius=-1)

        if circles is not None:
            circles= np.uint16(np.around(circles))
            major_circle = circles[0][0]
            center = (major_circle[0],major_circle[1])
            try:
                point_px_new = np.array([center[0], center[1], depth_image[center[0],center[1]]]) # (x, y, distance by (x, y)) measured in pixels
                if point_px_new[2] != 0:
                    point_px = point_px_new
                    # cv2.circle(filtered_rgb_image, center, 1, (0,255,255),3)
                    # print(point_px)
                    world_corrdinate = self._find_location(point_px, self.intr_m, self.extr_m, self.depth_scale)
                # print(world_corrdinate)
            except IndexError:
                world_corrdinate = None

        # Four cases:
        # 1. The ball's position has not been initialized   AND     the ball is not found   --> raise an error
        # 2. The ball's position has been initialized       AND     the ball is not found   --> continue with the last known position
        # 3. The ball's position has been initialized       AND     the ball is found       --> update the ball's position
        # 4. The ball's position has not been initialized   AND     the ball is found       --> update the ball's position
        
        if world_corrdinate is None:
            if self.last_position is None:
                raise RuntimeError("The ball is not found")
            else:
                world_corrdinate = self.last_position
        else:
            if self.last_position is None:
                self.last_position = world_corrdinate

            
        ball_position = np.array([world_corrdinate[0],world_corrdinate[2],0]) # truncate 3D to 2D states: x, z, roll
        ball_velocity = (ball_position - self.last_position) / self.period
        ball_state = np.concatenate([ball_position, ball_velocity])
        xb.set_value(ball_state)

        print(ball_state)

        self.last_position = ball_position # Update position memory

        # The following line is critical. Odd errors result if removed
        return EventStatus.Succeeded()

    def AddToBuilder(self, builder, scene_graph):
        builder.AddSystem(self)
        return self
            


def demo():
    # Whether or not to plot the safety/target regions in the meshcat visualizer
    plot_regions = True

    # Create a diagram
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())

    # Setup visualization
    # meshcat = StartMeshcat()
    # MeshcatVisualizer(meshcat).AddToBuilder(builder, scene_graph, meshcat)

    
    # Add modules to the diagram
    cam = Camera(params).AddToBuilder(builder, scene_graph)
    sink = Sink().AddToBuilder(builder, scene_graph)

    builder.Connect(cam.state_output_port, sink.input_port)
    
    # Build the diagram
    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    context.SetDiscreteState(params.xb0) # initial context for the ball system.
    context.SetContinuousState([]) # initial context for the solver (paddle acceleration)

    # Set the initial conditions
    # context.SetDiscreteState(params.xb0)
    # if plot_regions: context.SetDiscreteState(region.def_state_v) # initial context for the region plotting system if Region() is constructed.
    
    # Try to run simulation 4 times slower than real time
    simulator.set_target_realtime_rate(0.25)
    try:
        simulator_status = simulator.Initialize()
        simulator.AdvanceTo(float(100))
    except RuntimeError:
        print(simulator_status.message())
        return
    

if __name__ == "__main__":
    try:
        demo()
    except KeyboardInterrupt:
        exit(1)