"""
This script defines the class AprilTracker for tracking the movement of AprilTag.
"""

import numpy as np
import cv2
# intel realsense
import pyrealsense2 as rs
# apriltag detector
from dt_apriltags import Detector
# other libraries
from liegroups import SE3
import matplotlib.pyplot as plt
from calibrUtilities import plot_frame, averageSE3 
import time
import threading
import queue
class AprilTracker:
    """ Detect the pose of Apriltag.
    
        Frame of Apriltag:
                                      ^ y
                           ========================
                           |          |           |
                           |          |           |
                           |          |xx         |
                           |          |xx         |
                           |          |  xx       |
                           |         z---xx-------|--> x
                           |        xxxx          |
                           |        xxxx          |
                           |                      |
                           |                      |
                           ========================
                           
    """
    def __init__(self, tag_size = 0.0255, n_sample = 40, debug = False) -> None:
        self.tag_size = tag_size
        self.n_sample = n_sample 

        with open('camera_extrinsics.npy', 'rb') as f:
            R_world_cam = np.load(f)
            p_world_cam = np.load(f)
        self.X_world_cam = np.zeros([4, 4])
        self.X_world_cam[:3, :3] = R_world_cam
        self.X_world_cam[:3, 3] = p_world_cam.transpose()
        self.X_world_cam[3,3] = 1
        self.debug = debug
        if self.debug:
            fig = plt.figure()
            self.ax = fig.add_subplot(111, projection='3d')
            self.ax.set_aspect('equal')
            plt.ion() # turn on interactive mode (for plot.show(block=False))
            plt.show()
            pose = np.eye(4)
            plot_frame(self.ax, pose)
        self.at_detector = Detector(families='tagStandard41h12', # Configure AprilTag detector
                            nthreads=4,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)
        
        self.pipeline = rs.pipeline() # Declare RealSense pipeline, encapsulating the actual device and sensors
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Enable depth stream
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30) # Enable color stream
        cfg = self.pipeline.start(config) # Start streaming the pipeline and get the configuration
        """ Camera Intrinsics """
        """ Get camera parameters [fx, fy, cx, cy] from RealSense camera
        cam_params = [ 386.738, 386.738, 321.281, 238.221 ]
        https://github.com/IntelRealSense/librealsense/issues/869
        https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.intrinsics.html
        """
        profile = cfg.get_stream(rs.stream.color)                       # Fetch stream profile for color stream
        intrinsics = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
        self.cam_params = [intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy]

        # tracking setup
        self.subprocess = None
        self.results = [[],[]]
        self.track_thread = None
        self.stop_event = threading.Event()

    def detect(self):
        # Detect the Apriltag and return its pose (4x4 matrix)

        se3_list = [] # list of se3 elements
        for i in range(self.n_sample):
            frames = self.pipeline.wait_for_frames() # Wait for a coherent pair of frames: depth and color
            
            color_frame = frames.get_color_frame()
            if not color_frame:
                print("no frame detected.")
                continue
            
            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            # # Perform Apriltag detection
            gray_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
            # cv2.imwrite('test.png', gray_image)
            atag = self.at_detector.detect(
                    gray_image,
                    estimate_tag_pose=True,
                    camera_params=self.cam_params,
                    tag_size= self.tag_size
                    )
            
            """ Collect RealSense Data """
            if atag:
                pose = np.zeros([4,4])
                pose[:3, :3] = atag[0].pose_R
                pose[:3, 3] = np.squeeze(atag[0].pose_t, axis=1)
                pose[3, 3] = 1
                se3_list.append(SE3.from_matrix(pose).log())
                if self.debug:
                    plot_frame(self.ax, pose)
        avg_pose, _ = averageSE3(se3_list)
        if avg_pose is not None:
            if self.debug:
                plot_frame(self.ax,avg_pose, width = 10, alpha = 0.4)
                plt.draw()
                plt.pause(0.001)
            return self.X_world_cam @ avg_pose @ np.diag([1.0, -1.0, -1.0, 1.0])
        else:
            print("No AprilTag is detected.")

    def _track_loop(self):
        while not self.stop_event.is_set():
            frames = self.pipeline.wait_for_frames() # Wait for a coherent pair of frames: depth and color
            t = time.time()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            
            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            # # Perform Apriltag detection
            gray_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
            # cv2.imwrite('test.png', gray_image)
            atag = self.at_detector.detect(
                    gray_image,
                    estimate_tag_pose=True,
                    camera_params=self.cam_params,
                    tag_size= self.tag_size
                    )
            if atag:
                self.results[0].append(t)
                t_world_atag= np.zeros([4,1])
                t_world_atag[:3] = atag[0].pose_t
                t_world_atag[3] = 1
                self.results[1].append(np.dot(self.X_world_cam[1,:], t_world_atag)[0])

    def start_track(self):
        self.results = [[],[]]
        self.stop_event.clear()
        self.track_thread = threading.Thread(target=self._track_loop)
        self.track_thread.start()

    def end_track(self):
        self.stop_event.set()
        self.track_thread.join()
        return self.results 

    def __del__(self):
        self.pipeline.stop() # Stop streaming

if __name__ == "__main__":
    tracker = AprilTracker(n_sample=40, debug=False)
    tracker.start_track()
    time.sleep(4.0)
    results = tracker.end_track()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    # ax.set_aspect('equal')
    plt.ion() # turn on interactive mode (for plot.show(block=False))
    plt.plot(results[0], results[1])
    mng = plt.get_current_fig_manager()
    # mng.full_screen_toggle()
    plt.show(block=True)
    
