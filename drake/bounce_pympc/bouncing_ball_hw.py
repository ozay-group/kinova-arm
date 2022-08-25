# Drake imports
# External imports
import numpy as np
from pydrake.all import *

# Internal imports
import default_params as params
from bounce_dynamics import symbolic_bounce_dynamics_restitution
from pympc.control.hscc.controllers import HybridModelPredictiveController

# Vision import
import pyrealsense2 as rs
import cv2

# Generate two ball dynamic models. One for motion simulation and one for MPC.
S_sim = symbolic_bounce_dynamics_restitution()
S_mpc = symbolic_bounce_dynamics_restitution(stp=2*params.h)

# setting path
sys.path.append('/root/kinova_drake/')

from kinova_station import KinovaStationHardwareInterface, EndEffectorTarget, GripperTarget, KinovaStation
# from controllers.velocity import VelocityCommand, VelocityCommandSequence, VelocityCommandSequenceController

# setting path
sys.path.append('/root/kinova-arm/drake/')
from twist_sequence_controller.controller import Controller

# Logging
# Lines printed in the console will be automatically logged.
import logging
from datetime import datetime

logging.basicConfig(filename='runtimeLog.log', level=logging.DEBUG)
logging.info('======================= Started at {} ======================='.format(datetime.now()))

def dimension_trimmer(rotation=np.zeros(3), translation=np.zeros(3)):
    """_summary_
        On KINOVA interface, the pose is in 6 dimensions. However, our solver, due to computational limit,
        only supports certain dimensions. This function trims KINOVA poses to calculable poses.
    Args:
        pose (numpy array[,6]): [roll, pitch, yaw, x, y, z]

    Returns:
        numpy array [,3]: [x, y, t] in Marcucci --> [y, z, roll] in KINOVA
    """
    truncated_pose = np.array([pose[4], pose[5], pose[0]])
    return truncated_pose

class VelocityCalculator(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        # Constants
        self.last_ee_pose = np.zeros(6)
        self.last_time = 0.0
        self.last_ee_velocity = np.zeros(6)
        
        # Create Input Port for The Current State
        self.pose_port = self.DeclareVectorInputPort("current_ee_pose",
                                                                BasicVector(6))

        # Create Output Port which should share the pose of the block
        self.DeclareVectorOutputPort(
                "estimated_ee_velocity",
                BasicVector(6),
                self.ComputeVelocity,
                {self.time_ticket()}   # indicate that this doesn't depend on any inputs,
                )                      # but should still be updated each timestep

        # Build the diagram

    def ComputeVelocity(self, context, output):
        """
        Description:
            This function computes the velocity in pose-space of the signal connected to the input port.
        """
        # Constants
        t = context.get_time()
        eps0 = 0.002

        if (t - self.last_time) < eps0:
            output.SetFromVector(self.last_ee_velocity)
        else:
            # Get Current Pose from Port
            current_ee_pose = self.pose_port.Eval(context)

            ee_estimated_velocity = (current_ee_pose - self.last_ee_pose)/(t-self.last_time)
            
            # Save last variables
            self.last_time = t
            self.last_ee_velocity = ee_estimated_velocity
            self.last_ee_pose = current_ee_pose
            
            print("ee_velocity")
            print(ee_estimated_velocity)

            # Create Output
            output.SetFromVector(ee_estimated_velocity)        
        
class TwistController(Controller):
    """
    Description:
        A controller that attempts to execute each of the commands in the CommandSequence
        object given to it.

        Sends gripper position commands as well as end-effector twist/wrench commands.
    """

    def __init__(self,
                        Kp = np.diag([10,10,10,2,2,2])*10, Kd = np.eye(6)*np.power(10.0,-2)):
        """
        __init__
        Description:
            Constructor for CommandSequenceController objects.
        """
        Controller.__init__(self,command_type=EndEffectorTarget.kWrench)

        # self.cs = command_sequence
        self.gripper_target_value = [0] # Not interested but for the sake of completeness

        # [xpdd, ypdd, zdd]
        self.acc_input_port = self.DeclareVectorInputPort("paddle_acc", 3)

        #PD gains for Twist Controller
        self.Kp = Kp
        self.Kd = Kd

    def CalcGripperCommand(self,context,output):
        """
        Description:
            Computes the gripper command (position to consider).
        """
        # t = context.get_time()

        cmd_pos = np.array([0])
        output.SetFromVector(cmd_pos)
    def SetGripperCommandType(self, context, output):
        command_type = GripperTarget.kPosition
        output.SetFrom(AbstractValue.Make(command_type))

    def CalcEndEffectorCommand(self,context,output):
        """
        CalcEndEffectorCommand
        Description:
            Computes the output command to send to the controller. 
        """
        # t = context.get_time()
        # print("t = %s" % t)

        # Get Target End-Effector Target Type
        # command_t = self.cs.current_command(t)
        #print(command_t)

        # For Twist Control
        self.command_type = EndEffectorTarget.kTwist

        # Get acceleration input
        acc = self.acc_input_port.Eval(context) # [xd2f, yd2f, zd2f]
        print("------------------------------",acc,"-------------------------------")
        print("------------------------------", acc.shape, "------------------------------")

        # Get target end-effector twist and wrench
        # target_twist = command_t.ee_target_twist
        # [roll, pitch, yaw, x, y, z]
        period = params.h
        target_twist = np.array([0.0, 0.0, 0.0, period*acc[0], period*acc[1], period*acc[2]])
        target_wrench = np.zeros(6)

        # Get current end-effector pose and twist
        current_twist = self.ee_twist_port.Eval(context)
        current_wrench = self.ee_wrench_port.Eval(context)

        # Compute pose and twist errors
        twist_err = target_twist - current_twist
        wrench_err = target_wrench - current_wrench

        # Set command (i.e. end-effector twist or wrench) using a PD controller
        Kp = self.Kp
        Kd = self.Kd
        cmd = Kp@twist_err + Kd@wrench_err

        #print(cmd)

        # Return Output
        output.SetFromVector(cmd)

    def ConnectToStation(self, builder, station, time_step=-1.0):
        """
        Connect inputs and outputs of this controller to the given kinova station (either
        hardware or simulation). 
        """

        # Construct Default Value for time_step
        if time_step < 0.0:
            if isinstance(station,KinovaStation):
                time_step = station.plant.time_step()
            else:
                raise Exception("Time step should be given when running ConnectToStation() on the HarwareKinovaStation.")

        # Create a simple delay block
        delay_block = builder.AddSystem(DiscreteTimeDelay(
            time_step, # Setting the update_sec (width of each discrete step)
            1, # Setting the number of discrete steps to wait
            6  # Size of the input to the delay block
        ))

        #Connect: ee_command output port -> delay -> the station target
        builder.Connect(
            self.GetOutputPort("ee_command"),
            delay_block.get_input_port()
        )
        builder.Connect(                                  # Send commands to the station
                delay_block.get_output_port(),
                station.GetInputPort("ee_target"))
        
        # Connect the command type port to the station
        builder.Connect(
                self.GetOutputPort("ee_command_type"),
                station.GetInputPort("ee_target_type"))

        # Connect Gripper Commands to the station
        builder.Connect(
                self.GetOutputPort("gripper_command"),
                station.GetInputPort("gripper_target"))
        builder.Connect(
                self.GetOutputPort("gripper_command_type"),
                station.GetInputPort("gripper_target_type"))

        # builder.Connect(                                     # Send state information
        #         station.GetOutputPort("measured_ee_twist"),  # to the controller
        #         self.GetInputPort("ee_twist"))
        builder.Connect(
                station.GetOutputPort("measured_ee_wrench"),
                self.GetInputPort("ee_wrench"))


class Solver(LeafSystem):
    """_summary_
    SOLVER() is a computational system based on Model Predictive Controller. 
    It takes the state of the ball and the state of the paddle and renders the 
    control input (paddle acceleration) to the paddle. Because it is a mpc solver,
    the rendered control input is the first element of the control input sequence.

    Args:
        LeafSystem (LeafSystem class): A basic model unit in drake. Solver is inherited from LeafSystem.

    Diagram:
                        -------------------------
                        |                       |
    ball_state   -----> |                       | ----> paddle_acc_adv
                        |         Solver        |
    paddle_state -----> |                       |
                        |                       |
                        -------------------------        
    """

    def __init__(self, params):
        LeafSystem.__init__(self)

        # global counter: how many times the controller is being called.
        self.cntr = 0
        
        # Declare input ports for ball and paddle states
        self.ball_input_port = self.DeclareVectorInputPort("ball_state", 6)
        self.paddle_pose_input_port  = self.DeclareVectorInputPort("paddle_pose",  6)
        self.paddle_twist_input_port = self.DeclareVectorInputPort("paddle_twist", 6)
        
        # Declare output ports for control input
        self.state_index = self.DeclareDiscreteState(3) # [xddf, yddf, zddf]
        self.acc_adv_output_port = self.DeclareStateOutputPort("paddle_acc_adv", self.state_index)

        # Update paddle acceleration with discrete period according to function DesignController()
        self.period = params.h
        self.DeclarePeriodicDiscreteUpdateEvent(self.period, 0, self.DesignController)
    
    def DesignController(self, context, output):
        """_summary_
        DesignController() is an optimization solver that calculate the optimal control input sequence 
        and returns the first control input in the sequence.

        Args:
            context (drake class): The context of this LeafSystem.
            output (numpy array (1,2)): The first element of the optimal control input sequence. This is
                                        the calculation result and is a referred variable.

        Returns:
            EventStatus: The status of the updating event. If it is not success, the system will not update.
        """

        # Count the number of times the controller has been called
        self.cntr += 1
        msg = "Solver being activated: %d" % self.cntr
        print(msg)
        logging.debug(msg)

        # Load states from context
        ball_raw_state = self.ball_input_port.Eval(context) #[xb, yb, zb, xdb, ydb, zdb]
        # Truncate the raw state into the state that is available for the solver. No rotation is perceived in Camera().
        # [xb, yb, tb, xdb, ydb, tdb] <-- [y, z, 0, yd, zd, 0]
        ball_state = np.array([ball_raw_state[1], ball_raw_state[2], 0, ball_raw_state[4], ball_raw_state[5], 0])
        ball_msg = "Ball states: %s" % str(ball_state)
        logging.debug(ball_msg)
        print(ball_msg)

        paddle_pose = self.paddle_pose_input_port.Eval(context) # [roll, pitch, yaw, x, y, z] from KINOVA
        paddle_twist = self.paddle_twist_input_port.Eval(context) # d[roll, pitch, yaw, x, y, z] from KINOVA
        # Truncate the raw state into the state that is available for the solver.
        paddle_state = np.concatenate([paddle_pose[4:], paddle_twist[4:]]) # [xf, yf, xdf, ydf] <-- [y, z, dy, dz]
        paddle_msg = "Paddle states: %s" % str(paddle_state)
        print(paddle_msg)
        logging.debug(paddle_msg)

        
        # mixed-integer formulations
        methods = ['pf', 'ch', 'bm', 'mld']

        # norms of the objective
        norms = ['inf', 'one', 'two']

        # initial condition
        # [x1, x2, x3, x4, x5, x6,  x7,  x8,  x9,  x10]
        # [xb, yb, tb, xf, yf, xdb, ydb, tdb, xdf, ydf]
        x0 = np.concatenate((ball_state[:3], paddle_state[:2], ball_state[3:], paddle_state[2:]))

        # solves of the MICP
        solves = {}
        gurobi_options = {'OutputFlag': 1, 'LogToConsole': 0, 'LogFile': ""} # set OutputFlag to 0 to turn off gurobi log

        # Choose the norm-2 to solve the optimization instead of looping all
        norm = norms[2]
        solves[norm] = {}

        # Choose 'ch' mixed-integer formulations to solve instead of looping all
        method = methods[1]
            
        # Build the controller
        controller = HybridModelPredictiveController(
                S_mpc,
                params.N,
                params.Q,
                params.R,
                params.P,
                params.X_N,
                method,
                norm
            )
            
        # kill solution if longer than 1h
        # controller.prog.setParam('TimeLimit', 3600)
            
        # Solve and store result
        u_mip, x_mip, ms_mip, cost_mip = controller.feedforward(x0, gurobi_options)

        # Check if the solution is feasible
        if u_mip is None:
            print('No solution found. u_mip: %s' % str(u_mip))
            logging.error('No solution found. u_mip: %s' % str(u_mip))
            exit(1)

        # Organize the solution
        solves[norm][method] = {
                'time': controller.prog.Runtime,
                'nodes': controller.prog.NodeCount,
                'mip_gap': controller.prog.MIPGap,
                'u': u_mip,
                'x': x_mip,
                'ms': ms_mip,
                'cost': cost_mip
            }
    
        # log results
        logging.debug("-> norm: %s" % norm)
        logging.debug("-> method: %s" % method)
        logging.debug("-> time: %f" % solves[norm][method]['time'])
        logging.debug("-> mip gap: %f" % solves[norm][method]['mip_gap'])
        logging.debug("-> nodes: %d" % solves[norm][method]['nodes'])
        logging.debug("-> u: %s" % str(solves[norm][method]['u']))
        print("Next input acc: ", u_mip[0])
        
        # Update the output port with the first element of the optimal control input sequence
        acc = np.array([0, u_mip[0][0], u_mip[0][1]]) # [xdd, ydd, zdd]
        output.set_value(acc)
        
        # Return the status of the updating event
        # The following line is critical. Odd errors result if removed
        return EventStatus.Succeeded()
    
    def AddToBuilder(self, builder):
        builder.AddSystem(self)
        return self

class Camera(LeafSystem):
    def __init__(self, params):
        super().__init__()

        ## Physical parameter
        self.ball_radius = 0.02 # in meters

        ## Connect to RealSense depth camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        ## Enable streams
        frame_rate = 30 # fps
        self.period = 1 / frame_rate
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, frame_rate)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, frame_rate)

        ## --- Start streaming --- ##
        profile = self.pipeline.start(self.config)

        ## Get intrinsics of the depth stream
        depth_profile = profile.get_stream(rs.stream.depth)
        self.intr = depth_profile.as_video_stream_profile().get_intrinsics()
        self.extr_m = np.load('X_WorldRealsense.npy')

        ## Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        # print("Depth Scale is: " , self.depth_scale)

        ## Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        self.align = rs.align(rs.stream.color)

        ## Parameters to calculate ball position
        # Currently, the script doesn't support rotation due to physical limitations.
        # It is assumed that the ball will not be anywhere close to the origin. Therefore,
        # a [0, 0, 0] is used instead of NONE value so as to prevent corruption in drake data
        # transmission. It by itself covers the camera's inresponsiveness period for bufferring.
        self.last_position = np.array([0, 0, 0, 1]) # [x, y, z, 1]

        ## Define output ports
        # [xb, yb, zb, xdb, ydb, zdb]
        self.state_index = self.DeclareDiscreteState(6)
        # Update ball position with discrete period according to function DoUpdate
        self.DeclarePeriodicDiscreteUpdateEvent(self.period, 0, self.DoUpdate)
        self.state_output_port = self.DeclareStateOutputPort("ball_state", self.state_index)

    def DoUpdate(self, context, xb):
        # Log time in drake
        drake_time_msg = "----------------------- Drake time: %f s -----------------------" % context.get_time()
        print(drake_time_msg)
        logging.debug(drake_time_msg)

        filtered_rgb_image = self._color_thresholding()

        X_Ball = self._ball_finder(filtered_rgb_image)

        # The memory of ball's location will only updated if
        # 1) the ball has moved noticeably (larger than 1e-4),
        # 2) the new location is not all zeros by intention.
        if np.abs(X_Ball - self.last_position).all() > 1e-4:
            if not np.array_equal(X_Ball, np.array([0, 0, 0, 1])):
                self.last_position = X_Ball
        
        ## Tailor the position information to 2D drake implementation.
        # truncate 3D to 2D states: x, z, roll and roll is zero because it is not observable. 
        # ball_position = np.array([X_Ball[0],X_Ball[2],0]) 
        # ball_velocity = (ball_position - self.last_position[:3]) / self.period
        ball_velocity = (X_Ball - self.last_position) / self.period
        ball_state = np.concatenate([X_Ball[:3], ball_velocity[:3]])
        xb.set_value(ball_state)

        # print(ball_state)

        # The following line is critical. Odd errors result if removed
        return EventStatus.Succeeded()

    def _color_thresholding(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        rgb_image = np.asanyarray(color_frame.get_data())
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        # Create a color thresholding mask
        '''Thresholding Information RGB --> HSV
            Bouncy Blue: 0 30 100       --> 220 100 39
            Bouncy Yellow: 160 140 40   --> 50 75 63
            Bouncy Orange: 210 10 0     --> 3 100 82
            Bouncy Red: 190 1 5         --> 358 100 75
            Bouncy Green: 0 140 50      --> 141 100 55
            Ping Pong: 220 100 0        --> 27 100 86
        '''
        lower_bound = np.array([70, 50, 50])
        upper_bound = np.array([100, 255, 255]) # Green
        background_elimination_mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

        # Filter image
        filtered_rgb_image = cv2.bitwise_and(rgb_image, rgb_image, mask= background_elimination_mask)
        return filtered_rgb_image

    def _ball_finder(self, filtered_rgb_image):
        """_summary_
            No rotation is found inside this snippet of code.
        Args:
            filtered_rgb_image (_type_): _description_

        Returns:
            numpy array [,4]: [x, y, z, 1] relative to the base frame.
        """
        # Downgrade the image for algorithm effectiveness
        filtered_gray_image = cv2.cvtColor(filtered_rgb_image, cv2.COLOR_BGR2GRAY)      
        filtered_blurred_gray_image = cv2.medianBlur(filtered_gray_image,15)

        # Use HoughCircle to find the ball
        rows = filtered_blurred_gray_image.shape[0]
        circles = cv2.HoughCircles(filtered_blurred_gray_image, cv2.HOUGH_GRADIENT, 0.5, rows/8, param1=120, param2=15, minRadius=0, maxRadius=-1)

        # Initialize the position of the ball.
        # Currently, the script doesn't support rotation due to physical limitations.
        # It is assumed that the ball will not be anywhere close to the origin. Therefore,
        # a [0, 0, 0] is used instead of NONE value so as to prevent corruption in drake data
        # transmission. It by itself covers the camera's inresponsiveness period for bufferring.
        X_Ball = np.array([0, 0, 0, 1]) # [x, y, z, 1]

        # If the ball is found...
        if circles is not None:
            circles= np.uint16(np.around(circles))

            # Choose the most likely ball
            major_circle = circles[0][0]

            # Calculate the ball's position in respect to the base frame.
            center = [major_circle[0],major_circle[1]]

            world_coordinate = rs.rs2_deproject_pixel_to_point(self.intr, center, self.depth_scale)
            X_CameraBall = np.array([world_coordinate[0], world_coordinate[1], world_coordinate[2]+self.ball_radius, 1])
            X_Ball = self.extr_m @ X_CameraBall
            # print(X_Ball)
        return X_Ball

    def AddToBuilder(self, builder):
        builder.AddSystem(self)
        return self

def add_loggers_to_system(builder,station):
    # Loggers force certain outputs to be computed
    wrench_logger = LogVectorOutput(station.GetOutputPort("measured_ee_wrench"),builder)
    wrench_logger.set_name("wrench_logger")

    pose_logger = LogVectorOutput(station.GetOutputPort("measured_ee_pose"), builder)
    pose_logger.set_name("pose_logger")

    twist_logger = LogVectorOutput(station.GetOutputPort("measured_ee_twist"), builder)
    twist_logger.set_name("twist_logger")

    gripper_logger = LogVectorOutput(station.GetOutputPort("measured_gripper_velocity"), builder)
    gripper_logger.set_name("gripper_logger")

    return pose_logger
    
        
def balldemo():
    # 
    n_dof = 6
    time_step = 0.025

    # Connect to the hardware
    with KinovaStationHardwareInterface(n_dof) as station:
        # Constants
        dt = 0.001

        # Create a diagram
        builder = DiagramBuilder()

        # Setup loggers
        builder.AddSystem(station)
        pose_logger = add_loggers_to_system(builder,station)

    
        # Add modules to the diagram
        cam = Camera(params).AddToBuilder(builder)
        v_estimator = builder.AddSystem(VelocityCalculator()) # Create the velocity estimator
        sol = Solver(params).AddToBuilder(builder)

        # Create the controller and connect inputs and outputs appropriately
        Kp = 10*np.eye(6)
        Kd = 2*np.sqrt(Kp)
        controller = TwistController(Kp=Kp, Kd=Kd)
        builder.AddSystem(controller)
        controller.set_name("controller")
    
        # Connect i/o ports
        builder.Connect(cam.state_output_port,                                  sol.ball_input_port)
        builder.Connect(station.GetOutputPort("measured_ee_pose"),              sol.paddle_pose_input_port)
        builder.Connect(station.GetOutputPort("measured_ee_twist"),             sol.paddle_twist_input_port)
        builder.Connect(sol.acc_adv_output_port,                                controller.acc_input_port)
        builder.Connect(station.GetOutputPort("measured_ee_pose"),              v_estimator.GetInputPort("current_ee_pose")) # Connect the Station to the Estimator
        builder.Connect(v_estimator.GetOutputPort("estimated_ee_velocity"),     controller.GetInputPort("ee_twist")) # Connect the Estimator to the Controller

        controller.ConnectToStation(builder, station, time_step)

        # Log Velocity Estimator
        vel_estimate_logger = LogVectorOutput(v_estimator.GetOutputPort("estimated_ee_velocity"), builder)
        vel_estimate_logger.set_name("velocity_estimate_logger")

    
        # Build the diagram
        diagram = builder.Build()
        diagram.set_name("system_diagram")
        diagram_context = diagram.CreateDefaultContext()
        diagram.Publish(diagram_context)

        # First thing: send to home position
        station.go_home()

        # We use a simulator instance to run the example, but no actual simulation 
        # is being done: it's all on the hardware. 
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)
        simulator.set_publish_every_time_step(False)  # Usually, this should be set to False. Otherwise, simulation will be very slow and won't look like real time.
        simulator.Initialize()
        simulator.AdvanceTo(10.0)

        # Collect Data
        pose_log = pose_logger.FindLog(diagram_context)
        log_times  = pose_log.sample_times()
        pose_data = pose_log.data()
        print(pose_data.shape)

        vel_log = vel_estimate_logger.FindLog(diagram_context)
        vel_log_times = vel_log.sample_times()
        vel_data = vel_log.data()
        print(vel_data.shape)

    #Wait at end
    input('Press ENTER to end python program.')
    

if __name__ == "__main__":
    exit(balldemo())

