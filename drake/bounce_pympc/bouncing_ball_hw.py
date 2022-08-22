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
from observers.camera_viewer import CameraViewer

# Logging
# Lines printed in the console will be automatically logged.
import logging
from datetime import datetime

logging.basicConfig(filename='runtimeLog.log', level=logging.DEBUG)
logging.info('======================= Started at {} ======================='.format(datetime.now()))

# TODO - adjust feasible region automatically for pwa bounce dynamics

# This is a little hack for the geometry output port
# Drake needs a function that allocates a new FramePoseVector wrapped in an AbstractValue
# Maybe there is a built in way to do this?
def alloc_FramePoseVector():
    return AbstractValue.Make(FramePoseVector())

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
        
class PaddleController(LeafSystem):
    """
    Example controller for paddle
    
                        -------------------------
                        |                       |
    ball_state   -----> |                       | ----> paddle_acc
                        |   PaddleController    |
                        |                       |
                        |                       |
                        -------------------------

    ball_state: [xb, yb, tb, xbd, ybd, tbd]
    paddle_acc: [xpdd, ypdd]
    """
    
    def __init__(self, params):
        LeafSystem.__init__(self)
        self.ball_input_port = self.DeclareVectorInputPort("ball_state", 6)
        self.acc_output_port = self.DeclareVectorOutputPort("paddle_acc", 2,
                                                            self.CalcOutput)
        
    def CalcOutput(self, context, output):
        ball_state = self.ball_input_port.Eval(context)
        # proportional to ball position, results in oscillations
        u1 = 0.0
        u2 = -20 * (ball_state[1])
        output.SetFromVector([u1, u2])
        
    def AddToBuilder(self, builder, scene_graph):
        builder.AddSystem(self)
        return self

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
        self.paddle_input_port = self.DeclareVectorInputPort("paddle_state", 4)
        
        # Declare output ports for control input
        self.state_index = self.DeclareDiscreteState(2) # [xddf, yddf]
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
        ball_state = self.ball_input_port.Eval(context)
        ball_msg = "Ball states: %s" % str(ball_state)
        logging.debug(ball_msg)
        print(ball_msg)

        paddle_state = self.paddle_input_port.Eval(context)
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
        output.set_value(u_mip[0])
        
        # Return the status of the updating event
        # The following line is critical. Odd errors result if removed
        return EventStatus.Succeeded()
    
    def AddToBuilder(self, builder, scene_graph):
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
        # [xb, yb, tb, xbd, ybd, tbd]
        self.state_index = self.DeclareDiscreteState(6)
        # Update ball position with discrete period according to function DoUpdate
        self.DeclarePeriodicDiscreteUpdateEvent(self.period, 0, self.DoUpdate)
        self.state_output_port = self.DeclareStateOutputPort("ball_state", self.state_index)

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
            # print(X_ball)

        # The memory of ball's location will only updated if
        # 1) the ball has moved noticeably,
        # 2) the new location is not all zeros by intention.
        if np.abs(X_Ball - self.last_position).all() > 1e-4:
            if not np.array_equal(X_Ball, np.array([0, 0, 0, 1])):
                self.last_position = X_Ball
        
        ## Tailor the position information to 2D drake implementation.
        # truncate 3D to 2D states: x, z, roll and roll is zero because it is not observable. 
        ball_position = np.array([X_Ball[0],X_Ball[2],0]) 
        ball_velocity = (ball_position - self.last_position[:3]) / self.period
        ball_state = np.concatenate([ball_position, ball_velocity])
        xb.set_value(ball_state)

        # print(ball_state)

        # The following line is critical. Odd errors result if removed
        return EventStatus.Succeeded()

    def AddToBuilder(self, builder, scene_graph):
        builder.AddSystem(self)
        return self

def create_velocity_control_hw_scenario(station_in):
    """
    Description:
        Anchors it to a "ground plane" and gives it the
        RobotiQ 2f_85 gripper.
        This should also initialize the meshcat visualizer so that we can easily view the robot.
    Usage:
        builder, controller, diagram, diagram_context = create_velocity_control_hw_scenario(station)
    """

    builder = DiagramBuilder()

    # Constants
    gripper_type = '2f_85'
    dt = 0.001

    pusher_position = [0.8,0.5,0.25]
    # pusher_rotation=[0,np.pi/2,0]
    pusher_rotation=[0,0,0]

    # Start with the Kinova Station object
    # station = KinovaStationHardwareInterface(time_step=dt,n_dof=6)

    # Start assembling the overall system diagram
    builder = DiagramBuilder()
    builder.AddSystem(station)

    # Setup loggers
    pose_logger = add_loggers_to_system(builder,station)

    # Setup Controller
    cs = setup_triangle_command_sequence()
    controller, v_estimator = setup_controller_and_connect_to_station(cs,builder,station)

    # Log Velocity Estimator
    vel_estimate_logger = LogVectorOutput(v_estimator.GetOutputPort("estimated_ee_velocity"), builder)
    vel_estimate_logger.set_name("velocity_estimate_logger")

    # Build the system diagram
    diagram = builder.Build()
    diagram.set_name("system_diagram")
    diagram_context = diagram.CreateDefaultContext()

    # context = diagram.CreateDefaultContext()
    # station.meshcat.load()
    diagram.Publish(diagram_context)

    ## Set up initial positions ##

    # Set default arm positions
    # station.go_home(diagram, diagram_context)

    # Set starting position for any objects in the scene
    # station.SetManipulandStartPositions(diagram, diagram_context)

    # Return builder, controller, etc.
    return builder, controller, diagram, diagram_context, pose_logger, vel_estimate_logger

def setup_triangle_command_sequence():
    """
    Description:
        Creates the command sequence that we need to achieve the infinity sequence.
    Notes:
        Each velocity is a six-dimensional vector where each dimension represents the following rates:
        - [roll, pitch, yaw, x, y, z]
    """
    # Constants
    triangle_side_duration = 10.0

    # Create the command sequence object
    vcs = VelocityCommandSequence([])

    # 1. Initial Command (Pause for 5s)
    init_velocity = np.zeros((6,))
    vcs.append(VelocityCommand(
        name="pause1",
        target_velocity=init_velocity,
        duration=2,
        gripper_closed=False))

    # 2. Upper Right
    deltap1 = np.zeros((6,))
    deltap1[3:] = np.array([0.2,0.2,0])
    vcs.append(VelocityCommand(
        name="upper_right",
        target_velocity=deltap1/triangle_side_duration,
        duration=triangle_side_duration,
        gripper_closed=False))

    # 3. Lower Right
    deltap2 = np.zeros((6,))
    deltap2[3:] = np.array([0.2,-0.2,0])
    vcs.append(VelocityCommand(
        name="upper_right",
        target_velocity=deltap2/triangle_side_duration,
        duration=triangle_side_duration,
        gripper_closed=False))    

    # 4. Return to STart
    deltap3 = np.zeros((6,))
    deltap3[3:] = np.array([-0.4,0,0])
    vcs.append(VelocityCommand(
        name="return_home",
        target_velocity=deltap3/triangle_side_duration,
        duration=triangle_side_duration,
        gripper_closed=False))   

    # 5. Pause
    vcs.append(VelocityCommand(
        name="pause2",
        target_velocity=init_velocity,
        duration=2,
        gripper_closed=False))

    return vcs

def setup_controller_and_connect_to_station(cs,builder,station):
    """
    Description:
        Defines the controller (PID) which is a CommandSequenceController as defined in
        kinova_drake.
    Inputs:
        cs = A CommandSequence object which helps define the CommandSequenceController.
    """

    # Create the velocity estimator
    v_estimator = builder.AddSystem(VelocityCalculator())

    # Create the controller and connect inputs and outputs appropriately
    #Kp = 10*np.eye(6)
    # Kp = np.diag([0.2,0.2,0.2,2,2,2])
    # Kd = 2*np.sqrt(Kp)

    Kp = 10*np.eye(6)
    Kd = 2*np.sqrt(Kp)

    controller = builder.AddSystem(VelocityCommandSequenceController(
        cs,
        command_type=EndEffectorTarget.kWrench,  # wrench commands work best on hardware
        Kp=Kp,
        Kd=Kd))
    controller.set_name("controller")

    # Connect the Controller to the station
    builder.Connect(                                  # Send commands to the station
                controller.GetOutputPort("ee_command"),
                station.GetInputPort("ee_target"))
    builder.Connect(
            controller.GetOutputPort("ee_command_type"),
            station.GetInputPort("ee_target_type"))
    builder.Connect(
            controller.GetOutputPort("gripper_command"),
            station.GetInputPort("gripper_target"))
    builder.Connect(
            controller.GetOutputPort("gripper_command_type"),
            station.GetInputPort("gripper_target_type"))

    # Connect the Station to the Estimator
    builder.Connect(
        station.GetOutputPort("measured_ee_pose"),
        v_estimator.GetInputPort("current_ee_pose")
    )

    # Connect the Estimator to the Controller
    builder.Connect(                                        # Send estimated state information
            v_estimator.GetOutputPort("estimated_ee_velocity"), # to the controller
            controller.GetInputPort("ee_velocity"))
    builder.Connect(
            station.GetOutputPort("measured_ee_twist"),
            controller.GetInputPort("ee_twist"))
    #controller.ConnectToStation(builder, station)



    return controller, v_estimator

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
    # Whether or not to plot the safety/target regions in the meshcat visualizer
    plot_regions = False

    # Create a diagram
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())

    # Setup visualization
    meshcat = StartMeshcat()
    MeshcatVisualizer(meshcat).AddToBuilder(builder, scene_graph, meshcat)

    
    # Add modules to the diagram
    ball = BouncingBallPlant(params).AddToBuilder(builder, scene_graph)
    paddle = PaddlePlant(params).AddToBuilder(builder, scene_graph)
    cont = PaddleController(params).AddToBuilder(builder, scene_graph)
    sol = Solver(params).AddToBuilder(builder,scene_graph)
    if plot_regions: region = Region(params, scene_graph).AddToBuilder(builder)
    
    # Connect i/o ports
    builder.Connect(paddle.state_output_port,   ball.paddle_input_port)
    # builder.Connect(ball.state_output_port,     cont.ball_input_port)
    # builder.Connect(cont.acc_output_port,       paddle.acc_input_port)

    # Connect the solver
    builder.Connect(ball.state_output_port,     sol.ball_input_port)
    builder.Connect(paddle.state_output_port,   sol.paddle_input_port)
    builder.Connect(sol.acc_adv_output_port,    paddle.acc_input_port)
    
    # Build the diagram
    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # Set the initial conditions
    context.SetDiscreteState(0, params.xb0) # initial context for the ball system.
    context.SetDiscreteState(1, params.xd2f0) # initial context for the solver (paddle acceleration)
    if plot_regions: context.SetDiscreteState(2, region.def_state_v) # initial context for the region plotting system if Region() is constructed.
    context.SetContinuousState(params.xf0)
    
    # Try to run simulation 4 times slower than real time
    simulator.set_target_realtime_rate(0.25)
    simulator.AdvanceTo(0.5)
    

if __name__ == "__main__":
    ###############################################
    # Important Flags

    run = True
    show_station_diagram = False
    show_plots = True

    n_dof = 6
    ###############################################

    time_step = 0.025

    # Building Diagram
    with KinovaStationHardwareInterface(n_dof) as station:
        builder, controller, diagram, diagram_context, pose_logger, vel_estimate_logger = create_velocity_control_hw_scenario(station)

        if show_station_diagram:
            # Show the station's system diagram
            plt.figure()
            plot_system_graphviz(diagram,max_depth=1)
            plt.show()

        if run:
            # # First thing: send to home position
            # station.go_home(diagram,diagram_context)

            # We use a simulator instance to run the example, but no actual simulation 
            # is being done: it's all on the hardware. 
            simulator = Simulator(diagram, diagram_context)
            simulator.set_target_realtime_rate(1.0)
            simulator.set_publish_every_time_step(False)  # Usually, this should be set to False. Otherwise, simulation will be very slow and won't look like real time.

            # We'll use a super simple integration scheme (since we only update a dummy state)
            # and set the maximum timestep to correspond to roughly 40Hz 
            integration_scheme = "explicit_euler"
            time_step = 0.025
            #ResetIntegratorFromFlags(simulator, integration_scheme, time_step)

            # Run simulation
            simulator.Initialize()
            simulator.AdvanceTo(controller.cs.total_duration())

            # Collect Data
            pose_log = pose_logger.FindLog(diagram_context)
            log_times  = pose_log.sample_times()
            pose_data = pose_log.data()
            print(pose_data.shape)

            vel_log = vel_estimate_logger.FindLog(diagram_context)
            vel_log_times = vel_log.sample_times()
            vel_data = vel_log.data()
            print(vel_data.shape)

            if show_plots:

                # Plot Data - First Half
                fig = plt.figure()
                ax_list1 = []

                for plt_index1 in range(6):
                    ax_list1.append( fig.add_subplot(231+plt_index1) )
                    plt.plot(log_times,pose_data[plt_index1,:])
                    plt.title('Pose #' + str(plt_index1))

                fig2 = plt.figure()
                ax_list2 = []
                for plt_index2 in range(6):
                    ax_list2.append( fig2.add_subplot(231+plt_index2) )
                    plt.plot(vel_log_times[10:],vel_data[plt_index2,10:])
                    plt.title('Vel #' + str(plt_index2))

                plt.show()

        #Wait at end
        input('Press ENTER to end python program.')
