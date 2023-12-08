##
# adaptive_controller.py
# A simple controller which sends constant outputs. 
# takes in an additional estimated_parameter that influences the desired command sequence
#
##

from pydrake.all import *
from kinova_drake.kinova_station import EndEffectorTarget, GripperTarget

class AdaptiveController(LeafSystem):
    """
    A simple controller which sends comands and recieves messages 
    from a KinovaStation (simulated or real).

                            -------------------------
                            |                       |
                            |                       | ---> ee_command (desired wrench)
                            |                       | ---> ee_command_type
    ee_pose --------------> |                       |
    ee_twist -------------> |   AdaptiveController  |
    ee_wrench ------------> |                       |
    estimated_parameter---> |                       | ---> gripper_command
                            |                       | ---> gripper_command_type
                            |                       |
                            |                       |
                            |                       |
                            -------------------------

    """
    def __init__(self, command_type=EndEffectorTarget.kTwist):
        LeafSystem.__init__(self)

        # Specify what sort of end-effector target we'll use. Twist tends to
        # work better in simulation, and Wrench on the hardware. 
        self.command_type = command_type

        # Declare input ports (current robot state)
        self.ee_pose_port = self.DeclareVectorInputPort(
                                    "ee_pose",
                                    BasicVector(6))
        self.ee_twist_port = self.DeclareVectorInputPort(
                                    "ee_twist",
                                    BasicVector(6))
        self.ee_wrench_port = self.DeclareVectorInputPort(
                                    "ee_wrench",
                                    BasicVector(6))
        self.estimated_paramter_port = self.DeclareVectorInputPort(
                                            "estimated_parameter",
                                            BasicVector(1))

        # Declare output ports (desired end-effector and gripper behavior)
        self.DeclareVectorOutputPort(
                "ee_command",
                BasicVector(6),
                self.CalcEndEffectorCommand)
        self.DeclareAbstractOutputPort(
                "ee_command_type",
                lambda: AbstractValue.Make(self.command_type),
                self.SetEndEffectorCommandType,
                {self.time_ticket()})
        self.DeclareVectorOutputPort(
                "gripper_command",
                BasicVector(1),
                self.CalcGripperCommand,
                {self.time_ticket()})
        self.DeclareAbstractOutputPort(
                "gripper_command_type",
                lambda: AbstractValue.Make(GripperTarget.kPosition),
                self.SetGripperCommandType,
                {self.time_ticket()})

    def SetGripperCommandType(self, context, output):
        command_type = GripperTarget.kPosition
        output.SetFrom(AbstractValue.Make(command_type))

    def SetEndEffectorCommandType(self, context, output):
        output.SetFrom(AbstractValue.Make(self.command_type))

    def CalcGripperCommand(self, context, output):
        output.SetFromVector([0])  # open

    def CalcEndEffectorCommand(self, context, output):

        # Get target end-effector pose and twist
        target_twist = np.zeros(6)
        current_twist = self.ee_twist_port.Eval(context)

        # Default command is simple: just a (P)D controller setting twist to zero
        Kd = 2*np.eye(6)
        cmd = Kd@(target_twist - current_twist)

        output.SetFromVector(cmd)

