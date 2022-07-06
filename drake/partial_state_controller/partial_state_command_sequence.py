"""
partial_state_command_sequence.py
Description:
    An improvement of the CommandSequence and Command classes defined by Vince Kurtz in
    kinova_drake circa June 2022.
    Should include the following new capabilities:
    - Control Position of the Gripper (instead of just open/close)
    - Switch Command Type from Pose To twist
"""

import numpy as np
from kinova_station import EndEffectorTarget, GripperTarget

class PartialStateCommand():
    """
    Description:
        A Simple object describing:
        - A single end-effector pose or twist
        - Gripper State, and
        - Duration.
    """
    def __init__(self, target_type=EndEffectorTarget.kPose, target_value=np.zeros((6,)), gripper_value=0.0, duration=2.5, name=None):
        """
        Description:
            Constructor of the class.

        Parameters:
            target_type     : an enum value which indicates what type of target this command is giving (use EndEffectorTarget.kPose,EndEffectorTarget.kTwist, or EndEffectorTarget.kWrench)
            target_pose     : a 6D vector (np array) describing the desired end-effector pose or twist
            gripper_closed  : boolean, true if the gripper is to be closed
            duration        : the number of seconds allocated to achieving this command. 
            name (optional) : a string describing this command
        """

        self.ee_target_type = target_type
        self.ee_target_value = target_value
        self.gripper_target_value = gripper_value
        self.duration = duration

        if name is not None:
            self.name = name
        else:
            self.name = "command"

        assert (gripper_value >= 0.0) and (gripper_value <= 1.0), "The gripper position must be between 0 and 1."

    
    def __str__(self):
        string = "%s: \n" % self.name
        string += "    end effector target type : %s\n" % self.ee_target_type
        string += "    end effector target val  : %s\n" % self.ee_target_value
        string += "    gripper target val       : %s\n" % self.gripper_target_value
        string += "    duration                 : %s\n" % self.duration

        return string

class PSCSequence():
    """
    Description:
        This object contains a sequence of ComplexCommand objects.
        This is basically a fancy list with user-friendly ways of accessing the command
        data at any given time.
    """

    def __init__(self,command_list):
        """
        Description:
            Constructor of PSCSequence
        """
        self.commands = []      # stores the commands
        self.start_times = [0]  # stores the time each command is to start at

        for command in command_list:
            self.append(command)

    def __str__(self):
        string = ""
        for command in self.commands:
            string += command.__str__()
        return string
    
    def append(self,command):
        """ Add a ComplexCommand to the sequence. """
        self.commands.append(command)
        self.start_times.append(self.start_times[-1] + command.duration)

    def current_command(self,t):
        """
        Description:
            Return the command that is active at the given time, t.
        """
        assert len(self.commands) > 0 , "Empty command sequence. "
        assert t >= 0, "Only positive times allowed."

        # Look through the spaces in start times to try to place t
        for i in range(len(self.commands)):
            if (self.start_times[i] <= t) and (t < self.start_times[i+1]):
                return self.commands[i]

        # If t is not in any of those intervals, the last command holds.
        return self.commands[-1] # Seems like this could lead to unexpected behaviors... TODO: Return to this. 

    def ee_target_type(self,t):
        return self.current_command(t).ee_target_type

    def ee_target_value(self,t):
        return self.current_command(t).ee_target_val

    def gripper_target_value(self,t):
        return self.current_command(t).gripper_target_value

    def total_duration(self):
        return self.start_times[-1]