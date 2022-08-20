# Drake imports
# External imports
import numpy as np
from pydrake.all import *

# Internal imports
import default_params as params

# Logging
# Lines printed in the console will be automatically logged.
import logging
from datetime import datetime

class Feeder(LeafSystem):
    def __init__(self, params):
        LeafSystem.__init__(self)

        # Discrete port setting
        self.period = params.h
        self.num_disc_port = 6

        # Continuous port setting
        self.num_q = 2
        self.num_v = 2
        self.num_z = 0
        

        # 
        self.disc_state_index = self.DeclareDiscreteState(self.num_disc_port)
        # Update ball position with discrete period according to function DoUpdate
        self.DeclarePeriodicDiscreteUpdateEvent(self.period, 0, self.DoUpdate)
        self.disc_state_output_port = self.DeclareStateOutputPort("discrete_output", self.disc_state_index)
        
        self.cont_state_index = self.DeclareContinuousState(self.num_q, self.num_v, self.num_z)
        self.cont_state_output_port = self.DeclareStateOutputPort("continuous_output", self.cont_state_index)

    def DoUpdate(self, context, disc_output):
        """
        Discrete update of ball state. Set new state in xd
        """
        value = np.arange(self.num_disc_port)
        disc_output.set_value(value)
        
        # The following line is critical. Odd errors result if removed
        return EventStatus.Succeeded()

    def DoCalcTimeDerivatives(self, context, derivatives):
        q = np.arange(self.num_q)
        v = np.arange(self.num_v)
        # z = np.arange(self.num_z)
        value = np.concatenate([q, v])
        derivatives.SetFromVector(value)

    def AddToBuilder(self, builder, scene_graph):
        builder.AddSystem(self)
        return self

class Sink(LeafSystem):
    def __init__(self):
        super().__init__()

        self.input_port = self.DeclareVectorInputPort("ball_state", 6)
        # self.output_port = self.DeclareVectorOutputPort("enum", 1)

    def dump(self, context):
        state = context.get_continuous_state_vector()
        print(state)

    def AddToBuilder(self, builder, scene_graph):
        builder.AddSystem(self)
        return self