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
    def __init__(self, params, disc_port_size=None, cont_port_size=None):
        """_summary_

        Args:
            params (_type_): _description_
            disc_port_size (int, optional): _description_. Defaults to None.
            cont_port_size (array(,3), optional): _description_. Defaults to None.
        """

        LeafSystem.__init__(self)

        if disc_port_size is None and cont_port_size is None:
            raise("Please specify the output ports.")

        if disc_port_size is not None:
            # Discrete port setting
            self.period = params.h
            self.num_disc_port = disc_port_size

            self.disc_state_index = self.DeclareDiscreteState(self.num_disc_port)
            # Update ball position with discrete period according to function DoUpdate
            self.DeclarePeriodicDiscreteUpdateEvent(self.period, 0, self.DoUpdate)
            self.disc_state_output_port = self.DeclareStateOutputPort("discrete_output", self.disc_state_index)

        if cont_port_size is not None:
            # Continuous port setting
            self.num_q = cont_port_size[0]
            self.num_v = cont_port_size[1]
            self.num_z = cont_port_size[2]
            # 
            self.cont_state_index = self.DeclareContinuousState(self.num_q, self.num_v, self.num_z)
            self.cont_state_output_port = self.DeclareStateOutputPort("continuous_output", self.cont_state_index)

    def DoUpdate(self, context, disc_output):
        """
        Discrete update of ball state. Set new state in xd
        """
        value = np.arange(self.num_disc_port)
        disc_output.set_value(value)
        logging.debug("The discrete signal is generated: {}".format(value))
        
        # The following line is critical. Odd errors result if removed
        return EventStatus.Succeeded()

    def DoCalcTimeDerivatives(self, context, derivatives):
        q = np.arange(self.num_q)
        v = np.arange(self.num_v)
        # z = np.arange(self.num_z)
        value = np.concatenate([q, v])
        logging.debug("The continuous signal is generated: %s", str(value))
        derivatives.SetFromVector(value)

    def AddToBuilder(self, builder, scene_graph):
        builder.AddSystem(self)
        return self

class Sink(LeafSystem):
    def __init__(self, port_size=None):
        """_summary_

        Args:
            port_size (int, optional): _description_. Defaults to None.
        """
        super().__init__()
        if port_size is None:
            raise("Please specify the number of ports.")

        self.num_port = port_size
        self.input_port = self.DeclareVectorInputPort("Sink", self.num_port)
        # self.output_port = self.DeclareVectorOutputPort("enum", 1)

    def dump(self, context):
        state = context.get_continuous_state_vector()
        msg = "The result is: {}".format(state)
        logging.debug(msg)
        print(msg)

    def AddToBuilder(self, builder, scene_graph):
        builder.AddSystem(self)
        return self