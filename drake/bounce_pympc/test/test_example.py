# Drake imports
# External imports
import numpy as np
from pydrake.all import *

# Internal imports
import default_params as params
from bounce_dynamics import symbolic_bounce_dynamics_restitution
from pympc.control.hscc.controllers import HybridModelPredictiveController

# Generate two ball dynamic models. One for motion simulation and one for MPC.
S_sim = symbolic_bounce_dynamics_restitution()
S_mpc = symbolic_bounce_dynamics_restitution(stp=5*params.h)

# Logging
# Lines printed in the console will be automatically logged.
import logging
from datetime import datetime

logging.basicConfig(filename='runtimeLog.log', level=logging.DEBUG)
logging.info('======================= Started at {} ======================='.format(datetime.now()))

class Feeder(LeafSystem):
    def __init__(self, params):
        LeafSystem.__init__(self)

        self.period = params.h

        # 
        self.disc_state_index = self.DeclareDiscreteState(6)
        # Update ball position with discrete period according to function DoUpdate
        self.DeclarePeriodicDiscreteUpdateEvent(self.period, 0, self.DoUpdate)
        self.disc_state_output_port = self.DeclareStateOutputPort("discrete_output", self.disc_state_index)
        
        self.cont_state_index = self.DeclareContinuousState(2, 2, 0)
        self.cont_state_output_port = self.DeclareStateOutputPort("paddle_state", self.cont_state_index)

    def DoUpdate(self, context, xd):
        """
        Discrete update of ball state. Set new state in xd
        """
        state = context.get_discrete_state_vector()
        paddle_state = self.paddle_input_port.Eval(context)

        # Log time in drake
        drake_time_msg = "----------------------- Drake time: %f s -----------------------" % context.get_time()
        print(drake_time_msg)
        logging.debug(drake_time_msg)
        
        # PWA dynamics are formulated in terms of Ball + Paddle state
        # [xb, yb, tb, xp, yp, xbd, ybd, tbd, xpd, ypd]
        x = np.array([state[0], state[1],state[2], paddle_state[0], paddle_state[1],
                       state[3], state[4],state[5], paddle_state[2], paddle_state[3]])
        # Hack - approximate paddle as not accelerating (zero input)
        u = [np.array([0,0])]

        # Compare the relative process to the terminal set
        terminal_center = (params.xn_min + params.xn_max) / 2
        width = params.xn_max - params.xn_min
        r = (x - terminal_center) / width
        goal_left_msg = "Relative error to the terminal set: %s" % str(r)
        print(goal_left_msg)
        logging.debug(goal_left_msg)
        
        xp = self.pwa_sys.simulate(x, u)[0][1]
        
        xd.set_value([xp[i] for i in [0, 1, 2, 5, 6, 7]])
        
        # The following line is critical. Odd errors result if removed
        return EventStatus.Succeeded()

    def DoCalcTimeDerivatives(self, context, derivatives):
        state = context.get_continuous_state_vector()
        acc = self.acc_input_port.Eval(context)
        derivatives.SetFromVector(np.array([state[2], state[3], acc[0], acc[1]]))

    def AddToBuilder(self, builder, scene_graph):
        builder.AddSystem(self)
        return self