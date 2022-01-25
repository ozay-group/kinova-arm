"""
test3.py
Description:

"""

import control
import numpy as np

class AdaptiveCruiseControlSystem:

    def __init__(self) -> None:
        pass


"""
accDynamics
Description:

"""
def accDynamics(t,x,u,param=None):

    # constants
    v = x[0]
    D = x[1]

    m = 1650 # kg

    if param is None:
        param = np.array([0.1,5.0,0.25,15])

    theta = param[0:3]

    v0 = param[4] # m/s

    # algorithm

    return np.matrix([[0.0],[v0 - v]]) - (1/m) * np.matrix([[1,v,np.power(v,2)],[0,0,0]]) * theta + np.matrix([[1/m],[0.0]]) * u

def accClosedLoopSystem1(t,x,u,param=None):

    # constants
    K_v = 0.1
    K_D = 0.1
    v0 = 15 # m/s

    v_des = v0
    D_des = 5 # meters

    # algorithm

    v = x[0]
    D = x[1]

    return accDynamics(t, x, K_v * (v-v_des) + K_D * (D - D_des),param=np.array([0.1,5.0,0.25,v0]))

# Main

acc0 = control.NonlinearIOSystem(accDynamics,inputs = ['a_ego'])
print(acc0)

acc_cl0 = control.NonlinearIOSystem(accClosedLoopSystem1, inputs = ['dummy'])
print(acc_cl0)

T = np.arange(0,10,0.1)
U = np.zeros((1,T.shape[0]))
control.input_output_response( acc_cl0 , T , U )