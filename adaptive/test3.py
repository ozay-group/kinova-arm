"""
test3.py
Description:

"""

import control

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
        param = np.array([0.1,5.0,0.25])

    theta = param

    v0 = 15 # m/s

    # algorithm

    return np.matrix([[0.0],[v0 - v]]) - (1/m) * np.matrix([[1,v,np.power(v,2)],[0,0,0]]) * theta + np.matrix([[1/m],[0.0]]) * u

# Main

acc0 = control.NonlinearIOSystem(accDynamics)

print(acc0)