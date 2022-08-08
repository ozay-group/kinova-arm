# external imports
import numpy as np

# internal imports
from pympc.geometry.polyhedron import Polyhedron

# numeric parameters of the system
m = 1. # mass
r = .1 # radius
j = .4*m*r**2. # moment of inertia
d = .4 # nominal floor-ceiling distance
l = .3 # floor and ceiling width
mu = .2 # friction coefficient
g = 10. # gravity acceleration
h = .01 # discretization time step. The finer the time step, the more time it takes to simulate.

#                   [x1,    x2,     x3,         x4,     x5,     x6,     x7,     x8,     x9,     x10]
#                   [xb,    yb,     tb,         xf,     yf,     xdb,    ydb,    tdb,    xdf,    ydf]
x_max = np.array(   [l,     d-2.*r, 1.2*np.pi,  l,      d-2.*r, 2.,     2.,     10.,    2.,     2.])    # state upper bounds
x_min = - x_max                                                                                         # state lower bounds
xn_max = np.array(  [0.1,   0.1,    1.2*np.pi,  0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1])   # terminal state upper bounds
xn_min = np.zeros(10)                                                                                   # terminal state lower bounds

# terminal set
# X_N = Polyhedron.from_bounds(*[np.zeros(10)]*2)
X_N = Polyhedron.from_bounds(x_min, x_max)

# input bounds
u_max = np.array([
    30., 30., # floor acceleration
])
u_min = - u_max

# controller parameters

# time steps
N = 20

# weight matrices (*2 to cancel out the 1/2 in the controller code)
Q = np.diag([
    1., 1., .01,
    1., 1.,
    1., 1., .01,
    1., 1.
])*2.
R = np.diag([
    .01, .001
])*2.
P = np.zeros((10, 10))

coeff_rest = 1