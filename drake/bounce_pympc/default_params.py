# external imports
import numpy as np

# internal imports
from pympc.geometry.polyhedron import Polyhedron

#########################
# Constants Declaration #
#########################

# numeric parameters of the system
m = 1.          # mass
r = .1          # radius
j = .4*m*r**2.  # moment of inertia
d = .4          # nominal floor-ceiling distance
l = .3          # floor and ceiling width
mu = .2         # friction coefficient
g = 10.         # gravity acceleration
h = .01         # discretization time step. The finer the time step, the more time it takes to simulate.

# boundaries
#                   [x1,    x2,     x3,         x4,     x5,     x6,     x7,     x8,     x9,     x10]
#                   [xb,    yb,     tb,         xf,     yf,     xdb,    ydb,    tdb,    xdf,    ydf]
x_max = np.array(   [l,     8*d-2.*r, 1.2*np.pi,  l,      5*d-2.*r, 10.,     10.,     10.,    2.,     2.])    # state upper bounds
x_min = - x_max                                                                                         # state lower bounds
xn_max = np.array(  [0.1,   1.5,    1.2*np.pi,  0.1,    0.8,    0.1,    0.5,    0.1,    0.1,    5.0])   # terminal state upper bounds
xn_min = np.array(  [-0.1,  1.0,   -1.2*np.pi, -0.1,   -0.8,   -0.1,   -0.5,   -0.1,   -0.1,   -5.0])   # terminal state lower bounds
# xn_min = np.zeros(10)                                                                                   # terminal state lower bounds

# initial states
x0 = np.array(      [0.,    1.0,    0.,         0.,     0.3,     0.,     0.,     0.,     0.,     0.])
xb0 = np.array([x0[i] for i in [0, 1, 2, 5, 6, 7]])   # ball's initial state
xf0 = np.array([x0[i] for i in [3, 4, 8, 9]])         # floor's initial state
xd2f0 = np.array([0.0, 0.0])                        # floor's initial acceleration

# terminal set
X_N = Polyhedron.from_bounds(xn_min, xn_max)

# input bounds
u_max = np.array([
    30., 3e5, # floor acceleration
])
u_min = - u_max

# controller parameters
# time steps
N = 5

# weight matrices (*2 to cancel out the 1/2 in the controller code)
Q = np.diag([
    1., 0., 1.,
    1., 0.,
    1., 0., 1.,
    1., 0.
])*2.
# R = np.diag([
#     .01, .001
# ])*2.
R = np.zeros((2,2))
P = np.zeros((10, 10))

# coefficient of restitution
coeff_rest = 0.99

##################################################
# Variable derivation for visualizing boundaries #
##################################################

# extract the positional bounds
p_index = [0, 1, 3, 4]
x_max_p = np.array([x_max[i] for i in p_index])
x_min_p = np.array([x_min[i] for i in p_index])
xn_max_p = np.array([xn_max[i] for i in p_index])
xn_min_p = np.array([xn_min[i] for i in p_index])

# calculate the region specifications
safety_center = (x_max_p + x_min_p) / 2.0
safety_region = np.abs(x_max_p - x_min_p)
target_center = (xn_max_p + xn_min_p) / 2.0
target_region = np.abs(xn_max_p - xn_min_p)

if __name__ == "__main__":
    """_summary_
    If this file is run directly, it will use matplotlib to plot a 2D
    boundaries of the system. This part will not be called if
    default_params.py is imported as a module.
    """
    #TODO: plotting the region may have been already achieved. Polyhedron.plot().
    import matplotlib
    matplotlib.use('agg')
    import matplotlib.pyplot as plt

    # Prepare a figure
    plt.figure()
    plt.xlabel("x (m)")
    plt.ylabel("z (m)")

    # Get the current canvas reference
    ax = plt.gca()

    # Plot safety region
    safety_region_b = matplotlib.patches.Rectangle((x_min_p[0], x_min_p[1]), safety_region[0], safety_region[1], 
        color="magenta", alpha=1, fill=False, ls=':', lw=2.0, label="ball safety region")
    ax.add_patch(safety_region_b)
    safety_region_f = matplotlib.patches.Rectangle((x_min_p[2], x_min_p[3]), safety_region[2], safety_region[3], 
        color="green", alpha=0.5, fill=False, lw=2.0, label="floor safety region")
    ax.add_patch(safety_region_f)

    # Plot target region
    target_region_b = matplotlib.patches.Rectangle((xn_min_p[0], xn_min_p[1]), target_region[0], target_region[1], 
        color="blue", alpha=1, fill=False, ls=':', lw=2.0, label="ball target region")
    ax.add_patch(target_region_b)
    target_region_f = matplotlib.patches.Rectangle((xn_min_p[2], xn_min_p[3]), target_region[2], target_region[3], 
        color="red", alpha=0.5, fill=False, lw=2.0, label="floor target region")
    ax.add_patch(target_region_f)

    # Scale the axis
    p_mat = np.abs(np.hstack((x_max_p, xn_max_p, x_min_p, xn_min_p)))
    lim = np.max(p_mat) * 1.1
    ax.set_xlim([-lim, lim])
    ax.set_ylim([-lim, lim])
    plt.legend()

    # Show/save the figure
    if matplotlib.get_backend() == "agg":
        plt.savefig("safety_regions.png")
    else:
        plt.show()
