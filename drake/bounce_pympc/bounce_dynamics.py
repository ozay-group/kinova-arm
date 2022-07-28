# external imports
import numpy as np
import sympy as sp

# internal imports
from pympc.geometry.polyhedron import Polyhedron
from pympc.dynamics.discrete_time_systems import LinearSystem, AffineSystem, PieceWiseAffineSystem
from pympc.control.hscc.controllers import HybridModelPredictiveController


def symbolic_bounce_dynamics_restitution(params):
    """
    Construct a piecewise affine (PWA) representation of the bouncing ball dynamics in discrete time.
    
    Modified from ``pwa_dynamics.py`` in the hscc19 branch of [pympc](https://github.com/TobiaMarcucci/pympc/)
    """
    
    # symbolic state
    xb, yb, tb = sp.symbols('xb yb tb') # position of the ball
    xf, yf = sp.symbols('xf yf') # position of the floor
    xdb, ydb, tdb = sp.symbols('xdb ydb tdb') # velocity of the ball
    xdf, ydf = sp.symbols('xdf ydf') # velocity of the floor
    x = sp.Matrix([
        xb, yb, tb,
        xf, yf,
        xdb, ydb, tdb,
        xdf, ydf
    ])

    # symbolic input
    xd2f, yd2f = sp.symbols('xd2f yd2f') # acceleration of the floor
    u = sp.Matrix([
        xd2f, yd2f
    ])

    # symbolic contact forces
    ftf, fnf = sp.symbols('ftf fnf') # floor force

    # symbolic ball velocity update
    xdb_next = xdb + params.h*ftf/params.m
    ydb_next = ydb + params.h*fnf/params.m - params.h*params.g
    tdb_next = tdb + params.r*params.h*ftf/params.j

    # symbolic ball position update
    xb_next = xb + params.h*xdb_next
    yb_next = yb + params.h*ydb_next
    tb_next = tb + params.h*tdb_next

    # symbolic floor velocity update
    xdf_next = xdf + params.h*xd2f
    ydf_next = ydf + params.h*yd2f

    # symbolic floor position update
    xf_next = xf + params.h*xdf_next
    yf_next = yf + params.h*ydf_next

    # symbolic state update
    x_next = sp.Matrix([
        xb_next, yb_next, tb_next,
        xf_next, yf_next,
        xdb_next, ydb_next, tdb_next,
        xdf_next, ydf_next
    ])

    # symbolic relative tangential velocity (ball wrt floor)
    sliding_velocity_floor = xdb_next + params.r*tdb_next - xdf_next

    # symbolic relative normal velocity (ball wrt floor)
    rel_velocity_floor = ydb - ydf
    rel_velocity_floor_next = ydb_next - ydf_next

    # symbolic gap functions (ball wrt floor)
    gap_floor = yb_next - yf_next

    # symbolic ball distance to floor
    ball_on_floor = sp.Matrix([
        xb_next - xf_next - params.l,
        xf_next - xb_next - params.l
    ])

    # state + input bounds
    xu = x.col_join(u)
    xu_min = np.concatenate((params.x_min, params.u_min))
    xu_max = np.concatenate((params.x_max, params.u_max))

    # discrete time dynamics in mode 1
    # (ball in the air)

    # set forces to zero
    f_m1 = {ftf: 0., fnf: 0.}

    # get dynamics
    x_next_m1 = x_next.subs(f_m1)
    S1 = AffineSystem.from_symbolic(x, u, x_next_m1)

    # build domain
    D1 = Polyhedron.from_bounds(xu_min, xu_max)

    # - gap <= 0 with floor
    gap_floor_m1 = gap_floor.subs(f_m1)
    D1.add_symbolic_inequality(xu, sp.Matrix([- gap_floor_m1]))

    # check domain
    assert D1.bounded
    assert not D1.empty

    # discrete time dynamics in mode 2
    # (ball bouncing on the floor)

    # enforce bouncing
    ftf_m2 = sp.solve(sp.Eq(sliding_velocity_floor, 0), ftf)[0]
    fnf_m2 = sp.solve(sp.Eq(rel_velocity_floor_next + params.coeff_rest*rel_velocity_floor, 0), fnf)[0]
    f_m2 = {ftf: ftf_m2, fnf: fnf_m2}

    # get dynamics
    x_next_m2 = x_next.subs(f_m2)
    S2 = AffineSystem.from_symbolic(x, u, x_next_m2)

    # build domain
    D2 = Polyhedron.from_bounds(xu_min, xu_max)

    # gap <= 0 with floor
    D2.add_symbolic_inequality(xu, sp.Matrix([gap_floor_m1]))

    # ball not falling down the floor
    D2.add_symbolic_inequality(xu, ball_on_floor.subs(f_m2))

    # friction cone
    D2.add_symbolic_inequality(xu, sp.Matrix([ftf_m2 - params.mu*fnf_m2]))
    D2.add_symbolic_inequality(xu, sp.Matrix([- ftf_m2 - params.mu*fnf_m2]))

    # check domain
    assert D2.bounded
    assert not D2.empty

    # discrete time dynamics in mode 3
    # (ball sliding right on the floor)

    # enforce sticking
    f_m3 = {ftf: -params.mu*fnf_m2, fnf: fnf_m2}

    # get dynamics
    x_next_m3 = x_next.subs(f_m3)
    S3 = AffineSystem.from_symbolic(x, u, x_next_m3)

    # build domain
    D3 = Polyhedron.from_bounds(xu_min, xu_max)

    # gap <= 0 with floor
    D3.add_symbolic_inequality(xu, sp.Matrix([gap_floor_m1]))

    # ball not falling down the floor
    D3.add_symbolic_inequality(xu, ball_on_floor.subs(f_m3))

    # positive relative velocity
    D3.add_symbolic_inequality(xu, sp.Matrix([- sliding_velocity_floor.subs(f_m3)]))

    # check domain
    assert D3.bounded
    assert not D3.empty

    # discrete time dynamics in mode 4
    # (ball sliding left on the floor)

    # enforce sticking
    f_m4 = {ftf: params.mu*fnf_m2, fnf: fnf_m2}

    # get dynamics
    x_next_m4 = x_next.subs(f_m4)
    S4 = AffineSystem.from_symbolic(x, u, x_next_m4)

    # build domain
    D4 = Polyhedron.from_bounds(xu_min, xu_max)

    # gap <= 0 with floor
    D4.add_symbolic_inequality(xu, sp.Matrix([gap_floor_m1]))

    # ball not falling down the floor
    D4.add_symbolic_inequality(xu, ball_on_floor.subs(f_m4))

    # negative relative velocity
    D4.add_symbolic_inequality(xu, sp.Matrix([sliding_velocity_floor.subs(f_m4)]))

    # check domain
    assert D4.bounded
    assert not D4.empty


    # list of dynamics
    S_list = [S1, S2, S3, S4]

    # list of domains
    D_list = [D1, D2, D3, D4]

    # PWA system
    S = PieceWiseAffineSystem(S_list, D_list)
    return S
    
    
def _dyn_test():
    """
    Test the bouncing ball dynamics with simulation
    Requires X11 to view the matplotlib plots
    """
    import default_params as params
    S = symbolic_bounce_dynamics_restitution(params)
    
    x0 = np.array([
        0., 0.005, np.pi,
        0., 0.,
        0., 0., 0.,
        0., 0.
    ])
    
    # open loop controls
    u_zero = [np.array([0., 0.0])] * 2000
    u_osc = [np.array([0., np.cos(0.01 * i)]) for i in range(2000)]
    u_jolt = u_zero[:50] + [np.array([0, 2]) for i in range(30)] + [np.array([0, -2]) for i in range(60)] + [np.array([0, 2]) for i in range(30)]+ u_zero[:500]
    
    # Simulation
    xs = S.simulate(x0, u_osc)[0]
    
    heights = [xsv[1] for xsv in xs]
    floor_heights = [xsv[4] for xsv in xs]
    times = [params.h * i for i in range(len(heights))]
        
    # Plotting trajectories
    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(heights, label="Ball")
    plt.plot(floor_heights, label="Paddle")
    plt.xlabel("Time (s)")
    plt.ylabel("Height (m)")
    plt.legend()
    plt.show()
    

if __name__ == "__main__":
    _dyn_test()
    
