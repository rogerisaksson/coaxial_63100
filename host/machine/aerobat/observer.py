"""The aerobat's observer: its velocity, the push on it, the wind through its wing too, its spin."""
import math

from machine.aerobat.airframe import (DIVE_M_S, GRAVITY, INERTIA, K_PITCH, K_ROLL, K_YAW, MASS_KG,
                                      PROP_R, RHO, ROTATE_M_S, WING, WO_PULL, WO_TURN, _aero_cda,
                                      _aero_inflow, _prop_drag)
from machine.helicopter.kit import (_observe_wind_spin)


def observe(core, state, dt):
    """`core`'s estimates `dt` s on from `state`: its velocity, the push on it and the wind through
    its wing too; its spin and the turn on it, the airframe's own moments known."""
    turn = [[float(x) for x in r] for r in state['turn']]
    vel, spin, h = [float(x) for x in state['vel']], [float(x) for x in state['spin']], float(state['h'])
    if not (h > 0.03 and core.begun):
        core.vel_hat, core.spin_hat, core.begun = list(vel), list(spin), True
        core.push_hat = [p * max(0.0, 1.0 - WO_PULL * dt) for p in core.push_hat]
        core.turn_hat = [t * max(0.0, 1.0 - WO_TURN * dt) for t in core.turn_hat]
        core.wind = [w * max(0.0, 1.0 - WO_PULL * dt) for w in core.wind]
        return
    u = [vel[a] - core.wind[a] for a in range(3)]
    fast, sp = math.hypot(*u), state.get('speeds', (0.0, 0.0))
    w_p = float(sp[0]) if sp[0] > 1.0 else core.Controls[0]
    w_v = float(sp[1]) if sp[1] > 1.0 else core.Controls[1]
    throt, elev, ail, rud = state.get('controls', core.Controls[2:])
    alpha = 0.5 * RHO * _aero_cda(throt, elev, ail) / MASS_KG * fast
    up = (turn[0][1], turn[1][1], turn[2][1])
    along = sum(u[a] * up[a] for a in range(3))
    s_along = along * (1.0 if throt >= 0.0 else -1.0)
    lift = float(state.get('lift', 0.0)) * _aero_inflow(s_along, math.hypot(u[0], u[2]), w_p)
    w_lift = WING.lift(u, turn)
    vh, dive = math.hypot(u[0], u[2]), min(1.0, max(0.0, 1.0 + vel[1] / DIVE_M_S))
    core.wing_air = min(1.0, max(0.0, along / ROTATE_M_S)) ** 2 * dive
    core.wingborne = min(1.0, max(0.0, min(along, vh) / ROTATE_M_S)) ** 2 * dive
    for a in range(3):
        off = vel[a] - core.vel_hat[a]
        acc = (lift * up[a] + w_lift[a]) / MASS_KG - (GRAVITY if a == 1 else 0.0) \
            + core.push_hat[a] - alpha * u[a]
        core.vel_hat[a] += dt * (acc + 2.0 * WO_PULL * off)
        core.push_hat[a] += dt * WO_PULL * WO_PULL * off
    gamma = (abs(float(state.get('lift', 0.0))) / (2.0 * MASS_KG * w_p * PROP_R)
             if w_p > 1.0 and abs(s_along) < 4.0 * w_p * PROP_R else 0.0)
    # The airframe's own moments are known: left to `turn_hat` beside the law's trim, twice.
    q_p, own = _prop_drag(throt, elev, ail, w_p), WING.moments(u, turn, (0.0,) * 3 + tuple(spin))
    tau = (K_PITCH * w_p * w_p * elev + own[0] - (INERTIA[2] - INERTIA[1]) * spin[1] * spin[2],
           K_YAW * w_v * w_v * rud - q_p + own[1] - (INERTIA[0] - INERTIA[2]) * spin[2] * spin[0],
           K_ROLL * w_p * w_p * ail + own[2] - (INERTIA[1] - INERTIA[0]) * spin[0] * spin[1])
    # The wind as the wing's pull answers it too: on the drag's alone a push left from a hover
    # read 6 m/s in still air, and wing-borne none was read.
    ends = [[WING.lift([u[k] + d * (k == b) for k in range(3)], turn) for d in (0.1, -0.1)] for b in range(3)]
    _observe_wind_spin(core, u, fast, alpha, gamma, up, spin, tau, INERTIA, dt, RHO * WING.side * fast / MASS_KG,
                       [r[0] for r in turn], [[(ends[b][1][a] - ends[b][0][a]) / (0.2 * MASS_KG) for b in range(3)]
                                              for a in range(3)])
