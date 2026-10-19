"""The helicopter's observer: its velocity and the push on it, the wind, its spin and the turn on it."""
import math

from machine.helicopter.airframe import (BODY_CDA, GRAVITY, INERTIA, K_CYC, K_TAIL, MASS_KG, RHO,
                                         ROTOR_R, TAIL_M, WO_PULL, WO_TURN, _rotor_drag,
                                         _rotor_inflow)
from machine.helicopter.kit import (_observe_wind_spin)


def observe(core, state, dt):
    """`core`'s estimates `dt` s on from `state`: its velocity, the push on it and the wind, its
    spin and the turn on it."""
    t_in, v_in, s_in = state['turn'], state['vel'], state['spin']
    turn = t_in if (type(t_in) is tuple and type(t_in[0]) is tuple and type(t_in[0][0]) is float) \
        else (t_in.tolist() if hasattr(t_in, 'tolist') else [[float(t_in[r][c]) for c in range(3)] for r in range(3)])
    vel = v_in if (type(v_in) is tuple and type(v_in[0]) is float) else [float(x) for x in v_in]
    spin = s_in if (type(s_in) is tuple and type(s_in[0]) is float) else [float(x) for x in s_in]
    h = float(state['h'])
    if not (h > 0.03 and core.begun):
        core.vel_hat, core.spin_hat, core.begun = list(vel), list(spin), True
        core.push_hat = [p * max(0.0, 1.0 - WO_PULL * dt) for p in core.push_hat]
        core.turn_hat = [t * max(0.0, 1.0 - WO_TURN * dt) for t in core.turn_hat]
        core.wind = [w * max(0.0, 1.0 - WO_PULL * dt) for w in core.wind]
        return
    u = (vel[0] - core.wind[0], vel[1] - core.wind[1], vel[2] - core.wind[2])
    fast, sp = math.hypot(*u), state.get('speeds', (0.0, 0.0))
    alpha = 0.5 * RHO * BODY_CDA / MASS_KG * fast
    w_m, w_t = (float(sp[k]) if sp[k] > 1.0 else core.Controls[k] for k in (0, 1))
    coll, cyc_p, cyc_r, tail_p = state.get('controls', core.Controls[2:])
    up, side = (turn[0][1], turn[1][1], turn[2][1]), (turn[0][0], turn[1][0], turn[2][0])
    s_along = sum(u[a] * up[a] for a in range(3)) * (1.0 if coll >= 0.0 else -1.0)
    lift = float(state.get('lift', 0.0)) * _rotor_inflow(s_along, math.hypot(u[0], u[2]), w_m)
    tail = float(state.get('tail', 0.0))
    for a in range(3):
        off = vel[a] - core.vel_hat[a]
        acc = (lift * up[a] + tail * side[a]) / MASS_KG - (GRAVITY if a == 1 else 0.0) + core.push_hat[a] - alpha * u[a]
        core.vel_hat[a] += dt * (acc + 2.0 * WO_PULL * off)
        core.push_hat[a] += dt * WO_PULL * WO_PULL * off
    gamma = (abs(float(state.get('lift', 0.0))) / (2.0 * MASS_KG * w_m * ROTOR_R)
             if w_m > 1.0 and abs(s_along) < 4.0 * w_m * ROTOR_R else 0.0)
    q_m = _rotor_drag(coll, cyc_p, cyc_r, w_m)
    tau = (K_CYC * w_m * w_m * cyc_p - (INERTIA[2] - INERTIA[1]) * spin[1] * spin[2],
           TAIL_M * K_TAIL * w_t * w_t * tail_p - q_m - (INERTIA[0] - INERTIA[2]) * spin[2] * spin[0],
           K_CYC * w_m * w_m * cyc_r - (INERTIA[1] - INERTIA[0]) * spin[0] * spin[1])
    _observe_wind_spin(core, u, fast, alpha, gamma, up, spin, tau, INERTIA, dt)
