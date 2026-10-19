"""The ornithopter's observer: its velocity, the push on it and the wind, its spin and the turn on it."""
import math

from machine.helicopter.kit import _observe_wind_spin
from machine.ornithopter.airframe import (GRAVITY, INERTIA, K_PITCH, K_ROLL, K_YAW, MASS_KG, RHO,
                                          WING_SPAN, WO_PULL, WO_TURN, _flap_inflow, _wing_cda,
                                          _wing_drag)


def observe(core, state, dt):
    """`core`'s estimates `dt` s on from `state`: its velocity, the push on it and the wind, its
    spin and the turn on it."""
    turn = [[float(state['turn'][r][c]) for c in range(3)] for r in range(3)]
    vel, spin, h = [float(x) for x in state['vel']], [float(x) for x in state['spin']], float(state['h'])
    if not (h > 0.03 and core.begun):
        core.vel_hat, core.spin_hat, core.begun = list(vel), list(spin), True
        core.push_hat = [p * max(0.0, 1.0 - WO_PULL * dt) for p in core.push_hat]
        core.turn_hat = [t * max(0.0, 1.0 - WO_TURN * dt) for t in core.turn_hat]
        core.wind = [w * max(0.0, 1.0 - WO_PULL * dt) for w in core.wind]
        return
    u = [vel[a] - core.wind[a] for a in range(3)]
    fast = math.hypot(*u)
    sp = state.get('speeds', (0.0, 0.0))
    w_f = float(sp[0]) if sp[0] > 1.0 else core.Controls[0]
    w_a = float(sp[1]) if sp[1] > 1.0 else core.Controls[1]
    w_sq = 0.5 * (w_f * w_f + w_a * w_a)
    w_eq = math.sqrt(w_sq)
    stroke, pitch_t, roll_d, yaw_t = state.get('controls', core.Controls[2:])
    alpha = 0.5 * RHO * _wing_cda(stroke) / MASS_KG * fast
    up = (turn[0][1], turn[1][1], turn[2][1])
    s_along = sum(u[a] * up[a] for a in range(3)) * (1.0 if stroke >= 0.0 else -1.0)
    lift = float(state.get('lift', 0.0)) * _flap_inflow(s_along, math.hypot(u[0], u[2]), w_eq)
    for a in range(3):
        off = vel[a] - core.vel_hat[a]
        acc = (lift * up[a]) / MASS_KG - (GRAVITY if a == 1 else 0.0) \
            + core.push_hat[a] - alpha * u[a]
        core.vel_hat[a] += dt * (acc + 2.0 * WO_PULL * off)
        core.push_hat[a] += dt * WO_PULL * WO_PULL * off
    gamma = (abs(float(state.get('lift', 0.0))) / (2.0 * MASS_KG * w_eq * WING_SPAN)
             if w_eq > 1.0 and abs(s_along) < 4.0 * w_eq * WING_SPAN else 0.0)
    q_w = _wing_drag(stroke, pitch_t, roll_d, w_eq)
    tau = (K_PITCH * w_sq * pitch_t - (INERTIA[2] - INERTIA[1]) * spin[1] * spin[2],
           K_YAW * w_a * w_a * yaw_t - q_w - (INERTIA[0] - INERTIA[2]) * spin[2] * spin[0],
           K_ROLL * w_sq * roll_d - (INERTIA[1] - INERTIA[0]) * spin[0] * spin[1])
    _observe_wind_spin(core, u, fast, alpha, gamma, up, spin, tau, INERTIA, dt)
