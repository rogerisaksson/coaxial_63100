"""The aerobat's observer: its velocity, the push on it, the wind through its wing too, its spin."""
import ctypes
import math

from machine.aerobat.airframe import (DIVE_M_S, GRAVITY, INERTIA, K_PITCH, K_ROLL, K_YAW, MASS_KG,
                                      PROP_R, RHO, ROTATE_M_S, WING, WO_PULL, WO_TURN, _aero_cda,
                                      _aero_inflow, _prop_drag)
from machine.helicopter.kit import _OBS_P, _aero_lib, _observe_wind_spin

_AERO_STATE = (ctypes.c_double * 15)()
_AERO_VEL = (ctypes.c_double * 3)()
_AERO_SPIN = (ctypes.c_double * 3)()
_AERO_TURN = (ctypes.c_double * 9)()
_AERO_PARAMS = (ctypes.c_double * 17)(
    MASS_KG, GRAVITY, WO_PULL, 0.0, K_PITCH, K_YAW, K_ROLL,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
_AERO_INERTIA = (ctypes.c_double * 3)(*INERTIA)


def observe(core, state, dt):
    """`core`'s estimates `dt` s on from `state`: its velocity, the push on it and the wind through
    its wing too; its spin and the turn on it, the airframe's own moments known."""
    t_in, v_in, s_in = state['turn'], state['vel'], state['spin']
    turn = t_in if (type(t_in) is tuple and type(t_in[0]) is tuple and type(t_in[0][0]) is float) \
        else (t_in.tolist() if hasattr(t_in, 'tolist') else [[float(x) for x in r] for r in t_in])
    vel = v_in if (type(v_in) is tuple and type(v_in[0]) is float) else [float(x) for x in v_in]
    spin = s_in if (type(s_in) is tuple and type(s_in[0]) is float) else [float(x) for x in s_in]
    h = float(state['h'])
    if not (h > 0.03 and core.begun):
        core.vel_hat, core.spin_hat, core.begun = list(vel), list(spin), True
        core.push_hat = [p * max(0.0, 1.0 - WO_PULL * dt) for p in core.push_hat]
        core.turn_hat = [t * max(0.0, 1.0 - WO_TURN * dt) for t in core.turn_hat]
        core.wind = [w * max(0.0, 1.0 - WO_PULL * dt) for w in core.wind]
        return
    w0, w1, w2 = core.wind
    u = (vel[0] - w0, vel[1] - w1, vel[2] - w2)
    fast, sp = math.hypot(*u), state.get('speeds', (0.0, 0.0))
    w_p = float(sp[0]) if sp[0] > 1.0 else core.Controls[0]
    w_v = float(sp[1]) if sp[1] > 1.0 else core.Controls[1]
    throt, elev, ail, rud = state.get('controls', core.Controls[2:])
    alpha = 0.5 * RHO * _aero_cda(throt, elev, ail) / MASS_KG * fast
    t0, t1, t2 = turn
    up = (t0[1], t1[1], t2[1])
    along = u[0] * up[0] + u[1] * up[1] + u[2] * up[2]
    s_along = along * (1.0 if throt >= 0.0 else -1.0)
    vh = math.hypot(u[0], u[2])
    lift_raw = float(state.get('lift', 0.0))
    lift = lift_raw * _aero_inflow(s_along, vh, w_p)
    dive = min(1.0, max(0.0, 1.0 + vel[1] / DIVE_M_S))
    core.wing_air = min(1.0, max(0.0, along / ROTATE_M_S)) ** 2 * dive
    core.wingborne = min(1.0, max(0.0, min(along, vh) / ROTATE_M_S)) ** 2 * dive
    gamma = (abs(lift_raw) / (2.0 * MASS_KG * w_p * PROP_R)
             if w_p > 1.0 and abs(s_along) < 4.0 * w_p * PROP_R else 0.0)
    q_p = _prop_drag(throt, elev, ail, w_p)
    lib = _aero_lib()
    if lib is not None:
        p0, p1, p2 = core.push_hat
        s0, s1, s2 = core.spin_hat
        th0, th1, th2 = core.turn_hat
        vh0, vh1, vh2 = core.vel_hat
        _AERO_STATE[:] = (w0, w1, w2, p0, p1, p2, s0, s1, s2, th0, th1, th2, vh0, vh1, vh2)
        _AERO_VEL[:] = vel
        _AERO_SPIN[:] = spin
        _AERO_TURN[:] = (t0[0], t0[1], t0[2], t1[0], t1[1], t1[2], t2[0], t2[1], t2[2])
        _AERO_PARAMS[3] = gamma
        _AERO_PARAMS[7] = q_p
        _AERO_PARAMS[8] = w_p
        _AERO_PARAMS[9] = w_v
        _AERO_PARAMS[10] = elev
        _AERO_PARAMS[11] = rud
        _AERO_PARAMS[12] = ail
        _AERO_PARAMS[13] = lift
        _AERO_PARAMS[14] = alpha
        _AERO_PARAMS[15] = fast
        _AERO_PARAMS[16] = dt
        lib.aeh_aerobat_observe(_AERO_STATE, _AERO_VEL, _AERO_SPIN, _AERO_TURN, WING._w_lift,
                                WING._w_mom, _AERO_INERTIA, _OBS_P, _AERO_PARAMS)
        core.wind[:] = (_AERO_STATE[0], _AERO_STATE[1], _AERO_STATE[2])
        core.push_hat[:] = (_AERO_STATE[3], _AERO_STATE[4], _AERO_STATE[5])
        core.spin_hat[:] = (_AERO_STATE[6], _AERO_STATE[7], _AERO_STATE[8])
        core.turn_hat[:] = (_AERO_STATE[9], _AERO_STATE[10], _AERO_STATE[11])
        core.vel_hat[:] = (_AERO_STATE[12], _AERO_STATE[13], _AERO_STATE[14])
        return
    w_lift = WING.lift(u, turn)
    for a in (0, 1, 2):
        off = vel[a] - core.vel_hat[a]
        acc = (lift * up[a] + w_lift[a]) / MASS_KG - (GRAVITY if a == 1 else 0.0) \
            + core.push_hat[a] - alpha * u[a]
        core.vel_hat[a] += dt * (acc + 2.0 * WO_PULL * off)
        core.push_hat[a] += dt * WO_PULL * WO_PULL * off
    # The airframe's own moments are known: left to `turn_hat` beside the law's trim, twice.
    own = WING.moments(u, turn, (0.0, 0.0, 0.0, spin[0], spin[1], spin[2]))
    tau = (K_PITCH * w_p * w_p * elev + own[0] - (INERTIA[2] - INERTIA[1]) * spin[1] * spin[2],
           K_YAW * w_v * w_v * rud - q_p + own[1] - (INERTIA[0] - INERTIA[2]) * spin[2] * spin[0],
           K_ROLL * w_p * w_p * ail + own[2] - (INERTIA[1] - INERTIA[0]) * spin[0] * spin[1])
    # The wind as the wing's pull answers it too: on the drag's alone a push left from a hover
    # read 6 m/s in still air, and wing-borne none was read.
    _observe_wind_spin(core, u, fast, alpha, gamma, up, spin, tau, INERTIA, dt, RHO * WING.side * fast / MASS_KG,
                       (t0[0], t1[0], t2[0]), WING.jacobian(u, turn))
