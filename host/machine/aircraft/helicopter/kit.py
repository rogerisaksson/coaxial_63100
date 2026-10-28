"""The SO(3) law's pieces the helicopter, the ornithopter and the aerobat share.

Rotations, the attitude's error and its torques, the heading's turn, a stunt's and a hover's
target, the wind and spin observer, the answer.
"""
import ctypes
import math

from machine.aircraft import grounds
from machine.figure import cross
from machine.aircraft.helicopter.airframe import (GRAVITY, MASS_KG, MIN_AIR_M_S, WIND_REG, WO_TURN, WO_WIND)

_d3, _d4, _d9, _d12 = ctypes.c_double * 3, ctypes.c_double * 4, ctypes.c_double * 9, ctypes.c_double * 12
_OBS_P = _d4(MIN_AIR_M_S, WO_WIND, WIND_REG, WO_TURN)
_STATE_IO, _U_BUF, _UP_BUF, _SPIN_BUF, _TAU_BUF, _INERTIA_BUF, _SPAN_BUF, _WING_BUF = (
    _d12(), _d3(), _d3(), _d3(), _d3(), _d3(), _d3(), _d9())
_LIB = False


def _aero_lib():
    global _LIB
    if _LIB is False:
        try:
            from tools.cores import aero
            _LIB = aero.library()
        except Exception:
            _LIB = None
    return _LIB


def _mat_mul(a, b):
    a0, a1, a2 = a
    b0, b1, b2 = b
    a00, a01, a02 = a0
    a10, a11, a12 = a1
    a20, a21, a22 = a2
    b00, b01, b02 = b0
    b10, b11, b12 = b1
    b20, b21, b22 = b2
    return ((a00 * b00 + a01 * b10 + a02 * b20, a00 * b01 + a01 * b11 + a02 * b21, a00 * b02 + a01 * b12 + a02 * b22),
            (a10 * b00 + a11 * b10 + a12 * b20, a10 * b01 + a11 * b11 + a12 * b21, a10 * b02 + a11 * b12 + a12 * b22),
            (a20 * b00 + a21 * b10 + a22 * b20, a20 * b01 + a21 * b11 + a22 * b21, a20 * b02 + a21 * b12 + a22 * b22))


def _mat_t(a):
    a0, a1, a2 = a
    return ((a0[0], a1[0], a2[0]), (a0[1], a1[1], a2[1]), (a0[2], a1[2], a2[2]))


def _rot_x(r):
    c, s = math.cos(r), math.sin(r)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _rot_y(r):
    c, s = math.cos(r), math.sin(r)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rot_z(r):
    c, s = math.cos(r), math.sin(r)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _align_axis(from_u, to_u, mat):
    w, d = cross(from_u, to_u), 1.0 + sum(a * b for a, b in zip(from_u, to_u))
    if d <= 1e-4:
        return _mat_mul(_rot_x(math.pi), mat)
    m0, m1, m2 = mat
    cols = []
    for col in (0, 1, 2):
        v = (m0[col], m1[col], m2[col])
        u = cross(w, v)
        wu = cross(w, u)
        cols.append((v[0] + u[0] + wu[0] / d, v[1] + u[1] + wu[1] / d, v[2] + u[2] + wu[2] / d))
    return _mat_t(cols)


_d15 = ctypes.c_double * 15
_i2 = ctypes.c_int * 2
_FILT_IO, _FLAGS_IO, _R_CUR_BUF, _R_DES_BUF, _W_FF_BUF, _TURN_HAT_BUF, _KP_BUF, _KD_BUF, _OUT3_BUF = (
    _d15(), _i2(), _d9(), _d9(), _d3(), _d3(), _d3(), _d3(), _d3())


def _so3_error(r_cur, r_des):
    e = _mat_mul(_mat_t(r_cur), r_des)
    vx, vy, vz = 0.5 * (e[2][1] - e[1][2]), 0.5 * (e[0][2] - e[2][0]), 0.5 * (e[1][0] - e[0][1])
    sa, ca = math.sqrt((vx * vx + vy * vy) + vz * vz), max(-1.0, min(1.0, 0.5 * (e[0][0] + e[1][1] + e[2][2] - 1.0)))
    if sa <= 1e-4 and ca < 0.0:
        return (-math.pi * (math.sqrt(max(0.0, 0.5 * (e[0][0] + 1.0))) or 1.0),
                math.pi * math.sqrt(max(0.0, 0.5 * (e[1][1] + 1.0))),
                math.pi * math.sqrt(max(0.0, 0.5 * (e[2][2] + 1.0))))
    sc = math.atan2(sa, ca) / sa if sa > 1e-5 else 1.0
    return vx * sc, vy * sc, vz * sc


def _turn_heading(heading, rate, dt):
    h, (c0, s0) = 0.5 * rate * dt, heading
    c, s = (1.0 - h * h) * c0 - 2.0 * h * s0, 2.0 * h * c0 + (1.0 - h * h) * s0
    n = math.hypot(c, s) or 1.0
    return c / n, s / n


def _course_rate(heading, row):
    rate = math.radians(row.get('turn', 0.0))
    if row.get('height', 9.0) < 2.0 and abs(rate) <= 1e-6 and abs(row.get('slide', 0.0)) <= 1e-6 and row.get('nose', 1.0) > 0.0:
        err = (math.radians(grounds.GATES[0][3]) - math.atan2(heading[1], heading[0]) + math.pi) % math.tau - math.pi
        return max(-2.4, min(2.4, 3.0 * err))
    return rate


def _observe_wind_spin(core, u, fast, alpha, gamma, up, spin, tau, inertia, dt, side=0.0, span=(0.0, 0.0, 0.0),
                       wing=None):
    lib = _aero_lib()
    if lib is not None:
        w0, w1, w2 = core.wind
        p0, p1, p2 = core.push_hat
        s0, s1, s2 = core.spin_hat
        t0, t1, t2 = core.turn_hat
        _STATE_IO[:] = (w0, w1, w2, p0, p1, p2, s0, s1, s2, t0, t1, t2)
        _U_BUF[:] = u
        _UP_BUF[:] = up
        _SPIN_BUF[:] = spin
        _TAU_BUF[:] = tau
        _INERTIA_BUF[:] = inertia
        _SPAN_BUF[:] = span
        if wing is None:
            wj = None
        elif isinstance(wing, _d9):
            wj = wing
        else:
            r0, r1, r2 = wing
            _WING_BUF[:] = (r0[0], r0[1], r0[2], r1[0], r1[1], r1[2], r2[0], r2[1], r2[2])
            wj = _WING_BUF
        lib.aeh_observe_wind_spin(_STATE_IO, _U_BUF, fast, alpha, gamma, _UP_BUF, _SPIN_BUF,
                                  _TAU_BUF, _INERTIA_BUF, dt, side, _SPAN_BUF, wj, _OBS_P)
        core.wind[:] = (_STATE_IO[0], _STATE_IO[1], _STATE_IO[2])
        core.push_hat[:] = (_STATE_IO[3], _STATE_IO[4], _STATE_IO[5])
        core.spin_hat[:] = (_STATE_IO[6], _STATE_IO[7], _STATE_IO[8])
        core.turn_hat[:] = (_STATE_IO[9], _STATE_IO[10], _STATE_IO[11])
        return
    if wing is None:
        wing = ((0.0,) * 3,) * 3
    uh = tuple(u[a] / fast if fast > MIN_AIR_M_S else 0.0 for a in range(3))
    m = [[alpha * ((1.0 if a == b else 0.0) + uh[a] * uh[b]) + gamma * up[a] * up[b] + side * span[a] * span[b]
          + wing[a][b] for b in range(3)] for a in range(3)]
    for a in (0, 2):
        dw = dt * WO_WIND * sum(m[b][a] * core.push_hat[b] for b in range(3)) / (m[0][a] ** 2 + m[1][a] ** 2 + m[2][a] ** 2 + WIND_REG)
        core.wind[a] += dw
        for b in range(3):
            core.push_hat[b] -= m[b][a] * dw
    for a in range(3):
        e = spin[a] - core.spin_hat[a]
        core.spin_hat[a] += dt * (tau[a] / inertia[a] + core.turn_hat[a] + 2.0 * WO_TURN * e)
        core.turn_hat[a] = max(-12.0, min(12.0, (1.0 - 2.5 * dt) * core.turn_hat[a] + dt * WO_TURN * WO_TURN * e))


def _stunt_step(core, flips, rolls, nose_val, osc_amp, osc_hz, turn, yaw, rate, acc_des, dt):
    core.r_prev, core.up_filt, core.w_ff = None, None, (0.0, 0.0, 0.0)
    df, dr = -math.tau * flips * dt, -math.tau * rolls * dt
    if abs(osc_amp) > 1e-6:
        if core.tic_phase == 0.0:
            core.tic_phase = math.pi if turn[1][1] > 0.0 else 0.0
        core.flip_ang = core.roll_ang = core.piro_ang = 0.0
        core.tic_phase += math.tau * osc_hz * dt
        rk = _rot_z if nose_val <= -2.5 else _rot_x
        r_des = _mat_mul(_rot_y(yaw), rk(0.5 * math.pi + osc_amp * math.cos(core.tic_phase)))
        r_next = _mat_mul(_rot_y(yaw + rate * dt), rk(0.5 * math.pi + osc_amp * math.cos(core.tic_phase + math.tau * osc_hz * dt)))
    else:
        core.tic_phase = 0.0
        core.flip_ang, core.roll_ang, core.piro_ang = core.flip_ang + df, core.roll_ang + dr, core.piro_ang + rate * dt
        b = _rot_y(yaw - core.piro_ang)
        r_des = _mat_mul(_mat_mul(b, _rot_x(core.flip_ang)), _mat_mul(_rot_z(core.roll_ang), _rot_y(core.piro_ang)))
        r_next = _mat_mul(_mat_mul(b, _rot_x(core.flip_ang + df)), _mat_mul(_rot_z(core.roll_ang + dr), _rot_y(core.piro_ang + rate * dt)))
    w_ff = tuple(x / max(dt, 1e-6) for x in _so3_error(r_des, r_next))
    r11, (ax, ay, az) = turn[1][1], acc_des
    ay_s = GRAVITY * 2.0 * r11 if abs(osc_amp) <= 1e-6 else ay * r11 / max(r11 * r11, 0.34)
    return r_des, w_ff, MASS_KG * (ay_s + 0.35 * (ax * turn[0][1] + az * turn[2][1]))


def _exp_so3(x, y, z):
    a2 = (x * x + y * y) + z * z
    a = math.sqrt(a2)
    s, c = (math.sin(a) / a, (1.0 - math.cos(a)) / a2) if a > 1e-7 else (1.0 - a2 / 6.0, 0.5 - a2 / 24.0)
    return ((1.0 - c * (y * y + z * z), -s * z + c * x * y, s * y + c * x * z),
            (s * z + c * x * y, 1.0 - c * (x * x + z * z), -s * x + c * y * z),
            (-s * y + c * x * z, s * x + c * y * z, 1.0 - c * (x * x + y * y)))


def _allowed_rate(e, a_p, pole):
    d, near = abs(e), a_p / (pole * pole)
    far = float(d > near)
    return math.copysign((1.0 - far) * pole * d + far * math.sqrt(2.0 * a_p * max(d - 0.5 * near, 0.0)), e)


def _plan_hold(core, along, dt, wg=0.0):
    al0 = getattr(core, '_al_to', None) if core.r_prev is not None else None
    up0 = getattr(core, '_up_to', None) if core.r_prev is not None else None
    wg0 = getattr(core, '_wg_to', None) if core.r_prev is not None else None
    core._al_cur, core._al_to = (along if al0 is None else al0), along
    core._up_cur, core._up_to = (core.up_pull if up0 is None else up0), core.up_pull
    core._wg_cur, core._wg_to = (wg if wg0 is None else wg0), wg
    core._plan_dt = dt
    return along


def _step_hold(core, along, dt):
    if along is None or not hasattr(core, '_al_cur'):
        return along
    s = min(1.0, dt / max(core._plan_dt, 1e-6))
    core._up_cur = core.up_pull = core._up_cur + (core._up_to - core._up_cur) * s
    core._wg_cur = core._w_gate = core._wg_cur + (core._wg_to - core._wg_cur) * s
    core._al_cur = (core._al_cur[0] + (core._al_to[0] - core._al_cur[0]) * s,
                    core._al_cur[1] + (core._al_to[1] - core._al_cur[1]) * s)
    core._plan_dt = max(0.0, core._plan_dt - dt)
    return core._al_cur


def _smooth_target(core, want_up, sign, yaw, rate, pace, dt):
    lib = _aero_lib()
    if lib is not None:
        has_up, has_r = core.up_filt is not None, core.r_prev is not None
        _FLAGS_IO[0], _FLAGS_IO[1] = int(has_up), int(has_r)
        if has_up:
            _FILT_IO[0], _FILT_IO[1], _FILT_IO[2] = core.up_filt
        if has_r:
            r0, r1, r2 = core.r_prev
            _FILT_IO[3:12] = (r0[0], r0[1], r0[2], r1[0], r1[1], r1[2], r2[0], r2[1], r2[2])
        _FILT_IO[12], _FILT_IO[13], _FILT_IO[14] = core.w_ff
        _UP_BUF[:] = want_up
        lib.aeh_smooth_target(_FILT_IO, _FLAGS_IO, _UP_BUF, sign, yaw, rate, pace, dt,
                              core.flip_ang, _R_DES_BUF)
        core.up_filt = (_FILT_IO[0], _FILT_IO[1], _FILT_IO[2])
        core.r_prev = r_des = ((_R_DES_BUF[0], _R_DES_BUF[1], _R_DES_BUF[2]),
                               (_R_DES_BUF[3], _R_DES_BUF[4], _R_DES_BUF[5]),
                               (_R_DES_BUF[6], _R_DES_BUF[7], _R_DES_BUF[8]))
        core.w_ff = w_ff = (_FILT_IO[12], _FILT_IO[13], _FILT_IO[14])
        return r_des, w_ff
    r0 = _align_axis((0.0, sign, 0.0), want_up, _mat_mul(_rot_y(yaw), _rot_x(core.flip_ang)))
    if core.r_prev is None or dt <= 1e-6:
        core.up_filt, core.r_prev, core.w_ff = want_up, r0, (0.0, sign * rate, 0.0)
        return r0, core.w_ff
    r_ref = core.r_prev
    r_tr = _mat_mul(_align_axis((r_ref[0][1], r_ref[1][1], r_ref[2][1]), want_up, r_ref), _rot_y(sign * rate * dt))
    wy = max(0.0, sign * want_up[1])
    r_t = _mat_mul(r_tr, _rot_y((wy * wy) * _so3_error(r_tr, r0)[1]))
    er = _so3_error(r_ref, r_t)
    pole, kd, a_p, a_max, w_max = 5.5, 22.0, 30.25, 60.0, 6.0
    wt = (0.0, sign * rate, 0.0)
    w_ff = tuple(max(-w_max, min(w_max, core.w_ff[a] + max(
        -a_max, min(a_max, kd * (_allowed_rate(er[a], a_p, pole) + wt[a] - core.w_ff[a]))) * dt))
        for a in (0, 1, 2))
    r_des = _mat_mul(r_ref, _exp_so3(w_ff[0] * dt, w_ff[1] * dt, w_ff[2] * dt))
    core.up_filt, core.r_prev, core.w_ff = want_up, r_des, w_ff
    return r_des, w_ff


def _so3_torques(core, turn, r_des, w_ff, spin, inertia, kp, kd):
    lib = _aero_lib()
    if lib is not None:
        t0, t1, t2 = turn
        d0, d1, d2 = r_des
        _R_CUR_BUF[:] = (t0[0], t0[1], t0[2], t1[0], t1[1], t1[2], t2[0], t2[1], t2[2])
        _R_DES_BUF[:] = (d0[0], d0[1], d0[2], d1[0], d1[1], d1[2], d2[0], d2[1], d2[2])
        _W_FF_BUF[:] = w_ff
        _SPIN_BUF[:] = spin
        _TURN_HAT_BUF[:] = core.turn_hat
        _INERTIA_BUF[:] = inertia
        _KP_BUF[:] = kp
        _KD_BUF[:] = kd
        lib.aeh_so3_torques(_R_CUR_BUF, _R_DES_BUF, _W_FF_BUF, _SPIN_BUF, _TURN_HAT_BUF,
                            _INERTIA_BUF, _KP_BUF, _KD_BUF, _OUT3_BUF)
        return (_OUT3_BUF[0], _OUT3_BUF[1], _OUT3_BUF[2])
    er, ew = _so3_error(turn, r_des), (w_ff[0] - spin[0], w_ff[1] - spin[1], w_ff[2] - spin[2])
    return (inertia[0] * (kp[0] * er[0] + kd[0] * ew[0] - core.turn_hat[0]) + (inertia[2] - inertia[1]) * spin[1] * spin[2],
            inertia[1] * (kp[1] * er[1] + kd[1] * ew[1] - core.turn_hat[1]) + (inertia[0] - inertia[2]) * spin[2] * spin[0],
            inertia[2] * (kp[2] * er[2] + kd[2] * ew[2] - core.turn_hat[2]) + (inertia[1] - inertia[0]) * spin[0] * spin[1])


class Answer:

    """Controller answer for a pass."""

    def __init__(self, speeds, controls, thrust, head, wind, bound=0):
        self.speeds, self.controls, self.thrust, self.head, self.wind, self.bound = speeds, controls, thrust, head, wind, bound
