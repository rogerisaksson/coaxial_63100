"""The SO(3) law's pieces the helicopter, the ornithopter and the aerobat share.

Rotations, the attitude's error and its torques, the heading's turn, a stunt's and a hover's
target, the wind and spin observer, the answer.
"""
import math

from machine import grounds
from machine.figure import cross
from machine.helicopter.airframe import (GRAVITY, MASS_KG, MIN_AIR_M_S, WIND_REG, WO_TURN, WO_WIND)


def _mat_mul(a, b):
    return tuple(tuple(a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] for j in range(3)) for i in range(3))


def _mat_t(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


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
    cols = []
    for col in range(3):
        v = (mat[0][col], mat[1][col], mat[2][col])
        u = cross(w, v)
        cols.append(tuple(v[k] + u[k] + cross(w, u)[k] / d for k in range(3)))
    return _mat_t(cols)


def _so3_error(r_cur, r_des):
    e = _mat_mul(_mat_t(r_cur), r_des)
    vx, vy, vz = 0.5 * (e[2][1] - e[1][2]), 0.5 * (e[0][2] - e[2][0]), 0.5 * (e[1][0] - e[0][1])
    sa, ca = math.hypot(vx, vy, vz), max(-1.0, min(1.0, 0.5 * (e[0][0] + e[1][1] + e[2][2] - 1.0)))
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
                       wing=((0.0,) * 3,) * 3):
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


def _smooth_target(core, want_up, sign, yaw, rate, pace, dt):
    w_ff, a, aw = (0.0, sign * rate, 0.0), min(1.0, 24.0 * dt), min(1.0, 12.0 * dt)
    if core.up_filt is not None and dt > 1e-6:
        ub = [core.up_filt[k] + a * (want_up[k] - core.up_filt[k]) for k in range(3)]
        un = math.hypot(*ub) or 1.0
        want_up = (ub[0] / un, ub[1] / un, ub[2] / un)
    r0 = r_des = _align_axis((0.0, sign, 0.0), want_up, _mat_mul(_rot_y(yaw), _rot_x(core.flip_ang)))
    if pace > 20.0 and core.r_prev is not None:
        r_tr = _mat_mul(_align_axis((core.r_prev[0][1], core.r_prev[1][1], core.r_prev[2][1]), want_up, core.r_prev), _rot_y(sign * rate * dt))
        r_des = _mat_mul(r_tr, _rot_y(aw * _so3_error(r_tr, r0)[1])) if sign * want_up[1] > 0.25 else r_tr
        raw = tuple(max(-5.5, min(5.5, 0.50 * x / max(dt, 1e-6))) for x in _so3_error(core.r_prev, r_des))
        w_ff = tuple(core.w_ff[k] + aw * (raw[k] - core.w_ff[k]) for k in range(3))
    core.up_filt, core.r_prev, core.w_ff = want_up, r_des, w_ff
    return r_des, w_ff


def _so3_torques(core, turn, r_des, w_ff, spin, inertia, kp, kd):
    er, ew = _so3_error(turn, r_des), (w_ff[0] - spin[0], w_ff[1] - spin[1], w_ff[2] - spin[2])
    return (inertia[0] * (kp[0] * er[0] + kd[0] * ew[0] - core.turn_hat[0]) + (inertia[2] - inertia[1]) * spin[1] * spin[2],
            inertia[1] * (kp[1] * er[1] + kd[1] * ew[1] - core.turn_hat[1]) + (inertia[0] - inertia[2]) * spin[2] * spin[0],
            inertia[2] * (kp[2] * er[2] + kd[2] * ew[2] - core.turn_hat[2]) + (inertia[1] - inertia[0]) * spin[0] * spin[1])


class Answer:

    """Controller answer for a pass."""

    def __init__(self, speeds, controls, thrust, head, wind, bound=0):
        self.speeds, self.controls, self.thrust, self.head, self.wind, self.bound = speeds, controls, thrust, head, wind, bound
