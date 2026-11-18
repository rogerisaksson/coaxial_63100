"""Walk to run, brake, jump and leap (the kernel's ballistic texts) on the robot's body and its drives.

    fr = ballistic.froude(v, leg)                  # 1: v^2 / g leg
    beta = ballistic.duty(fr, knots, values)       # 1: a stance's share of a stride, the text's Hermite map
    dv = ballistic.braked(mu, cap, period)         # m/s: the most one step's stance brakes on a floor's friction
    n = ballistic.steps(v, v_walk, dv)             # steps from a run's speed to a walk's
    z = ballistic.crouch(fr)                       # axial: the robot's deepest crouch to jump from, the text's
    wire, names, s, ms = ballistic.bound(fr, turns, now, label)       # its leap in stride, a table's rows
    v, t, w = ballistic.launch(depth, legs)        # m/s, s, rad/s: its hips leaving the floor, its knees then
    h, x = ballistic.parabola(vz, vx)              # m: a flight's apex and span, landing as high as it left

The kernel's ballistic_*.lisp. The robot's legs' line as `leap._a_z` lays it (thigh, shank, the knee's lever to
it); the robot's drives' bounds `bemf`'s at `drives.PACK_V`. Flat arrays and arithmetic: no branch.
"""
import math

from machine import libm
from machine.body import bemf, drives, figure, servos
from machine.body.compile import G_M_S2 as G

#: The robot's knee among its drives (`bemf.DRIVEN`); unilateral throw height spring, N/m, from knee servo
#: stiffness mapped through its countermovement crouch lever.
_CROUCH_LEVER = 0.5 * (figure.THIGH + figure.SHANK) * libm.sin(math.radians(abs(figure.SQUAT['left_ankle'])))
THROW_K = round(servos.SERVO['knee'][1] / (_CROUCH_LEVER * _CROUCH_LEVER), -2)
KNEE = bemf.DRIVEN.index('left_knee')


def froude(v, leg):
    """1: a gait's Froude number at `v` m/s on a leg `leg` m long, hip over sole."""
    import numpy as np
    return libm.sq(np.asarray(v, float)) / (G * leg)


def duty(fr, knots, values):
    """1: a stance's share of a stride at Froude `fr` (`:hermite-froude-map`): a cubic Hermite through the text's
    knots, its tangents their slopes' harmonic means (monotone between them), its ends held."""
    import numpy as np
    x, y = np.asarray(knots, float), np.asarray(values, float)
    h, d = np.diff(x), np.diff(y) / np.diff(x)
    inner = np.where(d[:-1] * d[1:] > 0.0, 2.0 / (1.0 / d[:-1] + 1.0 / d[1:]), 0.0)
    m = np.concatenate([d[:1], inner, d[-1:]])
    f = np.clip(np.asarray(fr, float), x[0], x[-1])
    k = np.clip(np.searchsorted(x, f) - 1, 0, len(h) - 1)
    t = (f - x[k]) / h[k]
    return ((2 * (t * t * t) - 3 * (t * t) + 1) * y[k] + (t * t * t - 2 * (t * t) + t) * h[k] * m[k]
            + (3 * (t * t) - 2 * (t * t * t)) * y[k + 1] + (t * t * t - t * t) * h[k] * m[k + 1])


def braked(mu, cap, period):
    """m/s: the most a step's stance takes off the robot's speed, `cap` of the friction `mu` its soles meet (the
    text's cone-utilization cap): over a step of `period` s its soles bear its weight on average, so mu cap g
    period."""
    import numpy as np
    return np.asarray(mu, float) * cap * G * period


def steps(v, v_walk, dv):
    """Steps that brake `v` m/s to `v_walk` at `dv` a step: ceil((v - v_walk) / dv), the text's N_brake."""
    import numpy as np
    return np.ceil((np.asarray(v, float) - v_walk) / dv).astype(int)


def _push(depth, legs, n):
    """The robot's push from `depth` m under its straight legs (`launch`): (its rise from the crouch along its legs'
    line, m, its speed there, m/s, the sample it leaves at, its knees' levers, m)."""
    import numpy as np
    straight = figure.THIGH + figure.SHANK
    x = np.linspace(straight - depth, straight, n + 1)[:-1]
    cos = (figure.THIGH * figure.THIGH + figure.SHANK * figure.SHANK - x * x) / (2.0 * figure.THIGH * figure.SHANK)
    lever = figure.THIGH * figure.SHANK * np.sqrt(1.0 - cos * cos) / x
    m = figure.mass()
    big = m + legs * drives.armature('left_knee') / (lever * lever)
    dx = x[1] - x[0]
    energy, v = np.zeros(n), np.zeros(n)
    dq = np.zeros(len(bemf.DRIVEN))
    for k in range(n - 1):
        dq[KNEE] = -v[k] / lever[k]
        push = -legs * bemf.bounds(dq, drives.PACK_V, bemf.PEAK)[0][KNEE] / lever[k]
        energy[k + 1] = max(0.0, energy[k] + (push - m * G) * dx)
        v[k + 1] = np.sqrt(2.0 * energy[k + 1] / big[k + 1])
    acc = np.gradient(v, dx) * v
    return x - x[0], v, int(np.argmax(np.append(m * (acc[:-1] + G) <= 0.0, True))), lever


def launch(depth, legs=2.0, n=4000):
    """(m/s, s, rad/s): the robot's hips' speed leaving the floor, the push's seconds and its knees' speed then,
    pushed from `depth` m under its straight legs by `legs` knees at their back-EMF bounds, their rotors spun with
    them, its weight against them. The robot's kinetic energy, its rotors' with it, gains the push's work
    and loses the robot's weight's; it leaves where the floor's push would turn to a pull, or straight."""
    import numpy as np
    up, v, k, lever = _push(depth, legs, n)
    return float(v[k]), float(np.sum((up[1] - up[0]) / np.maximum(v[1:k + 1], 1e-9))), float(v[k] / lever[k])


def rise(depth, legs, at):
    """m: the robot's hips' rise from the crouch `depth` m under its straight legs at `at` s into the
    robot's push (`launch`'s), its leaving's from its end on."""
    import numpy as np
    up, v, k, _lever = _push(depth, legs, 4000)
    t = np.concatenate([[0.0], np.cumsum((up[1] - up[0]) / np.maximum(v[1:k + 1], 1e-9))])
    return np.interp(np.asarray(at, float), t, up[:k + 1])


def parabola(vz, vx):
    """(m, m): a flight's apex over where it left and its span, landing as high: vz^2 / 2g, 2 vx vz / g."""
    import numpy as np
    vz, vx = np.asarray(vz, float), np.asarray(vx, float)
    return vz * vz / (2.0 * G), 2.0 * vx * vz / G


def crouch(fr):
    """Axial: the robot's deepest crouch to jump from (`leap.standing_leap`), the text's countermovement at the
    robot's size (`xform-standing-bilateral-jump`) over its axial metre in its frame `fr` - from the
    human's crouch (`leap.CROUCH`) leant 55 deg it fell 36 ms free into it, 343 mm deep, onto one sole, and
    left 74 mm under the robot's standing height."""
    from machine.texts import kernel
    from machine.law import modal
    return 1e-3 * figure.SCALE * kernel.said('xform-standing-bilateral-jump', 'countermovement-crouch-mm') / (
        modal.AXIAL_M * fr[modal.HIPS_BY])


def bound(fr, turns, now, label):
    """(wire, names, seconds, the ms its push begins) of the robot's leap in stride
    (`xform-in-stride-unilateral-leap`) from the sole landing now: its stance dipped by the text's countermovement and
    pushed straight by one knee at its bound (`launch`, one leg) to leave as the robot's other sole's swing ends -
    half a turn of its rhythm `turns` rad/s on -, its flight in place of its double support,
    the robot's rhythm untouched: the other sole lands meant to stand, this one swings at its own time. The push's
    spring held into the flight (let go at its curve's end the robot stood on, 0.28 m/s up), its other swing
    held through the push (ZMP none), its hold and its drive as it had them (`now`) throughout,
    the robot's landing waiting on its soles; its gait's height, spring and support as it had
    them after, the last row named `label`. Each push row is the curve's height at its blend's end, its spring damped
    critically on the robot's mass, undamped in flight. Planned to leave at 1.16-1.18 m/s the robot flies 0.195 s
    walking (0.69 m/s) and 0.219 s running (1.40), its soles 51 and 68 mm clear, 0.13 and 0.27 m along its way. The
    robot's hold thrown 0.35-0.53 m ahead and its drive swept x1.66 through the leap it fell 8 of 9 walks and every
    jog, the push leg 0.68 m behind at 2 m/s as the other sole landed. The hold's charge has no lever here: dipped
    1.35 times the text's the robot stands and flies no further (0.14 and 0.25 m) - the stance sole behind the robot
    bears a third of it and it sinks 0.16-0.21 m for the 0.11-0.15 told, then outruns a push laid on the
    clock (0.76 m/s on its 0.38) and coasts, the robot's knee at 37-106 of its 171 N m."""
    import math
    import numpy as np
    from machine.texts import kernel, ortho
    from machine.law import latent, modal
    from machine.moves import leap
    z, k, zmp, d = latent.Z.start + modal.AXIAL, latent.K.start + modal.AXIAL, latent.ZMP, latent.D.start + modal.AXIAL
    per = modal.AXIAL_M * fr[modal.HIPS_BY]
    # the text's countermovement: half of it flew a walk's tap 0.13 s
    told = figure.SCALE * kernel.node('xform-in-stride-unilateral-leap', 'ssc-launch')['phase-1-countermovement']['crouch-depth-m']
    dip = float(now[z]) + told / per
    v, push_s, _w = launch((dip - leap.STRAIGHT) * per, legs=1.0)
    leaves = math.pi / max(float(turns), 1e-9)
    dip_s = max(min(push_s, leaves - push_s), 1e-3 * leap.PUSH_MS)
    t_dip = max(0.0, leaves - push_s - dip_s)
    n = max(2, int(math.ceil(1e3 * push_s / leap.PUSH_MS)))
    off = t_dip + dip_s + push_s
    land = off + 2.0 * v / G
    at = np.concatenate([[t_dip], t_dip + dip_s + push_s * np.arange(n) / n, [off, land, land + dip_s]])
    blend = np.concatenate([[dip_s], np.full(n, push_s / n), [1e-3 * leap.PUSH_MS, 1e-3 * leap.PUSH_MS, dip_s]])
    rows = np.zeros(n + 4, latent.WIRE)
    rows['p'], rows['flags'] = np.nan, ortho.CURVE
    rows['flags'][0], rows['flags'][n + 2:] = 0, 0
    rows['at'], rows['blend'] = np.round(1e3 * at), np.round(1e3 * blend)
    t = push_s * (np.arange(n) + 1.0) / n
    rows['p'][0, z], rows['p'][0, k] = dip, now[k]
    rows['p'][1:n + 1, z] = dip - rise((dip - leap.STRAIGHT) * per, 1.0, t) / per
    rows['p'][1:n + 2, k], rows['p'][1:n + 1, d], rows['p'][n + 1, d] = THROW_K, 2.0 * math.sqrt(THROW_K * fr[modal.WEIGHT] / G), 0.0
    rows['p'][0:n + 2, zmp] = modal.NONE
    rows['p'][n + 1, z] = leap.STRAIGHT
    rows['flags'][1] |= ortho.RISE
    # the robot's drive as it had it through the leap, the robot's hold untouched
    drive = latent.AMP.start + modal.DRIVE
    rows['p'][0:n + 3, drive] = now[drive]
    landing = next(r[2] for r in leap._leap_specs() if r[:2] == (2, d))
    rows['p'][n + 2, z], rows['p'][n + 2, k], rows['p'][n + 2, d] = leap.bottom(fr, v / per), 0.0, landing
    rows['p'][n + 2, zmp] = now[zmp]
    rows['p'][n + 3, z], rows['p'][n + 3, k], rows['p'][n + 3, d], rows['p'][n + 3, zmp] = now[z], now[k], now[d], now[zmp]
    names = ('load',) + ('throw',) * n + ('flight', 'land', label)
    return rows.tobytes(), names, 1e-3 * float(rows['at'][-1] + rows['blend'][-1]), int(rows['at'][1])
