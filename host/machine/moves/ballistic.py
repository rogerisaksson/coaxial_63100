"""Walk to run, a brake, a jump and a leap (the kernel's ballistic texts) on her body and her drives.

    fr = ballistic.froude(v, leg)                  # 1: v^2 / g leg
    beta = ballistic.duty(fr, knots, values)       # 1: a stance's share of a stride, the text's Hermite map
    dv = ballistic.braked(mu, cap, period)         # m/s: the most one step's stance brakes on a floor's friction
    n = ballistic.steps(v, v_walk, dv)             # steps from a run's speed to a walk's
    z = ballistic.crouch(fr)                       # axial: her deepest crouch to jump from, the text's
    wire, names, s, ms = ballistic.bound(fr, ch, turns, now, label)   # her leap in stride, a table's rows
    v, t, w = ballistic.launch(depth, legs)        # m/s, s, rad/s: her hips leaving the floor, her knees then
    h, x = ballistic.parabola(vz, vx)              # m: a flight's apex and span, landing as high as it left

The kernel's ballistic_*.lisp. Her legs' line as `leap._a_z` lays it (thigh, shank, the knee's lever to it); her
drives' bounds `bemf`'s at `drives.PACK_V`. Flat arrays and arithmetic: no branch.
"""
from machine.body import bemf, drives, figure

#: Gravity, m/s^2.
G = 9.81
#: Her knee among her drives (`bemf.DRIVEN`); a throw's height spring, 1/s^2 (the stride leap's table's).
THROW_K = 21600.0
KNEE = bemf.DRIVEN.index('left_knee')


def froude(v, leg):
    """1: a gait's Froude number at `v` m/s on a leg `leg` m long, hip over sole."""
    import numpy as np
    return np.asarray(v, float) ** 2 / (G * leg)


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
    return ((2 * t ** 3 - 3 * t ** 2 + 1) * y[k] + (t ** 3 - 2 * t ** 2 + t) * h[k] * m[k]
            + (3 * t ** 2 - 2 * t ** 3) * y[k + 1] + (t ** 3 - t ** 2) * h[k] * m[k + 1])


def braked(mu, cap, period):
    """m/s: the most a step's stance takes off her speed, `cap` of the friction `mu` her soles meet (the text's
    cone-utilization cap): over a step of `period` s her soles bear her weight on average, so mu cap g period."""
    import numpy as np
    return np.asarray(mu, float) * cap * G * period


def steps(v, v_walk, dv):
    """Steps that brake `v` m/s to `v_walk` at `dv` a step: ceil((v - v_walk) / dv), the text's N_brake."""
    import numpy as np
    return np.ceil((np.asarray(v, float) - v_walk) / dv).astype(int)


def _push(depth, legs, n):
    """Her push from `depth` m under her straight legs (`launch`): (her rise from the crouch along her legs' line, m,
    her speed there, m/s, the sample she leaves at, her knees' levers, m)."""
    import numpy as np
    straight = figure.THIGH + figure.SHANK
    x = np.linspace(straight - depth, straight, n + 1)[:-1]
    cos = (figure.THIGH ** 2 + figure.SHANK ** 2 - x ** 2) / (2.0 * figure.THIGH * figure.SHANK)
    lever = figure.THIGH * figure.SHANK * np.sqrt(1.0 - cos ** 2) / x
    m = figure.mass()
    big = m + legs * drives.armature('left_knee') / lever ** 2
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
    """(m/s, s, rad/s): her hips' speed leaving the floor, the push's seconds and her knees' speed then, pushed from
    `depth` m under her straight legs by `legs` knees at their back-EMF bounds, their rotors spun with them, her
    weight against them. Her kinetic energy, her rotors' with it, gains the push's work and loses her weight's; she
    leaves where the floor's push would turn to a pull, or straight."""
    import numpy as np
    up, v, k, lever = _push(depth, legs, n)
    return float(v[k]), float(np.sum((up[1] - up[0]) / np.maximum(v[1:k + 1], 1e-9))), float(v[k] / lever[k])


def rise(depth, legs, at):
    """m: her hips' rise from the crouch `depth` m under her straight legs at `at` s into her push (`launch`'s), her
    leaving's from its end on."""
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
    """Axial: her deepest crouch to jump from (`leap.standing_leap`), the text's countermovement at her size
    (`xform-standing-bilateral-jump`) over her axial metre in her frame `fr` - from the human's crouch (`leap.CROUCH`)
    leant 55 deg she fell 36 ms free into it, 343 mm deep, onto one sole, and left 74 mm under her standing height."""
    from machine.texts import kernel
    from machine.law import modal
    return 1e-3 * kernel.said('xform-standing-bilateral-jump', 'countermovement-crouch-mm') / (
        modal.AXIAL_M * fr[modal.HIPS_BY])


def bound(fr, ch, turns, now, label, speed=0.69):
    """(wire, names, seconds, the ms its push begins) of her leap in stride (`xform-in-stride-unilateral-leap`) from
    the sole landing now: its stance dipped by the text's countermovement (a tap, charge `ch` 0, half of it) and pushed
    straight by one knee at its WEP (`launch`, one leg) to leave as her other sole's swing ends - half a turn of her
    rhythm `turns` rad/s on -, her flight in place of her double support, her rhythm untouched: the other sole lands
    meant to stand, this one swings at its own time. The push's spring held into the flight (let go at its curve's end
    she stood on, 0.28 m/s up), her other swing held through the push (ZMP none), her landing waiting on her soles;
    her gait's height, spring and support as she had them (`now`) after, the last row named `label`. Braked to a stand
    first, her leap from a run went no further than her walk's."""
    import math
    import numpy as np
    from machine.texts import kernel, ortho
    from machine.law import latent, modal, stride
    from machine.moves import leap
    z, k, zmp, d = latent.Z.start + modal.AXIAL, latent.K.start + modal.AXIAL, latent.ZMP, latent.D.start + modal.AXIAL
    per = modal.AXIAL_M * fr[modal.HIPS_BY]
    # dipped by the leap's countermovement, a hold thrown further ahead (the stride leap's sagittal throw): half the
    # countermovement flew a walk's tap 0.13 s, the standing jump's felled a hold on landing
    told = kernel.node('xform-in-stride-unilateral-leap', 'ssc-launch')['phase-1-countermovement']['crouch-depth-m']
    dip = float(now[z]) + told / per
    v, push_s, _w = launch((dip - leap.STRAIGHT) * per, legs=1.0)
    leaves = math.pi / max(float(turns), 1e-9)
    dip_s = max(min(push_s, leaves - push_s), 1e-3 * leap.PUSH_MS)
    t_dip = max(0.0, leaves - push_s - dip_s)
    n = max(2, int(math.ceil(1e3 * push_s / leap.PUSH_MS)))
    off = t_dip + dip_s + push_s
    land = off + 2.0 * v / G
    at = np.concatenate([[t_dip], t_dip + dip_s + push_s * np.arange(n) / n, [off, land, land + dip_s]])
    blend = np.concatenate([[dip_s], np.full(n, push_s / n), [1e-3 * leap.PUSH_MS, dip_s, dip_s]])
    rows = np.zeros(n + 4, latent.WIRE)
    rows['p'], rows['flags'] = np.nan, ortho.CURVE
    rows['flags'][0], rows['flags'][n + 2:] = 0, 0
    rows['at'], rows['blend'] = np.round(1e3 * at), np.round(1e3 * blend)
    t = push_s * np.arange(n) / n
    rows['p'][0, z], rows['p'][0, k] = dip, now[k]
    rows['p'][1:n + 1, z] = dip - rise((dip - leap.STRAIGHT) * per, 1.0, t) / per
    rows['p'][1:n + 2, k], rows['p'][1:n + 2, d] = THROW_K, 0.0
    rows['p'][0:n + 2, zmp] = modal.NONE
    rows['p'][n + 1, z] = leap.STRAIGHT
    rows['flags'][1] |= ortho.RISE
    thrown = 0.06 + 0.28 * ch + 0.08 * min(1.0, max(0.0, (float(np.nan_to_num(speed)) - 0.55) / 0.20))
    rows['p'][1:n + 1, latent.Z.start + modal.SAGITTAL] = 0.28 + 0.60 * thrown
    rows['p'][1:n + 1, latent.K.start + modal.SAGITTAL] = 1800.0 + 1000.0 * thrown
    rows['p'][n + 2, latent.Z.start + modal.SAGITTAL] = now[latent.Z.start + modal.SAGITTAL]
    rows['p'][n + 2, latent.K.start + modal.SAGITTAL] = now[latent.K.start + modal.SAGITTAL]
    # her stride swept as much further as her flight carries her: at her walk's sweep her held sole reached 0.14 m
    # short of her centre and she landed on both soles behind it
    drive = latent.AMP.start + modal.DRIVE
    rows['p'][0:n + 2, drive] = now[drive] * (1.0 + 2.0 * v / G / (2.0 * stride.PHI_W / max(float(turns), 1e-9)))
    rows['p'][n + 2, drive] = now[drive]
    landing = next(r[2] for r in leap._leap_specs() if r[:2] == (2, d))
    rows['p'][n + 2, z], rows['p'][n + 2, k], rows['p'][n + 2, d] = leap.bottom(fr, v / per), 0.0, landing
    rows['p'][n + 2, zmp] = now[zmp]
    rows['p'][n + 3, z], rows['p'][n + 3, k], rows['p'][n + 3, d], rows['p'][n + 3, zmp] = now[z], now[k], now[d], now[zmp]
    names = ('load',) + ('throw',) * n + ('flight', 'land', label)
    return rows.tobytes(), names, 1e-3 * float(rows['at'][-1] + rows['blend'][-1]), int(rows['at'][1])
