"""Walk to run, a brake, a jump and a leap (the kernel's ballistic texts) on her body and her drives.

    fr = ballistic.froude(v, leg)                  # 1: v^2 / g leg
    beta = ballistic.duty(fr, knots, values)       # 1: a stance's share of a stride, the text's Hermite map
    dv = ballistic.braked(mu, cap, period)         # m/s: the most one step's stance brakes on a floor's friction
    n = ballistic.steps(v, v_walk, dv)             # steps from a run's speed to a walk's
    v, t, w = ballistic.launch(depth, legs)        # m/s, s, rad/s: her hips leaving the floor, her knees then
    h, x = ballistic.parabola(vz, vx)              # m: a flight's apex and span, landing as high as it left

The kernel's ballistic_*.lisp. Her legs' line as `leap._a_z` lays it (thigh, shank, the knee's lever to it); her
drives' bounds `bemf`'s at `drives.PACK_V`. Flat arrays and arithmetic: no branch.
"""
from machine import bemf, drives, figure

#: Gravity, m/s^2.
G = 9.81
#: Her knee among her drives (`bemf.DRIVEN`).
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


def launch(depth, legs=2.0, n=4000):
    """(m/s, s, rad/s): her hips' speed leaving the floor, the push's seconds and her knees' speed then, pushed from
    `depth` m under her straight legs by `legs` knees at their back-EMF bounds, their rotors spun with them, her
    weight against them. Her kinetic energy, her rotors' with it, gains the push's work and loses her weight's; she
    leaves where the floor's push would turn to a pull, or straight."""
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
    k = int(np.argmax(np.append(m * (acc[:-1] + G) <= 0.0, True)))
    return float(v[k]), float(np.sum(dx / np.maximum(v[1:k + 1], 1e-9))), float(v[k] / lever[k])


def parabola(vz, vx):
    """(m, m): a flight's apex over where it left and its span, landing as high: vz^2 / 2g, 2 vx vz / g."""
    import numpy as np
    vz, vx = np.asarray(vz, float), np.asarray(vx, float)
    return vz * vz / (2.0 * G), 2.0 * vx * vz / G
