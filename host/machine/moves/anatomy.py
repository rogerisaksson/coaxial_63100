"""The robot's moves' anatomy: each joint driven as its phase says, its trunk held by its hips.

A move the robot's joints play (`machine.process.joints.MOVES`) is its tables of phases over the robot's parts and
the senses that end them; how a part goes is this module's (HELD .. LEAST), the same for each move - the robot's
way out of a bed over its end, the robot's scoot on the floor, its moves whose phases end by what the
robot senses (`machine.moves.bedexit`, `scoot`, `sensed`) drive it through it, the scoot's strokes setting
its goals and rates. In C: wbc_anatomy.c, bit for bit.

    goal = anatomy.aimed(how, goal, x)                          # the goal it goes to: LEAST no straighter than x
    x = anatomy.driven(how, goal, rate, x, q, start, a, dt)   # each joint's command a pass on, deg
    x = anatomy.held(x, gain, pitch, trunk, dt, lo, hi)        # the robot's hips holding its trunk's pitch
"""
import functools
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

#: Target capsule surface gap (m), hip abduction and shoulder flexion (deg) for hands resting on mid-thighs.
THIGH_LAP_GAP, THIGH_HIP_YAW_DEG, THIGH_SHOULDER_DEG = -0.012, 12.0, -4.0

#: How a part goes: held where it is; blended over its phase from where it began; to its goal at its rate (deg/s);
#: the robot's sensed angle and its goal past it; drawn toward its goal from the robot's sensed angle by its rate
#: (deg) at most, yielding past it; to its goal at its rate but no straighter than it is.
HELD, BLEND, RATE, FOLLOW, TOWARD, LEAST = 0.0, 1.0, 2.0, 3.0, 4.0, 5.0


def aimed(how, goal, x):
    """Each joint's goal as it goes (deg): LEAST's no straighter than its command `x` is, the larger of the two."""
    return np.where(how == LEAST, np.maximum(x, goal), goal)


def driven(how, goal, rate, x, q, start, a, dt):
    """Each joint's command `dt` s on (deg): `how` it goes (HELD .. LEAST), its `goal` and `rate`, its command `x`,
    the robot's sensed angle `q`, where its phase began `start`, `a` of its phase's way."""
    reach = rate * dt
    goal = aimed(how, goal, x)
    return np.where(how == BLEND, start + a * (goal - start),
                    np.where((how == RATE) | (how == LEAST), x + np.clip(goal - x, -reach, reach),
                             np.where(how == FOLLOW, q + goal,
                                      np.where(how == TOWARD, q + np.clip(goal - q, 0.0, rate), x))))


def move(name, *numbers):
    """Move `name`'s table as the suite's lisp says it (`defmove`, moves.lisp), given the robot's body's
    `numbers`: {'constants': {name: value}, 'phases': ({'name', 'seconds', 'next', 'goals', 'over', 'trunk',
    ..} ..)}, its keywords ':'-led."""
    from machine.texts import dsp
    return dsp.value([name] + list(numbers), dsp.read(dsp.SUITE))


def held(x, gain, pitch, trunk, dt, lo, hi):
    """`x` (deg), the robot's hips holding its trunk's `pitch` at `trunk` (deg): moved `gain` deg/s a deg of
    its error, within [`lo`, `hi`]."""
    return np.clip(x + gain * (pitch - trunk) * dt, lo, hi)


@functools.cache
def _on_thighs_rad():
    from machine.body import figure
    deg = dict(figure.stand(), left_hip=-90.0, left_knee=90.0,
               left_hip_yaw=THIGH_HIP_YAW_DEG, left_shoulder=THIGH_SHOULDER_DEG)
    hands = tuple((seg, r, np.asarray(a, float), np.asarray(c, float))
                  for seg, r, a, c in figure.BODY if seg in ('hand', 'fingers'))
    thighs = tuple((r, np.asarray(a, float), np.asarray(c, float))
                   for seg, r, a, c in figure.BODY if seg == 'thigh')

    def gap(el_deg):
        deg['left_elbow'] = el_deg
        fr = figure.frames(deg, (0.0, 0.0, 0.0), figure.IDENT)
        tp, tR = fr['left_thigh']
        tp_v, tR_m = np.asarray(tp, float), np.asarray(tR, float)
        tw = [(tr, tp_v + tR_m @ ta0, tp_v + tR_m @ tb0) for tr, ta0, tb0 in thighs]
        best = math.inf
        for seg, hr, ha0, hb0 in hands:
            sp, sR = fr['left_' + seg]
            sp_v, sR_m = np.asarray(sp, float), np.asarray(sR, float)
            for frac in (0.0, 0.5, 1.0):
                pt = sp_v + sR_m @ (ha0 + frac * (hb0 - ha0))
                for tr, ta, tb in tw:
                    u = tb - ta
                    t = float(np.clip(((pt - ta) @ u) / (u @ u), 0.0, 1.0))
                    best = min(best, float(np.linalg.norm(pt - (ta + t * u)) - hr - tr))
        return best

    lo, hi = 50.0, 90.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if gap(mid) < THIGH_LAP_GAP else (lo, mid)
    return math.radians(THIGH_SHOULDER_DEG), math.radians(0.5 * (lo + hi)), math.radians(THIGH_HIP_YAW_DEG)


def on_thighs(modes=None):
    """(shoulder, elbow, hip_yaw) in rad, or arm modes with hip_yaw in rad given `modes`, resting the
    robot's hands lightly on top of its abducted mid-thighs while seated upright (`figure.BODY`)."""
    shoulder, elbow, hip_yaw = _on_thighs_rad()
    if modes is None:
        return shoulder, elbow, hip_yaw
    from machine.body import figure
    stand_elbow = math.radians(figure.stand()['left_elbow'])
    return shoulder / modes['mode_arm_reach'][0], (elbow - stand_elbow) / modes['mode_arm_fold'][0], hip_yaw
