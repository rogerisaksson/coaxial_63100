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
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

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
