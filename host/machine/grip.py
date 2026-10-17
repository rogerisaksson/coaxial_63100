"""Her soles' grip as she feels it: each one's floor friction, read from what it used as it slid.

    grip = grip.laid()                                          # her law's own, MU, a sole each
    grip = grip.felt(grip, frames, touch, slip, weight, dt, prior)    # a pass of her law on

A bearing sole that slides (`estimate`'s slip) uses the floor's friction: the share of its push
across the floor. One that holds has at least that. Read across her IMU's up: across a sole's own,
one pitched 0.2 rad at its heel's strike read its push 0.2 across. Unread, a grip goes back to
what she knows of the floor (`prior`: told, else MU). On the ice (0.02) sliding soles
used 0.02 at the median; walking a level floor her soles used 0.39 at the 99th percentile. Read
from what her step asked against what it got, the drives' lag at a walk's start read 0.05 on a
floor of 1.0.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import edge
from machine.wbc import MU

#: Each sole's segments with skin: its foot and its toes.
SOLES = np.array([[edge.SKIN.index(side + '_foot'), edge.SKIN.index(side + '_toes')] for side in ('left', 'right')])
#: A sole reads its floor bearing BEARS of her weight; past SLIDE_M_S of slip it slides, its grip
#: drawn to what it used over SLIDE_S. Unread, a grip goes back to its prior over RELAX_S.
BEARS, SLIDE_M_S, SLIDE_S, RELAX_S = 0.1, 0.15, 0.05, 3.0
UP = np.array([0.0, 1.0, 0.0])


def laid():
    """Each sole's grip before she has felt a floor: her law's own."""
    return np.full(2, MU)


def felt(grip, frames, touch, slip, weight, dt, prior=MU):
    """Each sole's grip `dt` s on (2,): `frames` (2, 2, 3, 3) her model's of each sole's foot and
    toes, `touch` the edge's (SKIN, 3), `slip` her bearing soles' (3,, world, m/s), `weight` hers,
    N; `prior` what she knows of the floor."""
    pushed = np.einsum('kjab,kjb->ka', frames, touch[SOLES])
    press = pushed @ UP
    used = np.linalg.norm(pushed - press[:, None] * UP, axis=1) / np.maximum(press, 1e-9)
    bears = press >= BEARS * weight
    slides = bears & (np.hypot(slip[0], slip[2]) > SLIDE_M_S)
    held = np.where(bears & ~slides, np.maximum(grip, used), grip)
    return np.minimum(np.where(slides, grip + min(1.0, dt / SLIDE_S) * (used - grip),
                               held + dt / RELAX_S * (prior - held)), MU)
