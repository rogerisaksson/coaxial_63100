"""Her soles' grip as she feels it: each one's floor friction, read from what it used as it slid.

    grip = grip.laid()                                          # her law's own, MU, a sole each
    grip = grip.felt(grip, seen, frames, touch, slip, weight, dt, prior)  # a pass of her law on

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
#: drawn to what it used over SLIDE_S - over 0.05 s a sole pushing off on ice slid back 0.3 m
#: before her law asked less of it -; holding, up to what it used over TRUST_S - at once, a sole
#: off the ice onto the floor freed her stride in a pass and she ran on to 1.7 m/s. Unread, a
#: grip goes back to its prior over RELAX_S: at 3 s the ice read at 0.103 was 0.25, her
#: envelope's, within a second, she drove on to 1.06 m/s and fell at 6.6-6.7 s; at 30 she kept
#: it, slowed to 0.2 m/s by 4 s and crossed.
BEARS, SLIDE_M_S, SLIDE_S, TRUST_S, RELAX_S = 0.1, 0.15, 0.006, 1.0, 30.0
#: A slide's friction within 45 deg of against it. A slide counts once its share has held within
#: PLATEAU of itself over its last SEEN passes: sliding, friction is a constant - on the ice 0.099
#: to 0.101 while its load fell 348 -> 194 N; a hard landing's skid read 0.174 -> 0.137 in three,
#: a sole over a sill's edge 0.122 -> 0.096, and the sill fell at every wobble.
AGAINST, PLATEAU, SEEN = 0.7, 0.05, 4
UP = np.array([0.0, 1.0, 0.0])


def laid():
    """Each sole's grip before she has felt a floor: her law's own."""
    return np.full(2, MU)


def outside(grip, grips):
    """Each sole's grip as her law takes it: her own, MU, inside her envelope - over every
    situation's bound on it (`grips`, `situation.text`) -, the grip felt as far as she is outside
    it, the fifth under the highest bound from one to the other. Read whole, her estimator's slides of a
    turnabout's pivot, a shove's skid and a weave read 0.27 to 0.55 on a floor of 1.0, bounded
    her landings, and walking 80 N shoves fell 30 -> 25."""
    top = np.max(grips, initial=0.0)
    out = np.clip((top - grip) / (0.2 * max(top, 1e-9)), 0.0, 1.0)
    return MU + out * (grip - MU)


def felt(grip, seen, frames, touch, slip, weight, dt, prior=MU, landed=(False, False)):
    """Each sole's grip `dt` s on (2,), `seen` (2, SEEN) its last shares used sliding (0 holding)
    moved on: `frames` (2, 2, 3, 3) her model's of each sole's foot and toes, `touch` the edge's
    (SKIN, 3), `slip` her bearing soles' (3,, world, m/s), `weight` hers, N; `prior` what she
    knows of the floor. A sole `landed` this pass is on the floor the other stands on, as far as
    that is known under the prior - kept from where it stood itself, past a slick patch a
    sole's reach was 5 cm on a floor of 1.0 for a second and she fell there."""
    # a landing sole meets the floor the other stands on, as far as that is known under her prior
    grip = np.where(landed, np.minimum(prior, grip.min()), grip)
    pushed = np.einsum('kjab,kjb->ka', frames, touch[SOLES])
    press = pushed @ UP
    across = pushed - press[:, None] * UP
    used = np.linalg.norm(across, axis=1) / np.maximum(press, 1e-9)
    bears = press >= BEARS * weight
    # friction on a sliding sole is against its slide: her estimator's slip past SLIDE_M_S in a
    # turnabout's pivot, a trip or a shove read as slides, and walking 80 N shoves fell 30 -> 25
    against = across @ slip <= -AGAINST * np.linalg.norm(across, axis=1) * np.linalg.norm(slip)
    moving = bears & (np.hypot(slip[0], slip[2]) > SLIDE_M_S) & against
    seen[:, :-1], seen[:, -1] = seen[:, 1:].copy(), moving * used
    slides = moving & (seen.min(axis=1) > 0.0) & (seen.max(axis=1) - seen.min(axis=1) <= PLATEAU * seen.mean(axis=1))
    held = grip + (bears & ~moving) * min(1.0, dt / TRUST_S) * np.maximum(used - grip, 0.0)
    return np.minimum(np.where(slides, grip + min(1.0, dt / SLIDE_S) * (used - grip),
                               held + min(1.0, dt / RELAX_S) * (prior - held)), MU)
