"""How a sole rolls: her rock's pitch of it, a knee's fold before its swing, a standing heel's rise.

    tip, tipping, tipped = rocker.rocks(rock, rate, acc)    # each sole's pitch as her rock has it
    fold, short = rocker.folds(off, loads, bent)            # a knee's fold before its swing, rad and m
    rocked, flatten, pin, round, reach = rocker.heels(sole, hips, floor, long, short, off, tip, stance, stepping, was, dt, ground)
    level = rocker.pitched(ground)                          # the floor's pitch as her soles take it

Arrays in, arrays out and no branch (tests/test_ortho.py); `machine.law.impedance` renders with it, her
stride's numbers (`machine.law.stride`) its own.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body.figure import ANKLE_H, BALL, SHANK, THIGH
from machine.law.poise import BIG, HALF_M, TINY, clip
from machine.law.stride import ALONG, DIP_M, EDGE_RAD, FLAT_K, FOLD_RAD, HEEL_RAD, HEEL_RAD_S, HEEL_UP, LIMBS, ROCK_RAD, UP
from machine.stack.wbc import CONTACT_K


def rocks(rock, rate, acc):
    """Each sole's pitch as her rock has it, rad, its rate and its acceleration: its heel up
    behind her, its toes up ahead (`rock` her rock's reference, `rate`, `acc` its own)."""
    heel = ROCK_RAD * (1.0 + HEEL_UP * np.sign(LIMBS * rock))
    return heel * LIMBS * rock, heel * LIMBS * rate, heel * LIMBS * acc


def folds(off, loads, bent):
    """A knee's fold before its swing, rad, and what it shortens its leg by, m (2 each): its
    stride has the sole off and its load leaves it - none of her soles' loads on it, all of
    FOLD_RAD. `bent` her knees' bend standing, rad. Before its swing alone (`off` less the swing's
    share done): at the swing's end its sole lands, and the fold leaked there bent her landing knee
    37-39 deg, a woman's 0-12."""
    fold = FOLD_RAD * off * (1.0 - loads / np.maximum(loads.sum(), TINY))
    return fold, (THIGH + SHANK) * (np.cos(0.5 * bent) - np.cos(0.5 * (bent + fold)))


def heels(sole, hips, floor, long, short, off, tip, stance, stepping, was, dt, ground):
    """A standing sole's turn about the edge it stands on, striding: (the heel's rise asked,
    rad; its turn's acceleration, rad/s^2; how far it stands on an edge, 1 its ball and -1 its
    heel; its middle's acceleration round that edge, her way's frame (2, 3); the hip's height over
    the floor its leg reaches with its heel so, m - `vault.vaults` holds her hip there). Flat; or, its
    stride `off` it, over its ball as far as her rock has its heel up (`tip`); or as far as
    its leg's length asks, her hip ahead of its ball - the heel's rise that lets the leg,
    `long` m less what its knee's fold shortens it (`short`), reach her hip DIP_M under
    where it stood over a sole (held there her straight leg reaches 0.26 m ahead at most, her step
    0.5 m whatever her drive; held where the other leg lands straight instead, a long step's
    compass, her hip came down 1 cm early in each stance and her landing knee bent 28 deg for 23,
    her march at 1.3 Hz fell). Flat till it left, a sole
    25 cm behind her had her hips 4 cm down as the other landed, on a knee at 36 deg. `sole`
    each sole's (place in her way's frame (2, 3), pitch, its rate); `hips` each hip's place
    along; `was` the rise asked the pass before, `dt` s ago. Its pitch is read against the
    floor's under it (`ground`, each's): flat on a 10 deg ramp, read as on its heel, a sole's
    box was its heel's line and she fell 1.9 m up it."""
    place, tilt, tilting = sole
    spoke = np.stack([np.sin(tilt), 0.0 * tilt, np.cos(tilt)], axis=1)
    inward = np.stack([np.cos(tilt), 0.0 * tilt, -np.sin(tilt)], axis=1)
    ball = (place + HALF_M * inward)[:, [ALONG, UP]]
    hip = hips - ball[:, 0]
    high = floor + ANKLE_H + long - DIP_M - ball[:, 1]
    lever = np.hypot(hip * BALL - high * ANKLE_H, hip * ANKLE_H + high * BALL)
    need = clip(np.arctan2(-(hip * ANKLE_H + high * BALL), hip * BALL - high * ANKLE_H) + np.arccos(clip(
        ((long - short) ** 2 - hip * hip - high * high - BALL * BALL - ANKLE_H * ANKLE_H) / (2.0 * lever),
        -1.0, 1.0)),
        0.0, HEEL_RAD) * clip(hip / BALL, 0.0, 1.0)
    rocked, lean = np.maximum(off * np.maximum(tip, 0.0), need), tilt - pitched(ground)
    flatten = stepping * stance * (FLAT_K * (rocked - lean)
                                   + CONTACT_K * clip((rocked - was) / dt, -HEEL_RAD_S, HEEL_RAD_S))
    edge = np.sign(lean + BIG * rocked)
    # the hip's height over the floor the leg reaches with its heel up as its length needs: the asked height while
    # the heel can give it, the rocker's past HEEL_RAD - held to the asked height, a woman's heel capped, the hold
    # was never met; with the rock's heel, the hold lifted a crouch asked 0.25 (text 102) to 0.15
    reach = (ball[:, 1] - floor + BALL * np.sin(need) + ANKLE_H * np.cos(need)
             + np.sqrt(np.maximum((long - short) ** 2 - (hip + BALL * np.cos(need) - ANKLE_H * np.sin(need)) ** 2, 0.0)))
    return (rocked, flatten, stepping * stance * edge * clip(np.maximum(np.abs(lean), rocked) / EDGE_RAD, 0.0, 1.0),
            (edge * HALF_M)[:, None] * (flatten[:, None] * spoke + (tilting * tilting)[:, None] * inward), reach)


def pitched(ground):
    """The floor's pitch as her soles take it, rad: under half EDGE_RAD level, past it all of
    it - a level floor read 0.003 to 0.006 rad as her soft soles pressed it, and her pelvis
    rolled 4.96 deg where 5.05."""
    return ground * clip((np.abs(ground) - 0.5 * EDGE_RAD) / (0.5 * EDGE_RAD), 0.0, 1.0)
