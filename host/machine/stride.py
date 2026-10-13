"""Her stride on her latent loop: when a limb swings, where its sole lands, how a sole rolls.

    stepping, way, sweep, phi, off, u, flying, swung = stride.beat(drive, psi, sway, lateral, turns)
    cruise = stride.cruises(way, drive, turns, going)       # the speed her drive makes of its steps
    meet = stride.meets(xi, c, flat, cruise, om, turns, wide)       # where each sole lands (2, 2)
    tip, tipping, tipped = stride.rocks(rock, rate, acc)    # each sole's pitch as her rock has it
    fold, short = stride.folds(off, loads, bent)            # a knee's fold before its swing, rad and m
    rocked, flatten, pin, round = stride.heels(sole, hips, floor, long, short, off, tip, stance, stepping, was, dt)
    clear = stride.clears(u, flying, swung, high, tip, tipping, tilt, late)     # (3, 2), over the floor
    acc = stride.goes(meet, flat, shod, v, u, turns, kp, kd)        # a striding sole's, along and across

Arrays in, arrays out and no branch (tests/test_ortho.py); `machine.modal` renders with it. Her
stride is her drive's: a limb swings SWING of a turn, about where its drive carries it on
fastest, and stands the rest. From where the other's swing ends it stands over its ball, its
heel rising, her ZMP in the other sole; it leaves as its own swing begins. In the air its
lowest point is held clear of the floor, its sole pitched as her rock asks - the heel up behind
her, the toes up ahead -, and it goes to where it meets her capture point.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np  # noqa: E402

from machine.figure import ANKLE_H, BALL, SHANK, THIGH, TOE_M  # noqa: E402
from machine.poise import BIG, HALF_M, TINY, clip  # noqa: E402
from machine.wbc import CONTACT_K  # noqa: E402

#: A limb's place, her way's frame: along, to her left, up. Her limbs' mirrors, the first her left.
ALONG, LEFT, UP = 0, 1, 2
LIMBS = np.array([1.0, -1.0])
#: A unit of her drive, a limb's place from her centre of mass along her way, m; of her
#: soles' rock, a sole's pitch, rad.
STRIDE_M, ROCK_RAD = 0.3, 0.5
#: Her drive's amplitude over STIRS, she strides: a limb swings SWING of a turn, a woman's 38
#: to 40 %. Let go only with her ZMP in the other sole, a step was no longer than a sole -
#: 0.23 legs at 0.28 m/s, a shuffle by name (`tools.sim.normal`).
STIRS, SWING = 0.02, 0.4
PHI_W = np.pi * SWING
#: Her sway a quarter turn behind her drive, past BACK of it, she goes the other way.
BACK = 0.5
#: Her going: her centre of mass asked no more than AHEAD_M_S faster or slower than her
#: soles have made of late.
AHEAD_M_S = 0.36
#: A striding sole lands no further than REACH_M from her along her way, APART_M and more
#: its own side of the other; its spring stiffens till SOON_S before it is there. It is up
#: over the first RISE_U of its swing, comes down over the last FALL_U and is over its
#: landing ARRIVE_U through it.
REACH_M, APART_M, SOON_S = 0.35, 0.1, 0.02
RISE_U, FALL_U, ARRIVE_U = 0.2, 0.25, 0.9
#: And further on by LEADS of her speed past her cruise, over omega. A step's push - her
#: centre of mass past the standing sole's ball till the next lands - is 0.14 m/s more than
#: its landing's brake at any speed, and the landing's blow took that: on a step that lands
#: softer she was down at 19 s with no lead, at 36 s with 0.25. With 0.5 a step leaves 0.42
#: of its speed's error to the next, with 1 it turns it -0.09.
LEADS = 0.5
#: Past its swing a sole not down reaches on down at REACH_M_S, for REACH_S at most.
REACH_M_S, REACH_S = 0.45, 0.25
#: A standing sole is turned about the edge it stands on, 1/s^2: flat, or over its ball as
#: its stride has it. The loop's own hold on a standing sole damps its turn at 20 /s: stood
#: on with its heel 4 deg up, it stayed on its ball for 3 s. At 100 /s^2 and her ZMP let
#: anywhere under it, a heel asked 6 deg up under all of her stayed down till the other
#: sole landed.
FLAT_K = 400.0
#: A standing sole's heel rises no further than HEEL_RAD, no faster than HEEL_RAD_S, as her
#: hip goes ahead of it: her hip let DIP_M down from where it stood over a sole first. It
#: is on its edge in full from EDGE_RAD of pitch. Striding only: her soles stand 0.5 deg
#: toes up, and held on their heels for that she went over them 0.7 s after a hop's
#: landing; let 1.1 deg before an edge counts, her walk was down in 17 s.
HEEL_RAD, HEEL_RAD_S, DIP_M, EDGE_RAD = 0.6, 6.0, 0.006, 0.03
#: A standing sole its stride has off folds its knee as its load leaves it, FOLD_RAD with
#: none on it - its leg that much shorter for its heel's rise, her knees' row told. Asked by
#: nothing, the leg stayed at its length till its toes left, 18 deg where a woman's is 30:
#: on stilts by name. At 0.17 stiff 0.41; at 0.35 her head nods 7 deg and she strikes 489 N;
#: at 0.52 down at 14 s. Its heel alone asked up, down within 32 s.
FOLD_RAD = 0.26
#: A swinging sole's pitch is its rock's: its heel HEEL_UP further up behind her, its toes
#: as much less up ahead.
HEEL_UP = 0.5


def beat(drive, psi, sway, lateral, turns):
    """Her stride of her drive (its amplitude and angle) and her sway's: (whether she strides,
    0 to 1; her way, 1 or -1; each limb's place in its sweep, 1 its furthest on her way; its
    turn of it, 0 as it passes under her going on; whether her stride has it off the floor's
    count - it swings PHI_W either side of that, and is off from where the other's swing
    ended -; its swing's share done; whether it swings; that share's rate, 1/s)."""
    stepping = clip(np.abs(drive) / STIRS, 0.0, 1.0)
    way = 1.0 - 2.0 * (np.sin(lateral - psi) * np.sign(sway) < -BACK)
    side = -LIMBS * np.sign(drive) * way
    sweep, phi = side * np.sin(psi), np.arctan2(side * np.sin(psi), side * np.cos(psi))
    off = stepping * (phi > PHI_W - np.pi) * (phi < PHI_W)
    u = clip(0.5 * (phi / PHI_W + 1.0), 0.0, 1.0)
    return stepping, way, sweep, phi, off, u, off * (u > 0.0), 0.5 * turns / PHI_W


def cruises(way, drive, turns, going):
    """The speed her drive makes of its steps - a step her sweep from landing to landing, two
    a turn -, no further than AHEAD_M_S from her `going`, the speed her soles have made of
    late: asked 0.48 m/s under a sway her soles refused, she leant to her toes' edge
    standing and went over them; damped to her going alone she never set out."""
    cruise = way * 2.0 / np.pi * STRIDE_M * np.abs(drive) * np.sin(PHI_W) * turns
    return clip(cruise, going - AHEAD_M_S, going + AHEAD_M_S)


def meets(xi, c, flat, cruise, om, turns, wide):
    """Where each sole lands (2, 2: along, across): it meets her capture point `xi`, carried
    along with her - a step's offset short of it along her going, what her `cruise` makes of
    a step, and the sole's own side of it by her width's (`wide` m between her soles). On its
    sweep's place from her as her drive has it, a sole came down 15 cm ahead of her at 0.31
    m/s, 7 cm past where she could go over it, and she stopped and went back; to where her
    capture point was foreseen from her ZMP, e^(omega t) on, she lost 0.06 m/s a landing and
    went 0.33 m/s where 0.46 was asked; half a step ahead of her, she went 0.2 m/s: over a
    leg 16 cm ahead she rises 17 mm, 0.58 m/s to do it. No further than REACH_M from her
    centre of mass `c`, and APART_M its own side of the other sole (`flat` their middles)."""
    step_s = np.pi / np.maximum(turns, TINY)
    grown = np.exp(om * step_s)
    led = LEADS * np.maximum((xi[ALONG] - c[ALONG]) - cruise / om, -0.02)
    meet = xi - np.stack([cruise * step_s / (grown - 1.0) - led + 0.0 * LIMBS,
                          -LIMBS * wide / (grown + 1.0)], axis=1)
    return np.stack([c[ALONG] + clip(meet[:, ALONG] - c[ALONG], -REACH_M, REACH_M),
                     flat[::-1, LEFT] + LIMBS * np.maximum(LIMBS * (meet[:, LEFT] - flat[::-1, LEFT]), APART_M)],
                    axis=1)


def rocks(rock, rate, acc):
    """Each sole's pitch as her rock has it, rad, its rate and its acceleration: its heel up
    behind her, its toes up ahead (`rock` her rock's reference, `rate`, `acc` its own)."""
    heel = ROCK_RAD * (1.0 + HEEL_UP * np.sign(LIMBS * rock))
    return heel * LIMBS * rock, heel * LIMBS * rate, heel * LIMBS * acc


def folds(off, loads, bent):
    """A knee's fold before its swing, rad, and what it shortens its leg by, m (2 each): its
    stride has the sole off and its load leaves it - none of her soles' loads on it, all of
    FOLD_RAD. `bent` her knees' bend standing, rad."""
    fold = FOLD_RAD * off * (1.0 - loads / np.maximum(loads.sum(), TINY))
    return fold, (THIGH + SHANK) * (np.cos(0.5 * bent) - np.cos(0.5 * (bent + fold)))


def heels(sole, hips, floor, long, short, off, tip, stance, stepping, was, dt):
    """A standing sole's turn about the edge it stands on, striding: (the heel's rise asked,
    rad; its turn's acceleration, rad/s^2; how far it stands on an edge, 1 its ball and -1 its
    heel; its middle's acceleration round that edge, her way's frame (2, 3)). Flat; or, its
    stride `off` it, over its ball as far as her rock has its heel up (`tip`); or as far as
    its leg's length asks, her hip ahead of its ball - the heel's rise that lets the leg,
    `long` m less what its knee's fold shortens it (`short`), reach her hip DIP_M under
    where it stood over a sole. Flat till it left, a sole
    25 cm behind her had her hips 4 cm down as the other landed, on a knee at 36 deg. `sole`
    each sole's (place in her way's frame (2, 3), pitch, its rate); `hips` each hip's place
    along; `was` the rise asked the pass before, `dt` s ago."""
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
    rocked = np.maximum(off * np.maximum(tip, 0.0), need)
    flatten = stepping * stance * (FLAT_K * (rocked - tilt)
                                   + CONTACT_K * clip((rocked - was) / dt, -HEEL_RAD_S, HEEL_RAD_S))
    edge = np.sign(tilt + BIG * rocked)
    return (rocked, flatten, stepping * stance * edge * clip(np.maximum(np.abs(tilt), rocked) / EDGE_RAD, 0.0, 1.0),
            (edge * HALF_M)[:, None] * (flatten[:, None] * spoke + (tilting * tilting)[:, None] * inward))


def clears(u, flying, swung, high, tip, tipping, tilt, late):
    """A swinging sole's middle over the floor the standing soles touch, m, its rate and its
    acceleration (3, 2): its lowest point `high` clear. Up over the first RISE_U of its swing,
    down over the last FALL_U, held between: on a sine's square, 2 cm at its top, it was 9 mm
    up three quarters through and touched the floor there at 0.45 m/s, 12 cm short of where
    it was going. And over what its pitch brings its heel or its toes down by: its heel by
    its pitch as asked (`tip`, `tipping`) - as it was, a sole let go under 265 N turned up on
    its heel, 12 deg and rising; its toes by its pitch as it is (`tilt`), where further down:
    left with its heel 12 deg up as it was let go, it caught the floor with them half way
    through its swing. Its swing out and nothing under it (`late` s), it reaches on down:
    held at the floor's height, a sole 13 cm ahead of her bore nothing for 0.3 s past its
    swing and she went from 0.7 to 1.05 m/s over the one behind her."""
    on, by = clip(u / RISE_U, 0.0, 1.0), clip((1.0 - u) / FALL_U, 0.0, 1.0)
    eased = np.array([[on * on * (3.0 - 2.0 * on), by * by * (3.0 - 2.0 * by)],
                      [6.0 * on * (1.0 - on) / RISE_U, -6.0 * by * (1.0 - by) / FALL_U],
                      [6.0 * (1.0 - 2.0 * on) * (on < 1.0) / RISE_U ** 2,
                       6.0 * (1.0 - 2.0 * by) * (by < 1.0) / FALL_U ** 2]])
    down = np.maximum(tip, tilt)
    return np.array([
        high * flying * eased[0, 0] * eased[0, 1] - REACH_M_S * late
        + flying * (HALF_M * np.maximum(-np.sin(tip), 0.0) + (HALF_M + TOE_M) * np.maximum(np.sin(down), 0.0)),
        high * flying * (eased[1, 0] * eased[0, 1] + eased[0, 0] * eased[1, 1]) * swung - REACH_M_S * (late > 0.0)
        + flying * (-HALF_M * (tip < 0.0) + (HALF_M + TOE_M) * (tip > 0.0)) * np.cos(tip) * tipping,
        high * flying * (eased[2, 0] * eased[0, 1] + eased[0, 0] * eased[2, 1]) * swung ** 2])


def lates(phi, turns, stepping):
    """How long past its swing each limb is, s, REACH_S at most."""
    return stepping * clip((phi - PHI_W) / np.maximum(turns, TINY), 0.0, REACH_S)


def goes(meet, flat, shod, v, u, turns, kp, kd):
    """A striding sole's acceleration along and across (2, 2), to where it lands (`meet`) from
    where it is (`flat`, its speed `shod`, hers `v`): on a spring that stiffens as it nears -
    6 over the time left squared, damped 4 over it -, no stiffer than the limb's own (`kp`,
    `kd`, an axis each): it is over its landing ARRIVE_U through its swing."""
    to_go = np.maximum((ARRIVE_U - u) * 2.0 * PHI_W / np.maximum(turns, TINY), SOON_S)[:, None]
    return np.minimum(6.0 / to_go ** 2, kp) * (meet - flat) - np.minimum(4.0 / to_go, kd) * (shod[:, :2] - v[:2])
