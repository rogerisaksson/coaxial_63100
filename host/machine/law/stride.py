"""Her stride on her latent loop: when a limb swings, where its sole lands, how a sole rolls.

    stepping, way, sweep, phi, on, off, u, flying, swung = stride.beat(drive, psi, sway, lateral, turns, was, need, lead, split, begun)
    start = stride.first(need, still, rate, was, beyond, loads, phi)    # rad: a step begun at once
    split = stride.split(split, support, stepping, free, ahead, phi, u, turns, dt, start)   # rad: each limb's own shift
    hurry = stride.hurried(aside, om, u, swung, kp)                  # (2,), 1 unhurried
    cruise = stride.cruises(way, drive, turns, going)       # the speed her drive makes of its steps
    meet = stride.meets(xi, c, flat, cruise, om, turns, wide)       # where each sole lands (2, 2)
    tip, tipping, tipped = stride.rocks(rock, rate, acc)    # each sole's pitch as her rock has it
    fold, short = stride.folds(off, loads, bent)            # a knee's fold before its swing, rad and m
    rocked, flatten, pin, round = stride.heels(sole, hips, floor, long, short, off, tip, stance, stepping, was, dt, ground)
    clear = stride.clears(u, flying, swung, high, tip, tipping, tilt, late)     # (3, 2), over the floor
    acc = stride.goes(meet, flat, shod, v, u, turns, kp, kd)        # a striding sole's, along and across

Arrays in, arrays out and no branch (tests/test_ortho.py); `machine.law.modal` renders with it. Her
stride is her drive's: a limb swings SWING of a turn, about where its drive carries it on
fastest, and stands the rest. From where the other's swing ends it stands over its ball, its
heel rising, her ZMP in the other sole; it leaves as its own swing begins, or as its heel is up
as far as a woman's as her toes leave - its swing then from there to its landing. In the air its
lowest point is held clear of the floor, its sole pitched as her rock asks - the heel up behind
her, the toes up ahead -, and it goes to where it meets her capture point.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body import gait
from machine.body.figure import ANKLE_H, BALL, SHANK, THIGH, TOE_M
from machine.law.poise import BIG, HALF_M, TINY, clip
from machine.stack.wbc import CONTACT_K

#: Her stride's numbers, her build's (`[gait.stride]`: each with what it was found at, Froude-similar
#: on another leg, `machine.body.gait`).
_G = gait.table('stride')
#: A limb's place, her way's frame: along, to her left, up. Her limbs' mirrors, the first her left.
ALONG, LEFT, UP = 0, 1, 2
LIMBS = np.array([1.0, -1.0])
#: A unit of her drive, a limb's place from her centre of mass along her way, m; of her
#: soles' rock, a sole's pitch, rad.
STRIDE_M, ROCK_RAD = _G['stride_m'], _G['rock_rad']
#: Her drive's amplitude over STIRS, she strides: a limb swings SWING of a turn.
STIRS, SWING = _G['stirs'], _G['swing']
PHI_W = np.pi * SWING
#: Her sway a quarter turn behind her drive, past BACK of it, she goes the other way.
BACK = _G['back']
#: Her going: her centre of mass asked no more than AHEAD_M_S faster or slower than her
#: soles have made of late.
AHEAD_M_S = _G['ahead_m_s']
#: A striding sole lands no further than REACH_M from her along her way, APART_M and more
#: its own side of the other; its spring stiffens till SOON_S before it is there. It is up
#: over the first RISE_U of its swing, comes down over the last FALL_U and is over its
#: landing ARRIVE_U through it.
REACH_M, APART_M, SOON_S = _G['reach_m'], _G['apart_m'], _G['soon_s']
RISE_U, FALL_U, ARRIVE_U = _G['rise_u'], _G['fall_u'], _G['arrive_u']
#: And further on by LEADS of her speed past her cruise, over omega.
LEADS = _G['leads']
#: And LAND_SHARE of the step's offset short of her capture point along her going, the linear inverted
#: pendulum's step at her cruise: 1 the walk whose centre of mass passes over the sole it lands.
LAND_SHARE = _G['land_share']
#: Past its swing a sole not down reaches on down at REACH_M_S, for REACH_S at most.
REACH_M_S, REACH_S = _G['reach_m_s'], _G['reach_s']
#: A standing sole is turned about the edge it stands on, 1/s^2: flat, or over its ball as
#: its stride has it.
FLAT_K = _G['flat_1_s2']
#: A standing sole's heel rises no further than HEEL_RAD, no faster than HEEL_RAD_S, as her hip
#: goes ahead of it: her hip let DIP_M down from where it stood over a sole first. It is on its
#: edge in full from EDGE_RAD of pitch.
HEEL_RAD, HEEL_RAD_S, DIP_M, EDGE_RAD = np.radians(_G['heel_deg']), _G['heel_rad_s'], _G['dip_m'], _G['edge_rad']
#: A standing sole its stride has off folds its knee as its load leaves it, FOLD_RAD with
#: none on it - its leg that much shorter for its heel's rise, her knees' row told.
FOLD_RAD = _G['fold_rad']
#: A swinging sole's pitch is its rock's: its heel HEEL_UP further up behind her, its toes
#: as much less up ahead.
HEEL_UP = _G['heel_up']


def beat(drive, psi, sway, lateral, turns, was, need, lead, split, begun):
    """Her stride of her drive (its amplitude and angle) and her sway's: (whether she strides,
    0 to 1, as far as her drive or her `need` has it; her way, 1 or -1; each limb's place in its
    sweep, 1 its furthest on her way; its turn of it, 0 as it passes under her going on;
    whether its rhythm has it off, from where the other's swing ended to PHI_W past that place;
    whether her stride has it off the floor's count; its swing's share done from where it `begun`,
    `lead` ahead of its rhythm's (`hurried`) - landed at its end -; whether it swings; its rhythm's rate of that
    share, 1/s). A swing begun is finished, off as far as her stride has had it since (`was`,
    the pass before's), its way kept with her drive let go: a spin in place stood where it fell
    at 7.94 s."""
    stepping = np.maximum(clip(np.abs(drive) / STIRS, 0.0, 1.0), need)
    way = 1.0 - 2.0 * (np.sin(lateral - psi) * np.copysign(1.0, sway) < -BACK)
    side = -LIMBS * np.copysign(1.0, drive) * way
    s = np.sin(psi) * np.cos(split) + np.cos(psi) * np.sin(split)
    c = np.cos(psi) * np.cos(split) - np.sin(psi) * np.sin(split)
    sweep, phi = side * s, np.arctan2(side * s, side * c)
    on = (phi > PHI_W - np.pi) * (phi < PHI_W)
    u = clip((0.5 * (phi / PHI_W + 1.0) - begun) / (1.0 - begun) + lead, 0.0, 1.0)
    off = np.maximum(stepping, was) * on * (1.0 - (lead > 0.0) * (u >= 1.0))
    return stepping, way, sweep, phi, on, off, u, off * (u > 0.0), 0.5 * turns / PHI_W


def split(split, support, stepping, free, ahead, phi, u, turns, dt, start=0.0):
    """Each limb's own shift of its place in her rhythm a pass on, rad (the ballistic text's split leap): held - a
    throw, a flight, her `support` asked none -, a sole `free` of the floor behind her centre (`ahead`, m along her
    way) and out of its swing goes to its swing's start, one ahead done swinging waits for the floor; else they relax
    to the nearest turn apart. Kept to her rhythm her trailing sole stood in the air 0.5 s and landed behind her; told
    by its rhythm alone, a sole pushed off early in its stance waited behind her 0.6 s. Both turned `start` (`first`)."""
    held, gap = 1.0 - clip(support, 0.0, 1.0), split[0] - split[1]
    return split + start + (stepping * held * free * ((ahead <= 0.0) * ((u <= 0.0) + (u >= 1.0)) * np.mod(-PHI_W - phi, 2.0 * np.pi)
                                                      - (ahead > 0.0) * (u >= 1.0) * turns * dt)
                            - (1.0 - held) * (0.5 * (gap - 2.0 * np.pi * np.round(gap / (2.0 * np.pi))) * LIMBS)
                            * min(1.0, turns * dt / (2.0 * np.pi)))


def first(need, still, rate, was, beyond, loads, phi):
    """Her rhythm's turn, rad, to the swing's start of the sole on her capture point's side, else the lighter, on a
    fresh `need` at rest (`still`, no `rate`, no swing `was`): to her rhythm, shoved 200 N, she fell from a lunge."""
    key = LIMBS * beyond[LEFT] - TINY * loads
    pick = 1.0 * (key > key[::-1]) + 1.0 * (key == key[::-1]) * (LIMBS > 0.0)
    fresh = still * (1.0 - clip(np.linalg.norm(rate) / TINY, 0.0, 1.0)) * (need > 0.5) * (np.max(was) < 0.5)
    return fresh * (pick @ np.mod(-PHI_W - phi, 2.0 * np.pi))


def hurried(aside, om, u, rate, kp):
    """How many times its rhythm's `rate` (1/s) each swing's share `u` goes: 1, or as fast as
    lands it before her capture point - `aside` m across past the soles that stood, its own side
    of them, growing as e^(omega t) - is a leg's reach (REACH_M) past them, in its spring's own
    time at the soonest (`kp`, 1/s^2: `goes`). Along her way it runs past them every step of a
    walk, and hurried swings from 10 s of her plain walk on; with no sole standing it read
    1000 m, a swing slammed down 3 times its pace and she fell. Its stance let go, a turn
    reversed landed its swing 0.54 s on and she fell; her whole rhythm hurried for it turned her
    pelvis with it and spun her 1.9 rad."""
    due = np.log(np.maximum(REACH_M / np.maximum(aside, TINY), 1.0)) / om
    return np.maximum(1.0, (1.0 - u) / (np.maximum(rate, TINY) * np.maximum(due, np.sqrt(6.0 / kp))))


def cruises(way, drive, turns, going):
    """The speed her drive makes of its steps - a step her sweep from landing to landing, two
    a turn -, no further than AHEAD_M_S from her `going`, the speed her soles have made of
    late: asked 0.48 m/s under a sway her soles refused, she leant to her toes' edge
    standing and went over them; damped to her going alone she never set out."""
    cruise = way * 2.0 / np.pi * STRIDE_M * np.abs(drive) * np.sin(PHI_W) * turns
    return clip(cruise, going - AHEAD_M_S, going + AHEAD_M_S)


def meets(xi, c, flat, cruise, om, turns, wide, across=0.0, reach=REACH_M, apart=APART_M):
    """Where each sole lands (2, 2: along, across): it meets her capture point `xi`, carried
    along with her - a step's offset short of it along her going, what her `cruise` makes of
    a step, and the sole's own side of it by her width's (`wide` m between her soles). On its
    sweep's place from her as her drive has it, a sole came down 15 cm ahead of her at 0.31
    m/s, 7 cm past where she could go over it, and she stopped and went back; to where her
    capture point was foreseen from her ZMP, e^(omega t) on, she lost 0.06 m/s a landing and
    went 0.33 m/s where 0.46 was asked; half a step ahead of her, she went 0.2 m/s: over a
    leg 16 cm ahead she rises 17 mm, 0.58 m/s to do it. No further than `reach` from her
    centre of mass `c`, and `apart` its own side of the other sole (`flat` their middles): APART_M,
    less as her width is asked narrower than she stood (`guarded` holds a swing off as it passes)."""
    step_s = np.pi / np.maximum(turns, TINY)
    grown = np.exp(om * step_s)
    led = LEADS * np.maximum((xi[ALONG] - c[ALONG]) - cruise / om, -0.04)
    s_ac = np.sign(across)
    led_lat = s_ac * clip(s_ac * (xi[LEFT] - c[LEFT]) - 1.35 * np.abs(across) / om, -0.052, 0.042)
    meet = xi - np.stack([LAND_SHARE * (cruise * step_s / (grown - 1.0)) - led + 0.0 * LIMBS,
                          -led_lat - LIMBS * wide / (grown + 1.0)], axis=1)
    return np.stack([c[ALONG] + clip(meet[:, ALONG] - c[ALONG], -reach, reach),
                     flat[::-1, LEFT] + LIMBS * np.maximum(LIMBS * (meet[:, LEFT] - flat[::-1, LEFT]), apart)],
                    axis=1)


def guarded(meet, over, flat, apart):
    """Each striding sole's target (2, 2: along, across) - from `flat` to its landing `meet` as
    far as it clears the floor (`over`) - held APART_M its own side of the other sole while their
    middles lie within a sole's length along her way, `apart` past it: a track narrower than her
    soles are wide passes the standing sole, then closes. As it was where far enough off."""
    target, other = flat + (meet - flat) * over[:, None], flat[::-1]
    overlap = clip(1.0 - np.abs(flat[:, ALONG] - other[:, ALONG]) / (2.0 * HALF_M), 0.0, 1.0)
    floor = apart + (APART_M - apart) * overlap
    held = LIMBS * np.maximum(floor - LIMBS * (target[:, LEFT] - other[:, LEFT]), 0.0)
    return np.stack([target[:, ALONG], target[:, LEFT] + held], axis=1)


def ducked(duck, held, stance, stood, dt):
    """Her height's sidechain (a row's `ortho.DUCK`: depth m, attack s, release s): a touchdown
    latches it, the envelope rises at the attack's rate to full, the latch lets go and it falls
    on the release's time constant. ((latch, envelope) as `held` was, (her height's duck, its
    rate), m): none at a depth of 0."""
    depth, attack, release = duck
    latch0, env0 = held
    latch = np.maximum(latch0 * float(env0 < 1.0), np.maximum(stance - stood, 0.0).max())
    env = np.minimum(env0 + latch * dt / np.maximum(attack, dt)
                     - (1.0 - latch) * env0 * np.minimum(dt / np.maximum(release, dt), 1.0), 1.0)
    return np.array([latch, env]), -depth * np.array([env, (env - env0) / dt])


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
    heel; its middle's acceleration round that edge, her way's frame (2, 3)). Flat; or, its
    stride `off` it, over its ball as far as her rock has its heel up (`tip`); or as far as
    its leg's length asks, her hip ahead of its ball - the heel's rise that lets the leg,
    `long` m less what its knee's fold shortens it (`short`), reach her hip DIP_M under
    where it stood over a sole. Flat till it left, a sole
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
    return (rocked, flatten, stepping * stance * edge * clip(np.maximum(np.abs(lean), rocked) / EDGE_RAD, 0.0, 1.0),
            (edge * HALF_M)[:, None] * (flatten[:, None] * spoke + (tilting * tilting)[:, None] * inward))


def pitched(ground):
    """The floor's pitch as her soles take it, rad: under half EDGE_RAD level, past it all of
    it - a level floor read 0.003 to 0.006 rad as her soft soles pressed it, and her pelvis
    rolled 4.96 deg where 5.05."""
    return ground * clip((np.abs(ground) - 0.5 * EDGE_RAD) / (0.5 * EDGE_RAD), 0.0, 1.0)


def clears(u, flying, swung, high, tip, tipping, tilt, late, land):
    """A swinging sole's middle over the floor the standing soles touch, m, its rate and its
    acceleration (3, 2): its lowest point `high` clear. Up over the first RISE_U of its swing,
    down over the last FALL_U, held between, on a quintic - its acceleration naught as it leaves
    and as it lands: on a cubic a sole let go was asked 6 high / RISE_U^2, 34 m/s^2 up, in the
    pass its swing began, her hip's torque leapt +24 -> -120 N m and her hip turned 100 -> -188
    deg/s in 40 ms; on a sine's square, 2 cm at its top, it was 9 mm
    up three quarters through and touched the floor there at 0.45 m/s, 12 cm short of where
    it was going. And over what its pitch brings its heel or its toes down by (`flying`, or
    the stride's `off`: the pass her stance let it go on its toes, 7 cm up, its hover asked it
    down 41 m/s^2 for the swing to ask it up 34 the next): its heel by
    its pitch as asked (`tip`, `tipping`) - as it was, a sole let go under 265 N turned up on
    its heel, 12 deg and rising; its toes by its pitch as it is (`tilt`), where further down:
    left with its heel 12 deg up as it was let go, it caught the floor with them half way
    through its swing. Its swing out and nothing under it (`late` s), it reaches on down:
    held at the floor's height, a sole 13 cm ahead of her bore nothing for 0.3 s past its
    swing and she went from 0.7 to 1.05 m/s over the one behind her. Down over the last FALL_U
    to `land` over the floor, a tread's height."""
    on, by = clip(u / RISE_U, 0.0, 1.0), clip((1.0 - u) / FALL_U, 0.0, 1.0)
    eased = np.array([[on * on * on * (10.0 + on * (6.0 * on - 15.0)),
                       by * by * by * (10.0 + by * (6.0 * by - 15.0))],
                      [30.0 * on * on * (1.0 - on) * (1.0 - on) / RISE_U,
                       -30.0 * by * by * (1.0 - by) * (1.0 - by) / FALL_U],
                      [60.0 * on * (1.0 - on) * (1.0 - 2.0 * on) / RISE_U ** 2,
                       60.0 * by * (1.0 - by) * (1.0 - 2.0 * by) / FALL_U ** 2]])
    down = np.maximum(tip, tilt)
    return np.array([
        high * flying * eased[0, 0] * eased[0, 1] - REACH_M_S * late + land * flying * (1.0 - eased[0, 1])
        + flying * (HALF_M * np.maximum(-np.sin(tip), 0.0) + (HALF_M + TOE_M) * np.maximum(np.sin(down), 0.0)),
        high * flying * (eased[1, 0] * eased[0, 1] + eased[0, 0] * eased[1, 1]) * swung - REACH_M_S * (late > 0.0)
        - land * flying * eased[1, 1] * swung
        + flying * (-HALF_M * (tip < 0.0) + (HALF_M + TOE_M) * (tip > 0.0)) * np.cos(tip) * tipping,
        high * flying * (eased[2, 0] * eased[0, 1] + eased[0, 0] * eased[2, 1]) * swung ** 2
        - land * flying * eased[2, 1] * swung ** 2])


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
