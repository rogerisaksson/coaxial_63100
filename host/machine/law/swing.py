"""A sole not standing, on the robot's latent loop: where its modes put it, or at rest on the floor.

    free, frame = swing.free(y, fr, at, said, clear, stance, cruise, grounded, propped)
    flies = swing.flew(thrown, grounded, v, low, shod)
    free, went, strides, level = swing.hung(free, step, strides, flies, (high, low, shod), (kp, kd))
    sole_acc, rests = swing.rested(sole_acc, held, (grounded, aloft, strides), (kp, kd), (floors, y, shod, rot))

Arrays in, arrays out and no branch (tests/test_ortho.py); `machine.law.impedance` asks with them.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.law import poise
from machine.law.modal import AXIAL, COM, DRIVE, G, HOT, LIFT, LOWEST, SOLE, VCOM, VSOLE, WIDTH, Z0
from machine.law.poise import TINY
from machine.law.stride import ALONG, SOON_S, UP
from machine.moves.leap import REST_M_S, TOUCH_M

#: The mode whose impedance holds a sole that does not stand, an axis each.
FREE_BY = np.array([DRIVE, WIDTH, LIFT])
#: The axis a sole in the air is held on against the soles that stand: up. Across as
#: along, from the robot's centre of mass and led by its speed: where its capture point goes.
BETWEEN, HOT3 = np.array([0.0, 0.0, 1.0]), np.eye(3)


def free(y, fr, at, said, clear, stance, cruise, grounded, propped):
    """A sole that does not stand, on the limbs' own impedance: (its acceleration (2, 3) in
    the frame its place is read in, that frame). Up against the soles that do, from the floor
    they touch: what is asked of it - `clear` over it, its rate and its acceleration (3, 2) -
    less what is asked of them; held from their middles, over a sole with its heel 45 deg up
    the other hung 6 cm off the floor and never landed. Along and across against the robot's centre
    of mass as it goes, led along by the robot's speed past its `cruise` over omega: against the
    standing sole it ran ahead of its steps at 0.36 m/s and went over forward. None
    standing, against the robot's centre of mass as it falls, in the robot's way's frame still - plumb under
    it: in its pelvis's own frame, turning with it, a standing hop's flight pitched the
    robot's 10 to 22 deg and swung its soles 0.19 m behind its centre (0.51 m of leg at 22 deg),
    the robot landed past its capture point and stepped 0.35 m (plumb in the world an earlier jump, leaving
    turning forward, gave its trunk all its spin, 0.7 rad in 0.15 s; the robot leaves turning upright now,
    1.7 rad/s); none standing and something else bearing it (`grounded` past them), up from the floor, in
    the robot's way's frame: in its pelvis's on a chair, pitched forward 0.5 rad, its soles rose 9
    cm off the floor and it slid off it. So each sole that does not stand while something else bears the
    robot (`propped`): one let go seated, the other standing, rose where its folded legs put it, 7 mm in 70
    ms, and the robot was thrown off the chair. Up from the floor under it (`modal.under`). `at` the robot (way's
    frame, speed in it, omega, soles' speeds in it, the floor under each sole, its rise along the robot's way);
    `said` its modes' (reference, rate, acceleration, 1/s^2, 1/s, hands)."""
    rot, v, om, shod, floors, rise = at
    ref, rate, acc, kp, kd, hands = said
    stood = stance.max()
    frame = rot
    soles = y[SOLE].reshape(2, 3)
    arm = soles - y[COM]
    speed = (y[VSOLE].reshape(2, 3) - y[VCOM]) @ frame.T
    # a sole in the air is asked no further from the robot than its legs reach standing
    reach = ref - HOT[AXIAL] * np.minimum(ref[AXIAL], 0.0)
    off3 = np.stack([np.einsum('nia,n->ia', hands, reach - fr[Z0]) - arm @ frame.T,
                     np.einsum('nia,n->ia', hands, rate) - speed, np.einsum('nia,n->ia', hands, acc)])
    off3[:, :, UP] += clear
    bears = (stance + TINY) / (stance.sum() + 2.0 * TINY)
    off3 = off3 - stood * BETWEEN * (bears @ off3)[:, None, :]
    aloft = stood * (1.0 - stance)
    along = (soles @ rot.T)[:, ALONG]
    off3[0, :, UP] -= aloft * (bears @ (soles[:, 1] - y[LOWEST]) - rise * (along - bears @ along))
    off3[1, :, UP] -= aloft * (bears @ shod[:, UP] - rise * (shod[:, ALONG] - bears @ shod[:, ALONG]))
    floor = np.stack([clear[0] - (y[LOWEST] - floors), clear[1] - shod[:, UP], clear[2]])
    off3[:, :, UP] += np.maximum(grounded - stood, propped * (1.0 - stance)) * (floor - off3[:, :, UP])
    lead = grounded * HOT3[ALONG] * (v[ALONG] - cruise) / om
    return off3[2] + kp[FREE_BY] * (off3[0] + lead) + kd[FREE_BY] * off3[1], frame


def rested(sole_acc, held, on, gains, at):
    """(each sole's acceleration, how far it rests): a sole that neither stands nor strides while anything
    bears the robot - a seat, its other sole -, its ZMP let go to its text (`held` 0),
    rests where it is, still on the floor under it. `on` (grounded, aloft (2,), strides (2, 1)), `gains` the
    robot's modes' (kp, kd), `at` (the floor under each sole, modal's y, the soles' speeds in the robot's
    way's frame, its way); its step weighs its rows as a pressing contact's point's
    (`wrench.W_STILL`: at W_SWING the robot's seated legs swung out straight). Swung on the robot's modes, a
    seated sole let go kicked its leg out straight, 0.35 m ahead and 1.24 m up, and one the robot's
    seat-off lifted 7 mm swung 0.33 m up and it fell; rested in its own stand, a stumble's step
    was held down and it bobbed 8 s, and on a bump into a bed's edge it fell onto it."""
    (grounded, aloft, strides), (kp, kd), (floors, y, shod, rot) = on, gains, at
    rest = ((1.0 - poise.clip(held, 0.0, 1.0)) * grounded * aloft)[:, None] * (1.0 - strides)
    kept = (kp[FREE_BY] * HOT3[UP] * (floors - y[LOWEST])[:, None] - kd[FREE_BY] * shod) @ rot
    return sole_acc + (rest * (kept - sole_acc.reshape(2, 3))).ravel(), rest[:, 0]


def flew(thrown, grounded, v, low, shod):
    """How far the robot flies, 0 to 1, by what it senses this pass alone: thrown (`thrown`, the robot's
    support asked none), no sole standing (`grounded`), and its centre going up (`v`, its speed in the
    robot's way's frame) or no sole left lying on the floor - within a touch of it (`leap.TOUCH_M`; `low` each
    sole's height over the floor under it) and not coming down on it (`leap.REST_M_S`; `shod` its speed in the
    robot's way's frame). Thrown with none standing alone, the robot's dip's passes flew: its stance sole bore
    under a touch (0-13 N) at 2.922, 2.961 and 2.970 s of a run's leap, still on the floor (within 0.14 mm,
    0.03-0.08 m/s) as it sank 0.57-0.83 m/s; its swing was asked to the floor's speed there, 52.8 m/s^2 back, and
    it bounced off its dip, 0.336 s aloft for 0.24. A sole's height alone closes before it lands: the floor felt
    lies 10-13 mm over where a pitched sole touches, 9 ms of its fall at 1.0-1.5 m/s."""
    lies = (low <= TOUCH_M) * (shod[:, UP] >= -REST_M_S)
    return thrown * (1.0 - grounded) * np.maximum(v[UP] > 0.0, 1.0 - lies.max())


def hung(free, step, strides, flies, heights, gains):
    """The robot's soles aloft, thrown: (`free` with nothing asked of the sole that trails its stride along
    the robot's way and, up, the floor's alone - held `high` over it, a swing's clearance, on the limbs' own
    impedance over the robot's fall, never asked down under it -; the robot's stride's ask (2, 3) in the robot's
    way's frame, the sole that leads it landing at the floor's speed; `strides` (2,), a column; how much of the
    robot's rock's pitch each sole is asked (2,): the one that trails none, it hangs level). `flies` (0 to 1) how
    far the robot flies (`flew`), `step` its stride's (ask along and across (2, 2) (`stride.goes`), each
    sole's aim and its place, the robot's centre's speed, each swing's ease: its way's frame), `heights`
    (that clearance, each sole's height over the floor under it, its speed in the robot's way's frame), `gains` the
    robot's modes' (kp, kd). The sole that leads has as long to its landing as its height has left to fall at the
    robot's centre's speed up: its landing is carried on with the robot that long, a place on the floor, and it is
    damped to the floor, not to the robot, on `stride.goes`' spring of that time to go. Damped to the robot's speed
    toward a landing carried with it it met the floor at 1.5 m/s along (the robot's centre at 1.15 a run's
    leap, 0.54 a walk's) and was stopped by it in 15 ms: the robot's grip read that skid's braking push 0.10-0.14 as
    ice and it slowed 1.16 to 0.29 m/s in 0.5 s. None standing, the robot's soles' asks along its way
    are met by their difference alone: the sole it pushed off asked on to its place under the robot's 21-61
    m/s^2 and down 59, the swing that lands lost 2.23 m/s in 0.2 s and hung 0.10-0.13 m short of its landing; the
    push leg straightened and touched first 0.30 m behind the robot (its walk's hop fell at 5.61 s). Rocked
    as its stride's toe-off has it, its heel 10-29 deg up, its toes hung 14-42 mm under its middle: that rose 44-62
    mm in 0.1 s, they stalled 22-28 mm over the floor."""
    (went, aim, flat, v, ease), (high, low, shod), (kp, kd) = step, heights, gains
    trails, leads = (flies * (1.0 - strides) * strides[::-1])[:, None], (flies * strides)[:, None]
    held = np.maximum(kp[FREE_BY][UP] * (high - low) - kd[FREE_BY][UP] * shod[:, UP] + G, 0.0)
    free = free + trails * (HOT3[UP] * held[:, None] - free)
    left = np.maximum((v[UP] + np.sqrt(v[UP] * v[UP] + 2.0 * G * np.maximum(low, 0.0))) / G, SOON_S)[:, None]
    down = ease[:, None] * (np.minimum(6.0 / left ** 2, kp[FREE_BY][:2]) * (aim + left * v[:2] - flat)
                            - np.minimum(4.0 / left, kd[FREE_BY][:2]) * shod[:, :2])
    return (free, np.concatenate([went + leads * (down - went), ease[:, None] * free[:, UP:]], axis=1),
            strides[:, None], 1.0 - trails[:, 0])
