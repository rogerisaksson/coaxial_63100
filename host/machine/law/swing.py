"""A sole that does not stand on her latent loop: where her modes put it, or at rest on the floor.

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
#: along, from her centre of mass and led by her speed: where her capture point goes.
BETWEEN, HOT3 = np.array([0.0, 0.0, 1.0]), np.eye(3)


def free(y, fr, at, said, clear, stance, cruise, grounded, propped):
    """A sole that does not stand, on the limbs' own impedance: (its acceleration (2, 3) in
    the frame its place is read in, that frame). Up against the soles that do, from the floor
    they touch: what is asked of it - `clear` over it, its rate and its acceleration (3, 2) -
    less what is asked of them; held from their middles, over a sole with its heel 45 deg up
    the other hung 6 cm off the floor and never landed. Along and across against her centre
    of mass as it goes, led along by her speed past her `cruise` over omega: against the
    standing sole she ran ahead of her steps at 0.36 m/s and went over forward. None
    standing, against her centre of mass as it falls, in her way's frame still - plumb under
    her: in her pelvis's own frame, turning with her, a standing hop's flight pitched her 10
    to 22 deg and swung her soles 0.19 m behind her centre (0.51 m of leg at 22 deg), she
    landed past her capture point and stepped 0.35 m (plumb in the world an earlier jump,
    leaving turning forward, gave her trunk all its spin, 0.7 rad in 0.15 s; she leaves
    turning upright now, 1.7 rad/s); none standing and something else bearing her
    (`grounded` past them), up from the floor, in her way's frame: in her
    pelvis's on a chair, pitched forward 0.5 rad, her soles rose 9 cm off the floor and she
    slid off it. So each sole that does not stand while something else bears her (`propped`):
    one let go seated, the other standing, rose where her folded legs put it, 7 mm in 70 ms,
    and she was thrown off the chair. Up from the floor under it (`modal.under`). `at` her (way's frame, speed in it,
    omega, soles' speeds in it, the floor under each sole, its rise along her way); `said` her
    modes' (reference, rate, acceleration, 1/s^2, 1/s, hands)."""
    rot, v, om, shod, floors, rise = at
    ref, rate, acc, kp, kd, hands = said
    stood = stance.max()
    frame = rot
    soles = y[SOLE].reshape(2, 3)
    arm = soles - y[COM]
    speed = (y[VSOLE].reshape(2, 3) - y[VCOM]) @ frame.T
    # a sole in the air is asked no further from her than her legs reach standing
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
    """(each sole's acceleration, how far it rests): a sole that neither stands nor strides while
    anything bears her - a seat, her other sole -, her ZMP let go to her text (`held` 0), rests
    where it is, still on the floor under it. `on` (grounded, aloft (2,), strides (2, 1)), `gains`
    her modes' (kp, kd), `at` (the floor under each sole, modal's y, the soles' speeds in her way's
    frame, her way); her step weighs its rows as a pressing contact's point's (`wrench.W_STILL`: at
    W_SWING her seated legs swung out straight). Swung on her modes, a seated sole let go kicked her
    leg out straight, 0.35 m ahead and 1.24 m up, and one her seat-off lifted 7 mm swung 0.33 m up and she fell; rested in
    her own stand, a stumble's step was held down and she bobbed 8 s, and on a bump into a bed's
    edge she fell onto it."""
    (grounded, aloft, strides), (kp, kd), (floors, y, shod, rot) = on, gains, at
    rest = ((1.0 - poise.clip(held, 0.0, 1.0)) * grounded * aloft)[:, None] * (1.0 - strides)
    kept = (kp[FREE_BY] * HOT3[UP] * (floors - y[LOWEST])[:, None] - kd[FREE_BY] * shod) @ rot
    return sole_acc + (rest * (kept - sole_acc.reshape(2, 3))).ravel(), rest[:, 0]


def flew(thrown, grounded, v, low, shod):
    """How far she flies, 0 to 1, by what she senses this pass alone: thrown (`thrown`, her support asked none), no
    sole standing (`grounded`), and her centre going up (`v`, its speed in her way's frame) or no sole left lying
    on the floor - within a touch of it (`leap.TOUCH_M`; `low` each sole's height over the floor under it) and not
    coming down on it (`leap.REST_M_S`; `shod` its speed in her way's frame). Thrown with none standing alone, her
    dip's passes flew: her stance sole bore under a touch (0-13 N) at 2.922, 2.961 and 2.970 s of a run's leap,
    still on the floor (within 0.14 mm, 0.03-0.08 m/s) as she sank 0.57-0.83 m/s; her swing was asked to the
    floor's speed there, 52.8 m/s^2 back, and she bounced off her dip, 0.336 s aloft for 0.24. A sole's height
    alone closes before it lands: the floor felt lies 10-13 mm over where a pitched sole touches, 9 ms of its fall
    at 1.0-1.5 m/s."""
    lies = (low <= TOUCH_M) * (shod[:, UP] >= -REST_M_S)
    return thrown * (1.0 - grounded) * np.maximum(v[UP] > 0.0, 1.0 - lies.max())


def hung(free, step, strides, flies, heights, gains):
    """Her soles aloft, thrown: (`free` with nothing asked of the sole that trails her stride along her way and, up,
    the floor's alone - held `high` over it, a swing's clearance, on the limbs' own impedance over her fall, never
    asked down under it -; her stride's ask (2, 3) in her way's frame, the sole that leads it landing at the
    floor's speed; `strides` (2,), a column; how much of her rock's pitch each sole is asked (2,): the one that
    trails none, it hangs level). `flies` (0 to 1) how far she flies (`flew`), `step` her stride's (ask along and
    across (2, 2) (`stride.goes`), each sole's aim and its place, her centre's speed, each swing's ease: her way's
    frame), `heights` (that clearance, each sole's height over the floor under it, its speed in her way's frame),
    `gains` her modes' (kp, kd). The sole that leads has as long to its landing as its height has left to fall at
    her centre's speed up: its landing is carried on with her that long, a place on the floor, and it is damped to
    the floor, not to her, on `stride.goes`' spring of that time to go. Damped to her speed toward a landing
    carried with her it met the floor at 1.5 m/s along (her centre at 1.15 a run's leap, 0.54 a walk's) and was
    stopped by it in 15 ms: her grip read that skid's braking push 0.10-0.14 as ice and she slowed 1.16 to
    0.29 m/s in 0.5 s. None standing, her soles' asks along her way are met by their difference alone: the sole
    she pushed off asked on to its place under her 21-61 m/s^2 and down 59, the swing that lands lost 2.23 m/s in
    0.2 s and hung 0.10-0.13 m short of its landing; the push leg straightened and touched first 0.30 m behind her
    (her walk's hop fell at 5.61 s). Rocked as its stride's toe-off has it, its heel 10-29 deg up, its toes hung
    14-42 mm under its middle: that rose 44-62 mm in 0.1 s, they stalled 22-28 mm over the floor."""
    (went, aim, flat, v, ease), (high, low, shod), (kp, kd) = step, heights, gains
    trails, leads = (flies * (1.0 - strides) * strides[::-1])[:, None], (flies * strides)[:, None]
    held = np.maximum(kp[FREE_BY][UP] * (high - low) - kd[FREE_BY][UP] * shod[:, UP] + G, 0.0)
    free = free + trails * (HOT3[UP] * held[:, None] - free)
    left = np.maximum((v[UP] + np.sqrt(v[UP] * v[UP] + 2.0 * G * np.maximum(low, 0.0))) / G, SOON_S)[:, None]
    down = ease[:, None] * (np.minimum(6.0 / left ** 2, kp[FREE_BY][:2]) * (aim + left * v[:2] - flat)
                            - np.minimum(4.0 / left, kd[FREE_BY][:2]) * shod[:, :2])
    return (free, np.concatenate([went + leads * (down - went), ease[:, None] * free[:, UP:]], axis=1),
            strides[:, None], 1.0 - trails[:, 0])
