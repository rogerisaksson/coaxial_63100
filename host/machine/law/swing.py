"""A sole that does not stand on her latent loop: where her modes put it, or at rest on the floor.

    free, frame = swing.free(y, fr, at, said, clear, stance, cruise, grounded, propped)
    sole_acc, rests = swing.rested(sole_acc, held, (grounded, aloft, strides), (kp, kd), (floors, y, shod, rot))

Arrays in, arrays out and no branch (tests/test_ortho.py); `machine.law.impedance` asks with them.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.law import poise
from machine.law.modal import AXIAL, COM, DRIVE, HOT, LIFT, LOWEST, QUAT, SOLE, SPIN, VCOM, VSOLE, WIDTH, Z0
from machine.law.poise import TINY
from machine.law.stride import ALONG, UP

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
    standing, against her centre of mass as it falls, in her pelvis's own frame, turning with
    her: held level and plumb under her in the world, her legs kept none of her spin and her
    trunk took it all - 0.7 rad forward in 0.15 s off the floor; none standing and something
    else bearing her (`grounded` past them), up from the floor, in her way's frame: in her
    pelvis's on a chair, pitched forward 0.5 rad, her soles rose 9 cm off the floor and she
    slid off it. So each sole that does not stand while something else bears her (`propped`):
    one let go seated, the other standing, rose where her folded legs put it, 7 mm in 70 ms,
    and she was thrown off the chair. Up from the floor under it (`modal.under`). `at` her (way's frame, speed in it,
    omega, soles' speeds in it, the floor under each sole, its rise along her way); `said` her
    modes' (reference, rate, acceleration, 1/s^2, 1/s, hands)."""
    rot, v, om, shod, floors, rise = at
    ref, rate, acc, kp, kd, hands = said
    stood = stance.max()
    q = y[QUAT][:4]
    hers = np.array([[2.0 * (q[1] * q[3] + q[0] * q[2]), 2.0 * (q[2] * q[3] - q[0] * q[1]),
                      1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2)],
                     [1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2), 2.0 * (q[1] * q[2] + q[0] * q[3]),
                      2.0 * (q[1] * q[3] - q[0] * q[2])],
                     [2.0 * (q[1] * q[2] - q[0] * q[3]), 1.0 - 2.0 * (q[1] ** 2 + q[3] ** 2),
                      2.0 * (q[2] * q[3] + q[0] * q[1])]])
    frame = grounded * rot + (1.0 - grounded) * hers
    soles = y[SOLE].reshape(2, 3)
    arm = soles - y[COM]
    speed = (y[VSOLE].reshape(2, 3) - y[VCOM] - (1.0 - grounded) * np.cross(y[SPIN][:3], arm)) @ frame.T
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
