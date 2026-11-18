"""The robot's turns, its arms and its knees into the loop's ask (wbc_poses.c): `impedance.ask`'s last.

    posed.posed(out, y, fr, a, said, turned, up, folds, still)   # into `a`, `impedance.ask`'s pass
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import libm
from machine.body import gait
from machine.law import latent, poise
from machine.law.modal import (ARM_RAD, ARMS, AXIAL, AXIAL_M, BEND, BEND_RATE, BORNE, CLEAR_M, ELBOWED, ELBOWS,
                               FOLD, FOLD_RATE, FOLDS, HEADED, HIPS_BY, HSPIN, KDOT, KNEE, LEG_M, LIFT, LIST, LIST_RAD,
                               MOMENT, PELVIS, PITCH, PITCH0, PITCH_RAD, REACH, REACHED, ROLLED, ROT, SWUNG, TURN,
                               TURN_RAD, TURNED, WANT)
from machine.law.poise import TINY
from machine.law.stride import ALONG, LEFT, LIMBS
from machine.stack.wbc import MOMENTUM_K, POSTURE_KD, TURN_KD

_TABLE = gait.table('impedance')
#: Up, the way the robot's yaw momentum is bled about.
UPWARD = np.array([0.0, 1.0, 0.0])
#: The posture's knees are bent no more than this ahead of the height, of the axial unit.
AHEAD = _TABLE['ahead']
#: Yaw momentum past its limit is bled this fast, 1/s.
YAW_K = _TABLE['yaw_1_s']
#: The robot's pelvis's axes a seat holds once it bears the robot - its list on its two ischia -, its pitch
#: and turn its own: each let go, findings/spots.md and seat.md.
SEAT_HOLDS = np.array([[0.0], [1.0], [0.0]])


def posed(out, y, fr, a, said, turned, up, folds, still):
    """The robot's turns, its arms and its knees into `a`: its trunk's and its pelvis's pitch,
    each on the pitch's spring, its pelvis's list and its turn on theirs - its trunk upright
    and on its way -, the loop's own turn law asked for what gives that; its arms on the
    posture's own spring, asked ahead of it by what its damper holds back. `said` the modes' (as it
    is, reference, rate, acceleration, 1/s^2, 1/s); `turned` the turns and their
    rates; `up` how far each sole
    is asked up, `folds` which soles' knees are asked a fold."""
    z, ref, rate, acc, kp, kd = said
    turn, spun = turned
    wh = fr[ROT].reshape(3, 3) @ y[HSPIN]
    dwh = np.array([wh[LEFT] - spun[0, 1], wh[ALONG] - spun[1, 1], 0.0])
    dwh4 = libm.sq(dwh * dwh)
    spun = spun + 0.5 * still * (1.0 - PELVIS) * (dwh * dwh4 / (dwh4 + 0.04))[:, None]
    turn_dc = ref[TURN] - out[latent.SWAY][TURN] * libm.sin(out[latent.PSI][TURN])
    aim = np.stack([fr[PITCH0] + PITCH_RAD * ref[PITCH], PELVIS * LIST_RAD * ref[LIST], TURN_RAD * (PELVIS * ref[TURN] + (1.0 - PELVIS) * 1.10 * turn_dc)])
    aim_rate = np.stack([PITCH_RAD * rate[PITCH] + 0.0 * PELVIS, PELVIS * LIST_RAD * rate[LIST], PELVIS * TURN_RAD * rate[TURN]])
    aim_acc = np.stack([PITCH_RAD * acc[PITCH] + 0.0 * PELVIS, PELVIS * LIST_RAD * acc[LIST], PELVIS * TURN_RAD * acc[TURN]])
    by = np.array([PITCH, LIST, TURN])
    # borne, the robot's ZMP let go to it, its pelvis's turns have no spring of their own - what bears the
    # robot holds it -, its rates its trunk's: sprung to its standing pitch on a seat, or on its trunk's
    # error, it snapped 0.33 rad on in 0.11 s as its trunk came upright
    sat = a[BORNE] * (1.0 - poise.clip(out[latent.HELD], 0.0, 1.0))
    turning = aim_acc + kp[by][:, None] * (aim - turn) * (1.0 - sat * PELVIS * SEAT_HOLDS) + kd[by][:, None] * (aim_rate - spun)
    a[WANT], a[ROLLED], a[HEADED] = turn + (turning + TURN_KD[1] * spun) / TURN_KD[0]
    a[TURNED] = 1.0 * (kp[PITCH] + kd[PITCH] > 0.0)
    a[SWUNG] = ARM_RAD * (ref[ARMS] + (acc[ARMS] + POSTURE_KD[1] * rate[ARMS]) / POSTURE_KD[0])
    a[REACHED] = ARM_RAD * (ref[REACH] + (acc[REACH] + POSTURE_KD[1] * rate[REACH]) / POSTURE_KD[0])
    a[ELBOWED] = ARM_RAD * (ref[ELBOWS] + (acc[ELBOWS] + POSTURE_KD[1] * rate[ELBOWS]) / POSTURE_KD[0])
    # the robot's posture's knees bent as its height is, AHEAD of it toward what is asked: bent as asked, a
    # throw's step threw its knees straight at 100 rad/s^2 and it over forward, 8 kg m^2/s about it; bent as
    # it is, a crouch asked 0.6 stopped at 0.56
    bent = z[AXIAL] + poise.clip(ref[AXIAL] - z[AXIAL], -AHEAD, AHEAD)
    cosine = libm.cos(0.5 * fr[KNEE]) - AXIAL_M * bent * fr[HIPS_BY] / LEG_M
    a[BEND] = 2.0 * libm.arccos(poise.clip(cosine, -1.0, 1.0)) - fr[KNEE]
    # and at the told height's rate where the rows let the robot's soles go (a throw's ZMP NONE): damped
    # against still, the stack's knee rows held a push's knees at 0.2 of their clamp, its height at its band's
    # floor, 3 m/s^2 under its ask; at the told rate under a held polygon the seats' texts, laid against that
    # damping, sat the robot onto the bed at 1.86 of its weight and it fell, or had it bear 0.00 of it; at its
    # own rate past AHEAD, a stride's instant throw left its knees undamped and the leap flew 0.20 s for 0.23;
    # at its own where it outran the told height's rate toward that height, 11 of the walk's leap's 27 starts
    # fell at the stop that followed (the told rate's: 1)
    a[BEND_RATE] = (2.0 * AXIAL_M * fr[HIPS_BY] * rate[AXIAL] * float(abs(cosine) < 1.0) / LEG_M
                    / np.sqrt(np.maximum(1.0 - cosine * cosine, TINY))) * (1.0 - poise.clip(out[latent.HELD], 0.0, 1.0))
    # a sole asked up where the robot stands: its knee folded as its leg is shorter by the lift
    # under its hip. Striding, a limb's knee is its sole's place's alone
    short = poise.clip(libm.cos(0.5 * (fr[KNEE] + a[BEND])) - CLEAR_M * np.maximum(up, 0.0) / LEG_M, -1.0, 1.0)
    a[FOLD] = 2.0 * libm.arccos(short) - fr[KNEE] - a[BEND]
    a[FOLD_RATE] = (2.0 * CLEAR_M * LIMBS * rate[LIFT] * (up > 0.0) / LEG_M
                    / np.sqrt(np.maximum(1.0 - short * short, TINY)))
    a[FOLDS] = folds
    k = y[MOMENT]
    a[KDOT] = -MOMENTUM_K * k - UPWARD * YAW_K * (k[1] - poise.clip(k[1], -out[latent.YAWED], out[latent.YAWED]))
