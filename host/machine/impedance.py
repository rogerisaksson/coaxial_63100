"""Her modes' springs on her body: `latent.step`'s out rendered, the whole-body loop's ask.

    a = impedance.ask(out, y, fr, a)        # flat, `machine.modal`'s layouts; a pass

Arrays in, arrays out and no branch (tests/test_ortho.py). Each mode is a mass on a spring and a
damper to its reference: the sum's on her whole mass, the difference's on a limb's, a turn's on
her trunk's. What the sum's springs ask is her centre of mass's acceleration, its ZMP in her
soles (`machine.poise`), up no harder than her legs throw her. A sole that does not stand is
sprung to the place her modes give it from the soles that do - none standing, from her centre
of mass as it falls -, each axis on the difference's impedance: the drive's along, the width's
across, the lift's up; striding, it is her stride's (`machine.stride`). The loop finds the
torques and clips them to her drives (`machine.wbc`, `machine.wrench`).
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np  # noqa: E402

from machine import latent, poise, stride  # noqa: E402
from machine.modal import (  # noqa: E402
    ARM_RAD, ARMS, AXIAL, AXIAL_M, BEND, CLEAR_M, COM, COM_ACC, DRIVE, DT, EDGE, FLOOR, FOLD, FOLD_RATE, FOLDS, G,
    GOING, HANDS, HEADED, HEELED, HIP, HIPS_BY, HOT, KDOT, KNEE, LATERAL, LEG_M, LIFT, LIST, LIST_RAD, LOAD, LOWEST,
    MASS, MOMENT, PELVIS, PITCH, PITCH0, PITCH_RAD, PREFOLD, QUAT, REACH0, ROCK, ROLLED, ROT, SAGITTAL, SETTLE,
    SOLE,
    SOLE_ACC, SPIN, STANCE, STRIDES, SWUNG, TILT, TILT_RATE, TIPPED, TURN, TURN_RAD, TURNED, UNIT, VCOM, VSOLE,
    W0, WANT, WIDTH, Z0, reading, shortened)
from machine.poise import HALF_M, TINY  # noqa: E402
from machine.stride import ALONG, LEFT, LIMBS, UP  # noqa: E402
from machine.wbc import MOMENTUM_K, POSTURE_KD, TURN_KD  # noqa: E402

#: The mode whose impedance holds a sole that does not stand, an axis each.
FREE_BY = np.array([DRIVE, WIDTH, LIFT])
#: A sole asked up leaves only while she sways as asked, within ON of her lateral shift and
#: of its rate: a sway asked past what her soles' width gives (0.35 at 1.1 Hz: her ZMP 1.9
#: of the way to a sole) crossed the other's box off its beat, a sole left and she went over
#: its side in 3 s.
ON = (0.1, 1.0)
#: Her going: her soles' speed along her way, meaned over GOING_S.
GOING_S = 0.28
#: Her posture's knees are bent no more than this ahead of her height, of the axial unit.
AHEAD = 0.15
#: Her yaw momentum past its limit is bled this fast, 1/s.
YAW_K = 50.0
#: Her centre of mass is asked up no harder than this, m/s^2 (`tools/sim/hop.py`: thrown at
#: 10 to 18 she flew and stood): the stiffest spring said asked 630, the loop gave 33 with
#: every drive it has, and her trunk went back 0.22 rad in the 45 ms of it.
THROW_M_S2 = 14.0
#: Her height no lower than this for omega; a hip no lower over its ankle than this of its leg.
LOW_M, LOW_LEG = 0.3, 0.5
FALLS, UPWARD = np.array([0.0, 0.0, -G]), np.array([0.0, 1.0, 0.0])
#: The axis a sole in the air is held on against the soles that stand: up. Across as
#: along, from her centre of mass and led by her speed: where her capture point goes.
BETWEEN, HOT3 = np.array([0.0, 0.0, 1.0]), np.eye(3)


def ask(out, y, fr, a):
    """`a`, the loop's ask this pass: `out` (`latent.step`) rendered on her as she is sensed."""
    rot = fr[ROT].reshape(3, 3)
    stood = a[STANCE].copy()
    z, zd, legs, turn, spun = reading(y, fr, stood)
    turns = out[latent.TURNS]
    c, v = rot @ y[COM], rot @ y[VCOM]
    om = np.sqrt(G / np.maximum(c[UP] - fr[FLOOR], LOW_M))
    soles, shod = y[SOLE].reshape(2, 3), y[VSOLE].reshape(2, 3) @ rot.T
    tilt, tilting = y[TILT], y[TILT_RATE]
    stepping, way, sweep, phi, off, u, flying, swung = stride.beat(
        out[latent.SWAY][DRIVE], out[latent.PSI][DRIVE], out[latent.SWAY][LATERAL], out[latent.PSI][LATERAL], turns)
    # the lift's own rhythm is her stride's clearance, not a sole's place
    beat = out[latent.PSI][LIFT]
    tone = out[latent.SWAY][LIFT] * np.array([np.sin(beat), turns * np.cos(beat), -turns * turns * np.sin(beat)])
    ref, rate, acc = (out[part] - HOT[LIFT] * t for part, t in zip((latent.REF, latent.RATE, latent.ACC), tone))
    kp, kd = out[latent.KS] / fr[MASS], out[latent.DS] / fr[MASS]
    # along her way her centre of mass is damped to her cruise, not to her soles' middle:
    # damped to that she stopped between steps and each sole waited for her
    cruise = stride.cruises(way, out[latent.SWAY][DRIVE], turns, a[GOING])
    zd = zd + HOT[SAGITTAL] * ((v[ALONG] - cruise) / HALF_M - zd[SAGITTAL])
    zdd = acc + kp * (ref - z) + kd * (rate - zd)
    # her limbs' places as her modes ask them; their sum's acceleration is her centre of
    # mass's, the other way; its ZMP into her soles, and which of them stand
    hands = HANDS * fr[UNIT][:, None, None]
    both = np.einsum('nia,n->ia', hands, zdd).mean(axis=0)
    flat, xi = (soles @ rot.T)[:, :2], c[:2] + v[:2] / om
    low, high = poise.boxes(flat, a[EDGE], tilt)
    swaying = (np.abs(ref[LATERAL] - z[LATERAL]) < ON[0]) & (np.abs(rate[LATERAL] - zd[LATERAL]) < ON[1])
    up = LIMBS * ref[LIFT]
    stance, zmp = poise.stands(c[:2] + both[:2] / om ** 2, xi, low, high, om, out[latent.HELD], up, swaying,
                               (sweep, off, flying), (y[LOAD], y[LOWEST] - fr[FLOOR], y[VCOM][1]), stood)
    grounded, aloft = stance.max(), 1.0 - stance
    bears = (stance + TINY) / (stance.sum() + 2.0 * TINY)
    along = om ** 2 * (c[:2] - zmp)
    # striding: where a sole lands, how it is pitched, how a standing one is turned
    meet = stride.meets(xi, c, flat, cruise, om, turns, 2.0 * fr[W0] * (1.0 + ref[WIDTH]))
    tip, tipping, tipped = stride.rocks(ref[ROCK], rate[ROCK], acc[ROCK])
    long = fr[REACH0] - AXIAL_M * fr[HIPS_BY] * np.maximum(ref[AXIAL], 0.0)
    a[PREFOLD], short = stride.folds(off, y[LOAD], fr[KNEE] + a[BEND])
    a[HEELED], flatten, a[EDGE], rounds = stride.heels(
        (soles @ rot.T, tilt, tilting), (y[HIP].reshape(2, 3) @ rot.T)[:, ALONG], fr[FLOOR], long, short, off,
        tip, stance, stepping, a[HEELED], fr[DT])
    a[SETTLE] = np.concatenate([flatten[:, None] * rot[LEFT], rounds @ rot], axis=1).ravel()
    # up as her height's spring asks of her standing legs' length: each leg's acceleration
    # along itself, what her going over it gives it taken off, by how far over its ankle
    # its hip is - her centre of mass less than her hips
    axial = shortened(legs, fr, stance)
    z += HOT[AXIAL] * (axial[0] - z[AXIAL])
    zd += HOT[AXIAL] * (axial[1] - zd[AXIAL])
    pressed = acc[AXIAL] + kp[AXIAL] * (ref[AXIAL] - z[AXIAL]) + kd[AXIAL] * (rate[AXIAL] - zd[AXIAL])
    risen = ((legs[2] ** 2 - ((v - shod) ** 2).sum(axis=1) - legs[0][:, :2] @ along
              - legs[1] * AXIAL_M * fr[HIPS_BY] * pressed) / np.maximum(legs[0][:, UP], LOW_LEG * fr[REACH0]))
    com = (grounded * np.array([along[0], along[1], poise.clip(bears @ risen / fr[HIPS_BY], -G, THROW_M_S2)])
           + (1.0 - grounded) * FALLS)
    late = stride.lates(phi, turns, stepping)
    clear = aloft * stride.clears(u, flying, swung, CLEAR_M * (1.0 + np.abs(out[latent.SWAY][LIFT])), tip,
                                  tipping, tilt, late)
    free, frame = _swings(y, fr, (rot, v, om, shod), (ref, rate, acc, kp, kd, hands), clear, stance, cruise)
    strides = stepping * grounded
    a[SOLE_ACC] = ((1.0 - strides) * (free @ frame + ((1.0 - grounded * BETWEEN) * com) @ rot) + strides * (
        np.concatenate([stride.goes(meet, flat, shod, v, u, turns, kp[FREE_BY][:2], kd[FREE_BY][:2]),
                        free[:, UP:]], axis=1) @ rot)).ravel()
    # its pitch on the rock's own spring; the loop's turn law asked for what gives that
    a[TIPPED] = tilt + (aloft * (tipped + kp[ROCK] * (tip - tilt) + kd[ROCK] * (tipping - tilting))
                        + TURN_KD[1] * tilting) / TURN_KD[0]
    a[GOING] += fr[DT] / GOING_S * ((y[VSOLE].reshape(2, 3) @ rot[ALONG]).mean() - a[GOING])
    a[STANCE], a[COM_ACC], a[STRIDES] = stance, com @ rot, stepping
    _poses(out, y, fr, a, (z, ref, rate, acc, kp, kd), (turn, spun), up, aloft * (1.0 - stepping))
    return a


def _swings(y, fr, at, said, clear, stance, cruise):
    """A sole that does not stand, on the limbs' own impedance: (its acceleration (2, 3) in
    the frame its place is read in, that frame). Up against the soles that do, from the floor
    they touch: what is asked of it - `clear` over it, its rate and its acceleration (3, 2) -
    less what is asked of them; held from their middles, over a sole with its heel 45 deg up
    the other hung 6 cm off the floor and never landed. Along and across against her centre
    of mass as it goes, led along by her speed past her `cruise` over omega: against the
    standing sole she ran ahead of her steps at 0.36 m/s and went over forward. None
    standing, against her centre of mass as it falls, in her pelvis's own frame, turning with
    her: held level and plumb under her in the world, her legs kept none of her spin and her
    trunk took it all - 0.7 rad forward in 0.15 s off the floor. `at` her (way's frame, speed
    in it, omega, soles' speeds in it); `said` her modes' (reference, rate, acceleration,
    1/s^2, 1/s, hands)."""
    rot, v, om, shod = at
    ref, rate, acc, kp, kd, hands = said
    grounded = stance.max()
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
    off3 = off3 - grounded * BETWEEN * (bears @ off3)[:, None, :]
    aloft = grounded * (1.0 - stance)
    off3[0, :, UP] -= aloft * (bears @ (soles[:, 1] - y[LOWEST]))
    off3[1, :, UP] -= aloft * (bears @ shod[:, UP])
    lead = grounded * HOT3[ALONG] * (v[ALONG] - cruise) / om
    return off3[2] + kp[FREE_BY] * (off3[0] + lead) + kd[FREE_BY] * off3[1], frame


def _poses(out, y, fr, a, said, turned, up, folds):
    """Her turns, her arms and her knees into `a`. Her trunk's and her pelvis's pitch, each on
    the pitch's spring, her pelvis's list and its turn on theirs - her trunk upright and on
    her way -, the loop's own turn law asked for what gives that; her arms on the posture's
    own spring, asked ahead of it by what its damper holds back. `said` her modes' (as she
    is, reference, rate, acceleration, 1/s^2, 1/s); `turned` her turns and their rates; `up`
    how far each sole is asked up, `folds` which soles' knees are asked a fold."""
    z, ref, rate, acc, kp, kd = said
    turn, spun = turned
    aim = np.stack([fr[PITCH0] + PITCH_RAD * ref[PITCH], PELVIS * LIST_RAD * ref[LIST],
                    PELVIS * TURN_RAD * ref[TURN]])
    aim_rate = np.stack([PITCH_RAD * rate[PITCH] + 0.0 * PELVIS, PELVIS * LIST_RAD * rate[LIST],
                         PELVIS * TURN_RAD * rate[TURN]])
    aim_acc = np.stack([PITCH_RAD * acc[PITCH] + 0.0 * PELVIS, PELVIS * LIST_RAD * acc[LIST],
                        PELVIS * TURN_RAD * acc[TURN]])
    by = np.array([PITCH, LIST, TURN])
    turning = aim_acc + kp[by][:, None] * (aim - turn) + kd[by][:, None] * (aim_rate - spun)
    a[WANT], a[ROLLED], a[HEADED] = turn + (turning + TURN_KD[1] * spun) / TURN_KD[0]
    a[TURNED] = a[STANCE].max() * (kp[PITCH] + kd[PITCH] > 0.0)
    a[SWUNG] = ARM_RAD * (ref[ARMS] + (acc[ARMS] + POSTURE_KD[1] * rate[ARMS]) / POSTURE_KD[0])
    # her posture's knees bent as her height is, AHEAD of it toward what is asked: bent as
    # asked, a throw's step threw her knees straight at 100 rad/s^2 and her over forward,
    # 8 kg m^2/s about her; bent as she is, a crouch asked 0.6 stopped at 0.56
    bent = z[AXIAL] + poise.clip(ref[AXIAL] - z[AXIAL], -AHEAD, AHEAD)
    a[BEND] = 2.0 * np.arccos(poise.clip(np.cos(0.5 * fr[KNEE]) - AXIAL_M * bent * fr[HIPS_BY] / LEG_M,
                                         -1.0, 1.0)) - fr[KNEE]
    # a sole asked up where she stands: its knee folded as its leg is shorter by the lift
    # under its hip. Striding, a limb's knee is its sole's place's alone
    short = poise.clip(np.cos(0.5 * (fr[KNEE] + a[BEND])) - CLEAR_M * np.maximum(up, 0.0) / LEG_M, -1.0, 1.0)
    a[FOLD] = 2.0 * np.arccos(short) - fr[KNEE] - a[BEND]
    a[FOLD_RATE] = (2.0 * CLEAR_M * LIMBS * rate[LIFT] * (up > 0.0) / LEG_M
                    / np.sqrt(np.maximum(1.0 - short * short, TINY)))
    a[FOLDS] = folds
    k = y[MOMENT]
    a[KDOT] = -MOMENTUM_K * k - UPWARD * YAW_K * (k[1] - poise.clip(k[1], -out[latent.YAWED], out[latent.YAWED]))
