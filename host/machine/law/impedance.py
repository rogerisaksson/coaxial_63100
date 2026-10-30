"""Her modes' springs on her body: `latent.step`'s out rendered, the whole-body loop's ask.

    a = impedance.ask(out, y, fr, a)        # flat, `machine.law.modal`'s layouts; a pass

Arrays in, arrays out and no branch (tests/test_ortho.py). Each mode is a mass on a spring and a
damper to its reference: the sum's on her whole mass, the difference's on a limb's, a turn's on
her trunk's. What the sum's springs ask is her centre of mass's acceleration, its ZMP in her
soles (`machine.law.poise`), up no harder than her legs throw her. A sole that does not stand is
sprung to the place her modes give it from the soles that do - none standing, from her centre
of mass as it falls -, each axis on the difference's impedance: the drive's along, the width's
across, the lift's up; striding, it is her stride's (`machine.law.stride`). The loop finds the
torques and clips them to her drives (`machine.stack.wbc`, `machine.stack.wrench`).
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.senses import borne
from machine.law import latent, poise, stride, swing
from machine.moves import leap
from machine.law.modal import ANKLE_H, ARM_RAD, ARMS, AXIAL, AXIAL_M, BEND, BEND_RATE, BORNE, CELL, CLEAR_M, COM, COM_ACC, DRIVE, DT, DUCKING, EDGE, ELBOWED, ELBOWS, FLOOR, FOLD, FOLD_RATE, FOLDS, G, GOING, GRIP, GROUND, HANDS, HEADED, HEELED, HIP, HIPS_BY, HOT, HSPIN, KDOT, KNEE, LATERAL, LEAD, LEG_M, LET_GO, LIFT, LIST, LIST_RAD, LOAD, LOWEST, MAP_ALONG, MAP_AT, MAP_N, MAP_RISE, MAP_WAY, MASS, MOMENT, NEED, OUTSIDE, PELVIS, PITCH, PITCH0, PITCH_RAD, PREFOLD, PROP, RAISE, REACH, REACH0, REACHED, RESTS, ROCK, ROLLED, ROT, SAGITTAL, SETTLE, SOLE, SOLE_ACC, SPLIT, STANCE, STRIDES, SWINGS, SWUNG, TILT, TILT_RATE, TIPPED, TURN, TURN_RAD, TURNED, UNIT, VCOM, VHEAD, VSOLE, W0, WANT, WEIGHT, WIDTH, reading, shortened, under
from machine.law.poise import HALF_M, TINY
from machine.law.stride import ALONG, LEFT, LIMBS, UP
from machine.law.swing import BETWEEN, FREE_BY, HOT3
from machine.stack.wbc import MOMENTUM_K, POSTURE_KD, TURN_KD

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


#: Borne off her soles this share of her weight, she is supported there wholly: her support
#: region spans where, her centre of mass asked of her springs, not let fall.
PROP_ON = 0.15
#: Her pelvis's axes a seat holds once it bears her - its list on her two ischia -, its pitch and turn
#: hers: let go with the list, sat 3 deg askew she turned 0.05 rad in 0.1 s and lifted a sole; its
#: pitch let go, it stayed 29 deg forward of her settled trunk, her spine at -25 deg.
SEAT_HOLDS = np.array([[0.0], [1.0], [0.0]])


def ask(out, y, fr, a):
    """`a`, the loop's ask this pass: `out` (`latent.step`) rendered on her as she is sensed."""
    rot = fr[ROT].reshape(3, 3)
    stood = a[STANCE].copy()
    z, zd, legs, turn, spun = reading(y, fr, stood)
    turns, beat = out[latent.TURNS], out[latent.PSI][LIFT]
    c, v, vh = rot @ y[COM], rot @ y[VCOM], rot @ y[VHEAD]
    om = np.sqrt(G / np.maximum(c[UP] - fr[FLOOR], LOW_M))
    soles, shod = y[SOLE].reshape(2, 3), y[VSOLE].reshape(2, 3) @ rot.T
    tilt, tilting = y[TILT], y[TILT_RATE]
    flat, xi = (soles @ rot.T)[:, :2], c[:2] + v[:2] / om
    low, high = poise.boxes(flat, a[EDGE], tilt)
    floors, rise = under(y, fr, stood)
    propped, prop = borne.latched(y, rot, a, fr[DT])
    # the lift's own rhythm is her stride's clearance, not a sole's place
    tone = out[latent.SWAY][LIFT] * np.array([np.sin(beat), turns * np.cos(beat), -turns * turns * np.sin(beat)])
    ref, rate, acc = (out[part] - HOT[LIFT] * t for part, t in zip((latent.REF, latent.RATE, latent.ACC), tone))
    up = LIMBS * ref[LIFT]
    # her capture point past the soles that stood - past her ZMP's margin in them - steps her
    # while the rows hold the polygon: let go, a push stepped a sole at its end (0/497 N)
    still = 1.0 - poise.clip(np.linalg.norm(out[latent.SWAY]) / TINY, 0.0, 1.0)
    out_of = poise.BIG * (1.0 - stood)[:, None]
    lifted = out_of * np.maximum(1.0 - still, np.maximum(a[SWINGS], up > poise.LIFT_EPS))[:, None]
    lo, hi = poise.boxes(flat, 0.0 * a[EDGE], tilt)
    beyond = xi - poise.clip(xi, *poise.spanned((lo + out_of).min(axis=0), (hi - out_of).max(axis=0), prop))
    past = np.linalg.norm(xi - poise.clip(xi, *poise.spanned((lo + lifted).min(axis=0), (hi - lifted).max(axis=0), prop)))
    a[NEED] = (1.0 - propped) * stood.max() * poise.clip(past / poise.MARGIN_M, 0.0, 1.0) * poise.clip(out[latent.HELD], 0.0, 1.0)
    stepping, way, sweep, phi, on, off, u, flying, swung = stride.beat(
        out[latent.SWAY][DRIVE], out[latent.PSI][DRIVE], out[latent.SWAY][LATERAL], out[latent.PSI][LATERAL], turns,
        a[SWINGS], a[NEED], a[LEAD], a[SPLIT])
    a[SWINGS], a[SPLIT] = off, stride.split(a[SPLIT], out[latent.HELD], stepping, y[LOAD] < leap.TOUCH_N, way * (flat[:, ALONG] - c[ALONG]), phi, u, turns, fr[DT])
    kp, kd = out[latent.KS] / fr[MASS], out[latent.DS] / fr[MASS]
    # a swing goes ahead of its rhythm as far as lands it before her capture point is a reach
    # across past the soles that stood, its own side of them - along her way too while her rhythm
    # is still, no stride pacing her steps: landed from the run's held leap her capture point ran
    # on at 1.6 m/s, a swing at the still rhythm's pace took 0.40 s and she went over a sole's ball
    aside = stood.max() * np.maximum(np.maximum(LIMBS * beyond[LEFT], 0.0), still * np.abs(beyond[ALONG]))
    # past a lone standing sole's own side no swing reaches: never in her plain walk
    a[OUTSIDE] = (stood * (stood.sum() < 1.5)) @ np.maximum(LIMBS * beyond[LEFT], 0.0)
    hurry = 1.0 + flying * (stride.hurried(aside, om, u, swung, kp[FREE_BY][:2]) - 1.0)
    a[LEAD] = on * (a[LEAD] + fr[DT] * swung * (hurry - 1.0))
    # a limb strides as far as her stride or its own swing has it
    limbs = np.maximum(stepping, off)
    # along her way her centre of mass is damped to her cruise, not to her soles' middle:
    # damped to that she stopped between steps and each sole waited for her
    cruise = stride.cruises(way, out[latent.SWAY][DRIVE], turns, a[GOING])
    lat_dc = stepping * (ref[LATERAL] - out[latent.SWAY][LATERAL] * np.sin(out[latent.PSI][LATERAL]))
    across = 2.5 * lat_dc
    dv2 = (v[ALONG] - cruise - HALF_M * rate[SAGITTAL]) ** 2 + (v[LEFT] - across - fr[W0] * rate[LATERAL]) ** 2
    dvh = np.array([vh[ALONG] - v[ALONG] - HALF_M * PITCH_RAD * rate[PITCH], vh[LEFT] - v[LEFT], vh[UP] - v[UP]])
    dvh4 = (dvh * dvh) ** 2
    # her head's lag on her trunk's asked pitch is no speed of her centre's while something bears her:
    # read so, a rise's preload asked her centre 5.9 m/s^2 ahead off the floor's grip and pitched her back
    dvh = 0.5 * still * (1.0 - propped) * dvh * dvh4 / (dvh4 + 0.02)
    ref = ref + HOT[AXIAL] * stood.max() * still * poise.clip(0.5 * dv2 * (dv2 * dv2 / (dv2 * dv2 + 0.02)) / (G * AXIAL_M), 0.0, 0.25)
    z = z - propped * HOT[LATERAL] * z[LATERAL]
    zd = (zd + HOT[SAGITTAL] * ((v[ALONG] - cruise + dvh[0]) / HALF_M - zd[SAGITTAL])
          + np.maximum(stepping, propped) * HOT[LATERAL] * ((v[LEFT] - across) / fr[W0] - zd[LATERAL])
          + HOT[LATERAL] * (dvh[1] / fr[W0]))
    zdd = acc + kp * (ref - z) + kd * (rate - zd)
    # her limbs' places as her modes ask them; their sum's acceleration is her centre of
    # mass's, the other way; its ZMP into her soles, and which of them stand
    hands = HANDS * fr[UNIT][:, None, None]
    both = np.einsum('nia,n->ia', hands, zdd).mean(axis=0)
    swaying = (np.abs(ref[LATERAL] - lat_dc - z[LATERAL]) < ON[0]) & (np.abs(rate[LATERAL] - zd[LATERAL]) < ON[1])
    hips = y[HIP].reshape(2, 3) @ rot.T
    long = fr[REACH0] - AXIAL_M * fr[HIPS_BY] * np.maximum(ref[AXIAL], 0.0)
    # a prop bearing her higher than her legs are asked lifts her soles by the excess and relaxes her
    # height's spring by it (`pressed`): down they bore 0.61 of her on the bed; by the prop's share she brushed its frame
    short_h = propped * np.maximum((hips[:, UP] - floors).mean() - (ANKLE_H + long), 0.0)
    up = up + short_h / CLEAR_M
    stance, zmp = poise.stands(c[:2] + both[:2] / om ** 2, xi, low, high, om, out[latent.HELD], up,
                               swaying, (sweep, off, flying),
                               (y[LOAD], y[LOWEST] - floors, y[LOWEST] - fr[FLOOR], y[VCOM][1], y[PROP] * fr[WEIGHT]),
                               stood, prop)
    stance = stance * (1.0 - a[LET_GO])
    grounded, aloft, upright = np.maximum(stance.max(), propped), 1.0 - stance, stance.max() * (1.0 - propped)
    bears = (stance + TINY) / (stance.sum() + 2.0 * TINY)
    along = om ** 2 * (c[:2] - zmp)
    # along the floor no harder than her standing soles' grip gives: on ice (0.02) her step asked
    # a sole 0.85 of its load across and it slid
    along = along * np.minimum(1.0, (bears @ y[GRIP]) * G / np.maximum(np.linalg.norm(along), TINY))
    # striding: where a sole lands, how it is pitched, how a standing one is turned
    # a sole lands no further from her than its leg can push along itself inside its grip's cone:
    # on ice (0.1) a sole landed 0.3 m ahead of her as the other slid, slid too, and she fell
    reach = np.minimum(stride.REACH_M, (bears @ y[GRIP]) * (c[UP] - fr[FLOOR]))
    cruise, meet, top, land, adj, over, apart = _landings(y, fr, a, rot, c, flat, xi, cruise, om, turns, 2.0 * fr[W0] * (1.0 + ref[WIDTH]), across, reach, stood, u, flying)
    tip, tipping, tipped = stride.rocks(ref[ROCK], rate[ROCK], acc[ROCK])
    a[PREFOLD], short = stride.folds(off, y[LOAD], fr[KNEE] + a[BEND])
    a[HEELED], flatten, a[EDGE], rounds = stride.heels(
        (soles @ rot.T, tilt, tilting), hips[:, ALONG], floors, long, short, off,
        tip, stance, stepping, a[HEELED], fr[DT], fr[GROUND])
    a[SETTLE] = np.concatenate([flatten[:, None] * rot[LEFT], rounds @ rot], axis=1).ravel()
    # up as her height's spring asks of her standing legs' length: each leg's acceleration
    # along itself, what her going over it gives it taken off, by how far over its ankle
    # its hip is - her centre of mass less than her hips
    axial, (a[DUCKING], duck) = shortened((legs[0], legs[1] + adj, legs[2]), fr, stance), stride.ducked(out[latent.DUCKED], a[DUCKING], stance, stood, fr[DT])
    air = propped * aloft
    z += HOT[AXIAL] * (axial[0] - (bears @ (air * np.maximum(y[LOWEST] - floors, 0.0))) / (AXIAL_M * fr[HIPS_BY]) - z[AXIAL])
    zd += HOT[AXIAL] * (axial[1] - (bears @ (air * shod[:, UP]) + stance.max() * dvh[2]) / (AXIAL_M * fr[HIPS_BY]) - zd[AXIAL])
    pressed = acc[AXIAL] + kp[AXIAL] * (ref[AXIAL] + duck[0] / AXIAL_M - short_h / (AXIAL_M * fr[HIPS_BY]) - z[AXIAL]) + kd[AXIAL] * (rate[AXIAL] + duck[1] / AXIAL_M - zd[AXIAL])
    la = legs[0][:, :2] @ along
    risen = ((legs[2] ** 2 - ((v - shod) ** 2).sum(axis=1) - np.maximum(la, (1.0 - propped) * la)
              - legs[1] * AXIAL_M * fr[HIPS_BY] * pressed) / np.maximum(legs[0][:, UP], LOW_LEG * fr[REACH0]))
    # borne off her soles alone (propped, her floor sit's legs threw her back at 110-330 N), up as her height asks
    lifted = upright * (bears @ risen) / fr[HIPS_BY] - (1.0 - upright) * AXIAL_M * pressed
    com = grounded * np.array([along[0], along[1], poise.clip(lifted, -G, THROW_M_S2)]) + (1.0 - grounded) * FALLS
    late = stride.lates(phi, turns, stepping)
    # what a sole met (`machine.senses.touch`) her swings clear, for a turn of her rhythm; something bearing her off her
    # soles, a sole off the floor is no swing: it hovers CLEAR_M over the floor under it - clearing what it met,
    # sat with one sole lifted as the seat struck, that sole rose 16 cm and she fell
    a[RAISE] -= fr[DT] * turns / (2.0 * np.pi) * a[RAISE]
    lift2 = np.maximum(CLEAR_M * (1.0 + np.abs(out[latent.SWAY][LIFT])) + a[RAISE], top + 2.0 * CLEAR_M) * (1.0 - propped) + CLEAR_M * propped
    clear = aloft * (stride.clears(u, flying, swung * hurry, lift2, tip, tipping, tilt, late, land)
                     + HOT3[ALONG][:, None] * short_h)
    free, frame = swing.free(y, fr, (rot, v, om, shod, floors, rise), (ref, rate, acc, kp, kd, hands), clear, stance, cruise,
                             grounded, propped)
    strides = (limbs * grounded)[:, None]
    a[SOLE_ACC] = ((1.0 - strides) * (free @ frame + ((1.0 - grounded * BETWEEN) * com) @ rot) + strides * (
        np.concatenate([stride.goes(stride.guarded(meet, over, flat, apart), flat, shod, v, u, turns * hurry, kp[FREE_BY][:2], kd[FREE_BY][:2]),
                        free[:, UP:]], axis=1) @ rot)).ravel()
    a[SOLE_ACC], a[RESTS] = swing.rested(a[SOLE_ACC], out[latent.HELD], (grounded, aloft, strides), (kp, kd), (floors, y, shod, rot))
    # its pitch on the rock's own spring, from the floor's under the soles that stand; the
    # loop's turn law asked for what gives that
    a[TIPPED] = tilt + (aloft * (tipped + kp[ROCK] * (tip + bears @ stride.pitched(fr[GROUND]) - tilt) + kd[ROCK] * (tipping - tilting))
                        + TURN_KD[1] * tilting) / TURN_KD[0]
    a[GOING] += fr[DT] / GOING_S * ((y[VSOLE].reshape(2, 3) @ rot[ALONG]).mean() - a[GOING])
    a[STANCE], a[COM_ACC], a[STRIDES], a[BORNE] = stance, com @ rot, stepping, propped
    _poses(out, y, fr, a, (z, ref, rate, acc, kp, kd), (turn, spun), up, aloft * (1.0 - limbs), still)
    return a


def _landings(y, fr, a, rot, c, flat, xi, cruise, om, turns, wide, across, reach, stood, u, flying):
    """Where each sole lands on a floor told her (`stride.footholds`, her frame's map): (her
    cruise no faster than a tread a step, each landing (2, 2) her way's frame, the highest floor
    between each sole and its landing, m, the landing's floor over the felt one, m, and what each
    leg's length is read longer by, m: the floor under its sole over the floor's mean a stride
    behind her centre, so her height climbs as her centre crosses a tread, the leg on it from
    bent to her stance's; how far each sole's lowest point is over that highest floor, 0 to 1
    over CLEAR_M: a swing goes on only as it clears it, up first as the mocap's foot). Its tread
    is picked by where the sole will touch down - its natural landing carried her cruise over the
    swing's time left (`u`, `flying`): `meets` is exact at touchdown and sits at the stance sole
    at lift-off - and kept in `a[CELL]` as it swings. Nothing told, her landings as `stride.meets`
    has them, cleared, and the apart they lie at (`stride.meets`). What felled her: findings/stairs.md."""
    way, told, start = fr[MAP_WAY], fr[MAP_N], fr[MAP_AT] @ fr[MAP_WAY]
    xz = np.stack([rot[ALONG][[0, 2]], rot[LEFT][[0, 2]]])
    s_now, s_com = y[SOLE].reshape(2, 3)[:, [0, 2]] @ way - start, y[COM][[0, 2]] @ way - start
    hold = np.where(stood > 0.5, -1.0, a[CELL])
    # the told floor over the felt one, anchored under her standing soles: anchored at the telling,
    # her observer's height drifted 1.2 cm/s on the landing and each swing was driven under the floor
    gap = -(((stood + TINY) / (stood.sum() + 2.0 * TINY)) @ stride.floor_at(fr[MAP_ALONG], fr[MAP_RISE], told, s_now))
    step_s = np.pi / np.maximum(turns, TINY)
    left_s = step_s * (1.0 - u * flying)
    apart = stride.APART_M * np.minimum(wide / (2.0 * fr[W0]), 1.0)
    meet = stride.meets(xi, c, flat, cruise, om, turns, wide, across, reach, apart)
    s_meet = (meet @ xz) @ way - start
    run = stride.footholds(fr[MAP_ALONG], fr[MAP_RISE], told, s_meet, s_meet + cruise * left_s, s_now, s_com, reach, hold, gap, cruise * step_s)[3]
    cruise = np.minimum(cruise, run.min() * turns / np.pi)
    meet = stride.meets(xi, c, flat, cruise, om, turns, wide, across, reach, apart)
    s_meet = (meet @ xz) @ way - start
    s_new, top, land, _run, a[CELL] = stride.footholds(
        fr[MAP_ALONG], fr[MAP_RISE], told, s_meet, s_meet + cruise * left_s, s_now, s_com, reach, hold, gap, cruise * step_s)
    adj = (told > 0) * (stride.floor_at(fr[MAP_ALONG], fr[MAP_RISE], told, s_now)
                        - stride.floor_mean(fr[MAP_ALONG], fr[MAP_RISE], told, s_com - stride.STRIDE_M, s_com))
    return (cruise, meet + ((s_new - s_meet)[:, None] * way[None, :]) @ xz.T, top, land, adj,
            poise.clip((y[LOWEST] - fr[FLOOR] - top) / CLEAR_M, 0.0, 1.0), apart)


def _poses(out, y, fr, a, said, turned, up, folds, still):
    """Her turns, her arms and her knees into `a`. Her trunk's and her pelvis's pitch, each on
    the pitch's spring, her pelvis's list and its turn on theirs - her trunk upright and on
    her way -, the loop's own turn law asked for what gives that; her arms on the posture's
    own spring, asked ahead of it by what its damper holds back. `said` her modes' (as she
    is, reference, rate, acceleration, 1/s^2, 1/s); `turned` her turns and their rates; `up`
    how far each sole is asked up, `folds` which soles' knees are asked a fold."""
    z, ref, rate, acc, kp, kd = said
    turn, spun = turned
    wh = fr[ROT].reshape(3, 3) @ y[HSPIN]
    dwh = np.array([wh[LEFT] - spun[0, 1], wh[ALONG] - spun[1, 1], 0.0])
    dwh4 = (dwh * dwh) ** 2
    spun = spun + 0.5 * still * (1.0 - PELVIS) * (dwh * dwh4 / (dwh4 + 0.04))[:, None]
    turn_dc = ref[TURN] - out[latent.SWAY][TURN] * np.sin(out[latent.PSI][TURN])
    aim = np.stack([fr[PITCH0] + PITCH_RAD * ref[PITCH], PELVIS * LIST_RAD * ref[LIST], TURN_RAD * (PELVIS * ref[TURN] + (1.0 - PELVIS) * 1.10 * turn_dc)])
    aim_rate = np.stack([PITCH_RAD * rate[PITCH] + 0.0 * PELVIS, PELVIS * LIST_RAD * rate[LIST], PELVIS * TURN_RAD * rate[TURN]])
    aim_acc = np.stack([PITCH_RAD * acc[PITCH] + 0.0 * PELVIS, PELVIS * LIST_RAD * acc[LIST], PELVIS * TURN_RAD * acc[TURN]])
    by = np.array([PITCH, LIST, TURN])
    # borne, her ZMP let go to it, her pelvis's turns have no spring of their own - what bears her
    # holds it -, its rates her trunk's: sprung to her standing pitch on a seat, or on her trunk's
    # error, it snapped 0.33 rad on in 0.11 s as her trunk came upright
    sat = a[BORNE] * (1.0 - poise.clip(out[latent.HELD], 0.0, 1.0))
    turning = aim_acc + kp[by][:, None] * (aim - turn) * (1.0 - sat * PELVIS * SEAT_HOLDS) + kd[by][:, None] * (aim_rate - spun)
    a[WANT], a[ROLLED], a[HEADED] = turn + (turning + TURN_KD[1] * spun) / TURN_KD[0]
    a[TURNED] = 1.0 * (kp[PITCH] + kd[PITCH] > 0.0)
    a[SWUNG] = ARM_RAD * (ref[ARMS] + (acc[ARMS] + POSTURE_KD[1] * rate[ARMS]) / POSTURE_KD[0])
    a[REACHED] = ARM_RAD * (ref[REACH] + (acc[REACH] + POSTURE_KD[1] * rate[REACH]) / POSTURE_KD[0])
    a[ELBOWED] = ARM_RAD * (ref[ELBOWS] + (acc[ELBOWS] + POSTURE_KD[1] * rate[ELBOWS]) / POSTURE_KD[0])
    # her posture's knees bent as her height is, AHEAD of it toward what is asked: bent as
    # asked, a throw's step threw her knees straight at 100 rad/s^2 and her over forward,
    # 8 kg m^2/s about her; bent as she is, a crouch asked 0.6 stopped at 0.56
    bent = z[AXIAL] + poise.clip(ref[AXIAL] - z[AXIAL], -AHEAD, AHEAD)
    cosine = np.cos(0.5 * fr[KNEE]) - AXIAL_M * bent * fr[HIPS_BY] / LEG_M
    a[BEND] = 2.0 * np.arccos(poise.clip(cosine, -1.0, 1.0)) - fr[KNEE]
    # and at the told height's rate where the rows let her soles go (a throw's ZMP NONE): damped
    # against still, the stack's knee rows held a push's knees at 0.2 of their clamp, her height
    # at its band's floor, 3 m/s^2 under its ask; at the told rate under a held polygon the
    # seats' texts, laid against that damping, sat her onto the bed at 1.86 of her weight and
    # she fell, or had it bear 0.00 of her; at her own rate past AHEAD, a stride's instant
    # throw left its knees undamped and the leap flew 0.20 s for 0.23
    a[BEND_RATE] = (2.0 * AXIAL_M * fr[HIPS_BY] * rate[AXIAL] * float(abs(cosine) < 1.0) / LEG_M
                    / np.sqrt(np.maximum(1.0 - cosine * cosine, TINY))) * (1.0 - poise.clip(out[latent.HELD], 0.0, 1.0))
    # a sole asked up where she stands: its knee folded as its leg is shorter by the lift
    # under its hip. Striding, a limb's knee is its sole's place's alone
    short = poise.clip(np.cos(0.5 * (fr[KNEE] + a[BEND])) - CLEAR_M * np.maximum(up, 0.0) / LEG_M, -1.0, 1.0)
    a[FOLD] = 2.0 * np.arccos(short) - fr[KNEE] - a[BEND]
    a[FOLD_RATE] = (2.0 * CLEAR_M * LIMBS * rate[LIFT] * (up > 0.0) / LEG_M
                    / np.sqrt(np.maximum(1.0 - short * short, TINY)))
    a[FOLDS] = folds
    k = y[MOMENT]
    a[KDOT] = -MOMENTUM_K * k - UPWARD * YAW_K * (k[1] - poise.clip(k[1], -out[latent.YAWED], out[latent.YAWED]))
