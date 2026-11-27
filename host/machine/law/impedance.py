"""The robot's modes' springs on its body: `latent.step`'s out rendered, the whole-body loop's ask.

    a = impedance.ask(out, y, fr, a)        # flat, `machine.law.modal`'s layouts; a pass

Arrays in, arrays out and no branch (tests/test_ortho.py). Each mode is a mass on a spring and a
damper to its reference: the sum's on the robot's whole mass, the difference's on a limb's, a turn's on
its trunk's; what the sum's springs ask is its centre of mass's acceleration, its ZMP in its
soles (`machine.law.poise`), up no harder than its legs throw it. A sole that does not
stand is sprung to the place the robot's modes give it from the soles that do - none standing, from its
centre of mass as it falls -, each axis on the difference's impedance: the drive's along, the width's
across, the lift's up; striding, it is the stride's (`machine.law.stride`), and aloft the swing that
lands first (`stride.flown`) strides on falling with it, the other hung (`swing.hung`). Hung plumb
under the robot, a leap's landing sole came down 5-8 cm ahead of it for a capture point 0.2-0.3 m on and
it fell a step later (its walk's leap 3 of 9, 9 so; the gym's walk, hop and stand 5, 9); both
soles striding aloft, the push leg behind it was thrown on (the run's leap 18 stumbles, the gym's jog,
hop and stand 0 of 9). The loop finds the torques and clips them to the drives (`machine.stack.wbc`,
`machine.stack.wrench`).
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import libm
from machine.body import gait
from machine.senses import borne
from machine.law import latent, poise, rocker, stride, swing, treads, vault
from machine.law.posed import posed
from machine.law.modal import ANKLE_H, AXIAL, AXIAL_M, BEGUN, BEND, BORNE, CELL, CLEAR_M, COM, COM_ACC, DRIVE, DT, DUCKING, EDGE, FLOOR, G, GOING, GRIP, GROUND, HANDS, DIP, AIM, MEET, STRODE, HEELED, HIP, HIPS_BY, HOT, KNEE, LATERAL, LEAD, LET_GO, LIFT, LOAD, LOWEST, MAP_ALONG, MAP_AT, MAP_N, MAP_RISE, MAP_WAY, MASS, NEED, OUTSIDE, PITCH, PLANTED, PITCH_RAD, PREFOLD, PROP, RAISE, REACH0, RESTS, ROCK, ROT, SAGITTAL, SETTLE, SOLE, SOLE_ACC, SPLIT, STANCE, STRIDES, SWINGS, TILT, TILT_RATE, TIPPED, UNIT, VCOM, VHEAD, VSOLE, W0, WEIGHT, WIDTH, reading, shortened, under
from machine.law.poise import HALF_M, TINY
from machine.law.stride import ALONG, LEFT, LIMBS, UP
from machine.law.swing import BETWEEN, FREE_BY, HOT3
from machine.stack.wbc import TURN_KD

#: The robot's springs' numbers, its build's (`[gait.impedance]`: each with what it was found at,
#: `machine.body.gait`).
_G = gait.table('impedance')
#: A sole asked up leaves only while the robot sways as asked, within ON of its lateral shift and rate.
ON = (_G['on_shift'], _G['on_shift_1_s'])
#: The going: the soles' speed along the way, meaned over GOING_S.
GOING_S = _G['going_s']
#: The centre of mass is asked up no harder than this, m/s^2.
THROW_M_S2 = _G['throw_m_s2']
#: The height no lower than this for omega; a hip no lower over its ankle than this of its leg.
LOW_M, LOW_LEG = _G['low_m'], _G['low_leg']
FALLS = np.array([0.0, 0.0, -G])


#: Borne off the robot's soles this share of its weight, it is supported there wholly: its support
#: region spans where, its centre of mass asked of its springs, not let fall.
PROP_ON = _G['prop_on']


def ask(out, y, fr, a):
    """`a`, the loop's ask this pass: `out` (`latent.step`) rendered on the robot as it is sensed."""
    rot = fr[ROT].reshape(3, 3)
    stood, begun, was, thrown = a[STANCE].copy(), a[BEGUN].copy(), a[SWINGS].copy(), 1.0 - poise.clip(out[latent.HELD], 0.0, 1.0)
    z, zd, legs, turn, spun = reading(y, fr, stood)
    turns, sway, psi, beat = out[latent.TURNS], out[latent.SWAY], out[latent.PSI], out[latent.PSI][LIFT]
    c, v, vh = rot @ y[COM], rot @ y[VCOM], rot @ y[VHEAD]
    om = np.sqrt(G / np.maximum(c[UP] - fr[FLOOR], LOW_M))
    soles, shod = y[SOLE].reshape(2, 3), y[VSOLE].reshape(2, 3) @ rot.T
    tilt, tilting = y[TILT], y[TILT_RATE]
    flat, xi = (soles @ rot.T)[:, :2], c[:2] + v[:2] / om
    low, high = poise.boxes(flat, a[EDGE], tilt)
    floors, rise = under(y, fr, stood)
    propped, prop = borne.latched(y, rot, a, fr[DT])
    # the lift's own rhythm is the stride's clearance, not a sole's place
    tone = sway[LIFT] * np.array([libm.sin(beat), turns * libm.cos(beat), -turns * turns * libm.sin(beat)])
    ref, rate, acc = (out[part] - HOT[LIFT] * t for part, t in zip((latent.REF, latent.RATE, latent.ACC), tone))
    up = LIMBS * ref[LIFT]
    # the robot's capture point past the soles that stood - past its ZMP's margin in them - steps it while the
    # rows hold the polygon: let go, a push stepped a sole at its end (0/497 N)
    still = 1.0 - poise.clip(np.linalg.norm(sway) / TINY, 0.0, 1.0)
    out_of = poise.BIG * (1.0 - stood)[:, None]
    lifted = out_of * np.maximum(1.0 - still, np.maximum(a[SWINGS], np.maximum(up > poise.LIFT_EPS, y[LOWEST] - floors > 0.5 * CLEAR_M)))[:, None]
    lo, hi = poise.boxes(flat, 0.0 * a[EDGE], tilt)
    beyond = xi - poise.clip(xi, *poise.spanned((lo + out_of).min(axis=0), (hi - out_of).max(axis=0), prop))
    past = np.linalg.norm(xi - poise.clip(xi, *poise.spanned((lo + lifted).min(axis=0), (hi - lifted).max(axis=0), prop)))
    a[NEED] = (1.0 - propped) * stood.max() * poise.clip(past / poise.MARGIN_M, 0.0, 1.0) * poise.clip(out[latent.HELD], 0.0, 1.0)
    phi_w, lat0 = stride.swings(sway[DRIVE], turns, a[GOING]), ref[LATERAL] - sway[LATERAL] * libm.sin(psi[LATERAL])
    stepping, way, sweep, phi, on, off, u, flying, swung = stride.beat(
        sway[DRIVE], psi[DRIVE], sway[LATERAL], psi[LATERAL], turns,
        a[SWINGS], a[NEED], a[LEAD], a[SPLIT], begun, stood, phi_w, thrown, lat0)
    a[SWINGS], a[SPLIT] = off, stride.split(a[SPLIT], out[latent.HELD], turns, fr[DT], stride.first(a[NEED], still, out[latent.RATE], a[SWINGS], stood, sway[DRIVE], out[latent.ASKED][DRIVE], lat0, beyond, y[LOAD], phi, phi_w))
    kp, kd, swung = out[latent.KS] / fr[MASS], out[latent.DS] / fr[MASS], swung / (1.0 - begun)
    # a swing goes ahead of its rhythm as far as lands it before the robot's capture point is a reach across
    # past the soles that stood, its own side of them - along the way too while its rhythm is still, no stride
    # pacing its steps: landed from the run's held leap its capture point ran on at 1.6 m/s, a swing at the
    # still rhythm's pace took 0.40 s and the robot went over a sole's ball
    aside = stood.max() * np.maximum(np.maximum(LIMBS * beyond[LEFT], 0.0), still * np.abs(beyond[ALONG]))
    # past a lone standing sole's own side no swing reaches: never in a plain walk
    a[OUTSIDE] = (stood * (stood.sum() < 1.5)) @ np.maximum(LIMBS * beyond[LEFT], 0.0)
    hurry = 1.0 + flying * (stride.hurried(aside, om, u, swung, kp[FREE_BY][:2]) - 1.0)
    a[LEAD] = on * (a[LEAD] + fr[DT] * swung * (hurry - 1.0))
    # a limb strides as far as the stride or its own swing has it
    limbs = np.maximum(stepping, off)
    # along the way the centre of mass is damped to the cruise, not to the soles' middle: damped to that it
    # stopped between steps and each sole waited for its
    lat_dc = stepping * lat0
    cruise, across = stride.cruises(way, sway[DRIVE], turns, a[GOING], phi_w), 2.5 * lat_dc
    dv2 = libm.sq(v[ALONG] - cruise - HALF_M * rate[SAGITTAL]) + libm.sq(v[LEFT] - across - fr[W0] * rate[LATERAL])
    dvh = np.array([vh[ALONG] - v[ALONG] - HALF_M * PITCH_RAD * rate[PITCH], vh[LEFT] - v[LEFT], vh[UP] - v[UP]])
    dvh4 = libm.sq(dvh * dvh)
    # the robot's head's lag on its trunk's asked pitch is no speed of its centre's while something bears it;
    # thrown into a leap, horizontal head lag is gated while vertical head lift remains active.
    dvh = 0.5 * still * (1.0 - propped) * np.array([1.0 - thrown, 1.0 - thrown, 1.0]) * dvh * dvh4 / (dvh4 + 0.02)
    ref = ref + HOT[AXIAL] * stood.max() * still * poise.clip(0.5 * dv2 * (dv2 * dv2 / (dv2 * dv2 + 0.02)) / (G * AXIAL_M), 0.0, 0.25)
    z = z - propped * HOT[LATERAL] * z[LATERAL]
    zd = (zd + HOT[SAGITTAL] * ((v[ALONG] - cruise + dvh[0]) / HALF_M - zd[SAGITTAL])
          + np.maximum(stepping, propped) * HOT[LATERAL] * ((v[LEFT] - across) / fr[W0] - zd[LATERAL])
          + HOT[LATERAL] * (dvh[1] / fr[W0]))
    zdd = acc + kp * (ref - z) + kd * (rate - zd)
    # the robot's limbs' places as its modes ask them; their sum's acceleration is its centre of mass's, the
    # other way; its ZMP into its soles, and which of them stand
    hands = HANDS * fr[UNIT][:, None, None]
    both = np.einsum('nia,n->ia', hands, zdd).mean(axis=0)
    swaying = (np.abs(ref[LATERAL] - lat_dc - z[LATERAL]) < ON[0]) & (np.abs(rate[LATERAL] - zd[LATERAL]) < ON[1])
    hips = y[HIP].reshape(2, 3) @ rot.T
    long = fr[REACH0] - AXIAL_M * fr[HIPS_BY] * np.maximum(ref[AXIAL], 0.0)
    # a prop bearing the robot higher than its legs are asked lifts its soles and eases its height's spring by
    # the excess (`pressed`): down they bore 0.61 of it on the bed; by the prop's share it brushed its frame
    short_h = propped * np.maximum((hips[:, UP] - floors).mean() - (ANKLE_H + long), 0.0)
    up = up + short_h / CLEAR_M
    stance, zmp = poise.stands(c[:2] + both[:2] / (om * om), xi, low, high, om, out[latent.HELD], up,
                               swaying, (sweep, off, flying, 1.0 * (tilt - rocker.pitched(fr[GROUND]) >= stride.HEEL_RAD)),
                               (y[LOAD], y[LOWEST] - floors, y[LOWEST] - fr[FLOOR], y[VCOM][1], y[PROP] * fr[WEIGHT]),
                               stood, prop, poise.clip(np.abs(sway[DRIVE]) / stride.STIRS, 0.0, 1.0))
    stance = stance * (1.0 - a[LET_GO])
    grounded, aloft, upright = np.maximum(stance.max(), propped), 1.0 - stance, stance.max() * (1.0 - propped)
    flew = swing.flew(thrown, grounded, v, y[LOWEST] - floors, shod)
    bears = (stance + TINY) / (stance.sum() + 2.0 * TINY)
    along = (om * om) * (c[:2] - zmp)
    # along the floor no harder than the standing soles' grip gives: on ice (0.02) a step asked a sole 0.85 of
    # its load across and it slid
    along = along * np.minimum(1.0, (bears @ y[GRIP]) * G / np.maximum(np.linalg.norm(along), TINY))
    # striding: where a sole lands, how it is pitched, how a standing one is turned
    # a sole lands no further from the robot than its leg can push along itself inside its grip's cone: on
    # ice (0.1) a sole landed 0.3 m ahead of it as the other slid, slid too, and the robot fell
    reach = np.minimum(stride.REACH_M, (bears @ y[GRIP]) * (c[UP] - fr[FLOOR]))
    cruise, meet, top, land, adj, over, apart = _landings(y, fr, a, rot, c, flat, xi, cruise * (np.abs(out[latent.ASKED][DRIVE]) > TINY), om, turns, 2.0 * fr[W0] * (1.0 + ref[WIDTH]), across, reach, stood, u, flying, flew)
    tip, tipping, tipped = rocker.rocks(ref[ROCK], rate[ROCK], acc[ROCK])
    a[PREFOLD], short = rocker.folds(off * (1.0 - u), y[LOAD], fr[KNEE] + a[BEND])
    a[HEELED], flatten, a[EDGE], rounds, reach_h = rocker.heels(
        (soles @ rot.T, tilt, tilting), hips[:, ALONG], floors, long, short, off,
        tip, stance, stepping, a[HEELED], fr[DT], fr[GROUND])
    a[SETTLE] = np.concatenate([flatten[:, None] * rot[LEFT], rounds @ rot], axis=1).ravel()
    # up as the robot's height's spring asks of its standing legs' length: each leg's acceleration along
    # itself, less what its going over it gives it, by how far over its ankle its hip is - its centre of mass
    # less than its hips -; its heel asked up as holds its hip at the height it rises for (`vault.vaults`)
    axial, (a[DUCKING], duck) = shortened((legs[0], legs[1] + adj, legs[2]), fr, stance), stride.ducked(out[latent.DUCKED], a[DUCKING], stance, stood, fr[DT])
    air = propped * aloft
    z += HOT[AXIAL] * (axial[0] - (bears @ (air * np.maximum(y[LOWEST] - floors, 0.0))) / (AXIAL_M * fr[HIPS_BY]) - z[AXIAL])
    zd += HOT[AXIAL] * (axial[1] - (bears @ (air * shod[:, UP]) + stance.max() * dvh[2]) / (AXIAL_M * fr[HIPS_BY]) - zd[AXIAL])
    pressed = acc[AXIAL] + kp[AXIAL] * (ref[AXIAL] + duck[0] / AXIAL_M - short_h / (AXIAL_M * fr[HIPS_BY]) - z[AXIAL]) + kd[AXIAL] * (rate[AXIAL] + duck[1] / AXIAL_M - zd[AXIAL])
    risen = vault.vaults(legs, v, shod, along, propped, pressed, AXIAL_M, fr[HIPS_BY], LOW_LEG * fr[REACH0], hips[:, UP] - floors, reach_h, duck, a[HEELED],
                         poise.clip(out[latent.HELD], 0.0, 1.0) * flying[::-1], out[latent.ASKED][DRIVE], kp[AXIAL], kd[AXIAL], fr[REACH0] / libm.cos(0.5 * fr[KNEE]), fr[DT], out[latent.HELD])
    # borne off the robot's soles alone (propped, its floor sit's legs threw it back at 110-330 N), up as its
    # height asks
    lifted = upright * (bears @ risen) / fr[HIPS_BY] - (1.0 - upright) * AXIAL_M * pressed
    com = grounded * np.array([along[0], along[1], poise.clip(lifted, -G, THROW_M_S2)]) + (1.0 - grounded) * FALLS
    late = stride.lates(phi, turns, stepping, phi_w)
    # what a sole met (`machine.senses.touch`) the robot's swings clear, for a turn of its rhythm; borne off
    # its soles, a sole off the floor is no swing: it hovers CLEAR_M over the floor under it - clearing what
    # it met, sat with one sole lifted as the seat struck, that sole rose 16 cm and the robot fell
    a[RAISE] -= fr[DT] * turns / (2.0 * np.pi) * a[RAISE]
    lift2 = np.maximum(CLEAR_M * (1.0 + np.abs(out[latent.SWAY][LIFT])) + a[RAISE], top + 2.0 * CLEAR_M) * (1.0 - propped) + CLEAR_M * propped
    clear = aloft * (stride.clears(u, np.maximum(flying, off), swung, lift2, tip, tipping, tilt, late, land, hurry)
                     + HOT3[ALONG][:, None] * short_h)
    free, frame = swing.free(y, fr, (rot, v, om, shod, floors, rise), (ref, rate, acc, kp, kd, hands), clear, stance, cruise,
                             grounded, propped)
    (free, went, strides, level), fell = swing.hung(free, (stride.goes(a[AIM].reshape(2, 2), flat, shod, v, u, off, turns * np.sqrt(hurry) / (1.0 - begun), kp[FREE_BY][:2], kd[FREE_BY][:2], phi_w), a[AIM].reshape(2, 2), flat, v, stride.eased(u, off)),
                                                    limbs * np.maximum(grounded, stride.flown(a[STRODE], off, stood, u, thrown, 1.0 * (v[UP] > 0.0))), flew, (lift2, y[LOWEST] - floors, shod), (kp, kd)), ((1.0 - grounded) * com) @ rot
    a[SOLE_ACC] = ((1.0 - strides) * (free @ frame + ((1.0 - grounded * BETWEEN) * com) @ rot) + strides * (fell + went @ rot)).ravel()
    a[SOLE_ACC], a[RESTS] = swing.rested(a[SOLE_ACC], out, (grounded, aloft, strides), (kp, kd), (floors, y, shod, rot))
    # a sole's pitch on the rock's own spring, from the floor's under the soles that stand; the loop's turn
    # law asked for what gives that
    a[TIPPED] = tilt + (aloft * stride.eased(u, off) * (level * tipped + kp[ROCK] * (level * tip + bears @ rocker.pitched(fr[GROUND]) - tilt) + kd[ROCK] * (level * tipping - tilting))
                        + TURN_KD[1] * tilting) / TURN_KD[0]
    a[GOING] += fr[DT] / GOING_S * ((y[VSOLE].reshape(2, 3) @ rot[ALONG]).mean() - a[GOING])
    a[STANCE], a[COM_ACC], a[STRIDES], a[BORNE], a[BEGUN], a[DIP], a[STRODE] = stance, com @ rot, stepping, propped, stride.begins(begun, was, stood, stance, on, off, phi, phi_w, thrown), reach_h - (hips[:, UP] - floors), strides[:, 0]
    posed(out, y, fr, a, (z, ref, rate, acc, kp, kd), (turn, spun), up, aloft * (1.0 - limbs), still)
    return a


def _landings(y, fr, a, rot, c, flat, xi, cruise, om, turns, wide, across, reach, stood, u, flying, flown):
    """Where each sole lands on a floor told the robot (`treads.footholds`, its frame's map): (its
    cruise no faster than a tread a step, each landing (2, 2) its way's frame, the highest floor
    between each sole and its landing, m, the landing's floor over the felt one, m, and what each
    leg's length is read longer by, m: the floor under its sole over the floor's mean a stride
    behind the robot's centre, so its height climbs as its centre crosses a tread, the leg on it from
    bent to its stance's; how far each sole's lowest point is over that highest floor, 0 to 1
    over CLEAR_M: a swing goes on only as it clears it, up first as the mocap's foot). A sole's tread
    is picked by where it will touch down - its natural landing carried the cruise over the
    swing's time left (`u`, `flying`): `meets` is exact at touchdown and sits at the stance sole
    at lift-off - and kept in `a[CELL]` as it swings. Nothing told, the landings as `stride.meets`
    has them, cleared, and the apart they lie at (none thrown aloft, `flown`: no sole stands to lie apart of); what felled the robot: findings/stairs.md.
    `cruise` is the robot's cruise while its drive is asked and none once it is let go (`latent.ASKED`): a stop's first step, planned on the march's cruise still
    running down, landed 1 cm ahead of its centre and 23 cm short of its capture point (0.82 m/s), it ran on to 1.24 and the next step braked it
    at 3.48 of its weight; on `stride.cruises` of none, floored at its going less AHEAD_M_S, the stop's steps kept 2-5 cm of a walk's offset short of its
    capture point; on the ask alone its march's first steps, placed for a pace it had not reached, fell it at 4.1 s on the 143 N m/rad leaf; on
    the lesser of the two, rhythm.modulated's re-asking of the drive every pass of a turn moved the landings through it and the turn's stop fell (1040 turns 9/9 -> 8/9, the
    143 leaf's -> 6/9)."""
    way, told, start = fr[MAP_WAY], fr[MAP_N], fr[MAP_AT] @ fr[MAP_WAY]
    xz = np.stack([rot[ALONG][[0, 2]], rot[LEFT][[0, 2]]])
    s_now, s_com = y[SOLE].reshape(2, 3)[:, [0, 2]] @ way - start, y[COM][[0, 2]] @ way - start
    hold = np.where(stood > 0.5, -1.0, a[CELL])
    # the told floor over the felt one, anchored under the standing soles: anchored at the telling,
    # its observer's height drifted 1.2 cm/s on the landing and each swing was driven under the floor
    gap = -(((stood + TINY) / (stood.sum() + 2.0 * TINY)) @ treads.floor_at(fr[MAP_ALONG], fr[MAP_RISE], told, s_now))
    step_s = np.pi / np.maximum(turns, TINY)
    left_s = step_s * (1.0 - u * flying)
    apart = stride.APART_M * np.minimum(wide / (2.0 * fr[W0]), 1.0) * (1.0 - flown)
    meet = stride.meets(xi, c, flat, cruise, om, turns, wide, across, reach, apart, fr[PLANTED])
    s_meet = (meet @ xz) @ way - start
    run = treads.footholds(fr[MAP_ALONG], fr[MAP_RISE], told, s_meet, s_meet + cruise * left_s, s_now, s_com, reach, hold, gap, cruise * step_s)[3]
    cruise = np.minimum(cruise, run.min() * turns / np.pi)
    meet = stride.meets(xi, c, flat, cruise, om, turns, wide, across, reach, apart, fr[PLANTED])
    s_meet = (meet @ xz) @ way - start
    s_new, top, land, _run, a[CELL] = treads.footholds(
        fr[MAP_ALONG], fr[MAP_RISE], told, s_meet, s_meet + cruise * left_s, s_now, s_com, reach, hold, gap, cruise * step_s)
    adj = (told > 0) * (treads.floor_at(fr[MAP_ALONG], fr[MAP_RISE], told, s_now)
                        - treads.floor_mean(fr[MAP_ALONG], fr[MAP_RISE], told, s_com - stride.STRIDE_M, s_com))
    meet = meet + ((s_new - s_meet)[:, None] * way[None, :]) @ xz.T
    over = poise.clip((y[LOWEST] - fr[FLOOR] - top) / CLEAR_M, 0.0, 1.0)
    # each sole's landing and its aim this pass (`stride.guarded`), told in the ask
    a[MEET], a[AIM] = meet.ravel(), stride.guarded(meet, over, flat, apart, fr[PLANTED]).ravel()
    return cruise, meet, top, land, adj, over, apart
