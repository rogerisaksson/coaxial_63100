"""Her latent modes on her body: what is sensed read as modes, the loop's ask laid flat.

    y = modal.sensed(s, loads, y)                       # flat, of `wbc.sense`'s s and her soles' N
    fr = modal.frame(y, mass, limb, trunk, arm, knee, dt)    # her statics, once
    rest = modal.rest(fr)                               # her rest vector (`latent.state`)
    z, rate = modal.read(y, fr, a[modal.STANCE])        # her modes as she is, and their rates
    posed = modal.poses(a, y, fr, posture, bend, arms, knees)  # the arithmetic of the stack's ask
    ask = modal.given(a, y, posed, knees, bend)         # the stack's ask (`wbc.step`) of `a`

Flat arrays and arithmetic on them; a choice is a mask, a limit a minimum or a maximum, a
change a blend. A limb's place is its sole's from her centre of mass, along her way, to her
left and up. Her modes are those places' sums and differences (HANDS): of the sum, her centre
of mass over her soles' middle - its place across and along; of the difference, her limbs
against each other - the drive along, the lift, the width. Her height is her standing legs'
compression: how much shorter they are, hip to ankle, than she stood on - over one leg held
at its length she rises and comes down as its arc does. Her trunk's pitch, her pelvis's list
and turn, her arms' swing and her soles' rock are turns. `machine.impedance` renders their
springs into `a`, the loop's ask.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import latent, leap, ortho, stride, terms
from machine.figure import ANKLE_H, SHANK, THIGH
from machine.poise import HALF_M, TINY, clip
from machine.stride import ALONG, LEFT, LIMBS, ROCK_RAD, STRIDE_M, UP
from machine.wbc import HEIGHT_KD, POSTURE_KD, TURN_KD

G = 9.81
N = ortho.N
(AXIAL, LATERAL, DRIVE, PITCH, SAGITTAL, LIFT, WIDTH, LIST, TURN, ARMS,
 ROCK) = (ortho.MODES.index(m) for m in (
     'axial_compression', 'lateral_shift', 'sagittal_drive', 'pitch_attitude', 'sagittal_shift',
     'limb_lift', 'stance_width', 'pelvis_list', 'pelvis_turn', 'arm_swing', 'foot_rock'))
#: Of the pelvis and the trunk, the pelvis.
PELVIS = np.array([1.0, 0.0])

#: What is sensed, world frame: her centre of mass and its speed; each sole's middle and its
#: speed; each sole's load, N; the pelvis's and the trunk's quaternions and spins; her angular
#: momentum about her centre of mass; each sole's lowest corner's height; each hip's joint;
#: each sole's pitch, its heel up, rad, and its rate; each shoulder's reach forward, rad; the
#: share of her weight borne off her soles, and the box it is borne on (low, high; world); each
#: sole's grip, the floor's friction under it as she has felt it (`machine.grip`).
COM, VCOM, SOLE, VSOLE = slice(0, 3), slice(3, 6), slice(6, 12), slice(12, 18)
LOAD, QUAT, SPIN, MOMENT = slice(18, 20), slice(20, 28), slice(28, 34), slice(34, 37)
LOWEST, HIP, TILT, TILT_RATE, ARM = slice(37, 39), slice(39, 45), slice(45, 47), slice(47, 49), slice(49, 51)
PROP, PROP_LO, PROP_HI, GRIP = 51, slice(52, 55), slice(55, 58), slice(58, 60)
VHEAD, HQUAT, HSPIN, CELLS = slice(60, 63), slice(63, 67), slice(67, 70), slice(70, 74)
SENSED = 74
#: Her statics: her way's frame (3 x 3: along, left, up in the world); her centre of mass over
#: her soles and her soles' half width as she stood; the floor; omega; her weight, N; her knees'
#: bend standing, rad; the pelvis's and the trunk's pitch then; her heading, rad; a hip from
#: its ankle as she stood, m; how far her hips come down a metre of her centre of mass - her
#: limbs' own mass comes half as far; her shoulders then; each mode's unit on her (m, a
#: turn's rad), its reading where its coordinate is 0, the mass it moves, kg.
ROT, H0, W0, FLOOR, OMEGA, WEIGHT, KNEE = slice(0, 9), 9, 10, 11, 12, 13, 14
PITCH0, HEADING, DT, REACH0, HIPS_BY, ARM0 = slice(15, 17), 17, 18, 19, 20, slice(21, 23)
UNIT, Z0, MASS = (slice(23 + i * N, 23 + (i + 1) * N) for i in range(3))
#: And the floor's pitch under each sole, as a sole pitched flat on it, rad, its heel up; the
#: point its height is read at, world x and z, m.
GROUND = slice(23 + 3 * N, 25 + 3 * N)
FLOOR_AT = slice(25 + 3 * N, 27 + 3 * N)
#: And a floor told her (`worker.told_floor`), MAP_K breakpoints at most: where she stood when
#: told and her way then, world x and z; each breakpoint along that way, m, and its height over
#: the floor felt then (MAP_FLOOR0); how many were told, 0 none.
MAP_K = 16
MAP_AT, MAP_WAY = slice(27 + 3 * N, 29 + 3 * N), slice(29 + 3 * N, 31 + 3 * N)
MAP_ALONG, MAP_RISE = slice(31 + 3 * N, 31 + 3 * N + MAP_K), slice(31 + 3 * N + MAP_K, 31 + 3 * N + 2 * MAP_K)
MAP_FLOOR0, MAP_N = 31 + 3 * N + 2 * MAP_K, 32 + 3 * N + 2 * MAP_K
FRAME = MAP_N + 1
#: The loop's ask, flat: each sole standing (1) or not; her centre of mass's acceleration and
#: each sole's, world; the pelvis's and the trunk's pitch wanted, rad; each knee's fold over its
#: posture and its rate; whether a turn is asked; her knees' bend with her height; her angular
#: momentum's rate.
STANCE, COM_ACC, SOLE_ACC = slice(0, 2), slice(2, 5), slice(5, 11)
WANT, FOLD, FOLD_RATE = slice(11, 13), slice(13, 15), slice(15, 17)
TURNED, BEND, KDOT = 17, 18, slice(19, 22)
#: Kept in the ask from pass to pass: her going, the speed her soles have made along her way
#: of late, m/s.
GOING = 22
#: And: the pelvis's and the trunk's roll and turn wanted, the turn from her heading; each
#: sole's pitch wanted; her arms' swing; which soles' knees are asked a fold.
ROLLED, HEADED, TIPPED, SWUNG, FOLDS = slice(23, 25), slice(25, 27), slice(27, 29), 29, slice(30, 32)
#: And each standing sole's own turn and its middle's acceleration, world: flat again, or
#: over its ball; how far each stands on an edge, 1 its ball and -1 its heel; and, kept, the
#: heel's rise asked of each the pass before.
SETTLE, EDGE, HEELED = slice(32, 44), slice(44, 46), slice(46, 48)
#: And how far she strides, 0 to 1; each standing knee's fold before its swing, rad; kept,
#: how far each limb's stride has it off, 0 to 1, and how much higher her swings clear the
#: floor for what a sole met, m; how far her capture point steps her, 0 to 1; how far each
#: swing has gone ahead of its rhythm, of its share; how far her capture point is past a lone
#: standing sole's own side, m - where no swing reaches; how far what bears her off her soles
#: supports her, 0 to 1.
STRIDES, PREFOLD, SWINGS, RAISE, NEED, LEAD, OUTSIDE, BORNE = 48, slice(49, 51), slice(51, 53), 53, 54, slice(55, 57), 57, 58
#: And her knees' bend's rate, rad/s: the stack's knee rows damp against it, not against still.
BEND_RATE = 59
#: And, kept, the tread of a floor told her each swing lands on (`stride.footholds`), -1 none.
CELL = slice(60, 62)
ASK = 62

#: A unit of each mode on her: her centre of mass down, m - to her ankles' stroke, her trunk
#: upright over her soles: asked 0.15 m she came down 0.117 and stood there; her trunk's
#: pitch, rad; a sole clear of the floor - at 3 cm her walk fell within 20 s in 10 of 12
#: settings, at 2 cm in 5 of 12 (45 s each). Her legs' length is her figure's.
AXIAL_M, PITCH_RAD, CLEAR_M = 0.115, 0.5, 0.02
#: And of her turns, rad: her pelvis's list, a hip up, and its turn, a hip forward; a
#: shoulder's reach, the other as far back.
LIST_RAD, TURN_RAD, ARM_RAD = 0.15, 0.2, 0.35
LEG_M = THIGH + SHANK
#: Each mode's hand on a limb's place: place = HANDS . (unit x mode), mode = HANDS . place / 2.
HANDS = np.zeros((N, 2, 3))
HANDS[AXIAL, :, UP] = 1.0
HANDS[LATERAL, :, LEFT] = -1.0
HANDS[SAGITTAL, :, ALONG] = -1.0
HANDS[DRIVE, :, ALONG] = -LIMBS
HANDS[LIFT, :, UP] = LIMBS
HANDS[WIDTH, :, LEFT] = LIMBS
HOT = np.eye(N)
#: At rest: along the floor the capture point's law, its ZMP REST_K / omega past her capture
#: point from where it is asked - a spring REST_K omega and a damper omega + REST_K over her
#: mass; her height's, her turns' and a swinging sole's as the loop has them, 1/s^2 and 1/s.
REST_K = 2.0
#: A limb in the air at rest, 1/s^2 and 1/s. One sole bears what moves it: her weight on a
#: sole 9 cm wide turns her 13 N m at most, a limb thrown across at 4 m/s^2 or lifted at 8.
#: Held at the loop's own 1 600 and 80, a sole 6 cm off its width asked 163 m/s^2 across,
#: the loop planned -10 - the lift's 23 m/s^2 took the moment - and threw her neck at three
#: times its clamp for the rest.
LIMB_KD = (400.0, 40.0)
#: Her ZMP's polygon: none, her support's, the dynamic footprint. No limit on her yaw momentum.
NONE, SUPPORT, FOOTPRINT, FREE = 0.0, 1.0, 2.0, 1e9
#: Her elbow's bend, of its shoulder's reach.
ELBOW_BY = 0.4


def _turns(y, rot):
    """The pelvis's and the trunk's pitch along her way, roll - the left up - and turn to her
    left, rad, and their rates: (3, 2) each."""
    w, x, j, k = y[QUAT].reshape(2, 4).T
    left = np.stack([1.0 - 2.0 * (j * j + k * k), 2.0 * (x * j + w * k), 2.0 * (x * k - w * j)], axis=1)
    up = np.stack([2.0 * (x * j - w * k), 1.0 - 2.0 * (x * x + k * k), 2.0 * (j * k + w * x)], axis=1)
    on = np.stack([2.0 * (x * k + w * j), 2.0 * (j * k - w * x), 1.0 - 2.0 * (x * x + j * j)], axis=1)
    spin = y[SPIN].reshape(2, 3)
    return (np.stack([np.arctan2(up @ rot[ALONG], up[:, 1]), np.arctan2(left[:, 1], left @ rot[LEFT]),
                      np.arctan2(on @ rot[LEFT], on @ rot[ALONG])]),
            np.stack([spin @ rot[LEFT], spin @ rot[ALONG], spin @ rot[UP]]))


def _turned(yaw, pitch, roll):
    """Quaternions (n, 4): a turn about up, then a pitch about the left, then a roll about
    the way on, rad."""
    (cy, cp, cr), (sy, sp, sr) = np.cos(0.5 * np.stack([yaw, pitch, roll])), np.sin(0.5 * np.stack([yaw, pitch, roll]))
    return np.stack([cy * cp * cr + sy * sp * sr, cy * sp * cr + sy * cp * sr,
                     sy * cp * cr - cy * sp * sr, cy * cp * sr - sy * sp * cr], axis=1)


def frame(y, mass, limb, trunk, arm, knee, dt):
    """Her statics of what is sensed standing: `mass` hers, a `limb`'s, her `trunk`'s and an
    `arm`'s, kg; `knee` her knees' bend, rad; `dt` s from an ask to the next."""
    q = y[QUAT]
    yaw = 2.0 * np.arctan2(q[2], q[0])
    s, c = np.sin(yaw), np.cos(yaw)
    rot = np.array([[s, 0.0, c], [c, 0.0, -s], [0.0, 1.0, 0.0]])
    soles = y[SOLE].reshape(2, 3)
    legs = y[HIP].reshape(2, 3) - soles + leap.MIDDLE * rot[ALONG] - ANKLE_H * rot[UP]
    fr = np.zeros(FRAME)
    fr[ROT] = rot.ravel()
    fr[FLOOR], fr[FLOOR_AT] = soles[:, 1].mean(), soles[:, [0, 2]].mean(axis=0)
    fr[H0] = y[COM][1] - fr[FLOOR]
    fr[W0] = 0.5 * abs((soles[0] - soles[1]) @ rot[LEFT])
    fr[OMEGA], fr[WEIGHT], fr[KNEE], fr[DT] = np.sqrt(G / fr[H0]), mass * G, knee, dt
    fr[PITCH0] = _turns(y, rot)[0][0]
    fr[HEADING], fr[ARM0] = yaw, y[ARM]
    fr[REACH0], fr[HIPS_BY] = np.linalg.norm(legs, axis=1).mean(), mass / (mass - limb)
    fr[UNIT] = (AXIAL_M, fr[W0], STRIDE_M, PITCH_RAD, HALF_M, 0.5 * CLEAR_M, fr[W0], LIST_RAD, TURN_RAD,
                ARM_RAD, ROCK_RAD)
    fr[Z0] = HOT[AXIAL] * fr[H0] / AXIAL_M - HOT[WIDTH]
    fr[MASS] = mass, mass, limb, trunk, mass, limb, limb, trunk, trunk, arm, limb
    return fr


def rest(fr):
    """Her rest vector (`ortho`'s parameters): every target 0, each mode's spring and damper,
    her rhythm off at 1 Hz, her ZMP in her support polygon, her yaw momentum free."""
    floor = REST_K * fr[OMEGA], fr[OMEGA] + REST_K
    p = np.zeros(ortho.P)
    for part, kd in ((latent.K, 0), (latent.D, 1)):
        p[part] = fr[MASS] * np.array([HEIGHT_KD[kd], floor[kd], LIMB_KD[kd], TURN_KD[kd], floor[kd],
                                       LIMB_KD[kd], LIMB_KD[kd], TURN_KD[kd], TURN_KD[kd],
                                       POSTURE_KD[kd], TURN_KD[kd]])
    p[latent.HZ], p[latent.ZMP], p[latent.YAW] = 1.0, SUPPORT, FREE
    return p


def _legs(y, rot):
    """Each leg, its hip from its ankle in her way's frame (2, 3), its length and that's
    rate: her hips go as her centre of mass does, an ankle as its sole."""
    tilt = y[TILT]
    spoke = np.stack([np.sin(tilt), 0.0 * tilt, np.cos(tilt)], axis=1)
    inward = np.stack([np.cos(tilt), 0.0 * tilt, -np.sin(tilt)], axis=1)
    leg = (y[HIP].reshape(2, 3) - y[SOLE].reshape(2, 3)) @ rot.T - ANKLE_H * spoke + leap.MIDDLE * inward
    long = np.linalg.norm(leg, axis=1)
    return leg, long, ((y[VCOM] - y[VSOLE].reshape(2, 3)) @ rot.T * leg).sum(axis=1) / long


def under(y, fr, stance):
    """(the floor under each sole's middle, m; its rise a metre along her way): a plane through
    the floor's height at its point, pitched as the soles in `stance` stand on it - level under
    half `stride.EDGE_RAD`. Read level, each swing up a 10 deg ramp came down on it early, 0.05
    to 0.09 m behind her centre of mass, and she ran on to 1.7 m/s and fell at 5.1 s."""
    bears = (stance + TINY) / (stance.sum() + 2.0 * TINY)
    rise = bears @ -np.tan(stride.pitched(fr[GROUND]))
    return fr[FLOOR] + rise * ((y[SOLE].reshape(2, 3)[:, [0, 2]] - fr[FLOOR_AT]) @ fr[ROT][[0, 2]]), rise


def shortened(legs, fr, stance):
    """Her axial compression and its rate: how much shorter than she stood on the legs in
    `stance` are - none in it, both. Each by the load it bears (not kept): the chairs fell 3 of
    3 laid, snow 2, the run leap lost its flight and the viewport's errands fell."""
    w = (stance + TINY) / (stance.sum() + 2.0 * TINY)
    return ((fr[REACH0] - w @ legs[1]) / (AXIAL_M * fr[HIPS_BY]), -(w @ legs[2]) / (AXIAL_M * fr[HIPS_BY]))


def reading(y, fr, stance):
    """(her modes, their rates, her legs, her turns and their rates) as she is, the soles in
    `stance` her standing ones."""
    rot = fr[ROT].reshape(3, 3)
    place = (y[SOLE].reshape(2, 3) - y[COM]) @ rot.T
    speed = (y[VSOLE].reshape(2, 3) - y[VCOM]) @ rot.T
    turn, spun = _turns(y, rot)
    legs = _legs(y, rot)
    axial = shortened(legs, fr, stance)
    z = 0.5 * np.einsum('nia,ia->n', HANDS, place) / fr[UNIT] + fr[Z0]
    zd = 0.5 * np.einsum('nia,ia->n', HANDS, speed) / fr[UNIT]
    z += HOT[AXIAL] * (axial[0] - z[AXIAL])
    zd += HOT[AXIAL] * (axial[1] - zd[AXIAL])
    angle = (HOT[PITCH] * (turn[0, 1] - fr[PITCH0][1]) + HOT[LIST] * turn[1, 0] + HOT[TURN] * turn[2, 0]
             + HOT[ROCK] * 0.5 * (LIMBS @ y[TILT]) + HOT[ARMS] * 0.5 * (LIMBS @ (y[ARM] - fr[ARM0])))
    rate = (HOT[PITCH] * spun[0, 1] + HOT[LIST] * spun[1, 0] + HOT[TURN] * spun[2, 0]
            + HOT[ROCK] * 0.5 * (LIMBS @ y[TILT_RATE]))
    return z + angle / fr[UNIT], zd + rate / fr[UNIT], legs, turn, spun


def read(y, fr, stance):
    """(her modes, their rates) as she is (`ortho.MODES`' order), in the units they are asked
    in; `stance` the soles that stand (the ask's STANCE)."""
    return reading(y, fr, stance)[:2]


def crouch(names):
    """How each driven joint (`names`) goes a radian of her knees' bend as she comes straight
    down: the knee by it, its hip and its ankle by half (`machine.terms`)."""
    bend = np.zeros(len(names))
    for limb in (0, 1):
        for part, term, by in (('hip', 'flexion', 0.5), ('knee', 'flexion', 1.0), ('ankle', 'dorsiflexion', 0.5)):
            kind, sign, _low, _high = terms.TERMS[(part, term)]
            bend[names.index(terms.joint(limb, kind))] = sign * by
    return bend


def swing(names):
    """How each driven joint (`names`) goes a radian of her arms' swing: her first shoulder
    forward by it and the other back, each elbow bent ELBOW_BY of its shoulder's way."""
    out = np.zeros(len(names))
    for limb in (0, 1):
        for part, by in (('shoulder', 1.0), ('elbow', ELBOW_BY)):
            kind, sign, _low, _high = terms.TERMS[(part, 'flexion')]
            out[names.index(terms.joint(limb, kind))] = sign * by * LIMBS[limb]
    return out


def sensed(s, loads, y):
    """`y` filled of `wbc.sense`'s `s` - with her hips' joints (`hips`, world) and her
    shoulders' reach forward (`arms`, rad) beside it - and her soles' `loads`, N."""
    feet, turns, head = s['feet'], s['turns'], s['head']
    y[COM], y[VCOM], y[LOAD], y[MOMENT] = s['com'], s['vcom'], loads, s['k']
    y[SOLE] = np.concatenate([f['sole'] for f in feet])
    y[VSOLE] = np.concatenate([f['J6'][3:] @ s['v'] for f in feet])
    y[QUAT] = np.concatenate([t['quat'] for t in turns])
    y[SPIN] = np.concatenate([t['w'] for t in turns])
    y[LOWEST] = [f['pts'][:, 1].min() for f in feet]
    y[HIP], y[ARM] = np.ravel(s['hips']), s['arms']
    y[PROP], y[PROP_LO], y[PROP_HI] = s.get('prop', (0.0, 0.0, 0.0))
    y[TILT] = np.arcsin(clip(np.array([-f['R'][1, 2] for f in feet]), -1.0, 1.0))
    y[TILT_RATE] = [(f['J6'][:3] @ s['v']) @ f['R'][:, 0] for f in feet]
    y[VHEAD], y[HQUAT], y[HSPIN] = head['v'], head['quat'], head['w']
    return y


def poses(a, y, fr, posture, bend, arms, knees, clear=0.0):
    """(each sole's place asked as it swings (2, 3), her posture (driven), her turns (pelvis,
    trunk) and a swinging sole's quaternion (2, 4)) of `a`, the stack's ask's arithmetic
    (`given`): `posture` her driven joints standing, rad; `bend` how each goes with her knees'
    (`crouch`), `arms` with her arms' swing (`swing`); `knees` each knee's index among them."""
    soles, speeds = y[SOLE].reshape(2, 3), y[VSOLE].reshape(2, 3)
    q0, q2 = y[QUAT][0], y[QUAT][2]
    lat = np.array([q0 * q0 - q2 * q2, 0.0, -2.0 * q0 * q2]) / np.maximum(q0 * q0 + q2 * q2, 1e-12)
    signs = np.array([1.0, -1.0])
    dl = signs * ((soles + 0.05 * speeds - soles[::-1]) @ lat)
    sw_at = soles + (a[STANCE][::-1] * signs * np.maximum(clear - dl, 0.0))[:, None] * lat
    pose = posture + bend * a[BEND] + arms * a[SWUNG]
    pose[list(knees)] += a[STANCE] * a[PREFOLD]
    turns = _turned(fr[HEADING] + a[HEADED], a[WANT], a[ROLLED])
    tips = _turned(fr[HEADING] + 0.0 * a[TIPPED], a[TIPPED], 0.0 * a[TIPPED])
    return sw_at, pose, turns, tips


def given(a, y, posed, knees, bend):
    """The stack's ask (`wbc.step`) of `a` and its poses (`posed`, `poses`), by her stance: a sole
    that does not stand is asked its acceleration where it is; no turn is asked with none standing;
    each knee's posture goes at its share (`bend`, `crouch`) of her knees' bend's rate."""
    (sw_at, pose, turns, tips), speeds, accs = posed, y[VSOLE].reshape(2, 3), a[SOLE_ACC].reshape(2, 3)
    free = np.flatnonzero(a[STANCE] < 0.5)
    folded = np.flatnonzero((a[STANCE] < 0.5) & (a[FOLDS] > 0.5))
    return {'stance': tuple(a[STANCE] > 0.5), 'com_acc': a[COM_ACC].copy(),
            'turns': tuple(turns[:2 * int(a[TURNED])]), 'posture': pose, 'kdot': a[KDOT].copy(),
            'knee_rate': tuple(float(bend[k] * a[BEND_RATE]) for k in knees),
            'settle': a[SETTLE].reshape(2, 6).copy(), 'edge': tuple(a[EDGE]), 'strides': float(a[STRIDES]),
            'borne': float(a[BORNE]),
            'swing': {int(i): (sw_at[i].copy(), speeds[i].copy(), accs[i].copy(), tips[i]) for i in free},
            'fold': {int(i): (pose[knees[i]] + a[FOLD][i], a[FOLD_RATE][i], 0.0) for i in folded}}
