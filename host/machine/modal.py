"""Her latent modes on her body: what is sensed read as modes, their springs the loop's ask.

    y = modal.sensed(s, loads, y)                       # flat, of `wbc.sense`'s s and her soles' N
    fr = modal.frame(y, mass, limb, trunk, knee, dt)    # her statics, once
    rest = modal.rest(fr)                               # her rest vector (`latent.state`)
    a = modal.ask(out, y, fr, a)                        # flat: `latent.step`'s out, rendered
    ask = modal.given(a, y, fr, posture, bend, knees)   # the stack's ask (`wbc.step`)
    z, rate = modal.read(y, fr)                         # her modes as she is, and their rates

Flat arrays and arithmetic on them; a choice is a mask, a limit a minimum or a maximum, a
change a blend. A limb's place is its sole's from her centre of mass, along her way, to her
left and up. Her modes are those places' sums and differences (HANDS): of the sum, her centre
of mass over her soles' middle - its height, its place across and along; of the difference,
her limbs against each other - the drive along, the lift, the width. Her trunk's pitch is the
last. Each mode is a mass on a spring and a damper to its reference: the sum's on her whole
mass, the difference's on a limb's, the pitch's on her trunk's.

What the sum's springs ask is her centre of mass's acceleration, its ZMP brought into her
soles' polygon first - and, the polygon her support's, to where her capture point stays in it
(`maintain_zmp`) -, up no harder than her legs throw her. A sole that does not stand is sprung
to the place her modes give it from the soles that do - none standing, from her centre of
mass as it falls -, each axis on the difference's impedance: the drive's along, the width's
across, the lift's up. A sole stands that touches the floor (`flight.standing`) and is not asked up; asked
up, it leaves when her weight is off it. The limb her drive carries forward is asked clear of the floor.
The loop finds the torques and clips them to her drives (`machine.wbc`).
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np  # noqa: E402

from machine import flight, latent, ortho, terms  # noqa: E402
from machine.balance import HEIGHT_KD  # noqa: E402
from machine.figure import SOLE_HALF  # noqa: E402
from machine.gait import BALL, HEEL, SHANK, THIGH  # noqa: E402
from machine.wbc import MOMENTUM_K, TURN_KD  # noqa: E402

G = 9.81
N = ortho.N
AXIAL, LATERAL, DRIVE, PITCH, SAGITTAL, LIFT, WIDTH = (ortho.MODES.index(m) for m in (
    'axial_compression', 'lateral_shift', 'sagittal_drive', 'pitch_attitude', 'sagittal_shift',
    'limb_lift', 'stance_width'))
#: A limb's place, her way's frame: along, to her left, up. Her limbs' mirrors, the first her left.
ALONG, LEFT, UP = 0, 1, 2
LIMBS = np.array([1.0, -1.0])

#: What is sensed, world frame: her centre of mass and its speed; each sole's middle and its
#: speed; each sole's load, N; the pelvis's and the trunk's quaternions and spins; her angular
#: momentum about her centre of mass; each sole's lowest corner's height.
COM, VCOM, SOLE, VSOLE = slice(0, 3), slice(3, 6), slice(6, 12), slice(12, 18)
LOAD, QUAT, SPIN, MOMENT = slice(18, 20), slice(20, 28), slice(28, 34), slice(34, 37)
LOWEST = slice(37, 39)
SENSED = 39
#: Her statics: her way's frame (3 x 3: along, left, up in the world); her centre of mass over
#: her soles and her soles' half width as she stood; the floor; omega; her weight, N; her knees'
#: bend standing, rad; the pelvis's and the trunk's pitch then; a level sole's quaternion; each
#: mode's unit on her (m, the pitch's rad), its reading where its coordinate is 0, the mass it
#: moves, kg.
ROT, H0, W0, FLOOR, OMEGA, WEIGHT, KNEE = slice(0, 9), 9, 10, 11, 12, 13, 14
PITCH0, LEVEL, DT = slice(15, 17), slice(17, 21), 21
UNIT, Z0, MASS = (slice(22 + i * N, 22 + (i + 1) * N) for i in range(3))
FRAME = 22 + 3 * N
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
ASK = 23

#: A unit of each mode on her: her centre of mass down, m - to her ankles' stroke, her trunk
#: upright over her soles: asked 0.15 m she came down 0.117 and stood there; a limb's place
#: from her centre of mass along her way; her trunk's pitch, rad; a sole clear of the floor -
#: at 3 cm her walk fell within 20 s in 10 of 12 settings, at 2 cm in 5 of 12 (45 s each).
#: Her soles' half length, and her legs' length, are her figure's.
AXIAL_M, STRIDE_M, PITCH_RAD, CLEAR_M = 0.115, 0.3, 0.5, 0.02
HALF_M, LEG_M = (BALL + HEEL) / 2.0, THIGH + SHANK
#: Each mode's hand on a limb's place: place = HANDS . (unit x mode), mode = HANDS . place / 2.
HANDS = np.zeros((N, 2, 3))
HANDS[AXIAL, :, UP] = 1.0
HANDS[LATERAL, :, LEFT] = -1.0
HANDS[SAGITTAL, :, ALONG] = -1.0
HANDS[DRIVE, :, ALONG] = -LIMBS
HANDS[LIFT, :, UP] = LIMBS
HANDS[WIDTH, :, LEFT] = LIMBS
#: The mode whose impedance holds a sole that does not stand, an axis each.
FREE_BY = np.array([DRIVE, WIDTH, LIFT])
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
#: Her ZMP kept this far inside a sole's edge, m, and her capture point let toward an edge no
#: faster than its room there over CAPTURE_S: on the ZMP's polygon alone a spring of 3 000 N/m
#: took her centre of mass a quarter of the way past her sole's middle and she fell over it; 5
#: mm in, the edge was past where the loop's own margin lets her soles press.
MARGIN_M, CAPTURE_S = 0.018, 0.2
EXTENT = np.array([HALF_M - MARGIN_M, SOLE_HALF - MARGIN_M])
#: A sole stands by `flight.standing`, unless it is asked up: over LIFT_EPS of the lift's
#: unit, it leaves the floor once her ZMP as her springs ask it lies in the other sole's
#: box, GATE_M past its edge at most. Let go by its own load, it never went: the loop keeps
#: a third of her weight on a sole her ZMP is 0.6 of the way off, a twelfth at its furthest.
LIFT_EPS, GATE_M = 0.02, 0.005
#: And only while she sways as asked, within ON of her lateral shift and of its rate: a sway
#: asked past what her soles' width gives (0.35 at 1.1 Hz: her ZMP 1.9 of the way to a sole)
#: crossed the other's box off its beat, a sole left and she went over its side in 3 s.
ON = (0.1, 1.0)
#: While her drive has amplitude over STIRS, a sole is asked clear of the floor as long as
#: her ZMP, as her sway across asks it, lies in the other sole's box; its lift's whole way
#: over no less than SPAN of that sway. Its knee folds as its leg is that much shorter under
#: its hip: folded 0.6 rad whatever was asked, the sole went up 13 cm where 4 were asked and
#: her centre of mass 6 cm with it.
STIRS, SPAN = 0.02, 0.05
#: A sole is asked up once that ZMP is DEEP_M inside the other's box: asked up as it crossed
#: the edge, the gate opened late - at her sway's furthest, her centre of mass 0.18 short
#: of it -, the sole flew 6 cm up to catch its turn and she swayed 0.42 where 0.28 was asked.
DEEP_M = 0.01
#: Her going: her soles' speed along her way, meaned over GOING_S; her centre of mass asked
#: no more than AHEAD_M_S faster or slower.
GOING_S, AHEAD_M_S = 1.0, 0.15
#: Her posture's knees are bent no more than this ahead of her height, of the axial unit.
AHEAD = 0.15
#: Her yaw momentum past its limit is bled this fast, 1/s.
YAW_K = 50.0
#: Her centre of mass is asked up no harder than this, m/s^2 (`tools/sim/hop.py`: thrown at
#: 10 to 18 she flew and stood): the stiffest spring said asked 630, the loop gave 33 with
#: every drive it has, and her trunk went back 0.22 rad in the 45 ms of it.
THROW_M_S2 = 14.0
#: A sole out of the polygon; a sway of nothing; her height no lower than this for omega.
BIG, TINY, LOW_M = 1e3, 1e-9, 0.3
FALLS, UPWARD = np.array([0.0, 0.0, -G]), np.array([0.0, 1.0, 0.0])
#: The axes a sole in the air is held on against the soles that stand: across and up.
BETWEEN, HOT3 = np.array([0.0, 1.0, 1.0]), np.eye(3)
HOT = np.eye(N)


def _pitch(y, rot):
    """The pelvis's and the trunk's pitch along her way, rad, and its rate."""
    q = y[QUAT].reshape(2, 4)
    up = np.stack([2.0 * (q[:, 1] * q[:, 2] - q[:, 0] * q[:, 3]),
                   1.0 - 2.0 * (q[:, 1] ** 2 + q[:, 3] ** 2),
                   2.0 * (q[:, 2] * q[:, 3] + q[:, 0] * q[:, 1])], axis=1)
    return np.arctan2(up @ rot[ALONG], up[:, 1]), y[SPIN].reshape(2, 3) @ rot[LEFT]


def frame(y, mass, limb, trunk, knee, dt):
    """Her statics of what is sensed standing: `mass` hers, a `limb`'s and her `trunk`'s, kg;
    `knee` her knees' bend, rad; `dt` s from an ask to the next."""
    q = y[QUAT]
    yaw = 2.0 * np.arctan2(q[2], q[0])
    s, c = np.sin(yaw), np.cos(yaw)
    rot = np.array([[s, 0.0, c], [c, 0.0, -s], [0.0, 1.0, 0.0]])
    soles = y[SOLE].reshape(2, 3)
    fr = np.zeros(FRAME)
    fr[ROT] = rot.ravel()
    fr[FLOOR] = soles[:, 1].mean()
    fr[H0] = y[COM][1] - fr[FLOOR]
    fr[W0] = 0.5 * abs((soles[0] - soles[1]) @ rot[LEFT])
    fr[OMEGA], fr[WEIGHT], fr[KNEE], fr[DT] = np.sqrt(G / fr[H0]), mass * G, knee, dt
    fr[PITCH0] = _pitch(y, rot)[0]
    fr[LEVEL] = np.cos(0.5 * yaw), 0.0, np.sin(0.5 * yaw), 0.0
    fr[UNIT] = AXIAL_M, fr[W0], STRIDE_M, PITCH_RAD, HALF_M, 0.5 * CLEAR_M, fr[W0]
    fr[Z0] = HOT[AXIAL] * fr[H0] / AXIAL_M - HOT[WIDTH]
    fr[MASS] = mass, mass, limb, trunk, mass, limb, limb
    return fr


def rest(fr):
    """Her rest vector (`ortho`'s parameters): every target 0, each mode's spring and damper,
    her rhythm off at 1 Hz, her ZMP in her support polygon, her yaw momentum free."""
    floor = REST_K * fr[OMEGA], fr[OMEGA] + REST_K
    p = np.zeros(ortho.P)
    for part, kd in ((latent.K, 0), (latent.D, 1)):
        p[part] = fr[MASS] * np.array([HEIGHT_KD[kd], floor[kd], LIMB_KD[kd], TURN_KD[kd], floor[kd],
                                       LIMB_KD[kd], LIMB_KD[kd]])
    p[latent.HZ], p[latent.ZMP], p[latent.YAW] = 1.0, SUPPORT, FREE
    return p


def _limbs(y, rot):
    """Each limb's place from her centre of mass and its speed, her way's frame (2, 3)."""
    return ((y[SOLE].reshape(2, 3) - y[COM]) @ rot.T, (y[VSOLE].reshape(2, 3) - y[VCOM]) @ rot.T)


def read(y, fr):
    """(her modes, their rates) as she is (`ortho.MODES`' order), in the units they are asked in."""
    rot = fr[ROT].reshape(3, 3)
    place, speed = _limbs(y, rot)
    pitch, rate = _pitch(y, rot)
    z = 0.5 * np.einsum('nia,ia->n', HANDS, place) / fr[UNIT] + fr[Z0]
    zd = 0.5 * np.einsum('nia,ia->n', HANDS, speed) / fr[UNIT]
    turned = HOT[PITCH] / PITCH_RAD
    return z + turned * (pitch[1] - fr[PITCH0][1]), zd + turned * rate[1]


def ask(out, y, fr, a):
    """`a`, the loop's ask this pass: `out` (`latent.step`) rendered on her as she is sensed."""
    rot = fr[ROT].reshape(3, 3)
    z, zd = read(y, fr)
    turns = out[latent.TURNS]
    c, v = rot @ y[COM], rot @ y[VCOM]
    om = np.sqrt(G / np.maximum(c[UP] - fr[FLOOR], LOW_M))
    # a sole is asked clear of the floor while her ZMP, as her sway across asks it, lies in
    # the other: past `inner` of its furthest
    sway, psi = out[latent.SWAY][LATERAL], out[latent.PSI][LATERAL]
    x, xd = np.sign(sway) * np.sin(psi), np.sign(sway) * np.cos(psi) * turns
    inner = np.clip((1.0 - (EXTENT[1] - DEEP_M) / fr[W0])
                    / np.maximum(np.abs(sway) * (1.0 + (turns / om) ** 2), TINY), 0.0, 1.0)
    span = np.maximum(1.0 - inner, SPAN)
    m = np.maximum(np.abs(x) - inner, 0.0) / span
    clear = -np.clip(np.abs(out[latent.SWAY][DRIVE]) / STIRS, 0.0, 1.0) * np.array(
        [np.sign(x) * m * m, 2.0 * m * xd / span,
         2.0 * (np.sign(x) * (m > 0.0) * xd * xd / span - m * turns * turns * x) / span])
    ref = out[latent.REF] + HOT[LIFT] * clear[0]
    rate = out[latent.RATE] + HOT[LIFT] * clear[1]
    acc = out[latent.ACC] + HOT[LIFT] * clear[2]
    kp, kd = out[latent.KS] / fr[MASS], out[latent.DS] / fr[MASS]
    # along her way her centre of mass is damped to the speed her drive makes of its steps
    # - the stride a turn, as far as her soles are let go -, not to her soles' middle: damped
    # to that she stopped between steps and each sole waited for her. And no further than
    # AHEAD_M_S from her going, the speed her soles have made of late: asked 0.48 m/s under
    # a sway her soles refused, she leant to her toes' edge standing and went over them;
    # damped to her going alone she never set out.
    cruise = (2.0 / np.pi * STRIDE_M * out[latent.SWAY][DRIVE] * turns * np.sqrt(1.0 - inner * inner)
              * np.sin(psi - out[latent.PSI][DRIVE]) * np.sign(sway))
    # - and that far only while she sways as asked, none of it past twice ON: her sway
    # refused, her soles do not step and nothing may draw her on
    ahead = AHEAD_M_S * np.clip(2.0 - np.abs(out[latent.REF][LATERAL] - z[LATERAL]) / ON[0], 0.0, 1.0)
    cruise = np.clip(cruise, a[GOING] - ahead, a[GOING] + ahead)
    zd = zd + HOT[SAGITTAL] * ((v[ALONG] - cruise) / HALF_M - zd[SAGITTAL])
    zdd = acc + kp * (ref - z) + kd * (rate - zd)
    # her limbs' places as her modes ask them; their sum's acceleration is her centre of
    # mass's, the other way; a sole that does not stand on the limbs' own impedance
    hands = HANDS * fr[UNIT][:, None, None]
    both = np.einsum('nia,n->ia', hands, zdd).mean(axis=0)
    # her ZMP as her springs ask it, into her soles' polygon
    soles, loads, over = y[SOLE].reshape(2, 3), y[LOAD], y[LOWEST] - fr[FLOOR]
    held = np.clip(out[latent.HELD], 0.0, 1.0)
    foot = np.clip(out[latent.HELD] - 1.0, 0.0, 1.0)
    flat = (soles @ rot.T)[:, :2]
    low, high = flat - EXTENT, flat + EXTENT
    lo, hi = low.min(axis=0), high.max(axis=0)
    xi = c[:2] + v[:2] / om
    zmp = c[:2] + both[:2] / om ** 2
    plain = np.minimum(np.maximum(zmp, lo), hi)
    viable = np.minimum(np.maximum(zmp, xi - (hi - xi) / (om * CAPTURE_S)), xi + (xi - lo) / (om * CAPTURE_S))
    viable = np.minimum(np.maximum(viable, lo), hi)
    zmp = zmp + held * (plain + (1.0 - foot) * (viable - plain) - zmp)
    # which soles stand: one asked up leaves the floor once that ZMP lies in the other
    up = LIMBS * ref[LIFT]
    within = ((zmp >= low - GATE_M) & (zmp <= high + GATE_M)).all(axis=1)
    swaying = (np.abs(ref[LATERAL] - z[LATERAL]) < ON[0]) & (np.abs(rate[LATERAL] - zd[LATERAL]) < ON[1])
    lifting = (up > LIFT_EPS) & ((within[::-1] & swaying) | (over > flight.TOUCH_M))
    stance = np.array(flight.standing(~lifting, loads, over, y[VCOM][1]), float)
    grounded = stance.max()
    # and into the dynamic footprint, the soles that stand - her capture point kept in it
    # too while both do: let go there, a drive asked under a sway her soles refused leant
    # her past her toes standing
    out_of = BIG * foot * (1.0 - stance)[:, None]
    lo, hi = (low + out_of).min(axis=0), (high - out_of).max(axis=0)
    plain = np.minimum(np.maximum(zmp, lo), hi)
    viable = np.minimum(np.maximum(zmp, xi - (hi - xi) / (om * CAPTURE_S)), xi + (xi - lo) / (om * CAPTURE_S))
    viable = np.minimum(np.maximum(viable, lo), hi)
    zmp = zmp + held * (plain + foot * stance.min() * (viable - plain) - zmp)
    along = om ** 2 * (c[:2] - zmp)
    com = (grounded * np.array([along[0], along[1], np.clip(-both[UP], -G, THROW_M_S2)])
           + (1.0 - grounded) * FALLS)
    # a sole that does not stand, on the limbs' own impedance. Across and up against the
    # soles that do: what is asked of it less what is asked of them. Along against her
    # centre of mass as it goes, led by her speed past her cruise over omega: against the
    # standing sole she ran ahead of her steps at 0.36 m/s and went over forward. None
    # standing, against her centre of mass as it falls.
    # With none standing a limb's place is read in her pelvis's own frame, turning with her:
    # held level and plumb under her in the world, her legs kept none of her spin and her
    # trunk took it all - 0.7 rad forward in 0.15 s off the floor.
    q = y[QUAT][:4]
    hers = np.array([[2.0 * (q[1] * q[3] + q[0] * q[2]), 2.0 * (q[2] * q[3] - q[0] * q[1]),
                      1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2)],
                     [1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2), 2.0 * (q[1] * q[2] + q[0] * q[3]),
                      2.0 * (q[1] * q[3] - q[0] * q[2])],
                     [2.0 * (q[1] * q[2] - q[0] * q[3]), 1.0 - 2.0 * (q[1] ** 2 + q[3] ** 2),
                      2.0 * (q[2] * q[3] + q[0] * q[1])]])
    frame = grounded * rot + (1.0 - grounded) * hers
    arm = soles - y[COM]
    place = arm @ frame.T
    speed = (y[VSOLE].reshape(2, 3) - y[VCOM] - (1.0 - grounded) * np.cross(y[SPIN][:3], arm)) @ frame.T
    # a sole in the air is asked no further from her than her legs reach standing
    reach = ref - HOT[AXIAL] * np.minimum(ref[AXIAL], 0.0)
    off = np.stack([np.einsum('nia,n->ia', hands, reach - fr[Z0]) - place,
                    np.einsum('nia,n->ia', hands, rate) - speed, np.einsum('nia,n->ia', hands, acc)])
    between = grounded * BETWEEN
    off = off - between * (((stance + TINY) / (stance.sum() + 2.0 * TINY)) @ off)[:, None, :]
    lead = HOT3[ALONG] * grounded * (v[ALONG] - cruise) / om
    free = off[2] + kp[FREE_BY] * (off[0] + lead) + kd[FREE_BY] * off[1]
    a[GOING] += fr[DT] / GOING_S * ((y[VSOLE].reshape(2, 3) @ rot[ALONG]).mean() - a[GOING])
    a[STANCE] = stance
    a[COM_ACC] = com @ rot
    a[SOLE_ACC] = (free @ frame + ((1.0 - between) * com) @ rot).ravel()
    # her trunk's and her pelvis's pitch, each on the pitch's spring; the loop's own turn law
    # asked for what gives that
    th, thd = _pitch(y, rot)
    thdd = (PITCH_RAD * acc[PITCH] + kp[PITCH] * (fr[PITCH0] + PITCH_RAD * ref[PITCH] - th)
            + kd[PITCH] * (PITCH_RAD * rate[PITCH] - thd))
    a[WANT] = th + (thdd + TURN_KD[1] * thd) / TURN_KD[0]
    a[TURNED] = grounded * (kp[PITCH] + kd[PITCH] > 0.0)
    # her posture's knees bent as her height is, AHEAD of it toward what is asked: bent as
    # asked, a throw's step threw her knees straight at 100 rad/s^2 and her over forward,
    # 8 kg m^2/s about her; bent as she is, a crouch asked 0.6 stopped at 0.56. Her hips
    # come down further than her centre of mass: her limbs' own mass comes half as far
    bent = z[AXIAL] + np.clip(ref[AXIAL] - z[AXIAL], -AHEAD, AHEAD)
    hips = AXIAL_M * bent * fr[MASS][AXIAL] / (fr[MASS][AXIAL] - fr[MASS][DRIVE])
    a[BEND] = 2.0 * np.arccos(np.clip(np.cos(0.5 * fr[KNEE]) - hips / LEG_M, -1.0, 1.0)) - fr[KNEE]
    # a lifted sole's knee, folded as its leg is shorter by the lift under its hip
    short = np.clip(np.cos(0.5 * (fr[KNEE] + a[BEND])) - CLEAR_M * np.clip(up, 0.0, None) / LEG_M, -1.0, 1.0)
    a[FOLD] = 2.0 * np.arccos(short) - fr[KNEE] - a[BEND]
    a[FOLD_RATE] = (2.0 * CLEAR_M * LIMBS * rate[LIFT] * (up > 0.0) / LEG_M
                    / np.sqrt(np.maximum(1.0 - short * short, TINY)))
    k = y[MOMENT]
    a[KDOT] = -MOMENTUM_K * k - UPWARD * YAW_K * (k[1] - np.clip(k[1], -out[latent.YAWED], out[latent.YAWED]))
    return a


def crouch(names):
    """How each driven joint (`names`) goes a radian of her knees' bend as she comes straight
    down: the knee by it, its hip and its ankle by half (`machine.terms`)."""
    bend = np.zeros(len(names))
    for limb in (0, 1):
        for part, term, by in (('hip', 'flexion', 0.5), ('knee', 'flexion', 1.0), ('ankle', 'dorsiflexion', 0.5)):
            kind, sign, _low, _high = terms.TERMS[(part, term)]
            bend[names.index(terms.joint(limb, kind))] = sign * by
    return bend


def sensed(s, loads, y):
    """`y` filled of `wbc.sense`'s `s` and her soles' `loads`, N."""
    feet, turns = s['feet'], s['turns']
    y[COM], y[VCOM], y[LOAD], y[MOMENT] = s['com'], s['vcom'], loads, s['k']
    y[SOLE] = np.concatenate([f['sole'] for f in feet])
    y[VSOLE] = np.concatenate([f['J6'][3:] @ s['v'] for f in feet])
    y[QUAT] = np.concatenate([t['quat'] for t in turns])
    y[SPIN] = np.concatenate([t['w'] for t in turns])
    y[LOWEST] = [f['pts'][:, 1].min() for f in feet]
    return y


def given(a, y, fr, posture, bend, knees):
    """The stack's ask (`wbc.step`) of `a`: `posture` her driven joints standing, rad; `bend`
    how each goes with her knees' (`crouch`); `knees` each knee's index among them. A sole that
    does not stand is asked its acceleration where it is; no turn is asked with none standing."""
    free = np.flatnonzero(a[STANCE] < 0.5)
    soles, speeds, accs = (v.reshape(2, 3) for v in (y[SOLE], y[VSOLE], a[SOLE_ACC]))
    pose = posture + bend * a[BEND]
    half, level = 0.5 * a[WANT], fr[LEVEL]
    turns = np.stack([level[0] * np.cos(half), level[0] * np.sin(half),
                      level[2] * np.cos(half), -level[2] * np.sin(half)], axis=1)
    return {'stance': tuple(a[STANCE] > 0.5), 'com_acc': a[COM_ACC].copy(),
            'turns': tuple(turns[:2 * int(a[TURNED])]), 'posture': pose, 'kdot': a[KDOT].copy(),
            'swing': {int(i): (soles[i].copy(), speeds[i].copy(), accs[i].copy(), level) for i in free},
            'fold': {int(i): (pose[knees[i]] + a[FOLD][i], a[FOLD_RATE][i], 0.0) for i in free}}
