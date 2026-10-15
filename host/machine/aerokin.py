"""The quad's kinematics in a vacuum: a row of setpoints and where the frame is, to a payload.

    law = aerokin.fresh()                                     # the law's own: its spot, its heading
    payload = aerokin.step(law, row, frame, dt, pull, head)   # a pass: aero/'s payload for it

A row is `machine.aerobatics`': a height and the pace it may go there at, a speed along its
heading and across, a place, a turn, turns about its nose and its wing. Up and down the frame
flies the speed its distance from its height allows - the loop's own approach near it, past
that what its pull stops from - and along the floor its spot goes at the speed asked and the
frame is kept on it. What that asks of it, a force a kg, is a collective and where the rotors'
axis points in the world: its tilt in the level frame of the heading the core holds (`head`,
the core's answer) - a payload's latent targets, at the frame's impedance. The air is not in
it: what it does to the frame is the core's observers'. No branch: a choice is a number of 0
or 1, a bound a least or a most.
"""
import math

from machine import quad
from machine.aerolisp import CompiledAeroPayload, K_MODES, LATENT_SYMBOL_MAP
from machine.figure import cross

_IMP, _KIN = quad.DYNAMICS['impedance'], quad.KINEMATICS
_ENV, _VERT, _HORIZ, _HOLD = (
    _KIN['envelope'], _KIN['vertical'], _KIN['horizontal'], _KIN['hold'])

#: The frame's impedance as a payload asks it: the tilt's stiffness, 1/s^2, and damping, 1/s;
#: a turn's about its nose or its wing, on its rate's miss summed and on its miss; the
#: heading's rate's damping, and how fast a nose off its heading comes round to it, 1/s.
#: The tilt at 201, the lap's seam - 2.4 m down in 6 - was crossed 0.51 m high, at 0.75 of
#: the frame's size 0.76: at 300, 0.06 low and 0.04 high.
TILT_K, TILT_D = _IMP['stiffness']['attitude_pitch'], _IMP['damping']['attitude_pitch']
TURN_K, TURN_D = _IMP['stiffness']['pitch_rate'], _IMP['damping']['pitch_rate']
YAW_D, NOSE_K = _IMP['damping']['attitude_yaw'], 1.0

#: The height's loop, 1/s on its rate, both its poles at half of it; a height is flown to
#: braked at this share of what the rotors' pull leaves - rising, of gravity -, a pace come to
#: and stopped from in PACE_S.
KD, BRAKE_SHARE, PACE_S = _VERT['settle_kd'], _VERT['brake_gain'], 1.0

#: Along the floor: the spot's loop, 1/s^2 and 1/s; its spot goes to its place at HOME_K, 1/s,
#: HOME_M_S at the most.
SPOT_KP, SPOT_KD, HOME_K, HOME_M_S = _HORIZ['spot_kp'], _HORIZ['spot_kd'], 1.5, _HORIZ['orbit_speed']

#: What the frame is sped up on at the least, of its weight, whatever pull it is given; and
#: what of its rotors' top a card is planned on: the rest the tilt's and the observers'.
LEAST, HEADROOM = _ENV['least_pull'], 0.9

#: Held: within this of its height, m, and of its spot, m, this slow, m/s, its axis this
#: near up, rad - what it may lean into a wind.
HELD_M, HELD_SPOT_M = _HOLD['height_tol'], _HOLD['spot_tol']
HELD_M_S, HELD_RAD = _HOLD['speed_tol'], _HOLD['tilt_tol']

#: What it is doing where it is asked a height far under it, by how light its lift is:
#: burning, let fall.
BURN, FALL = 'burn', 'fall'
DOING = (None, BURN, FALL)

#: The rotors' axis is asked no further from where it points than this, as its (cos, sin) -
#: a sixth of a turn; the axis it turns about keeps this much of the last pass's.
AHEAD, KEEP = (math.cos(math.pi / 3.0), math.sin(math.pi / 3.0)), 0.2

PITCH, ROLL = LATENT_SYMBOL_MAP['mode_attitude_pitch'], LATENT_SYMBOL_MAP['mode_attitude_roll']
YAW_RATE, PITCH_RATE, ROLL_RATE = (LATENT_SYMBOL_MAP[name] for name in (
    'mode_yaw_rate', 'mode_pitch_rate', 'mode_roll_rate'))

#: The force vector's slew: max turn rate, rad/s, acceleration, rad/s^2, and approach slope,
#: 1/s, within reach; the share of its rate fed onto the frame's when pushed down.
FORCE_W, FORCE_A, FORCE_K, FORCE_SHARE = 8.0, 200.0, 10.0, 0.25

#: The law's constants that have a unit, by the powers of their metres and seconds
#: (`quad.rescaled`).
_UNITS = {'TILT_K': (0, -2), 'TILT_D': (0, -1), 'TURN_K': (0, -2), 'TURN_D': (0, -1),
          'YAW_D': (0, -1), 'NOSE_K': (0, -1), 'KD': (0, -1), 'PACE_S': (0, 1),
          'SPOT_KP': (0, -2), 'SPOT_KD': (0, -1), 'HOME_K': (0, -1), 'HOME_M_S': (1, -1),
          'HELD_M': (1, 0), 'HELD_SPOT_M': (1, 0), 'HELD_M_S': (1, -1),
          'FORCE_W': (0, -1), 'FORCE_A': (0, -2), 'FORCE_K': (0, -1)}
_BUILT = {}


def sized():
    """The kinematics' constants for the frame as `quad.sized` has it."""
    quad.rescaled(globals(), _UNITS, _BUILT)


def idle():
    """What the four give a kg at their idle, m/s^2."""
    return 4.0 * quad.K_THRUST * quad.IDLE_RAD_S ** 2 / quad.MASS_KG


def fresh():
    """The law's own before its first pass: its spot, world (x, z), m, and the speed that went
    at last, m/s; its heading as (cos, sin) about up from z toward x; the collective asked
    last, m/s^2; the axis its rotors' axis turned about last, world; the payloads made;
    whether it is held, what it is doing; the force axis and its turn rate in the world."""
    return {'spot': [0.0, 0.0], 'speed': (0.0, 0.0), 'heading': (1.0, 0.0), 'lift': idle(),
            'about': [1.0, 0.0, 0.0], 'force': [0.0, 1.0, 0.0], 'rate': [0.0, 0.0, 0.0],
            'sequence': 0, 'held': False, 'doing': None}


def allowed(over, brake):
    """The speed the frame flies `over` m from its height, m/s: the loop's own approach near
    it, `brake`'s pull past where the two meet in speed and pull."""
    pole = 0.5 * KD
    near = brake / (pole * pole)
    far = float(over > near)
    return (1.0 - far) * pole * over + far * math.sqrt(2.0 * brake * max(over - 0.5 * near, 0.0))


def climb(row, h, v, room):
    """(the pull asked up, m/s^2, what it is doing) for the frame at `h` climbing `v`, `room`
    the pull up its rotors leave it: braked as its pace asks or as its height needs, their
    share - rising, gravity's - at the most; the speed its distance allows on its height's
    own rate, its pace at the most, and that speed's own fall, begun where its pace's stop
    turns; sped toward its height as gently - up, on its room at the most -, the height's
    own pull in it, and stopped outright going from it."""
    pace = row['pace']
    gentle, pole = pace / PACE_S, 0.5 * KD
    rate = max(-pace, min(pace, row['climb']))
    push = float(rate == row['climb']) * max(-gentle, min(gentle, row['push']))
    over = row['height'] - h
    way = math.copysign(1.0, over)
    up = float(way > 0.0)
    left, toward, going = abs(over), way * (v - rate), way * v
    hard = BRAKE_SHARE * (up * quad.GRAVITY + (1.0 - up) * room)
    soft = min(gentle, hard)
    need = float(toward > 0.0) * pole * pole * (
        left - math.sqrt(max(0.0, left * left - (toward / pole) ** 2)))
    brake = min(hard, max(soft, need))
    cruise = pace - way * rate
    speed = min(allowed(left, brake), cruise)
    slows = min(brake, pole * pole * left, KD * max(0.0, cruise - allowed(left, soft)))
    pull = way * (min(KD * (way * rate + speed - max(going, 0.0)) - slows + way * push,
                      up * min(gentle, room) + (1.0 - up) * gentle) - KD * min(going, 0.0))
    dive = (1.0 - up) * float(v < 0.0) * float(rate < 0.0) * float(push + quad.GRAVITY < 0.0)
    pull = (1.0 - dive) * pull + dive * min(pull, push)
    far = (1.0 - up) * float(left > brake / (pole * pole))
    return pull, DOING[int(far * (1.0 + float(quad.GRAVITY + pull <= idle())))]


def lean(law, row, at, vel, dt, most):
    """The pull asked along the floor, world (x, z), m/s^2, `most` at the most: the law's spot
    gone on at the speed asked and to its place, the frame kept on it and leant into what
    that speed changed by, the row's own pull with it."""
    c, s = law['heading']
    off = (law['spot'][0] - row['x'], law['spot'][1] - row['z'])
    home = -row['home'] * min(HOME_K, HOME_M_S / max(math.hypot(off[0], off[1]), 1e-9))
    speed = (row['speed'] * s + row['slide'] * c + home * off[0],
             row['speed'] * c - row['slide'] * s + home * off[1])
    own = (row['surge'] * s + row['sway'] * c, row['surge'] * c - row['sway'] * s)
    law['spot'] = [p + u * dt for p, u in zip(law['spot'], speed)]
    out = [(speed[k] - law['speed'][k]) / dt + own[k] + SPOT_KD * (speed[k] - float(vel[axis]))
           + SPOT_KP * (law['spot'][k] - float(at[axis])) for k, axis in enumerate((0, 2))]
    law['speed'] = speed
    scale = min(1.0, most / max(math.hypot(out[0], out[1]), 1e-9))
    return out[0] * scale, out[1] * scale


def seen(law, frame, dt, pull, top):
    """What a row's giver may know of the flight before the pass of `dt` s (`course.line`):
    where the frame is and goes, the law's spot and heading, the pull it is given and what all
    of it would be - HEADROOM of `top`, the rotors' at theirs -, m/s^2, the air along the
    discs their thrust is none at, m/s. The air is the core's: none of it is here."""
    c, s = law['heading']
    full = HEADROOM * top
    return {'dt': dt, 'share': pull / full, 'heading': math.atan2(s, c), 'full': full,
            'pull': max(LEAST * quad.GRAVITY, pull),
            'pitch': quad.INFLOW_J0 * quad.PITCH_M * quad.speed_for(
                top * quad.MASS_KG / 4.0) / math.tau,
            'spot': tuple(law['spot']), 'at': [float(x) for x in frame['at']],
            'vel': [float(x) for x in frame['vel']]}


def step(law, row, frame, dt, pull, head=(0.0, 0.0), pitch=math.inf):
    """The payload for a pass of `dt` s: the row `row` asked of the frame as `frame`
    (`quad.Sky.state`) has it, `pull` the most its rotors give a kg, m/s^2, `head` the heading
    the core holds, (cos, sin) from z toward x (its answer's) - none held yet, both none, the
    frame's nose along the floor: the core's own from a level stand. The law's heading turned
    on;
    the pull up and along the floor (`climb`, `lean`); the force that is, as a collective
    along its direction and that direction's tilt in the core's level frame; the turn its
    heading gives the axis, as the rates about its wing and its nose; its nose brought round
    to the law's heading as firmly as asked; a turn about its nose or its wing as a rate, that
    attitude dropped and the collective its weight."""
    turn = [[float(frame['turn'][r][k]) for k in range(3)] for r in range(3)]
    held = float(head[0] * head[0] + head[1] * head[1] > 0.0)
    ahead = max(math.hypot(turn[2][2], turn[0][2]), 1e-9)
    head = tuple(held * h + (1.0 - held) * own / ahead
                 for h, own in zip(head, (turn[2][2], turn[0][2])))
    rate = math.radians(row['turn'])
    half = 0.5 * rate * dt
    c, s = law['heading']
    c, s = (((1.0 - half * half) * c - 2.0 * half * s) / (1.0 + half * half),
            (2.0 * half * c + (1.0 - half * half) * s) / (1.0 + half * half))
    law['heading'] = (c, s)
    axis = (turn[0][1], turn[1][1], turn[2][1])
    through = sum(float(v) * a for v, a in zip(frame['vel'], axis))
    reach = max(LEAST * quad.GRAVITY, pull * min(1.0, max(0.0, 1.0 - through / pitch)))
    up_pull, law['doing'] = climb(row, frame['h'], frame['v'], reach - quad.GRAVITY)
    light = row['light'] * quad.GRAVITY + float(law['doing'] == FALL) * min(
        1.5 * quad.GRAVITY, 0.2 * KD * max(0.0, float(frame['v'])))
    beside = math.sqrt(max(reach * reach - quad.GRAVITY ** 2, 0.0))
    along = lean(law, row, frame['at'], frame['vel'], dt, min(row['lean'], beside))
    # where the rotors' axis points for it in the world, against `light` of gravity at the
    # least; asked no further from where it points than AHEAD, round the axis it turns about
    # - the last pass's, where that is next to none: opposite, any way round is as short, and
    # a way chosen anew a pass is no way
    aim = (along[0], max(quad.GRAVITY + up_pull, light), along[1])
    size = math.sqrt(aim[0] * aim[0] + aim[1] * aim[1] + aim[2] * aim[2])
    raw_want = (aim[0] / size, aim[1] / size, aim[2] / size)
    # the force vector slewed inside reach: turn rate FORCE_W and acceleration FORCE_A
    w_cur, w_rate = law['force'], law['rate']
    e = cross(w_cur, raw_want)
    e_norm = math.sqrt(sum(x * x for x in e))
    theta = math.atan2(e_norm, sum(a * b for a, b in zip(w_cur, raw_want)))
    w_mag = min(FORCE_W, FORCE_K * theta, math.sqrt(2.0 * FORCE_A * theta), theta / dt)
    e_unit = [x / max(e_norm, 1e-9) for x in e]
    dw = [w_mag * e_unit[k] - w_rate[k] for k in range(3)]
    dw_norm = math.sqrt(sum(x * x for x in dw))
    sc_a = min(1.0, FORCE_A * dt / max(dw_norm, 1e-9))
    law['rate'] = w_rate = [w_rate[k] + dw[k] * sc_a for k in range(3)]
    d_rot = cross(w_rate, w_cur)
    w_next = [w_cur[k] + d_rot[k] * dt for k in range(3)]
    w_norm = math.sqrt(sum(x * x for x in w_next))
    law['force'] = want = [x / w_norm for x in w_next]
    dot = axis[0] * want[0] + axis[1] * want[1] + axis[2] * want[2]
    last = sum(a * k for a, k in zip(axis, law['about']))
    about = cross(axis, want)
    about = [about[k] + KEEP * (law['about'][k] - last * axis[k]) for k in range(3)]
    norm = max(math.sqrt(sum(x * x for x in about)), 1e-9)
    law['about'] = about = [x / norm for x in about]
    side = cross(about, axis)
    far = float(dot < AHEAD[0])
    want = [far * (AHEAD[0] * axis[k] + AHEAD[1] * side[k]) + (1.0 - far) * want[k]
            for k in range(3)]
    # the collective what the force asked is along the axis as it points - down no further
    # than `light` lets it; under that, with the axis up, less than none: the air's to give,
    # and the core's not to make good - asked through the core's own share for where it is
    # asked
    down = max(quad.GRAVITY + up_pull, min(0.0, light))
    down += float(axis[1] > 0.0) * min(0.0, quad.GRAVITY + up_pull - down)
    lift = (along[0] * axis[0] + down * axis[1] + along[1] * axis[2]) / max(dot, AHEAD[0])
    # the axis in the level frame of the heading the core holds: its tilt back and to the
    # right, the two angles of one turn
    there = (want[0] * head[0] - want[2] * head[1], want[1], want[0] * head[1] + want[2] * head[0])
    flat = math.hypot(there[0], there[2])
    arc = math.atan2(flat, there[1]) / max(flat, 1e-9)
    # a turn about its nose or its wing: its rate, that attitude dropped, its weight borne
    rolls, flips = float(row['roll'] != 0.0), float(row['flip'] != 0.0)
    turning = max(rolls, flips)
    law['lift'] = turning * quad.GRAVITY + (1.0 - turning) * lift
    # its nose round to the law's heading: the miss's sine as its nose lies along the floor,
    # a unit past a quarter turn
    nose = (s * turn[2][2] - c * turn[0][2], c * turn[2][2] + s * turn[0][2])
    behind = float(nose[1] < 0.0)
    miss = (1.0 - behind) * nose[0] + behind * math.copysign(1.0, nose[0])
    targets = [math.nan] * K_MODES
    targets[0] = law['lift']
    targets[PITCH] = (-there[2] * arc, math.nan)[int(flips)]
    targets[ROLL] = (there[0] * arc, math.nan)[int(rolls)]
    targets[YAW_RATE] = row['nose'] * (rate * turn[1][1] + NOSE_K * miss)
    # the axis turns with the law's heading and with the force vector itself:
    # about the frame's wing and nose that is the heading's and force's rate along each -
    # the tilt's damping is on what the frame turns short of it
    w_wing = sum(r * w for r, w in zip(w_rate, (turn[0][0], turn[1][0], turn[2][0])))
    w_nose = sum(r * n for r, n in zip(w_rate, (turn[0][2], turn[1][2], turn[2][2])))
    w_share = (1.0 - turning) * (1.0 - (1.0 - FORCE_SHARE) * float(axis[1] < 0.0) * float(row['climb'] < 0.0))
    targets[PITCH_RATE] = -rate * turn[1][0] - math.tau * row['flip'] - w_share * w_wing
    targets[ROLL_RATE] = -rate * turn[1][2] - math.tau * row['roll'] - w_share * w_nose
    stiffness, damping = [0.0] * K_MODES, [0.0] * K_MODES
    stiffness[PITCH] = stiffness[ROLL] = TILT_K
    stiffness[PITCH_RATE], stiffness[ROLL_RATE] = flips * TURN_K, rolls * TURN_K
    damping[PITCH_RATE] = flips * TURN_D + (1.0 - flips) * TILT_D
    damping[ROLL_RATE] = rolls * TURN_D + (1.0 - rolls) * TILT_D
    damping[YAW_RATE] = YAW_D
    law['held'] = bool(
        float(abs(row['height'] - frame['h']) <= HELD_M)
        * float(max(abs(float(x)) for x in frame['vel']) <= HELD_M_S)
        * float(math.hypot(float(frame['at'][0]) - law['spot'][0],
                           float(frame['at'][2]) - law['spot'][1]) <= HELD_SPOT_M)
        * float(turn[1][1] >= math.cos(HELD_RAD)) * (1.0 - turning))
    law['sequence'] += 1
    return CompiledAeroPayload(
        sequence_id=law['sequence'], blend_duration_ms=int(round(1e3 * dt)),
        latent_targets=targets, stiffness=stiffness, damping=damping, cpg_enabled=0,
        cpg_frequency=0.0, cpg_amplitudes=[0.0] * K_MODES, task_constraints_mask=0)
