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

#: The frame's impedance as a payload asks it: the tilt's stiffness, 1/s^2, and damping, 1/s;
#: a turn's about its nose or its wing, on its rate's miss summed and on its miss; the
#: heading's rate's damping, and how fast a nose off its heading comes round to it, 1/s.
TILT_K, TILT_D, TURN_K, TURN_D, YAW_D, NOSE_K = 201.0, 17.0, 201.0, 17.0, 4.0, 1.0

#: The height's loop, 1/s on its rate, both its poles at half of it; a height is flown to
#: braked at this share of what the rotors' pull leaves - rising, of gravity -, a pace come to
#: and stopped from in PACE_S.
KD, BRAKE_SHARE, PACE_S = 12.8, 0.55, 1.0

#: Along the floor: the spot's loop, 1/s^2 and 1/s; its spot goes to its place at HOME_K, 1/s,
#: HOME_M_S at the most.
SPOT_KP, SPOT_KD, HOME_K, HOME_M_S = 1.95, 2.94, 1.5, 3.0

#: What the frame is sped up on at the least, of its weight, whatever pull it is given; and
#: what of its rotors' top a card is planned on: the rest the tilt's and the observers'.
LEAST, HEADROOM = 1.5, 0.9

#: Held: within this of its height, m, and of its spot, m, this slow, m/s, its axis this
#: near up, rad.
HELD_M, HELD_SPOT_M, HELD_M_S, HELD_RAD = 0.03, 0.15, 0.15, 0.05

#: What it is doing where it is asked a height far under it, by how light its lift is.
DOING = (None, 'burn', 'fall')

#: A pull no bound is put on, m/s^2.
ANY = 1e4

#: The rotors' axis is asked no further from where it points than this, as its (cos, sin) -
#: a sixth of a turn; the axis it turns about keeps this much of the last pass's.
AHEAD, KEEP = (math.cos(math.pi / 3.0), math.sin(math.pi / 3.0)), 0.2

PITCH, ROLL = LATENT_SYMBOL_MAP['mode_attitude_pitch'], LATENT_SYMBOL_MAP['mode_attitude_roll']
YAW_RATE, PITCH_RATE, ROLL_RATE = (LATENT_SYMBOL_MAP[name] for name in (
    'mode_yaw_rate', 'mode_pitch_rate', 'mode_roll_rate'))

#: The law's constants that have a unit, by the powers of their metres and seconds
#: (`quad.rescaled`).
_UNITS = {'TILT_K': (0, -2), 'TILT_D': (0, -1), 'TURN_K': (0, -2), 'TURN_D': (0, -1),
          'YAW_D': (0, -1), 'NOSE_K': (0, -1), 'KD': (0, -1), 'PACE_S': (0, 1),
          'SPOT_KP': (0, -2), 'SPOT_KD': (0, -1), 'HOME_K': (0, -1), 'HOME_M_S': (1, -1),
          'HELD_M': (1, 0), 'HELD_SPOT_M': (1, 0), 'HELD_M_S': (1, -1)}
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
    whether it is held, what it is doing."""
    return {'spot': [0.0, 0.0], 'speed': (0.0, 0.0), 'heading': (1.0, 0.0), 'lift': idle(),
            'about': [1.0, 0.0, 0.0], 'sequence': 0, 'held': False, 'doing': None}


def allowed(over, brake):
    """(the speed the frame flies `over` m from its height, m/s, what that changes by a metre,
    1/s): the loop's own approach near it, `brake`'s pull past where the two meet in speed
    and slope."""
    pole = 0.5 * KD
    near = brake / (pole * pole)
    far = float(over > near)
    curve = math.sqrt(2.0 * brake * max(over - 0.5 * near, 0.0))
    return ((1.0 - far) * pole * over + far * curve,
            (1.0 - far) * pole + far * brake / max(curve, 1e-9))


def climb(row, h, v, room):
    """(the pull asked up, m/s^2, what it is doing) for the frame at `h` climbing `v`, `room`
    the pull up its rotors leave it: the speed its distance allows on its height's own rate,
    its pace at the most; that speed come to no faster than its pace is, a fall or a climb
    the wrong way stopped on whatever it takes - what the rotors have of it is their share's
    to say, and a pull under none of theirs is the air's to give."""
    pace = row['pace']
    gentle = pace / PACE_S
    rate = max(-pace, min(pace, row['climb']))
    over = row['height'] - h
    way = math.copysign(1.0, over)
    down = float(over < 0.0)
    brake = BRAKE_SHARE * (down * room + (1.0 - down) * quad.GRAVITY)
    most, slope = allowed(abs(over), brake)
    cruise = pace - way * rate
    speed = rate + way * min(most, cruise)
    falling, rising = float(v < 0.0), float(v > 0.0)
    up = falling * ANY + (1.0 - falling) * min(gentle, room)
    under = rising * ANY + (1.0 - rising) * gentle
    push = float(rate == row['climb']) * max(-gentle, min(gentle, row['push']))
    # the speed allowed falls as the height nears: that fall is asked with it
    slows = float(most < cruise) * slope * (rate - v)
    pull = max(-under, min(up, KD * (speed - v) + slows + push))
    far = down * float(abs(over) > brake / (0.25 * KD * KD))
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
    discs their thrust is none at, m/s. No wind: the air is the core's."""
    c, s = law['heading']
    full = HEADROOM * top
    return {'dt': dt, 'share': pull / full, 'heading': math.atan2(s, c), 'full': full,
            'pull': max(LEAST * quad.GRAVITY, pull),
            'pitch': quad.INFLOW_J0 * quad.PITCH_M * quad.speed_for(
                top * quad.MASS_KG / 4.0) / math.tau,
            'spot': tuple(law['spot']), 'at': [float(x) for x in frame['at']],
            'vel': [float(x) for x in frame['vel']], 'wind': (0.0, 0.0)}


def step(law, row, frame, dt, pull, head=None):
    """The payload for a pass of `dt` s: the row `row` asked of the frame as `frame`
    (`quad.Sky.state`) has it, `pull` the most its rotors give a kg, m/s^2, `head` the heading
    the core holds, (cos, sin) from z toward x (its answer's) - none held yet, the frame's
    nose along the floor: the core's own from a level stand. The law's heading turned on;
    the pull up and along the floor (`climb`, `lean`); the force that is, as a collective
    along its direction and that direction's tilt in the core's level frame; the turn its
    heading gives the axis, as the rates about its wing and its nose; its nose brought round
    to the law's heading as firmly as asked; a turn about its nose or its wing as a rate, that
    attitude dropped and the collective its weight."""
    turn = [[float(frame['turn'][r][k]) for k in range(3)] for r in range(3)]
    held = float(head is not None and any(head))
    ahead = max(math.hypot(turn[2][2], turn[0][2]), 1e-9)
    head = tuple(held * h + (1.0 - held) * own / ahead
                 for h, own in zip(head or (0.0, 0.0), (turn[2][2], turn[0][2])))
    rate = math.radians(row['turn'])
    half = 0.5 * rate * dt
    c, s = law['heading']
    c, s = (((1.0 - half * half) * c - 2.0 * half * s) / (1.0 + half * half),
            (2.0 * half * c + (1.0 - half * half) * s) / (1.0 + half * half))
    law['heading'] = (c, s)
    reach = max(LEAST * quad.GRAVITY, pull)
    up_pull, law['doing'] = climb(row, frame['h'], frame['v'], reach - quad.GRAVITY)
    most = min(row['lean'], math.sqrt(max(reach * reach - quad.GRAVITY ** 2, 0.0)))
    along = lean(law, row, frame['at'], frame['vel'], dt, most)
    light = row['light'] * quad.GRAVITY
    # where the rotors' axis points for it in the world, against `light` of gravity at the
    # least; asked no further from where it points than AHEAD, round the axis it turns about
    # - the last pass's, where that is next to none: opposite, any way round is as short, and
    # a way chosen anew a pass is no way
    aim = (along[0], max(quad.GRAVITY + up_pull, light), along[1])
    size = math.sqrt(aim[0] * aim[0] + aim[1] * aim[1] + aim[2] * aim[2])
    want = (aim[0] / size, aim[1] / size, aim[2] / size)
    axis = (turn[0][1], turn[1][1], turn[2][1])
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
    # the axis turns with the law's heading: about the frame's wing and nose that is the
    # heading's rate along each - the tilt's damping is on what the frame turns short of it
    targets[PITCH_RATE] = -rate * turn[1][0] - math.tau * row['flip']
    targets[ROLL_RATE] = -rate * turn[1][2] - math.tau * row['roll']
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
        sequence_id=law['sequence'], blend_duration_ms=0,
        latent_targets=targets, stiffness=stiffness, damping=damping, cpg_enabled=0,
        cpg_frequency=0.0, cpg_amplitudes=[0.0] * K_MODES, task_constraints_mask=0)
