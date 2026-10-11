"""Her errands in the room: a word's target walked to on the one law, faced, and done there.

    errands.ask(director, 'sit')                      # a word of `room.WORDS`, from the page
    out = errands.step(director, bus, dt, out)        # every pass on the law: steered, then done

An errand goes 'go' - the law's heading turned toward the next waypoint at TURN_RAD_S at most,
only TURNS_S into a steady row; faced within FACED_RAD of the way she walks, FACES_RAD or more
off it she turns standing, stepping round (`hold.staying`); a waypoint passed within NEAR_M,
the last LAST_M before the target on the word's own heading (`room.WORDS`), the slow row from
there and the way kept from HOLD_M, the target come within HOME_M -; 'face' - standing,
turned to face as the word says, within FACED_RAD -;
'settle' - the stand's row, SETTLE_S; then the word's keyframes are played from her pose as
read (`arrival.play`, the director's stage 'errand') and the last held: sat, lain; 'up' plays
her back onto her feet, and the law takes her again. The lamp's switch is reached standing on
the law, the right arm raised over its setpoints until the lamp toggles, held, let down
('reach'): on the arrival's standing keyframes, her feet stepped round 93 deg, she sank into
a squat in 1 s; raised to a set angle 6 cm into the wall, the push turned her 11 deg and she
fell stepping (2026-10-11).
"""
import math

from machine import figure, hold, pace, room
from machine.figure import add, apply, ry

#: The law's heading turned this fast at most, rad/s, (walking, standing): walking at 0.3 she
#: turned 292 deg in 17 s and at 0.5 fell at 8 s; standing at 0.5 she stepped round 258 deg in
#: 9 s, at 1.0 fell at 4.5 s; turned from the law's first pass, standing or walking, she fell
#: at 4.5 s, so only TURNS_S (walking, standing) into a steady row (2026-10-11). Faced within FACED_RAD of the
#: way she walks, FACES_RAD or more off it she turns standing; a waypoint passed within NEAR_M,
#: the target come within HOME_M, the way kept walking within HOLD_M of it - aimed from there a
#: step swings the bearing 40 deg -; the stand's row held SETTLE_S before the word is done;
#: the slow row within LAST_M, where the approach is laid.
TURN_RAD_S, TURNS_S, FACES_RAD, FACED_RAD = (0.3, 0.5), (0.5, 1.5), 0.5, 0.1
NEAR_M, HOME_M, HOLD_M, SETTLE_S, LAST_M = 0.35, 0.3, 0.4, 1.0, 1.8
ROW_GO, ROW_SLOW, ROW_STAND = 0.0, -0.6, -1.0

#: A word's doing, s a keyframe: down onto the seat, laid back onto the bed; the arm raised
#: REACH_DEG_S to the switch, REACH_DEG at most, held HOLD_S at the touch; the pelvis back over
#: the seat, m - the target 0.6 m before the seat's centre, her toes clear of the chair's legs
#: as she steps round -, and down to it.
SIT_S, LIE_S, UP_S, HOLD_S, REACH_DEG_S, REACH_DEG = 1.2, 1.5, 1.2, 0.5, 40.0, 80.0
BACK_M, RISE_M = 0.45, 0.08
#: Her pelvis standing, m.
STAND_Y = 0.90

#: Doing a word she is down this far under its keyframe's pelvis, or tipped this far past its
#: tilt: sat her pelvis is 0.55 m up, lain she is tipped 88 deg.
FALLEN_M, FALLEN_DEG = 0.2, 35.0
#: Her right hand reaches HAND_RIGHT_M to the right of her pelvis's centre (measured
#: 2026-10-11): facing the switch, her heading is aimed so that line runs through it.
HAND_RIGHT_M = 0.27
#: The arms at rest, deg; the reaching elbow's.
ARMS = {'left_shoulder': 10.0, 'right_shoulder': 10.0, 'left_elbow': 20.0, 'right_elbow': 20.0}
REACH_ELBOW = 10.0


def _moved(now, back, up, tilt, joints=None):
    """`now` (the director's pose keyframe) with the pelvis `back` m along her heading and `up`
    m from its height, tipped `tilt` deg, the feet where they stand, the arms `joints`."""
    pel = add(now['pelvis'], apply(ry(math.radians(now['yaw'])), (0.0, up, -back)))
    return dict(now, pelvis=pel, tilt=tilt, joints={**now['joints'], **ARMS, **(joints or {})})


def frames_of(word, now):
    """[(stage, seconds, keyframe)] a word plays from her pose `now`, the last held."""
    rest = now['pelvis'][1]
    if word == 'sit':
        seat = room.SEAT_H + room.SEAT[1] + RISE_M - rest
        return [('errand', 0.0, now), ('errand', SIT_S, _moved(now, BACK_M, seat + 0.05, 10.0)),
                ('errand', 0.6, _moved(now, BACK_M + 0.03, seat, 0.0))]
    if word == 'lie':
        edge = 2.0 * room.BED[1] + RISE_M - rest
        flat = {'left_shoulder': 0.0, 'right_shoulder': 0.0, 'left_elbow': 5.0, 'right_elbow': 5.0}
        return [('errand', 0.0, now), ('errand', SIT_S, _moved(now, BACK_M, edge + 0.05, 10.0)),
                ('errand', 0.6, _moved(now, BACK_M + 0.03, edge, 0.0)),
                ('errand', LIE_S, _moved(now, BACK_M + 0.15, edge, -70.0, flat)),
                ('errand', 1.0, _moved(now, BACK_M + 0.25, edge, -88.0, flat))]
    if word == 'up':
        return [('errand', 0.0, now),
                ('errand', UP_S, _moved(now, -float(now.get('back', 0.0)), STAND_Y - rest, 0.0))]
    return [('errand', 0.0, now)]


class Errand:

    """One word under way: its waypoints [(x, z)] world - LAST_M before the target on the
    word's heading, then the target -, the facing it ends in, rad about up from z; where it
    stands in its going."""

    def __init__(self, word):
        if word not in room.WORDS:
            raise ValueError('no word %r in the room; they are %s' % (word, ', '.join(room.WORDS)))
        self.word = word
        target, self.facing, self.does, come = room.WORDS[word]
        self.waypoints = ([(target[0] - LAST_M * math.sin(come), target[1] - LAST_M * math.cos(come)),
                           target] if target else [])
        self.phase, self.since, self.walks = 'go' if target else 'settle', 0.0, False

    def __repr__(self):
        return '%s: %s' % (self.word, self.phase)


def ask(director, word):
    """The page's `word`: an errand begun on the law - 'up' from a seat or a bed, at once."""
    if word == 'up':
        if director.stage == 'errand' and director.doing in ('sit', 'lie'):
            bus = director.machine.loop.bus
            now = director._now(bus, director.walker.last or {})
            now['back'] = director.back
            _play(director, 'up', now, director.walker.last or {})
        return
    director.errand = Errand(word)
    if director.pace is None or director.pace <= ROW_STAND:
        director.pace = ROW_GO      # the law takes her from her stand at a row above it


def _aimed(at, to):
    """The heading from `at` (x, z) that puts `to` before her right hand, HAND_RIGHT_M to her
    right of the line she faces along."""
    far = math.hypot(to[0] - at[0], to[1] - at[1])
    return _bearing(at, to) + math.asin(min(1.0, HAND_RIGHT_M / max(far, HAND_RIGHT_M)))


def _bearing(at, to):
    """The law's heading from `at` (x, z) to `to`, rad about up from z, positive toward +x."""
    return math.atan2(to[0] - at[0], to[1] - at[1])


def _play(director, word, now, out):
    """The word's keyframes played from `now`, the director's stage 'errand' and its word."""
    frames = frames_of(word, now)
    director.arrival.play(frames)
    director.stage, director.blend, director.age = 'errand', dict(out), 0.0
    director.doing, director.errand = word, None
    director.back = {'sit': BACK_M + 0.03, 'lie': BACK_M + 0.25}.get(word, 0.0)


def fallen(director, bus):
    """Whether she is down doing a word, against the keyframe held."""
    frame = director.arrival._at()[1]
    return (bus['pelvis.pose.y'] < frame['pelvis'][1] - FALLEN_M
            or director._tilt(bus) > abs(frame['tilt']) + FALLEN_DEG)


def held(director, bus, dt):
    """Her setpoints at a word's doing (the director's stage 'errand'): the keyframes played and
    the last held; 'up' done, the law takes her again."""
    out = director.walker.last = pace.blended(director, director.arrival.step(dt), dt)
    if director.doing == 'up' and director.arrival.t >= sum(s for _n, s, _f in director.arrival.frames):
        director.doing = None
        director.pace = ROW_STAND
        pace.take(director, bus, out)
    return out


def step(director, bus, dt, out):
    """Her setpoints this pass with her errand steered on the law and, at its target, begun:
    `out` the law's own, kept while she goes."""
    e, law = director.errand, director.going
    if e is None or law is None:
        return out
    at = (bus['pelvis.pose.x'], bus['pelvis.pose.z'])
    if e.phase == 'go':
        to = e.waypoints[0]
        far = math.hypot(to[0] - at[0], to[1] - at[1])
        if len(e.waypoints) > 1 and far <= NEAR_M:
            # the approach point: stood and aligned before the last stretch - walked on up to
            # FACES_RAD off it, she passed the chair's target 0.3 m to the side (2026-10-11)
            e.waypoints.pop(0)
            e.walks = False
            to = e.waypoints[0]
            far = math.hypot(to[0] - at[0], to[1] - at[1])
        along = (to[0] - at[0]) * math.sin(law.heading) + (to[1] - at[1]) * math.cos(law.heading)
        last = len(e.waypoints) == 1
        turn = hold.wrapped(_bearing(at, to) - law.heading)
        if last and (far <= HOME_M
                     or (far < HOLD_M and along <= HOME_M and abs(turn) <= FACES_RAD)):
            e.phase, e.since = 'face', 0.0
            director.pace = ROW_STAND
            return out
        # the last stretch walked through, its way corrected walking: stopped there to turn,
        # 0.65 m from the bed's target and 29 deg off, the halt carried her into the bed
        e.walks = (abs(turn) < FACED_RAD if not e.walks else
                   abs(turn) <= FACES_RAD or (last and far < LAST_M))
        if last and far < HOLD_M and e.walks:
            turn = 0.0
        steady = director.k[0] >= -0.01 if e.walks else director.k[0] <= -0.99
        e.since = e.since + dt if steady else 0.0
        if e.since >= TURNS_S[0 if e.walks else 1]:
            rate = TURN_RAD_S[0] if e.walks else TURN_RAD_S[1]
            law.heading += max(-rate * dt, min(rate * dt, turn))
        # the slow row on the last stretch and before a waypoint: stopped from the walk's row she
        # fell at the approach point and, dwelt on it, overran the switch's target 0.5 m
        director.pace = (ROW_SLOW if last or far < LAST_M else ROW_GO) if e.walks else ROW_STAND
        return out
    if e.phase == 'face':
        e.since = e.since + dt if director.k[0] <= -0.99 else 0.0
        want = _aimed(at, room.SWITCH_XZ) if e.does == 'lamp' else e.facing
        turn = hold.wrapped(want - law.heading)
        if e.since >= TURNS_S[1]:
            law.heading += max(-TURN_RAD_S[1] * dt, min(TURN_RAD_S[1] * dt, turn))
            if abs(hold.wrapped(want - facing_of(bus))) < FACED_RAD:
                e.phase, e.since = 'settle', 0.0
        return out
    if e.phase == 'reach':
        lamp = bool(getattr(director.world, 'lamp', False))
        if e.touched is None and (lamp != e.lamp_was or e.arm >= REACH_DEG):
            e.touched = 0.0
        if e.touched is None:
            e.arm += REACH_DEG_S * dt
        else:
            e.touched += dt
            if e.touched >= HOLD_S:
                e.arm -= REACH_DEG_S * dt
        rest = out.get('right_shoulder', 0.0)
        k = max(0.0, min(1.0, e.arm / REACH_DEG))
        out['right_shoulder'] = max(rest, e.arm)
        out['right_elbow'] = out.get('right_elbow', 0.0) + k * (REACH_ELBOW - out.get('right_elbow', 0.0))
        if e.touched is not None and e.arm <= rest:
            director.errand = None
        return out
    e.since += dt
    if e.since >= SETTLE_S and director.k[0] <= -0.99:
        if e.does == 'out':
            director.errand = None
        elif e.does == 'lamp':
            e.phase, e.arm, e.touched = 'reach', 0.0, None
            e.lamp_was = bool(getattr(director.world, 'lamp', False))
        else:
            _play(director, e.does, director._now(bus, out), out)
    return out


def facing_of(bus):
    """Her heading as read, rad about up from z."""
    turn = figure.quat(bus['pelvis.pose.qw'], bus['pelvis.pose.qx'], bus['pelvis.pose.qy'],
                       bus['pelvis.pose.qz'])
    return math.atan2(turn[0][2], turn[2][2])
