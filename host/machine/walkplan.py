"""The walk's plan as the walker reads it: each leg's and the upper body's table by phase.

`plan(phase, stride)` gives the pelvis's place and turn and each foot's, tabled by stride from
`machine.gait` and weighed between; `carried(q)` how much a leg at phase `q` bears. The walker's
tracks (TRACK_M, WIDEN_M) and the pose's turn helpers live here beside them.
"""
import math

from machine import figure, gait
from machine.figure import LEG, mul


#: A foot takes the weight over ACCEPT of a stride from its landing, and gives it up over UNLOAD
#: before toe-off: taken at once, a landing off its mark jolted the pelvis.
ACCEPT, UNLOAD = 0.04, 0.06


#: A catwalk: the feet planted TRACK_M off the line, swung WIDEN_M further out round the standing
#: one; the pelvis turned TURN_GAIN of the walk's turn, the torso turning it back. At 0.02 and
#: 0.023 the sneakers passed 22 mm into each other; colliding (`physics.ME`), they bumped and the
#: thigh left the floor 1.6 degrees ahead of upright where it had 9.4 behind. At 0.03 and 0.035
#: they pass 5.8 mm apart (2026-09-28).
TRACK_M, WIDEN_M, TURN_GAIN = 0.03, 0.035, 1.3


#: The table's phases a stride.
SAMPLES = 240


SIDES = (('left', 1.0), ('right', -1.0))

_OTHER = {'left': 'right', 'right': 'left'}


def carried(q):
    """How much a leg carries at its phase `q`: 0 in the air, 1 in stance, eased between."""
    q %= 1.0
    if q >= gait.TOE_OFF:
        return 0.0
    return gait.eased(min(q / ACCEPT, (gait.TOE_OFF - q) / UNLOAD))


def _sample(p, stride):
    """The walk at the left leg's phase `p`: (pelvis lateral, height, yaw rad, per leg (ankle
    from the pelvis x y z, twist rad, pitch rad, toes deg), the upper body's joints deg, the
    pelvis's roll rad - dropped on the swing side, the spine taking it back out)."""
    angles = gait.walk(0.0, stride=stride, glance=False, phase=p)
    yaw = math.radians(angles['pelvis'] * TURN_GAIN)
    angles['waist'] *= TURN_GAIN
    lateral, roll, height, _level = gait.sway(0.0, stride=stride, phase=p)
    left, right, twist_l, twist_r = gait.tracks(0.0, phase=p, track=TRACK_M, widen=WIDEN_M)
    legs = []
    for q, track, twist in ((p, left, twist_l), ((p + 0.5) % 1.0, right, twist_r)):
        x, y = gait.planted(q, stride)[:2] if q < gait.TOE_OFF else gait.swung(q, stride)
        pitch = gait.pitch_of(q, stride)
        legs.append(((track - lateral, y, x), math.radians(twist), math.radians(pitch),
                     gait.toes_of(q, pitch)))
    angles['spine_roll'] = -roll
    upper = tuple(angles.get(j, 0.0) for j in UPPER)
    return lateral, height, yaw, tuple(legs), upper, math.radians(roll)


#: The joints the plan sets directly: the spine and head, the arms.
UPPER = tuple(j for j in figure.JOINTS
              if not j.endswith(tuple(LEG)) and not j.endswith('_foot'))


class _Tables(dict):
    def __missing__(self, stride):
        self[stride] = [_sample(k / SAMPLES, stride) for k in range(SAMPLES)]
        return self[stride]


#: The plan by stride, every `gait.PACE_STEP`, tabled on first ask.
_TABLES = _Tables()


def _mix(a, b, k):
    """`a` k of the way to `b`, element by element, through nested tuples."""
    if isinstance(a, tuple):
        return tuple(_mix(x, y, k) for x, y in zip(a, b))
    return a + (b - a) * k


def plan(p, stride) -> tuple:
    """The walk at phase `p` and `stride`, as `_sample` answers, from the tables."""
    low = math.floor(stride / gait.PACE_STEP) * gait.PACE_STEP
    w = (stride - low) / gait.PACE_STEP
    x = (p % 1.0) * SAMPLES
    i, u = int(x), x - int(x)

    def at(table):
        a, b = table[i % SAMPLES], table[(i + 1) % SAMPLES]
        if abs(b[2] - a[2]) > math.pi:
            b = (b[0], b[1], a[2]) + b[3:]
        return _mix(a, b, u)
    below = at(_TABLES[round(max(low, gait.PACE_STEP), 6)])
    if w < 1e-9:
        return below
    return _mix(below, at(_TABLES[round(low + gait.PACE_STEP, 6)]), w)


def vee(r):
    """The small turn a rotation is, as a vector: axis times angle for small angles."""
    return ((r[2][1] - r[1][2]) / 2.0, (r[0][2] - r[2][0]) / 2.0, (r[1][0] - r[0][1]) / 2.0)


def turned(v):
    """The rotation by the vector `v`: about its axis, its length in radians (Rodrigues)."""
    angle = math.sqrt(sum(c * c for c in v))
    if angle < 1e-12:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    k = tuple(c / angle for c in v)
    kx = ((0.0, -k[2], k[1]), (k[2], 0.0, -k[0]), (-k[1], k[0], 0.0))
    kk = mul(kx, kx)
    s, c = math.sin(angle), 1.0 - math.cos(angle)
    return tuple(tuple((1.0 if i == j else 0.0) + s * kx[i][j] + c * kk[i][j] for j in range(3))
                 for i in range(3))
