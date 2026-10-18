"""Her maneuver horizon, drawn from her law's own reading: an externality each.

Each sole's room - the floor its leg reaches with it flat (`machine.reach`), dim, and the room
her law lets it land in (`stride.meets`: REACH_M about her centre of mass, APART_M its own side
of the other sole), bright while it swings -, her capture point; each arm's band, its hand from
its elbow's stop to straight over its shoulder's range. The reading is `sprung.state`'s
'horizon'; a mode of MODES says which are drawn, Z cycles them.

    layers = horizon.segments(now, props, show)     # [(segments, ink)] for `gynoid.render`
"""
import math

from coaxial.graphics.arrows import _ring_segments, surface_at
from machine import reach
from machine.figure import ANKLE_H, FOREARM, UPPER_ARM, frames, quat
from machine.mjcf import STOPS
from machine.stride import APART_M, REACH_M

#: What is drawn: nothing, the soles' rooms and her capture point, the arms' bands, all.
MODES = ('off', 'soles', 'arms', 'all')
#: The floor a leg reaches, a swinging sole's room, a standing sole's - off the floor grid's
#: blue -, her capture point, an arm's band.
REACH_INK, SWING_INK, ROOM_INK = (110, 110, 140), (255, 184, 72), (170, 160, 100)
CAPTURE_INK, ARM_INK = (255, 96, 128), (150, 120, 220)
#: Points to a half ring; her capture point's ring, m; the farthest a leg is searched, m.
RING_N, CAPTURE_M, FAR_M = 24, 0.04, 1.0


def _turn(heading):
    """A flat foot headed `heading`, a 3x3 of rows: across to her left, up, along the toes."""
    sh, ch = math.sin(heading), math.cos(heading)
    return ((ch, 0.0, sh), (0.0, 1.0, 0.0), (-sh, 0.0, ch))


def reachable(limb, hip, heading, props=()):
    """The floor `limb`'s leg reaches with its sole flat and headed `heading` (`reach.margins`,
    its stops and its length): the outline (segments) about its `hip` (world) in RING_N
    directions, each as far out as reaches, none where none does."""
    sh, ch = math.sin(heading), math.cos(heading)
    turn, floor = _turn(heading), surface_at(hip[0], hip[2], props)[0]

    def inside(x, z):
        return min(reach.margins(reach.over(limb, hip, (x, floor + ANKLE_H, z), turn))) >= 0.0

    ring = []
    for k in range(RING_N):
        a = math.tau * k / RING_N
        dx, dz = math.cos(a) * ch + math.sin(a) * sh, -math.cos(a) * sh + math.sin(a) * ch
        low, high = 0.0, FAR_M
        if not inside(hip[0], hip[2]):
            continue
        for _ in range(14):
            mid = 0.5 * (low + high)
            low, high = (mid, high) if inside(hip[0] + mid * dx, hip[2] + mid * dz) else (low, mid)
        ring.append((hip[0] + low * dx, floor + 0.005, hip[2] + low * dz))
    return [(ring[k], ring[(k + 1) % len(ring)]) for k in range(len(ring))] if len(ring) > 2 else []


def room(c, other, side, heading, props=()):
    """A sole's landing room's outline on the floor (segments): the arc REACH_M about her
    centre of mass `c` (x, z) on its `side` (1 her left), cut APART_M its own side of the
    `other` sole (x, z); none where the cut is past the arc."""
    sh, ch = math.sin(heading), math.cos(heading)
    out_x, out_z = side * ch, -side * sh
    cut = side * ((other[0] - c[0]) * ch - (other[1] - c[1]) * sh) + APART_M
    if cut >= REACH_M:
        return []
    half = math.acos(max(-1.0, cut / REACH_M))

    def at(a):
        across, along = REACH_M * math.cos(a), REACH_M * math.sin(a)
        x, z = c[0] + across * out_x + along * sh, c[1] + across * out_z + along * ch
        return (x, surface_at(x, z, props)[0] + 0.005, z)
    arc = [at(-half + 2.0 * half * k / RING_N) for k in range(RING_N + 1)]
    return list(zip(arc, arc[1:])) + ([(arc[-1], arc[0])] if half < math.pi else [])


def band(shoulder, heading, low, high):
    """An arm's band in her sagittal plane (segments): its hand's arc straight over the
    shoulder's range `low` to `high` (rad of flexion, hanging 0), the arc at the elbow's stop
    inside it, the two closed at the range's ends."""
    sh, ch = math.sin(heading), math.cos(heading)
    stop = math.radians(STOPS['elbow'][1])

    def hand(flex, elbow):
        fwd = UPPER_ARM * math.sin(flex) + FOREARM * math.sin(flex + elbow)
        down = UPPER_ARM * math.cos(flex) + FOREARM * math.cos(flex + elbow)
        return (shoulder[0] + fwd * sh, shoulder[1] - down, shoulder[2] + fwd * ch)
    arcs = [[hand(low + (high - low) * k / RING_N, elbow) for k in range(RING_N + 1)] for elbow in (0.0, stop)]
    return (list(zip(arcs[0], arcs[0][1:])) + list(zip(arcs[1], arcs[1][1:]))
            + [(arcs[0][0], arcs[1][0]), (arcs[0][-1], arcs[1][-1])])


def segments(now, props=(), show='all'):
    """[(segments, ink)] of `show` (MODES): each leg's reach and each sole's landing room -
    bright while it swings -, her capture point; each arm's band; none without her law's
    reading."""
    seen = (now or {}).get('horizon')
    if not seen or show == 'off':
        return []
    heading, (cx, _cy, cz) = float(now.get('heading') or 0.0), seen['com']
    placed = frames(now.get('angles') or {}, now.get('where', (0.0, 0.9, 0.0)), quat(*now.get('turn', (1, 0, 0, 0))))
    out = []
    if show in ('soles', 'all'):
        soles = [(s[0], s[2]) for s in seen['soles']]
        for k, side in enumerate(('left', 'right')):
            out.append((reachable(k, tuple(float(v) for v in placed[side + '_thigh'][0]), heading, props), REACH_INK))
        for k, side in enumerate((1.0, -1.0)):
            out.append((room((cx, cz), soles[1 - k], side, heading, props),
                        ROOM_INK if seen['stance'][k] > 0.5 else SWING_INK))
        out.append((_ring_segments(seen['xi'][0], seen['xi'][1], CAPTURE_M, props, 8), CAPTURE_INK))
    if show in ('arms', 'all'):
        low, high = seen.get('shoulder', (-1.13, 2.62))
        for side in ('left', 'right'):
            out.append((band(tuple(float(v) for v in placed[side + '_upper_arm'][0]), heading, low, high), ARM_INK))
    return [(segs, ink) for segs, ink in out if segs]
