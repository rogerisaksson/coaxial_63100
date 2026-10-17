"""Her maneuver horizon on the floor, drawn from her law's own reading.

Where each sole may land - her law's room for it (`stride.meets`), REACH_M about her centre of
mass, APART_M its own side of the other sole -, her capture point, each arm's reach; the reading
is `sprung.state`'s 'horizon'.

    layers = horizon.segments(now, props)     # [(segments, ink)] for `gynoid.render`
"""
import math

from coaxial.graphics.arrows import _ring_segments, surface_at
from machine.figure import FOREARM, UPPER_ARM, frames, quat
from machine.stride import APART_M, REACH_M

#: A swinging sole's room, a standing sole's - off the floor grid's blue -, her capture point, an
#: arm's reach.
SWING_INK, ROOM_INK, CAPTURE_INK, ARM_INK = (255, 184, 72), (170, 160, 100), (255, 96, 128), (150, 120, 220)
#: Points to a half ring; her capture point's ring, m.
RING_N, CAPTURE_M = 24, 0.04


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


def segments(now, props=()):
    """[(segments, ink)]: each sole's landing room - bright while it swings -, her capture point,
    each arm's reach about its shoulder across her front; none without her law's reading."""
    seen = (now or {}).get('horizon')
    if not seen:
        return []
    heading, (cx, _cy, cz) = float(now.get('heading') or 0.0), seen['com']
    soles = [(s[0], s[2]) for s in seen['soles']]
    out = [(room((cx, cz), soles[1 - k], side, heading, props), ROOM_INK if seen['stance'][k] > 0.5 else SWING_INK)
           for k, side in enumerate((1.0, -1.0))]
    out.append((_ring_segments(seen['xi'][0], seen['xi'][1], CAPTURE_M, props, 8), CAPTURE_INK))
    placed = frames(now.get('angles') or {}, now.get('where', (0.0, 0.9, 0.0)), quat(*now.get('turn', (1, 0, 0, 0))))
    arm, lx, lz = UPPER_ARM + FOREARM, math.cos(heading), -math.sin(heading)
    for side in ('left', 'right'):
        sx, sy, sz = (float(v) for v in placed[side + '_upper_arm'][0])
        ring = [(sx + arm * math.cos(a) * lx, sy + arm * math.sin(a), sz + arm * math.cos(a) * lz)
                for a in (math.tau * k / RING_N for k in range(RING_N + 1))]
        out.append((list(zip(ring, ring[1:])), ARM_INK))
    return [(segs, ink) for segs, ink in out if segs]
