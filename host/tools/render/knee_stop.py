"""The knee's stop: how far it bends till what turns meets what stays, and the lead that frees it.

    from tools.render import knee_stop

`stop` turns the shank's side of the knee as `knee_clamped.bodies` draws it until it comes within the elbow's
CLEAR_M of the thigh's side, and names the two parts that set it; `ahead` halves for the least whole millimetres
the thigh's tube runs ahead of the knee's axis for the knee to bend to the end of its travel - what the build's
spec lays (`leg_clamped.stated`).
"""
import math

from tools.render import elbow_mount as em
from tools.render import knee_clamped as kc
from tools.render import knee_mount as km


def _culprits(still, turning, deg, h):
    """(still part, turning part, m): the two nearest each other with the shank's side turned `deg`."""
    import numpy as np
    best = ('', '', float('inf'))
    for s, tris in still.items():
        a = km._points(tris)
        keys = km._key(np.floor(a / h).astype(np.int64))
        order = np.argsort(keys)
        for t, moving in turning.items():
            d = km._least(keys[order], a[order], km._points(moving) @ km._rx(deg).T, h)
            if d < best[2]:
                best = (s, t, d)
    return best


def stop(ahead=0.0, step=2.5, coarse=4):
    """(the knee's stop, deg, (still part, turning part, m) that set it or None): the last turn from straight at which
    what turns stays CLEAR_M off what stays, up to the end of the knee's travel - looked at every `coarse` steps, then
    step by step after the last clear look."""
    import numpy as np
    from machine.body import drives
    _p, _c, still, turning = kc.bodies(ahead=ahead)
    moving = {k: v for k, v in turning.items() if k != 'output ring'}     # the output ring rides its bearing
    h = 0.006
    a = km._points([t for v in still.values() for t in v])
    keys = km._key(np.floor(a / h).astype(np.int64))
    order = np.argsort(keys)
    pts = km._points([t for v in moving.values() for t in v])
    turns = np.arange(0.0, drives.TRAVEL['knee'][1] + 0.1, step)

    def hit(i):
        return km._least(keys[order], a[order], pts @ km._rx(float(turns[i])).T, h) < em.CLEAR_M
    looks = list(range(0, len(turns), coarse)) + ([len(turns) - 1] if (len(turns) - 1) % coarse else [])
    first = next((i for i in looks if hit(i)), None)
    if first is None:
        return float(turns[-1]), None
    for i in range(max(first - coarse + 1, 0), first + 1):
        if i == first or hit(i):
            return (float(turns[i - 1]) if i else 0.0), _culprits(still, moving, float(turns[i]), h)
    return float(turns[first - 1]), None


_STOPS = {}


def _stopped(lead_mm):
    """`stop` with the bone line `lead_mm` mm ahead, each looked at once."""
    if lead_mm not in _STOPS:
        _STOPS[lead_mm] = stop(lead_mm / 1000.0)
    return _STOPS[lead_mm]


def ahead():
    """(m, deg): the least whole millimetres the bone line runs ahead of the knee's axis for the knee to bend to the
    end of its travel (`drives.TRAVEL`), halving between none and the stator band's own radius - or, short of that
    there, that radius - and the bend it gets."""
    from machine.body import drives
    end = drives.TRAVEL['knee'][1]
    lo, hi = 0, int(math.ceil(1e3 * kc.laid()['stator'][-1][2]))
    if _stopped(lo)[0] >= end or _stopped(hi)[0] < end:
        lead = lo if _stopped(lo)[0] >= end else hi
        return lead / 1000.0, _stopped(lead)[0]
    while hi - lo > 1:
        mid = (lo + hi) // 2
        lo, hi = (lo, mid) if _stopped(mid)[0] >= end else (mid, hi)
    return hi / 1000.0, _stopped(hi)[0]
