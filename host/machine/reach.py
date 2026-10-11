"""What a leg reaches over its foot, read from static arrays (`reach_rows`): no side in the code.

    r = reach.over(limb, hip, ankle, turn)                  # the hip in its foot's frame, m
    straight, stop = reach.margins(r)                       # m inside its length, its stops
    (ankle, yaw), m = reach.land(limb, hip, heading, want)  # `want` drawn in to its reach
    bend = reach.knee(r)                                    # rad
    gap = reach.clear(one, other)                           # m between two feet on the floor
    reach_rows.SOLE                                         # a sole's corners about its ankle

A limb is an index into LIMBS, its mirror there: whichever stands may land again. A foot's frame
is across to its limb's own side, up, along its toes; a hip's place in it is its ankle's roll
and pitch and its knee, whatever the pelvis's turn - the foot flat or pitched, the same rows
(tools/cores/reach.py). The hip's yaw's stops are told from the pelvis's heading, the pelvis
level. Not in it: her legs above their soles clear of each other; a hip's roll and pitch, no
stop theirs; a foot landing pitched - `land` lays it flat.
"""
import math

from machine.reach_rows import ACROSS, BACK, DU, FOOT, FORE, LIMBS, SHANK, THIGH, U0, YAW


def _run(u):
    """(back, fore) m along the toes the stops leave the hip `u` m up in its leg's plane: between
    rows; over the last, the leg's length alone."""
    at = min(max((u - U0) / DU, 0.0), len(FORE) - 1.001)
    k = int(at)
    return (BACK[k] + (BACK[k + 1] - BACK[k]) * (at - k),
            FORE[k] + (FORE[k + 1] - FORE[k]) * (at - k))


def over(limb, hip, ankle, turn):
    """The hip's place in its foot's frame - across to the limb's own side, up, along the toes -,
    m: the foot's ankle at `ankle`, turned `turn` (a 3x3 of rows), world."""
    d = (hip[0] - ankle[0], hip[1] - ankle[1], hip[2] - ankle[2])
    x, y, z = (turn[0][k] * d[0] + turn[1][k] * d[1] + turn[2][k] * d[2] for k in range(3))
    return LIMBS[limb] * x, y, z


def margins(r, short=0.0):
    """(m inside the leg's length less `short`, m inside its stops: the ankle's roll's across,
    its pitch's and the knee's along) of a hip at `r` (`over`); negative, out by that. Under
    the rows' first, U0 m up - sat on her heels -, out."""
    x, y, z = r
    u = math.hypot(x, y)
    back, fore = _run(u)
    return (THIGH + SHANK - short - math.hypot(u, z),
            min(u - U0, y * ACROSS[1] - x, x + y * ACROSS[0], fore - z, z - back))


def knee(r):
    """The knee's bend, rad, the hip at `r` (`over`)."""
    d2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2]
    return math.acos(max(-1.0, min(1.0, (d2 - THIGH * THIGH - SHANK * SHANK)
                                   / (2.0 * THIGH * SHANK))))


def land(limb, hip, heading, want, short=0.0):
    """((ankle, yaw), m): `want` - (ankle (x, y, z), yaw), world - drawn in to where the limb's
    leg reaches it flat from `hip`, `short` m of its length kept: its yaw to within the hip's
    stops of `heading`, then along the floor to the leg's length, across to the roll's stroke,
    along to the pitch's. m on the floor inside its reach; out of it, how far, negative."""
    (wx, wy, wz), yaw = want
    sign = LIMBS[limb]
    yaw = heading + sign * max(-YAW[0], min(YAW[1], sign * math.remainder(yaw - heading, math.tau)))
    c, s = math.cos(yaw), math.sin(yaw)
    dx, h, dz = hip[0] - wx, hip[1] - wy, hip[2] - wz
    x, z = sign * (dx * c - dz * s), dx * s + dz * c
    far = math.sqrt(max(0.0, (THIGH + SHANK - short) ** 2 - h * h))
    d = math.hypot(x, z)
    k = min(1.0, far / d) if d > 0.0 else 1.0
    ax = max(-h * ACROSS[0], min(h * ACROSS[1], x * k))
    u = math.hypot(ax, h)
    back, fore = _run(u)
    az = max(back, min(fore, z * k))
    inside = min(far - math.hypot(ax, az), u - U0, h * ACROSS[1] - ax, ax + h * ACROSS[0],
                 fore - az, az - back)
    out = math.hypot(ax - x, az - z) + max(0.0, h - (THIGH + SHANK - short)) - min(0.0, inside)
    ankle = (hip[0] - sign * ax * c - az * s, wy, hip[2] + sign * ax * s - az * c)
    return (ankle, yaw), (inside if out <= 0.0 else -out)


def _corners(foot):
    (x, _y, z), yaw = foot
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + px * c + pz * s, z - px * s + pz * c) for px, pz in FOOT]


def _apart(one, other):
    """The widest gap an edge of `one` leaves to `other`'s corners, m; negative, none does."""
    out = -math.inf
    for (x0, z0), (x1, z1) in zip(one, one[1:] + one[:1]):
        out = max(out, min((px - x0) * (z1 - z0) - (pz - z0) * (x1 - x0) for px, pz in other)
                  / math.hypot(x1 - x0, z1 - z0))
    return out


def _to(p, a, b):
    """m from the point `p` to the stretch `a`-`b`."""
    ex, ez = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ex + (p[1] - a[1]) * ez) / (ex * ex + ez * ez)))
    return math.hypot(p[0] - a[0] - t * ex, p[1] - a[1] - t * ez)


def clear(one, other):
    """m between two feet flat on the floor, each (ankle, yaw), heel to toes' tips; negative,
    how deep they overlap."""
    a, b = _corners(one), _corners(other)
    apart = max(_apart(a, b), _apart(b, a))
    if apart <= 0.0:
        return apart
    return min(_to(p, q, r) for these, those in ((a, b), (b, a))
               for p in these for q, r in zip(those, those[1:] + those[:1]))
