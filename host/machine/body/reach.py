"""What a leg reaches over its foot, from static arrays or the build's figure: no side in the code.

    r = reach.over(limb, hip, ankle, turn)                  # the hip in its foot's frame, m
    straight, stop = reach.margins(r)                       # m inside its length, its stops
    (ankle, yaw), m = reach.land(limb, hip, heading, want)  # `want` drawn in to its reach
    bend = reach.knee(r)                                    # rad
    gap = reach.clear(one, other)                           # m between two feet on the floor
    reach.SOLE                                              # a sole's corners about its ankle

A limb is an index into LIMBS, its mirror there: whichever stands may land again. A foot's frame
is across to its limb's own side, up, along its toes; a hip's place in it is its ankle's roll
and pitch and its knee, whatever the pelvis's turn - the foot flat or pitched, the same rows
(`reach_rows` on `mechatronics.STOCK`, compiled from the robot's figure on another build,
`tools/cores/reach.py`). The hip's yaw's stops are told from the pelvis's heading, the pelvis
level. Not in it: the robot's legs above their soles clear of each other; a hip's roll and pitch, no
stop theirs; a foot landing pitched - `land` lays it flat.
"""
import math

from machine.body import drives, figure, mechatronics
from machine.body.figure import BALL, HEEL, HIP_DROP, HIP_HALF, SHANK, THIGH, TOE_M

#: The rows: the hip from U0 m up in its leg's plane, DU apart; a row scanned SCAN_M apart, its
#: run's ends bisected.
_U0_RAW, _DU_RAW, SCAN_M = (round(0.10 * figure.SCALE, 6), round(0.005 * figure.SCALE, 6),
                            round(0.001 * figure.SCALE, 6))
LENGTH = THIGH + SHANK


def stops():
    """{kind: (low, high) rad} of a leg's travel from its hip's yaw down (`drives.TRAVEL`): the hip's pitch
    and roll turn the robot's pelvis over the leg, not the foot's frame."""
    return {k: tuple(math.radians(v) for v in drives.TRAVEL[k])
            for k in ('hip_yaw', 'knee', 'ankle', 'ankle_roll')}


def bend(r):
    """(knee, ankle, ankle_roll) rad of the robot's left leg, its hip at `r` in its foot's frame."""
    return figure.leg(1.0, (r[0] - HIP_HALF, r[1] + HIP_DROP, r[2]), figure.IDENT, (0.0, 0.0, 0.0),
                      figure.IDENT)[3:]


def held(r, limits):
    """Whether the robot's left leg, its hip at `r` in its foot's frame, is within its length and
    `limits` (`stops`)."""
    return math.hypot(*r) <= LENGTH + 1e-12 and all(
        limits[k][0] <= q <= limits[k][1] for k, q in zip(('knee', 'ankle', 'ankle_roll'), bend(r)))


def run(u, limits):
    """(back, fore) m along the toes: the longest run the robot's hip stands in, `u` m up in its leg's
    plane; an end the leg's length alone bounds, -+LENGTH."""
    edge = math.sqrt(LENGTH * LENGTH - u * u)
    n = int(edge / SCAN_M)
    ok = [held((0.0, u, k * SCAN_M), limits) for k in range(-n, n + 1)]
    best, k = (0, 0), 0
    while k < len(ok):
        j = k
        while j < len(ok) and ok[j]:
            j += 1
        best, k = max(best, (j - k, k)), j + 1
    count, first = best
    if not count:
        raise ValueError('nothing reached %g m up' % u)

    def end(k, step):
        inside, outside = k * SCAN_M, (k + step) * SCAN_M if abs(k + step) <= n else step * edge
        if abs(k) == n and held((0.0, u, step * edge), limits):
            return math.copysign(LENGTH, step)
        for _ in range(10):
            mid = 0.5 * (inside + outside)
            inside, outside = (mid, outside) if held((0.0, u, mid), limits) else (inside, mid)
        return inside

    return end(first - n, -1), end(first + count - 1 - n, 1)


def sole(toes):
    """A sole's corners (across, along) m about its ankle, counter-clockwise: heel to ball, or
    on to its toes' tips; alike either side of its length."""
    _seg, _shape, (half, _hy, _hz), (x, _y, _z) = figure.CONTACTS[0]
    _seg, _shape, (toe_half, _ty, _tz), (toe_x, _y, _z) = figure.CONTACTS[1]
    if x or toe_x or (toes and toe_half != half):
        raise ValueError('a sole not alike either side: its mirror is not in the rows')
    front = BALL + (TOE_M if toes else 0.0)
    return ((-half, -HEEL), (half, -HEEL), (half, front), (-half, front))


def rows():
    """What machine/body/reach_rows.py holds, {name: value}."""
    limits = stops()
    count = int(round((LENGTH - _U0_RAW) / _DU_RAW))
    true = [run(_U0_RAW + _DU_RAW * k, limits) for k in range(count)]
    edge = [math.sqrt(LENGTH * LENGTH - u * u) for u in (_U0_RAW + _DU_RAW * k for k in range(count))]
    near = [range(max(0, k - 1), min(count, k + 2)) for k in range(count)]
    back = [-LENGTH if all(true[j][0] <= -LENGTH for j in near[k]) else
            math.ceil(max(max(true[j][0], -edge[j]) for j in near[k]) * 1e4) / 1e4
            for k in range(count)]
    fore = [LENGTH if all(true[j][1] >= LENGTH for j in near[k]) else
            math.floor(min(min(true[j][1], edge[j]) for j in near[k]) * 1e4) / 1e4
            for k in range(count)]
    free = [b <= -LENGTH and f >= LENGTH for b, f in zip(back, fore)]
    top = free.index(True)
    if not all(free[top:]):
        raise ValueError("a bound over the rows' top, %g m up" % (_U0_RAW + _DU_RAW * top))
    yaw, roll = limits['hip_yaw'], limits['ankle_roll']
    return {'LIMBS': (1.0, -1.0), 'THIGH': THIGH, 'SHANK': SHANK,
            'YAW': tuple(math.floor(abs(v) * 1e6) / 1e6 for v in yaw),
            'ACROSS': tuple(math.floor(math.tan(abs(v)) * 1e6) / 1e6 for v in roll),
            'U0': _U0_RAW, 'DU': _DU_RAW, 'BACK': tuple(back[:top + 1]), 'FORE': tuple(fore[:top + 1]),
            'SOLE': sole(False), 'FOOT': sole(True)}


if mechatronics.ROBOT == mechatronics.STOCK:
    from machine.body.reach_rows import ACROSS, BACK, DU, FOOT, FORE, LIMBS, SOLE, U0, YAW
else:
    _LAID = mechatronics.compiled('reach_' + mechatronics.ROBOT,
                                  mechatronics.robot_sources() + (__file__,), rows)
    LIMBS, YAW, ACROSS = _LAID['LIMBS'], _LAID['YAW'], _LAID['ACROSS']
    U0, DU, BACK, FORE = _LAID['U0'], _LAID['DU'], _LAID['BACK'], _LAID['FORE']
    SOLE, FOOT = _LAID['SOLE'], _LAID['FOOT']


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
    the rows' first, U0 m up - sat on the robot's heels -, out."""
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


def land(limb, hip, heading, want, short=0.0, pitch=0.0):
    """((ankle, yaw), m): `want` - (ankle (x, y, z), yaw), world - drawn in to where the limb's
    leg reaches it from `hip`, `short` m of its length kept, pitched `pitch` rad: its yaw to
    within the hip's stops of `heading`, then along the floor to the leg's length, across to the
    roll's stroke, along to the pitch's. m on the floor inside its reach; out of it, how far,
    negative."""
    (wx, wy, wz), yaw = want
    sign = LIMBS[limb]
    yaw = heading + sign * max(-YAW[0], min(YAW[1], sign * math.remainder(yaw - heading, math.tau)))
    c, s = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    dx, h, dz = hip[0] - wx, hip[1] - wy, hip[2] - wz
    x, z0 = sign * (dx * c - dz * s), dx * s + dz * c
    y, z = h * cp - z0 * sp, h * sp + z0 * cp
    far = math.sqrt(max(0.0, (THIGH + SHANK - short) ** 2 - y * y))
    d = math.hypot(x, z)
    k = min(1.0, far / d) if d > 0.0 else 1.0
    ax = max(-y * ACROSS[0], min(y * ACROSS[1], x * k))
    u = math.hypot(ax, y)
    back, fore = _run(u)
    az = max(back, min(fore, z * k))
    inside = min(far - math.hypot(ax, az), u - U0, y * ACROSS[1] - ax, ax + y * ACROSS[0],
                 fore - az, az - back)
    out = math.hypot(ax - x, az - z) + max(0.0, y - (THIGH + SHANK - short)) - min(0.0, inside)
    az0 = az * cp - y * sp
    ankle = (hip[0] - sign * ax * c - az0 * s, wy, hip[2] + sign * ax * s - az0 * c)
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
    overlapping by that."""
    a, b = _corners(one), _corners(other)
    gap = max(_apart(a, b), _apart(b, a))
    if gap <= 0.0:
        return gap
    return min(min(_to(p, q0, q1) for p in a for q0, q1 in zip(b, b[1:] + b[:1])),
               min(_to(p, q0, q1) for p in b for q0, q1 in zip(a, a[1:] + a[:1])))
