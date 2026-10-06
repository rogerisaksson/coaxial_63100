"""Her kinematics' algebra: a pose a motor, a joint a screw, a chain their product.

    m = motors.of(turn, at)                      # a frame: a 3x3 of rows at a place
    at, turn = motors.place(m), motors.turn(m)
    m = motors.between(a, b, 0.3)                # 0.3 of the screw from a to b
    leg = motors.chain('left_foot')              # off figure.SEGMENTS, from the pelvis's frame
    foot = motors.forward(leg, pelvis, angles)
    angles, left = motors.solve(leg, pelvis, foot, angles)

A motor is a unit dual quaternion (rotor, dual), the dual 0.5 t rotor: the even part of 3D
projective geometric algebra - a turn about a line and a slide along it, a screw. The way
from one pose to another is that screw's six numbers (`log`: its turn vector, then theta m +
d n, m the line's moment about the origin), so a blend, a part of a move and an error to
solve away are one thing. `figure.leg` is `solve`'s closed form for a leg, 8 us against 109
a pass (docs/findings/kinematics.md).
"""
import math

from machine import figure

ONE = ((1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
AXIS = {'x': (1.0, 0.0, 0.0), 'y': (0.0, 1.0, 0.0), 'z': (0.0, 0.0, 1.0)}

#: Under this, rad or its sine, a turn is none: its screw's line is not asked for.
TINY = 1e-9


def _q(a, b):
    """Two quaternions' product, (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def _c(a):
    return (a[0], -a[1], -a[2], -a[3])


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def mul(a, b):
    """The frame `b`, given in `a`'s, in the frame `a` is given in."""
    (ar, ad), (br, bd) = a, b
    return _q(ar, br), tuple(p + q for p, q in zip(_q(ar, bd), _q(ad, br)))


def inv(m):
    return _c(m[0]), _c(m[1])


def of(turn=None, at=(0.0, 0.0, 0.0)):
    """The motor of a frame: a 3x3 `turn` of rows, none unturned, at `at`."""
    if turn is None:
        r = ONE[0]
    else:                                       # Shepperd's: by the largest of its four
        (a, b, c), (d, e, f), (g, h, i) = turn
        k, n = max((a + e + i, 0), (a - e - i, 1), (e - a - i, 2), (i - a - e, 3))
        k = 2.0 * math.sqrt(max(0.0, 1.0 + k))
        r = ((0.25 * k, (h - f) / k, (c - g) / k, (d - b) / k),
             ((h - f) / k, 0.25 * k, (b + d) / k, (c + g) / k),
             ((c - g) / k, (b + d) / k, 0.25 * k, (f + h) / k),
             ((d - b) / k, (c + g) / k, (f + h) / k, 0.25 * k))[n]
    return r, tuple(0.5 * q for q in _q((0.0,) + tuple(at), r))


def turn(m):
    """A motor's turn, a 3x3 of rows."""
    return figure.quat(*m[0])


def place(m):
    """Where a motor puts its frame's origin."""
    t = _q(m[1], _c(m[0]))
    return (2.0 * t[1], 2.0 * t[2], 2.0 * t[3])


def turned(m, v):
    """The vector `v` of a motor's frame, in the frame the motor is given in."""
    return _q(_q(m[0], (0.0,) + tuple(v)), _c(m[0]))[1:]


def moved(m, p):
    """The point `p` of a motor's frame, in the frame the motor is given in."""
    return figure.add(turned(m, p), place(m))


def _screw(n, theta, moment, slide):
    c, s = math.cos(0.5 * theta), math.sin(0.5 * theta)
    return ((c, s * n[0], s * n[1], s * n[2]),
            (-0.5 * slide * s,) + tuple(0.5 * slide * c * a + s * b for a, b in zip(n, moment)))


def about(axis, angle, at=None, slide=0.0):
    """A screw's motor: `angle` rad about the line along the unit `axis` through `at` - none,
    the origin -, slid `slide` m along it."""
    return _screw(axis, angle, (0.0, 0.0, 0.0) if at is None else cross(at, axis), slide)


def log(m):
    """A motor's screw, six numbers: its turn vector, then theta m + d n."""
    (w, x, y, z), (dw, dx, dy, dz) = m
    if w < 0.0:
        w, x, y, z, dw, dx, dy, dz = -w, -x, -y, -z, -dw, -dx, -dy, -dz
    s = math.sqrt(x * x + y * y + z * z)
    if s < TINY:
        return (2.0 * x, 2.0 * y, 2.0 * z, 2.0 * dx, 2.0 * dy, 2.0 * dz)
    k, slide = 2.0 * math.atan2(s, w) / s, -2.0 * dw / s
    return (k * x, k * y, k * z) + tuple(
        k * (d - 0.5 * slide * w * a / s) + slide * a / s for d, a in zip((dx, dy, dz), (x, y, z)))


def exp(screw):
    """The motor of a screw's six numbers (`log`)."""
    wx, wy, wz, vx, vy, vz = screw
    theta = math.sqrt(wx * wx + wy * wy + wz * wz)
    if theta < TINY:
        return (1.0, 0.5 * wx, 0.5 * wy, 0.5 * wz), (0.0, 0.5 * vx, 0.5 * vy, 0.5 * vz)
    n = (wx / theta, wy / theta, wz / theta)
    slide = n[0] * vx + n[1] * vy + n[2] * vz
    return _screw(n, theta, tuple((v - slide * a) / theta for v, a in zip((vx, vy, vz), n)), slide)


def between(a, b, u):
    """The pose `u` of the way from `a` to `b` along the one screw between them."""
    return mul(a, exp(tuple(u * k for k in log(mul(inv(a), b)))))


def chain(end, base='pelvis'):
    """[(joint, the motor before it, its axis)] from `base`'s frame to `end`'s, off
    `figure.SEGMENTS`: a segment's offset and its rest turn before its first joint."""
    by = {seg[0]: seg for seg in figure.SEGMENTS}
    path = []
    while end != base:
        path.append(by[end])
        end = by[end][1]
    out, pre = [], ONE
    for _name, _parent, joints, offset, rest, *_mass in reversed(path):
        pre = mul(pre, of(None, offset))
        if rest:
            pre = mul(pre, about(AXIS['z'], math.radians(rest)))
        for joint, axis, sign in joints:
            out.append((joint, pre, tuple(sign * a for a in AXIS[axis])))
            pre = ONE
    return out


def forward(rows, base, angles, lines=None):
    """The end's motor for `base` and the chain's `angles`, rad; `lines`, a list, takes each
    joint's line: (its axis, a point on it), in the frame `base` is given in."""
    m = base
    for (_joint, pre, axis), q in zip(rows, angles):
        if pre is not ONE:
            m = mul(m, pre)
        if lines is not None:
            lines.append((turned(m, axis), place(m)))
        m = mul(m, about(axis, q))
    return m


def _solved(a, b):
    """x of a x = b, a small square system: Gauss, the largest pivot a column."""
    n = len(b)
    a = [list(row) + [v] for row, v in zip(a, b)]
    for i in range(n):
        p = max(range(i, n), key=lambda r: abs(a[r][i]))
        a[i], a[p] = a[p], a[i]
        for r in range(i + 1, n):
            k = a[r][i] / a[i][i]
            for col in range(i, n + 1):
                a[r][col] -= k * a[i][col]
    x = [0.0] * n
    for i in reversed(range(n)):
        x[i] = (a[i][n] - sum(a[i][col] * x[col] for col in range(i + 1, n))) / a[i][i]
    return x


def solve(rows, base, target, angles, damp=1e-3, passes=2, rest=None, ease=0.0, low=None,
          high=None):
    """(angles, what the last pass found left): the chain's `angles` brought toward the end
    at `target` - `passes` damped least-squares steps on the screw from where it is, its
    turn vector and its place's way. Where the end asks nothing of a joint - damped away
    near a straight knee, or one joint too many - it goes `ease` of its way to `rest` a pass,
    and each stays within `low`..`high`."""
    q, left = list(angles), 0.0
    for _ in range(passes):
        lines = []
        m = forward(rows, base, q, lines)
        end = place(m)
        e = log(mul(target, inv(m)))[:3] + figure.sub(place(target), end)
        left = math.sqrt(sum(k * k for k in e))
        cols = [a + cross(a, figure.sub(end, p)) for a, p in lines]
        jjt = [[sum(c[i] * c[j] for c in cols) + (damp if i == j else 0.0) for j in range(6)]
               for i in range(6)]
        y = _solved(jjt, e)
        step = [sum(c[i] * y[i] for i in range(6)) for c in cols]
        if rest is not None and ease:
            pull = [ease * (r - k) for r, k in zip(rest, q)]
            y = _solved(jjt, [sum(c[i] * z for c, z in zip(cols, pull)) for i in range(6)])
            step = [d + z - sum(c[i] * y[i] for i in range(6))
                    for d, z, c in zip(step, pull, cols)]
        q = [k + d for k, d in zip(q, step)]
        if low is not None:
            q = [max(k, lim) for k, lim in zip(q, low)]
        if high is not None:
            q = [min(k, lim) for k, lim in zip(q, high)]
    return tuple(q), left
