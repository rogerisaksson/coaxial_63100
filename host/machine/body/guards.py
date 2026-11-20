"""The robot's guards as built: foam slabs on its limbs and its trunk, each on two struts off a tube.

    guards.LAYOUT        # {'upper_arm': ((from, to, mid, half), ...), ..., 'torso': (...)}: `[guards]`
    guards.slabs(seg)    # [(centre, around, along, half sizes)]: `seg`'s guards as boxes, its frame
    guards.pieces(seg)   # [(kg, centre, 3x3 inertia about it)]: those boxes and their struts
    guards.tubes(seg)    # [(a, b, radius)]: the tubes a strut of `seg`'s stands on

A guard lies where a rider wears one (`[guards]`): down a limb from and to shares of its length, or up
the trunk from and to m, `half` deg either side of `mid` about the axis - a limb's from its outer side,
the trunk's from its left, toward ahead. A guard's face stands SKIN_M over the robot's capsules
(`figure.skin_of`), and its foam, THICK_M under that face, AIR_M clear of the mechanism by it at rest - a
drive's stack or board, a bone (`skeleton.cans`, `drives.boards`, `skeleton.runs`) the segment's axis
meets within NEAR_M. A guard is laid in flat pieces round the axis, none more than half its foam off the
arc it spans, each tilted to the surface's taper down it, KG_M2 a square metre of its face - as much of
it as its outline's superellipse takes of the box (`shape`: boxy, unless its row tells its power, its taper
and its tilt); two solid steel struts STRUT_R round hold each guard, STRUTS of its half width either side of
its middle, from its foam to the segment's nearest tube - or, where the build tells hoops (`HOOPS`), carbon
bars cut flat round behind the segment's guards, each guard clipped on, a leg in from each end.
"""
import math

from machine import libm
from machine.body import drives, mechatronics, skeleton
from machine.body.figure import FOREARM, IDENT, SEGMENTS, SHANK, THIGH, UPPER_ARM, skin_of

_SPEC = mechatronics.robot()['guards']
#: A guard's face with its rim and its rivets, kg a square metre; its foam's thickness and a strut's radius, m.
KG_M2, THICK_M, STRUT_R = float(_SPEC['kg_m2']), float(_SPEC['thick_m']), float(_SPEC['strut_r_m'])
#: The guards a part wears (`[guards]`), (from, to, its middle, its half width, deg) each: a limb's down
#: the limb, shares of its length, the torso's up the torso, m.
LAYOUT = {part: tuple(tuple(float(v) for v in row) for row in rows)
          for part, rows in _SPEC.items() if isinstance(rows, list)}
#: A guard's outline past its four numbers (a row's fifth to seventh, where told): its superellipse's power - POWER
#: a boxy plate, 2 an ellipse -, how much wider its top is than its middle and its foot narrower, a share, and how
#: far its middle line slants up round its axis, deg - a plate along a crease.
POWER = 6.0
#: The hoops a part's guards are held on in place of struts (`[guards] hoops`, where a build tells them): {part:
#: ((its height - up the trunk, m, down a limb, a share of its length as its rows -, from, to, deg round its axis
#: as its guards'), ...)}; a hoop's width out from the axis and its thickness - a bar cut from a stock carbon sheet,
#: flat in its own height -, and how far a guard's back stands off it, its clips' reach, m.
HOOPS = {part: tuple(tuple(float(v) for v in hoop) for hoop in rows) for part, rows in _SPEC.get('hoops', {}).items()}
HOOP_M, CLIP_M = (0.008, 0.004), 0.002
#: How far toward its axis a trunk's guards follow its body's surface, a share of its reach (`[guards] slim`).
SLIM = float(_SPEC.get('slim', 1.0))
#: A guard's face SKIN_M over what it lies on, its foam AIR_M clear of the mechanism the axis meets within
#: NEAR_M, a strut STRUTS of the guard's half width from its middle.
SKIN_M, AIR_M, NEAR_M, STRUTS = 0.002, 0.006, 0.11, 0.45
#: Steel and the carbon laminate, kg/m^3 (`materials.toml`): a strut's, a hoop's.
STEEL_KG_M3, CF_KG_M3 = (float(mechatronics.materials()[k]) for k in ('steel_kg_m3', 'cf_kg_m3'))
#: A limb's length, m: what its guards' shares are of.
LONG = {'upper_arm': UPPER_ARM, 'forearm': FOREARM, 'thigh': THIGH, 'shank': SHANK}
#: A piece is told at ROWS heights from its head to its foot and COLUMNS ways round.
ROWS, COLUMNS = 5, 3

_LAID = {}


def shape(row):
    """(power, taper, tilt) of a `[guards]` row: its fifth to seventh numbers, those it does not tell a box's."""
    told = tuple(row[4:7])
    return told + (POWER, 0.0, 0.0)[len(told):]


def _area(power):
    """A superellipse's share of its box at `power`: its quarter's, summed over a hundred strips (`machine.libm`)."""
    return sum(libm.power(1.0 - libm.power((k + 0.5) / 100.0, power), 1.0 / power) for k in range(100)) / 100.0


def hoops(part):
    """[(its height in its segment's frame, m, from, to, deg)]: the hoops `part`'s guards are held on - a limb's
    heights down it from its shares (`HOOPS`)."""
    long = LONG.get(part)
    return [((-h * long) if long else h, a0, a1) for h, a0, a1 in HOOPS.get(part, ())]


def held(part, top, foot, mid):
    """The height of the hoop a guard of `part` from `top` down to `foot`, m, its middle `mid` deg round, is clipped
    on - its part's that passes behind it -, else None: its struts."""
    return next((h for h, a0, a1 in hoops(part) if foot < h < top and min(a0, a1) <= mid <= max(a0, a1)), None)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a, b):
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def _ball(o, d, c, r):
    """The furthest t at which o + t d (d a unit) leaves the ball `r` round about `c`, else -1."""
    w = _sub(o, c)
    b = _dot(w, d)
    q = b * b - _dot(w, w) + r * r
    return -b + math.sqrt(q) if q >= 0.0 else -1.0


def _barrel(o, d, a, b, r, capped):
    """The furthest t at which o + t d leaves the cylinder `r` round from `a` to `b` - its ends flat where
    `capped`, else open (a capsule's middle) -, else -1."""
    long = math.dist(a, b)
    u = tuple((q - p) / long for p, q in zip(a, b))
    w = _sub(o, a)
    du, wu = _dot(d, u), _dot(w, u)
    dp, wp = tuple(d[k] - du * u[k] for k in range(3)), tuple(w[k] - wu * u[k] for k in range(3))
    best, aa = -1.0, _dot(dp, dp)
    if aa > 1e-12:
        bb = _dot(wp, dp)
        q = bb * bb - aa * (_dot(wp, wp) - r * r)
        t = (-bb + math.sqrt(q)) / aa if q >= 0.0 else -1.0
        best = t if 0.0 <= wu + t * du <= long else -1.0
    for end in ((0.0, long) if capped and abs(du) > 1e-12 else ()):
        t = (end - wu) / du
        off = tuple(wp[k] + t * dp[k] for k in range(3))
        best = max(best, t) if _dot(off, off) <= r * r else best
    return best


def _capsule(o, d, a, b, r):
    return max(_ball(o, d, a, r), _ball(o, d, b, r), _barrel(o, d, a, b, r, False) if a != b else -1.0)


def _rest():
    """{body: (turn, origin)}: each segment's frame at rest in the first's, a stage's its segment's."""
    out = {}
    for name, parent, joints, offset, rest, *_ in SEGMENTS:
        turn, at = out[parent] if parent else (IDENT, (0.0, 0.0, 0.0))
        c, s = math.cos(math.radians(rest)), math.sin(math.radians(rest))
        out[name] = (tuple((row[0] * c + row[1] * s, row[1] * c - row[0] * s, row[2]) for row in turn),
                     tuple(at[k] + _dot(turn[k], offset) for k in range(3)))
        out.update({j: out[name] for j, _a, _s in joints[:-1]})
    return out


def _mechanism():
    """[(rides, a, b, radius, capped)]: what a guard clears, each in the frame it rides - a drive's stack
    part by part and each board a cylinder from `a` to `b`, each limb's bones a capsule."""
    def ends(c, axis, half):
        return tuple(p - v * half for p, v in zip(c, axis)), tuple(p + v * half for p, v in zip(c, axis))
    out = [(rides,) + ends(c, axis, half) + (r, True) for _j, rides, c, axis, r, half in skeleton.cans()]
    out += [(seg,) + ends(at, skeleton.AXES[faces], drives.tall(joint) / 2.0) + (radius, True)
            for joint, (seg, at, _kg, radius, faces) in drives.boards().items()]
    return out + [(side + seg, a, b, r, False) for side, s in (('left_', 1.0), ('right_', -1.0))
                  for seg in skeleton.HUNG for a, b, r in skeleton.runs(seg, s)]


def _by(seg, low, high):
    """[(a, b, radius, capped)]: the mechanism in `seg`'s frame at rest within NEAR_M of its axis between
    the heights `low` and `high`."""
    rest = _rest()
    turn, at = rest[seg]

    def here(rides, p):
        r, o = rest[rides]
        world = tuple(_dot(r[k], p) + o[k] for k in range(3))
        return tuple(sum(turn[i][k] * (world[i] - at[i]) for i in range(3)) for k in range(3))
    out = []
    for rides, a, b, r, capped in _mechanism():
        if rides not in rest:
            continue
        a, b = here(rides, a), here(rides, b)
        mid = tuple((p + q) / 2.0 for p, q in zip(a, b))
        if (min(math.hypot(p[0], p[2]) for p in (a, b, mid)) - r < NEAR_M
                and min(a[1], b[1]) - r < high and max(a[1], b[1]) + r > low):
            out.append((a, b, r, capped))
    return out


def _reach(y, way, skin, hard):
    """How far from the axis a guard's surface lies at the height `y` the `way` out: the capsules `skin`'s
    furthest, or AIR_M and a foam past the mechanism `hard` the way meets within NEAR_M; -1 where neither."""
    o = (0.0, y, 0.0)
    out = max([_capsule(o, way, a, b, r) for a, b, r in skin] + [-1.0])
    for a, b, r, capped in hard:
        t = _barrel(o, way, a, b, r, True) if capped else _capsule(o, way, a, b, r)
        out = max(out, t + AIR_M + THICK_M - SKIN_M) if 0.0 <= t <= NEAR_M else out
    return out


def _box(kg, centre, axes, half):
    """(kg, centre, 3x3 inertia about it) of a box `half` its sizes along `axes`."""
    moments = [kg * (half[(k + 1) % 3] ** 2 + half[(k + 2) % 3] ** 2) / 3.0 for k in range(3)]
    return kg, centre, [[sum(m * e[r] * e[c] for m, e in zip(moments, axes)) for c in range(3)] for r in range(3)]


def _strut(foot, tubes):
    """(kg, centre, 3x3 inertia about it) of a strut from `foot` to the nearest of `tubes`' walls."""
    near = []
    for a, b, r in tubes:
        ab = _sub(b, a)
        t = min(1.0, max(0.0, _dot(_sub(foot, a), ab) / _dot(ab, ab)))
        on = tuple(p + t * v for p, v in zip(a, ab))
        near.append((math.dist(foot, on), on, r))
    apart, on, r = min(near)
    w = tuple(v / apart for v in _sub(foot, on))
    long = max(apart - r, 0.0)
    kg = math.pi * STRUT_R * STRUT_R * long * STEEL_KG_M3
    across, about = kg * (3.0 * STRUT_R * STRUT_R + long * long) / 12.0, kg * STRUT_R * STRUT_R / 2.0
    return kg, tuple(f - v * long / 2.0 for f, v in zip(foot, w)), [
        [(across if i == j else 0.0) + (about - across) * w[i] * w[j] for j in range(3)] for i in range(3)]


def _guard(s, top, foot, am, ah, skin, hard):
    """([(centre, around, along, half sizes, kg)], [a strut's foot]): a guard from the height `top` down to
    `foot`, `ah` rad either side of `am`, in flat pieces round its axis; its x times `s`."""
    def way(a):
        return s * math.cos(a), 0.0, math.sin(a)
    ym = (top + foot) / 2.0
    widest = max(_reach(y, way(a), skin, hard) for y in (top, ym, foot) for a in (am - ah, am, am + ah))
    if widest < 0.0:
        return [], []
    n = max(1, math.ceil(ah / math.acos(1.0 - THICK_M / (2.0 * (widest + SKIN_M)))))
    rows = [top + (foot - top) * i / (ROWS - 1) for i in range(ROWS)]
    out = []
    for k in range(n):
        a0, a1 = am - ah + 2.0 * ah * k / n, am - ah + 2.0 * ah * (k + 1) / n
        ac = (a0 + a1) / 2.0
        told = [[_reach(y, way(a), skin, hard) * math.cos(a - ac) for a in
                 (a0 + (a1 - a0) * j / (COLUMNS - 1) for j in range(COLUMNS))] for y in rows]
        every = [v for row in told for v in row if v >= 0.0]
        if not every:
            continue
        head, base = (max([v for v in told[i] if v >= 0.0] or every) + SKIN_M for i in (0, -1))
        lean = (head - base) / (top - foot)
        face = base + lean * (ym - foot) + max(0.0, max(
            v + SKIN_M - base - lean * (y - foot) for y, row in zip(rows, told) for v in row if v >= 0.0))
        n_, scale = way(ac), math.sqrt(1.0 + lean * lean)
        around = (-s * math.sin(ac), 0.0, math.cos(ac))
        along = (lean * n_[0] / scale, 1.0 / scale, lean * n_[2] / scale)
        out_ = (n_[0] / scale, -lean / scale, n_[2] / scale)
        half = (face * math.tan((a1 - a0) / 2.0), (top - foot) / 2.0 * scale, THICK_M / 2.0)
        centre = tuple(face * n_[i] + (ym if i == 1 else 0.0) - out_[i] * THICK_M / 2.0 for i in range(3))
        out.append((centre, around, along, out_, half, KG_M2 * 4.0 * half[0] * half[1]))
    feet = []
    if len(out) < n:
        return out, feet
    for a in (am - STRUTS * ah, am + STRUTS * ah):
        centre, _r, _l, normal, half, _kg = out[min(len(out) - 1, int((a - am + ah) / (2.0 * ah) * len(out)))]
        inner, d = tuple(c - v * half[2] for c, v in zip(centre, normal)), way(a)
        t = (_dot(inner, normal) - ym * normal[1]) / _dot(d, normal)
        feet.append((t * d[0], ym + t * d[1], t * d[2]))
    return out, feet


def _hoop(s, hoop, skin, hard):
    """[(kg, centre, 3x3 inertia about it)]: a hoop (`HOOPS`) flat at its height round from and to, its middle
    CLIP_M behind its guards' backs, a chord of carbon between each two of its points, and a leg in from each end in
    its own plane to the axis - the hoop and its legs one sheet's cut."""
    y, a0, a1 = hoop[0], math.radians(hoop[1]), math.radians(hoop[2])

    def at(a):
        sine, cosine = libm.sincos(a)
        way = (s * cosine, 0.0, sine)
        r = max(_reach(y, way, skin, hard), 0.0) + SKIN_M - THICK_M - CLIP_M - HOOP_M[0] / 2.0
        return tuple(r * v + (y if k == 1 else 0.0) for k, v in enumerate(way))
    path = [at(a0 + (a1 - a0) * k / 16.0) for k in range(17)]
    legs = [(end, (0.0, y, 0.0)) for end in (path[0], path[-1])]
    out = []
    for p, q in list(zip(path, path[1:])) + legs:
        long = math.dist(p, q)
        if long < 1e-6:
            continue
        along = tuple((b - a) / long for a, b in zip(p, q))
        lift = tuple((1.0 if k == 1 else 0.0) - along[1] * along[k] for k in range(3))
        up = tuple(v / max(math.sqrt(_dot(lift, lift)), 1e-12) for v in lift)
        out_ = (along[1] * up[2] - along[2] * up[1], along[2] * up[0] - along[0] * up[2], along[0] * up[1] - along[1] * up[0])
        out.append(_box(CF_KG_M3 * long * HOOP_M[0] * HOOP_M[1], tuple((a + b) / 2.0 for a, b in zip(p, q)),
                        (along, out_, up), (long / 2.0, HOOP_M[0] / 2.0, HOOP_M[1] / 2.0)))
    return out


def tubes(seg):
    """[(a, b, radius)]: the tubes `seg`'s guards' struts stand on, its frame - a limb's bones
    (`skeleton.runs`), the torso's frame's (`skeleton.trunk`) or, with none, its axis."""
    side, _, limb = seg.partition('_')
    if limb in LONG:
        return skeleton.runs(limb, -1.0 if side == 'right' else 1.0)
    return [shape[1:] for frame, shape in skeleton.trunk() if frame == seg and shape[0] == 'tube'] or [
        ((0.0, 0.0, 0.0), (0.0, 0.38, 0.0), 0.0)]


def _lay(seg):
    """(slabs, pieces) of `seg`: `slabs`' and `pieces`'."""
    side, _, limb = seg.partition('_')
    part, s = (limb if limb in LONG else seg), (-1.0 if side == 'right' else 1.0)
    if part not in LAYOUT:
        return [], []
    rows = []
    for r in LAYOUT[part]:
        top, foot = (-r[0] * LONG[part], -r[1] * LONG[part]) if part in LONG else (r[1], r[0])
        rows.append((top, foot, r[2], r[3], shape(r)))
    skin = [(a, b, r) for r, a, b in skin_of(seg)]
    hard = _by(seg, min(foot for _t, foot, *_ in rows), max(top for top, *_ in rows))
    slabs, pieces = [], []
    for top, foot, mid, half, form in rows:
        laid, feet = _guard(s, top, foot, math.radians(mid), math.radians(half), skin, hard)
        share = _area(form[0]) / _area(POWER)                  # its outline's share of the box laid
        slabs += [(centre, around, along, size) for centre, around, along, _n, size, _kg in laid]
        pieces += [_box(kg * share, centre, (around, along, normal), size)
                   for centre, around, along, normal, size, kg in laid]
        pieces += [] if held(part, top, foot, mid) is not None else [_strut(f, tubes(seg)) for f in feet]
    pieces += [p for hoop in hoops(part) for p in _hoop(s, hoop, skin, hard)]
    return slabs, pieces


def _laid(seg):
    if seg not in _LAID:
        _LAID[seg] = _lay(seg)
    return _LAID[seg]


def slabs(seg):
    """[(centre, around, along, half sizes)]: `seg`'s guards as flat boxes, its frame - each's middle, its
    two axes in its face (round the segment's axis, and down it) and its half sizes along those and through."""
    return _laid(seg)[0]


def pieces(seg):
    """[(kg, centre, 3x3 inertia about it)]: `seg`'s guards' boxes and struts, its frame."""
    return _laid(seg)[1]
