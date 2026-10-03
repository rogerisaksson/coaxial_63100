"""Her transmissions off a joint's axis: the ankle's two rods, the hip roll's spur pair, the belts.

A rod a four-bar - a crank on its drive, a rigid carbon rod on two ball joints, the heel's horn
-; the ankle a parallel pair (PAIRS), its two drives' rods turning its pitch together and its roll
against each other; the arms' and the toes' toothed belts.

    linkage.lever('left_ankle', 10.0)    # d crank / d joint at 10 deg: the rods' ratio there
    linkage.crank('left_ankle_roll', 10.0)  # a rod's crank's angle from its rest, its ball's
                                         # joint at 10 deg, rad
    linkage.RODS, linkage.BELTS          # {drive's kind: geometry}; linkage.GEARS, ratios

A rod lies in its joint's plane, its drive's segment's frame about the joint: up (y) and ahead (z),
m. Its crank's pin is its hub plus its radius (-cos t, sin t); its ball, on the segment the joint
turns, b (-cos beta, sin beta) turned with it; its length theirs at rest, held. The crank's angle
solves that length on the branch its rest lies on - no dead point over the stroke it was laid for.
"""
import functools
import math

#: Each rod by its drive's kind, from its crank at the front of the knee through the shin to the
#: heel's back, crossed: (up, ahead, crank, crank's rest rad, ball, ball's rest rad) about its
#: ball's joint, m, its stroke - the ankle's two M 80 and 150 mm under the knee. Searched inside her over
#: -50..45 deg rolled +-15 (2026-10-02): the upper's lever 0.73-0.90, the lower's 0.76-0.92,
#: their transmission 40.7 and 39.2 deg at worst, 1.4 and 1.7 mm clear of the drums and the tibia,
#: 1.5 and 1.4 inside her shell standing; a 48 mm crank on a 48 mm horn stood 29 mm out of her
#: ankle - in its 32-36 mm round any rod's line passes the axis within 25.
RODS = {'ankle': ((0.300, 0.015, 0.020, math.radians(90.0), 0.018, math.radians(255.0)),
                  (-50.0, 45.0)),
        'ankle_roll': ((0.230, 0.000, 0.024, math.radians(90.0), 0.022, math.radians(255.0)),
                       (-50.0, 45.0))}

#: Each rod's ball's joint, its crank's plane and its ball's: x of its left limb's frame, m (the
#: right's mirrored) - each crank on its drum's outer end, each ball on the heel's horn beside
#: the tibia's end. Straight inside the leg one rod stood 51 mm out of her shell and 15 out of
#: her jeans, its crank 50-60 mm in met the other leg's walking (2026-10-02).
ROD_AT = {'ankle': ('ankle', 0.045, 0.012), 'ankle_roll': ('ankle', -0.047, -0.016)}

#: A parallel pair: a joint and its roll, two drives turning both - the first through their rods'
#: levers together, the second through their offsets over their cranks against each other.
PAIRS = {'ankle': 'ankle_roll'}

#: A rod's radius, m: solid carbon, 12 mm round and 0.3 m long it buckles at 7 kN; a drive's 88 N m
#: on its 20 mm crank is 4.4 kN, 6.8 at the transmission's worst, pulling at the push-off
#: (estimated) - 18 mm round the rods stood 26-29 mm out of her ankle.
ROD_R = 0.006

#: A rod's bend, laid at rest in its left joint's frame as RODS' (x across), m, carried with its
#: ends (`bent`): down its crank's plane past the drums, then in to its ball - straight, each
#: rod's run past its own drum's end face, the crank no longer than the drum's radius.
BENDS = {'ankle': (0.045, 0.180, 0.0139), 'ankle_roll': (-0.047, 0.205, 0.0193)}

#: Each belt, toothed steel cord: its drive's pulley's radius and its joint's, m - the ratio
#: theirs, constant - and how far out to her side of the limb's axis it runs. None where its give
#: under her weight would show in her walk - the knee's 141 N m was a 3.4 kN pull (the user,
#: 2026-10-02). The elbow's from its M under the arm's quick-release, its 165 degrees past a
#: rod's; the wrist's and the toes' from their S.
BELTS = {'elbow': (0.018, 0.018, 0.034), 'wrist': (0.012, 0.012, 0.026),
         'foot': (0.010, 0.010, 0.03)}

#: Each spur pair after its drive's gearbox, its ratio: the gearbox takes its joint's torque over
#: it. The hip roll's M at 1:100 on two stages put its 126 N m peak through a box rated 100, and
#: walking 133-139 (`World.geared`, 2026-10-02): one stage at 1:60 and 1.67 here. The trunk's
#: roll's M 100 mm under its axis, 1:1, the wheel 7 mm before the pitch's L, 13 mm off it and the
#: hips' yaw drums rolled 35 deg (2026-10-03).
GEARS = {'hip_roll': 1.67, 'spine_roll': 1.0}

#: Each bevel pair after its drive's gearbox, 1:1, its pitch radius, m: a right-angle stage, a
#: motorcycle's shaft drive's, turning an output along its limb onto its joint's axis - across her
#: arms the elbow's M and the wrist's S stood 3-26 mm out of her shell, their belts 8-27, along
#: them 1 mm in (2026-10-03). Each BEVEL_T thick, BEVEL_EFF of the torque through it (estimated).
BEVELS = {'elbow': 0.012, 'wrist': 0.009}
BEVEL_T, BEVEL_EFF = 0.006, 0.97

#: A rod's lever tabled every GRID deg over SPAN, interpolated: worked out each step the ankle's
#: pair cost 34 us a leg against the world's 184 (2026-10-02).
GRID, SPAN = 0.5, 90.0


@functools.lru_cache(None)
def _bend(kind):
    """A rod's bend in the frame its ends make at rest: along it, across it, the third."""
    up, ahead, r, t0 = RODS[kind][0][:4]
    pin = (ROD_AT[kind][1], up - r * math.cos(t0), ahead + r * math.sin(t0))
    return _local(BENDS[kind], pin, ball(kind), (1.0, 0.0, 0.0), 1.0)


def _frame(pin, ball, across, side):
    d = [b - a for a, b in zip(pin, ball)]
    n = math.sqrt(sum(v * v for v in d))
    e1 = [v / n for v in d]
    k = sum(a * e for a, e in zip(across, e1))
    e2 = [a - k * e for a, e in zip(across, e1)]
    n = math.sqrt(sum(v * v for v in e2))
    e2 = [v / n for v in e2]
    e3 = [side * (e1[1] * e2[2] - e1[2] * e2[1]), side * (e1[2] * e2[0] - e1[0] * e2[2]),
          side * (e1[0] * e2[1] - e1[1] * e2[0])]
    return e1, e2, e3


def _local(point, pin, ball, across, side):
    rel = [p - a for p, a in zip(point, pin)]
    return tuple(sum(r * e for r, e in zip(rel, axis)) for axis in _frame(pin, ball, across, side))


def bent(kind, pin, ball, across, side):
    """A rod's bend, world, from its ends `pin` and `ball` and its limb's x `across` (unit), `side`
    1 her left and -1 her right: its rest's place (`BENDS`) in the frame they make now."""
    frame = _frame(pin, ball, across, side)
    return tuple(p + sum(c * axis[i] for c, axis in zip(_bend(kind), frame))
                 for i, p in enumerate(pin))


def joints(kind):
    """The joints of a kind: her left's and her right's, or the trunk's one."""
    from machine.figure import JOINTS
    return [j for j in ('left_' + kind, 'right_' + kind, kind) if j in JOINTS]


def _kind(joint):
    from machine.drives import kind
    return kind(joint)


def ball(kind, q=0.0, roll=0.0):
    """A rod's ball about its joint, (x, y, z) of its left limb's frame, m: its joint at `q`,
    that joint's roll (`PAIRS`) at `roll`, rad."""
    b, beta = RODS[kind][0][4:6]
    x, y, z = ROD_AT[kind][2], -b * math.cos(beta), b * math.sin(beta)
    x, y = x * math.cos(roll) - y * math.sin(roll), x * math.sin(roll) + y * math.cos(roll)
    return x, y * math.cos(q) - z * math.sin(q), y * math.sin(q) + z * math.cos(q)


@functools.lru_cache(None)
def _laid(kind):
    """(rest length, branch) of a rod: the crank's angle at rest is on `branch`, +1 or -1."""
    up, ahead, r, t0 = RODS[kind][0][:4]
    _x, by, bz = ball(kind)
    length = math.hypot(up - r * math.cos(t0) - by, ahead + r * math.sin(t0) - bz)
    roots = [_root(kind, length, 0.0, 0.0, s) for s in (1.0, -1.0)]
    branch = min((1.0, -1.0), key=lambda s: abs(math.remainder(roots[s < 0] - t0, math.tau)))
    return length, branch


def _root(kind, length, q, roll, branch):
    """The crank's angle keeping `length` with its ball at `q` and `roll` rad, on `branch`: the
    rod out of the crank's plane as far as its ball rolls."""
    up, ahead, r = RODS[kind][0][:3]
    x, by, bz = ball(kind, q, roll)
    dy, dz = by - up, bz - ahead
    k = (r * r + dy * dy + dz * dz - length * length + (x - ROD_AT[kind][1]) ** 2) / (2.0 * r)
    h = math.hypot(dy, dz)
    return math.atan2(dz, -dy) + branch * math.acos(max(-1.0, min(1.0, k / h)))


def crank(joint, deg, roll=0.0):
    """A rod's crank's angle from its rest, rad: its drive's `joint`'s rod, its ball's joint
    (`ROD_AT`) at `deg` and that joint's roll at `roll`."""
    kind = _kind(joint)
    length, branch = _laid(kind)
    return math.remainder(_root(kind, length, math.radians(deg), math.radians(roll), branch)
                          - RODS[kind][0][3], math.tau)


def turns(joint, angles):
    """(deg, roll deg) a rod's ball rides, its drive's `joint`'s, of `angles` {joint: deg}."""
    kind = _kind(joint)
    side, at = joint[:-len(kind)], ROD_AT[kind][0]
    return angles.get(side + at, 0.0), angles.get(side + PAIRS[at], 0.0) if at in PAIRS else 0.0


def lever(joint, deg=0.0):
    """d crank / d joint at `deg`: a rod's ratio there, a pair's its rods' mean, a belt's or a
    spur pair's always, 1 on the joint's axis."""
    kind = _kind(joint)
    if kind in GEARS:
        return GEARS[kind]
    if kind in BELTS:
        return BELTS[kind][1] / BELTS[kind][0]
    table = _table(kind)
    at = min(max((deg + SPAN) / GRID, 0.0), len(table) - 1.001)
    k = int(at)
    return table[k] + (table[k + 1] - table[k]) * (at - k)


@functools.lru_cache(None)
def _table(kind):
    return [_lever(kind, k * GRID - SPAN) for k in range(int(2.0 * SPAN / GRID) + 1)]


def _lever(kind, deg):
    rods = [r for r, (at, *_x) in ROD_AT.items() if kind in (at, PAIRS.get(at))]
    if not rods:
        return 1.0
    rolled, step = kind in PAIRS.values(), 0.05
    turned = [((0.0, d) if rolled else (d,)) for d in (deg + step, deg - step)]
    return sum(abs(crank(r, *turned[0]) - crank(r, *turned[1]))
               for r in rods) / math.radians(2.0 * step) / len(rods)
