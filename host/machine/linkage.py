"""Her transmissions off a joint's axis: the ankle's rod, the hip roll's spur pair, the belts.

The ankle's rod a four-bar - a crank on its drive, a rigid carbon rod on two ball joints, the
heel's horn -; the arms' and the toes' toothed belts.

    linkage.lever('left_ankle', 10.0)    # d crank / d joint at 10 deg: the rod's ratio there
    linkage.crank('left_ankle', 10.0)    # the crank's angle from its rest, rad
    linkage.RODS, linkage.BELTS          # {kind: geometry}; linkage.GEARS, ratios

A rod lies in its joint's plane, its drive's segment's frame about the joint: up (y) and ahead (z),
m. Its crank's pin is its hub plus its radius (-cos t, sin t); its ball, on the segment the joint
turns, b (-cos beta, sin beta) turned with it; its length theirs at rest, held. The crank's angle
solves that length on the branch its rest lies on - no dead point over the stroke it was laid for.
"""
import functools
import math

#: The ankle's: its hub 210 mm up the shank from the ankle on the shin's axis, on its drive's
#: right-angle stage, 170-270 mm under the knee; a 44 mm crank; the ball 46 mm behind the ankle
#: at the heel's tuberosity - at 33 and 34.5, the hub kept, the lever's curve moved a percent and
#: the walk from the squat fell at 13 s. Searched over -50..45 deg: the lever 1.04-1.17, rising
#: to the push-off, its transmission 39 deg at worst; flat at 1.0 42; the rod 3.7 kN at 171 N m
#: (2026-10-02). (up, ahead, crank, crank's rest rad, ball, ball's rest rad), its stroke.
RODS = {'ankle': ((0.210, -0.004, 0.044, math.radians(268.0), 0.046, math.radians(270.0)),
                  (-50.0, 45.0))}

#: A rod's crank's plane and its ball's, x of its left limb's frame, m (the right's mirrored): the
#: ankle's crank inside its leg - outside, the thigh's boards met it sat back on her heels, 15 mm,
#: and found no place of their own; its ball on the heel's back 20 mm in. Straight inside the leg
#: the rod stood 51 mm out of her shell and 15 out of her jeans (the user, 2026-10-02).
ROD_OUT, BALL_OUT = -0.06, -0.02

#: A rod's bend, laid at rest in its left joint's frame as RODS' (x across), m, carried with its
#: ends (`bent`): the ankle's from its crank round the back of its drive and down the calf's
#: inner back, 2 mm clear of the drive at full dorsiflexion - in its middle the folded femur met
#: it, 2-16 mm (`tools/sim/fit.py`).
BENDS = {'ankle': (-0.045, 0.18, -0.076)}

#: Each belt, toothed steel cord: its drive's pulley's radius and its joint's, m - the ratio
#: theirs, constant - and how far out to her side of the limb's axis it runs. None where its give
#: under her weight would show in her walk - the knee's 141 N m was a 3.4 kN pull (the user,
#: 2026-10-02). The elbow's from its M under the arm's quick-release, its 165 degrees past a
#: rod's; the wrist's and the toes' from their S.
BELTS = {'elbow': (0.018, 0.018, 0.034), 'wrist': (0.012, 0.012, 0.026),
         'foot': (0.010, 0.010, 0.03)}

#: Each spur pair after its drive's gearbox, its ratio: the gearbox takes its joint's torque over
#: it. The hip roll's M at 1:100 on two stages put its 126 N m peak through a box rated 100, and
#: walking 133-139 (`World.geared`, 2026-10-02): one stage at 1:60 and 1.67 here.
GEARS = {'hip_roll': 1.67}


@functools.lru_cache(None)
def _bend(kind):
    """A rod's bend in the frame its ends make at rest: along it, across it, the third."""
    up, ahead, r, t0, b, beta = RODS[kind][0]
    pin = (ROD_OUT, up - r * math.cos(t0), ahead + r * math.sin(t0))
    ball = (BALL_OUT, -b * math.cos(beta), b * math.sin(beta))
    return _local(BENDS[kind], pin, ball, (1.0, 0.0, 0.0), 1.0)


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


def _kind(joint):
    from machine.drives import kind
    return kind(joint)


@functools.lru_cache(None)
def _laid(kind):
    """(rest length, branch) of a rod: the crank's angle at rest is on `branch`, +1 or -1."""
    up, ahead, r, t0, b, beta = RODS[kind][0]
    by, bz = -b * math.cos(beta), b * math.sin(beta)
    py, pz = up - r * math.cos(t0), ahead + r * math.sin(t0)
    length = math.hypot(py - by, pz - bz)
    roots = [_root(RODS[kind][0], length, 0.0, s) for s in (1.0, -1.0)]
    branch = min((1.0, -1.0), key=lambda s: abs(math.remainder(roots[s < 0] - t0, math.tau)))
    return length, branch


def _root(rod, length, q, branch):
    """The crank's angle keeping `length` with the joint at `q` rad, on `branch`."""
    up, ahead, r, _t0, b, beta = rod
    by0, bz0 = -b * math.cos(beta), b * math.sin(beta)
    c, s = math.cos(q), math.sin(q)
    dy, dz = by0 * c - bz0 * s - up, by0 * s + bz0 * c - ahead
    k = (r * r + dy * dy + dz * dz - length * length) / (2.0 * r)
    h = math.hypot(dy, dz)
    return math.atan2(dz, -dy) + branch * math.acos(max(-1.0, min(1.0, k / h)))


def crank(joint, deg):
    """The crank's angle from its rest, rad, for the joint at `deg`."""
    kind = _kind(joint)
    rod = RODS[kind][0]
    length, branch = _laid(kind)
    return math.remainder(_root(rod, length, math.radians(deg), branch) - rod[3], math.tau)


def lever(joint, deg=0.0):
    """d crank / d joint at `deg`: a rod's ratio there, a belt's or a spur pair's always, 1 on
    the joint's axis."""
    kind = _kind(joint)
    if kind in GEARS:
        return GEARS[kind]
    if kind in BELTS:
        return BELTS[kind][1] / BELTS[kind][0]
    if kind not in RODS:
        return 1.0
    step = 0.05
    return abs(crank(joint, deg + step) - crank(joint, deg - step)) / math.radians(2.0 * step)
