"""Her build as made: carbon-fibre shells over her shape, what they hold, her drives where they sit.

    build.SHELLS = 1.0        # her segments as built (`machine.mjcf`, `figure.com`); 0 a woman's
    build.segments()          # {segment: (kg, centre, (ixx, iyy, izz, ixy, ixz, iyz))}, its frame
    build.mass()              # her weight as built, kg
    build.riders()            # {segment: [(joint, kg, where, (ixx, iyy, izz))]}: the drives on it

A shell is a laminate WALL_M thick over her capsules (`figure.BODY`) and her soles' boxes
(`figure.CONTACTS`), each capsule a thin cylinder and its two caps; what a segment holds besides
(HOLDS) is a point; a drive's assembly a cylinder of its size about its joint's axis.
"""
import math

from machine import drives
from machine.figure import BODY, CONTACTS, SEGMENTS

#: Her body as made or as a woman's (de Leva's shares of `figure.MASS_KG`), 1 or 0.
SHELLS = 1.0

#: The laminate: carbon in epoxy, its wall, m, and density, kg/m^3.
WALL_M, CF_KG_M3 = 0.002, 1600.0

#: What a segment holds besides its shell and its drives, (kg, where in its frame, m) - estimated:
#: in the torso two 15S2P packs of 21700 cells, 48-63 V, 540 Wh each with its BMS and case, one in
#: each flank of her lower ribs (the user, 2026-10-02), and the computer and the harness; the
#: pelvis's power distribution; the head's cameras and IMU; the ankle's rod down the shin
#: (`linkage`); her clothes and her sneakers.
HOLDS = {'torso': ((2.4, (0.065, 0.17, -0.005)), (2.4, (-0.065, 0.17, -0.005)),
                   (1.2, (0.0, 0.26, 0.02)), (0.1, (0.0, 0.2, 0.0))),
         'pelvis': ((0.5, (0.0, 0.0, 0.0)), (0.2, (0.0, -0.02, 0.0))),
         'head': ((0.4, (0.0, 0.09, 0.03)),),
         'upper_arm': ((0.05, (0.0, -0.14, 0.0)),),
         'thigh': ((0.15, (0.0, -0.2, 0.0)),),
         'shank': ((0.06, (0.0, -0.2, -0.03)), (0.05, (0.0, -0.1, 0.0))),
         'foot': ((0.25, (0.0, -0.03, 0.03)),)}

#: Walking she drew 691 W at 0.76 m/s (work 197, copper 424, her boards 70) and 439 at 0.48 (96,
#: 273, 70): 253 Wh a km either way - on the two packs' 972 Wh used (90 %) 3.8 km, on one 1.9;
#: lying limp her 25 boards' 35 W, 28 h (2026-10-02, the build before the gimbal hip).

#: Each limb's quick-release, by its segment: a printed-polymer collar with pogo pins for its power
#: and its bus at its root, clear of the drive above it - the arm's under the shoulder's 70 mm, the
#: leg's under the hip's 100 - (its centre down the segment m, radius, length, kg).
RELEASES = {'upper_arm': (-0.05, 0.03, 0.02, 0.12), 'thigh': (-0.065, 0.04, 0.015, 0.2)}


def _part(name):
    return name.split('_', 1)[1] if name.startswith(('left_', 'right_')) else name


def riders():
    """{segment: [(joint, kg, where in its frame, its inertia's diagonal)]}: each drive's assembly
    where it sits - mounted where `drives.mount` says, else on its joint's axis, a segment's first
    joint's on the parent at the segment's place, a later one's at the segment's own - a cylinder
    of its size's diameter and length about that axis -, and its board (`drives.board`), a disc,
    with it or apart."""
    out = {}
    for name, parent, joints, offset, *_ in SEGMENTS:
        for k, (joint, axis, _sign) in enumerate(joints):
            if drives.passive(joint):
                continue
            size, where = drives.of(joint)[1], drives.mount(joint)
            if where is not None:
                rides, at = where[0], tuple(where[1])
            elif k == 0 and parent is not None:
                rides, at = parent, tuple(offset)
            else:
                rides, at = name, (0.0, 0.0, 0.0)
            kg, long = drives.mass(joint), drives.length(joint)
            about = kg * size.diameter ** 2 / 8.0
            across = kg * (3.0 * size.diameter ** 2 / 4.0 + long ** 2) / 12.0
            inertia = tuple(about if a == axis else across for a in 'xyz')
            out.setdefault(rides, []).append((joint, kg, at, inertia))
            board, place, b_kg, radius, faces = (drives.board(joint) or (rides, at) + drives.BOARD[
                drives.of(joint)[0]] + (axis,))
            flat = b_kg * radius ** 2 / 4.0
            out.setdefault(board, []).append((joint + '_board', b_kg, place, tuple(
                2.0 * flat if a == faces else flat for a in 'xyz')))
    return out


def _shells(part):
    """[(kg, centre, 3x3 inertia about it)] of `part`'s shell pieces."""
    out = []
    for p, r, top, end in BODY:
        if p != part:
            continue
        length = math.dist(top, end)
        mid = tuple((a + b) / 2.0 for a, b in zip(top, end))
        axis = (tuple((b - a) / length for a, b in zip(top, end)) if length > 1e-9
                else (0.0, 1.0, 0.0))
        tube, caps = (2.0 * math.pi * r * length * WALL_M * CF_KG_M3,
                      4.0 * math.pi * r * r * WALL_M * CF_KG_M3)
        along = tube * r * r + caps * 2.0 / 3.0 * r * r
        across = (tube * (r * r / 2.0 + length * length / 12.0)
                  + caps * (2.0 / 3.0 * r * r + length * length / 4.0))
        out.append((tube + caps, mid, [[across + (along - across) * axis[i] * axis[j]
                                        if i == j else (along - across) * axis[i] * axis[j]
                                        for j in range(3)] for i in range(3)]))
    for p, shape, size, at in CONTACTS:
        if p == part and shape == 'box':
            x, y, z = (2.0 * h for h in size)
            kg = 2.0 * (x * y + y * z + x * z) * WALL_M * CF_KG_M3
            out.append((kg, tuple(at), [[kg * (y * y + z * z) / 6.0, 0.0, 0.0],
                                        [0.0, kg * (x * x + z * z) / 6.0, 0.0],
                                        [0.0, 0.0, kg * (x * x + y * y) / 6.0]]))
    return out


def segments():
    """{segment: (kg, centre, (ixx, iyy, izz, ixy, ixz, iyz) about it)}, each in its own frame: its
    shell, what it holds and the drives on it - laid once a wall and a drives' layout
    (`figure.com` asks every pass)."""
    key = (WALL_M, CF_KG_M3, drives.TOES, drives.FINGERS, tuple(drives.JOINTS.items()))
    if key not in _LAID:
        _LAID[key] = _segments()
    return _LAID[key]


_LAID = {}


def _segments():
    on = riders()
    out = {}
    for seg in SEGMENTS:
        name = seg[0]
        pieces = _shells(_part(name))
        pieces += [(kg, at, [[0.0] * 3 for _ in range(3)]) for kg, at in HOLDS.get(_part(name), ())]
        pieces += [(kg, (0.0, y, 0.0), [[0.0] * 3 for _ in range(3)])
                   for y, _r, _l, kg in ([RELEASES[_part(name)]] if _part(name) in RELEASES else [])]
        pieces += [(kg, at, [[i[k] if k == j else 0.0 for j in range(3)] for k in range(3)])
                   for _joint, kg, at, i in on.get(name, ())]
        kg = sum(p[0] for p in pieces)
        centre = tuple(sum(p[0] * p[1][k] for p in pieces) / kg for k in range(3))
        tensor = [[0.0] * 3 for _ in range(3)]
        for m, at, own in pieces:
            d = [at[k] - centre[k] for k in range(3)]
            dd = sum(v * v for v in d)
            for i in range(3):
                for j in range(3):
                    tensor[i][j] += own[i][j] + m * ((dd if i == j else 0.0) - d[i] * d[j])
        out[name] = (kg, centre, (tensor[0][0], tensor[1][1], tensor[2][2], tensor[0][1],
                                  tensor[0][2], tensor[1][2]))
    return out


def mass():
    """Her weight as built, kg."""
    return sum(kg for kg, _c, _i in segments().values())
