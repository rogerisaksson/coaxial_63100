"""The bus's plumbing: black pipes off outlets, T-crosses, bridges and the hoses between fittings.

    tris, end = tees.pipe(at, toward)              # an outlet's pipe bent across the axis, its fitting's end
    tris, end = tees.tee(at, toward, come=way)     # a T-cross on a holder, its marks on its body; tees.mouth(...)
    tris, end, line = tees.fed(at, toward, feed)   # that cross, the trunk into it from an outlet up the axis
    tris = tees.bridge(a, b)                       # a stiff pipe between two outlets too near for a hose
    line = tees.laid(a, u, b, v)                   # a hose lying still between two fittings, bent the least
    tris = tees.hose(line)                         # it drawn; tees.tightest(tris) the least any hose drawn bends at

The robot's bus is one trunk a limb - its power pair, its RS-485 pair and its shield's lead in a hose - from drive to
drive. At each drive the trunk passes a T-cross: the cross's stem stands in the guard's holder, over the board's
bus's terminals, and takes the board's branch - the power pair and the shield's lead a lug each, the RS-485's pair
in to one connector and out again from the other, so the serial link has no stub -; its two arms take the trunk in
and on; its body, a puck over the stem (`body`), holds the arms' bends - the trunk's splices - and carries the
chain's marks (`chain`) on its back: in toward the arm the trunk comes by, out toward the one it goes on from. A
hose bends no tighter than its own least radius (`materials.toml` [hoses]); every tighter turn is a black pipe's.
So where the trunk reaches a cross out of a saddle's channel, the saddle's outlet's pipe and the cross's arm
each lean LEAN off the line between them, away from the joint's axis, the hose a quarter circle round outside the
guard's lid (`fed`); where the two stand too near for that, the cross's arm is a stiff pipe on into the outlet
(`bridge`). A hose lying still along a limb is a cubic from fitting to fitting, its handles as long as bends it the
least tightly (`laid`; through knots, `run`), and clipped: where a hose need follow no turning it is held - a ring
open across from its stem, the hose snapped in, the stem on what carries it (`clip`) - by the fewest clips that leave
no stretch of it free longer than `unheld` tells (`clipped`); only across a joint it hangs free, fitting to fitting.
What is drawn is what is held: a hose's rods' middles are its line again (`lines`), the least radius any of them
bends at `tightest`, the longest stretch one lies free between its ends and its clips `loosest`.
"""
import math

from machine.body import mechatronics
from tools.render import chain
from tools.render import hung
from tools.render import knee_mount as km
from tools.render import mounting as m

#: The hose's radius (`materials.toml` [hoses]) and its corrugation's pitch, m; the drives' magenta, the
#: corrugation's valleys darker.
HOSE_R, PITCH = float(mechatronics.materials()['hoses']['bus'][0]), 0.003
HOSE, HOSE_DEEP = (255, 48, 200), (186, 30, 142)
#: The black pipes: their bends' radius to their middle and their own radius, m, their colour.
PIPE, PIPE_INK = (0.009, 0.0055), (22, 22, 26)
#: The pipe's straight out of its outlet before its bend, and its leg after it, m; the fitting at its end, where the
#: hose goes on: its length and its radius, m.
STUB_M, PIPE_LEG, FITTING = 0.004, 0.008, (0.007, PIPE[1] + 0.0016)
#: How far a trunk's two pipes each lean off the line between them, away from the joint's axis, rad: the hose
#: between their fittings a quarter circle. The handles a laid hose's cubic is tried with: from, to (shares of its
#: fittings' reach) and how many.
LEAN, HANDLES = 0.25 * math.pi, (0.1, 1.2, 45)
#: A pipe's root's collar's radius, m; a T-cross's body's over its stem: as far round as its arms' bends reach.
COLLAR_R, BODY_R = PIPE[1] + 0.0018, PIPE[0]
#: A clip where a hose lies still: the air its ring keeps round the hose, the ring's wall, its width along the hose
#: and its stem's radius, m; the share of the ring open across from its stem, where the hose snaps in; its ink - the
#: guards' black a shade off, what `loosest` knows a clip by -; the farthest a stem reaches to what carries it and
#: how far short of a hose's free end a clip sits, m.
CLIP, OPEN, CLIP_INK, CARRIED_M, SHORT_M = (0.001, 0.0015, 0.004, 0.0015), 0.25, (31, 32, 37), 0.025, 0.015


def hose(points, r=HOSE_R):
    """A slim corrugated robot hose along the path."""
    import numpy as np
    pts = np.asarray(points, float)
    seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    path = np.c_[[np.interp(np.arange(0.0, seg[-1], PITCH), seg, pts[:, k]) for k in range(3)]].T
    tris = []
    for i, (a, b) in enumerate(zip(path[:-1], path[1:])):
        tris += km._rod(a, b, r if i % 2 else 0.84 * r, HOSE if i % 2 else HOSE_DEEP)
    return tris


def outlet(at, way, stub=None):
    """The channel's outlet on a saddle's back face at `at` and the black pipe from it: straight out along the axis
    `stub` (past the stack's back), a 90-degree bend round its corner in toward the axis (`way` along y), a fitting
    at its end where the hose goes on: (triangles, the fitting's end)."""
    import numpy as np
    rb, r = PIPE
    at = np.asarray(at, float)
    out = at + np.array([-(STUB_M if stub is None else stub), 0.0, 0.0])
    centre = out + np.array([0.0, way * rb, 0.0])
    path = [at] + [centre + rb * np.array([-math.sin(a), -way * math.cos(a), 0.0])
                   for a in np.linspace(0.0, 0.5 * math.pi, 13)]
    end = path[-1] + np.array([0.0, way * PIPE_LEG, 0.0])
    path.append(end)
    tris = km._rod(at + np.array([0.0012, 0.0, 0.0]), at - np.array([0.0015, 0.0, 0.0]), COLLAR_R, PIPE_INK)
    for a, b in zip(path[:-1], path[1:]):
        tris += km._rod(a, b, r, PIPE_INK) if np.linalg.norm(b - a) > 1e-9 else []    # no straight: a bend at once
    tris += km._rod(end - np.array([0.0, way * FITTING[0], 0.0]), end, FITTING[1], PIPE_INK)
    return tris, end


def pipe(at, toward, stub=None, mark=None):
    """The outlet at `at` and its pipe (`outlet`) bent toward `toward` (y, z), a unit way across the axis:
    (triangles, the fitting's end); the chain's `mark` ('in', 'out') on its fitting where told, on the side the
    guard's lid is seen from."""
    import numpy as np
    at = np.asarray(at, float)
    turn = km._rx(math.degrees(math.atan2(toward[1], toward[0])))     # from up (+y) to `toward`
    tris, end = outlet(at, 1.0, stub)
    if mark:
        tris = tris + chain.mark(mark, end - (FITTING[1], 0.5 * FITTING[0], 0.0), (-1.0, 0.0, 0.0))
    return [((tri - at) @ turn.T + at, colour) for tri, colour in tris], (end - at) @ turn.T + at


def body(at, ways, kinds=('out', 'in'), stub=None):
    """[(tri, colour)]: a T-cross's body over its stem at `at` (x, y, z): a puck BODY_R round from where its arms'
    bends begin, `stub` out along the axis, back to just past their backs - the bends inside it -, and the chain's
    marks on its back, the side the guard's lid is seen from: each of `kinds` a millimetre inside its edge toward
    its own arm's way (`ways` [(y, z)], units)."""
    import numpy as np
    at = np.asarray(at, float)
    x0 = at[0] - (STUB_M if stub is None else stub)
    back, off = x0 - PIPE[0] - PIPE[1] - chain.PROUD_M, BODY_R - chain.RING_M - 0.001
    tris = m._cylinder_x(BODY_R, back, x0, PIPE_INK, k=40, at=(at[1], at[2]))
    for kind, way in zip(kinds, ways):
        tris += chain.mark(kind, (back, at[1] + off * way[0], at[2] + off * way[1]), (-1.0, 0.0, 0.0),
                           (0.0, way[0], way[1]))
    return tris


def tee(at, toward, stub=None, come=None, marks=('out', 'in')):
    """([(tri, colour)], the fitting's end): a T-cross on the holder at `at` (x, y, z) - its stem out along the axis
    `stub`, its body over that (`body`), one arm out of it `toward` (y, z, a unit) with the fitting the trunk's hose
    goes on from, the other `come`'s way (the opposite unless told) where the trunk comes in (`mouth`); the
    chain's `marks` on the body, the first arm's and the other's - out and in, unless the trunk runs the other way
    through it."""
    come = come or (-toward[0], -toward[1])
    on, end = pipe(at, toward, stub)
    return on + pipe(at, come, stub)[0] + body(at, (toward, come), marks, stub), end


def mouth(at, toward, stub=None, come=None):
    """The end of a T-cross's (`tee`) arm the trunk comes in at."""
    return pipe(at, come or (-toward[0], -toward[1]), stub)[1]


def fed(at, toward, feed, stub=None):
    """([(tri, colour)], the fitting's end, the trunk's hose's line (k, 3)): a T-cross on the holder at `at` (`tee`),
    one arm bent `toward`, and the trunk into its other from the outlet at `feed` (x, y, z) on a face farther up the
    axis - that outlet's pipe out to the cross's arms' plane, it and the cross's arm each leant LEAN off the line
    between them, away from the axis, the hose between their fittings a quarter circle round outside the lid."""
    import numpy as np
    at, feed = np.asarray(at, float), np.asarray(feed, float)
    line = (at[1:] - feed[1:]) / np.linalg.norm(at[1:] - feed[1:])
    out = np.array([-line[1], line[0]])
    out = out if out @ (at[1:] + feed[1:]) > 0.0 else -out               # away from the axis
    leave, come = (tuple(math.cos(LEAN) * way + math.sin(LEAN) * out) for way in (line, -line))
    length = STUB_M if stub is None else stub
    arm, far = pipe(feed, leave, feed[0] - at[0] + length)
    cross, end = tee(at, toward, length, come)
    path = laid(far, (0.0,) + leave, mouth(at, toward, length, come), (0.0,) + come)
    return cross + arm + hose(path), end, path


def bridge(a, b, stub=None):
    """[(tri, colour)]: a stiff black pipe from the outlet at `a` to the one at `b` (x, y, z), each on a face across
    the axis, where they stand too near for a hose at its least radius: straight out of each along the axis - `stub`
    past the one farther back -, a quarter bend toward the other, straight between; a collar round each outlet."""
    import numpy as np
    from machine.errors import MachineError
    rb, r = PIPE
    a, b = np.asarray(a, float), np.asarray(b, float)
    plane = min(a[0], b[0]) - (STUB_M if stub is None else stub) - rb
    apart = float(np.linalg.norm(b[1:] - a[1:]))
    if apart < 2.0 * rb:
        raise MachineError("a bridge's outlets %.4f m apart: no room for its two bends" % apart)
    way, axis = np.r_[0.0, (b[1:] - a[1:]) / apart], np.array([1.0, 0.0, 0.0])
    sides = []
    for at, to in ((a, way), (b, -way)):
        centre = np.array([plane + rb, at[1], at[2]]) + rb * to
        sides.append([at] + [centre - rb * (math.cos(t) * to + math.sin(t) * axis)
                             for t in np.linspace(0.0, 0.5 * math.pi, 13)])
    path = sides[0] + sides[1][::-1]
    tris = [t for at in (a, b) for t in km._rod(at + 0.0012 * axis, at - 0.0015 * axis, COLLAR_R, PIPE_INK)]
    return tris + [t for p, q in zip(path[:-1], path[1:]) if np.linalg.norm(q - p) > 1e-9
                   for t in km._rod(p, q, r, PIPE_INK)]


def laid(a, u, b, v):
    """(k, 3): a hose lying still from a fitting's mouth at `a`, leaving it along `u`, to another's at `b`, leaving
    that one along `v` (units): a cubic between them - of the handles tried (HANDLES) the one that bends it the least
    tightly (`hung._radius`) -, PITCH between its points."""
    import numpy as np
    a, u, b, v = (np.asarray(q, float) for q in (a, u, b, v))
    s = np.linspace(0.0, 1.0, 200)[:, None]

    def cut(k):
        pts = (1 - s) ** 3 * a + 3 * (1 - s) ** 2 * s * (a + k * u) + 3 * (1 - s) * s ** 2 * (b + k * v) + s ** 3 * b
        along = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
        at = np.linspace(0.0, along[-1], max(int(round(along[-1] / PITCH)), 2) + 1)
        return np.c_[[np.interp(at, along, pts[:, j]) for j in range(3)]].T
    reach = float(np.linalg.norm(b - a))
    return max((cut(k) for k in np.linspace(*HANDLES) * reach), key=hung._radius)


def run(knots):
    """(k, 3): a hose lying still through `knots` [(at, heading)]: `laid` from each to the next, leaving the one
    along its heading and reaching the next along the next's."""
    import numpy as np
    legs = [laid(a, u, b, -np.asarray(w, float)) for (a, u), (b, w) in zip(knots[:-1], knots[1:])]
    return np.vstack(legs[:1] + [leg[1:] for leg in legs[1:]])


def unheld():
    """The longest stretch of the bus's hose left free between two holds, m: its first bending mode with its ends
    held (`hoses.FIRST`) no slower than the rate the bus tells the drives at (`gearbox.RATE_S`) - nothing a limb is
    told rings the hose on it."""
    from machine.body import gearbox
    from machine.world import hoses
    _r, mass, bend = mechatronics.materials()['hoses']['bus'][:3]
    return math.sqrt(hoses.FIRST * gearbox.RATE_S / (2.0 * math.pi) * math.sqrt(bend / mass))


def clip(at, along, base, r=HOSE_R):
    """[(tri, colour)]: a clip round the hose at `at` (its way `along`): a ring, a share OPEN of it open across from
    its stem - the hose snaps in -, and the stem out to `base` on what carries it; no stem where the ring itself
    stands on that."""
    import numpy as np
    air, wall, width, stem = CLIP
    at, a = np.asarray(at, float), np.asarray(along, float) / np.linalg.norm(along)
    out = np.asarray(base, float) - at
    out = out - a * float(out @ a)
    far, out = float(np.linalg.norm(out)), out / np.linalg.norm(out)
    turn, half = np.c_[a, out, np.cross(a, out)], OPEN * math.pi       # the ring's x along the hose, its y the stem's
    ring = m._band_x(r + air, r + air + wall, -0.5 * width, 0.5 * width, CLIP_INK, gap=(math.pi - half, math.pi + half))
    tris = [(tri @ turn.T + at, colour) for tri, colour in ring]
    return tris + (km._rod(at + out * (r + air + 0.5 * wall), at + out * far, stem, CLIP_INK)
                   if far > r + air + wall else [])


def clipped(path, parts, r=HOSE_R, free=(False, False)):
    """[(tri, colour)]: the clips that hold a hose lying still along `path` (k, 3) on `parts` [(tri, colour)] - the
    fewest that leave no stretch of it free longer than `unheld` tells: its ends held in their fittings or, an end
    told `free` - the hose going on from there across a joint -, a clip SHORT_M short of it, the rest spread evenly
    between those holds; each stemmed to the nearest of `parts`, CARRIED_M off at the most."""
    import numpy as np
    from scipy.spatial import KDTree
    from machine.errors import MachineError
    path = np.asarray(path, float)
    along = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    lo, hi = (SHORT_M if free[0] else 0.0), along[-1] - (SHORT_M if free[1] else 0.0)
    more = max(int(math.ceil((hi - lo) / unheld() - 1e-9)) - 1, 0)
    stations = [lo] * free[0] + [lo + (hi - lo) * k / (more + 1) for k in range(1, more + 1)] + [hi] * free[1]
    tree, tris = KDTree(km._points(parts, 0.002)), []
    for s in stations:
        i = min(max(int(np.searchsorted(along, s)), 1), len(path) - 2)
        far, j = tree.query(path[i])
        if far > CARRIED_M:
            raise MachineError('a clip %.3f m along a hose finds nothing to stand on within %.3f m' % (s, CARRIED_M))
        tris += clip(path[i], path[i + 1] - path[i - 1], tree.data[j], r)
    return tris


def lines(tris):
    """[(k, 3)]: the centre line of each hose drawn in `tris` [(tri, colour)] - `hose`'s rods by their inks, a rod's
    triangles' middle a point of its line, a new line where two rods stand apart."""
    import numpy as np
    per = len(km._rod(np.zeros(3), np.array([1.0, 0.0, 0.0]), 1.0, HOSE))
    rods = [tri for tri, ink in tris if tuple(ink) in (HOSE, HOSE_DEEP)]
    mids = np.array([np.mean(rods[i:i + per], axis=(0, 1)) for i in range(0, len(rods) - per + 1, per)])
    if len(mids) < 3:
        return []
    cut = np.r_[0, np.nonzero(np.linalg.norm(np.diff(mids, axis=0), axis=1) > 1.5 * PITCH)[0] + 1, len(mids)]
    return [mids[i:j] for i, j in zip(cut[:-1], cut[1:]) if j - i >= 3]


def tightest(tris):
    """The least radius any hose drawn in `tris` bends at (`lines`, `hung._radius`), m; 0 with none drawn."""
    radii = [hung._radius(line) for line in lines(tris)]
    return min(radii) if radii else 0.0


def loosest(tris):
    """The longest stretch any hose drawn in `tris` lies free between two holds - its ends, in their fittings, and
    its clips, known by their ink where their rings stand round it -, m; 0 with no hose drawn."""
    import numpy as np
    from scipy.spatial import KDTree
    rings = [tri for tri, ink in tris if tuple(ink) == CLIP_INK]
    tree, out = (KDTree(np.concatenate(rings)) if rings else None), []
    for line in lines(tris):
        along = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(line, axis=0), axis=1))]
        held = tree.query(line)[0] < HOSE_R + CLIP[0] + CLIP[1] if tree else np.zeros(len(line), bool)
        out.append(float(np.diff(np.r_[0.0, along[held], along[-1]]).max()))
    return max(out) if out else 0.0
