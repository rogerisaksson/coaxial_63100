"""The ankle's two rods as built: a printed cartridge of leaves, a fork above it and an eye below.

    pin, bend, ball, turn = leaves.ends('ankle', 30.0, 5.0)   # m in the knee's frame, rad: the ankle at 30 and 5 deg
    leaf, low, up = leaves.laid('ankle', 30.0, 5.0)           # [(tri, colour)] each, there
    leaves.held()                                             # {check: what it may bear over what it bears}

A rod's ball rides the heel's shaft as the foot carries it (`linkage.ball`); its lever turns to keep the rod's
length (`ends`). Both rods are alike (`[leaf]`): a cartridge printed flat in one piece - two blocks, between them
thin leaves side by side, half bowed one way about the rod's line and half the other, every root round - and a
spine up its middle. Pulled, the leaves straighten until they lie on the spine's flanks, shaped as the leaves
themselves at their allowable stress; pushed, they bow until the spine's end meets the upper block on its pad
(`stops`): the leaves never bend past what the print bears, whatever the rod is asked. Under the cartridge a print
of its own, laid on its side: a stem and an eye square to the shaft - a spherical bearing in it, the foot's roll
its tilt. Over it the head, a third: a neck down the drive's outer side, set back behind the lever's band, and a
fork round the lever's lug. `reach` tells what the rods ask of the fork over the ankle and the tube's feet.
"""
import functools
import math

from tools.render import clamped as cl
from tools.render import elbow_mount as em
from tools.render import foot
from tools.render import knee_mount as km

#: An eye's radius round its ball and its half width along the shaft; the ball's radius, the shaft's and the pins', m.
EYE_M, EYE_X, BALL_M, SHAFT_M = 0.011, 0.004, 0.007, 0.004
#: From the ball up to the cartridge's lower block, a block's length, the stem's half depth ahead of and behind
#: the rod's line and the bearing's seat in the eye - its radius and its width, m.
LOW_M, BLOCK_M, STEM_H, SEAT_M = 0.040, 0.010, 0.006, (0.008, 0.005)
#: The cartridge, as deep either side of the rod's line: its innermost leaves' roots off that line, the air between
#: two leaves side by side, a root's round, the pad's thickness on the spine, m.
ROOT_M, AIR_M, ROUND_M, PAD_M = 0.005, 0.0017, 0.003, 0.001
#: The head: its fork's cheeks' thickness, how far a face of the lever's lug stands off the pin with its play and
#: that play, the cheeks' radius round the pin, the steel bushes' in them, how far under the pin they join; the neck
#: under it: its width in from the fork's outer face - beside the band round the drive's motor, never behind it -,
#: its depth back from the rod's line, how far down it runs so before it comes in over the cartridge, m.
CHEEK_M, GAP_X, PLAY_M, FORK_R, BUSH_R, FORK_M = 0.003, 0.004, 0.0005, 0.009, 0.005, 0.018
NECK_W, HEAD_T, STRAIGHT_M = 0.017, 0.016, 0.066
#: The inks: the pins' and the balls' steel, the pads' soft polymer.
STEEL, PAD = (200, 204, 212), foot.TPU
KINDS = ('ankle', 'ankle_roll')


def sphere(c, r, colour, k=10):
    """A ball `r` round about `c`."""
    import numpy as np
    rows = [[np.asarray(c, float) + r * np.array([math.sin(t) * math.cos(p), math.cos(t), math.sin(t) * math.sin(p)])
             for p in np.linspace(0.0, 2.0 * math.pi, 2 * k + 1)] for t in np.linspace(0.0, math.pi, k + 1)]
    tris = []
    for i in range(k):
        for j in range(2 * k):
            a, b, cc, d = rows[i][j], rows[i][j + 1], rows[i + 1][j + 1], rows[i + 1][j]
            tris += [(np.array((a, b, cc)), colour), (np.array((a, cc, d)), colour)]
    return tris


def hull(points):
    """The convex hull of 2-d points, its outline in order round."""
    import numpy as np
    from scipy.spatial import ConvexHull
    pts = np.asarray(points, float)
    return pts[ConvexHull(pts).vertices]


def ring(c, r, k=24):
    return [(c[0] + r * math.cos(t), c[1] + r * math.sin(t)) for t in [2.0 * math.pi * i / k for i in range(k)]]


@functools.lru_cache(maxsize=None)
def sizes():
    """{'n', 'w', 't', 'a', 'l', 'e', 'allow'}: the cartridge as `[leaf]` states it - how many leaves, each one's
    width and thickness, their bow and their length, m - and its print's modulus and allowable stress, Pa."""
    from machine.body import mechatronics, structural
    leaf = mechatronics.robot()['leaf']
    solid = structural._SOLIDS[leaf['material']]
    return {'n': int(leaf['leaves']), 'w': float(leaf['width_m']), 't': float(leaf['thick_m']),
            'a': float(leaf['arch_m']), 'l': float(leaf['length_m']), 'e': 1e9 * float(solid[0]),
            'allow': 1e6 * float(solid[2])}


@functools.lru_cache(maxsize=None)
def worst():
    """The most a rod is asked, N: half the pitch drives' torque at the margin (`sized.torque`) on the rod's least
    arm - its lever at the travel's end - with the roll's on the balls' half spacing, both at once."""
    from machine.body import drives, linkage
    from tools.render import sized
    arm = linkage.RODS['ankle'][0][2] * math.cos(math.radians(max(abs(t) for t in drives.TRAVEL['ankle'])))
    return 1e-3 * sized.torque('ankle') / (2.0 * arm) + 1e-3 * sized.torque('ankle_roll') / (
        2.0 * abs(linkage.ROD_AT['ankle'][2]))


@functools.lru_cache(maxsize=None)
def stops():
    """{'pull', 'push': (the leaves' bow there m, the rod's stroke m, its force N)}: where the leaves lie on the
    spine, pulled - their bending with the most a rod is asked (`worst`) across them at the print's allowable -,
    and where the spine's end meets the upper block, pushed - their bending with their own push there at it. A leaf
    held at both roots, a bump between: its bending E t pi^2 (a - a0) / l^2, the rod's force 4 n pi^2 E I / l^2
    ln(a0 / a), its stroke pi^2 (a0^2 - a^2) / 4 l."""
    s = sizes()
    n, w, t, a0, l, e, allow = (s[k] for k in ('n', 'w', 't', 'a', 'l', 'e', 'allow'))
    bend, force = e * t * math.pi ** 2 / l ** 2, 4.0 * n * math.pi ** 2 * e * w * t ** 3 / 12.0 / l ** 2
    pull = a0 - (allow - worst() / (n * w * t)) / bend
    push = a0
    for _round in range(20):                               # its own push across the leaves: a fixed point
        push = a0 + (allow - force * math.log(push / a0) / (n * w * t)) / bend
    return {'pull': (pull, math.pi ** 2 * (a0 ** 2 - pull ** 2) / (4.0 * l), force * math.log(a0 / pull)),
            'push': (push, math.pi ** 2 * (push ** 2 - a0 ** 2) / (4.0 * l), force * math.log(push / a0))}


def held():
    """{check: what a part may bear over what it bears}, the most a rod is asked on each (`worst`): the spine as a
    post, pushed - squashed at its root, and buckling, its root held -; the head's neck, pulled off the rod's line
    as far as it stands beside the drive; a pin in its fork's two cheeks, sheared and borne on their bushes; the
    stem across its section; the eye on its bearing's seat. The leaves' own is their stops' (`stops`)."""
    from machine.body import structural
    s, load = sizes(), worst()
    steel, post = 1e6 * structural.STEEL[2], 2.0 * (ROOT_M - 0.5 * s['t'] - PAD_M)
    tall = s['l'] - stops()['push'][1]
    spine = 2.0 * (ROOT_M + stops()['pull'][0] - 0.5 * s['t'] - PAD_M)        # across, where it is widest
    off = STRAIGHT_M * abs(_lean()) - (0.5 * NECK_W - GAP_X - CHEEK_M)        # the neck's middle off the rod's line
    return {'spine root': s['allow'] * post * s['w'] / load,
            'spine post': 2.05 * math.pi ** 2 * s['e'] * s['w'] * (0.5 * (post + spine)) ** 3 / 12.0 / tall ** 2 / load,
            'neck': s['allow'] / (load / (NECK_W * HEAD_T) + 6.0 * load * off / (HEAD_T * NECK_W ** 2)),
            'pin': steel / math.sqrt(3.0) * 2.0 * math.pi * SHAFT_M ** 2 / load,
            'cheeks': s['allow'] * 2.0 * 2.0 * BUSH_R * CHEEK_M / load,
            'stem': s['allow'] * 2.0 * EYE_X * 2.0 * STEM_H / load,
            'eye': s['allow'] * 2.0 * SEAT_M[0] * SEAT_M[1] / load}


def _lean():
    """How far a rod's line comes in toward the leg's middle a metre down it, at rest."""
    from machine.body import skeleton
    _seg, _hub, pin, _bend, ball = skeleton._rod('left_', 'ankle')
    return (abs(pin[0]) - abs(ball[0])) / math.dist(pin, ball)


def ends(kind, pitch=0.0, roll=0.0):
    """(pin, bend, ball, turn) of a rod, m in the knee's frame and rad, the ankle at `pitch` and `roll`, deg: its
    ball as the foot carries it (`linkage.ball`), its lever's pin where the lever, turned `turn` from its rest, keeps
    the length the rod is drawn with at rest (`skeleton._rod`), its bend where its ends then put it."""
    from machine.body import linkage, skeleton
    seg, hub, pin, _bend, ball = skeleton._rod('left_', kind)
    r, t0 = linkage.RODS[kind][0][2:4]
    t = linkage._root(kind, math.dist(pin, ball), math.radians(pitch), math.radians(roll), linkage._laid(kind)[1])
    at = (pin[0], hub[1] - r * math.cos(t), hub[2] + r * math.sin(t))
    to = tuple(o + v for o, v in zip(seg[3], linkage.ball(kind, math.radians(pitch), math.radians(roll))))
    return at, linkage.bent(kind, at, to, (1.0, 0.0, 0.0), 1.0), to, math.remainder(t - t0, math.tau)


def frame(kind, pitch=0.0, roll=0.0):
    """(its ball, its three ways - back from its line, up it, out across it toward the leg's outer side -, its
    length) of a rod, the knee's frame, the ankle at `pitch` and `roll`, deg: what its prints are laid in."""
    import numpy as np
    pin, _bend, ball, _turn = ends(kind, pitch, roll)
    out, at, span = math.copysign(1.0, pin[0]), np.array(ball), math.dist(pin, ball)
    along = (np.array(pin) - at) / span
    side = np.array((out, 0.0, 0.0)) - out * along[0] * along
    side /= np.linalg.norm(side)
    return at, np.array([out * np.cross(along, side), along, side]), span


def _bump(u):
    """A leaf's bow a share `u` of the way up it, its most 1: flat into both its roots."""
    return math.sin(math.pi * u) ** 2


def laid(kind, pitch=0.0, roll=0.0, lug=None, bow=None):
    """(leaf, low, up), [(tri, colour)] each: a rod's cartridge - its two blocks, its leaves bowed `bow` (else as
    printed), its spine and its pad -, its lower end - the stem, the eye on the heel's shaft, its ball - and its
    head - the neck, the fork round the lever's lug, its bushes and its pin -, the ankle at `pitch` and `roll`, deg.
    Each laid along the rod's line from its ball, across it toward the leg's outer side and back from it; the
    fork's cheeks square to x, PLAY_M off the lug's two faces `lug` (x, its inner and its outer), else GAP_X."""
    import numpy as np
    pin, _bend, ball, _turn = ends(kind, pitch, roll)
    s, out = sizes(), math.copysign(1.0, pin[0])
    n, w, t, a0, long = s['n'], s['w'], s['t'], s['a'] if bow is None else bow, s['l']
    at, axes, span = frame(kind, pitch, roll)
    along, side = axes[1], axes[2]

    def slab(outline, h0, h1, colour=em.FRAME):
        """A profile [(up the rod, out)] from `h0` to `h1` back from the rod's line."""
        return [(tri @ axes + at, c) for tri, c in foot._slab(outline, h0, h1, colour)]
    half, pitch_m = n // 2, t + AIR_M
    wide = ROOT_M + (half - 1) * pitch_m + 0.5 * t + 0.002           # a block's half width
    base, top = LOW_M + BLOCK_M, LOW_M + BLOCK_M + long
    face, deep = -0.5 * w, 0.5 * w
    leaf = slab([(LOW_M, -wide), (LOW_M, wide), (base, wide), (base, -wide)], face, deep)
    leaf += slab([(top, -wide), (top, wide), (top + BLOCK_M, wide), (top + BLOCK_M, -wide)], face, deep)
    us = np.linspace(0.0, 1.0, 33)
    for way in (1.0, -1.0):
        for j in range(half):
            mid = [way * (ROOT_M + j * pitch_m + a0 * _bump(u)) for u in us]
            fat = [0.5 * t + ROUND_M - math.sqrt(ROUND_M ** 2 - (ROUND_M - d) ** 2)
                   for d in (min(u * long, (1.0 - u) * long, ROUND_M) for u in us)]    # round into both roots
            leaf += slab([(base + u * long, m + f) for u, m, f in zip(us, mid, fat)]
                         + [(base + u * long, m - f) for u, m, f in zip(us, mid, fat)][::-1], face, deep)
    pull, push = stops()['pull'][0], stops()['push'][1]
    ups = np.linspace(0.0, 1.0 - push / long, 25)
    flank = [ROOT_M + pull * _bump(u) - 0.5 * t - PAD_M for u in ups]
    leaf += slab([(base + u * long, f) for u, f in zip(ups, flank)]
                 + [(base + u * long, -f) for u, f in zip(ups, flank)][::-1], face, deep)
    end = base + ups[-1] * long
    leaf += slab([(end, -flank[-1]), (end, flank[-1]), (end + PAD_M, flank[-1]), (end + PAD_M, -flank[-1])],
                 face, deep, PAD)
    # The lower end, a print of its own laid on its side: an eye square to the shaft and a stem as wide, up the
    # rod's line into the lower block.
    low = slab([(0.6 * EYE_M, -EYE_X), (0.6 * EYE_M, EYE_X), (LOW_M, EYE_X), (LOW_M, -EYE_X)], -STEM_H, STEM_H)
    low += km._rod(tuple(at - EYE_X * axes[2]), tuple(at + EYE_X * axes[2]), EYE_M, em.FRAME)
    low += sphere(ball, BALL_M, STEEL)
    # The head, a print of its own: the neck beside the drive, set back, in over the cartridge; the fork on it.
    inner, outer = lug or (pin[0] - out * (GAP_X - PLAY_M), pin[0] + out * (GAP_X - PLAY_M))
    reach = abs(outer - pin[0]) + PLAY_M + CHEEK_M
    neck, down = top + BLOCK_M, np.linspace(0.0, 1.0, 17)

    def middle(u):
        """How far out of the rod's line the neck's middle stands `u` m up it: square under the fork, then in."""
        straight = (pin[0] + out * (reach - 0.5 * NECK_W) - (at + u * along)[0]) / side[0]
        share = min(1.0, max(0.0, (span - STRAIGHT_M - u) / (span - STRAIGHT_M - neck)))
        return straight * 0.5 * (1.0 + math.cos(math.pi * share))
    path = [neck + f * (span - FORK_M - neck) for f in down]
    width = [0.5 * NECK_W + (wide - 0.5 * NECK_W) * 0.5 * (1.0 - math.cos(math.pi * min(1.0, max(
        0.0, (span - STRAIGHT_M - u) / (span - STRAIGHT_M - neck))))) for u in path]
    up = slab([(u, middle(u) + h) for u, h in zip(path, width)]
              + [(u, middle(u) - h) for u, h in zip(path, width)][::-1], 0.0, HEAD_T)
    feet = [(pin[1] - FORK_M, pin[2]), (pin[1] - FORK_M, pin[2] - HEAD_T), (pin[1] + 0.4 * FORK_R, pin[2] - HEAD_T)]
    cheek = cl.filleted(hull(ring(pin[1:], FORK_R) + feet))
    for face_x, way in ((inner, -out), (outer, out)):
        x0, x1 = sorted((face_x + way * PLAY_M, face_x + way * (PLAY_M + CHEEK_M)))
        up += em._rounded(cheek, x0, x1, em.FRAME, axis=0, r=0.001, steps=1)
        up += km._rod((x0, pin[1], pin[2]), (x1, pin[1], pin[2]), BUSH_R, STEEL)
    up += km._rod((inner - out * (PLAY_M + CHEEK_M), pin[1], pin[2]), (pin[0] + out * (reach + 0.0015), pin[1], pin[2]),
                  SHAFT_M, STEEL)
    up += km._rod((pin[0] + out * reach, pin[1], pin[2]), (pin[0] + out * (reach + 0.0015), pin[1], pin[2]),
                  1.6 * SHAFT_M, STEEL)
    return leaf, low, up


@functools.lru_cache(maxsize=None)
def reach():
    """(top, ahead), m: how high over the ankle's axis the lower ends' eyes rise and how far ahead of it the rods
    then come above them - their stems, their cartridges -, the toes farthest down and the foot rolled either way
    and not: what the fork over the ankle and the tube's two feet keep off (`leg_clamped.under`,
    `knee_clamped.shank_line`)."""
    import numpy as np
    from machine.body import drives
    from machine.body.figure import SHANK
    top, ahead, (up, down) = -1.0, -1.0, drives.TRAVEL['ankle']
    for roll in tuple(drives.TRAVEL['ankle_roll']) + (0.0,):
        for kind in KINDS:
            top = max(top, ends(kind, down, roll)[2][1] + SHANK + EYE_M)
            for pitch in (up, down):                       # the rods stand nearest the tube at the travel's ends
                ball = ends(kind, pitch, roll)[2]
                leaf, stem, _head = laid(kind, pitch, roll)
                low = np.concatenate([tri for tri, _c in leaf + stem])
                ahead = max(ahead, float(low[low[:, 1] > ball[1] + EYE_M][:, 2].max()))
    return top, ahead
