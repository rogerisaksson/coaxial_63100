"""The trunk's two rods as built: a cardan cross at each end, a swivel over the lower one.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/trunk_rods.py

A rod runs from its crank's pin on the chest down to its pin on the pelvis (`trunk_laid`). Through the trunk's
travel the rod stands up to 36 deg out of the pelvis's pin's square and 25 out of its crank's (`trunk_laid.able`'s
'lean'), more than a spherical eye leans; so each end rides a cardan cross - a steel collar turning on its pin, two
stubs out of the collar square to the pin and to the rod, a printed fork's two cheeks on the stubs over steel
bushes, the fork narrowing to its base as wide as the leaves, clear of the pin's boss leant. Two crosses alone
would bind: as the trunk rolls and bows, the lower cross's stubs and the upper's turn up to 34 deg apart about the
rod (`ends`), so the lower fork's stem turns in the cartridge's lower block, a swivel on a thrust washer. From that
block to the upper fork's base - the cartridge's upper block - the calf's leaves, side by side fore and aft
(`leaves.sizes`); the cartridge turns with the upper cross, as its crank does. `ends` gives a rod's ends at a pose,
`rods` draws both there, `held` what each part may bear over what it bears; the pins and what holds them are the
trunk's sketch's (`trunk_clamped`).
"""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import elbow_mount as em  # noqa: E402
from tools.render import foot  # noqa: E402
from tools.render import hip_frame as hf  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import leaves  # noqa: E402
from tools.render import mounting as m  # noqa: E402
from tools.render import trunk_laid as tl  # noqa: E402

#: A cross: its collar's radius - a 1 mm steel bush and 4 mm of wall round the 16 mm pin - and half length along
#: the pin, its stubs' radius; m.
COLLAR_R, COLLAR_X, STUB_R = 0.013, 0.007, 0.005
#: A fork: its cheeks' thickness and play off the collar, their round about the stubs and the steel bushes' in
#: them, its base's thickness and how far up the rod from its pin's middle the base stands - at the pelvis's pin,
#: at the crank's: leant 36 deg the collar's corners reach 14.6 mm up toward it -; m.
CHEEK_M, PLAY_M, FORK_R, BUSH_R = 0.004, 0.0005, 0.013, 0.007
BASE_M, BASE_UP = 0.008, (0.018, 0.018)
#: The swivel: its stem's radius and its thread's pitch - an M8 -, its thrust washer's radius and thickness, the
#: pocket for the stem's nut in the lower block; m.
STEM_R, STEM_P, WASHER_R, WASHER_M, NUT_M = 0.004, 0.00125, 0.008, 0.0015, 0.006
#: A leaf's bow as a share of its length, the calf's.
BOW = 0.095


def ends(roll=0.0, pitch=0.0):
    """[(the pelvis's pin, its crank's, the rod's way up, the lower cross's stubs' way, the upper's, how far the
    swivel turns from rest, deg)] the left rod's then the right's, the joint's frame, the trunk at `roll` and
    `pitch`, deg: a cross's stubs square to its pin and to the rod; None where a rod cannot reach."""
    import numpy as np
    got = tl.cranked(roll, pitch)
    if got is None:
        return None
    x = np.array((1.0, 0.0, 0.0))
    own, out = tl.turn(roll, pitch)[1] @ x, []
    for _bearing, pin, ball in got:
        up = (pin - ball) / np.linalg.norm(pin - ball)
        low, high = (np.cross(a, up) / np.linalg.norm(np.cross(a, up)) for a in (x, own))
        out.append((ball, pin, up, low, high,
                    math.degrees(math.atan2(float(np.cross(low, high) @ up), float(low @ high)))))
    return out


def _laid(tris, at, ways):
    """`tris` drawn in a frame of three ways - rows, each a unit -, laid at `at`."""
    import numpy as np
    turn = np.array(ways)
    return [(tri @ turn + at, colour) for tri, colour in tris]


def _cross(at, axis, stubs):
    """A cross on its pin: the collar round `axis` about `at`, its two stubs out along `stubs` through the cheeks."""
    import numpy as np
    a, c = np.asarray(axis, float), np.asarray(stubs, float)
    reach = COLLAR_R + PLAY_M + CHEEK_M + 0.0015
    tris = m._cylinder_x(COLLAR_R, -COLLAR_X, COLLAR_X, hf.STEEL)
    for way in (1.0, -1.0):
        tris += km._rod((0.0, 0.0, way * (COLLAR_R - 0.002)), (0.0, 0.0, way * reach), STUB_R, hf.STEEL)
    return _laid(tris, at, (a, np.cross(c, a), c))


def _fork(at, along, stubs, base):
    """A fork on a cross at `at`: its two cheeks on the stubs (`stubs` their way) over their bushes, narrowing to
    its base `base` m `along` the rod, as wide as the leaves."""
    import numpy as np
    h, c = np.asarray(along, float), np.asarray(stubs, float)
    inner, wide = COLLAR_R + PLAY_M, 0.5 * leaves.sizes()['w']
    cheek = leaves.hull(leaves.ring((0.0, 0.0), FORK_R) + [(-wide, base), (wide, base)])
    tris = hf._slab(hf._box((-wide, wide), (base, base + BASE_M)), -inner - CHEEK_M, inner + CHEEK_M, em.FRAME, 2)
    for c0, c1 in ((inner, inner + CHEEK_M), (-inner - CHEEK_M, -inner)):
        tris += hf._slab(cheek, c0, c1, em.FRAME, 2, False)
        tris += km._rod((0.0, 0.0, c0 - 0.0003), (0.0, 0.0, c1 + 0.0003), BUSH_R, hf.STEEL)
    return _laid(tris, at, (np.cross(h, c), h, c))


def _cartridge(ball, up, side, low, top, block):
    """The cartridge from `low` to `top` m up the rod from the pelvis's pin: its lower block, `block` m long, and
    over it the calf's leaves side by side along `side`, bowed either way about the rod's line."""
    import numpy as np
    s = leaves.sizes()
    pitch = s['t'] + leaves.AIR_M
    half = 0.5 * s['n'] * pitch + 0.003

    def slab(outline):
        return foot._slab(np.array(outline), -0.5 * s['w'], 0.5 * s['w'], em.FRAME)
    tris = slab([(low, -half), (low, half), (low + block, half), (low + block, -half)])
    low, us = low + block, np.linspace(0.0, 1.0, 17)
    for k in range(s['n']):
        mid = (k - 0.5 * (s['n'] - 1)) * pitch
        edge = [(low + u * (top - low), mid + math.copysign(BOW * (top - low), mid) * leaves._bump(u)) for u in us]
        tris += slab([(h, o + 0.5 * s['t']) for h, o in edge] + [(h, o - 0.5 * s['t']) for h, o in edge][::-1])
    return _laid(tris, ball, (np.cross(up, side), up, side))


def rods(roll=0.0, pitch=0.0):
    """{(the rod, its part): [(tri, colour)]} of the two rods at `roll` and `pitch`, deg, the joint's frame: each
    rod's two crosses and two forks, its swivel's washer and stem, its leaves over the lower block."""
    import numpy as np
    got = ends(roll, pitch)
    if got is None:
        raise ValueError('a rod does not reach at roll %.1f, pitch %.1f deg' % (roll, pitch))
    x = np.array((1.0, 0.0, 0.0))
    own, block, out = tl.turn(roll, pitch)[1] @ x, leaves.BLOCK_M + NUT_M, {}
    for side, (ball, pin, up, low, high, _turn) in zip(('left', 'right'), got):
        rod, long = side + ' rod', float(np.linalg.norm(pin - ball))
        over = ball + (BASE_UP[0] + BASE_M) * up                     # the lower fork's base's top face
        out[(rod, 'lower cross')], out[(rod, 'upper cross')] = _cross(ball, x, low), _cross(pin, own, high)
        out[(rod, 'lower fork')] = _fork(ball, up, low, BASE_UP[0])
        out[(rod, 'upper fork')] = _fork(pin, -up, high, BASE_UP[1])
        out[(rod, 'swivel')] = (km._rod(over, over + WASHER_M * up, WASHER_R, hf.STEEL)
                                + km._rod(over, over + (WASHER_M + block) * up, STEM_R, hf.STEEL))
        out[(rod, 'leaves')] = _cartridge(ball, up, high, BASE_UP[0] + BASE_M + WASHER_M,
                                          long - BASE_UP[1] - BASE_M, block)
    return out


@functools.lru_cache(maxsize=None)
def worst():
    """The most a rod is asked, N: a drive's own torque at the margin (`structural.MARGIN`) on its crank's least
    lever over the travel (`trunk_laid.able`'s 'pull')."""
    from machine.body import structural
    return structural.MARGIN * max(v['pull'] for v in tl.reached().values() if v)


def held():
    """{check: what a part may bear over what it bears}, the most a rod is asked on each (`worst`): a cross's
    stubs bent and sheared on their cheeks (von Mises); the cheeks borne on their bushes and pulled across beside
    them; the swivel's stem pulled through its thread's stress area (ISO 898: d - 0.9382 P across); the printed
    block under its washer, pushed."""
    from machine.body import structural
    load, steel, solid = worst(), 1e6 * structural.STEEL[2], 1e6 * structural.PAHT_CF[2]
    arm, d = PLAY_M + 0.5 * CHEEK_M, 2.0 * STUB_R
    bent, sheared = 32.0 * 0.5 * load * arm / (math.pi * d ** 3), 4.0 / 3.0 * 0.5 * load / (math.pi * STUB_R ** 2)
    return {'stubs': steel / math.hypot(bent, math.sqrt(3.0) * sheared),
            'cheeks': solid * 2.0 * 2.0 * BUSH_R * CHEEK_M / load,
            'cheeks across': solid * 2.0 * 2.0 * (FORK_R - BUSH_R) * CHEEK_M / load,
            'stem': steel * 0.25 * math.pi * (2.0 * STEM_R - 0.9382 * STEM_P) ** 2 / load,
            'washer': solid * math.pi * (WASHER_R ** 2 - STEM_R ** 2) / load}


def main():
    import numpy as np
    (r0, r1), (p0, p1) = tl.travel()
    got = [ends(float(a), float(b)) for a in np.linspace(r0, r1, 5) for b in np.linspace(p0, p1, 9)]
    turns = [t for pose in got if pose is not None for *_e, t in pose]
    print('a rod asked %.0f N at the most; its swivel turns %.0f to %.0f deg' % (worst(), min(turns), max(turns)))
    for name, ratio in held().items():
        print('%-14s x%.2f' % (name, ratio))
    return 0


if __name__ == '__main__':
    sys.exit(main())
