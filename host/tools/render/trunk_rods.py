"""The trunk's linkage: a printed clamp on each drive, a rod to the pelvis on two cardan crosses.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/trunk_rods.py

Each drive's crank is a lever as the calf's (`leg_clamped.lever`): a slit band printed round the drive's output
ring, gripping its torque with the pinch screws hid (`sized.band`), and an arm out to an eye at the crank's radius,
printed as one from the band's inner face; through the eye a solid carbon pin, bonded, out to the rod's upper
cross. The rod runs down to its lower cross on a carbon pin bonded into the end of the pelvis's yoke's bar
(`trunk_laid`). Through the trunk's travel the rod stands up to 36 deg out of the pelvis's pin's square and 25 out
of its crank's (`trunk_laid.able`'s 'lean'), more than a spherical eye leans, so each end rides a cardan cross - a
steel collar on a polymer bushing round its pin, two stubs out of it square to the pin and to the rod, a printed
fork's two cheeks on the stubs over polymer bushings, the fork narrowing to its base as wide as the leaves. Two
crosses alone would bind: as the trunk rolls and bows, their stubs turn up to 34 deg apart about the rod (`ends`),
so the lower fork's stem turns in the cartridge's lower block, a swivel on a polymer thrust washer. From that block
to the upper fork's base the calf's leaves, narrowed, side by side fore and aft (`leaves.sizes`); the cartridge
turns with the upper cross, as its crank does. No ball bearing: a full-ceramic one of 12 mm bore - a 6201, 32 mm
round - holds 1.5 to 2.2 kN standing where a pivot here is asked 1.7 kN, and the rod's ends are 26 to 32 mm round.
`ends` gives a rod's ends at a pose, `lever` draws a drive's lever, `rods` both rods, `held` what each part may
bear over what it bears.
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

#: The rods' pins, solid pultruded carbon (`structural.ROD`), their radii - the pelvis's, then the crank's, as thick
#: as the lever's eye's 22 mm of print bears it bonded -; how far in from the lower cross's middle the pelvis's pin's
#: seat's face stands - leant 36 deg in toward the middle and bowed back, the lower fork, washer and block pass over
#: the bar's end, the leaves' width's edge a line beside the rod: 21 mm in they came within 1 mm - and how deep the
#: pin is bonded into the bar's end; m.
PIN_R, LEAN_M, SEAT_M = (0.008, 0.011), 0.026, 0.030
#: A cross: its collar's radius round each end's pin - a 1 mm polymer bushing and 4 mm of steel -, its half length
#: along the pin, its stubs' radius; m.
COLLAR_R, COLLAR_X, STUB_R = (0.013, 0.016), 0.007, 0.005
#: A fork: its cheeks' thickness and play off the collar, their round about the stubs and the polymer bushings' in
#: them, its base's thickness and how far up the rod from its cross's middle the base stands - at the pelvis, at the
#: crank: leant 36 and 25 deg the collars' corners reach 14.6 and 17.5 mm up toward it -; m.
CHEEK_M, PLAY_M, FORK_R, BUSH_R = 0.004, 0.0005, 0.013, 0.007
BASE_M, BASE_UP = 0.008, (0.018, 0.020)
#: The swivel: its stem's radius and its thread's pitch - a steel M8 -, its polymer thrust washer's radius and
#: thickness, the pocket for the stem's nut in the lower block; m.
STEM_R, STEM_P, WASHER_R, WASHER_M, NUT_M = 0.004, 0.00125, 0.008, 0.0015, 0.006
#: A lever: how far round its band the arm's root spreads, deg - as wide as the arm's bending and twist at the
#: margin ask -, the eye's wall round the crank's pin, m, the arm's and the eye's outer face out from the joint's
#: middle, m - leant 25 deg the upper fork's cheeks come to 75.7 mm -, how far in from the gable the band's outer
#: face stands, still on the output ring - leant 35 deg and bowed back near dead centre the leaves' inner edge sweeps
#: the gable's plane 0.7 mm off, 35 to 38 mm round -, and how far off the gearbox's still housing the band's bed, m.
SPREAD, EYE_M, EYE_X, BAND_IN, HOUSE_AIR = 80.0, 0.008, 0.073, 0.002, 0.002
#: The cartridge's width - the calf's leaves narrowed: there a 15 mm cartridge passes the drive's output ring's
#: face 1.5 mm off -, m, and a leaf's bow as a share of its length.
WIDE_M, BOW = 0.0135, 0.095
#: A polymer plain bushing's surface pressure at rest at room temperature, MPa: the makers' usual figure for a
#: standard dry-running polymer bushing, the bushings' and the thrust washer's ink.
BUSHING_MPA, POLYMER = 80.0, (226, 222, 206)


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


@functools.lru_cache(maxsize=None)
def _laid():
    """{'at': the hub's (y, z), 'x0': the band's inner face - the lever's bed -, 'x1': its outer, BAND_IN in from
    the gable, 'grip': how long its bore grips the ring, 'gable', 'g0': the gearbox's inner end, 'ring': the output
    ring's radius, 'wall': the band's, 'pin': the crank's pin's (y, z) at rest, 'rim': the band's outside} of the left
    drive's lever, the chest's frame, m: the band on the ring the gearbox's still housing leaves (`knee_clamped`)."""
    import numpy as np
    from tools.render import knee_clamped as kc
    from tools.render import sized as ks
    g0, g1, ring = kc.laid()['pitch'][0]['gear']
    x0, x1 = g0 + em.OUTPUT_SHARE * (g1 - g0) + HOUSE_AIR, g1 - BAND_IN
    wall, at = 1e-3 * ks.band(ks.torque('ankle'), 1e3 * ring, 1e3 * (x1 - x0)), np.array(tl.HUB[1:])
    return {'at': at, 'x0': x0, 'x1': x1, 'grip': x1 - x0, 'gable': g1, 'g0': g0, 'ring': ring,
            'wall': wall, 'rim': ring + wall - 0.0005, 'pin': at + tl.CRANK[0] * tl._spoke(math.radians(tl.CRANK[1]))[1:]}


def lever():
    """{part: tris} of the left drive's lever at rest, the chest's frame: the drive's output ring, the lever - its
    slit band round the ring and its arm out to the eye, one print - and the carbon pin bonded through the eye."""
    import numpy as np
    from tools.render import clamped as cl
    lay = _laid()
    at, pin, x0, x1, ring = lay['at'], lay['pin'], lay['x0'], lay['x1'], lay['ring']
    bearing = math.atan2(pin[1] - at[1], pin[0] - at[0])
    root = [tuple(at + lay['rim'] * np.array((math.cos(bearing + a), math.sin(bearing + a))))
            for a in np.radians(np.linspace(-0.5 * SPREAD, 0.5 * SPREAD, 9))]
    arm = cl.filleted(leaves.hull(root + leaves.ring(tuple(pin), PIN_R[1] + EYE_M)))
    out = ring + lay['wall']
    band = cl.turned([(x0, ring, out), (x1, ring, out)], em.SLIT)
    g0, gable = lay['g0'], lay['gable']
    return {'ring': em._drum(ring, g0 + em.OUTPUT_SHARE * (gable - g0), gable, em.OUTPUT, tuple(at)),
            'lever': ([(t + (0.0, at[0], at[1]), c) for t, c in band]
                      + em._rounded(arm, x0, EYE_X, em.FRAME, axis=0, r=cl.EDGE_M, steps=1)),
            'pin': m._cylinder_x(PIN_R[1], x0, tl.HUB[0] + COLLAR_X + 0.003, em.CARBON, at=tuple(pin))}


def _put(tris, at, ways):
    """`tris` drawn in a frame of three ways - rows, each a unit -, laid at `at`."""
    import numpy as np
    turn = np.array(ways)
    return [(tri @ turn + at, colour) for tri, colour in tris]


def _cross(at, axis, stubs, r):
    """A cross on its pin: the collar `r` round `axis` about `at`, its two stubs out along `stubs` through the
    cheeks."""
    import numpy as np
    a, c = np.asarray(axis, float), np.asarray(stubs, float)
    reach = r + PLAY_M + CHEEK_M + 0.0015
    tris = m._cylinder_x(r, -COLLAR_X, COLLAR_X, hf.STEEL)
    for way in (1.0, -1.0):
        tris += km._rod((0.0, 0.0, way * (r - 0.002)), (0.0, 0.0, way * reach), STUB_R, hf.STEEL)
    return _put(tris, at, (a, np.cross(c, a), c))


def _fork(at, along, stubs, base, r):
    """A fork on a cross at `at`, its collar `r` round: its two cheeks on the stubs (`stubs` their way) over their
    bushings, narrowing to its base `base` m `along` the rod, as wide as the leaves."""
    import numpy as np
    h, c = np.asarray(along, float), np.asarray(stubs, float)
    inner, wide = r + PLAY_M, 0.5 * WIDE_M
    cheek = leaves.hull(leaves.ring((0.0, 0.0), FORK_R) + [(-wide, base), (wide, base)])
    tris = hf._slab(hf._box((-wide, wide), (base, base + BASE_M)), -inner - CHEEK_M, inner + CHEEK_M, em.FRAME, 2)
    for c0, c1 in ((inner, inner + CHEEK_M), (-inner - CHEEK_M, -inner)):
        tris += hf._slab(cheek, c0, c1, em.FRAME, 2, False)
        tris += km._rod((0.0, 0.0, c0 - 0.0003), (0.0, 0.0, c1 + 0.0003), BUSH_R, POLYMER)
    return _put(tris, at, (np.cross(h, c), h, c))


def _cartridge(ball, up, side, low, top, block):
    """The cartridge from `low` to `top` m up the rod from the pelvis's pin: its lower block, `block` m long, and
    over it the calf's leaves side by side along `side`, bowed either way about the rod's line."""
    import numpy as np
    s = leaves.sizes()
    pitch = s['t'] + leaves.AIR_M
    half = 0.5 * s['n'] * pitch + 0.003

    def slab(outline):
        return foot._slab(np.array(outline), -0.5 * WIDE_M, 0.5 * WIDE_M, em.FRAME)
    tris = slab([(low, -half), (low, half), (low + block, half), (low + block, -half)])
    low, us = low + block, np.linspace(0.0, 1.0, 17)
    for k in range(s['n']):
        mid = (k - 0.5 * (s['n'] - 1)) * pitch
        edge = [(low + u * (top - low), mid + math.copysign(BOW * (top - low), mid) * leaves._bump(u)) for u in us]
        tris += slab([(h, o + 0.5 * s['t']) for h, o in edge] + [(h, o - 0.5 * s['t']) for h, o in edge][::-1])
    return _put(tris, ball, (np.cross(up, side), up, side))


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
        out[(rod, 'lower cross')] = _cross(ball, x, low, COLLAR_R[0])
        out[(rod, 'upper cross')] = _cross(pin, own, high, COLLAR_R[1])
        out[(rod, 'lower fork')] = _fork(ball, up, low, BASE_UP[0], COLLAR_R[0])
        out[(rod, 'upper fork')] = _fork(pin, -up, high, BASE_UP[1], COLLAR_R[1])
        out[(rod, 'swivel')] = (km._rod(over, over + WASHER_M * up, WASHER_R, POLYMER)
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


def _arm(load):
    """The most stress in a lever's arm where it leaves the band, Pa, the rod's pull `load` N on the pin at any way
    across it: bent in the arm's plane and out of it - the pull reaches out along the pin past the arm's middle -,
    pulled along it and twisted (a solid rectangle's twist, as `sized.neck`'s), von Mises."""
    import numpy as np
    lay = _laid()
    w, t = 2.0 * lay['rim'] * math.sin(math.radians(0.5 * SPREAD)), EYE_X - lay['x0']
    reach, out = tl.HUB[0] - 0.5 * (EYE_X + lay['x0']), tl.CRANK[0] - lay['rim']
    lo, hi = sorted((w, t))
    worst_pa = 0.0
    for way in np.linspace(0.0, 0.5 * math.pi, 19):
        across, along = load * math.sin(way), load * math.cos(way)
        bent = 6.0 * across * out / (t * w * w) + 6.0 * along * reach / (w * t * t) + along / (w * t)
        twist = across * reach * (3.0 * hi + 1.8 * lo) / (hi * hi * lo * lo)
        worst_pa = max(worst_pa, math.hypot(bent, math.sqrt(3.0) * twist))
    return worst_pa


def held():
    """{check: what a part may bear over what it bears}, the most a rod is asked on each (`worst`): a lever's band
    gripping the drive's torque (`sized.hooped`), its arm (`_arm`), its eye round the bonded pin - the pin's moment
    and pull on the bore -, the crank's and the pelvis's carbon pins bent where they leave what holds them, the
    pelvis's pin's seat in the bar; a cross's stubs bent and sheared on their cheeks (von Mises); the collars' and the
    cheeks' polymer bushings; the cheeks borne on their bushings and pulled across beside them; the swivel's stem
    pulled through its thread's stress area (ISO 898: d - 0.9382 P across); the printed block under its washer."""
    from machine.body import structural
    from tools.render import sized as ks
    load, steel, solid, rod = worst(), 1e6 * structural.STEEL[2], 1e6 * structural.PAHT_CF[2], 1e6 * structural.ROD[2]
    lay, bushing = _laid(), 1e6 * BUSHING_MPA
    arm, d = PLAY_M + 0.5 * CHEEK_M, 2.0 * STUB_R
    bent, sheared = 32.0 * 0.5 * load * arm / (math.pi * d ** 3), 4.0 / 3.0 * 0.5 * load / (math.pi * STUB_R ** 2)
    crank, eye_l = load * (tl.HUB[0] - EYE_X), EYE_X - lay['x0']
    pelvis = load * LEAN_M

    def seated(moment, r, deep):                         # a bonded pin's most pressure on its bore, Pa
        return 6.0 * moment / (2.0 * r * deep * deep) + load / (2.0 * r * deep)
    return {'band': ks.hooped(ks.torque('ankle'), 1e3 * lay['ring'], 1e3 * lay['grip'], 1e3 * lay['wall']),
            'lever arm': solid / _arm(load),
            'lever eye': solid / seated(crank, PIN_R[1], eye_l),
            'crank pin': rod / (32.0 * crank / (math.pi * (2.0 * PIN_R[1]) ** 3)),
            'pelvis pin': rod / (32.0 * pelvis / (math.pi * (2.0 * PIN_R[0]) ** 3)),
            'pelvis seat': solid / seated(pelvis, PIN_R[0], SEAT_M),
            'stubs': steel / math.hypot(bent, math.sqrt(3.0) * sheared),
            'collar bushings': bushing * 2.0 * PIN_R[0] * 2.0 * COLLAR_X / load,
            'cheek bushings': bushing * 2.0 * 2.0 * STUB_R * CHEEK_M / load,
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
        print('%-16s x%.2f' % (name, ratio))
    return 0


if __name__ == '__main__':
    sys.exit(main())
