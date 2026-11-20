"""The trunk, a first draft, sketched: the pelvis's column, the drive pair, the cross, the fork.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/trunk_clamped.py OUT.png

`trunk_laid` lays it and tells what the pair turns; here its parts are drawn, the joint's frame, a printed part
a print (PRINTS: the way each grows off its bed). What stands with the pelvis: a web between the two hips' yaw
drives' bands in the column their yokes leave free; a column on it, its head cupped under the pair's cage; the
calf's two drives; the clamp - the cage round their inverters and, off a 45 degree cone, the band round the left
drive's motor -; the collar - the right drive's band, coned out to the cage's rim -; the tower - a saddle over the
cage, its back sloped clear of the chest's bosses, and on it a housing round the cross's roll journal, behind the
cross: nothing ahead of the joint, the chest bows over it. What rolls: the cross - a block, its journal back into the tower, its pitch pin across. What rolls
and bows with the chest: the fork - the seat the spine's tube's foot is screwed on, an arm either side down to
its eye on the pitch pin and behind each arm a wall, back and out to the boss round the chest's pin: the chest's
cross -, the pins, the tube's foot. What each drive turns: its output ring, a plate on it outboard of the gable
and the crank's pin; from that pin to the chest's a rod as the calf's - an eye either end, a block by each, the
calf's leaves between (`leaves.sizes`), side by side fore and aft: the rod no wider across than a leaf, clear of
its crank's plate and of the chest's boss. `posed` turns it all to a pose, `aired` tells the least air between what
moves apart there - and, apart, the neck's and the head's boards' bay's on the chest's tube (`neck_mount`).
"""
import argparse
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import clamped as cl  # noqa: E402
from tools.render import elbow_mount as em  # noqa: E402
from tools.render import foot  # noqa: E402
from tools.render import hip_clamped as hc  # noqa: E402
from tools.render import hip_frame as hf  # noqa: E402
from tools.render import hip_laid as hl  # noqa: E402
from tools.render import knee_clamped as kc  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import leaves  # noqa: E402
from tools.render import mounting as m  # noqa: E402
from tools.render import neck_laid as nl  # noqa: E402
from tools.render import sized as ks  # noqa: E402
from tools.render import trunk_laid as tl  # noqa: E402

#: The cross: its block's half side, its roll journal's radius, its pitch pin's and that pin's reach either side.
BLOCK_M, JOURNAL_R, PIN_R, PIN_X = 0.017, 0.010, 0.008, 0.038
#: The pelvis's tower round the roll's journal: its housing's radius - inside the chest's fork's arms however the
#: chest stands -, its z behind the block (from, to), the saddle's top under what the block sweeps as it rolls,
#: its foot's and its saddle's half width - inside where the fork's walls come, the chest leant and bowed back; m.
TOWER_R, TOWER_Z, SADDLE_Y, FOOT_X = 0.019, (-0.062, -0.021), -0.027, 0.016
#: The chest's fork: its arms' inner face off the middle and their thickness, an arm's eye's radius and its half
#: depth, the seat's under face - over what the tower's housing sweeps under it - and its top, the bed; a wall's
#: thickness, its under edge - over the tower's foot as the chest leans and bows back - and the share of its
#: length that edge runs level before it slopes down to the boss; how far in from a rod's eye's middle the
#: chest's boss's face stands - what the rod's end sweeps toward it, leant 37 deg (`trunk_laid.able`): its
#: block's inner edge 21 mm up it, and 2 mm of air; m.
ARM_X, ARM_M, EYE_R, ARM_Z, BRIDGE = 0.021, 0.010, 0.012, 0.012, (0.042, 0.056)
WALL_M, WALL_UP, WALL_LEVEL, LEAN_M = 0.013, 0.010, 0.6, 0.021
#: A crank: its plate's thickness and air off its drive's gable, its boss's radius round the hub and its eye's
#: round the pin - the chest's pin's boss's too -, the pin's radius - 3.5 kN at the margin 21 mm out of its boss
#: bends a 16 mm pin 180 MPa -, a rod's eye's half width on a pin; m.
PLATE_M, GABLE_AIR, BOSS_R, PIN_EYE, ROD_PIN, ROD_EYE = 0.0065, 0.002, 0.030, 0.014, 0.008, 0.007
#: The column off the pelvis: its half width and its z (from, to), the web's z between the yaw drives' bands and
#: its reach into a band's wall, m; a leaf's bow as a share of its length, the calf's.
COLUMN_X, COLUMN_Z, WEB_Z, WEB_IN, BOW = 0.030, (-0.030, 0.018), (0.0, 0.018), 0.0025, 0.095
#: What touches by design, left out of `aired`: the journal in the tower, a drive's ring in its housing, a pin
#: in its rod's eye.
MATED = ('roll journal', 'ring', 'cross pins')
#: The printed parts, each a print: (its link, the way it grows off its bed in the joint's frame).
PRINTS = {'web': ('pelvis', (0.0, 0.0, 1.0)), 'column': ('pelvis', (0.0, 0.0, -1.0)),
          'clamp': ('pelvis', (-1.0, 0.0, 0.0)), 'collar': ('pelvis', (1.0, 0.0, 0.0)),
          'tower': ('pelvis', (0.0, 0.0, 1.0)), 'fork': ('chest', (0.0, -1.0, 0.0))}


def seat():
    """How far over the build's torso's anchor the spine's tube's low end stands on the fork's seat, m."""
    return tl.RAISED_M + BRIDGE[1] + nl.SEAT_M


def _pair():
    """({'left', 'right': {part: (from, to, radius)}} along x, the pair's axis's (y, z)): the calf's two drives as
    the build lays them (`knee_clamped.laid`), their axis the pair's hubs'."""
    lay = kc.laid()
    return {'left': lay['pitch'][0], 'right': lay['roll'][0]}, tl.HUB[1:]


def _moved(tris, at):
    """`tris` drawn about the x axis, moved to the axis through `at` (y, z)."""
    return [(tri + (0.0, at[0], at[1]), colour) for tri, colour in tris]


def _mirrored(tris):
    """`tris` on the robot's other side, their faces kept outward."""
    return [((tri * (-1.0, 1.0, 1.0))[::-1], colour) for tri, colour in tris]


def _stood():
    """{part: tris} of what stands with the pelvis: the web between the yaw drives' bands, the column and its
    cupped head, the pair, the clamp, the collar and the tower."""
    (j, hip), yaw = tl.joint(), hl.stacks()['yaw']['motor']
    stacks, at = _pair()
    c0, c1, bore, outside = kc.laid()['cage']
    low, high = (hip[1] + yaw[k] + d - j[1] for k, d in ((0, 0.002), (1, -0.002)))        # the yaw bands' y here
    half = hip[0] - yaw[2] - WEB_IN
    still, bands = [], {}
    for side, stack in stacks.items():
        (b0, b1, r_b), (m0, m1, can), (g0, g1, ring) = stack['board'], stack['motor'], stack['gear']
        gable, inner = (g1, g0) if side == 'left' else (g0, g1)
        split = gable - math.copysign(em.OUTPUT_SHARE * (g1 - g0), gable)
        still += (em._drum(r_b, b0, b1, em.BOARD, at) + em._drum(can, m0, m1, em.SERVO, at)
                  + em._drum(ring, min(inner, split), max(inner, split), em.GEAR, at))
        over = can + 1e-3 * ks.band(ks.torque('left_ankle'), 1e3 * can, 1e3 * (m1 - m0 - 0.004))
        rim, end = (c1, m1 - 0.002) if side == 'left' else (c0, m0 + 0.002)
        cone = rim + math.copysign(outside - over, end)
        bands[side] = _moved(cl.turned(sorted([(rim, can, outside), (cone, can, over), (end, can, over)]), math.pi), at)
    zs = [TOWER_Z[0] + k * (TOWER_Z[1] + 0.002 - TOWER_Z[0]) / 48.0 for k in range(49)]
    under = [max(at[0] + math.sqrt(max((outside + 0.001) ** 2 - (z - at[1]) ** 2, 0.0)), SADDLE_Y - (z - zs[0]))
             for z in zs[1:]]                              # on the cage, its back sloped 45 degrees off the bed
    saddle = hf._slab([(SADDLE_Y, zs[0])] + list(zip(under, zs[1:])) + [(SADDLE_Y, zs[-1])], -FOOT_X, FOOT_X,
                      em.FRAME, rounded=False)
    tower = (hf._put(m._cylinder_x(TOWER_R, TOWER_Z[0], TOWER_Z[1], em.FRAME), hf._along('roll'))
             + hf._prism(hf._box((-FOOT_X, FOOT_X), (SADDLE_Y, 0.0)), TOWER_Z[0], TOWER_Z[1], em.FRAME, 2))
    return {'web': hf._prism(hf._box((-half, half), (low, high)), WEB_Z[0], WEB_Z[1], em.FRAME, 2),
            'column': hf._slab(cl.cupped(at, outside + 0.001, high - 0.002, COLUMN_Z), -COLUMN_X, COLUMN_X, em.FRAME,
                               rounded=False),
            'pair still': still, 'clamp': _moved(cl.honeycomb(bore, outside, c0, c1), at) + bands['left'],
            'collar': bands['right'], 'tower': saddle + tower}


def _chest():
    """{part: tris} of what the chest carries, upright: the fork - the seat, an arm either side to its eye on the
    pitch pin, behind each arm a wall back and out to the boss round the chest's pin, a post under that boss to
    the bed -, those two pins out through the rods' eyes, the tube's foot on its seat and a stub of the tube."""
    import numpy as np
    _seat, spine_z, tube = nl.laid()['seat']
    arm = leaves.hull(list(m._circle(EYE_R, 24)) + [(BRIDGE[0], -ARM_Z), (BRIDGE[1], -ARM_Z), (BRIDGE[1], ARM_Z),
                                                    (BRIDGE[0], ARM_Z)])
    ball = np.array(tl.BALL)
    boss = (ball[0] - LEAN_M - WALL_M, ball[0] - LEAN_M)
    a, b = np.array((ARM_X + 0.5 * ARM_M, 0.0, -ARM_Z)), np.array((0.5 * (boss[0] + boss[1]), 0.0, ball[2]))
    reach = float(np.linalg.norm(b - a))
    axes = np.array((np.cross((0.0, 1.0, 0.0), (b - a) / reach), (0.0, 1.0, 0.0), (b - a) / reach))
    side = np.array(((WALL_UP, 0.0), (BRIDGE[1], 0.0), (BRIDGE[1], reach), (ball[1] - PIN_EYE, reach),
                     (WALL_UP, WALL_LEVEL * reach)))
    wall = [(tri @ axes + a, c) for tri, c in foot._slab(side, -0.5 * WALL_M, 0.5 * WALL_M, em.FRAME)]
    half = (hf._prism(arm, ARM_X, ARM_X + ARM_M, em.FRAME, 0) + wall
            + m._cylinder_x(PIN_EYE, boss[0], boss[1], em.FRAME, at=(ball[1], ball[2]))
            + m._prism(hf._box(boss, (ball[2] - PIN_EYE, ball[2] + PIN_EYE)), ball[1], BRIDGE[1], em.FRAME, axis=1))
    pin = m._cylinder_x(ROD_PIN, boss[0], ball[0] + ROD_EYE + 0.003, hf.STEEL, at=(ball[1], ball[2]))
    oval = em._oval(tube) + (0.0, spine_z)
    top = BRIDGE[1] + nl.SEAT_M
    return {'fork': half + _mirrored(half) + m._prism(oval, BRIDGE[0], BRIDGE[1], em.FRAME, axis=1),
            'cross pins': pin + _mirrored(pin),
            'tube foot': (m._prism(oval, BRIDGE[1], top, em.PLATE, axis=1)
                          + m._prism(oval, top, top + em.FLANGE_M, em.FRAME, axis=1)),
            'tube': m._prism(m._circle(tube, 40, (0.0, spine_z)), top, top + 0.120, em.CARBON, axis=1)}


def _crank():
    """{part: tris} of the left drive's crank at rest: its output ring, the plate on it outboard of the gable -
    a boss round the hub, an eye round the pin -, the pin out through the rod's eye."""
    import numpy as np
    stack, at = _pair()
    g0, g1, ring = stack['left']['gear']
    pin = np.array(at) + tl.CRANK[0] * tl._spoke(math.radians(tl.CRANK[1]))[1:]
    face = g1 + GABLE_AIR
    plate = leaves.hull(list(m._circle(BOSS_R, 24, at)) + list(m._circle(PIN_EYE, 24, tuple(pin))))
    return {'ring': em._drum(ring, g1 - em.OUTPUT_SHARE * (g1 - g0), g1, em.OUTPUT, at),
            'plate': hf._prism(plate, face, face + PLATE_M, hf.STEEL, 0),
            'pin': m._cylinder_x(ROD_PIN, face, tl.HUB[0] + ROD_EYE + 0.003, hf.STEEL, at=tuple(pin))}


@functools.lru_cache(maxsize=None)
def links():
    """{'pelvis', 'hips', 'cross', 'chest', 'left crank', 'right crank': {part: [(tri, colour)]}} in the joint's
    frame, the trunk upright: 'hips' the two hips' yaw drives as the hip's sketch lays them (`hip_clamped`)."""
    import numpy as np
    j, hip = tl.joint()
    off = np.asarray(hip, float) - np.asarray(j, float)
    hips = {}
    for name, tris in hc.links()['pelvis'].items():
        hips['left ' + name] = [(tri + off, c) for tri, c in tris]
        hips['right ' + name] = [((tri + off) * (-1.0, 1.0, 1.0), c) for tri, c in tris]
    crank = _crank()
    return {'pelvis': _stood(), 'hips': hips, 'chest': _chest(), 'left crank': crank,
            'right crank': {name: _mirrored(tris) for name, tris in crank.items()},
            'cross': {'block': hf._prism(hf._box((-BLOCK_M, BLOCK_M), (-BLOCK_M, BLOCK_M)), -BLOCK_M, BLOCK_M, hf.STEEL, 2),
                      'roll journal': hf._put(m._cylinder_x(JOURNAL_R, TOWER_Z[0] - 0.003, -BLOCK_M, hf.STEEL),
                                              hf._along('roll')),
                      'pitch pin': m._cylinder_x(PIN_R, -PIN_X, PIN_X, hf.STEEL)}}


def _leafed(pin, ball, ways):
    """A rod as the calf's from its crank's pin to the chest's: an eye round each pin (`ways`: the pins' own),
    a block by each and between them the calf's leaves, side by side fore and aft, bowed either way about the
    rod's line."""
    import numpy as np
    s, long = leaves.sizes(), float(np.linalg.norm(ball - pin))
    along = (ball - pin) / long
    out = ways[0] - (ways[0] @ along) * along
    out /= np.linalg.norm(out)
    axes = np.array((-out, along, np.cross(along, out)))
    low, top, pitch = leaves.EYE_M + leaves.BLOCK_M, long - leaves.EYE_M - leaves.BLOCK_M, s['t'] + leaves.AIR_M
    half = 0.5 * s['n'] * pitch + 0.003

    def slab(outline):
        return [(tri @ axes + pin, c) for tri, c in foot._slab(np.array(outline), -0.5 * s['w'], 0.5 * s['w'], em.FRAME)]
    tris = [t for a, b in ((0.6 * leaves.EYE_M, low), (top, long - 0.6 * leaves.EYE_M))
            for t in slab([(a, -half), (a, half), (b, half), (b, -half)])]
    us = np.linspace(0.0, 1.0, 17)
    for k in range(s['n']):
        mid = (k - 0.5 * (s['n'] - 1)) * pitch
        edge = [(low + u * (top - low), mid + math.copysign(BOW * (top - low), mid) * leaves._bump(u)) for u in us]
        tris += slab([(h, o + 0.5 * s['t']) for h, o in edge] + [(h, o - 0.5 * s['t']) for h, o in edge][::-1])
    for end, way in zip((pin, ball), ways):
        tris += km._rod(end - ROD_EYE * way, end + ROD_EYE * way, leaves.EYE_M, em.FRAME)
    return tris


def posed(roll=0.0, pitch=0.0):
    """{(link, part): [(tri, colour)]} of the trunk at `roll` and `pitch`, deg: the cross rolled, the chest rolled
    and bowed, each crank turned to where its rod holds it (`trunk_laid.cranked`), the two rods between their
    pins - 'left rod', 'right rod'."""
    import numpy as np
    cross, chest = tl.turn(roll, pitch)
    got = tl.cranked(roll, pitch)
    if got is None:
        raise ValueError('the trunk cannot stand at roll %.1f, pitch %.1f deg: a rod does not reach' % (roll, pitch))
    turns = {'cross': cross, 'chest': chest}
    out = {(link, name): hf._put(tris, turns.get(link, np.eye(3))) for link, parts in links().items()
           for name, tris in parts.items() if 'crank' not in link}
    x, rest = np.array((1.0, 0.0, 0.0)), math.radians(tl.CRANK[1])
    for side, (bearing, pin, ball) in zip(('left', 'right'), got):
        for name, tris in links()[side + ' crank'].items():
            out[(side + ' crank', name)] = cl.tilted(tris, bearing - rest, tl.HUB[1:])
        out[(side + ' rod', 'rod')] = _leafed(pin, ball, (x, cross @ x))
    return out


def _bay():
    """{part: tris} of the neck's and the head's boards' bay on the chest's tube, the joint's frame, upright."""
    from tools.render import neck_mount as nm
    return {name: [(tri - (0.0, tl.RAISED_M, 0.0), c) for tri, c in tris] for name, tris in nm.still().items()
            if name.startswith('bay')}


def aired(poses=((-35.0, -30.0), (-35.0, 27.5), (-35.0, 85.0), (0.0, -30.0), (0.0, 27.5), (0.0, 85.0))):
    """{(roll, pitch): ((m, a part, the part it passes), the bay's (m, its part, the part it passes))}: the least
    air at `poses`, deg, between what moves apart - the cross, the chest, the cranks and the rods off what stands
    (the pelvis's, the hips' yaw drives), the cranks and the rods off the chest's - and, apart, the bay's on the
    chest's tube off what stands; the left side's poses the right's mirrored."""
    import numpy as np
    from scipy.spatial import KDTree

    def planted(parts):                                  # named parts as one tree: (names, each point's, the tree)
        names = [name for name, tris in parts if tris]
        cloud = [km._points(tris, 0.003) for _n, tris in parts if tris]
        return names, np.concatenate([np.full(len(p), k) for k, p in enumerate(cloud)]), KDTree(np.concatenate(cloud))

    def least(mine, others):                             # the nearest two of named parts and a tree's
        names, owner, tree = others
        best = (1.0, '', '')
        for name, tris in mine:
            d, i = tree.query(km._points(tris, 0.003))
            k = int(np.argmin(d))
            best = min(best, (float(d[k]), name, names[int(owner[i[k]])]))
        return best
    stood = planted([(link + ' ' + name, tris) for link in ('pelvis', 'hips') for name, tris in links()[link].items()])
    bay, out = _bay(), {}
    for roll, pitch in poses:
        got, chest = posed(roll, pitch), tl.turn(roll, pitch)[1]
        got.update({('chest', name): hf._put(tris, chest) for name, tris in bay.items()})
        moving = [(link + ' ' + name, tris) for (link, name), tris in got.items()
                  if link not in ('pelvis', 'hips') and name not in MATED and name not in bay]
        driven = [(name, tris) for name, tris in moving if 'crank' in name or 'rod' in name]
        carried = planted([(link + ' ' + name, tris) for (link, name), tris in got.items()
                           if link == 'chest' and name not in MATED])
        out[(roll, pitch)] = (min(least(moving, stood), least(driven, carried)),
                              least([(name, got[('chest', name)]) for name in bay], stood))
    return out


def main(argv=None):
    from PIL import Image, ImageDraw
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    args = parser.parse_args(argv)
    looks = [((0.0, 0.0), 200.0, 10.0), ((0.0, 0.0), 90.0, 4.0), ((0.0, 0.0), 20.0, 8.0), ((0.0, 0.0), 215.0, 40.0),
             ((35.0, 0.0), 180.0, 6.0), ((0.0, 85.0), 90.0, 4.0), ((0.0, -30.0), 90.0, 4.0), ((35.0, 60.0), 215.0, 12.0)]
    px = 640
    sheet = Image.new('RGB', (px * 4, px * 2 + 30), (18, 18, 18))
    for i, (pose, yaw, pitch) in enumerate(looks):
        tris = [t for part in posed(*pose).values() for t in part]
        sheet.paste(m.draw(tris, yaw=yaw, pitch=pitch, px=px, half=0.25, centre=(0.0, -0.05, -0.01)),
                    (px * (i % 4), 30 + px * (i // 4)))
    ImageDraw.Draw(sheet).text((12, 8), 'trunk, a first draft: behind | from its side | ahead | behind, above | leant 35 |'
                               ' bowed 85 | bowed back 30 | leant 35 and bowed 60', fill=(235, 235, 235))
    sheet.save(args.out)
    print(args.out)
    for (roll, pitch), ((gap, mine, other), (far, bay, met)) in aired().items():
        print('roll %4.0f pitch %5.1f: %5.1f mm, %s off %s; the bay %5.1f mm, %s off %s'
              % (roll, pitch, 1e3 * gap, mine, other, 1e3 * far, bay, met))
    return 0


if __name__ == '__main__':
    sys.exit(main())
