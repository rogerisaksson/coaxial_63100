"""The trunk, a first draft, sketched: the pelvis's yoke, the cardan joint, the chest's drive pair.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/trunk_clamped.py OUT.png

`trunk_laid` lays it and tells what the pair turns; here its parts are drawn, the joint's frame, a printed part a
print (PRINTS: the way each grows off its bed). What stands with the pelvis: a web between the two hips' yaw drives'
bands in the column their yokes leave free; on it the pelvis's yoke - a base, a plate either side up to its eye on
the cross's pitch pin and back to a bar across the back, the bar out at each end to the boss round its rod's pin:
the pelvis's cross, which carries a rod's pull over to the other plate, not out of a plate's plane -, those two
pins. What pitches: the cross - a block, its pitch pin across, its roll journals ahead and behind. What pitches and
rolls with the chest: a yoke ahead of the cross and one behind it, each a stack of carbon sheets round its journal
up to a foot under the pair's cage (`trunk_yokes`); the calf's two drives; the clamp - the cage round their inverters and, off a 45 degree
cone, the band round the left drive's motor -; the collar - the right drive's band, coned out to the cage's rim -;
the seat on the cage the spine's tube's foot is screwed on, that foot, a stub of the tube. What each drive turns:
its output ring and its lever (`trunk_rods.lever`) - a slit band round the ring, an arm out to an eye, a carbon pin
bonded through it; from that pin down to the pelvis's a rod (`trunk_rods`) - a cardan cross either end, a swivel
over the lower one, the calf's leaves between. The pins are carbon, the cross's block steel. `posed` turns it all
to a pose, `aired` tells the least air between what moves apart
there - and, apart, the neck's and the head's boards' bay's on the spine's tube (`neck_mount`).
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
from tools.render import hip_clamped as hc  # noqa: E402
from tools.render import hip_frame as hf  # noqa: E402
from tools.render import hip_laid as hl  # noqa: E402
from tools.render import knee_clamped as kc  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import mounting as m  # noqa: E402
from tools.render import neck_laid as nl  # noqa: E402
from tools.render import sized as ks  # noqa: E402
from tools.render import trunk_laid as tl  # noqa: E402
from tools.render import trunk_rods as tr  # noqa: E402
from tools.render import trunk_yokes as ty  # noqa: E402

#: The cross: its block's half side, its pitch pin's radius and reach either side - through the pelvis's plates -,
#: m; its roll journals through the chest's yokes' stacks, theirs (`trunk_yokes`).
BLOCK_M, PIN_R, PIN_X = 0.017, 0.008, 0.033
#: The pelvis's yoke: its plates' inner face off the middle - outside where the chest's rings turn - and their
#: thickness, an eye's radius round the pitch pin - the plate's front its round where steeper than 45 deg, over it
#: cut 45 deg on its tangent, as printed down: the pair's cage leant 35 deg and bowed passes -, the bed's height -
#: the plates' and the beams' top, the print grown down from it -, a plate's low edge behind - over the hips' blocks'
#: sweep -, the
#: 45 deg chamfer along its top's outer edge - the cage's cone to a drive's band dips to it leant 34 deg and bowed
#: back 30 - and the depth of the 45 deg notch in its top between the eye and the bar, where the chest's rear
#: yoke's waist and the cage come leant and bowed back; the base's
#: half width - inside the yaw drives -, its z (from, to) - behind the web as far as the hips' yokes leave room
#: over their sweep, under the plates' roots - and its top at the plates, falling 45 deg from there to
#: the middle - in print each layer reaches in from the plates - under what the chest's rings sweep bowed 85 deg
#: (43 mm off the pitch's axis); the bar's under face, over what the hips' risers sweep behind; a plate's skirt
#: outside it, as wide as the yaw drives leave, its top - a 45 deg chamfer out from the plate - under where the
#: cage comes leant: the plate's depth where the notch takes it, and down the notch's sides where they cut the
#: plate lower, so that in print no edge of it hangs off the plate; m.
ARM_X, ARM_M, EYE_R, TOP_Y, LOW_Y, CHAMFER_M, NOTCH_M = 0.021, 0.0145, 0.013, 0.013, -0.091, 0.0080, 0.030
BASE_X, BASE_Z, BASE_Y, BAR_Y, SKIRT_Y = 0.037, (-0.040, 0.012), -0.044, -0.048, 0.0045
#: The seat's thickness over the cage's top under the spine's tube's foot; the cage's floor - flat under the
#: inverters, its wall and its air over them: the round cage dips onto the pelvis's eyes leant 35 deg and bowed
#: back, and the hubs 6 mm higher would cost a tenth of the pair's least share; m.
SADDLE_M, FLOOR_M = 0.008, (0.002, 0.002)
#: The bar's end round each rod's pin, its half depth and how far it stands over the pin, m; the pin's seat in it
#: and its face are the rods' (`trunk_rods`).
PIN_EYE = 0.014
#: The web's z between the yaw drives' bands and its reach into a band's wall, m.
WEB_Z, WEB_IN = (0.0, 0.018), 0.0025
#: What touches by design, left out of `aired`: the pitch pin in the pelvis's eyes, the journals in the chest's
#: rings, a drive's ring in its housing, the pelvis's pins in the rods' eyes.
MATED = ('pitch pin', 'roll journals', 'ring', 'pins')
#: The printed parts, each a print: (its link, the way it grows off its bed in the joint's frame).
PRINTS = {'web': ('pelvis', (0.0, 0.0, 1.0)), 'yoke': ('pelvis', (0.0, -1.0, 0.0)),
          'clamp': ('chest', (-1.0, 0.0, 0.0)), 'collar': ('chest', (1.0, 0.0, 0.0)), 'seat': ('chest', (0.0, -1.0, 0.0)),
          'lever': ('left crank', (1.0, 0.0, 0.0))}


def _seat_y():
    """The seat's top over the joint's middle, the chest's frame, m: SADDLE_M over the pair's cage."""
    return tl.HUB[1] + kc.laid()['cage'][3] + SADDLE_M


def seat():
    """How far over the build's torso's anchor the spine's tube's low end stands on the seat, m."""
    return tl.RAISED_M + _seat_y() + nl.SEAT_M


def _pair():
    """({'left', 'right': {part: (from, to, radius)}} along x, the pair's axis's (y, z)): the calf's two drives as
    the build lays them (`knee_clamped.laid`), their axis the hubs'."""
    lay = kc.laid()
    return {'left': lay['pitch'][0], 'right': lay['roll'][0]}, tl.HUB[1:]


def _moved(tris, at):
    """`tris` drawn about the x axis, moved to the axis through `at` (y, z)."""
    return [(tri + (0.0, at[0], at[1]), colour) for tri, colour in tris]


def _mirrored(tris):
    """`tris` on the robot's other side, their faces kept outward."""
    return [((tri * (-1.0, 1.0, 1.0))[::-1], colour) for tri, colour in tris]


def _hulled(points, colour):
    """The convex solid round `points` (k, 3), its faces wound outward."""
    import numpy as np
    from scipy.spatial import ConvexHull
    pts = np.asarray(points, float)
    middle, out = pts.mean(axis=0), []
    for face in ConvexHull(pts).simplices:
        tri = pts[face]
        out.append((tri if np.cross(tri[1] - tri[0], tri[2] - tri[0]) @ (tri[0] - middle) > 0.0 else tri[::-1], colour))
    return out


def _stood():
    """{part: tris} of what stands with the pelvis: the web between the yaw drives' bands, the yoke - its base, its
    plates with their eyes, its bar across the back with the bosses at its ends - and the rods' two pins."""
    import numpy as np
    (j, hip), yaw = tl.joint(), hl.stacks()['yaw']['motor']
    low, high = (hip[1] + yaw[k] + d - j[1] for k, d in ((0, 0.002), (1, -0.002)))        # the yaw bands' y here
    half, ball = hip[0] - yaw[2] - WEB_IN, np.array(tl.BALL)
    back, boss = ball[2] - PIN_EYE, (ball[0] - tr.LEAN_M - tr.SEAT_M, ball[0] - tr.LEAN_M)
    front, rear = -EYE_R, ball[2] + PIN_EYE                          # the notch's ends: by the eye, by the beam
    dip, mid, tip = TOP_Y - NOTCH_M, 0.5 * (front + rear), EYE_R * math.sqrt(2.0)   # tip: y + z on the eye's tangent

    def outline(top):                                    # a plate's (y, z) round, its top at `top`, the notch in it
        start = min(math.asin(min(top / EYE_R, 1.0)), 0.25 * math.pi)  # the eye's round where it falls past 45 deg
        arc = [(EYE_R * math.sin(a), EYE_R * math.cos(a)) for a in np.linspace(start, -0.5 * math.pi, 13)]
        return np.array(([(top, tip - top)] if top > arc[0][0] + 1e-9 else []) + arc + [
            (BASE_Y, BASE_Z[1]), (LOW_Y, back), (top, back), (top, mid - (top - dip)), (dip, mid),
            (top, mid + (top - dip))])

    def skirt(top):                                      # a skirt's (x, y) corners, its top at the plate `top`
        return [(ARM_X + ARM_M, LOW_Y), (BASE_X, LOW_Y), (BASE_X, top - (BASE_X - ARM_X - ARM_M)), (ARM_X + ARM_M, top)]
    cut, edge = TOP_Y - CHAMFER_M, ARM_X + ARM_M - CHAMFER_M
    tri = [(edge, cut), (ARM_X + ARM_M, cut), (edge, TOP_Y)]          # the chamfer's section, ended on the notch's flanks
    ahead = [p for z in np.linspace(0.0, math.sqrt(EYE_R ** 2 - cut ** 2), 9)   # and ahead as the plate's front falls
             for top in (min(TOP_Y, tip - z, math.sqrt(EYE_R ** 2 - z * z)),)
             for p in ((edge, cut, z), (ARM_X + ARM_M, cut, z), (edge + TOP_Y - top, top, z), (edge, top, z))]
    knee = boss[1] - (TOP_Y - ball[1] - PIN_EYE)                     # the bar's top falls 45 deg to each boss
    bar = [(-boss[1], BAR_Y), (boss[1], BAR_Y), (boss[1], ball[1] + PIN_EYE), (knee, TOP_Y), (-knee, TOP_Y),
           (-boss[1], ball[1] + PIN_EYE)]
    za, zb = mid - (SKIRT_Y - dip), mid + (SKIRT_Y - dip)
    side = (hf._slab(outline(TOP_Y), ARM_X, edge, em.FRAME, rounded=False)
            + hf._slab(outline(cut), edge, ARM_X + ARM_M, em.FRAME, rounded=False)
            + _hulled([(x, y, back) for x, y in tri] + [(x, y, mid - (y - dip)) for x, y in tri], em.FRAME)
            + _hulled([(x, y, mid + (y - dip)) for x, y in tri] + ahead, em.FRAME)
            + [t for z0, z1 in ((back, za), (zb, BASE_Z[1]))
               for t in hf._slab(np.array(skirt(SKIRT_Y)), z0, z1, em.FRAME, 2, False)]
            + [t for z0, z1, t0, t1 in ((za, mid, SKIRT_Y, dip), (mid, zb, dip, SKIRT_Y))
               for t in _hulled([(x, y, z0) for x, y in skirt(t0)] + [(x, y, z1) for x, y in skirt(t1)], em.FRAME)])
    roof_y = min(high, -0.0455)
    roof = [(-BASE_X, roof_y), (BASE_X, roof_y), (BASE_X, BASE_Y - (BASE_X - ARM_X - ARM_M)), (ARM_X + ARM_M, BASE_Y),
            (ARM_X, BASE_Y), (0.0, BASE_Y - ARM_X), (-ARM_X, BASE_Y), (-ARM_X - ARM_M, BASE_Y),
            (-BASE_X, BASE_Y - (BASE_X - ARM_X - ARM_M))]
    pin = m._cylinder_x(tr.PIN_R[0], boss[0], ball[0] + tr.COLLAR_X + 0.003, em.CARBON, at=(ball[1], ball[2]))
    from tools.render import csg
    return {'web': hf._prism(hf._box((-half, half), (low, high)), WEB_Z[0], WEB_Z[1], em.FRAME, 2),
            'yoke': (side + _mirrored(side) + hf._slab(np.array(roof), BASE_Z[0], BASE_Z[1], em.FRAME, 2, False)
                     + csg.extrude(csg.filleted2d(bar, r=0.002), (), a=back, b=rear, axis=2,
                                   colour=em.FRAME, chamfer=0.0010, slope=0.85).tris()),
            'pins': pin + _mirrored(pin)}


def _chest():
    """{part: tris} of what the chest carries, upright: the pair, the clamp and the collar, the two yokes, the seat,
    the tube's foot and a stub of the tube."""
    import numpy as np
    stacks, at = _pair()
    c0, c1, bore, outside = kc.laid()['cage']
    still, bands = [], {}
    for side, stack in stacks.items():
        (b0, b1, r_b), (m0, m1, can), (g0, g1, ring) = stack['board'], stack['motor'], stack['gear']
        gable, inner = (g1, g0) if side == 'left' else (g0, g1)
        split = inner + math.copysign(em.OUTPUT_SHARE * (g1 - g0), gable)     # the still housing, then the ring

        still += (em._drum(r_b, b0, b1, em.BOARD, at) + em._drum(can, m0, m1, em.SERVO, at)
                  + em._drum(ring, min(inner, split), max(inner, split), em.GEAR, at))
        over = can + 1e-3 * ks.band(ks.torque('left_ankle'), 1e3 * can, 1e3 * (m1 - m0 - 0.004))
        rim, end = (c1, m1 - 0.002) if side == 'left' else (c0, m0 + 0.002)
        cone = rim + math.copysign(outside - over, end)
        bands[side] = _moved(cl.turned(sorted([(rim, can, outside), (cone, can, over), (end, can, over)]), em.SLIT), at)
    floor = tl.HUB[1] - stacks['left']['board'][2] - sum(FLOOR_M)            # the floor's under face, its height
    half = math.acos((tl.HUB[1] - floor) / outside)                          # the bearing its edges stand at
    chord = outside * math.sin(half)
    cage = cl.honeycomb(bore, outside, c0, c1, gap=(math.pi - half, math.pi + half))
    flat = hf._prism(hf._box((c0, c1), (at[1] - chord, at[1] + chord)), floor, floor + FLOOR_M[0], em.FRAME, 1)
    out = {'pair still': still, 'clamp': _moved(cage, at) + flat + bands['left'], 'collar': bands['right']}
    out.update(ty.drawn())
    _seat, spine_z, tube = nl.laid()['seat']
    oval, top = em._oval(tube) + (0.0, spine_z), _seat_y()
    half_x, half_z = tube + em.OVAL_PAST[0], tube + em.OVAL_PAST[1]
    out['seat'] = hf._slab(cl.cupped(at, outside + 0.001, top, (spine_z - half_z, spine_z + half_z), below=False),
                           -half_x, half_x, em.FRAME, rounded=False)
    out['tube foot'] = (m._prism(oval, top, top + nl.SEAT_M, em.PLATE, axis=1)
                        + m._prism(oval, top + nl.SEAT_M, top + nl.SEAT_M + em.FLANGE_M, em.FRAME, axis=1))
    out['tube'] = m._prism(m._circle(tube, 40, (0.0, spine_z)), top + nl.SEAT_M, top + 0.120, em.CARBON, axis=1)
    return out


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
    crank, reach = tr.lever(), [max(abs(z) for z in ty._sheets(name)[-1][:2]) for name in ty.STACKS]
    return {'pelvis': _stood(), 'hips': hips, 'chest': _chest(), 'left crank': crank,
            'right crank': {name: _mirrored(tris) for name, tris in crank.items()},
            'cross': {'block': hf._prism(hf._box((-BLOCK_M, BLOCK_M), (-BLOCK_M, BLOCK_M)), -BLOCK_M, BLOCK_M, hf.STEEL, 2),
                      'pitch pin': m._cylinder_x(PIN_R, -PIN_X, PIN_X, em.CARBON),
                      'roll journals': hf._put(m._cylinder_x(ty.JOURNAL_R, -reach[1], reach[0], em.CARBON),
                                               hf._along('roll'))}}


def posed(roll=0.0, pitch=0.0):
    """{(link, part): [(tri, colour)]} of the trunk at `roll` and `pitch`, deg: the cross pitched, the chest pitched
    and rolled, each crank turned on its drive to where its rod holds it (`trunk_laid.cranked`), the two rods
    between their pins - 'left rod', 'right rod', each its parts (`trunk_rods.rods`)."""
    import numpy as np
    cross, chest = tl.turn(roll, pitch)
    got = tl.cranked(roll, pitch)
    if got is None:
        raise ValueError('the trunk cannot stand at roll %.1f, pitch %.1f deg: a rod does not reach' % (roll, pitch))
    turns = {'cross': cross, 'chest': chest}
    out = {(link, name): hf._put(tris, turns.get(link, np.eye(3))) for link, parts in links().items()
           for name, tris in parts.items() if 'crank' not in link}
    rest = math.radians(tl.CRANK[1])
    for side, (bearing, _pin, _ball) in zip(('left', 'right'), got):
        for name, tris in links()[side + ' crank'].items():
            out[(side + ' crank', name)] = hf._put(cl.tilted(tris, bearing - rest, tl.HUB[1:]), chest)
    out.update(tr.rods(roll, pitch))
    return out


def _bay():
    """{part: tris} of the neck's and the head's boards' bay on the chest's tube, the joint's frame, upright."""
    from tools.render import neck_mount as nm
    return {name: [(tri - (0.0, tl.RAISED_M, 0.0), c) for tri, c in tris] for name, tris in nm.still().items()
            if name.startswith('bay')}


def aired(poses=((-35.0, -30.0), (-35.0, 27.5), (-35.0, 85.0), (0.0, -30.0), (0.0, 27.5), (0.0, 85.0))):
    """{(roll, pitch): ((m, a part, the part it passes), the bay's (m, its part, the part it passes))}: the least
    air at `poses`, deg, between what moves apart - the cross, the chest, the cranks and the rods off what stands
    (the pelvis's, the hips' yaw drives), the cross, the cranks and the rods off the chest's, the rods off the
    drives' rings and levers - and, apart, the bay's on the chest's tube off what stands; the left side's poses
    the right's mirrored."""
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
    stood = planted([(link + ' ' + name, tris) for link in ('pelvis', 'hips') for name, tris in links()[link].items()
                     if name not in MATED])
    bay, out = _bay(), {}
    for roll, pitch in poses:
        got, chest = posed(roll, pitch), tl.turn(roll, pitch)[1]
        got.update({('chest', name): hf._put(tris, chest) for name, tris in bay.items()})
        moving = [(link + ' ' + name, tris) for (link, name), tris in got.items()
                  if link not in ('pelvis', 'hips') and name not in MATED and name not in bay]
        turned = [(name, tris) for name, tris in moving if not name.startswith('chest ')]
        carried = planted([(link + ' ' + name, tris) for (link, name), tris in got.items()
                           if link == 'chest' and name not in MATED and name not in bay])
        ends = planted([(link + ' ' + name, tris) for (link, name), tris in got.items()
                        if 'crank' in link and name in ('ring', 'lever')])
        rods = [(name, tris) for name, tris in moving if ' rod ' in name]
        out[(roll, pitch)] = (min(least(moving, stood), least(turned, carried), least(rods, ends)),
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
        sheet.paste(m.draw(tris, yaw=yaw, pitch=pitch, px=px, half=0.25, centre=(0.0, 0.02, -0.01)),
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
