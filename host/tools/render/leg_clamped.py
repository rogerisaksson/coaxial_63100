"""The robot's leg round the knee's clamps: the thigh's bone up to the hip, the foot on its rods.

    python -X utf8 tools/render/leg_clamped.py OUT.png

The knee's clamps as knee_clamped lays them. Up the thigh: the hip as hip_clamped sketches it, at rest - the thigh's
head on the pitch drive's hub, its arm down the hip's outer side, its knuckle in under the cradle and one
straight tube on from it, radial at the hip and `ahead` of the knee's axis at the knee (`knee_clamped.tilt`, the
line the build states); above it the cradle with the pitch drive, the block with the roll drive and the yaw drive,
their boards under their guards and the bus between them (`hip`). With the knee at its travel's end the thigh's
neck stays clear of the shank's side - its arm's back cut away for the ankle's leaf (`folded`); the foot folded up
behind a straight hip meets the hip's back before that (`heel`).
Down the shank: its tube to a printed fork on the ankle's cross, the foot printed in profile, its plate and
soles (`foot`); each ankle drive's tight clamp on its output ring, its lug out to its rod's pin, the rod - a
printed cartridge of leaves between a head and an eye (`leaves`) - down to the heel's shaft on its steel horn, as
`skeleton._rod` lays its ends, a printed cover on each drive's gable (`covers`); all of it turned with the ankle
by `linked` (`foot_swung` holds its air through the ankle's travel). The cables (`knee_hose`): the knee's into its
board's cover, the foot's to the soles' box. Drawn from ahead, from the side, from behind and with the knee bent.
"""
import argparse
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import clamped as cl  # noqa: E402
from tools.render import elbow_mount as em  # noqa: E402
from tools.render import foot  # noqa: E402
from tools.render import hip_clamped as hc  # noqa: E402
from tools.render import knee_clamped as kc  # noqa: E402
from tools.render import knee_hose  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import knee_stop  # noqa: E402
from tools.render import leaves  # noqa: E402
from tools.render import sized as ks  # noqa: E402
from tools.render import mounting as m  # noqa: E402
from tools.render.knee_loads import stated  # noqa: E402

#: The fork on the ankle's cross: its arms' inner face off the leg's middle and their thickness, the bosses round
#: the cross's pins, and the air its yoke keeps over the rods' eyes, m.
ARM_X, ARM_M, BOSS_M, AIR = 0.022, 0.010, 0.012, 0.002
#: The heel's horn, a steel plate between the foot's two halves: its half thickness and its radius round the shaft,
#: m - 8 mm of steel twists 80 MPa under the roll's two rods (23 N m about its length), a printed lug 61 and across
#: its layers.
HORN_X, HORN_M = 0.004, 0.009
#: How far round its band a lever's lug's root spreads from under its pin, deg; what folds up toward the hip; a
#: gable cover's thickness and its air over what turns under it, m.
SPREAD, HEEL, COVER_M = 50.0, ('foot', 'plate', 'soles', 'heel bar', 'leaf ankle', 'leaf ankle_roll'), 0.002
STEEL = leaves.STEEL


def lever(kind):
    """(band, lug, the lug's faces): the tight clamp on an ankle drive's output ring - a slit band, its screws hid
    (`sized.band`) - and its lug out to its rod's pin (`skeleton._rod`'s), the knee's frame: a plate between the
    rod's fork's cheeks (`leaves`), from the band's inner face - the bed the two are printed on, as one - to GAP_X
    past the pin (the faces, x), as wide round the pin as the ball's boss, its root spread SPREAD round the band,
    up the side its rod never hangs."""
    from machine.body import skeleton
    drive = kc.laid()['pitch' if kind == 'ankle' else 'roll'][0]
    _seg, hub, pin, _bend, _ball = skeleton._rod('left_', kind)
    (gx0, gx1, ring), (mx0, _mx1, _mr) = drive['gear'], drive['motor']
    face, way = (gx1, 1.0) if gx0 >= mx0 else (gx0, -1.0)
    x0, x1 = sorted((face, face - way * ((1.0 - em.OUTPUT_SHARE) * (gx1 - gx0) - 0.0025)))
    wall = 1e-3 * ks.band(ks.torque(kind), 1e3 * ring, 1e3 * (x1 - x0))
    band = [(t + (0.0, hub[1], hub[2]), c) for t, c in cl.turned([(x0, ring, ring + wall), (x1, ring, ring + wall)],
                                                              em.SLIT)]
    faces, rim = ((x0, x1)[way < 0.0], pin[0] + way * (leaves.GAP_X - leaves.PLAY_M)), ring + wall - 0.0005
    at = math.atan2(pin[2] - hub[2], pin[1] - hub[1])      # the pin's bearing about the hub, +y toward +z
    root = [(hub[1] + rim * math.cos(at + a), hub[2] + rim * math.sin(at + a))
            for a in [math.radians(SPREAD) * i / 8.0 for i in range(9)]]
    lug = em._rounded(cl.filleted(leaves.hull(root + leaves.ring(pin[1:], leaves.BALL_M + 0.003))),
                      min(faces), max(faces), em.FRAME, axis=0, r=cl.EDGE_M, steps=1)
    return band, lug, faces


def linked(pitch=0.0, roll=0.0, bow=None):
    """{'bands', 'lugs', 'leaf KIND': tris}: each ankle drive's clamp and its lug, turned to where its rod then holds
    it, and the rod - its cartridge of leaves bowed `bow` (else as printed), its stem and eye, its head
    (`leaves.laid`) - the ankle at `pitch` and `roll`, deg."""
    from machine.body import skeleton
    out = {'bands': [], 'lugs': []}
    for kind in leaves.KINDS:
        turn, hub = leaves.ends(kind, pitch, roll)[3], skeleton._rod('left_', kind)[1]
        band, lug, faces = lever(kind)
        out['bands'] += cl.tilted(band, -turn, (hub[1], hub[2]))
        out['lugs'] += cl.tilted(lug, -turn, (hub[1], hub[2]))
        out['leaf ' + kind] = [t for part in leaves.laid(kind, pitch, roll, faces, bow) for t in part]
    return out


def swept(turns=7):
    """[(tri, colour)]: each ankle drive's lug and its whole rod, its leaves bowed as far as their stop lets them
    (`leaves.stops`), at `turns` pitches through the ankle's travel, each at its roll's ends and middle: what a
    guard by them clears."""
    import numpy as np
    from machine.body import drives
    out, (r0, r1), bow = [], drives.TRAVEL['ankle_roll'], leaves.stops()['push'][0]
    for pitch in np.linspace(*drives.TRAVEL['ankle'], turns):
        for roll in (r0, 0.5 * (r0 + r1), r1):
            out += [t for name, part in linked(pitch, roll, bow).items() if name != 'bands' for t in part]
    return out


def covers():
    """[(tri, colour)]: a printed cover on each ankle drive's gable, COVER_M thick and as far off what turns under
    it: round the gearbox and out over all its lever's fork sweeps (`leaves.ends`, the travel's ends, the roll's and
    the leaves' strokes with them), on three arms from the drive's own clamp, under the drive: ahead of its lever
    and its rod, clear of the knee's pipes."""
    import numpy as np
    from machine.body import drives, linkage, skeleton
    out = []
    for kind in leaves.KINDS:
        _seg, hub, pin, _bend, _ball = skeleton._rod('left_', kind)
        band, _lug, faces = lever(kind)
        way, rim = math.copysign(1.0, pin[0]), max(float(np.hypot(*(tri[:, 1:] - hub[1:]).T).max()) for tri, _c in band)
        spring = max(v[1] for v in leaves.stops().values()) / linkage.RODS[kind][0][2]
        turns = [leaves.ends(kind, p, r)[3] + s * spring for p in drives.TRAVEL['ankle']
                 for r in drives.TRAVEL['ankle_roll'] for s in (1.0, -1.0)]
        at, reach = math.atan2(pin[2] - hub[2], pin[1] - hub[1]), math.dist(pin[1:], hub[1:]) + leaves.FORK_R + COVER_M
        sweep = [(hub[1] + reach * math.cos(at - t), hub[2] + reach * math.sin(at - t))
                 for t in np.linspace(min(turns) - 0.2, max(turns) + 0.2, 17)]
        face = abs(faces[1]) + leaves.PLAY_M + leaves.CHEEK_M + 0.0015 + COVER_M      # over the pin's head
        x0, x1 = sorted((way * face, way * (face + COVER_M)))
        out += em._rounded(cl.filleted(leaves.hull(leaves.ring(hub[1:], rim + 2.0 * COVER_M) + sweep)), x0, x1,
                           em.FRAME, axis=0, r=0.001, steps=1)
        for bearing in (135.0, 165.0, 195.0):               # under the drive, ahead of its lever and its rod
            y, z = (hub[1] + (rim + 2.5 * COVER_M) * math.cos(math.radians(bearing)),
                    hub[2] + (rim + 2.5 * COVER_M) * math.sin(math.radians(bearing)))
            out += km._rod((way * (abs(faces[0]) - 0.008), y, z), (x0, y, z), 1.75 * COVER_M, em.FRAME)
    return out


def _lifted(parts, word=''):
    """{word + name: tris}: parts of the hip's sketch (`hip_clamped`, the hip's frame) in the knee's - the hip THIGH
    up the knee's frame, where the thigh's lean lays it."""
    from machine.body.figure import THIGH
    return {word + name: [(tri + (0.0, THIGH, 0.0), c) for tri, c in tris] for name, tris in parts.items() if tris}


def thigh(ahead):
    """{name: tris} of the thigh above the knee's clamp, the knee's frame, as the hip's sketch lays it
    (`hip_clamped`): its head on the hip's pitch drive's hub, its arm down the hip's outer side and its knuckle
    in under the cradle, the tube on from that - radial at the hip, leant to pass `ahead` of the knee's axis
    (`kc.tilt`) - and the hose's riser. The hip is sketched for the line the build states (`stated`): no other."""
    from machine.errors import MachineError
    if abs(ahead - stated()) > 1e-9:
        raise MachineError('the hip is sketched %.4f m ahead of the knee, not %.4f' % (stated(), ahead))
    return _lifted(hc.links()['thigh'])


def hip():
    """{name: tris} of the hip above the thigh, the knee's frame, every joint of it at rest (`hip_clamped`): the
    cradle with the pitch drive ('cradle ...'), the yaw's block with the roll drive ('yaw ...'), the pelvis's yaw
    drive ('pelvis ...') - their boards under their guards, the bus between them - and its two hoses hung ('hip
    hoses')."""
    links, out = hc.links(), {}
    for link, word in (('roll', 'cradle '), ('yaw', 'yaw '), ('pelvis', 'pelvis ')):
        out.update(_lifted(links[link], word))
    return dict(out, **_lifted({'hip hoses': hc.slung()}))


def under():
    """The fork's yoke's underside, m up the knee's frame: AIR over the rods' eyes where the toes are farthest down
    (`leaves.reach`)."""
    from machine.body.figure import SHANK
    return -SHANK + leaves.reach()[0] + AIR


def shank():
    """{name: tris} of the shank's side below the multi-clamp, the knee's frame straight: the tube to the fork on the
    ankle's cross - its yoke over the rods' eyes however far the toes go down (`under`) -, the foot, the heel's horn
    and shaft, each ankle drive's clamp and lug and its rod or leaf (`linked`)."""
    import numpy as np
    from machine.body import skeleton
    from machine.body.figure import SHANK
    (_m0, _m1, _can), _gear, _ankles, _r_t, r_s = kc._stack()
    lay = kc.laid()
    bone = lay['bone']
    out = {}
    reach, deep = r_s + em.OVAL_PAST[0], r_s + em.OVAL_PAST[1]
    ahead = kc.shank_line()                  # the shank's own line, nearly through the drives' axis
    z_s = (ahead - deep, ahead + deep)
    yoke = 1e-3 * ks.yoke(2e3 * ARM_X, 2e3 * deep)     # the least that spans the arms under the tube's axial load
    y_b = under() + yoke
    xs = (min(bone - reach, -(ARM_X + ARM_M)), max(bone + reach, ARM_X + ARM_M))
    fork = em._rounded(cl.filleted(cl.box(xs, z_s), ARM_M), y_b - yoke, y_b, em.FRAME, r=cl.EDGE_M, steps=1)
    fork += em._rounded(cl.oval(reach, deep, (bone, ahead)), y_b - 0.002, y_b + em.SEAT_M, em.PLATE, r=cl.EDGE_M,
                        steps=1)
    tops = (z_s[0] + ARM_M, z_s[1] - ARM_M)                    # inside the yoke's rounded corners
    arm = cl.filleted(leaves.hull(leaves.ring((-SHANK, 0.0), BOSS_M) + [(y_b - yoke + 0.001, z) for z in tops]
                                  + [(y_b, z) for z in tops]))
    for s in (1.0, -1.0):
        x0, x1 = sorted((s * ARM_X, s * (ARM_X + ARM_M)))
        fork += em._rounded(arm, x0, x1, em.FRAME, axis=0, r=cl.EDGE_M, steps=1)
    out['fork'] = fork
    face_f = y_b + em.SEAT_M
    e = r_s - 0.0012 - em.BORE_R - 0.003
    dress, limb = em._limb(face_f, 1.0, r_s, -e, 'torx', False)
    out['fork flange'] = [(tri + (bone, 0.0, ahead), c) for tri, c in dress + limb]
    _top, low = kc.seats(ahead)
    out['shank tube'] = m._prism(m._circle(r_s, 40, (bone, ahead)), face_f + em.FLANGE_M,
                                 low - em.SEAT_M - em.FLANGE_M, em.CARBON)
    _stage, pin_r, half = skeleton.CROSS
    ankle = (0.0, -SHANK, 0.0)
    out['cross'] = (em._drum(pin_r, -half, half, STEEL, at=(-SHANK, 0.0))
                    + km._rod((0.0, -SHANK, -half), (0.0, -SHANK, half), pin_r, STEEL))
    out.update(foot.parts())                             # the printed foot, its plate and soles
    out.update(linked(), **{'crank covers': covers()})
    balls = [skeleton._rod('left_', k)[4] for k in leaves.KINDS]
    (_x, y_h, z_h), far = balls[0], ARM_X - AIR             # the heel's shaft through both balls, between the arms
    horn = leaves.hull(leaves.ring((y_h, z_h), HORN_M) + [(-SHANK + s * BOSS_M, z) for s in (1.0, -1.0)
                                                         for z in (-half, foot.PILLAR_M - half)])
    heel = em._rounded(cl.filleted(horn), -HORN_X, HORN_X, STEEL, axis=0, r=0.001, steps=1)
    heel += km._rod((-far, y_h, z_h), (far, y_h, z_h), leaves.SHAFT_M, STEEL)
    for b in balls:                                         # a cone a side of each eye: the foot's roll its tilt
        inner, outer = sorted((math.copysign(HORN_X, b[0]), b[0] - math.copysign(leaves.EYE_X, b[0])), key=abs)
        heel += km._rod((inner, y_h, z_h), (outer, y_h, z_h), 1.4 * leaves.SHAFT_M, STEEL)
        heel += km._rod((b[0] + math.copysign(leaves.EYE_X, b[0]), y_h, z_h), (math.copysign(far, b[0]), y_h, z_h),
                        1.4 * leaves.SHAFT_M, STEEL)
    out['heel bar'] = heel
    return out


def stop(ahead, step=2.5, coarse=4):
    """(the leg's stop, deg, (still part, turning part, m) that set it or None): the last turn of the knee from
    straight at which nothing of the shank's side - its clamp, drives, rods, leaf, foot - comes within CLEAR_M of the
    thigh's - its clamps, tube, drives and inverters - up to the end of the knee's travel."""
    import numpy as np
    from machine.body import drives
    _p, _c, still, turning = kc.bodies(ahead=ahead)
    still = dict(still, **thigh(ahead))
    moving = dict({k: v for k, v in turning.items() if k != 'output ring'}, **shank())
    h = 0.006
    a = km._points([t for v in still.values() for t in v])
    keys = km._key(np.floor(a / h).astype(np.int64))
    order = np.argsort(keys)
    pts = km._points([t for v in moving.values() for t in v])
    turns = np.arange(0.0, drives.TRAVEL['knee'][1] + 0.1, step)

    def hit(i):
        return km._least(keys[order], a[order], pts @ km._rx(float(turns[i])).T, h) < em.CLEAR_M
    looks = list(range(0, len(turns), coarse)) + ([len(turns) - 1] if (len(turns) - 1) % coarse else [])
    first = next((i for i in looks if hit(i)), None)
    if first is None:
        return float(turns[-1]), None
    for i in range(max(first - coarse + 1, 0), first + 1):
        if i == first or hit(i):
            return (float(turns[i - 1]) if i else 0.0), knee_stop._culprits(still, moving, float(turns[i]), h)
    return float(turns[first - 1]), None


def _air(still, low, deg):
    """(m, name): the least air between `still` {name: tris} and the shank's side's points `low` (k, 3), the knee
    turned `deg`, and the part of `still` nearest them - each part told only where it stands by them."""
    import numpy as np
    from scipy.spatial import KDTree
    at = low @ km._rx(float(deg)).T
    tree, lo, hi, gaps = KDTree(at), at.min(axis=0) - 0.006, at.max(axis=0) + 0.006, []
    for name, part in still.items():
        tris = np.array([tri for tri, _c in part])
        by = tris[np.all((tris.max(axis=1) > lo) & (tris.min(axis=1) < hi), axis=1)]
        if len(by):
            gaps.append((float(tree.query(km._points([(tri, 0) for tri in by], 0.002))[0].min()), name))
    return min(gaps, default=(1.0, None))


def folded():
    """(m, the thigh's part): the least air between the thigh above the knee's clamp (`thigh`) and the shank's side -
    its clamp, drives, rods, leaf, foot - with the knee at the end of its travel: what its last degrees ask of the
    thigh's neck (`hip_laid.ARM_CUT`)."""
    from machine.body import drives
    _p, _c, _still, turning = kc.bodies(ahead=stated())
    moving = dict({k: v for k, v in turning.items() if k != 'output ring'}, **shank())
    return _air(thigh(stated()), km._points([t for v in moving.values() for t in v], 0.002), drives.TRAVEL['knee'][1])


def heel(step=2.5):
    """(deg, (the hip's part, m) or None): the last turn of the knee from straight at which the foot - its keel,
    plate, soles and heel bar - and the rods on its shaft (HEEL) stay CLEAR_M off the hip at rest (`hip`), up to the
    end of the knee's travel, and what of the hip stood nearest one step on: the hip's back, short of the leg's own
    stop (`stop`)."""
    import numpy as np
    from machine.body import drives
    still, met = hip(), None
    low = km._points([t for k, v in shank().items() if k in HEEL for t in v], 0.002)
    for deg in np.arange(drives.TRAVEL['knee'][1], 0.0, -step):     # back from the fold's end: the first that clears
        gap, name = _air(still, low, deg)
        if gap >= em.CLEAR_M:
            return float(deg), met
        met = (name, round(gap, 5))
    return 0.0, met


def bodies(ahead, flex=0.0):
    """The leg's triangles, the knee bent `flex` degrees: the hip at rest above it (`hip`), its cables across its
    joints (`knee_hose`)."""
    parts, _cables, _still, _turning = kc.bodies(flex=flex, ahead=ahead)
    up = [t for v in dict(thigh(ahead), **hip(), **knee_hose.knee(flex)).values() for t in v]
    lower = shank()
    down = [t for v in dict(lower, **knee_hose.ankle(under(), lower['fork'])).values() for t in v]
    return parts + up + km._turn(down, flex)


def main(argv=None):
    from PIL import Image, ImageDraw
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    args = parser.parse_args(argv)
    lead = stated()
    px, views = 900, []
    parts = bodies(lead)
    centre, half = kc._framed(parts)
    for name, yaw, pitch in (('from ahead', 35.0, 15.0), ('from the side', 90.0, 8.0), ('from behind', -145.0, 15.0)):
        views.append((name, m.draw(parts, yaw=yaw, pitch=pitch, px=px, half=half, centre=centre)))
    parts = bodies(lead, flex=90.0)
    centre, half = kc._framed(parts)
    views.append(('the knee bent 90 deg', m.draw(parts, yaw=70.0, pitch=15.0, px=px, half=half, centre=centre)))
    sheet = Image.new('RGB', (2 * px, 2 * (px + 30) + 30), (18, 18, 18))
    pen = ImageDraw.Draw(sheet)
    pen.text((12, 8), "leg: the hip as sketched, its thigh's neck and bone %.0f mm ahead of the knee's axis, the knee's"
             " clamps, the shank's bone to the ankle's fork, the arched foot on its soft soles and two leaf springs"
             % (1e3 * lead),
             fill=(235, 235, 235))
    for i, (name, img) in enumerate(views):
        x, y = px * (i % 2), 30 + (px + 30) * (i // 2)
        sheet.paste(img, (x, y + 30))
        pen.text((x + 12, y + 10), name, fill=(200, 200, 200))
    sheet.save(args.out)
    (limit, why), (reach, what) = stop(lead), heel()
    print(args.out, 'the leg folds to %.1f deg, set by %s; the hip straight, its foot clears the hip to %.1f deg, set '
          'by %s' % (limit, why, reach, what))
    return 0


if __name__ == '__main__':
    sys.exit(main())
