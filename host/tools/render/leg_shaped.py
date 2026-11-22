"""The leg's prints shaped by their loads, the heaviest first: the hip's, and the knee's stator clamp.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/leg_shaped.py [OUT.png]

As `trunk_shaped` does the trunk's: a print of the hip's sketch (`hip_clamped.links`) is the optimiser's room as
drawn - its drawing filled to voxels on its bed, grown the way it is printed (`prints.piece`, `topo.filled`),
nothing added to it -, held where it is screwed on and loaded where the next part bears on it with the hip's loads
at the margin (`hip_laid.loads`), a case a load: each force and each moment alone about the hip's middle, and all
of them at once - the scenarios' worst need not come together, and the hand checks take them so. The thigh's arm
hangs from the pitch drive's hub: held by its head's face on the hub, its head and skirt and the bosses round the
knuckle's four screws kept, the thigh's loads on those bosses. The knuckle is held by the same four screws on the
arm's pad and bears the thigh's tube in its collar and spigot. The yoke is held by its crown under the yaw drive's
hub and bears all the block hands on at the block's four screws. The block is held by those screws through its
ears; the pin turns in its needles' bore - the forces and the moments across the pin there - and the roll's moment
comes through the sector into the pinion: the tooth's push on the bore and back on the roll drive's band. The
knee's stator clamp is held by its saddle's top under the thigh's tube's foot and bears the knee's loads in its band
round the knee's motor (`knee_loads.loads`). The stator clamp, a slab up the way it prints, is
shaped extruded - each column through one density, the pockets through, the outline drawn smooth
(`topo.outlined`) -, the rest grown standing (`topo.printed`). `topo.shaped`
runs a case once (.cache/topo keeps it), `drawn` gives its solid's faces in the hip's frame - None until it carries
its loads with nothing told to stay lost -, `told` what each kept.
"""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import elbow_mount as em  # noqa: E402
from tools.render import hip_laid as hl  # noqa: E402
from tools.render import knee_loads as kl  # noqa: E402
from tools.render import topo  # noqa: E402

#: The prints shaped: {name: (its voxel, mm, the share of what is the optimiser's to shape that it may keep)};
#: how far round a screw's axis its boss is kept and loaded, and past a seat's face its voxels are, mm. Every room
#: as drawn carries all solid (`topo.whole`, 3 mm voxels): the knuckle x1.55, the yoke x1.32, the arm x1.23, the
#: stator clamp x1.15 and the block x1.11.
SHAPED = {'thigh arm': (3.0, 0.7), 'thigh knuckle': (3.0, 0.4), 'yoke': (3.0, 0.5), 'block': (3.0, 0.65),
          'stator clamp': (3.0, 0.6)}
BOSS_MM, SEAT_MM = 9.0, 4.0


@functools.lru_cache(maxsize=None)
def _room(name, h):
    """(room [across, up, deep], turn, corner, (x, y, z) mm in the hip's frame, (y, z) mm upright): a print's
    drawing filled to `h`-mm voxels on its bed, and its voxels' middles - upright as well: a thigh's print with the
    thigh's lean turned back about the hub, the rest as they stand."""
    from tools.render import knee_clamped as kc
    from tools.render import prints
    tris, way = prints.piece(name)
    room, turn, corner, (x, y, z) = topo.filled(tris, way, h)
    lean = kc.tilt(kl.stated()) if name.startswith('thigh ') else 0.0
    c, s = math.cos(lean), math.sin(lean)
    return room, turn, corner, (x, y, z), (c * y - s * z, s * y + c * z)


def _loads(patch, turn, corner):
    """The hip's loads at the margin on `patch`, about the hip's middle, in the bed's frame: each force and each
    moment alone, and all at once."""
    import numpy as np
    ld = hl.loads()
    force, moment, none = np.array(ld['force'], float), np.array(ld['moment'], float), np.zeros(3)
    about = -corner                                       # the hip's middle, the frame's origin, on the bed

    def load(f, m):
        return [(patch, turn @ f, turn @ m, about)]
    single = [load(np.where(np.arange(3) == k, force, 0.0), none) for k in range(3)]
    single += [load(none, np.where(np.arange(3) == k, moment, 0.0)) for k in range(3)]
    return single + [load(force, moment)]


def _screws(x, yu, zu, x0, x1):
    """The voxels within BOSS_MM of the knuckle's four screws' axes (along x, from x0 to x1, mm), upright."""
    import numpy as np
    near = np.zeros(x.shape, bool)
    for r, w in hl.KNUCKLE_AT:
        near |= np.hypot(yu + 1e3 * r, zu - 1e3 * w) <= BOSS_MM
    return near & (x >= x0) & (x <= x1)


def _arm(name, h, share):
    """The thigh's arm's case: held by its head's face on the hub, its head, its skirt and the knuckle's screws'
    bosses kept, the hip's loads on those bosses."""
    import numpy as np
    room, turn, corner, (x, y, z), (yu, zu) = _room(name, h)
    (q0, q1, r_q), r = hl.stacks()['pitch']['gear'], np.hypot(y, z)
    face, split, x0 = 1e3 * (q1 + 0.0005), 1e3 * (q0 + em.OUTPUT_SHARE * (q1 - q0)), 1e3 * (q1 + hl.ARM_X)
    x1, head, pad = x0 + 1e3 * hl.ARM_M, 1e3 * hl.HEAD_R, x0 - 1e3 * hl.PAD_M
    held = (x >= face) & (x <= face + 1.5 * h) & (r <= head)
    kept = (held | ((x >= face) & (x <= x1) & (r <= head + SEAT_MM))
            | ((x >= 1e3 * split) & (x <= face) & (r <= 1e3 * r_q + kl.WALL + SEAT_MM)))
    bosses = _screws(x, yu, zu, pad, x1)
    return topo.case(name, room, kept | bosses, held, _loads(bosses & room, turn, corner), h, share, extruded=False)


def _knuckle(name, h, share):
    """The thigh's knuckle's case: held by the four screws through it into the arm's pad, its collar and spigot
    kept, the hip's loads on them where the thigh's tube bears."""
    import numpy as np
    from tools.render import knee_clamped as kc
    room, turn, corner, (x, y, z), (yu, zu) = _room(name, h)
    x0 = 1e3 * (hl.stacks()['pitch']['gear'][1] + hl.ARM_X)
    bone, tube = 1e3 * kc.laid()['bone'], 1e3 * (kc._stack()[3] + em.COLLAR_M + 0.0002)
    held = _screws(x, yu, zu, -1e9, x0 - 1e3 * hl.PAD_M)
    low = -1e3 * hl.JOG[1]                                # the collar's top, the tube's way down from it
    seat = (np.hypot(x - bone, zu) <= tube + SEAT_MM) & (yu <= low + SEAT_MM)
    return topo.case(name, room, held | seat, held, _loads(seat & room, turn, corner), h, share)


def _block_screws(x, y, z, z0, z1):
    """The voxels within BOSS_MM of the block's four screws' axes (along z, from z0 to z1, mm): two up each ear."""
    import numpy as np
    near = np.zeros(x.shape, bool)
    for way in (1.0, -1.0):
        for bx, by in hl.BLOCK_AT:
            near |= np.hypot(x - 1e3 * way * bx, y - 1e3 * by) <= BOSS_MM
    return near & (z >= z0) & (z <= z1)


def _yoke(name, h, share):
    """The yaw's yoke's case: held by its crown's face under the yaw drive's hub, the crown under the ring and the
    block's screws' bosses kept, the hip's loads - all the block hands on - on those bosses."""
    import numpy as np
    room, turn, corner, (x, y, z), _up = _room(name, h)
    st = hl.stacks()
    low, back = 1e3 * min(a for a, _b, _r in st['yaw'].values()), 1e3 * st['roll']['gear'][1]
    seat = (np.hypot(x, z) <= 1e3 * st['yaw']['gear'][2] + SEAT_MM) & (y <= low + 1.0)
    held = (np.hypot(x, z) <= 1e3 * hl.HUB_R) & (y >= low - 1.5 * h) & (y <= low + 1.0)
    bosses = _block_screws(x, y, z, back, back + 1e3 * hl.EAR_M + 2.0 * BOSS_MM)
    return topo.case(name, room, seat | bosses, held, _loads(bosses & room, turn, corner), h, share)


def _block(name, h, share):
    """The yaw's block's case: held by the four screws through its ears into the yoke, its band round the roll
    drive and its needle bearings' bore kept; the hip's forces and its moments across the pin on the bore, about
    the hip's middle - the pin turns in it about the roll's axis -, and the roll's moment through the sector into
    the pinion: its tooth's push on the bore, and back on the band where the pinion meets the sector."""
    import numpy as np
    room, turn, corner, (x, y, z), _up = _room(name, h)
    st, (small, big, _tip) = hl.stacks(), hl.teeth()
    (m0, m1, can), (_g0, g1, r_g) = st['roll']['motor'], st['roll']['gear']
    (_f0, f1), (b0, _b1) = hl.needles()
    roll = np.hypot(x - 1e3 * hl.ROLL_AT[0], y - 1e3 * hl.ROLL_AT[1])
    room = (room & ~((np.hypot(x, y) < 1e3 * hl.NEEDLE[0]) & (z >= 1e3 * b0 - 5.0) & (z <= 1e3 * f1 + 5.0))
            & ~((roll < 1e3 * can) & (z >= 1e3 * m0 - 2.0) & (z <= 1e3 * m1 + 1.0))
            & ~((roll < 1e3 * (r_g + 0.001)) & (z >= 1e3 * m1) & (z <= 1e3 * g1 + 5.0)))
    held = _block_screws(x, y, z, 1e3 * (g1 - hl.EAR_M), 1e3 * g1)
    ears = np.zeros(x.shape, bool)
    for way in (1.0, -1.0):
        for bx, by in hl.BLOCK_AT:
            ears |= (np.hypot(x - 1e3 * way * bx, y - 1e3 * by) <= BOSS_MM + SEAT_MM) & (z >= 1e3 * (g1 - hl.EAR_M - 0.004))
    band = ((roll <= 1e3 * (can + hl.walls()['roll']) + SEAT_MM) & (roll >= 1e3 * r_g) & (z >= 1e3 * m0)
            & (z <= 1e3 * g1) & room)
    bore = (np.hypot(x, y) <= 1e3 * (hl.NEEDLE[0] + hl.HOUSED_M + 0.006)) & (z >= 1e3 * b0) & (z <= 1e3 * g1) & room
    ld = hl.loads()
    force, moment = np.array(ld['force'], float), np.array(ld['moment'], float)
    push = np.array([moment[2] / (1e3 * big), 0.0, 0.0])                 # the sector's tooth's, N: the roll's moment
    mesh = turn @ np.array([1e3 * hl.ROLL_AT[0], 1e3 * (hl.ROLL_AT[1] - small), 500.0 * sum(hl.plate())]) - corner
    across = np.array([moment[0], moment[1], 0.0])

    def rolled(f, m):                                     # the pin's load and the roll's through the pinion
        return [(bore, turn @ (f + push), turn @ m, -corner), (band, -(turn @ push), np.zeros(3), mesh)]
    loads = _loads(bore, turn, corner)[:5] + [rolled(np.zeros(3), np.zeros(3)), rolled(force, across)]
    return topo.case(name, room, ears | band | bore, held, loads, h, share, extruded=False)


def _knee_loads(patch, turn, corner):
    """The knee's loads at the margin (`knee_loads.loads`) on `patch`, about the knee's middle, in the bed's frame,
    the leg straight - its swing about the knee's axis x, its lateral bending about z, its twist about the shank's
    line y, its push along it -: each alone, and all at once."""
    import numpy as np
    ld, none = kl.loads('knee'), np.zeros(3)
    each = [(none, np.array([ld['swing'], 0.0, 0.0])), (none, np.array([0.0, 0.0, ld['lateral']])),
            (none, np.array([0.0, ld['torsion'], 0.0])), (np.array([0.0, ld['axial'], 0.0]), none)]
    each.append((sum(f for f, _m in each), sum(m for _f, m in each)))
    return [[(patch, turn @ f, turn @ m, -corner)] for f, m in each]


def _stator(name, h, share):
    """The knee's stator clamp's case: held by its saddle's top where the thigh's tube's foot is seated, its band
    round the knee's motor and on round the gearbox's housing kept, the knee's loads where it grips the motor - its
    stator's, the drive's torque and what the shank hands up through the drive's bearings."""
    import numpy as np
    from tools.render import knee_clamped as kc, leg_clamped as lc
    room, turn, corner, (x, y, z), _up = _room(name, h)
    lay, top, (m0, m1, _can) = kc.laid(), 1e3 * kc.seats(lc.stated())[0], kc._stack()[0]
    (x0, _r0, _o0), (x1, _r1, out) = lay['stator'][0], lay['stator'][-1]
    held = room & (y >= top - 1.5 * h)
    band = room & (np.hypot(y, z) <= 1e3 * out + SEAT_MM) & (x >= 1e3 * x0) & (x <= 1e3 * x1)
    grip = band & (x >= 1e3 * m0) & (x <= 1e3 * (m1 - 0.002))                        # where it holds the motor
    return topo.case(name, room, held | band, held, _knee_loads(grip, turn, corner), h, share, extruded=True)


@functools.lru_cache(maxsize=None)
def cases():
    """{name: its case for `topo`}: the room, what must stay, what holds it and its loads - in its bed's frame."""
    made = {'thigh arm': _arm, 'thigh knuckle': _knuckle, 'yoke': _yoke, 'block': _block, 'stator clamp': _stator}
    return {name: made[name](name, h, share) for name, (h, share) in SHAPED.items()}


def drawn(name):
    """[(tri, colour)] of a shaped print's voxels' faces, its sketch's frame, m - None where it is not shaped yet, or
    its solid does not carry its loads or lost a voxel told to stay."""
    case = cases()[name]
    if not topo.ready(case):
        return None
    _room_, turn, corner, _at, _up = _room(name, SHAPED[name][0])
    got = topo.shaped(case)
    if got['margin'] < 1.0 or got['lost']:
        return None
    return [(1e-3 * (tri + corner) @ turn, em.FRAME) for tri in topo.faced(got, case)]


def posed(ahead=None, flex=0.0):
    """`leg_clamped.bodies` with every shaped print that carries its loads drawn in place of its sketch."""
    from tools.render import hip_clamped as hc, knee_clamped as kc, knee_hose, knee_mount as km, leg_clamped as lc
    from machine.body.figure import THIGH
    lead = lc.stated() if ahead is None else ahead
    _parts, _cables, still, turning = kc.bodies(flex=flex, ahead=lead)
    stator = drawn('stator clamp')
    if stator is not None:
        still = dict(still, **{'stator band': stator, 'saddle': []})
    links = {k: dict(v) for k, v in hc.links().items()}
    for name, link, key in (('thigh arm', 'thigh', 'neck'), ('thigh knuckle', 'thigh', 'knuckle'),
                            ('yoke', 'yaw', 'yoke'), ('block', 'yaw', 'block')):
        tris = drawn(name)
        if tris is not None:
            links[link][key] = tris
    def lifted(parts, word=''):
        return {word + n: [(t + (0.0, THIGH, 0.0), c) for t, c in ts] for n, ts in parts.items() if ts}
    hip = dict(lifted(links['roll'], 'cradle '), **lifted(links['yaw'], 'yaw '),
               **lifted(links['pelvis'], 'pelvis '), **lifted({'hip hoses': hc.slung()}))
    up = [t for v in dict(lifted(links['thigh']), **hip, **knee_hose.knee(flex)).values() for t in v]
    lower = lc.shank()
    down = [t for v in dict(lower, **knee_hose.ankle(lc.under(), lower['fork'])).values() for t in v]
    all_still = [t for v in still.values() for t in v]
    all_turn = km._turn([t for v in turning.values() for t in v], flex)
    return all_still + all_turn + up + km._turn(down, flex)


def told(log=None):
    """{name: {'room', 'grams': g before and after, 'margin', 'level', 'lost', 'hung'}} of each print shaped."""
    out = {}
    for name in SHAPED:
        got = topo.shaped(cases()[name], log)
        layers = got['solid'].transpose(1, 0, 2)
        out[name] = {'room': round(float(got['room']), 1), 'grams': round(float(got['grams']), 1),
                     'margin': round(float(got['margin']), 2), 'level': float(got['level']), 'lost': int(got['lost']),
                     'hung': int((layers & ~topo.stood(layers)).sum())}
    return out


def main(argv=None):
    import numpy as np
    from PIL import Image, ImageDraw
    from tools.render import mounting as m, prints
    out = (sys.argv[1:] if argv is None else argv)[:1]
    told_ = told(log=print)
    for name, got in told_.items():
        print('%-14s %6.1f g of %6.1f g at %.1f, margin x%.2f, %d hung, %d kept voxels lost' % (
            name, got['grams'], got['room'], got['level'], got['margin'], got['hung'], got['lost']))
    if out:                                               # each print shaped - or as drawn, where it does not carry
        px = 640
        sheet = Image.new('RGB', (px * len(SHAPED), px + 60), (18, 18, 18))
        for i, (name, got) in enumerate(told_.items()):
            tris = drawn(name) or prints.piece(name)[0]
            centre = tuple(float(v) for v in np.concatenate([t for t, _c in tris]).mean(axis=0))
            sheet.paste(m.draw(tris, yaw=35.0, pitch=25.0, px=px, half=0.075, centre=centre), (px * i, 60))
            ImageDraw.Draw(sheet).text((px * i + 10, 10), '%s: %.0f g of %.0f, x%.2f%s' % (
                name, got['grams'], got['room'], got['margin'], '' if drawn(name) else ' (as drawn)'),
                fill=(235, 235, 235))
        sheet.save(out[0])
        print(out[0])
    return 0


if __name__ == '__main__':
    sys.exit(main())
