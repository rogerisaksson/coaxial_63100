"""The neck's layout: the torso's block on the crossbar, the neck's drive in it, the head's on a yoke.

    python -X utf8 tools/render/neck_laid.py        # its sizes and what holds, each by its margin

The torso's block is one printed part on the crossbar: a slit band round the crossbar between the two shoulders'
drums, a saddle under it the spine's tube's foot is screwed to, and over it - the two bores a web apart - the slit
band round the neck's motor. The neck's drive stands as the figure has it, its gearbox's output ring turning a yoke:
a carbon plate screwed on the ring's hub - thin, inside the shoulders' drums' mouths - and on its top a printed crown:
a cheek screwed to the plate, a floor on the head's axis, a slit band round the head's motor standing on that floor
and under the floor a shoe that bears the head's thrust on the neck's band. The head's drive stands upright in that
band, its output ring uppermost, and on the ring's hub a carbon carrier the skull is screwed to: nothing of the drive
turns with the head but its ring. Neither the neck's side moment nor the head's is left to a band's friction alone:
each band's screws pull the hoop that keeps it shut under its joint's tilt (`sized.leant`), and each output carries
its limb on its own hub's screws. The two drives' boards are the build's remote twin (`drives.BOARDS`): a bay on
the spine's tube under the block, no board round the neck or in the skull - a guard there would stand through the
skin, and the skull is the cameras'. So no bus crosses the neck: the neck's motor's leads cross no joint, and the
head's cross the neck's alone, in a hose beside the neck's motor's back (`neck_hosed`). Frame: the torso's (x to the
robot's left, y up, z ahead). m.
"""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import elbow_mount as em  # noqa: E402
from tools.render import knee_loads as kl  # noqa: E402
from tools.render import shoulder_laid as sl  # noqa: E402
from tools.render import sized  # noqa: E402

#: How far over the neck's axis the head's motor's band begins, m (0.490 m up the torso): from its root on the
#: crown's floor, 2 mm under that, the likeness's head's skin, turned through the head's travel, keeps 51 mm off the
#: head's axis all round - the band 46 mm round, its air and the skull's shell inside that; lower, the neck keeps
#: 47 (`neck_mount.checked`: 'skin').
HEAD_M = 0.105
#: The yoke's plate's width across the neck, how far its crown's floor stands round past the head's motor, and how
#: long the crown's shoe lies on the neck's band across the neck, m.
WEB_M, PAST_M, SHOE_M = 0.060, 0.0015, 0.030
#: Over the tee's collar the plate keeps inside the likeness's neck's skin - turned through the head's travel it
#: keeps 42 mm off the head's axis from 0.4275 m up the torso, and the skull's shell is inside that: how far over
#: the neck's axis that begins, and how far behind that axis the plate's edge then stands, 39 mm off the head's, m.
COLLAR_M, NAPE_M = 0.028, 0.0135
#: The crown's cheek on the plate: its screws (the hub's size), one a corner, and how far their centres keep inside
#: its edges - a thread and a half -, m.
CHEEK, EDGE_M = 4, 0.0075
#: How far inside the shoulders' blocks' sides the block's own body ends, the air a turning part keeps off a still
#: one, the saddle under the crossbar's band and the seat on it, m.
INSIDE_M, AIR_M, SADDLE_M, SEAT_M = 0.004, 0.002, 0.010, em.SEAT_M


def placed():
    """{'neck': (y, z) of the neck's axis, 'yaw': the head's axis's z, 'arm': the upper arm's pivot (x, y, z), 'lean':
    its lean, rad, 'spine': the spine's tube's (top's z, radius), 'bay': the twin's boards' (y, the neck's z, the
    head's z)}: the figure's, the skeleton's and the build's remote boards', the torso's frame."""
    from machine.body import drives, figure, skeleton
    seg = {s[0]: s for s in figure.SEGMENTS}
    neck, head, arm = seg['neck'][3], seg['head'][3], seg['left_upper_arm']
    _low, top = skeleton.TRUNK['torso'][0][0]              # the spine's tube, the first of the torso's
    near, far = drives.BOARDS['head'][1], drives.BOARDS['neck'][1]
    return {'neck': (float(neck[1]), float(neck[2])), 'yaw': float(neck[2] + head[2]),
            'arm': tuple(float(v) for v in arm[3]), 'lean': math.radians(float(arm[4])),
            'spine': (float(top[2]), float(skeleton.TRUNK_R['torso'][0])),
            'bay': (float(far[1]), float(far[2]), float(near[2]))}


def stacks():
    """{'neck': {'motor', 'gear': (from, to, radius) along its axis}, 'head': {'motor', 'gear': (radius, length)},
    'board': (radius, length)}: the drives' parts (`skeleton.cans`); the boards the shoulder's, the one inverter."""
    from machine.body import drives, skeleton
    out = {}
    for joint in ('neck', 'head'):
        names = [p for p, *_ in drives.along(joint)]
        cans = [c for c in skeleton.cans() if c[0] == joint]
        out[joint] = {n: (float(c[2][0]) - float(c[5]), float(c[2][0]) + float(c[5]), float(c[4]))
                      if joint == 'neck' else (float(c[4]), 2.0 * float(c[5])) for n, c in zip(names, cans)}
    out['board'] = sl.stack()['board']
    return out


def _band(joint, can, grip):
    """A slit band's wall round a drum `can` round over `grip` (from, to), m: its screws' hoop the drive's own
    torque's and what keeps it shut under the joint's tilt."""
    ld = kl.loads(joint)
    long_ = 1e3 * (grip[1] - grip[0])
    asked = sized.leant(ld['swing'], math.hypot(ld['lateral'], ld['torsion']), 1e3 * can, long_)
    return 1e-3 * sized.band(asked, 1e3 * can, long_)


def _web(thick):
    """The highest stress, MPa, in the yoke's plate `thick` mm through, at its root - the top of the neck's hub,
    WEB_M wide: the head's nod in the plate's plane, its side moment and the neck's across it. The head's thrust
    down its axis is not the plate's: the crown's shoe bears it on the neck's band."""
    neck, head = kl.loads('neck'), kl.loads('head')
    wide = 1e3 * WEB_M
    across = head['torsion'] + neck['lateral']
    return 6.0 * max(head['lateral'], neck['swing']) / (thick * wide * wide) + 6.0 * across / (wide * thick * thick)


def _carried(thick, ring):
    """The stress, MPa, in the skull's carrier `thick` mm through round its hub's screws' circle: the head's thrust
    on its rim, `ring` mm round, and its tilt there."""
    head, circle = kl.loads('head'), kl.HUB[1]
    tilt = math.hypot(head['lateral'], head['torsion'])
    edge = head['axial'] / (2.0 * math.pi * ring) + tilt / (math.pi * ring * ring)     # the rim's load, N/mm
    return 6.0 * edge * ring / circle * (ring - circle) / (thick * thick)


@functools.lru_cache(maxsize=None)
def sizes():
    """{'web': the yoke's carbon plate, 'plate': its crown's floor, 'carrier': the skull's carbon carrier, mm
    thick, 'collar': the spine's foot's, mm, 'bore': the crossbar's band's wall, mm}: each the least its check asks."""
    from machine.body import structural
    allow, carbon = structural.PAHT_CF[2], structural.CARBON[2]
    head, spine, arm = kl.loads('head'), kl.loads('spine'), kl.loads('shoulder')
    web = next(sl.STEP * n for n in range(int(kl.WALL / sl.STEP), 80) if _web(sl.STEP * n) <= carbon)
    can, ring = 1e3 * stacks()['head']['motor'][0], 1e3 * stacks()['head']['gear'][0]
    span = max(2.0 * can - 1e3 * SHOE_M, 0.0)            # the thrust round the motor's back, the shoe under it
    plate = next(t for t in range(int(kl.WALL + kl.SCREWS['M5'][2]), 60)      # no thinner than sinks its screws
                 if 6.0 * head['axial'] * span / 4.0 / (2.0 * (can + 1e3 * PAST_M) * t * t) <= allow)
    carrier = next(sl.STEP * n for n in range(int(kl.WALL / sl.STEP), 60) if _carried(sl.STEP * n, ring) <= carbon)
    tube = (1e3 * placed()['spine'][1], 2.0, 0.0)
    bend = math.hypot(spine['swing'], spine['lateral'])
    collar = next((c for c in range(0, 13) if sl._mouth(c, tube, bend, {'swing': spine['torsion']}, spine['axial'])
                   <= allow), 12)
    bar = sl.sizes()['bar'][0]
    long_ = 2e3 * (sl.BLOCK_M - INSIDE_M)
    asked = sized.leant(2.0 * arm['swing'], math.hypot(arm['lateral'], arm['torsion']), bar, long_)
    return {'web': web, 'plate': plate, 'carrier': carrier, 'collar': collar, 'bore': sized.band(asked, bar, long_),
            'asked': asked}


@functools.lru_cache(maxsize=None)
def laid():
    """The layout, m, the torso's frame: {'bar': the crossbar's (y, z, radius), 'side': the block's body's half
    width, 'bore': its band's outside round the crossbar, 'under': that past the neck's housing - AIR_M under the
    neck's output ring and the yoke's plate -, 'seat': (the saddle's face's y, the spine's tube's z, its radius),
    'neck': (y, z), 'motor', 'house', 'ring': the neck's (from, to, radius) along x, 'band': its band's (from, to,
    outside), 'web': the yoke's carbon plate's (from, to) along x, 'cheek': its crown's cheek's, 'patch': where that
    cheek lies on the plate, ((low, high) over the neck's axis, (rear, front) across it), 'plate': the crown's
    floor's (bottom, top) up the torso, 'round': its radius, 'yaw': the head's axis's z, 'head': {'motor', 'house',
    'ring': (from, to) up the torso, 'can', 'gear': radii, 'band': (from, to, outside), 'carrier': the skull's
    (from, to)}}."""
    at, st, sz = placed(), stacks(), sizes()
    (ny, nz), lean = at['neck'], at['lean']
    cant = sl.laid()['cant']
    bar = (at['arm'][1] - cant * math.sin(lean), at['arm'][2], sl.laid()['bar'])
    motor, gear = st['neck']['motor'], st['neck']['gear']
    split = gear[0] + em.OUTPUT_SHARE * (gear[1] - gear[0])
    grip = (motor[0] + 0.002, split)
    band = grip + (motor[2] + _band('neck', motor[2], grip),)
    (can, long_m), (ring, long_g) = st['head']['motor'], st['head']['gear']
    floor = ny + HEAD_M - 0.002                          # the motor's back on the floor, its band 2 mm up it
    top = floor + long_m + long_g
    part = floor + long_m + em.OUTPUT_SHARE * long_g
    hold = (floor + 0.002, part)
    head = {'motor': (floor, floor + long_m), 'house': (floor + long_m, part), 'ring': (part, top), 'can': can,
            'gear': ring, 'band': hold + (can + _band('head', can, hold),), 'carrier': (top, top + 1e-3 * sz['carrier'])}
    bore = bar[2] + 1e-3 * (kl.BOND + sz['bore'])
    under = math.hypot(ny - bar[0], nz - bar[1]) - gear[2] - AIR_M
    low = floor - 1e-3 * sz['plate']
    return {'bar': bar, 'side': sl.BLOCK_M - INSIDE_M, 'bore': bore, 'under': min(bore, under),
            'seat': (bar[0] - bore - SADDLE_M, at['spine'][0], at['spine'][1]), 'neck': (ny, nz), 'motor': motor,
            'house': (gear[0], split, gear[2]), 'ring': (split, gear[1], gear[2]), 'band': band,
            'web': (gear[1], gear[1] + 1e-3 * sz['web']), 'cheek': (split + AIR_M, gear[1]),
            'patch': ((gear[2] + AIR_M, low - ny), (-NAPE_M, 0.5 * WEB_M - 0.002)),
            'plate': (low, floor), 'round': can + PAST_M, 'yaw': at['yaw'], 'head': head}


def held():
    """{name: margin} of what holds the neck and the head, each its strength over what it carries at the margin:
    the two hubs' screws parting and slipping, the yoke's carbon plate, its crown's floor and the skull's carrier,
    the screws that hold the crown's cheek on the plate - by friction at once, their holes' bearing at the margin -,
    the crown's shoe on the neck's band under the head's thrust, the neck's and the head's bands' screws under their
    tilts and the crossbar's band's under both arms' torques and one arm's, that band's wall where the neck's output
    ring passes over it, the block where its two bands lap, the spine's foot at its tube's mouth and its screws,
    the spine's tube; and the air between the yoke's plate and the shoulder's drum's rib where it leans in the
    nearest, over AIR_M."""
    from machine.body import structural
    lay, sz = laid(), sizes()
    allow, carbon, k = structural.PAHT_CF[2], structural.CARBON[2], structural.MARGIN
    neck, head, spine = kl.loads('neck'), kl.loads('head'), kl.loads('spine')
    count, circle = kl.HUB
    m5, m4 = kl.SCREWS['M5'][0], kl.SCREWS['M4'][0]
    out = {}
    for name, ld in (('neck', neck), ('head', head)):    # the thrust down the head's axis is the shoe's
        tilt = math.hypot(ld['lateral'], ld['torsion']) + (head['torsion'] if name == 'neck' else 0.0)
        pull = 2.0 * tilt / (count * circle)
        slide = max(ld['swing'], head['lateral'] if name == 'neck' else 0.0) / (count * circle)
        out[name + ' hub screws'] = m5 / (0.8 * pull)
        out[name + ' hub slip at 1x'] = kl.FRICTION * (m5 - 0.8 * pull / k) / (slide / k)
    can = 1e3 * lay['head']['can']
    out['yoke web'] = carbon / _web(float(sz['web']))
    (low, high), (rear, front) = lay['patch']
    lever = 0.5e3 * math.hypot(high - low - 2.0 * EDGE_M, front - rear - 2.0 * EDGE_M)
    turn = max(head['lateral'], neck['swing']) / (CHEEK * lever)          # a screw's shear, N
    out['cheek screws'] = min(kl.FRICTION * m5 / (turn / k), allow * 5.0 * 1e3 * (lay['cheek'][1] - lay['cheek'][0]) / turn)
    span = max(2.0 * can - 1e3 * SHOE_M, 0.0)
    floor = 6.0 * head['axial'] * span / 4.0
    out['crown floor'] = allow * 2.0 * (can + 1e3 * PAST_M) * sz['plate'] * sz['plate'] / max(floor, 1e-9)
    out['skull carrier'] = carbon / _carried(sz['carrier'], 1e3 * lay['head']['gear'])
    out['shoe'] = allow * 1e6 * SHOE_M * (lay['band'][1] - lay['band'][0]) / head['axial']
    for name, ld, drum, grip in (('neck', neck, lay['motor'][2], lay['band']), ('head', head, lay['head']['can'],
                                                                               lay['head']['band'])):
        long_ = 1e3 * (grip[1] - grip[0])
        out[name + ' band screws'] = sized.held(sized.leant(ld['swing'], math.hypot(ld['lateral'], ld['torsion']),
                                                            1e3 * drum, long_), 1e3 * drum)
    out['crossbar band screws'] = sized.held(sz['asked'], sl.sizes()['bar'][0])
    thin = 1e3 * (lay['under'] - lay['bar'][2]) - kl.BOND
    out['crossbar band'] = sized.hooped(sz['asked'], sl.sizes()['bar'][0], 2e3 * lay['side'], thin)
    wide = 1e3 * (lay['band'][1] - lay['band'][0])
    apart, ra, rb = math.hypot(lay['neck'][0] - lay['bar'][0], lay['neck'][1] - lay['bar'][1]), lay['bore'], lay['band'][2]
    lens = 2e3 * math.sqrt(max(0.0, ra * ra - ((apart * apart + ra * ra - rb * rb) / (2.0 * apart)) ** 2))
    out['block web'] = lens / sized.neck(wide, 'neck')   # the two bands lap: as wide as their lens, across the neck
    tube = sized.tube(math.hypot(spine['swing'], spine['lateral']), spine['torsion'], spine['axial'],
                      1e3 * lay['seat'][2])
    out['spine tube'] = carbon / tube[2]
    out['spine foot'] = allow / sl._mouth(sz['collar'], (tube[0], tube[1], 0.0), math.hypot(spine['swing'],
                                          spine['lateral']), {'swing': spine['torsion']}, spine['axial'])
    reach = (1e3 * lay['seat'][2] + 1e3 * em.BOLTS_PAST[0], 1e3 * lay['seat'][2] + 1e3 * em.BOLTS_PAST[1])
    pull = spine['swing'] / (4.0 * reach[1]) + spine['lateral'] / (4.0 * reach[0]) + spine['axial'] / 4.0
    out['spine foot screws'] = m4 / (0.8 * pull)
    arm, mount = placed(), sl.laid()                     # the drum's rib, turned up, leans in over the plate
    rim = (arm['arm'][0] - (mount['cant'] + mount['mouth']) * math.cos(arm['lean'])
           - (mount['meet'] + 0.5 * sl.RIB_UP) * math.sin(arm['lean']))
    out['drum air'] = (rim - lay['web'][1] - em.HEAD[1] * 0.25) / AIR_M
    return out


def main():
    lay, sz = laid(), sizes()
    print('crossbar at y %.1f mm, its band %.1f mm; yoke web %.1f, floor %d, carrier %.1f mm; spine collar %d mm'
          % (1e3 * lay['bar'][0], sz['bore'], sz['web'], sz['plate'], sz['carrier'], sz['collar']))
    print('neck band to %.1f mm round; floor %.1f .. %.1f mm up the torso; head band to %.1f mm, its hub at %.1f mm'
          % (1e3 * lay['band'][2], 1e3 * lay['plate'][0], 1e3 * lay['plate'][1], 1e3 * lay['head']['band'][2],
             1e3 * lay['head']['ring'][1]))
    for name, margin in held().items():
        print('  %-22s %.2f' % (name, margin))
    return 0


if __name__ == '__main__':
    sys.exit(main())
