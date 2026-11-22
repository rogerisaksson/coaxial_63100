"""The shoulder's pitch layout: boardless MN6007 + B on the gimbal, concentric bus coil.

    python -X utf8 tools/render/shoulder_laid.py        # its sizes, its travel and what holds, each by its margin

The shoulder's inverter board sits in the upper-back bay on the torso (`[boards] shoulder`), leaving only the compact
MN6007 motor and B gearbox (71.2 mm round, 48 mm long) at the glenohumeral pivot. Stage 1 (`shoulder_rolled`) pivots
the abduction gimbal about the z axis through the joint; Stage 2 (here) turns the arm in pitch (`BACK` to `AHEAD`)
about the x axis on that gimbal so pitching the arm never twists the pectoral pushrod. The gearbox's output ring is
bolted to a carbon hub plate studded onto a printed foot on the gimbal's concentric hollow arbor (`ARBOR_M`), and the
bus's hose winds in a compact 2.25-turn concentric coil round that arbor inside a honeycomb drum that turns with the
motor, housing and upper-arm saddle. Frame: x along the pitch axis toward the torso, y up, z behind, pivot at 0; m.
"""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import elbow_mount as em  # noqa: E402
from tools.render import knee_loads as kl  # noqa: E402
from tools.render import sized  # noqa: E402
from tools.render import tees  # noqa: E402

#: Pitch stops, deg, positive the arm ahead: human extension (`BACK`), farthest non-fall swing (`SWUNG`), overhead (`AHEAD`).
BACK, SWUNG, AHEAD = -60.0, 77.0, 180.0
#: Axial push along the shoulder's axis at 1x, N (`tools/sim/legload.py --table`).
THRUST = 337.0
#: Where the torso's central block ends on the crossbar off the trunk's middle, and the air at the coil drum's mouth, m.
BLOCK_M, MOUTH_M = 0.045, 0.002
#: Concentric hollow carbon arbor radius and wall on Stage 1's gimbal, and where the foot's inner face stands on x, m.
ARBOR_M, ARBOR_WALL_MM, FOOT_X = 0.0175, 2.0, 0.015
#: Drawn hardware and stop geometry, m.
STUDS, WASHER_M, HORN_M, RIB_M, HOLD_M, DRUM_M, AIR_M, PLAY_M = 8, 0.0055, 0.010, 0.012, 0.012, 0.003, 0.0015, 0.0002
#: Stop rib height off the coil drum, m.
RIB_UP = 0.0055
#: Concentric bus coil: turns at mid-travel and gap between turns wound tightest, m.
TURNS, GAP_M = 2.25, 0.0012
#: Bearing of the stop rib and drum fitting with the arm at mid-travel, deg from up toward behind.
MIDDLE_DEG = 180.0
#: Centring bore, mm, and carbon plate thickness step, mm.
BORE, STEP = 6.0, 0.5


def pivot():
    """(The upper arm's pivot off the trunk's middle, m, the arm's lean out, deg): the figure's."""
    from machine.body import figure
    seg = next(s for s in figure.SEGMENTS if s[0] == 'left_upper_arm')
    return float(seg[3][0]), float(seg[4])


def stack():
    """{'motor', 'gear', 'board': (radius, length)} of the shoulder's drive kind (`drives`), m."""
    from machine.body import drives, skeleton
    names = [p for p, *_ in drives.along('left_shoulder')]
    cans = [c for c in skeleton.cans() if c[0] == 'left_shoulder']
    out = {name: (float(c[4]), 2.0 * float(c[5])) for name, c in zip(names, cans)}
    if 'board' not in out:
        out['board'] = (float(drives.of('left_shoulder')[1].board[1]), float(drives.tall('left_shoulder')))
    return out


def carried():
    """(Loads at the margin - `knee_loads.loads` -, hub tilting moment, N mm, axial push at margin, N)."""
    from machine.body import structural
    ld = kl.loads('shoulder')
    return ld, math.hypot(ld['lateral'], ld['torsion']), structural.MARGIN * THRUST


def _mouth(collar, bar, tilt, ld, push):
    """Highest principal stress, MPa, in a printed socket at a tube's mouth (`spigot` + `collar` mm)."""
    r, wall, _s = bar
    rings = [(r - wall - kl.BOND, BORE)] + ([(r + kl.BOND + collar, r + kl.BOND)] if collar else [])
    second = sum(math.pi / 4.0 * (a ** 4 - b ** 4) for a, b in rings)
    area, far = sum(math.pi * (a * a - b * b) for a, b in rings), max(a for a, _b in rings)
    normal, shear = tilt * far / second + push / area, ld['swing'] * far / (2.0 * second)
    return normal / 2.0 + math.hypot(normal / 2.0, shear)


@functools.lru_cache(maxsize=None)
def sizes():
    """{'bar': crossbar (radius, wall, stress), 'arbor': concentric arbor (radius, wall, stress), 'collar', 'foot',
    'plate': mm, 'studs', 'disc': radii mm, 'pull': stud pull N}: each the least its check asks."""
    from machine.body import structural
    ld, tilt, push = carried()
    allow, carbon = structural.PAHT_CF[2], structural.CARBON[2]
    reach = 1e3 * (pivot()[0] - BLOCK_M)
    bar = sized.tube(tilt + ld['axial'] * reach, ld['swing'], push)
    arbor = sized.tube(tilt, ld['swing'], push, 1e3 * ARBOR_M)
    collar = next((c for c in range(0, 13) if _mouth(c, arbor, tilt, ld, push) <= allow), 12)
    studs = max(1e3 * ARBOR_M + kl.BOND + collar + 1e3 * WASHER_M + 0.5, kl.HUB[1] + 4.5)
    disc = studs + 1e3 * WASHER_M + 1.0
    pull = 2.0 * tilt / (STUDS * studs) + push / STUDS
    wide = 2.0 * math.pi * studs / STUDS
    foot = next(t for t in range(int(kl.WALL), 40) if 6.0 * pull * (1e3 * WASHER_M + 0.5) / (wide * t * t)
                <= kl.ACROSS * allow)
    plate = next(STEP * n for n in range(int(kl.WALL / STEP), 60)
                 if 6.0 * pull * (studs - kl.HUB[1]) / (wide * (STEP * n) ** 2) <= carbon)
    return {'bar': bar, 'arbor': arbor, 'collar': collar, 'foot': foot, 'plate': plate,
            'studs': studs, 'disc': disc, 'pull': pull}


def _gripped(lay, grip):
    """(Lever between band middle and rims middle, mm, torque whose hoop the band's screws pull, N mm)."""
    ld, tilt, _push = carried()
    lever = 1e3 * (0.5 * (lay['foot'][1] + lay['plate'][0]) - 0.5 * (grip[0] + grip[1]))
    turn = 2.0 * math.pi * kl.FRICTION * 1e3 * stack()['motor'][0]
    return lever, ld['swing'] + 0.5 * tilt / lever * turn


@functools.lru_cache(maxsize=None)
def _spool():
    """(Coil tightest radius, ends radius, most turns, least turns, (start, mouth) off FOOT_X, crossbar cant), m."""
    lo, hi = travel()
    most, least = TURNS + 0.5 * (hi - lo) / 360.0, TURNS - 0.5 * (hi - lo) / 360.0
    sz = sizes()
    tight = ARBOR_M + tees.HOSE_R + 0.001
    ends = max(tight * most / TURNS, ARBOR_M + 1e-3 * (kl.BOND + sz['collar']) + tees.FITTING[1] + AIR_M)
    start = kl.COLLAR_SHARE * 2.0 * ARBOR_M + AIR_M
    mouth = start + (most + 0.5) * (2.0 * tees.HOSE_R + GAP_M)
    reach, lean = pivot()
    cant = (reach - BLOCK_M) / math.cos(math.radians(lean)) - 0.0528
    return tight, ends, most, least, (start, mouth), cant


@functools.lru_cache(maxsize=None)
def laid():
    """The pitch drive's layout, m, pivot at 0: boardless MN6007 + B on Stage 1's concentric arbor."""
    from tools.render import covered as cv
    sz, st, lean = sizes(), stack(), pivot()[1]
    tight, ends, most, least, (start, mouth), cant = _spool()
    bar = 1e-3 * sz['bar'][0]
    out, at = {}, FOOT_X
    for name, long_ in (('foot', 1e-3 * sz['foot']), ('plate', 1e-3 * sz['plate']),
                        ('ring', (1.0 - em.OUTPUT_SHARE) * st['gear'][1]), ('house', em.OUTPUT_SHARE * st['gear'][1]),
                        ('motor', st['motor'][1])):
        out[name], at = (at - long_, at), at - long_
    can, ring = st['motor'][0], st['gear'][0]
    grip = (out['motor'][0] + 0.002, out['house'][1])
    wall = 1e-3 * sized.band(_gripped(out, grip)[1], 1e3 * can, 1e3 * (grip[1] - grip[0]))
    face = out['motor'][0] - 0.003 - cv.WALL_M
    guard = (face, out['motor'][0] + 0.001, can + wall)
    bore = wound()[1] + AIR_M
    saddle_h = 0.011
    top_arm = can + wall + saddle_h
    out.update(bar=bar, arbor=ARBOR_M, disc=1e-3 * sz['disc'], studs=1e-3 * sz['studs'], cant=cant, f1=FOOT_X,
               mouth=FOOT_X + mouth - cant, coil=(start, mouth), face=face, guard=guard, band=grip + (can + wall,),
               bore=bore, skirt=ring + AIR_M, wind=(tight, ends, most, least), meet=bore + DRUM_M + 0.5 * RIB_UP,
               tube=em._stack()[2], lean=lean, saddle=saddle_h, top=top_arm, down=top_arm + em.SEAT_M)
    return out


def _lean():
    """((3, 3) crossbar frame's turn into sketch frame, (3,) where crossbar axis meets `cant`)."""
    import numpy as np
    lean = math.radians(pivot()[1])
    c, s = math.cos(lean), math.sin(lean)
    return np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]]), np.array([_spool()[5], 0.0, 0.0])


def canted(points):
    """`points` (k, 3) laid along the torso's crossbar from `cant` toward the trunk's middle, in sketch frame."""
    import numpy as np
    turn, at = _lean()
    return np.asarray(points, float) @ turn + at


@functools.lru_cache(maxsize=None)
def travel():
    """(from, to), deg: the shoulder's pitch travel between its stops, BACK to AHEAD."""
    from machine.errors import MachineError
    lo, hi = BACK, AHEAD
    half = 0.5 * (hi - lo) / 360.0
    tight = ARBOR_M + tees.HOSE_R + 0.001
    meet = tight * (TURNS + half) / (TURNS - half) + tees.HOSE_R + AIR_M + DRUM_M + 0.5 * RIB_UP
    if 360.0 - (hi - lo) < math.degrees((HORN_M + RIB_M) / meet):
        raise MachineError('the shoulder travels %.0f deg: no room left between its stops for horn and rib' % (hi - lo))
    return lo, hi


def turns(turn):
    """The coil's turns with the arm pitched `turn` deg: TURNS at mid-travel, one more a whole turn on."""
    lo, hi = travel()
    return TURNS + (turn - 0.5 * (lo + hi)) / 360.0


def drummed():
    """Bearing of the drum's rib and coil fitting with the arm hanging, deg from up toward behind."""
    lo, hi = travel()
    return MIDDLE_DEG - 0.5 * (lo + hi)


def _wound(count, middle, k):
    """(k, 3): concentric coil of `count` turns round Stage 1's arbor from the gimbal flange to the drum's stem."""
    import numpy as np
    _tight, ends, _most, _least, (x0, x1), _cant = _spool()
    u = np.linspace(0.0, 1.0, k)
    ease = np.clip(np.minimum(u, 1.0 - u) * 2.0 * count, 0.0, 1.0)
    ease = ease * ease * (3.0 - 2.0 * ease)
    along = x1 - tees.HOSE_R + (x0 - x1 + 2.0 * tees.HOSE_R) * u
    r = ends + (middle - ends) * ease
    start = math.radians(drummed() - 360.0 * turns(0.0))
    a = start + 2.0 * math.pi * count * u
    return np.c_[FOOT_X + along, r * np.cos(a), r * np.sin(a)]


def _long(points):
    import numpy as np
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


@functools.lru_cache(maxsize=None)
def cut():
    """The coil's hose length, m: what its most turns take wound tightest round the concentric arbor."""
    tight, _ends, most, _least, _zone, _cant = _spool()
    return _long(_wound(most, tight, 2000))


def coil(turn=0.0, k=400):
    """(k, 3): hose centre line with the arm pitched `turn` deg, from Stage 1's flange fitting to the drum's."""
    count, lo, hi = turns(turn), _spool()[0], 4.0 * _spool()[0]
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if _long(_wound(count, mid, 800)) < cut() else (lo, mid)
    return _wound(count, 0.5 * (lo + hi), k)


@functools.lru_cache(maxsize=None)
def wound():
    """(Least bend radius, farthest hose skin off the pitch axis, least air off the concentric arbor), m."""
    import numpy as np
    from tools.render import hung
    lo, hi = travel()
    bends, far, near = [], [], []
    for turn in np.arange(lo, hi + 0.1, 5.0):
        line = coil(float(turn), 160)
        bends.append(hung._radius(line))
        rad = np.hypot(line[:, 1], line[:, 2])
        far.append(float(rad.max()) + tees.HOSE_R)
        near.append(float(rad.min()) - tees.HOSE_R - ARBOR_M)
    return min(bends), max(far), min(near)


def held():
    """{name: margin} of every pitch-stage member at the margin (> 1 holds)."""
    from machine.body import mechatronics, structural
    ld, tilt, push = carried()
    sz, lay, k = sizes(), laid(), structural.MARGIN
    allow, carbon = structural.PAHT_CF[2], structural.CARBON[2]
    bar, arbor, pull = sz['bar'], sz['arbor'], sz['pull']
    wide = 2.0 * math.pi * sz['studs'] / STUDS
    count, circle = kl.HUB
    m5, m4 = kl.SCREWS['M5'][0], kl.SCREWS['M4'][0]
    hub = 2.0 * tilt / (count * circle) + push / count
    slide = ld['swing'] / (count * circle) + ld['axial'] / count
    washer = math.pi * ((1e3 * WASHER_M) ** 2 - 2.2 ** 2)
    spigot, deep = arbor[0] - arbor[1] - kl.BOND, kl.SPIGOT_SHARE * 2.0 * arbor[0]
    arm = sized.tube(math.hypot(ld['swing'], ld['lateral']), ld['torsion'], ld['axial'], 1e3 * lay['tube'])
    lever, gripped = _gripped(lay, lay['band'][:2])
    rims = 1e3 * (lay['foot'][1] - lay['plate'][0])
    hoop = sized._hoop(gripped, 1e3 * stack()['motor'][0])
    pinch = next((p for p, _h in sized.PINCHES if sized.PINCH * p >= hoop), sized.PINCHES[-1][0])
    stop = ld['swing'] / (1e3 * lay['meet'])
    bend, _far, near = wound()
    hose = float(mechatronics.materials()['hoses']['bus'][4])
    return {'hub screws': m5 / (0.8 * hub), 'hub slip at 1x': kl.FRICTION * (m5 - 0.8 * hub / k) / (slide / k),
            'studs': m4 / (0.8 * pull), 'plate': carbon * wide * sz['plate'] ** 2 / (6.0 * pull * (sz['studs'] - circle)),
            'foot': kl.ACROSS * allow * wide * sz['foot'] ** 2 / (6.0 * pull * (1e3 * WASHER_M + 0.5)),
            'mouth': allow / _mouth(sz['collar'], arbor, tilt, ld, push),
            'spigot': allow * 2.0 * spigot * deep * deep / (6.0 * tilt),
            'crossbar': carbon / bar[2], 'arbor': carbon / arbor[2], 'arm tube': carbon / arm[2],
            'washers': allow * washer / pull, 'stud holes': allow * 4.0 * sz['foot'] * STUDS * sz['studs'] / ld['swing'],
            'rims': allow * 2.0 * sz['disc'] * rims * lever / tilt, 'band screws': sized.PINCH * pinch / hoop,
            'stop': allow * 1e6 * HOLD_M * min(kl.ACROSS * RIB_M, RIB_UP - AIR_M) / stop,
            'coil bend': bend / hose, 'coil air': 1.0 + near / tees.HOSE_R}


def main():
    sz, lay = sizes(), laid()
    print('crossbar %.0f x %.0f mm, arbor %.0f x %.1f mm; collar %d, foot %d, plate %.1f mm; discs %.1f mm'
          % (2 * sz['bar'][0], sz['bar'][1], 2 * sz['arbor'][0], sz['arbor'][1], sz['collar'], sz['foot'],
             sz['plate'], sz['disc']))
    print('travel %.0f .. %.0f deg; coil %.0f mm of hose, %.2f .. %.2f turns; drum %.0f mm round'
          % (*travel(), 1e3 * cut(), lay['wind'][3], lay['wind'][2], 2e3 * (lay['bore'] + DRUM_M)))
    for name, margin in held().items():
        print('  %-16s %.2f' % (name, margin))
    return 0


if __name__ == '__main__':
    sys.exit(main())
