"""The shoulder's layout: its drive turned over onto the arm, its hose a coil round the crossbar.

    python -X utf8 tools/render/shoulder_laid.py        # its sizes, its travel and what holds, each by its margin

What the shoulder is asked goes past a whole turn (STOOD), and a hose hung across a joint follows some 200 degrees:
so the drive stands the other way round. The gearbox's output ring is screwed to a carbon plate, the plate studded to
a printed foot bonded in the crossbar's end, and the motor, the board and the arm turn about that: the still side is
a tube and two discs, and the bus's hose a coil round the tube (`coil`) that a turn of the arm winds tighter or
looser, in a drum that turns with the arm. The drum's bore runs on the two discs' rims: struck sideways, the arm
leans on them and not on the gearbox's own bearing alone (`held`: 'rims'). A horn on the torso's block meets the
drum's rib at the travel's ends (`travel`), the dead arc laid past the farthest the robot swung an arm back while it
stood. The shoulder's axis is the figure's: square to the arm, which hangs leant out (`pivot`) - so the axis leans
that much up off the crossbar's, the foot's face cut that much off square, and the crossbar and its coil stand leant
in the sketch's frame (`canted`), the coil no tighter than clears them. The frame is the elbow sketch's: x along the axis
toward the torso - the gearbox at +x -, y up, z behind. Each size is the least its check asks; m, the checks in mm.
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

#: The farthest the robot swung an arm back and ahead through the scenarios it stood, deg (legload.py --all:
#: standing_jump -231, falling_on_butt 143). What its texts ask of the shoulder, unwrapped, goes round whole turns
#: (standing_jump one, standing_yawn two): a stop cuts some of that wherever it stands - laid EASE_DEG past the
#: arm's farthest back, so it is the fall's reach ahead that is cut: ahead of SWUNG, the farthest a scenario that is
#: no fall swung an arm ahead (hop_up 77).
STOOD, SWUNG, EASE_DEG = (-231.0, 143.0), 77.0, 2.0
#: The arm's push along the shoulder's axis, N: the most of its mean over 20 ms through the scenarios the build
#: stood (slip; tools/sim/legload.py --table).
THRUST = 337.0
#: Where the torso's block ends on the crossbar off the trunk's middle, and the air between its side and the drum's
#: mouth, m: the drums lean in at their tops, as the shoulders' axes lean, and between them stand the neck's drive,
#: the plate on its hub (`neck_laid.held`: 'drum air') and, beside its motor's back, the hose that takes the head's
#: motor's leads across the neck's joint (`neck_hosed.checked`: 'air').
BLOCK_M, MOUTH_M = 0.045, 0.002
#: Drawn: the studs a foot (as many as the hub's screws) and their washers' radius; the horn's and the rib's widths
#: where they meet and how far along the axis they lap; the drum's wall; the air a turning part keeps off a still
#: one; the rims' play in the bore, m.
STUDS, WASHER_M, HORN_M, RIB_M, HOLD_M, DRUM_M, AIR_M, PLAY_M = 8, 0.006, 0.010, 0.012, 0.014, 0.003, 0.0015, 0.0002
#: How far the rib stands off the drum, m: low - what it sweeps passes close under the neck's axis, and the neck's
#: yoke swings there (`neck_mount.aired`).
RIB_UP = 0.006
#: The coil: its turns with the arm at its travel's middle, and the air between two of them wound their tightest, m.
TURNS, GAP_M = 2.5, 0.0013
#: The foot's centring bore, mm, and the carbon plate's steps, mm.
BORE, STEP = 6.0, 0.5


def pivot():
    """(The upper arm's pivot off the trunk's middle, m, the arm's lean out, deg): the figure's."""
    from machine.body import figure
    seg = next(s for s in figure.SEGMENTS if s[0] == 'left_upper_arm')
    return float(seg[3][0]), float(seg[4])


def stack():
    """{'board', 'motor', 'gear': (radius, length)} of the shoulder's drive (`skeleton.cans`), m."""
    from machine.body import drives, skeleton
    names = [p for p, *_ in drives.along('left_shoulder')]
    cans = [c for c in skeleton.cans() if c[0] == 'left_shoulder']
    return {name: (float(c[4]), 2.0 * float(c[5])) for name, c in zip(names, cans)}


def carried():
    """(The loads at the margin - `knee_loads.loads` -, the moment tilting the hub, N mm, the push along the axis, N)."""
    from machine.body import structural
    ld = kl.loads('shoulder')
    return ld, math.hypot(ld['lateral'], ld['torsion']), structural.MARGIN * THRUST


def _mouth(collar, bar, tilt, ld, push):
    """The highest principal stress, MPa, in the printed foot at the crossbar's mouth: its spigot and a collar
    `collar` mm thick, the tilt bending them, the drive's own torque twisting them."""
    r, wall, _s = bar
    rings = [(r - wall - kl.BOND, BORE)] + ([(r + kl.BOND + collar, r + kl.BOND)] if collar else [])
    second = sum(math.pi / 4.0 * (a ** 4 - b ** 4) for a, b in rings)
    area, far = sum(math.pi * (a * a - b * b) for a, b in rings), max(a for a, _b in rings)
    normal, shear = tilt * far / second + push / area, ld['swing'] * far / (2.0 * second)
    return normal / 2.0 + math.hypot(normal / 2.0, shear)


@functools.lru_cache(maxsize=None)
def sizes():
    """{'bar': the crossbar's (radius, wall, stress), 'collar', 'foot', 'plate': mm thick, 'studs': their circle's
    radius, 'disc': the two discs' radius, mm, 'pull': a stud's at the margin, N}: each the least its check asks."""
    from machine.body import structural
    ld, tilt, push = carried()
    allow, carbon = structural.PAHT_CF[2], structural.CARBON[2]
    reach = 1e3 * (pivot()[0] - BLOCK_M)
    bar = sized.tube(tilt + ld['axial'] * reach, ld['swing'], push)
    collar = next((c for c in range(0, 13) if _mouth(c, bar, tilt, ld, push) <= allow), 12)
    studs = bar[0] + kl.BOND + collar + 1e3 * WASHER_M + 0.5
    disc = studs + 1e3 * WASHER_M + 1.0
    pull = 2.0 * tilt / (STUDS * studs) + push / STUDS
    wide = 2.0 * math.pi * studs / STUDS
    foot = next(t for t in range(int(kl.WALL), 40) if 6.0 * pull * (1e3 * WASHER_M + 0.5) / (wide * t * t)
                <= kl.ACROSS * allow)
    plate = next(STEP * n for n in range(int(kl.WALL / STEP), 60)
                 if 6.0 * pull * (studs - kl.HUB[1]) / (wide * (STEP * n) ** 2) <= carbon)
    return {'bar': bar, 'collar': collar, 'foot': foot, 'plate': plate, 'studs': studs, 'disc': disc, 'pull': pull}


def _gripped(lay, grip):
    """(The lever between the band's middle and the rims', mm, the torque whose hoop the band's screws pull, N mm):
    the drive's own and - a blow leaning the housing on its rims - half of what the band then bears, as the torque
    of that hoop."""
    ld, tilt, _push = carried()
    lever = 1e3 * (0.5 * (lay['foot'][1] + lay['plate'][0]) - 0.5 * (grip[0] + grip[1]))
    turn = 2.0 * math.pi * kl.FRICTION * 1e3 * stack()['motor'][0]
    return lever, ld['swing'] + 0.5 * tilt / lever * turn


@functools.lru_cache(maxsize=None)
def laid():
    """The shoulder's layout, m, the pivot at 0 on the axis: {'foot', 'plate', 'ring', 'house', 'motor', 'board':
    each (from, to) along x, 'bar': the crossbar's radius, 'disc', 'studs': radii, 'cant': where the foot's face
    toward the torso stands - the crossbar leaves it `lean` deg off the axis -, 'coil': (from, to) and 'mouth': the
    drum's, along the crossbar from that face, 'face': the guard's, 'guard': its (from, to, outside), 'band': (from,
    to, outside), 'bore': the drum's, 'skirt': the housing's bore over the ring,
    'wind': the coil's (tightest radius, ends' radius, most turns, least), 'meet': how far off the axis horn and rib
    meet, 'over': the saddle's hug round the guard, 'tube': the arm's radius, 'seat': (the arm's foot's half widths
    along the axis and across it, its face down the arm)}."""
    from tools.render import covered as cv
    sz, st, lean = sizes(), stack(), pivot()[1]
    tight, ends, most, least, (start, mouth), cant = _spool()
    bar = 1e-3 * sz['bar'][0]
    out, at = {}, cant                                  # the foot's face toward the torso, then outward part by part
    for name, long_ in (('foot', 1e-3 * sz['foot']), ('plate', 1e-3 * sz['plate']),
                        ('ring', (1.0 - em.OUTPUT_SHARE) * st['gear'][1]), ('house', em.OUTPUT_SHARE * st['gear'][1]),
                        ('motor', st['motor'][1]), ('board', st['board'][1])):
        out[name], at = (at - long_, at), at - long_
    can, ring = st['motor'][0], st['gear'][0]
    grip = (out['motor'][0] + 0.002, out['house'][1])
    wall = 1e-3 * sized.band(_gripped(out, grip)[1], 1e3 * can, 1e3 * (grip[1] - grip[0]))
    face = out['board'][0] - em.ROTOR_AIR - cv.WALL_M
    guard = cv.span(face, out['motor'][0], st['board'][0], can + wall)
    bore = wound()[1] + AIR_M
    arm = em._stack()[2]
    over = guard[2] + cv.FIT_M
    out.update(bar=bar, disc=1e-3 * sz['disc'], studs=1e-3 * sz['studs'], cant=cant, mouth=mouth,
               coil=(start, mouth), face=face, guard=guard, band=grip + (can + wall,), bore=bore,
               skirt=ring + AIR_M, wind=(tight, ends, most, least), meet=bore + DRUM_M + 0.5 * RIB_UP, tube=arm,
               lean=lean, over=over, seat=(arm + em.OVAL_PAST[0], arm + em.OVAL_PAST[1], over + em.SADDLE_M + em.SEAT_M))
    return out


@functools.lru_cache(maxsize=None)
def _spool():
    """(The coil's tightest radius at the foot's face, its ends' radius there, its most turns, its least, where it
    lies along the axis off the foot's face (from, to), where that face stands on the axis), m. Wound its tightest
    the coil clears the crossbar by a millimetre - wider toward the torso, as the crossbar leans off the axis -,
    its turns GAP_M apart, past the foot's collar; its ends a fitting clear of that collar; the foot where the
    block's side stands MOUTH_M past the coil's far end."""
    lo, hi = travel()
    most, least = TURNS + 0.5 * (hi - lo) / 360.0, TURNS - 0.5 * (hi - lo) / 360.0
    sz = sizes()
    bar = 1e-3 * sz['bar'][0]
    tight = bar + tees.HOSE_R + 0.001
    ends = max(tight * most / TURNS, bar + 1e-3 * (kl.BOND + sz['collar']) + tees.FITTING[1] + AIR_M)
    start = kl.COLLAR_SHARE * 2.0 * bar + AIR_M
    mouth = start + (most + 0.5) * (2.0 * tees.HOSE_R + GAP_M)
    reach, lean = pivot()
    return tight, ends, most, least, (start, mouth), (reach - BLOCK_M) / math.cos(math.radians(lean)) - mouth - MOUTH_M


def _lean():
    """((3, 3) the crossbar's frame's turn into the sketch's, (3,) where the foot's face stands): the crossbar
    leaves that face the arm's lean off the shoulder's axis, up toward the torso."""
    import numpy as np
    lean = math.radians(pivot()[1])
    c, s = math.cos(lean), math.sin(lean)
    return np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]]), np.array([_spool()[5], 0.0, 0.0])


def canted(points):
    """`points` (k, 3) laid along the crossbar from the foot's face - x along it, toward the torso - in the
    sketch's frame."""
    import numpy as np
    turn, at = _lean()
    return np.asarray(points, float) @ turn + at


@functools.lru_cache(maxsize=None)
def travel():
    """(from, to), deg: the shoulder's travel between its stops - a whole turn less what the horn and the rib take
    where they meet, from EASE_DEG past the farthest the robot swung an arm back while it stood."""
    tight, dead = 1e-3 * sizes()['bar'][0] + tees.HOSE_R + 0.001, 0.0
    for _ in range(8):                                   # the drum as wide as the coil's fewest turns ask
        half = 0.5 * (360.0 - dead) / 360.0
        meet = tight * (TURNS + half) / (TURNS - half) + tees.HOSE_R + AIR_M + DRUM_M + 0.5 * RIB_UP
        dead = math.degrees((HORN_M + RIB_M) / meet)
    return STOOD[0] - EASE_DEG, STOOD[0] - EASE_DEG + 360.0 - dead


def turns(turn):
    """The coil's turns with the arm turned `turn` deg: TURNS at its travel's middle, one more a whole turn on."""
    lo, hi = travel()
    return TURNS + (turn - 0.5 * (lo + hi)) / 360.0


def _wound(count, middle, k):
    """(k, 3): a coil of `count` turns about the axis from the block's fitting to the drum's, the sketch's frame:
    its ends at the fittings' radius and its middle turns `middle` round, eased between over half a turn - none
    tighter than clears the crossbar where it has leant off the axis."""
    import numpy as np
    tight, ends, _most, _least, (x0, x1), cant = _spool()
    u = np.linspace(0.0, 1.0, k)
    ease = np.clip(np.minimum(u, 1.0 - u) * 2.0 * count, 0.0, 1.0)
    ease = ease * ease * (3.0 - 2.0 * ease)
    along = x1 - tees.HOSE_R + (x0 - x1 + 2.0 * tees.HOSE_R) * u         # off the foot's face
    r = np.maximum(ends + (middle - ends) * ease, tight + math.tan(math.radians(pivot()[1])) * along)
    lo, hi = travel()
    start = math.pi - math.radians(360.0 * TURNS - 0.5 * (lo + hi))     # the drum's fitting under the axis at rest
    a = start + 2.0 * math.pi * count * u
    return np.c_[cant + along, r * np.cos(a), r * np.sin(a)]


def _long(points):
    import numpy as np
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


@functools.lru_cache(maxsize=None)
def cut():
    """The coil's hose's length, m: what its most turns take wound their tightest round the crossbar."""
    tight, _ends, most, _least, _zone, _cant = _spool()
    return _long(_wound(most, tight, 2000))


def coil(turn=0.0, k=400):
    """(k, 3): the hose's centre line with the arm turned `turn` deg, from the block's fitting to the drum's: a
    coil of `turns(turn)` about the axis, its middle turns as wide as its length (`cut`) lets them be."""
    count, lo, hi = turns(turn), _spool()[0], 4.0 * _spool()[0]
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if _long(_wound(count, mid, 800)) < cut() else (lo, mid)
    return _wound(count, 0.5 * (lo + hi), k)


@functools.lru_cache(maxsize=None)
def wound():
    """(The least radius the coil bends at, the farthest its hose's skin reaches off the axis, the least air that
    skin keeps off the leant crossbar), m, through the travel, every fifth degree."""
    import numpy as np
    from tools.render import hung
    lo, hi = travel()
    turn_, at = _lean()
    bends, far, near = [], [], []
    for turn in np.arange(lo, hi + 0.1, 5.0):
        line = coil(float(turn), 160)
        about = (line - at) @ turn_.T                    # along the crossbar
        bends.append(hung._radius(line))
        far.append(float(np.hypot(line[:, 1], line[:, 2]).max()) + tees.HOSE_R)
        near.append(float(np.hypot(about[:, 1], about[:, 2]).min()) - tees.HOSE_R - 1e-3 * sizes()['bar'][0])
    return min(bends), max(far), min(near)


def held():
    """{name: margin} of what holds the shoulder, each its strength over what it carries at the margin - past 1 it
    holds: the hub's screws and the foot's studs parting, the hub slipping at once, the plate and the foot bending
    between them, the foot at the crossbar's mouth, the spigot bearing in the tube, the crossbar and the arm's tube,
    the washers and the studs' holes in the foot, the housing's rims where the arm is struck sideways, the band's
    screws, the horn and the rib at a stop - the rib sheared off its drum, its face borne on -, and the coil: its least bend against the hose's own and its air round
    the crossbar - the drum is as wide as the coil ever grows."""
    from machine.body import mechatronics, structural
    ld, tilt, push = carried()
    sz, lay, k = sizes(), laid(), structural.MARGIN
    allow, carbon = structural.PAHT_CF[2], structural.CARBON[2]
    bar, pull, wide = sz['bar'], sz['pull'], 2.0 * math.pi * sz['studs'] / STUDS
    count, circle = kl.HUB
    m5, m4 = kl.SCREWS['M5'][0], kl.SCREWS['M4'][0]
    hub = 2.0 * tilt / (count * circle) + push / count
    slide = ld['swing'] / (count * circle) + ld['axial'] / count
    washer = math.pi * ((1e3 * WASHER_M) ** 2 - 2.2 ** 2)
    spigot, deep = bar[0] - bar[1] - kl.BOND, kl.SPIGOT_SHARE * 2.0 * bar[0]
    arm = sized.tube(math.hypot(ld['swing'], ld['lateral']), ld['torsion'], ld['axial'], 1e3 * lay['tube'])
    lever, gripped = _gripped(lay, lay['band'][:2])
    rims = 1e3 * (lay['foot'][1] - lay['plate'][0])
    hoop = sized._hoop(gripped, 1e3 * stack()['motor'][0])
    pinch = next((p for p, _h in sized.PINCHES if sized.PINCH * p >= hoop), sized.PINCHES[-1][0])
    stop = ld['swing'] / (1e3 * lay['meet'])             # the drive's own torque at the margin, on the rib, N
    bend, far, near = wound()
    hose = float(mechatronics.materials()['hoses']['bus'][4])
    return {'hub screws': m5 / (0.8 * hub), 'hub slip at 1x': kl.FRICTION * (m5 - 0.8 * hub / k) / (slide / k),
            'studs': m4 / (0.8 * pull), 'plate': carbon * wide * sz['plate'] ** 2 / (6.0 * pull * (sz['studs'] - circle)),
            'foot': kl.ACROSS * allow * wide * sz['foot'] ** 2 / (6.0 * pull * (1e3 * WASHER_M + 0.5)),
            'mouth': allow / _mouth(sz['collar'], bar, tilt, ld, push),
            'spigot': allow * 2.0 * spigot * deep * deep / (6.0 * tilt),
            'crossbar': carbon / bar[2], 'arm tube': carbon / arm[2], 'washers': allow * washer / pull,
            'stud holes': allow * 4.0 * sz['foot'] * STUDS * sz['studs'] / ld['swing'],
            'rims': allow * 2.0 * sz['disc'] * rims * lever / tilt, 'band screws': sized.PINCH * pinch / hoop,
            'stop': allow * 1e6 * HOLD_M * min(kl.ACROSS * RIB_M, RIB_UP - AIR_M) / stop,
            'coil bend': bend / hose, 'coil air': 1.0 + near / tees.HOSE_R}


def main():
    sz, lay = sizes(), laid()
    print('crossbar %.0f x %.0f mm; collar %d, foot %d, plate %.1f mm; %d studs on %.1f mm, discs %.1f mm round'
          % (2 * sz['bar'][0], sz['bar'][1], sz['collar'], sz['foot'], sz['plate'], STUDS, sz['studs'], sz['disc']))
    print('travel %.0f .. %.0f deg; coil %.0f mm of hose, %.2f .. %.2f turns; drum %.0f mm round'
          % (*travel(), 1e3 * cut(), lay['wind'][3], lay['wind'][2], 2e3 * (lay['bore'] + DRUM_M)))
    for name, margin in held().items():
        print('  %-16s %.2f' % (name, margin))
    return 0


if __name__ == '__main__':
    sys.exit(main())
