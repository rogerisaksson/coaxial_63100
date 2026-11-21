"""The shoulder's mount, sketched: the drive turned onto the arm, its hose a coil in a turning drum.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/shoulder_mount.py OUT.png

`shoulder_laid` sizes it; here its parts are drawn. What stays with the torso: the crossbar, its printed foot bonded in
its end - the foot's face cut the arm's lean off square -, the studs through the foot into the carbon plate, the plate
screwed on the gearbox's output ring, the torso's block's side - cut square to the axis too - with its horn, and the
coil's fitting on that side. What turns with the arm: the drive's housing, its motor and its board under its guard
and lid (`hip_clamped._drive`, the bus through the guard's wall), and one printed housing - a slit band round the
motor and the gearbox's housing, a skirt on over the output ring whose bore runs on the plate's and the foot's rims,
a cone out to the drum of honeycomb the coil lies in, the rib under the drum the horn meets -, and on it the
abduction (`shoulder_rolled`): the hood over the housing with its drive behind, the board under, and the yoke the arm
hangs in, raised about the axis ahead. The hose is drawn as it lies at a turn (`shoulder_laid.coil`), a fitting along
its way at each end: on the block's side, and on a stem off the drum's wall, the leads on down the rib. Frame:
`shoulder_laid`'s.
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
from tools.render import knee_loads as kl  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import mounting as m  # noqa: E402
from tools.render import shoulder_laid as sl  # noqa: E402
from tools.render import tees  # noqa: E402

#: The gap that lets the band pinch beside the skirt, the block's side's thickness, how far that side stands round
#: past the coil's fitting, a nut's radius and height, m.
RELIEF_M, SIDE_M, PAST_M, NUT = 0.0012, 0.010, 0.004, (0.007 / 2.0, 0.0032)
#: The block's side as far round the crossbar as the block's own band (`neck_laid`), m, and the half angle of its
#: arm out to the coil's fitting, rad: the neck's drive stands close over it.
HUB_M, ARM = 0.012, 0.45
#: The pairs of parts the suite counts crossings of: what turns against what stays, the arm hanging - the rest
#: are a screw in its hole, a band on its drum, a lead on its lug.
STILL, TURNING = ('crossbar', 'crossbar foot', 'studs', 'hub plate', 'block'), ('housing', 'guard', 'lid')


def _leant(tris):
    """`tris` drawn along the crossbar from the foot's face, in the sketch's frame (`shoulder_laid.canted`)."""
    return [(sl.canted(tri), colour) for tri, colour in tris]


def _arc(inner, outer, x0, x1, low, high, colour, k=12):
    """A piece of a ring round the axis between radii and bearings (rad, +y toward +z), from x0 to x1, rounded."""
    import numpy as np
    turn = np.linspace(low, high, k)
    outline = np.r_[np.c_[outer * np.cos(turn), outer * np.sin(turn)],
                    np.c_[inner * np.cos(turn[::-1]), inner * np.sin(turn[::-1])]]
    return em._rounded(outline, x0, x1, colour, axis=0, r=0.0015)


def _banjo(at, along, stem=(0.0, 0.0, 0.0), out=0.0):
    """A fitting lying back along the hose's way `along` from its end `at`, and its stem into what carries it:
    the vector `stem` from the fitting's back, or straight off the axis to the radius `out` about it."""
    import numpy as np
    at, along = np.asarray(at, float), np.asarray(along, float)
    back = at - tees.FITTING[0] * along / np.linalg.norm(along)
    off = float(np.hypot(back[1], back[2]))
    reach = np.array([0.0, back[1], back[2]]) * (out - off) / off if out else np.asarray(stem, float)
    return (km._rod(back, at, tees.FITTING[1], tees.PIPE_INK)
            + km._rod(back, back + reach, tees.PIPE[1], tees.PIPE_INK))


def _horn():
    """(from, to), rad: the bearings the block's horn stands between - the dead arc, less the rib's half width at
    either stop."""
    lay, (lo, hi) = sl.laid(), sl.travel()
    half = 0.5 * sl.RIB_M / lay['meet']
    return math.pi + math.radians(hi) + half, math.pi + math.radians(lo) + 2.0 * math.pi - half


@functools.lru_cache(maxsize=None)
def still():
    """{name: tris} of what stays with the torso."""
    import numpy as np
    lay, sz = sl.laid(), sl.sizes()
    bar, disc, play = lay['bar'], lay['disc'], kl.BOND * 1e-3
    (f0, f1), (p0, p1), ring = lay['foot'], lay['plate'], sl.stack()['gear'][0]
    side = lay['cant'] + lay['mouth'] + sl.MOUTH_M
    collar = [(-0.002, bar + play, bar + play + 1e-3 * sz['collar']),
              (kl.COLLAR_SHARE * 2.0 * bar, bar + play, bar + play + 1e-3 * sz['collar'])]
    studs = []
    for i in range(sl.STUDS):
        a = (i + 0.5) * 2.0 * math.pi / sl.STUDS
        at = (lay['studs'] * math.cos(a), lay['studs'] * math.sin(a))
        studs += em._drum(sl.WASHER_M, f1, f1 + 0.001, em.BOLT, at) + em._drum(NUT[0], f1, f1 + 0.001 + NUT[1], em.BOLT, at)
    low, high = _horn()
    far = lay['meet'] + 0.5 * sl.RIB_UP
    start = sl.coil(0.0, 9)[0]
    round_, at = float(np.hypot(*start[1:])) + tees.FITTING[1] + PAST_M, math.atan2(start[2], start[1])
    hub = bar + HUB_M                                    # the block's own band round the crossbar, no wider
    block = (em._drum(hub, side, side + SIDE_M, em.FRAME)
             + _arc(hub - 0.002, round_, side, side + SIDE_M, at - ARM, at + ARM, em.FRAME)
             + _arc(hub - 0.002, far, side, side + SIDE_M, low, high, em.FRAME)
             + _arc(far - sl.RIB_UP + sl.AIR_M, far, side - sl.MOUTH_M - sl.HOLD_M, side, low, high, em.FRAME))
    return {'crossbar': _leant(em._drum(bar, 0.0, lay['mouth'] + sl.MOUTH_M + SIDE_M + sl.BLOCK_M, em.CARBON)),
            'crossbar foot': em._drum(disc, f0, f1, em.FRAME) + _leant(cl.turned(collar, None)),
            'studs': studs, 'hub plate': em._drum(disc, p0, p1, em.CARBON),
            'output ring': em._drum(ring, lay['ring'][0], lay['ring'][1], em.OUTPUT), 'block': block}


def _housing(lay, can, ring):
    """The printed housing that turns with the arm: band, skirt, rims' bore, cone, drum and rib."""
    band0, band1, out = lay['band']
    plate0, cant, rims = lay['plate'][0], lay['cant'], lay['disc'] + sl.PLAY_M
    bore = lay['bore']
    cone, mouth = cant + bore - rims, cant + lay['mouth']
    shell = cl.turned([(band0, can, out), (lay['motor'][1], can, out), (lay['motor'][1], ring, out),
                       (band1, ring, out)], 0.0)
    shell += cl.turned([(band1 + RELIEF_M, lay['skirt'], out), (plate0, lay['skirt'], out), (plate0, rims, out),
                        (cant, rims, out), (cone, bore, bore + sl.DRUM_M)], None)
    shell += cl.honeycomb(bore, bore + sl.DRUM_M, cone, mouth)
    low = bore + sl.DRUM_M
    rib = [(-(low + sl.RIB_UP), -0.5 * sl.RIB_M), (-(low + sl.RIB_UP), 0.5 * sl.RIB_M), (-(bore + 0.0005), 0.5 * sl.RIB_M),
           (-(bore + 0.0005), -0.5 * sl.RIB_M)]
    return shell + em._rounded(rib, cone - 0.002, mouth, em.FRAME, axis=0, r=0.0015)


@functools.lru_cache(maxsize=None)
def turning():
    """{name: tris} of what turns with the arm, the arm hanging: the drive under its guard and lid, its leads, the
    housing, the coil's fitting on its stem."""
    from tools.render import hip_clamped as hc
    lay, st = sl.laid(), sl.stack()
    can, ring = st['motor'][0], st['gear'][0]
    drive = hc._drive({'board': lay['board'] + (st['board'][0],), 'motor': lay['motor'] + (can,),
                       'gear': (lay['house'][0], lay['ring'][1], ring)}, lay['band'][2] - can, math.pi, leave=math.pi)
    line = sl.coil(0.0, 400)
    return {'drive': drive['still'], 'leads': drive['leads'], 'guard': drive['guard'], 'lid': drive['lid'],
            'housing': _housing(lay, can, ring),
            'coil stem': _banjo(line[-1], line[-1] - line[-2], out=lay['bore'] + 0.001)}


def posed(turn=0.0, roll=0.0):
    """{name: tris}: the shoulder with the arm turned `turn` deg and raised `roll` - what turns turned, the
    abduction on it (`shoulder_rolled`: 'abduction', 'yoke', 'span'), the bus on through it (`shoulder_hosed`:
    'bus', 'bus on the arm'), the coil as it then lies."""
    from tools.render import shoulder_hosed as sh, shoulder_rolled as sr
    line = sl.coil(turn, 400)
    out = dict(still(), **{name: km._turn(tris, turn) for name, tris in turning().items()})
    raised, bus = sr.posed(roll), sh.parts()
    out.update({'abduction': km._turn(raised['still'], turn), 'yoke': km._turn(raised['yoke'], turn),
                'span': km._turn(raised['span'], turn), 'bus': km._turn(bus['still'], turn),
                'bus on the arm': km._turn(sr.rolled(bus['yoke'], roll), turn)})
    out['coil'] = tees.hose(line) + _banjo(line[0], line[0] - line[1], (sl.MOUTH_M + tees.HOSE_R + 0.001, 0.0, 0.0))
    return out


def _abduction():
    """[(tri, colour)]: the abduction that turns with the arm about the shoulder's axis, the arm hanging - its hood,
    drive and board, the yoke above the arm's seat (`shoulder_rolled.parts`) and the bus on them
    (`shoulder_hosed.parts`)."""
    from tools.render import shoulder_hosed as sh, shoulder_rolled as sr
    low = -sr.laid()['down']
    return (sr.parts()['still'] + sr.parts()['ring'] + [t for t in sr.parts()['yoke'] if t[0][:, 1].min() > low]
            + sh.parts()['still'] + sh.parts()['yoke'])


@functools.lru_cache(maxsize=None)
def aired(step=10.0, ease=3.0):
    """The least air between what turns and what stays, m, through the travel `ease` deg short of each stop - at a
    stop horn and rib meet -, every `step` deg; the rims in their bore apart: they run there."""
    import numpy as np
    lo, hi = sl.travel()
    h = 0.006
    a = km._points([t for name in STILL if name != 'hub plate' for t in still()[name]
                    if name != 'crossbar foot' or t[0][:, 0].min() > sl.laid()['foot'][1] - 1e-9])
    pts = km._points([t for name in TURNING for t in turning()[name]] + _abduction())
    keys = km._key(np.floor(a / h).astype(np.int64))
    order = np.argsort(keys)
    return min(float(km._least(keys[order], a[order], pts @ km._rx(float(deg)).T, h))
               for deg in np.arange(lo + ease, hi - ease + 0.1, step))


def checked():
    """{'held': the least of `shoulder_laid.held`'s margins, 'air': what turns off what stays through the travel,
    mm, 'crossing': triangle pairs of what turns through what stays with the arm hanging, 'coil': the hose's
    crossings with every part but its own fittings at five turns of the arm, 'bend': (the least radius the coil is
    drawn bent at, the hose's own least), mm, 'travel': (from, to) deg, 'asked': the farthest the robot swung an arm
    back while it stood and ahead short of a fall}: what the suite holds."""
    import numpy as np
    from machine.body import mechatronics
    from tools.render import clips
    lo, hi = sl.travel()
    crossing = sum(clips.crossed(tris, still()[b]) for tris in [turning()[a] for a in TURNING] + [_abduction()]
                   for b in STILL)
    coil, bend = 0, []
    for turn in np.linspace(lo, hi, 5):
        parts = posed(float(turn))
        hose = tees.hose(sl.coil(float(turn), 400))
        coil += sum(clips.crossed(hose, tris) for name, tris in parts.items() if name not in ('coil', 'coil stem'))
        bend.append(tees.tightest(hose))
    return {'held': round(min(sl.held().values()), 3), 'air': round(1e3 * aired(), 2), 'crossing': crossing,
            'coil': coil, 'bend': (round(1e3 * min(bend), 1), 1e3 * float(mechatronics.materials()['hoses']['bus'][4])),
            'travel': tuple(round(v, 1) for v in (lo, hi)), 'asked': (sl.STOOD[0], sl.SWUNG)}


def main(argv=None):
    from PIL import Image, ImageDraw
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    args = parser.parse_args(argv)
    lo, hi = sl.travel()
    views = [(0.0, 35.0, 18.0), (0.0, -150.0, 14.0), (lo, 35.0, 18.0), (hi, 35.0, 18.0)]
    sheet = Image.new('RGB', (760 * len(views), 800), (18, 18, 18))
    for i, (turn, yaw, pitch) in enumerate(views):
        tris = [t for part in posed(turn).values() for t in part]
        sheet.paste(m.draw(tris, yaw=yaw, pitch=pitch, px=760, half=0.115, centre=(0.035, -0.03, 0.0)), (760 * i, 40))
    ImageDraw.Draw(sheet).text((12, 10), 'shoulder: hanging, from ahead | from behind | at its stops %.0f and %.0f deg'
                               % (lo, hi), fill=(235, 235, 235))
    sheet.save(args.out)
    print(args.out, 'travel %.0f .. %.0f deg' % (lo, hi))
    return 0


if __name__ == '__main__':
    sys.exit(main())
