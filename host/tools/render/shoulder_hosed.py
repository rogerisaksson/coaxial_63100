"""The bus through the shoulder: the pitch's board, the abduction's, the span and on into the arm.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/shoulder_hosed.py OUT.png

The trunk comes off the coil onto the pitch's board through its guard's wall underneath (`shoulder_mount`), and
on straight down a grommet across the air to the abduction's board below (`shoulder_rolled`), the chain's junction.
Out of that board's guard a black pipe leaves toward the torso, where the yoke's saddle never comes, bent ahead
(`tees.pipe`); from its fitting the hose lies still, rising ahead past the yoke's front plate beside the hood's
front leg, up and over into the back of the span's still fitting, a stem from that fitting to the leg - held by
clips on what turns with the pitch (`tees.clipped`). Across the abduction the span hangs free (`shoulder_rolled`);
from its fitting on the lever the hose lies still down ahead of the front plate into the outlet of the channel
through the saddle and the arm's seat, on down the upper arm's tube's bore to the elbow (`elbow_mount`), no hole
in the carbon. Each hose bends no tighter than its own least radius and keeps AIR_M off what turns by it through
the abduction's travel. Frame: `shoulder_laid`'s.
"""
import argparse
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import knee_mount as km  # noqa: E402
from tools.render import mounting as m  # noqa: E402
from tools.render import shoulder_laid as sl  # noqa: E402
from tools.render import shoulder_rolled as sr  # noqa: E402
from tools.render import tees  # noqa: E402

#: The air a hose keeps off what turns by it, m; the bearing round the pitch's axis, from under it toward ahead,
#: the board's run leaves the drum at - on round from the pipe's end at its reach, inboard of the yoke's front
#: plate's sweep -, up and ahead into the span's still fitting from under: late enough that its one clip stands on
#: the drum, soon enough that it bends no tighter than the hose's own (45 deg: 27.5 mm off the drum; 60: 20.4 mm
#: round), rad.
AIR_M, ROUND = 0.001, (math.radians(55.0),)


def _pipe(at, out, up, toward, stub=None):
    """`tees.pipe` off a face whose outward way is `out`, bent toward `toward` - a unit (a, b) along `up` and along
    -out x up -: (tris, the fitting's end, the hose's way out of it)."""
    import numpy as np
    out, up, at = np.asarray(out, float), np.asarray(up, float), np.asarray(at, float)
    turn = np.c_[-out, up, np.cross(-out, up)]          # tees' frame: its outlet leaves along its -x
    tris, end = tees.pipe(np.zeros(3), toward, stub)
    return [(tri @ turn.T + at, c) for tri, c in tris], turn @ end + at, turn @ np.array([0.0, toward[0], toward[1]])


def _fitting(at, leaves, face):
    """[(tri, colour)], its back: a span's fitting, its mouth at `at` - the hose leaving along `leaves` -, lying back
    from there, a stem from its middle back along z to the face at z = `face` it stands on."""
    import numpy as np
    at, leaves = np.asarray(at, float), np.asarray(leaves, float)
    back = at - tees.FITTING[0] * leaves
    mid = 0.5 * (at + back)
    return (km._rod(back, at, tees.FITTING[1], tees.PIPE_INK)
            + km._rod(mid, np.array([mid[0], mid[1], face]), tees.PIPE[1], tees.PIPE_INK)), back


def _elbow(at, leaves, way, face):
    """([(tri, colour)], its inlet's mouth): a span's still fitting as a black elbow - its fitting's mouth at `at`,
    the hose leaving along `leaves`; behind it the pipe bent round from the trunk's way in `way` (a unit), its
    tighter turn the pipe's (`tees`), a straight leg and the inlet's fitting the trunk's hose ends in -, a stem from
    the bend back along z to the face at z = `face` it stands on."""
    import numpy as np
    at, v, w, (rb, r) = np.asarray(at, float), np.asarray(leaves, float), np.asarray(way, float), tees.PIPE
    back = at - tees.FITTING[0] * v
    to_v, to_w = (w @ v) * v - w, v - (v @ w) * w
    centre = back + rb * to_v / np.linalg.norm(to_v)
    to_w = to_w / np.linalg.norm(to_w)
    bend = [centre + rb * (-to_w * math.cos(a) + w * math.sin(a)) for a in np.linspace(0.0, math.acos(w @ v), 9)]
    inlet = bend[0] - tees.PIPE_LEG * w
    mid = bend[4]
    tris = (km._rod(back, at, tees.FITTING[1], tees.PIPE_INK) + km._rod(inlet - tees.FITTING[0] * w, inlet,
                                                                         tees.FITTING[1], tees.PIPE_INK)
            + km._rod(inlet, bend[0], r, tees.PIPE_INK) + km._rod(mid, np.array([mid[0], mid[1], face]), r, tees.PIPE_INK))
    return tris + [t for p, q in zip(bend[:-1], bend[1:]) for t in km._rod(p, q, r, tees.PIPE_INK)], \
        inlet - tees.FITTING[0] * w


def _ends():
    """{'still', 'turning': (mouth, leaves)}: the span's fittings in the sketch's frame (`shoulder_rolled.span`)."""
    import numpy as np
    back, span = np.array(sr.BACK), sr.span()
    return {side: (back @ np.array(span[key][1]), back @ np.array(span[key][2]))
            for side, key in (('still', 'from'), ('turning', 'to'))}


@functools.lru_cache(maxsize=None)
def laid():
    """{'grommet', 'board', 'still', 'lever', 'saddle': tris of each fitting and pipe, 'up', 'down': the two runs'
    lines (k, 3)}: the board's run up to the span and the arm's down from it, the sketch's frame."""
    import numpy as np
    from machine.body import mechatronics
    lay, pitch, (top, _bottom) = sr.laid(), sl.laid(), sr.board()
    b0, b1 = pitch['board']
    x, wall = 0.5 * (b0 + b1), -pitch['guard'][2]
    grommet = (km._rod((x, wall + 0.002, 0.0), (x, -top - 0.001, 0.0), tees.PIPE[1], tees.PIPE_INK)
               + km._rod((x, wall, 0.0), (x, wall - 0.0012, 0.0), tees.COLLAR_R, tees.PIPE_INK))
    ends, rise = _ends(), np.array([0.0, 1.0, 0.0])
    still, inlet = _elbow(*ends['still'], rise, lay['boss'][0])            # the trunk up into it from under
    r, mid = sr.stack()['board'][0] + sr.GUARD_M, -0.5 * sum(sr.board())
    turn = math.asin((tees.PIPE[0] + tees.PIPE_LEG) / -mid)          # its end's way round the pitch's axis
    board, start, way = _pipe((r, mid, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (math.sin(turn), math.cos(turn)))
    off = float(np.hypot(start[1], start[2]))                # the run on round at the pipe's end's reach
    knots = [(start, way)] + [(np.array([start[0], -off * math.cos(a), -off * math.sin(a)]),
                               np.array([0.0, math.sin(a), -math.cos(a)])) for a in ROUND]
    lever, back_t = _fitting(*ends['turning'], sr._lever_z()[0])
    p1, drop = lay['plate'][1], lay['drop']
    saddle, end, out = _pipe((0.0, -drop, -p1), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0))
    up = tees.run(knots + [(inlet, rise)])
    down = tees.laid(back_t, -ends['turning'][1], end, out)
    return {'grommet': grommet, 'board': board, 'still': still, 'lever': lever, 'saddle': saddle, 'up': up,
            'down': down}


@functools.lru_cache(maxsize=None)
def parts():
    """{'still': with the pitch - the grommet, the board's pipe, the span's still fitting, the board's run and its
    clips -, 'yoke': with the arm - the lever's fitting, the saddle's outlet, the arm's run and its clips}: a clip
    stands on what turns with its run, its fittings and pipes with it."""
    from tools.render import shoulder_mount as sm
    got = laid()
    pitch = ([t for name in ('housing', 'guard', 'lid') for t in sm.turning()[name]] + sr.parts()['still']
             + got['board'] + got['still'])
    yoke = sr.parts()['yoke'] + got['lever'] + got['saddle']
    return {'still': (got['grommet'] + got['board'] + got['still'] + tees.hose(got['up'])
                      + tees.clipped(got['up'], pitch)),
            'yoke': got['lever'] + got['saddle'] + tees.hose(got['down']) + tees.clipped(got['down'], yoke)}


@functools.lru_cache(maxsize=None)
def aired(step=5.0):
    """The least air, m, through the abduction's travel `step` deg at a time: the board's run and its fittings off
    the yoke, the arm's run and the span's hose; the arm's run and its fittings off what turns with the pitch."""
    import numpy as np
    from tools.render import shoulder_mount as sm
    pitch = km._points(sr.parts()['still'] + [t for name in ('housing', 'guard', 'lid', 'drive')
                                              for t in sm.turning()[name]], 0.002)
    still = km._points(parts()['still'], 0.002)
    yoke = km._points(parts()['yoke'] + sr.parts()['yoke'] + sr.parts()['ring'], 0.002)
    run = km._points(parts()['yoke'], 0.002)
    h, out = 0.006, 9.0
    for fixed, moving in ((still, yoke), (pitch, run)):
        keys = km._key(np.floor(fixed / h).astype(np.int64))
        order = np.argsort(keys)
        for roll in np.append(np.arange(sr.TRAVEL[0], sr.TRAVEL[1], step), sr.TRAVEL[1]):
            c, s = math.cos(math.radians(roll)), math.sin(math.radians(roll))
            turn = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
            out = min(out, float(km._least(keys[order], fixed[order], moving @ turn.T, h)))
    for roll in np.linspace(sr.TRAVEL[0], sr.TRAVEL[1], 5):
        span = km._points(sr.posed(float(roll))['span'], 0.002)
        keys = km._key(np.floor(span / h).astype(np.int64))
        order = np.argsort(keys)
        out = min(out, float(km._least(keys[order], span[order], km._points(tees.hose(laid()['up']), 0.002), h)))
    return out


def checked():
    """{'bend': (the least either run bends at, the hose's own), mm, 'free': (the longest stretch either lies free,
    `tees.unheld`), mm, 'air': the least either keeps off what turns by it through the travel, mm}."""
    from machine.body import mechatronics
    got, own = parts(), 1e3 * float(mechatronics.materials()['hoses']['bus'][4])
    bend = min(tees.tightest(got[side]) for side in ('still', 'yoke'))
    free = max(tees.loosest(got[side]) for side in ('still', 'yoke'))
    return {'bend': (round(1e3 * bend, 1), own), 'free': (round(1e3 * free, 1), round(1e3 * tees.unheld(), 1)),
            'air': round(1e3 * aired(), 2)}


def main(argv=None):
    from PIL import Image, ImageDraw
    from tools.render import shoulder_mount as sm
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    args = parser.parse_args(argv)
    views = [(0.0, 35.0, 18.0), (0.0, 120.0, 10.0), (sr.TRAVEL[1], 35.0, 18.0), (sr.TRAVEL[0], 120.0, 10.0)]
    sheet = Image.new('RGB', (760 * len(views), 800), (18, 18, 18))
    for i, (roll, yaw, tilt) in enumerate(views):
        tris = [t for part in sm.posed(0.0, roll).values() for t in part]
        sheet.paste(m.draw(tris, yaw=yaw, pitch=tilt, px=760, half=0.13, centre=(0.01, -0.03, -0.04)), (760 * i, 40))
    ImageDraw.Draw(sheet).text((12, 10), "the shoulder's bus: hanging, from ahead | from inside | raised %.0f | in %.0f"
                               % sr.TRAVEL[::-1], fill=(235, 235, 235))
    sheet.save(args.out)
    print(args.out, checked())
    return 0


if __name__ == '__main__':
    sys.exit(main())
