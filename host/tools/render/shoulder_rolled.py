"""The shoulder's abduction: its drive behind on a hood off the pitch's housing, the arm in a yoke.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/shoulder_rolled.py OUT.png

`shoulder_laid` lays the shoulder's pitch; the arm is raised aside about a second axis, ahead through the same pivot,
as the hip's roll crosses its pitch: the abduction, its drive the pitch's own kind (KIND). The drive stands behind the
shoulder, its gearbox toward the pivot, in a band on the back of a printed hood that arches over the pitch's housing,
its foot on the housing's band. The arm hangs in a yoke astride it: a carbon plate screwed to the drive's output ring
behind, one ahead on a steel pin that turns in a bush in the hood's front leg, both farther off the pivot than the
pitch's guard and the abduction's board reach - turning about the axis ahead they never meet them -, a printed saddle
across under them, the upper arm's tube's foot under that. The board lies flat under the pitch's housing, where the
yoke never comes. The bus crosses the abduction ahead of the hood's leg, a span from a fitting on the leg to one on a
lever on the pin's end (`span`, hung as the elbow's). A rib on the rear plate meets a horn on the band at either end
of the travel (TRAVEL), the drive's own torque at the margin borne there - pushed harder, the arm gives and comes to
lie on the body. Each size the least its check asks (`held`); every piece prints along an axis it is a prism of. The
frame is `shoulder_laid`'s: x along the pitch's axis toward the torso, y up, z behind, the pivot at 0; raised (`roll`
positive), the arm's foot swings out, toward -x.
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
from tools.render import knee_loads as kl  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import mounting as m  # noqa: E402
from tools.render import shoulder_laid as sl  # noqa: E402
from tools.render import tees  # noqa: E402

#: The abduction's drive's kind - the pitch's own, an MN6007 on B (`drives.along`) - and its travel between its stops,
#: deg off the arm's rest: from a little in past the body to past a T (the arm rests `shoulder_laid.pivot` out).
KIND, TRAVEL = 'shoulder', (-20.0, 95.0)
#: The air what turns about the axis ahead keeps off what does not; a carbon sheet (`trunk_yokes`); the plates' hub
#: round the axis and their width at the saddle; the boss along the axis and its wall round the bush; the bush's
#: wall; the rib's and the horns' width and how far along the axis they lap; the hood's foot along the axis either
#: way off the pivot; the span's fittings off the axis; m.
AIR_M, SHEET_M, HUB_M, WIDE_M, BOSS_M, BOSS_WALL, BUSH_M = 0.003, 0.004, 0.030, 0.044, 0.012, 0.004, 0.001
HORN_M, LAP_M, FOOT_M, REACH_M, PAST_M = 0.010, 0.006, 0.030, 0.035, 0.012
#: The board's air under the pitch's guard, and round it the guard's wall and its lid's, m.
BOARD_AIR, GUARD_M = 0.004, 0.003


def stack():
    """{'board', 'motor', 'gear': (radius, length)} of the abduction's drive, m (`shoulder_laid.stack`'s kind)."""
    return sl.stack()


def gap():
    """How far off the pivot along the axis ahead the plates' inner faces stand, m: past the pitch's housing and
    guard and the board's guard, and the air."""
    lay = sl.laid()
    return max(lay['band'][2], lay['guard'][2], stack()['board'][0] + GUARD_M) + AIR_M


def board():
    """(Top, bottom) of the abduction's board, m under the pivot: clear under the pitch's guard."""
    top = sl.laid()['guard'][2] + BOARD_AIR
    return top, top + stack()['board'][1]


def _drop(saddle_mm):
    """How far under the pivot the saddle's middle hangs, m: its top the air past the board's guard, turned any way."""
    r, (_top, bottom) = stack()['board'][0] + GUARD_M, board()
    return math.hypot(r, bottom + GUARD_M) + AIR_M + 0.5e-3 * saddle_mm


@functools.lru_cache(maxsize=None)
def carried():
    """(The loads at the margin - `knee_loads.loads`, N mm and N -, the drive's own torque at the margin, N mm, each
    support's push across the axis, N, its twist's share, N): the yoke's two supports take the arm's twist and swing
    as couples across their span, and half its pull each."""
    from machine.body import drives, structural
    ld, span = kl.loads('shoulder'), 1e3 * 2.0 * (gap() + 0.5 * SHEET_M)
    twist, swing = ld['torsion'] / span, ld['swing'] / span
    return (ld, structural.MARGIN * 1e3 * drives.peak('left_' + KIND), math.hypot(twist, swing + 0.5 * ld['axial']),
            twist)


def _stress(name, sz):
    """A member's stress over its allowable at the margin, sizes `sz` (mm): the rear plate at the saddle - the twist's
    lever and the drive's torque in its plane -, the pin out of its bush to the plate, the saddle across the plates -
    the twist aside, the swing and the pull up -, the hood's arch twisted by the drive's torque between its back and
    its foot, and that foot bent on the housing's band."""
    from machine.body import structural
    ld, own, push, twist = carried()
    t, wide, h = 1e3 * SHEET_M * sz['sheets'], 1e3 * WIDE_M, sz['saddle']
    carbon, steel, allow = structural.CARBON[2], structural.STEEL[2], structural.PAHT_CF[2]
    if name == 'sheets':
        return 6.0 * (twist * 1e3 * _drop(h) + own) / (t * wide * wide) / carbon
    if name == 'pin':
        lever = 0.5 * t + 1e3 * AIR_M + 0.5e3 * BOSS_M
        return 32.0 * push * lever / (math.pi * (2.0 * sz['pin']) ** 3) / steel
    if name == 'saddle':
        span = 2e3 * gap()
        side = 6.0 * 0.5 * ld['torsion'] / (h * wide * wide)
        up = 6.0 * (0.5 * ld['swing'] + ld['axial'] * span / 4.0) / (wide * h * h)
        return (side + up) / allow
    x0, x1 = _hood_x()
    if name == 'hood':                                     # an open section's twist, at half the allowable in shear
        return 3.0 * own / (1e3 * (x1 - x0 + 2.0 * PAST_M) * sz['hood'] ** 2) / (0.5 * allow)
    return 6.0 * own / (2e3 * FOOT_M * (1e3 * (x1 - x0) - 4.0) ** 2) / allow          # 'foot'


def _hood_x():
    """The hood's foot's width along the pitch's axis, (from, to), m: on the housing's band, as far as the guard
    lets; the arch reaches PAST_M past it either way, over the guard."""
    lay = sl.laid()
    return lay['guard'][1] + 0.002, lay['band'][1]


@functools.lru_cache(maxsize=None)
def sizes():
    """{'sheets': the plates' carbon sheets, 'pin': the pin's radius, 'saddle': its height, 'hood': its wall, mm}:
    each the least its check asks (`_stress`)."""
    out = {'sheets': 1, 'pin': 3.0, 'saddle': 10.0, 'hood': kl.WALL}
    for name, step, top in (('sheets', 1, 6), ('pin', 0.5, 12.0), ('saddle', 1.0, 40.0), ('hood', 0.5, 25.0)):
        while _stress(name, out) > 1.0 and out[name] < top:
            out[name] += step
    return out


@functools.lru_cache(maxsize=None)
def laid():
    """The abduction's layout, m: {'gap', 'plate': (from, to) along z behind, 'ring', 'house', 'motor': the drive's
    parts behind it, 'boss': (from, to) ahead, 'pin': its radius, 'drop': the saddle's middle under the pivot,
    'saddle': its height, 'top', 'down': the arm's seat's face and its foot's, 'board': (top, bottom), 'inside',
    'roof': the hood's arch, 'band': the drive's band's outside, 'rib': its inside off the axis}."""
    sz, st = sizes(), stack()
    g, t = gap(), SHEET_M * sz['sheets']
    out, at = {'gap': g, 'plate': (g, g + t)}, g + t
    for name, long_ in (('ring', (1.0 - em.OUTPUT_SHARE) * st['gear'][1]), ('house', em.OUTPUT_SHARE * st['gear'][1]),
                        ('motor', st['motor'][1])):
        out[name], at = (at, at + long_), at + long_
    near, saddle = g + t + AIR_M, 1e-3 * sz['saddle']
    drop = _drop(sz['saddle'])
    out.update(boss=(-(near + BOSS_M), -near), pin=1e-3 * sz['pin'], drop=drop, saddle=saddle,
               top=drop + 0.5 * saddle, down=drop + 0.5 * saddle + em.SEAT_M, board=board(), inside=g,
               roof=g + 1e-3 * sz['hood'], band=st['motor'][0] + 1e-3 * kl.WALL + 0.002, rib=st['gear'][0] + AIR_M)
    return out


def held():
    """{name: margin} of what holds the abduction: the plates, the pin, the saddle, the hood's arch and its foot, the
    plate's bore on its bush, the output ring's screws slipping, a horn at a stop, and the span's hose's least bend."""
    from machine.body import mechatronics, structural
    sz, lay, (_ld, own, push, _twist) = sizes(), laid(), carried()
    out = {name: 1.0 / _stress(name, sz) for name in ('sheets', 'pin', 'saddle', 'hood', 'foot')}
    out['bore'] = structural.CARBON[2] * 2e3 * (lay['pin'] + BUSH_M) * 1e3 * SHEET_M * sz['sheets'] / push
    count, circle = kl.HUB
    out['ring screws'] = kl.FRICTION * count * kl.SCREWS['M5'][0] / (own / circle + push)
    out['horn'] = structural.PAHT_CF[2] * 1e6 * HORN_M * LAP_M / (own / (1e3 * (lay['rib'] + 0.5 * HORN_M)))
    out['span bend'] = span_least() / float(mechatronics.materials()['hoses']['bus'][4])
    return out


def _along(tris, axis):
    """`tris` drawn along x laid along `axis` ('y' or 'z')."""
    import numpy as np
    turn = {'y': np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
            'z': np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])}[axis]
    return [(tri @ turn.T, c) for tri, c in tris]


def _box(x, y, z, colour, r=0.002):
    """A rounded box over the spans x, y, z, m."""
    return em._rounded([(x[0], y[0]), (x[1], y[0]), (x[1], y[1]), (x[0], y[1])], z[0], z[1], colour, axis=2, r=r)


def _plate(z0, z1, rib=False):
    """A yoke's plate, carbon, from z0 to z1: its hub round the axis tapering to the saddle's width under it; behind,
    the stop's rib on its back outside the output ring."""
    import numpy as np
    from scipy.spatial import ConvexHull
    lay = laid()
    t = np.linspace(0.0, 2.0 * math.pi, 49)[:-1]
    low = -lay['drop'] - 0.5 * lay['saddle']
    pts = np.vstack([np.c_[HUB_M * np.cos(t), HUB_M * np.sin(t)], [[-0.5 * WIDE_M, low], [0.5 * WIDE_M, low]]])
    out = em._rounded(pts[ConvexHull(pts).vertices], z0, z1, em.CARBON, axis=2, r=0.001)
    if rib:
        out += _box((-0.5 * HORN_M, 0.5 * HORN_M), (-lay['rib'] - HORN_M, -lay['rib']), (z1, z1 + LAP_M), em.FRAME, 0.001)
    return out


def _hood():
    """The printed hood on the pitch's housing: a foot on the band, an arch over the guard, its front leg down round
    the pin's bush, its back down to the band round the drive; a prism along x, printed on its side."""
    lay = laid()
    x0, x1 = _hood_x()
    (b0, b1), low = lay['boss'], -(lay['pin'] + BUSH_M + BOSS_WALL)
    outline = [(low, b0), (lay['roof'], b0), (lay['roof'], lay['motor'][1]), (lay['band'] - 0.004, lay['motor'][1]),
               (lay['band'] - 0.004, lay['house'][0]), (lay['inside'], lay['house'][0]), (lay['inside'], b1),
               (low, b1)]
    shell = foot._slab(cl.filleted(outline, r=0.002), x0 - PAST_M, x1 + PAST_M, em.FRAME)
    return shell + _box((x0 + 0.002, x1 - 0.002), (sl.laid()['band'][2] - 0.001, lay['inside'] + 0.001),
                        (-FOOT_M, FOOT_M), em.FRAME)


@functools.lru_cache(maxsize=None)
def parts():
    """{name: tris} of the abduction, the arm hanging, the sketch's frame: 'still' (with the pitch) - the drive, its
    band and horns, the hood and the pin's bush in it, the board, its guard and its post -, 'yoke' - the plates,
    the pin and its lever, the saddle, the arm's seat and its tube's top - and 'ring', the drive's output ring the
    rear plate is screwed to, running on the drive's housing."""
    lay, st = laid(), stack()
    ring, can = st['gear'][0], st['motor'][0]
    (h0, h1), (m0, m1), (r0, r1) = lay['house'], lay['motor'], lay['ring']
    drive = em._drum(ring, h0, h1, em.GEAR) + em._drum(can, m0, m1, em.SERVO)
    drive += cl.turned([(lay['house'][0] + 0.001, ring, lay['band']), (lay['motor'][1] - 0.001, can, lay['band'])], 0.0)
    horns = []
    for deg, way in ((TRAVEL[0], -1.0), (TRAVEL[1], 1.0)):  # past the rib at each end, on the band's front
        b = math.radians(deg) + way * HORN_M / (lay['rib'] + 0.5 * HORN_M)
        r0 = lay['rib'] + 0.5 * HORN_M
        at = (-r0 * math.sin(b), -r0 * math.cos(b))
        horns += km._rod((at[0], at[1], lay['plate'][1] + AIR_M), (at[0], at[1], lay['house'][0] + 0.002), 0.5 * HORN_M,
                         em.FRAME)
    top, bottom = lay['board']
    r = st['board'][0]
    board = (em._drum(r, -bottom, -top, em.BOARD) + em._drum(r + GUARD_M, -bottom - GUARD_M, -bottom, em.FRAME)
             + cl.turned([(-bottom - GUARD_M, r + 0.0005, r + GUARD_M), (-top + 0.001, r + 0.0005, r + GUARD_M)], None))
    x0, x1 = _hood_x()
    post = _box((x0 + 0.002, x1 - 0.002), (-top, -sl.laid()['band'][2] + 0.001), (-0.008, 0.008), em.FRAME)
    bush = _along(em._drum(lay['pin'] + BUSH_M, lay['boss'][0], lay['boss'][1], em.PLATE), 'z')
    from tools.render import shoulder_shaped as ss       # the hood as its loads shaped it, once it carries them
    hood = ss.drawn()
    still = _along(drive, 'z') + horns + (_hood() if hood is None else hood) + bush + _along(board, 'y') + post
    p0, p1 = lay['plate']
    low, high = -lay['drop'] - 0.5 * lay['saddle'], -lay['drop'] + 0.5 * lay['saddle']
    lever_z = _lever_z()
    pin = _along(em._drum(lay['pin'], lever_z[0], -p0, em.BOLT)
                 + em._drum(lay['pin'] + BOSS_WALL, lever_z[0], lever_z[1], em.FRAME), 'z')
    lever = _box((-0.006, 0.006), (-REACH_M - 0.006, 0.0), lever_z, em.FRAME)
    tube = sl.laid()['tube']
    e = tube - 0.0012 - em.BORE_R - 0.003
    dress, limb = em._limb(-lay['down'], -1.0, tube, -e, 'torx', False)
    yoke = (_plate(p0, p1, rib=True) + _plate(-p1, -p0)
            + _box((-0.5 * WIDE_M, 0.5 * WIDE_M), (low, high), (-p1, p1), em.FRAME) + pin + lever
            + em._rounded(em._oval(tube), -lay['top'], -lay['down'], em.PLATE, r=0.002) + dress + limb)
    return {'still': still, 'yoke': yoke, 'ring': _along(em._drum(ring, r0, r1, em.OUTPUT), 'z')}


def _lever_z():
    """(From, to) along z of the lever on the pin's end, ahead of the hood's leg, m."""
    b0 = laid()['boss'][0]
    return b0 - AIR_M - 0.004, b0 - AIR_M


def rolled(tris, roll):
    """`tris` turned `roll` deg about the axis ahead: raised, out toward -x."""
    import numpy as np
    c, s = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    rz = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
    return [(tri @ rz.T, colour) for tri, colour in tris]


#: The turn from `hung`'s frame - the axis ahead its x - to the sketch's: (a, b, c) -> (-c, b, a).
BACK = ((0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0))


@functools.lru_cache(maxsize=None)
def span():
    """The bus's hose across the abduction, the elbow's way (`hung`): from a fitting on the hood's front leg to one on
    the lever on the pin's end - straight down with the arm hanging -, each aimed at the axis, facing each other
    across it at the travel's middle; in `hung`'s frame, the axis ahead its x (BACK), the arm's raise its turn's
    negative."""
    lay, (l0, l1) = laid(), _lever_z()
    x, near = l0 - tees.FITTING[0], lay['pin'] + BOSS_WALL + 0.001
    a = math.radians(0.5 * (TRAVEL[0] + TRAVEL[1]))       # the lever's way at the travel's middle, off straight down
    return {'name': 'abduction', 'kind': 'bus', 'travel': (-TRAVEL[1], -TRAVEL[0]),
            'from': ('still', (x, REACH_M * math.cos(a), -REACH_M * math.sin(a)), (0.0, -math.cos(a), math.sin(a))),
            'to': ('turning', (x, -REACH_M, 0.0), (0.0, 1.0, 0.0)),
            'solids': [('still', (0.5 * sum(lay['boss']), 0.0, 0.0), (1.0, 0.0, 0.0), near, 0.5 * BOSS_M),
                       ('turning', (0.5 * (l0 + l1), 0.0, 0.0), (1.0, 0.0, 0.0), near, 0.5 * (l1 - l0))]}


def span_least():
    """The least radius the span's hose bends at through the travel, m (`hung.least`)."""
    from tools.render import hung
    return hung.least(span())


def posed(roll=0.0):
    """{name: tris}: the abduction with the arm raised `roll` deg - the yoke turned, the span's hose as it lies."""
    import numpy as np
    from tools.render import hung
    got = parts()
    line = hung.path(span(), -roll) @ np.array(BACK).T
    return {'still': got['still'], 'yoke': rolled(got['yoke'] + got['ring'], roll), 'span': tees.hose(line)}


@functools.lru_cache(maxsize=None)
def aired(step=5.0, ease=3.0):
    """The least air between the yoke and what stays with the pitch - the abduction's still parts and the pitch's
    turning ones - through the travel `ease` deg short of each stop - at a stop rib and horn meet -, m; the pin in its
    bush apart: it runs there."""
    import numpy as np
    from tools.render import shoulder_mount as sm
    a = km._points(parts()['still'] + [t for name in ('housing', 'guard', 'lid', 'drive') for t in sm.turning()[name]],
                   0.002)
    h = 0.006
    keys = km._key(np.floor(a / h).astype(np.int64))
    order = np.argsort(keys)
    near = laid()['pin'] + BOSS_WALL + 0.001
    yoke = km._points([t for t in parts()['yoke'] if t[0][:, 1].min() > -laid()['down']
                       and np.hypot(t[0][:, 0], t[0][:, 1]).max() > near], 0.002)
    out = 9.0
    for roll in np.arange(TRAVEL[0] + ease, TRAVEL[1] - ease + 0.1, step):
        c, s = math.cos(math.radians(roll)), math.sin(math.radians(roll))
        out = min(out, float(km._least(keys[order], a[order], yoke @ np.array([[c, s, 0.0], [-s, c, 0.0],
                                                                                [0.0, 0.0, 1.0]]).T, h)))
    return out


def checked():
    """{'held': the least of `held`'s margins, 'air': the yoke off what stays through the travel, mm, 'travel': (from,
    to) deg, 'reaches': the arm off plumb at its highest, deg, 'bend': (the span's least, the hose's own), mm,
    'length': the span's hose, mm}: what the suite holds."""
    from machine.body import mechatronics
    from tools.render import hung
    return {'held': round(min(held().values()), 3), 'air': round(1e3 * aired(), 2), 'travel': TRAVEL,
            'reaches': TRAVEL[1] + sl.pivot()[1],
            'bend': (round(1e3 * span_least(), 1), 1e3 * float(mechatronics.materials()['hoses']['bus'][4])),
            'length': round(1e3 * hung.long(span()), 1)}


def main(argv=None):
    from PIL import Image, ImageDraw
    from tools.render import shoulder_mount as sm
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    args = parser.parse_args(argv)
    views = [(0.0, 35.0, 18.0), (0.0, -150.0, 14.0), (TRAVEL[1], 35.0, 18.0), (TRAVEL[0], -60.0, 10.0)]
    sheet = Image.new('RGB', (760 * len(views), 800), (18, 18, 18))
    for i, (roll, yaw, tilt) in enumerate(views):
        tris = [t for part in sm.posed(0.0, roll).values() for t in part]
        sheet.paste(m.draw(tris, yaw=yaw, pitch=tilt, px=760, half=0.14, centre=(0.0, -0.04, 0.0)), (760 * i, 40))
    ImageDraw.Draw(sheet).text((12, 10), 'abduction: hanging, from ahead | from behind | raised %.0f | in %.0f deg'
                               % (TRAVEL[1], TRAVEL[0]), fill=(235, 235, 235))
    sheet.save(args.out)
    for name, margin in held().items():
        print('  %-12s %.2f' % (name, margin))
    print(args.out, sizes(), checked())
    return 0


if __name__ == '__main__':
    sys.exit(main())
