"""The shoulder's mount, sketched: boardless MN6007 + B pitch drum on the abduction gimbal.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/shoulder_mount.py OUT.png

`shoulder_laid` sizes Stage 2 (the compact boardless MN6007 + B coaxial pitch drive and concentric bus coil);
`shoulder_rolled` sizes Stage 1 (the pectoral drive, pushrod and z-axis crossbar gimbal). Stage 2's hollow carbon arbor
is carried by Stage 1's gimbal bridge so arm pitch (`turn`) spins only the 78 mm housing, coil drum, motor endcap and
upper-arm carbon tube, never twisting the pushrod. Frame: `shoulder_laid`'s.
"""
import argparse
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import clamped as cl  # noqa: E402
from tools.render import covered as cv  # noqa: E402
from tools.render import elbow_mount as em  # noqa: E402
from tools.render import knee_loads as kl  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import mounting as m  # noqa: E402
from tools.render import shoulder_laid as sl  # noqa: E402
from tools.render import tees  # noqa: E402

#: Band pinch relief, gimbal flange thickness, clearance past coil fitting, M4 nut (radius, height), m.
RELIEF_M, SIDE_M, PAST_M, NUT = 0.0012, 0.008, 0.004, (0.007 / 2.0, 0.0032)
HUB_M, ARM = 0.009, 0.45
STILL = ('arbor', 'crossbar foot', 'studs', 'hub plate', 'block')
TURNING = ('housing', 'arm saddle', 'guard', 'lid')


def _leant(tris):
    """`tris` drawn along the torso's crossbar from `cant`, in `shoulder_laid`'s frame."""
    return [(sl.canted(tri), colour) for tri, colour in tris]


def _arc(inner, outer, x0, x1, low, high, colour, k=12):
    """Rounded ring sector around x between radii and bearings (rad, +y toward +z), from x0 to x1."""
    import numpy as np
    from tools.render import foot
    turn = np.linspace(low, high, max(k, int(math.degrees(abs(high - low)) / 5.0) + 2))
    outline = np.r_[np.c_[outer * np.cos(turn), outer * np.sin(turn)],
                    np.c_[inner * np.cos(turn[::-1]), inner * np.sin(turn[::-1])]]
    return foot._slab(cl.filleted(outline, r=0.0015), x0, x1, colour)


def _banjo(at, along, stem=(0.0, 0.0, 0.0), out=0.0):
    """Hose fitting lying back along `along` from `at`, with a stem to `out` or `stem`."""
    import numpy as np
    at, along = np.asarray(at, float), np.asarray(along, float)
    back = at - tees.FITTING[0] * along / np.linalg.norm(along)
    off = float(np.hypot(back[1], back[2]))
    reach = np.array([0.0, back[1], back[2]]) * (out - off) / off if out else np.asarray(stem, float)
    return (km._rod(back, at, tees.FITTING[1], tees.PIPE_INK)
            + km._rod(back, back + reach, tees.PIPE[1], tees.PIPE_INK))


def _horn():
    """(from, to), rad: bearings of the pitch stop horns on Stage 1's flange."""
    lay, (lo, hi), rib = sl.laid(), sl.travel(), math.radians(sl.drummed())
    half = 0.5 * sl.RIB_M / lay['meet']
    return rib + math.radians(hi) + half, rib + math.radians(lo) + 2.0 * math.pi - half


@functools.lru_cache(maxsize=None)
def still():
    """{name: tris} of Stage 2's stator on Stage 1's gimbal, plus `'crossbar'` fixed on the torso."""
    import numpy as np
    from tools.render import shoulder_rolled as sr
    lay, sz = sl.laid(), sl.sizes()
    bar, arbor, disc, play = lay['bar'], lay['arbor'], lay['disc'], kl.BOND * 1e-3
    (f0, f1), (p0, p1), ring = lay['foot'], lay['plate'], sl.stack()['gear'][0]
    side = sl.FOOT_X + lay['coil'][1] + sl.MOUTH_M
    collar = [(f1 - 0.002, arbor + play, arbor + play + 1e-3 * sz['collar']),
              (f1 + kl.COLLAR_SHARE * 2.0 * arbor, arbor + play, arbor + play + 1e-3 * sz['collar'])]
    studs = []
    for i in range(sl.STUDS):
        a = (i + 0.5) * 2.0 * math.pi / sl.STUDS
        at = (lay['studs'] * math.cos(a), lay['studs'] * math.sin(a))
        studs += (em._drum(sl.WASHER_M, f1, f1 + 0.001, em.BOLT, at)
                  + em._drum(NUT[0], f1, f1 + 0.001 + NUT[1], em.BOLT, at))
    low, high = _horn()
    far = lay['meet'] + 0.5 * sl.RIB_UP
    start = sl.coil(0.0, 9)[0]
    round_, at = float(np.hypot(*start[1:])) + tees.FITTING[1] + PAST_M, math.atan2(start[2], start[1])
    hub = arbor + HUB_M
    block = (em._drum(hub, side, side + SIDE_M, em.FRAME)
             + _arc(hub - 0.002, round_, side, side + SIDE_M, at - ARM, at + ARM, em.FRAME))
    wide = min(sl.HORN_M / far, 0.5 * (high - low))
    for a, b in ((low, low + wide), (high - wide, high)):
        block += (_arc(hub - 0.002, far, side, side + SIDE_M, a, b, em.FRAME)
                  + _arc(far - sl.RIB_UP + sl.AIR_M, far, side - sl.MOUTH_M - sl.HOLD_M, side, a, b, em.FRAME))
    c0_bar = max(sr.laid()['root'] - lay['cant'] + 0.005, 0.025)
    c1_bar = (sl.pivot()[0] - sl.BLOCK_M) / math.cos(math.radians(sl.pivot()[1])) - lay['cant']
    return {'crossbar': _leant(em._drum(bar, c0_bar, c1_bar, em.CARBON)),
            'arbor': em._drum(arbor, f0 + 0.002, side + SIDE_M + 0.006, em.CARBON),
            'crossbar foot': em._drum(disc, f0, f1, em.FRAME) + cl.turned(collar, None),
            'studs': studs, 'hub plate': em._drum(disc, p0, p1, em.CARBON),
            'output ring': em._drum(ring, lay['ring'][0], lay['ring'][1], em.OUTPUT), 'block': block}


def _saddle(lay):
    """Compact PAHT-CF upper-arm saddle under Stage 2's slit band, grown cleanly along +x without overhang."""
    tube, out = lay['tube'], lay['band'][2]
    x_half, z_seat = tube + em.OVAL_PAST[1] + 0.002, tube + em.OVAL_PAST[0] + 0.002
    y_hi, y_lo = -(out - 0.002), -lay['top']
    prof = cl.filleted([(y_hi, -z_seat - 0.004), (y_hi, z_seat + 0.004), (y_lo, z_seat), (y_lo, -z_seat)], r=0.0025)
    return [(tri[:, (2, 0, 1)], col) for tri, col in em._rounded(prof, -x_half, x_half, em.FRAME, axis=2, r=0.002)]


def _housing(lay, can, ring):
    """Printed PAHT-CF housing turning with the arm: slit band, skirt, rims' bore, coil drum, rib and arm saddle."""
    import numpy as np
    from tools.render import foot
    band0, band1, out = lay['band']
    plate0, f1, rims = lay['plate'][0], lay['f1'], lay['disc'] + sl.PLAY_M
    bore, m1, skirt = lay['bore'], lay['motor'][1], lay['skirt']
    cone, mouth = f1 + max(bore - rims, 0.003), f1 + lay['coil'][1]
    shell = cl.turned([(band0, can, out), (m1, can, out), (m1 + max(can - ring, 0.0), ring, out),
                       (band1, ring, out)], 0.0)
    shell += cl.slotted(skirt, out, band1 - 0.0005, plate0 - max(rims - skirt, 0.0) + 0.0005,
                        [(0.0, band1 - 0.0005, band1 + RELIEF_M, 0.020)], em.FRAME, k=96)
    shell += cl.turned([(band1, ring, out), (band1 + max(skirt - ring, 0.0), skirt, out)], 0.0, gap=0.020 / skirt)
    shell += cl.turned([(plate0 - max(rims - skirt, 0.0), skirt, out), (plate0, rims, out),
                        (f1, rims, out), (cone, bore, bore + sl.DRUM_M)], None)
    shell += cl.honeycomb(bore, bore + sl.DRUM_M, cone - 0.0005, mouth)
    low, r0 = bore + sl.DRUM_M, cone - sl.RIB_UP - 0.003
    foot_rib = np.array([(r0, -(out - 0.002)), (cone, -(low + sl.RIB_UP)), (mouth, -(low + sl.RIB_UP)),
                         (mouth, -(bore + 0.0005)), (cone, -(bore + 0.0005)), (r0, -(rims + 0.001))])
    rib_tris = [(tri[:, (1, 2, 0)], col) for tri, col in foot._slab(foot_rib, -0.5 * sl.RIB_M, 0.5 * sl.RIB_M, em.FRAME)]
    return shell + km._turn(rib_tris, sl.drummed() - 180.0)


@functools.lru_cache(maxsize=None)
def turning():
    """{name: tris} of Stage 2's rotor at `turn = 0, roll = 0`: boardless MN6007 + B, slim endcap, housing & arm foot."""
    lay, st = sl.laid(), sl.stack()
    can, ring, tube = st['motor'][0], st['gear'][0], lay['tube']
    (m0, m1), (h0, _h1), out = lay['motor'], lay['house'], lay['band'][2]
    drive = em._drum(can, m0, m1, em.SERVO) + em._drum(ring, h0, lay['ring'][1], em.GEAR)
    g0, g1, g_out = lay['guard']
    guard = [(t, cv.BLACK) for t, _c in cl.turned([(g0 + cv.WALL_M, can + 0.0005, g_out),
                                                   (g1, can + 0.0005, g_out)], None)]
    lid = em._drum(g_out, g0, g0 + cv.WALL_M, cv.BLACK)
    e = tube - 0.0012 - em.BORE_R - 0.003
    dress, limb = em._limb(-lay['down'], -1.0, tube, -e, 'torx', False)
    seat = em._rounded(em._oval(tube), -lay['top'], -lay['down'], em.PLATE, r=0.002) + dress + limb
    line = sl.coil(0.0, 400)
    return {'drive': drive, 'leads': [], 'guard': guard, 'lid': lid, 'housing': _housing(lay, can, ring),
            'arm saddle': _saddle(lay), 'arm seat': seat,
            'coil stem': _banjo(line[-1], line[-1] - line[-2], out=lay['bore'] + 0.001)}


def posed(turn=0.0, roll=0.0, chest=False):
    """{name: tris}: Stage-1 clevis & gimbal (`roll`) carrying Stage-2 pitch drum (`turn`), coil and bus hoses."""
    from tools.render import shoulder_hosed as sh, shoulder_rolled as sr
    st_still, st_turn = still(), turning()
    line = sl.coil(turn, 400)
    coil_tris = tees.hose(line) + _banjo(line[0], line[0] - line[1], (sl.MOUTH_M + tees.HOSE_R + 0.001, 0.0, 0.0))
    raised, bus = sr.posed(roll, chest_parts=chest), sh.parts()
    out = {'crossbar': st_still['crossbar'], 'clevis': raised['still'], 'span': raised['span']}
    if 'pectoral' in raised:
        out['pectoral'] = raised['pectoral']
    # Stage 1 parts turn with `roll` only:
    out['yoke'] = raised['yoke']
    for k in ('arbor', 'crossbar foot', 'studs', 'hub plate', 'output ring', 'block'):
        out[k] = sr.rolled(st_still[k], roll)
    out['coil'] = sr.rolled(coil_tris, roll)
    out['bus'] = sr.rolled(bus['still'], roll)
    # Stage 2 parts turn first by `turn` about x, then by `roll` about z:
    for k, tris in st_turn.items():
        out[k] = sr.rolled(km._turn(tris, turn), roll)
    out['bus on the arm'] = sr.rolled(km._turn(bus['yoke'], turn), roll)
    return out


@functools.lru_cache(maxsize=None)
def aired(step=10.0, ease=3.0):
    """Least air between Stage 2's turning housing/cap and Stage 1's gimbal/stator across pitch travel, m."""
    import numpy as np
    from tools.render import shoulder_rolled as sr
    lo, hi = sl.travel()
    h = 0.006
    a = km._points([t for name in STILL if name != 'hub plate' for t in still()[name]
                    if name not in ('crossbar foot', 'arbor') or t[0][:, 0].min() > sl.laid()['foot'][1] - 1e-9]
                   + sr.parts()['yoke'])
    pts = km._points([t for name in TURNING for t in turning()[name]])
    keys = km._key(np.floor(a / h).astype(np.int64))
    order = np.argsort(keys)
    return min(float(km._least(keys[order], a[order], pts @ km._rx(float(deg)).T, h))
               for deg in np.arange(lo + ease, hi - ease + 0.1, step))


def checked():
    """{'held', 'air', 'crossing', 'coil', 'bend', 'travel', 'asked'}: what `test_views_built` holds."""
    import numpy as np
    from machine.body import mechatronics
    from tools.render import clips, shoulder_rolled as sr
    lo, hi = sl.travel()
    st_fixed = [still()[b] for b in STILL] + [sr.parts()['yoke']]
    crossing = sum(clips.crossed(turning()[a], b_tris) for a in TURNING for b_tris in st_fixed)
    coil, bend = 0, []
    for turn in np.linspace(lo, hi, 5):
        parts = posed(float(turn), 0.0)
        hose = tees.hose(sl.coil(float(turn), 400))
        coil += sum(clips.crossed(hose, tris) for name, tris in parts.items()
                    if name not in ('coil', 'coil stem', 'bus'))
        bend.append(tees.tightest(hose))
    return {'held': round(min(sl.held().values()), 3), 'air': round(1e3 * aired(), 2), 'crossing': crossing,
            'coil': coil, 'bend': (round(1e3 * min(bend), 1), 1e3 * float(mechatronics.materials()['hoses']['bus'][4])),
            'travel': tuple(round(v, 1) for v in (lo, hi)), 'asked': (sl.BACK, sl.SWUNG)}


def main(argv=None):
    from PIL import Image, ImageDraw
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    args = parser.parse_args(argv)
    lo, hi = sl.travel()
    views = [(0.0, 35.0, 18.0), (0.0, -150.0, 14.0), (lo, 35.0, 18.0), (hi, 35.0, 18.0)]
    sheet = Image.new('RGB', (760 * len(views), 800), (18, 18, 18))
    for i, (turn, yaw, pitch) in enumerate(views):
        tris = [t for part in posed(turn, 0.0, chest=True).values() for t in part]
        sheet.paste(m.draw(tris, yaw=yaw, pitch=pitch, px=760, half=0.18, centre=(0.04, -0.04, -0.04)), (760 * i, 40))
    ImageDraw.Draw(sheet).text((12, 10), 'shoulder pitch (boardless): rest ahead | behind | stops %.0f and %.0f deg'
                               % (lo, hi), fill=(235, 235, 235))
    sheet.save(args.out)
    print(args.out, 'travel %.0f .. %.0f deg' % (lo, hi))
    return 0


if __name__ == '__main__':
    sys.exit(main())
