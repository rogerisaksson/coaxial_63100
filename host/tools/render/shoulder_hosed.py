"""The bus through the shoulder: Stage-1 span to Stage-2 coil and down into the upper-arm tube.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/shoulder_hosed.py OUT.png

Across Stage 1's rear z-axis clevis pin (`shoulder_rolled.span`), the bus hangs free from the torso's crossbar fork
onto a black elbow on the gimbal's rear lever. From that elbow (`still`), a clipped hose runs gently inboard along the
rear gimbal cheek into Stage 2's concentric bus coil (`shoulder_laid.coil`). Leaving the coil's drum stem (`yoke`), a
second clipped hose curves outside Stage 2's turning housing into a black pipe on the upper-arm saddle and down the
carbon tube's bore to the elbow (`elbow_mount`). Frame: `shoulder_laid`'s.
"""
import argparse
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import elbow_mount as em  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import mounting as m  # noqa: E402
from tools.render import shoulder_laid as sl  # noqa: E402
from tools.render import shoulder_rolled as sr  # noqa: E402
from tools.render import tees  # noqa: E402

AIR_M = 0.001


def _pipe(at, out, up, toward, stub=None):
    """`tees.pipe` off a face whose outward normal is `out`, bent toward `toward`."""
    import numpy as np
    out, up, at = np.asarray(out, float), np.asarray(up, float), np.asarray(at, float)
    turn = np.c_[-out, up, np.cross(-out, up)]
    tris, end = tees.pipe(np.zeros(3), toward, stub)
    return [(tri @ turn.T + at, c) for tri, c in tris], turn @ end + at, turn @ np.array([0.0, toward[0], toward[1]])


def _fitting(at, leaves, face_z):
    """Span fitting with mouth at `at` (hose leaving along `leaves`) and stem along z to `face_z`."""
    import numpy as np
    at, leaves = np.asarray(at, float), np.asarray(leaves, float)
    back = at - tees.FITTING[0] * leaves
    mid = 0.5 * (at + back)
    return (km._rod(back, at, tees.FITTING[1], tees.PIPE_INK)
            + km._rod(mid, np.array([mid[0], mid[1], face_z]), tees.PIPE[1], tees.PIPE_INK)), back


def _elbow(at, leaves, way, face_z):
    """Black pipe elbow turning from `way` (incoming hose unit) to `leaves` (span mouth unit), stemmed to `face_z`."""
    import numpy as np
    at, v, w, (rb, r) = np.asarray(at, float), np.asarray(leaves, float), np.asarray(way, float), tees.PIPE
    back = at - tees.FITTING[0] * v
    to_v, to_w = (w @ v) * v - w, v - (v @ w) * w
    centre = back + rb * to_v / np.linalg.norm(to_v)
    to_w = to_w / np.linalg.norm(to_w)
    bend = [centre + rb * (-to_w * math.cos(a) + w * math.sin(a)) for a in np.linspace(0.0, math.acos(w @ v), 9)]
    inlet = bend[0] - tees.PIPE_LEG * w
    mid = bend[4]
    tris = (km._rod(back, at, tees.FITTING[1], tees.PIPE_INK)
            + km._rod(inlet - tees.FITTING[0] * w, inlet, tees.FITTING[1], tees.PIPE_INK)
            + km._rod(inlet, bend[0], r, tees.PIPE_INK)
            + km._rod(mid, np.array([mid[0], mid[1], face_z]), r, tees.PIPE_INK))
    return (tris + [t for p, q in zip(bend[:-1], bend[1:]) for t in km._rod(p, q, r, tees.PIPE_INK)],
            inlet - tees.FITTING[0] * w)


def _ends():
    """{'still', 'turning': (mouth, leaves)} of Stage 1's rear span in `shoulder_laid`'s frame."""
    import numpy as np
    back, sp, piv = np.array(sr.BACK), sr.span(), np.array([sr.X_ABD, 0.0, 0.0])
    return {side: (back @ np.array(sp[key][1]) + piv, back @ np.array(sp[key][2]))
            for side, key in (('still', 'from'), ('turning', 'to'))}


@functools.lru_cache(maxsize=None)
def laid():
    """{'still', 'lever', 'saddle': fitting/pipe tris, 'up', 'down': hose centre lines (k, 3)}."""
    import numpy as np
    lay_r, lay_s = sr.laid(), sl.laid()
    ends = _ends()
    still_fit, _b0 = _fitting(*ends['still'], lay_r['fork'][1])
    c_line = sl.coil(0.0, 64)
    c_in = c_line[0] + np.array([sl.MOUTH_M + tees.HOSE_R + 0.008, 0.0, 0.0])
    c_dir = (c_line[0] - c_line[1])
    c_dir = c_dir / float(np.linalg.norm(c_dir))
    way_in = np.array([-0.52, -0.35, 0.78])
    way_in /= float(np.linalg.norm(way_in))
    lever_fit, elbow_in = _elbow(*ends['turning'], way_in, sr._lever_z()[1])
    up = tees.laid(elbow_in, -way_in, c_in, c_dir)
    # Run 2 (`down`, on Stage 2 rotor): from coil stem (`c_line[-1]`) outside housing into rear-bottom saddle pipe!
    tube = lay_s['tube']
    z_sad = tube + em.OVAL_PAST[0] + 0.002
    y_sad = -0.5 * (lay_s['band'][2] + lay_s['top'])
    saddle_pipe, s_end, s_way = _pipe((0.0, y_sad, z_sad), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (0.60, 0.80))
    r_clear = lay_s['bore'] + sl.DRUM_M + tees.HOSE_R + 0.004
    a_stem = math.atan2(float(c_line[-1][2]), float(c_line[-1][1]))
    stem_out = np.array([min(float(c_line[-1][0]) - 0.006, sr.X_ABD - sr.HUB_R - 0.006),
                         r_clear * math.cos(a_stem), r_clear * math.sin(a_stem)])
    stem_tan = np.array([-0.60, -math.sin(a_stem), math.cos(a_stem)])
    stem_tan /= float(np.linalg.norm(stem_tan))
    down = tees.laid(stem_out, stem_tan, s_end, s_way)
    return {'still': still_fit, 'lever': lever_fit, 'saddle': saddle_pipe, 'up': up, 'down': down}


@functools.lru_cache(maxsize=None)
def parts():
    """{'still': on Stage 1's gimbal, 'yoke': on Stage 2's turning housing}."""
    from tools.render import shoulder_mount as sm
    got = laid()
    gimbal = sr.parts()['yoke'] + sm.still()['block'] + got['lever']
    rotor = sm.turning()['housing'] + sm.turning()['arm saddle'] + got['saddle']
    return {'still': got['lever'] + tees.hose(got['up']) + tees.clipped(got['up'], gimbal),
            'yoke': got['saddle'] + tees.hose(got['down']) + tees.clipped(got['down'], rotor)}


@functools.lru_cache(maxsize=None)
def aired(step=15.0):
    """Least air, m, between Stage 1's hose run, Stage 2's hose run and moving neighbors across travel."""
    import numpy as np
    got = parts()
    h, out = 0.006, 9.0
    fixed_1 = km._points(got['still'] + sr.parts()['yoke'], 0.002)
    mov_2 = km._points(got['yoke'], 0.002)
    keys = km._key(np.floor(fixed_1 / h).astype(np.int64))
    order = np.argsort(keys)
    lo, hi = sl.travel()
    for turn in np.arange(lo, hi + 0.1, step):
        out = min(out, float(km._least(keys[order], fixed_1[order], mov_2 @ km._rx(float(turn)).T, h)))
    fork_pts = km._points(sr.parts()['still'], 0.002)
    keys_f = km._key(np.floor(fork_pts / h).astype(np.int64))
    order_f = np.argsort(keys_f)
    piv = np.array([sr.X_ABD, 0.0, 0.0])
    run_1 = km._points(tees.hose(laid()['up']), 0.002) - piv
    for roll in np.linspace(sr.TRAVEL[0], sr.TRAVEL[1], 7):
        c, s = math.cos(math.radians(float(roll))), math.sin(math.radians(float(roll)))
        rz = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
        out = min(out, float(km._least(keys_f[order_f], fork_pts[order_f], run_1 @ rz.T + piv, h)))
    return out


def checked():
    """{'bend', 'free', 'air'}: what `test_views_built` holds."""
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
    views = [(0.0, 35.0, 18.0), (0.0, -145.0, 14.0), (sr.TRAVEL[1], 35.0, 18.0), (sr.TRAVEL[0], -145.0, 14.0)]
    sheet = Image.new('RGB', (760 * len(views), 800), (18, 18, 18))
    for i, (roll, yaw, tilt) in enumerate(views):
        tris = [t for part in sm.posed(0.0, roll, chest=True).values() for t in part]
        sheet.paste(m.draw(tris, yaw=yaw, pitch=tilt, px=760, half=0.18, centre=(0.04, -0.04, -0.04)), (760 * i, 40))
    ImageDraw.Draw(sheet).text((12, 10), "the shoulder's bus: rest ahead | behind | T+ %.0f | in %.0f"
                               % sr.TRAVEL[::-1], fill=(235, 235, 235))
    sheet.save(args.out)
    print(args.out, checked())
    return 0


if __name__ == '__main__':
    sys.exit(main())
