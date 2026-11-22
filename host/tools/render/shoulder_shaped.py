"""The pectoral abduction mount shaped by its loads: triangulated PAHT-CF truss on the chest.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/shoulder_shaped.py [OUT.png]

Each pectoral abduction stack (`shoulder_rolled.pectoral`) sits in the chest's bust envelope facing +z; here the SIMP
optimiser (`topo`) shapes its PAHT-CF mount from the motor clamping band (`z = +90..+114 mm`) back around the 100 mm
inverter cover (`<= 45 deg` self-supporting skirt, zero bridges) to two triangulated clamps on the 40 mm carbon spine
tube and the 35 mm carbon crossbar, clear of the trunk's swept cranks and rods. `drawn` surfaces the converged solid
smoothly via `topo.faced` (`knee_topo.surfaced` Dual Surface-Net B-spline SDF) in `torso` m.
"""
import functools
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import elbow_mount as em  # noqa: E402
from tools.render import shoulder_rolled as sr  # noqa: E402
from tools.render import topo  # noqa: E402

#: Voxel size mm, target volume share, print direction (+1 face-down on motor band along -z, -1 along +x), band wall mm.
H_MM, SHARE, UP, BAND_MM = 4.0, 0.35, 1.0, 5.0


def _axes(up):
    """(3, 3): bed rows (across, up, deep) in `torso` coordinates `(x, y, z)`."""
    import numpy as np
    return np.array([(1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)] if up > 0
                    else [(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)])


def _bed(v, up):
    """Torso vector `v = (x, y, z)` mapped into bed coordinates `(across, up, deep)` without float drift."""
    import numpy as np
    v = np.asarray(v, float)
    return np.array([v[0], -v[2], v[1]]) if up > 0 else np.array([v[2], v[0], v[1]])


@functools.lru_cache(maxsize=None)
def _room(up=UP):
    """(room, kept, held [across, up, deep], band voxels, bed corner mm): triangulated pectoral mount."""
    import numpy as np
    from tools.render import neck_laid as nl
    pec, st, lay_n = sr.pectoral(), sr.stack(), nl.laid()
    (cx, cy), (b0, _b1), (m0, _m1) = pec['xy'], pec['board'], pec['motor']
    bar_y, bar_z, bar_r = lay_n['bar']
    _seat_y, sp_z, sp_r = lay_n['seat']
    h = H_MM
    xs = np.arange(0.0, 108.0, h) + 0.5 * h
    ys = np.arange(176.0, 336.0, h) + 0.5 * h
    zs = (np.arange(1e3 * pec['band'][1], -14.0, -h) - 0.5 * h) if up > 0 else (
        np.arange(-14.0, 1e3 * pec['band'][1], h) + 0.5 * h)
    if up > 0:
        x, z, y = np.meshgrid(xs, zs, ys, indexing='ij')
    else:
        z, x, y = np.meshgrid(zs, xs, ys, indexing='ij')
    r_sp = np.hypot(x, z - 1e3 * sp_z)
    r_bar = np.hypot(y - 1e3 * bar_y, z - 1e3 * bar_z)
    r_pec = np.hypot(x - 1e3 * cx, y - 1e3 * cy)
    z_brd_lo, z_brd_hi = 1e3 * (b0 - 0.003), 1e3 * m0
    can_mm, brd_mm = 1e3 * st['motor'][0], 1e3 * pec['guard'][2] + 1.5
    out = ((r_sp < 1e3 * sp_r + 0.6)
           | ((r_bar < 1e3 * bar_r + 1.0) & (x >= 41.0))
           | ((x < 42.0) & (y > 276.0) & (z < 49.0))
           | ((x >= 44.0) & (y < 235.0) & (z < 71.5))
           | ((y < 201.0) & (z < 63.0))
           | ((r_pec < brd_mm) & (z >= z_brd_lo) & (z <= z_brd_hi))
           | ((r_pec < can_mm) & (z > z_brd_hi)))
    z_top = 1e3 * pec['band'][1]
    r_skirt = np.where(z >= z_brd_hi, can_mm + BAND_MM + 3.0 + 0.88 * (z_top - z),
                       brd_mm + 9.0 + 0.45 * (z_brd_hi - z))
    env = ((r_pec <= r_skirt)
           | ((z <= 66.0) & (x <= 60.0) & (y >= 210.0) & (y <= 276.0) & (r_sp <= 50.0 + 0.60 * np.maximum(z, 0.0)))
           | ((z <= 66.0) & (x >= 41.0) & (x <= 84.0) & (y >= 246.0) & (z <= 22.0 + 0.60 * (330.0 - y))))
    room = topo.stood((~out & env).transpose(1, 0, 2)).transpose(1, 0, 2)
    band = room & (r_pec <= can_mm + BAND_MM + 2.5) & (z >= z_brd_hi + 1.5)
    held_sp = room & (r_sp <= 1e3 * sp_r + 8.5) & (y >= 212.0) & (y <= 274.0) & (z <= 32.0)
    held_bar = room & (r_bar <= 1e3 * bar_r + 10.5) & (x >= 42.5) & (x <= 82.0)
    held = held_sp | held_bar
    corner = _bed((xs[0], ys[0], zs[0]), up) - 0.5 * h
    return room, held | band, held, band, corner


@functools.lru_cache(maxsize=None)
def case(share=SHARE, up=UP):
    """`topo` case for the triangulated pectoral mount in bed coordinates, mm."""
    room, kept, held, band, corner = _room(up)
    pec = sr.pectoral()
    _ld, own, _push, rod_f = sr.carried()
    cx, cy = 1e3 * pec['xy'][0], 1e3 * pec['xy'][1]
    z_band = 0.5e3 * (pec['band'][0] + pec['band'][1])
    z_crank = 1e3 * sr.Z_CRANK

    def load(force, moment, about_torso):
        return band, _bed(force, up), _bed(moment, up), _bed(about_torso, up) - corner
    loads = [[load((0.0, 0.0, 0.0), (0.0, 0.0, own), (cx, cy, z_band))]]
    for deg, sgn in ((sr.TRAVEL[0], 1.0), (sr.TRAVEL[1], -1.0)):
        _th, _cp, _hp, u_rod = sr.crank_at(deg)
        f_vec = tuple(float(sgn * rod_f * v) for v in u_rod)
        loads.append([load(f_vec, (0.0, 0.0, sgn * own), (cx, cy, z_crank))])
    return topo.case('shoulder hood', room, kept, held, loads, H_MM, share)


def shaped(log=None, share=SHARE, up=UP):
    """Run or load the SIMP-shaped pectoral mount from `.cache/topo`."""
    return topo.shaped(case(share, up), log)


@functools.lru_cache(maxsize=None)
def drawn(share=SHARE, up=UP):
    """[(tri, colour)] of the smoothly surfaced pectoral mount in `torso` m, or None until shaped."""
    if not topo.ready(case(share, up)):
        return None
    got, corner = topo.shaped(case(share, up)), _room(up)[4]
    if got['margin'] < 1.0 or got['lost']:
        return None
    return [(1e-3 * (tri + corner) @ _axes(up), em.FRAME) for tri in topo.faced(got, case(share, up))]


def told(log=None, share=SHARE, up=UP):
    """{'room', 'grams', 'margin', 'level', 'lost', 'hung'} of the shaped pectoral mount."""
    got = shaped(log, share, up)
    layers = got['solid'].transpose(1, 0, 2)
    return {'room': round(float(got['room']), 1), 'grams': round(float(got['grams']), 1),
            'margin': round(float(got['margin']), 2), 'level': float(got['level']), 'lost': int(got['lost']),
            'hung': int((layers & ~topo.stood(layers)).sum())}


def main(argv=None):
    from PIL import Image, ImageDraw
    from tools.render import mounting as m, shoulder_mount as sm
    out = (sys.argv[1:] if argv is None else argv)[:1]
    got = told(log=print)
    print('pectoral mount %(grams).1f g of %(room).1f g at %(level).1f, margin x%(margin).2f, %(hung)d hung, '
          '%(lost)d lost' % got)
    if out:
        views = [(0.0, 28.0, 14.0), (0.0, -155.0, 14.0), (sr.TRAVEL[1], 25.0, 12.0), (sr.TRAVEL[0], 35.0, 12.0)]
        sheet = Image.new('RGB', (760 * len(views), 800), (18, 18, 18))
        for i, (roll, yaw, tilt) in enumerate(views):
            tris = [t for part in sm.posed(0.0, roll, chest=True).values() for t in part]
            sheet.paste(m.draw(tris, yaw=yaw, pitch=tilt, px=760, half=0.19, centre=(0.04, -0.05, -0.04)), (760 * i, 40))
        ImageDraw.Draw(sheet).text((12, 10), 'pectoral SIMP mount shaped: %(grams).0f g, margin x%(margin).2f' % got,
                                   fill=(235, 235, 235))
        sheet.save(out[0])
        print(out[0])
    return 0


if __name__ == '__main__':
    sys.exit(main())
