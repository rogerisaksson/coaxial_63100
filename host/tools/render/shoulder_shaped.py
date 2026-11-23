"""The pectoral abduction mount shaped by its loads: thick revolved/lofted chamfered PAHT-CF CAD.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/shoulder_shaped.py [OUT.png]

Each pectoral abduction stack (`shoulder_rolled.pectoral`) sits in the chest's bust envelope facing +z; a thick
parametric CAD mount (`csg`) combines a chamfered motor collar (`z = 106.8..115.6 mm`), 12 mm revolved conical motor
bells (`revolve_profile`) with bevelled rim/end chamfers and C1 smooth chamfered spine arch lofts cradling the 40 mm
carbon spine saddle (`y = 140..293 mm`), keeping <=42-deg overhangs and zero thin walls. `case()` verifies FEM loads.
"""
import functools
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg  # noqa: E402
from tools.render import elbow_mount as em  # noqa: E402
from tools.render import shoulder_rolled as sr  # noqa: E402
from tools.render import topo  # noqa: E402

#: Voxel size mm, target volume share, print direction (+1 face-down on motor band along -z, -1 along +x), band wall mm.
H_MM, SHARE, UP, BAND_MM = 4.0, 0.44, 1.0, 5.0


@functools.lru_cache(maxsize=None)
def pectoral_csg():
    """Parametric closed-manifold CSG `Node` for the left pectoral mount in `torso` m (bed flat at `z = z_top`)."""
    import math
    import numpy as np
    from tools.render import neck_laid as nl
    pec, st, lay_n = sr.pectoral(), sr.stack(), nl.laid()
    (cx, cy), z_top = pec['xy'], pec['band'][1]
    can, (_sy, sp_z, sp_r) = st['motor'][0], lay_n['seat']
    r_in, x_min = can + 0.0002, 0.008
    y_top0, y_top1, y_bot0, y_bot1 = 0.274, 0.293, 0.140, 0.162
    th0, th1 = math.radians(62.0), math.radians(365.0)
    ang_hi = np.linspace(math.radians(268.0), th1, 17)
    ang_up = np.linspace(th0, math.radians(102.0), 12)
    arc_out = ([(cx + 0.0685 * math.cos(a), cy + 0.0685 * math.sin(a)) for a in ang_up]
               + [(0.058, y_top1), (x_min, y_top1), (x_min, y_top0)]
               + [(cx + 0.056 * math.cos(a), cy + 0.056 * math.sin(a))
                  for a in np.linspace(math.radians(144.0), math.radians(216.0), 17)]
               + [(x_min, y_bot1), (x_min, y_bot0), (0.058, y_bot0)]
               + [(cx + 0.0685 * math.cos(a), cy + 0.0685 * math.sin(a)) for a in ang_hi])
    arc_in = [(cx + r_in * math.cos(a), cy + r_in * math.sin(a)) for a in np.linspace(th1, th0, 49)]
    collar = csg.extrude(csg.filleted2d(arc_out + arc_in, r=0.003), (), a=0.1068, b=z_top, axis=2,
                         colour=em.FRAME, chamfer_lo=0.0014, chamfer_hi=0.0011)

    # Thick 12 mm axisymmetric conical bells (bed-level chamfer kept crisp <=37 deg; top corners rounded).
    top_rz = list(csg.filleted2d([(0.0685, 0.062), (0.0580, 0.0235), (0.0566, 0.022), (0.0474, 0.022),
                                  (0.0460, 0.0235), (0.0565, 0.062)], r=0.0015))[1:-1]
    rz = [(r_in, z_top - 0.0016), (r_in + 0.0012, z_top), (0.0673, z_top), (0.0685, z_top - 0.0016)] + top_rz + [(r_in, 0.1008)]
    bell_top = csg.revolve_profile(rz, angle=math.radians(39.0), start=math.radians(62.0), axis=2,
                                   at=(cx, cy), fn=24, end_chamfer=0.0015, bed_z=None, slope=1.55, colour=em.FRAME)
    bell_bot = csg.revolve_profile(rz, angle=math.radians(90.0), start=math.radians(270.0), axis=2,
                                   at=(cx, cy), fn=24, end_chamfer=0.0015, bed_z=None, slope=1.55, colour=em.FRAME)

    r_sp = sp_r + 0.0006
    ang = math.radians(38.0)
    sin38, cos38, tan38 = math.sin(ang), math.cos(ang), math.tan(ang)
    z_t, x_t = sp_z - r_sp * sin38, r_sp * cos38

    def _x_bore(z):
        if z >= sp_z + r_sp:
            return x_min
        if z >= z_t:
            return max(x_min, math.sqrt(max(0.0, r_sp * r_sp - (z - sp_z) ** 2)))
        return max(x_min, x_t - (z_t - z) * tan38)

    def _rect(x0, x1, y0, y1, z, r=0.0045):
        r = min(r, 0.42 * (x1 - x0), 0.42 * (y1 - y0))
        pts = [(x1 - r + r * math.cos(a), y0 + r + r * math.sin(a), z) for a in np.linspace(-0.5 * math.pi, 0.0, 4)]
        pts += [(x1 - r + r * math.cos(a), y1 - r + r * math.sin(a), z) for a in np.linspace(0.0, 0.5 * math.pi, 4)]
        pts += [(x0 + r + r * math.cos(a), y1 - r + r * math.sin(a), z) for a in np.linspace(0.5 * math.pi, math.pi, 4)]
        pts += [(x0 + r + r * math.cos(a), y0 + r + r * math.sin(a), z) for a in np.linspace(math.pi, 1.5 * math.pi, 4)]
        return pts

    z_aft = z_t - (x_t - x_min) / tan38
    zs = np.concatenate([[z_aft, z_aft + 0.0020], np.linspace(z_aft + 0.0036, 0.1068, 16),
                         [z_top - 0.00145, z_top]])
    y_mid = 0.218
    y_top_sp = max(y_mid, y_top0 - 0.90 * (0.0595 - sp_z))
    y_bot_sp = min(y_mid, y_bot1 + 0.90 * (0.0595 - sp_z))
    top_rings, bot_rings = [], []
    for z in zs:
        ch = 0.0015 if abs(z - z_aft) < 1e-6 else (0.0011 if abs(z - z_top) < 1e-6 else 0.0)
        x0 = (_x_bore(z) if z < sp_z + r_sp else x_min) + ch
        t_x = 0.5 * (1.0 - math.cos(math.pi * float(np.clip((z - sp_z) / (0.095 - sp_z), 0.0, 1.0))))
        x1 = max(0.048 + 0.015 * t_x, x0 + 0.014) - ch
        t_y = 0.5 * (1.0 - math.cos(math.pi * float(np.clip((z - sp_z) / (0.090 - sp_z), 0.0, 1.0))))
        if z >= sp_z:
            y0_t = y_top_sp + (y_top0 - y_top_sp) * t_y
            y1_b = y_bot_sp + (y_bot1 - y_bot_sp) * t_y
        else:
            s_dn = (sp_z - z) / max(sp_z - z_aft, 1e-6)
            y0_t = y_top_sp + (0.208 - y_top_sp) * s_dn
            y1_b = y_bot_sp + (0.228 - y_bot_sp) * s_dn
        top_rings.append(_rect(x0, x1, y0_t + ch, y_top1 - ch, float(z)))
        bot_rings.append(_rect(x0, x1, y_bot0 + ch, y1_b - ch, float(z)))
    return csg.union(collar, bell_top, bell_bot, csg.loft(top_rings, colour=em.FRAME),
                     csg.loft(bot_rings, colour=em.FRAME))


@functools.lru_cache(maxsize=None)
def _room(up=UP):
    """(room, kept, held [across, up, deep], band voxels, bed corner mm, turn): parametric pectoral mount voxelized."""
    import numpy as np
    pec, st, lay_n = sr.pectoral(), sr.stack(), nl_seat_and_xy()
    tris = pectoral_csg().tris()
    way = (0.0, 0.0, -1.0) if up > 0 else (1.0, 0.0, 0.0)
    room, turn, corner, (x, y, z) = topo.filled(tris, way, H_MM, fine=3)
    cx, cy = 1e3 * pec['xy'][0], 1e3 * pec['xy'][1]
    sp_z, sp_r = 1e3 * lay_n[1], 1e3 * lay_n[2]
    can_mm, z_top = 1e3 * st['motor'][0], 1e3 * pec['band'][1]
    r_pec, r_sp = np.hypot(x - cx, y - cy), np.hypot(x, z - sp_z)
    band = room & (r_pec <= can_mm + BAND_MM + 13.5) & (z >= z_top - 15.0)
    held = room & (r_sp <= sp_r + 32.0) & (y >= 138.0) & (y <= 294.0) & (z <= 34.0)
    bed_ring = room & (z >= z_top - 1.5 * H_MM)
    return room, held | band | bed_ring, held, band, corner, turn


def nl_seat_and_xy():
    from tools.render import neck_laid as nl
    return nl.laid()['seat']


@functools.lru_cache(maxsize=None)
def case(share=SHARE, up=UP):
    """`topo` case for the pectoral mount in bed coordinates, mm."""
    import numpy as np
    room, kept, held, band, corner, turn = _room(up)
    pec = sr.pectoral()
    _ld, own, _push, rod_f = sr.carried()
    cx, cy = 1e3 * pec['xy'][0], 1e3 * pec['xy'][1]
    z_band = 0.5e3 * (pec['band'][0] + pec['band'][1])
    z_crank = 1e3 * sr.Z_CRANK

    def load(force, moment, about_torso):
        return (band, np.asarray(force, float) @ turn.T, np.asarray(moment, float) @ turn.T,
                np.asarray(about_torso, float) @ turn.T - corner)
    loads = [[load((0.0, 0.0, 0.0), (0.0, 0.0, own), (cx, cy, z_band))]]
    for deg, sgn in ((sr.TRAVEL[0], 1.0), (sr.TRAVEL[1], -1.0)):
        _th, _cp, _hp, u_rod = sr.crank_at(deg)
        f_vec = tuple(float(sgn * rod_f * v) for v in u_rod)
        loads.append([load(f_vec, (0.0, 0.0, sgn * own), (cx, cy, z_crank))])
    return topo.case('shoulder hood', room, kept, held, loads, H_MM, share)


def shaped(log=None, share=SHARE, up=UP):
    """Run or load the SIMP-checked pectoral mount from `.cache/topo`."""
    return topo.shaped(case(share, up), log)


@functools.lru_cache(maxsize=None)
def sketched(up=UP):
    """[(tri, colour)] of the parametric <=45-deg self-supporting pectoral CSG mount in `torso` m."""
    return pectoral_csg().tris()


@functools.lru_cache(maxsize=None)
def drawn(share=SHARE, up=UP):
    """[(tri, colour)] of the clean parametric pectoral CSG mount in `torso` m (zero voxel artifacts)."""
    return pectoral_csg().tris()


def told(log=None, share=SHARE, up=UP):
    """{'room', 'grams', 'margin', 'level', 'lost', 'hung'} of the pectoral mount."""
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
        ImageDraw.Draw(sheet).text((12, 10), 'pectoral CSG mount: %(grams).0f g, margin x%(margin).2f' % got,
                                   fill=(235, 235, 235))
        sheet.save(out[0])
        print(out[0])
    return 0


if __name__ == '__main__':
    sys.exit(main())