"""The pectoral abduction mount shaped by its loads: triangulated PAHT-CF truss on the chest.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/shoulder_shaped.py [OUT.png]

Each pectoral abduction stack (`shoulder_rolled.pectoral`) sits in the chest's bust envelope facing +z; here the SIMP
optimiser (`topo`) shapes its PAHT-CF mount from the motor clamping band (`z = +90..+114 mm`) back around the 100 mm
inverter cover (`<= 45 deg` self-supporting skirt, zero bridges) onto the 40 mm carbon spine tube (`y = 192..274 mm`),
clear of swept shoulder joints, clevis, bus hoses and pushrods. `drawn` surfaces the converged solid cleanly via
`topo.faced` (`knee_topo.surfaced` Dual Surface-Net B-spline SDF) in `torso` m.
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
H_MM, SHARE, UP, BAND_MM = 4.0, 0.44, 1.0, 5.0


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


def _teardrop(along, across, r, up):
    """True where `(along, across)` lies inside circle `r` plus circumscribed <=45-deg roof apex along `up`."""
    import math
    import numpy as np
    dist, perp = np.hypot(along, across), np.abs(across)
    return np.where(up * along <= perp, dist < r, perp + up * along < math.sqrt(2.0) * r)


@functools.lru_cache(maxsize=None)
def _room(up=UP):
    """(room, kept, held [across, up, deep], band voxels, bed corner mm): triangulated pectoral mount."""
    import math
    import numpy as np
    from scipy.spatial import KDTree
    from tools.render import knee_mount as km, neck_laid as nl
    from tools.render import shoulder_hosed as sh, shoulder_laid as sl, shoulder_mount as sm, tees
    pec, st, lay_n = sr.pectoral(), sr.stack(), nl.laid()
    (cx, cy), (b0, _b1), (m0, m1) = pec['xy'], pec['board'], pec['motor']
    bar_y, bar_z, bar_r = lay_n['bar']
    _seat_y, sp_z, sp_r = lay_n['seat']
    h = H_MM
    xs = np.arange(4.0, 132.0, h) + 0.5 * h
    ys = np.arange(168.0, 336.0, h) + 0.5 * h
    zs = (np.arange(1e3 * pec['band'][1], -14.0, -h) - 0.5 * h) if up > 0 else (
        np.arange(-14.0, 1e3 * pec['band'][1], h) + 0.5 * h)
    if up > 0:
        x, z, y = np.meshgrid(xs, zs, ys, indexing='ij')
    else:
        z, x, y = np.meshgrid(zs, xs, ys, indexing='ij')
    r_sp = np.hypot(x, z - 1e3 * sp_z)
    r_pec = np.hypot(x - 1e3 * cx, y - 1e3 * cy)
    z_brd_lo, z_brd_hi = 1e3 * (b0 - 0.003), 1e3 * m0
    can_mm, brd_mm = 1e3 * st['motor'][0], 1e3 * pec['guard'][2] + 1.5
    z_top = 1e3 * pec['band'][1]
    # Conical inner cavity from motor grip out to board guard radius at <=43 deg, plus 45-deg teardrop crowns
    # behind spine, crossbar, swept pushrods and swept Stage-1/Stage-2 shoulder assemblies across travel + clasps:
    r_cone = np.minimum(brd_mm, can_mm + 0.94 * np.maximum(1e3 * m1 - 2.0 - z, 0.0))
    sp_bore = _teardrop(z - 1e3 * sp_z, x, 1e3 * sp_r + 0.6, -1.0)
    bar_bore = _teardrop(z - 1e3 * bar_z, y - 1e3 * bar_y, 1e3 * bar_r + 1.0, -1.0) & (x >= 41.0)
    rod_out, pts = np.zeros_like(x, bool), np.stack([x, y, z], axis=-1)
    for deg in np.linspace(sr.TRAVEL[0], sr.TRAVEL[1], 13):
        _th, cp, hp, _u = sr.crank_at(float(deg))
        a, b = 1e3 * cp, 1e3 * hp
        ab = b - a
        t = np.clip(((pts - a) @ ab) / float(ab @ ab), 0.0, 1.0)
        near = a + t[..., None] * ab
        rod_out |= _teardrop(z - near[..., 2], np.hypot(x - near[..., 0], y - near[..., 1]),
                             1e3 * sr.ROD_R + 4.5, -1.0)
    turn_tf, arm_tf, _p_abd = sr._torso_frame()
    piv_sk = np.array([sr.X_ABD, 0.0, 0.0])
    st_1 = (sr.parts()['yoke']
            + [t for k in ('arbor', 'crossbar foot', 'studs', 'hub plate', 'output ring', 'block')
               for t in sm.still()[k]]
            + sh.parts()['still'] + tees.hose(sl.coil(0.0, 96)))
    st_2 = [t for v in sm.turning().values() for t in v] + sh.parts()['yoke']
    p_clev = km._points(sr.parts()['still'], 0.003) @ turn_tf + arm_tf
    p_gimb = km._points(st_1, 0.004)
    p_rot2 = km._points(st_2, 0.004)
    poses = [(0.0, float(r)) for r in np.linspace(sr.TRAVEL[0], sr.TRAVEL[1], 19)] + [
        (64.0, -25.75), (62.0, -25.0), (50.0, -22.0), (-38.0, -18.0), (-25.0, -12.0)]
    clouds = [1e3 * p_clev]
    for turn_d, roll_d in poses:
        c_r, s_r = math.cos(math.radians(roll_d)), math.sin(math.radians(roll_d))
        rz = np.array([[c_r, s_r, 0.0], [-s_r, c_r, 0.0], [0.0, 0.0, 1.0]])
        g_sk = (p_gimb - piv_sk) @ rz.T + piv_sk
        r_sk = ((p_rot2 @ km._rx(turn_d).T) - piv_sk) @ rz.T + piv_sk
        clouds.append(1e3 * (np.vstack([g_sk, r_sk]) @ turn_tf + arm_tf))
    all_sw = np.vstack(clouds)
    m_box = ((all_sw[:, 0] >= -5.0) & (all_sw[:, 0] <= 145.0) & (all_sw[:, 1] >= 155.0)
             & (all_sw[:, 1] <= 345.0) & (all_sw[:, 2] >= -25.0) & (all_sw[:, 2] <= 125.0))
    dists, _ = KDTree(all_sw[m_box]).query(pts.reshape(-1, 3), k=1, distance_upper_bound=6.0)
    arm_clear = dists.reshape(x.shape) < 4.5
    out = (sp_bore | bar_bore | rod_out | arm_clear
           | ((x < 42.0) & (y > 276.0) & (z < 49.0))
           | ((y < 172.0) & (z < 40.0))
           | ((r_pec < brd_mm) & (z >= z_brd_lo) & (z < z_brd_hi))
           | ((r_pec < r_cone) & (z >= z_brd_hi)))
    r_skirt = np.where(z >= z_brd_hi, np.maximum(r_cone + 12.0, can_mm + BAND_MM + 13.5 + 0.90 * (z_top - z)),
                       brd_mm + 12.0 + 0.85 * (z_brd_hi - z))
    env = ((r_pec <= r_skirt)
           | ((x <= 68.0) & (y >= 174.0) & (y <= 276.0) & (z <= 104.0 - 0.55 * np.maximum(204.0 - y, 0.0))
              & (r_sp <= 72.0 + 0.90 * np.maximum(z, 0.0))))
    rl = (~out & env).transpose(1, 0, 2).copy()
    for k in range(1, len(rl)):
        if k % 4 == 0:
            rl[k] &= rl[k - 1]
        else:
            p = np.pad(rl[k - 1], 1)
            rl[k] &= p[1:-1, 1:-1] | p[:-2, 1:-1] | p[2:, 1:-1] | p[1:-1, :-2] | p[1:-1, 2:]
    room = rl.transpose(1, 0, 2)
    bed_ring = room & (z >= z_top - 1.5 * h) & (r_pec <= can_mm + BAND_MM + 13.5)
    band = room & (r_pec <= can_mm + BAND_MM + 13.5) & (z >= z_top - 15.0)
    held = room & (r_sp <= 1e3 * sp_r + 9.0) & (y >= 178.0) & (y <= 274.0) & (z <= 36.0)
    corner = _bed((xs[0], ys[0], zs[0]), up) - 0.5 * h
    return room, held | band | bed_ring, held, band, corner


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
def sketched(up=UP):
    """[(tri, colour)] of the <=45-deg self-supporting pectoral room in `torso` m when `.cache/topo` is cold."""
    from tools.render import knee_topo as kt
    room, kept, _held, _band, corner = _room(up)
    return [(1e-3 * (tri + corner) @ _axes(up), em.FRAME)
            for tri in kt.surfaced(room, kept & room, H_MM, room=room)]


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
