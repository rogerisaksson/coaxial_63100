"""Parametric mechanical head CAD with stereo gimbals, bolted armor and curved OLED displays."""
import argparse
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg, face_cranium as fc, face_honeycomb as fh, face_laid as fl, neck_mount as nm  # noqa: E402
from tools.render.robot_worn import NECK_RINGS  # noqa: E402

PRESETS = {
    'neutral': {'gaze': (0.0, 0.0, 0.02), 'blink': 0.0, 'brow_lift': 0.0, 'brow_tilt': 0.0, 'smile': 0.0, 'jaw_open': 0.0},
    'smile': {'gaze': (0.0, -0.03, 0.05), 'blink': 0.22, 'brow_lift': 0.64, 'brow_tilt': 0.56, 'smile': 1.0, 'jaw_open': 0.14},
    'curious': {'gaze': (0.24, -0.10, 0.06), 'blink': 0.0, 'brow_lift': 1.0, 'brow_tilt': 1.0, 'smile': 0.24, 'jaw_open': 0.30},
    'focused': {'gaze': (0.0, 0.12, 0.14), 'blink': 0.54, 'brow_lift': -0.95, 'brow_tilt': -1.0, 'smile': -0.20, 'jaw_open': 0.0},
    'speaking': {'gaze': (-0.10, -0.03, 0.05), 'blink': 0.08, 'brow_lift': 0.42, 'brow_tilt': 0.26, 'smile': 0.48, 'jaw_open': 0.84},
}

STATIC_PLATE_KEYS = ('cranium_frontal', 'cranium_dome', 'cranium_occipital', 'left_temporal', 'right_temporal',
                     'left_mastoid', 'right_mastoid', 'left_orbital_pillar', 'right_orbital_pillar',
                     'nasal_bridge', 'left_maxilla', 'right_maxilla', 'oled_bezel', 'oled_screen')


def rod(p0, p1, r=0.00080, col=fh.TITANIUM):
    """Build titanium pushrod between `p0` and `p1`."""
    np = fl.np_mod()
    a, b = np.asarray(p0, float), np.asarray(p1, float)
    ln = float(np.linalg.norm(b - a))
    if ln < 0.003:
        return []
    u, c2, c3 = (b - a) / ln, csg.circle2d(r, fn=14), csg.circle2d(1.45 * r, fn=14)
    return (csg.sweep(c2, [a + u * s for s in (0.0, 0.0016, 0.5 * ln, ln - 0.0016, ln)], chamfer_ends=0.35 * r, colour=col).tris()
            + [t for s0, s1 in ((a, a + 0.0014 * u), (b - 0.0014 * u, b)) for t in csg.sweep(c3, [s0, s1], chamfer_ends=0.3 * r, colour=fh.PANEL_RIM).tris()])


def micro_servo(at, axis, ang, hlen=0.0054, hdir=(0.0, 0.0, 1.0), dual=False, sgn=1.0):
    """Build sub-micro servo at `at` with horn at `ang` rad."""
    np, (cx, cy, cz), ai = fl.np_mod(), (float(v) for v in at), {'x': 0, 'y': 1, 'z': 2}[axis]
    hx, hy, hz = ((0.0048, 0.0042, 0.0046), (0.0042, 0.0048, 0.0046), (0.0042, 0.0046, 0.0048))[ai]
    ur, vr = (((-hy, hy), (-hz, hz)), ((-hz, hz), (-hx, hx)), ((-hx, hx), (-hy, hy)))[ai]
    lo, hi, off = (cx - hx, cy - hy, cz - hz)[ai], (cx + hx, cy + hy, cz + hz)[ai], ((0.0, cy, cz), (cx, 0.0, cz), (cx, cy, 0.0))[ai]
    mid = 0.5 * (lo + hi)
    body = (csg.extrude(csg.filleted2d(csg.box2d(ur, vr), r=0.0008, n=4), a=lo, b=hi, axis=ai, colour=fh.SERVO_CASE, chamfer=0.0004).translate(*off).tris()
            + csg.extrude(csg.filleted2d(csg.box2d((ur[0] - 0.0014, ur[1] + 0.0014), (vr[0] * 0.45, vr[1] * 0.45)), r=0.0004),
                          a=mid - 0.0007, b=mid + 0.0007, axis=ai, colour=fh.PANEL_RIM, chamfer=0.00025).translate(*off).tris())
    sp, hd = np.array([cx, cy, cz], float), np.asarray(hdir, float)
    sp[ai], hd[ai] = (hi + 0.0012) if sgn >= 0.0 else (lo - 0.0012), 0.0
    hd = fh.rot(axis, ang) @ (hd / max(float(np.linalg.norm(hd)), 1e-12))
    t1, t0 = sp + float(hlen) * hd, sp - (float(hlen) if dual else 0.0012) * hd
    return body, csg.sweep(csg.filleted2d(csg.box2d((-0.0012, 0.0012), (-0.0006, 0.0006)), r=0.0003), [t0, sp, t1], chamfer_ends=0.0002, colour=fh.TITANIUM).tris(), t1, t0


def oled_driver_module():
    """Build internal OLED driver PCB and FPC flex."""
    return (csg.extrude(csg.filleted2d(csg.box2d((-0.011, 0.011), (0.022, 0.040)), r=0.002), a=0.048, b=0.051, axis=2, colour=(22, 54, 42), chamfer=0.0004).tris()
            + csg.extrude(csg.filleted2d(csg.box2d((-0.0050, 0.0050), (0.026, 0.036)), r=0.0008), a=0.0465, b=0.048, axis=2, colour=fh.BAFFLE_DARK, chamfer=0.0002).tris()
            + csg.sweep(csg.box2d((-0.0045, 0.0045), (-0.0004, 0.0004)), [(0.0, 0.022, 0.050), (0.0, 0.018, 0.062), (0.0, 0.020, 0.072)], chamfer_ends=0.0001, colour=fh.MIC_GOLD).tris())


def eye_oled_local():
    """Build anime eye OLED with circular iris and camera hole."""
    from machine.eyes import gimbal, stereo
    from tools.render import face_contours as fcon, face_hull as fhull, face_ray as fr
    np, fn, fcam = fl.np_mod(), 68, fr.CANONICAL_CAMERAS['front']
    at0 = np.array([0.5 * float(stereo.BASELINE_M), float(gimbal.EYES[1]), float(gimbal.EYES[2])])
    uc_a, vc_a, *_ = fr.project_uv_batch(np.array([fr.head_to_canon(at0)]), 'front')
    uc, vc, loop = float(uc_a[0]), float(vc_a[0]), np.asarray(fcon.active_loop('EYE_CUP_L_LOOP'), float)
    ang = np.linspace(0.0, 2.0 * math.pi, fn, endpoint=False)
    ca, sa, r_alm, nl = np.cos(ang), np.sin(ang), np.zeros(fn), len(loop)
    for k in range(fn):
        dx, dy, bt = float(ca[k]), float(sa[k]), 28.0
        for i in range(nl):
            x1, y1, x2, y2 = loop[i, 0] - uc, loop[i, 1] - vc, loop[(i + 1) % nl, 0] - uc, loop[(i + 1) % nl, 1] - vc
            det = dx * (y2 - y1) - dy * (x2 - x1)
            if abs(det) > 1e-9:
                t, us = (x1 * (y2 - y1) - y1 * (x2 - x1)) / det, (x1 * dy - y1 * dx) / det
                if t > 2.0 and -1e-6 <= us <= 1.0 + 1e-6 and (bt == 28.0 or t < bt):
                    bt = t
        if dx > 0.15:
            bt = min(bt, (646.0 - uc) / dx)
        r_alm[k] = max(12.0, bt - 2.35)
    for _ in range(3):
        r_alm = 0.25 * np.roll(r_alm, 1) + 0.50 * r_alm + 0.25 * np.roll(r_alm, -1)
    r_hole, r_iris = np.full(fn, 12.6), np.minimum(r_alm - 1.15, np.full(fn, 33.0))
    z_dome = lambda x, y: 48.2 - 0.30 * (abs(float(x)) - 24.8) + 4.8 * math.sqrt(max(0.0, 1.0 - (((abs(float(x)) - 24.8) / 16.5) ** 2) - (((float(y) - 139.6) / 12.5) ** 2)))
    po_c = lambda k, x, y: min(0.0, fhull.panel_offset(k, float(x), float(y)), fhull.panel_offset(k, -float(x), float(y)))
    rim_z = lambda x, y: min((fl._eval_icheek_z(x, y) + po_c('face-inner-cheeks', x, y)) if abs(float(x)) < 34.0
                             else (fl._eval_ocheek_z(x, y) + po_c('face-outer-cheeks', x, y)),
                             fl._eval_brow_z(x, y) + po_c('face-brows', x, y))

    def mk_ring(s, is_back=False, z_ch=0.0):
        rp = (r_hole + (r_iris - r_hole) * (s / 0.62)) if s <= 0.62 else (r_iris + (r_alm - r_iris) * ((s - 0.62) / 0.38))
        wc0, pts = max(0.0, (s - 0.36) / 0.64) ** 0.85, []
        for k in range(fn):
            xyz = np.asarray(fr.project_ray_z(uc + float(rp[k] * ca[k]), vc + float(rp[k] * sa[k]), fcam, z_dome, 0.0)[0], float)
            wc, rz0 = max(wc0, fc._sstep(29.0, 32.5, xyz[0])), rim_z(xyz[0], xyz[1])
            xyz[2] = min(xyz[2], rz0 - 1.65) * (1.0 - wc) + min(xyz[2], rz0 - 4.85) * wc + z_ch - (3.8 if (is_back and s > 0.8) else (0.94 if is_back else 0.0))
            if 18.9 <= xyz[0] <= 29.6:
                xyz[2] = max(xyz[2], 40.98 if is_back else 41.75)
            pts.append(fr.canon_to_head(xyz) - at0)
        return np.asarray(pts, float)

    rings = (mk_ring(0.0, True), mk_ring(0.0, False, -0.28), mk_ring(0.06, False), mk_ring(0.24, False),
             mk_ring(0.48, False), mk_ring(0.56, False), mk_ring(0.63, False), mk_ring(0.94, False),
             mk_ring(1.0, False, -0.32), mk_ring(1.0, True), mk_ring(0.48, True))
    cols = ((14, 16, 20), (36, 40, 48), (16, 18, 24), (26, 30, 38), (40, 44, 52),
            (12, 14, 18), (118, 124, 132), (88, 94, 102), fh.PANEL_RIM, fh.CHASSIS_CF, fh.CHASSIS_CF)
    return [t for k in range(len(rings)) for t in fh._stitch_rings(rings[(k + 1) % len(rings)], rings[k], cols[k])]


def _eyes_posed(gaze):
    """Pose stereo cameras, eye OLEDs, 2-axis gimbals and servos."""
    from machine.eyes import gaze as gm, gimbal, stereo
    np = fl.np_mod()
    gz = (max(-gm.YAW, min(gm.YAW, float(gaze[0]))), max(gm.PITCH[0], min(gm.PITCH[1], float(gaze[1]))),
          max(0.0, min(gm.VERGE, float(gaze[2]) if len(gaze) > 2 else 0.0)))
    cam_l, ring_l, pins_l = fl.camera_and_gimbal_local()
    oled_l = eye_oled_local()
    sy_b, sy_h, py_l, py_r = micro_servo((0.0, 0.094, 0.046), 'y', gz[0], 0.0195, (1.0, 0.0, 0.0), True)
    sp_b, sp_h, pp_h, _ = micro_servo((0.0, 0.114, 0.044), 'x', gz[1], 0.0052)
    ry, rz = 0.111, 0.054
    rock = csg.sweep(csg.circle2d(0.0010, fn=12), [(-0.026, ry, rz), (0.026, ry, rz)],
                     chamfer_ends=0.0003, colour=fh.TITANIUM).tris()
    links = sy_h + sp_h + rock + rod(pp_h, (0.0060, ry + 0.0040 * math.cos(gz[1]), rz + 0.0040 * math.sin(gz[1])), 0.00072)
    out = {'gaze_servos': sy_b + sp_b}
    for side, sgn in (('left', 1.0), ('right', -1.0)):
        at0 = np.array([sgn * 0.5 * stereo.BASELINE_M, float(gimbal.EYES[1]), float(gimbal.EYES[2])])
        rm = gimbal.mounted(side)
        piv = at0 - rm @ np.array([0.0, 0.0, -float(gimbal.LEVER[side])])
        ang = gimbal.angles(side, gz)
        rc, rr = gimbal.eye(side, ang), gimbal.turned((ang[0], 0.0)) @ rm
        lx = sgn * 0.0086
        lbox = csg.filleted2d(csg.box2d((min(lx, sgn * 0.0068), max(lx, sgn * 0.0068)), (-0.0010, 0.0010)), r=0.0003)
        ylug = csg.extrude(lbox, a=-0.0007, b=0.0007, axis=2, colour=fh.TITANIUM, chamfer=0.00025).tris()
        out[side + '_camera'] = fh.turn_tris(cam_l, rc, (0.0, 0.0, 0.0), piv)
        out[side + '_eye_oled'] = [(t + at0, c) for t, c in (oled_l if sgn > 0 else fl.mirror_x(oled_l))]
        out[side + '_gimbal'] = (fh.turn_tris(ring_l + ylug, rr, (0.0, 0.0, 0.0), piv)
                                 + fh.turn_tris(pins_l, rm, (0.0, 0.0, 0.0), piv))
        links += (rod((py_l if sgn > 0 else py_r) + np.array([0.0, 0.0, 0.0014]), piv + rr @ np.array([lx, 0.0, 0.0]), 0.00072)
                  + rod(np.array([sgn * 0.025, ry - 0.0040 * math.sin(gz[1]), rz + 0.0040 * math.cos(gz[1])]),
                        piv + rc @ np.array([0.0, 0.0060, 0.0018]), 0.00072))
    out['gaze_linkages'] = links
    return out


def _expressions_posed(blink, brow_lift, brow_tilt, smile, jaw_open):
    """Pose brows, eyelids, OLED PCB and mouth pixels."""
    np = fl.np_mod()
    bl, blf = max(0.0, min(1.0, float(blink))), max(-1.0, min(1.0, float(brow_lift)))
    btl, sm, jw = max(-1.0, min(1.0, float(brow_tilt))), max(-1.0, min(1.0, float(smile))), max(0.0, min(1.0, float(jaw_open)))
    brow0, ulid0, llid0, piv_l = fl.moving_rest()
    out = {'brow_servos': [], 'brow_linkages': [], 'lid_servos': [], 'lid_linkages': [],
           'oled_driver_pcb': oled_driver_module(), 'oled_mouth_graphic': fl.oled_mouth_pixels(sm, jw)}
    dy, ex = 0.0021 * blf, float(piv_l[0])
    for side, sgn in (('left', 1.0), ('right', -1.0)):
        bpiv, rb = np.array([sgn * 0.032, 0.116, 0.072]), fh.rot('z', -sgn * math.radians(4.5 * btl))
        out[side + '_brow'] = fh.turn_tris(brow0 if sgn > 0 else fl.mirror_x(brow0), rb, bpiv, (0.0, dy, 0.0))
        sb1, hb1, tb1, _ = micro_servo((sgn * (ex - 0.012), 0.132, 0.048), 'x', -0.45 * (blf + 0.5 * btl), 0.0050, sgn=sgn)
        sb2, hb2, tb2, _ = micro_servo((sgn * (ex - 0.001), 0.130, 0.042), 'x', -0.45 * (blf - 0.5 * btl), 0.0050, sgn=sgn)
        out['brow_servos'] += sb1 + sb2
        out['brow_linkages'] += (hb1 + hb2 + rod(tb1, bpiv + rb @ np.array([-sgn * 0.007, 0.0, -0.003]) + (0.0, dy, 0.0), 0.00070)
                                 + rod(tb2, bpiv + rb @ np.array([sgn * 0.005, 0.0, -0.005]) + (0.0, dy, 0.0), 0.00070))
        ps, ru, rl = piv_l * (sgn, 1.0, 1.0), fh.rot('x', math.radians(17.0 * bl)), fh.rot('x', -math.radians(10.0 * bl + 3.0 * max(0.0, sm)))
        out[side + '_upper_lid'] = fh.turn_tris(ulid0 if sgn > 0 else fl.mirror_x(ulid0), ru, ps)
        out[side + '_lower_lid'] = fh.turn_tris(llid0 if sgn > 0 else fl.mirror_x(llid0), rl, ps)
        slb, slh, tl, _ = micro_servo((sgn * (ex + 0.004), 0.107, 0.046), 'x', 0.50 * bl, 0.0046, sgn=sgn)
        out['lid_servos'] += slb
        out['lid_linkages'] += slh + rod(tl, ps + ru @ np.array([sgn * 0.0032, 0.0068, 0.0062]), 0.00068)
    return out


def head_parts(gaze=(0.0, 0.0, 0.02), blink=0.0, brow_lift=0.0, brow_tilt=0.0, smile=0.0, jaw_open=0.0, plates=True):
    """Return `{name: [(tri, rgb)]}` of the mechanical head in `head` local frame (m)."""
    out = {'chassis': fl.chassis_static(), 'compute_heatsink': fl.compute_heatsink(), **fl.ear_microphones(), **_eyes_posed(gaze)}
    expr = _expressions_posed(blink, brow_lift, brow_tilt, smile, jaw_open)
    return ({**out, 'inner_liner': fl.inner_liner(), **fl.static_plates(), **expr} if plates
            else {**out, **{k: v for k, v in expr.items() if k.endswith(('_servos', '_linkages', '_pcb'))}})


def neck_skin_tris(nod=0.0, head_yaw=0.0):
    from coaxial.graphics import lit
    from coaxial.graphics.shapes import finer, loft
    np, (nt, hr, *_r) = fl.np_mod(), fl.anchors()
    c, tris, _uv, _m = loft(finer(NECK_RINGS, -0.040, 0.048, 0.004), lit.paint(fh.NECK_SKIN), poles=(-0.044, 0.050))
    yn = c[:, 1].copy()
    c[:, 0], c[:, 2] = c[:, 0] * 0.38, c[:, 2] * 0.38 - 0.022
    if head_yaw:
        wy = np.clip((yn - 0.016) / 0.032, 0.0, 1.0)
        a, dx, dz = np.radians(head_yaw * wy * wy * (3.0 - 2.0 * wy)), c[:, 0] - hr[0], c[:, 2] - hr[2]
        c[:, 0], c[:, 2] = hr[0] + np.cos(a) * dx + np.sin(a) * dz, hr[2] - np.sin(a) * dx + np.cos(a) * dz
    if nod:
        wn = np.clip((yn + 0.036) / 0.061, 0.0, 1.0)
        a, cy, cz = np.radians(nod * wn * wn * (3.0 - 2.0 * wn)), c[:, 1].copy(), c[:, 2].copy()
        c[:, 1], c[:, 2] = np.cos(a) * cy - np.sin(a) * cz, np.sin(a) * cy + np.cos(a) * cz
    pts, ntv = c + np.asarray(nt, float), np.asarray(nt, float)
    return [(pts[t], fh.BAFFLE_DARK) for t in tris] + [(t + ntv, col) for t, col in fc.neck_armor_local(nod=nod, head_yaw=head_yaw)]


def posed(preset=None, nod=0.0, head_yaw=0.0, plates=True, include_neck=True, **kwargs):
    kw = {**PRESETS[preset or 'neutral'], **kwargs}
    nt, _hr, ht, *_r = fl.anchors()
    ry, rx = fh.rot('y', math.radians(float(head_yaw))), fh.rot('x', math.radians(float(nod)))
    out = {k: (fh.turn_tris(fh.turn_tris(ts, ry, (0.0, 0.0, 0.0), ht), rx, nt) if nod else fh.turn_tris(ts, ry, (0.0, 0.0, 0.0), ht))
           for k, ts in head_parts(plates=plates, **kw).items()}
    if include_neck:
        from tools.render import robot_worn as rw
        nmp = nm.posed(nod=nod, turn=head_yaw)
        for k in ('crown', 'head drive', 'head ring', 'carrier'):
            out['mount_' + k.replace(' ', '_')] = rw.apply_livery(nmp[k])
        out['neck_skin'] = neck_skin_tris(nod=nod, head_yaw=head_yaw)
    return out


def _normals(raw, cols, crease=58.0):
    from tools.render import robot_worn as rw
    np = fl.np_mod()
    key = np.c_[np.round(raw.reshape(-1, 3), 6), np.repeat(cols.astype(np.float64), 3, axis=0)]
    _, inv = np.unique(key, axis=0, return_inverse=True)
    return rw._smooth_normals(raw.reshape(-1, 3)[np.unique(inv, return_index=True)[1]], inv.reshape(-1, 3), crease)[2]


def lit_mesh(parts_dict):
    """Pack `{name: [(tri, rgb)]}` into `(positions, normals, uv, materials, index)` for `gpu.LitRaster`."""
    from coaxial.graphics import lit
    np = fl.np_mod()
    flat_keys = {'cranium_frontal', 'left_maxilla', 'right_maxilla', 'oled_bezel', 'left_temporal', 'right_temporal'}
    pl, nl_, ul, ml, il, base = [], [], [], [], [], 0
    for key, tris in parts_dict.items():
        if not tris:
            continue
        raw, cols = np.array([t for t, _c in tris], np.float64), np.array([c for _t, c in tris], np.uint32)
        crease = 68.0 if key in ('nasal_bridge', 'oled_screen', 'left_eye_oled', 'right_eye_oled', *flat_keys) else (
            44.0 if key in ('cranium_dome', 'cranium_occipital', 'neck_skin') else 50.0)
        vn, fc_ = _normals(raw, cols, crease), raw.reshape(-1, 3)
        gloss = ((cols == fh.OPTIC_GLASS).all(1) | (cols == fh.OPTIC_SCLERA).all(1) | (cols == fh.TITANIUM).all(1)
                 | ((key in ('left_eye_oled', 'right_eye_oled')) & (cols[:, 2] >= 36))
                 | (cols == (236, 244, 255)).all(1) | ((key == 'neck_skin') & (cols == fh.SHELL_CF).all(1)))
        flags = (np.uint32(lit.PAINTED) | np.where(gloss, np.uint32(lit.HARD), np.uint32(lit.WOVEN))
                 | (cols[:, 0] << np.uint32(16)) | (cols[:, 1] << np.uint32(8)) | cols[:, 2])
        pl.append(fc_.astype(np.float32))
        nl_.append(vn.astype(np.float32))
        ul.append(np.zeros((len(fc_), 2), np.float32))
        ml.append(np.repeat(flags, 3))
        il.append(np.arange(len(fc_), dtype=np.uint32).reshape(-1, 3) + np.uint32(base))
        base += len(fc_)
    return np.vstack(pl), np.vstack(nl_), np.vstack(ul), np.concatenate(ml), np.vstack(il)


def render_panel(parts_dict, yaw=0.0, pitch=4.0, width=640, height=840, centre=(0.0, 0.530, 0.014),
                 scale=3450.0, distance=1.50, raster=None, bg_dark=False):
    """Render one 2x-SSAA studio panel via `gpu.LitRaster` (or `lit.splat` fallback)."""
    from PIL import Image
    from coaxial.graphics import gpu, lit
    from coaxial.graphics.shapes import view
    np, w2, h2 = fl.np_mod(), 2 * int(width), 2 * int(height)
    pos, nrm, uv, mat, idx = lit_mesh(parts_dict)
    reach = float(np.linalg.norm(pos - np.asarray(centre, np.float32), axis=1).max()) + 0.05
    cam = {'width': w2, 'height': h2, 'cx': 0.5 * w2 - 0.5, 'cy': 0.5 * h2 - 0.5,
           'scale': 2.0 * float(scale), 'aspect': 1.0, 'distance': float(distance)}
    m, lr = view(float(yaw), float(pitch)), raster
    if lr is None:
        try:
            lr = gpu.LitRaster()
        except Exception:
            lr = None
    depth, rgba = lr.raster(pos, nrm, uv, mat, idx, m, cam, centre, reach) if lr else lit.splat((pos, nrm, mat, uv), m, cam, centre)
    gy = np.linspace(0.0, 1.0, h2, dtype=np.float32)[:, None, None]
    bg = np.zeros((h2, 1, 3), np.float32) if bg_dark else ((1.0 - gy) * np.array([22.0, 24.0, 28.0], np.float32) + gy * np.array([8.0, 9.0, 12.0], np.float32))
    ssaa = np.where((depth > 0.0)[..., None], rgba[..., :3].astype(np.float32), bg).reshape(int(height), 2, int(width), 2, 3).mean((1, 3))
    return Image.fromarray(np.clip(np.rint(ssaa), 0, 255).astype(np.uint8))


def render_sheets(out_dir):
    """Render `face_artificial_ortho.png` and `face_artificial_expressions.png`."""
    from PIL import Image, ImageDraw, ImageFont
    from coaxial.graphics import gpu
    os.makedirs(out_dir, exist_ok=True)
    lr = gpu.LitRaster()
    font = ImageFont.load_default()
    specs = (('FRONT ORTHO', posed('neutral'), 0.0, 1.5), ('3/4 PERSPECTIVE', posed('neutral'), -56.0, 3.0),
             ('SIDE PROFILE', posed('neutral'), -90.0, 1.5), ('BACK X-BRACE & SPINE', posed('neutral'), 180.0, 1.5),
             ('INTERNAL SERVOS & OLED PCB', posed('curious', plates=False), -28.0, 10.0))
    ortho = Image.new('RGB', (600 * len(specs), 872), (12, 14, 18))
    po = ImageDraw.Draw(ortho)
    for i, (lbl, pts, yw, pt) in enumerate(specs):
        ortho.paste(render_panel(pts, yw, pt, 600, 820, raster=lr), (i * 600, 52))
        po.text((i * 600 + 20, 16), lbl, fill=(230, 234, 242), font=font)
    op = os.path.join(out_dir, 'face_artificial_ortho.png')
    ortho.save(op)
    names = ('neutral', 'smile', 'curious', 'focused', 'speaking')
    expr = Image.new('RGB', (560 * len(names), 812), (12, 14, 18))
    pe = ImageDraw.Draw(expr)
    for i, nm_ in enumerate(names):
        expr.paste(render_panel(posed(nm_), -24.0, 4.0, 560, 760, scale=3380.0, raster=lr), (i * 560, 52))
        pe.text((i * 560 + 20, 16), nm_.upper(), fill=(230, 234, 242), font=font)
    ep = os.path.join(out_dir, 'face_artificial_expressions.png')
    expr.save(ep)
    return op, ep


def _preset_groups(p):
    st = [t for k in STATIC_PLATE_KEYS for t in p[k]]
    mv = [t for k in ('left_brow', 'right_brow', 'left_upper_lid', 'right_upper_lid', 'left_lower_lid', 'right_lower_lid') for t in p[k]]
    ey = [t for k in ('left_camera', 'right_camera', 'left_eye_oled', 'right_eye_oled', 'left_gimbal', 'right_gimbal', 'gaze_linkages') for t in p[k]]
    return st, mv, ey, st + mv


def _neck_mount_cross(poses):
    from tools.render import clips
    nc = mc = 0
    for pr, yw, nd in poses:
        q = posed(pr, nod=nd, head_yaw=yw, plates=True, include_neck=True)
        ha = [t for k, ts in q.items() if not k.startswith('mount_') and k != 'neck_skin' for t in ts]
        nc += clips.crossed(ha, q['neck_skin'])
        mc += clips.crossed(ha, [t for k, ts in q.items() if k.startswith('mount_') for t in ts])
    return int(nc), int(mc)


def fast_hull_clear():
    """Fast (`~1s`) hull-dependent collision check."""
    from tools.render import clips
    p = head_parts(**PRESETS['neutral'], plates=True)
    st, mv, ey, pl = _preset_groups(p)
    if (clips.crossed(mv, st) or clips.crossed(p['left_ear_mic'] + p['right_ear_mic'], pl)
            or clips.crossed(p['compute_heatsink'], pl) or clips.crossed(ey, p['inner_liner'])):
        return False
    nc, mc = _neck_mount_cross((('neutral', 0.0, 0.0), ('neutral', 60.0, 0.0), ('speaking', -60.0, 25.0)))
    return nc == 0 and mc == 0


def checked():
    """Verify zero crossings, C1 neck clearance, baseline and manifolds."""
    from machine.eyes import gimbal, stereo
    from tools.render import clips, solids
    np, pc = fl.np_mod(), {}
    for nm_ in PRESETS:
        p = head_parts(**PRESETS[nm_], plates=True)
        st, mv, ey, pl = _preset_groups(p)
        pc[nm_] = int(clips.crossed(mv, st)
                      + clips.crossed(p['left_upper_lid'] + p['right_upper_lid'], p['left_lower_lid'] + p['right_lower_lid'])
                      + clips.crossed(p['oled_mouth_graphic'], p['oled_bezel']) + clips.crossed(ey, pl + p['compute_heatsink'])
                      + clips.crossed(p['oled_bezel'], p['left_maxilla'] + p['right_maxilla'] + p['oled_screen'])
                      + clips.crossed(p['cranium_dome'], p['cranium_frontal'] + p['cranium_occipital'])
                      + clips.crossed(p['left_ear_mic'] + p['right_ear_mic'], pl) + clips.crossed(p['compute_heatsink'], pl)
                      + clips.crossed(ey, p['chassis'] + p['inner_liner']))
    bf = head_parts(blink=1.0, plates=True)
    lshut = int(clips.crossed(bf['left_upper_lid'] + bf['right_upper_lid'], bf['left_lower_lid'] + bf['right_lower_lid']))
    nc, mc = _neck_mount_cross((('neutral', 0.0, 0.0), ('speaking', 0.0, 0.0), ('neutral', 60.0, 0.0), ('speaking', -60.0, 25.0)))
    neu = head_parts(**PRESETS['neutral'], plates=True)
    cl, cr = (np.vstack([t for t, _c in neu[k]]).mean(0) for k in ('left_camera', 'right_camera'))
    berr = round(1e3 * abs(float(cl[0] - cr[0]) - float(stereo.BASELINE_M)), 2)
    crown_mm = round(1e3 * float(np.vstack([t for t, _c in neu['cranium_dome']])[:, 1].max()), 2)
    oe = {k: int(solids.printable([1e3 * t for t, _c in neu[k]])['open'])
          for k in ('cranium_frontal', 'cranium_dome', 'cranium_occipital', 'left_temporal', 'left_mastoid',
                    'nasal_bridge', 'left_brow', 'left_maxilla', 'left_orbital_pillar', 'oled_bezel', 'oled_screen',
                    'compute_heatsink', 'inner_liner')}
    return {'ok': sum(pc.values()) == 0 and lshut == 0 and nc == 0 and mc == 0 and berr < 1.0 and sum(oe.values()) == 0 and abs(float(gimbal.DOWN_DEG) - 15.0) < 1e-6,
            'preset_cross': pc, 'lid_shut_cross': lshut, 'neck_cross': nc, 'mount_cross': mc,
            'baseline_mm': round(1e3 * float(cl[0] - cr[0]), 2), 'crown_mm': crown_mm, 'open_edges': oe, 'part_count': len(neu)}


def main(argv=None):
    """CLI entry point to verify metrics and render studio sheets."""
    ap = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    ap.add_argument('out_dir', nargs='?')
    args, metrics = ap.parse_args(argv), checked()
    print(metrics)
    if args.out_dir:
        for p in render_sheets(args.out_dir):
            print(p)
    return 0 if metrics['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
