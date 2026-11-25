"""Parametric mechanical head and expressive face CAD for the coaxial humanoid build.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/face_built.py OUT_DIR

Poses the parametric CSG solids of `tools.render.face_laid` without decorative props: coupled
2-axis stereo camera gimbals and three cranial gaze/vergence servos (`gimbal`, `stereo`), flush
binaural ear microphone arrays, high ovoid feminine cranium and segmented facial plates, and
miniature expression servos/linkages mounted onto `neck_mount.turning()['carrier']` clear of `NECK_RINGS`.
"""
import argparse
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import csg, face_honeycomb as fh, face_laid as fl, neck_mount as nm  # noqa: E402
from tools.render.robot_worn import NECK_RINGS  # noqa: E402

PRESETS = {
    'neutral': {'gaze': (0.0, 0.0, 0.02), 'blink': 0.0, 'brow_lift': 0.0, 'brow_tilt': 0.0, 'smile': 0.0, 'jaw_open': 0.0},
    'smile': {'gaze': (0.0, -0.04, 0.05), 'blink': 0.20, 'brow_lift': 0.38, 'brow_tilt': 0.28, 'smile': 1.0, 'jaw_open': 0.16},
    'curious': {'gaze': (0.22, -0.10, 0.06), 'blink': 0.0, 'brow_lift': 0.82, 'brow_tilt': 0.85, 'smile': 0.22, 'jaw_open': 0.08},
    'focused': {'gaze': (0.0, 0.12, 0.14), 'blink': 0.32, 'brow_lift': -0.75, 'brow_tilt': -0.88, 'smile': -0.18, 'jaw_open': 0.0},
    'speaking': {'gaze': (-0.10, -0.03, 0.05), 'blink': 0.06, 'brow_lift': 0.34, 'brow_tilt': 0.20, 'smile': 0.48, 'jaw_open': 0.82},
}

STATIC_PLATE_KEYS = ('cranium_frontal', 'cranium_dome', 'cranium_occipital', 'left_temporal', 'right_temporal',
                     'left_mastoid', 'right_mastoid', 'left_orbital_pillar', 'right_orbital_pillar',
                     'nasal_bridge', 'left_maxilla', 'right_maxilla', 'philtrum', 'upper_lip')


def _eyes_posed(gaze):
    """Pose coupled stereo cameras, 2-axis gimbals and two compact model servos driving eye linkages."""
    from machine.eyes import gaze as gm, gimbal, stereo
    np = fl.np_mod()
    gz = (max(-gm.YAW, min(gm.YAW, float(gaze[0]))), max(gm.PITCH[0], min(gm.PITCH[1], float(gaze[1]))),
          max(0.0, min(gm.VERGE, float(gaze[2]) if len(gaze) > 2 else 0.0)))
    cam_l, ring_l, pins_l = fl.camera_and_gimbal_local()
    sy_b, sy_h, py_l, py_r = fh.micro_servo((0.0, 0.094, 0.043), 'y', gz[0], 0.0215, (1.0, 0.0, 0.0), True)
    sp_b, sp_h, pp_h, _ = fh.micro_servo((0.0, 0.114, 0.040), 'x', gz[1], 0.0056)
    ry, rz = 0.111, 0.050
    rock = csg.sweep(csg.circle2d(0.0011, fn=14), [(-0.029, ry, rz), (0.029, ry, rz)],
                     chamfer_ends=0.0003, colour=fl.TITANIUM).tris()
    links = sy_h + sp_h + rock + fh.rod(pp_h, (0.0063, ry + 0.0044 * math.cos(gz[1]), rz + 0.0044 * math.sin(gz[1])), 0.00080)
    out = {'gaze_servos': sy_b + sp_b}
    for side, sgn in (('left', 1.0), ('right', -1.0)):
        at0 = np.array([sgn * 0.5 * stereo.BASELINE_M, float(gimbal.EYES[1]), float(gimbal.EYES[2])])
        rm = gimbal.mounted(side)
        piv = at0 - rm @ np.array([0.0, 0.0, -float(gimbal.LEVER[side])])
        ang = gimbal.angles(side, gz)
        rc, rr = gimbal.eye(side, ang), gimbal.turned((ang[0], 0.0)) @ rm
        lx = sgn * 0.0094
        lbox = csg.filleted2d(csg.box2d((min(lx, sgn * 0.0074), max(lx, sgn * 0.0074)), (-0.0011, 0.0011)), r=0.00036)
        ylug = csg.extrude(lbox, a=-0.0008, b=0.0008, axis=2, colour=fl.TITANIUM, chamfer=0.0003).tris()
        out[side + '_camera'] = fh.turn_tris(cam_l, rc, (0.0, 0.0, 0.0), piv)
        out[side + '_gimbal'] = (fh.turn_tris(ring_l + ylug, rr, (0.0, 0.0, 0.0), piv)
                                 + fh.turn_tris(pins_l, rm, (0.0, 0.0, 0.0), piv))
        links += (fh.rod((py_l if sgn > 0 else py_r) + np.array([0.0, 0.0, 0.0016]), piv + rr @ np.array([lx, 0.0, 0.0]), 0.00080)
                  + fh.rod(np.array([sgn * 0.028, ry - 0.0044 * math.sin(gz[1]), rz + 0.0044 * math.cos(gz[1])]),
                           piv + rc @ np.array([0.0, 0.0065, 0.0020]), 0.00080))
    out['gaze_linkages'] = links
    return out


def _expressions_posed(blink, brow_lift, brow_tilt, smile, jaw_open):
    """Pose articulated brows, eyelids, lip-corner linkages, hinged mandible, servos and pushrods."""
    np = fl.np_mod()
    bl = max(0.0, min(1.0, float(blink)))
    blf = max(-1.0, min(1.0, float(brow_lift)))
    btl = max(-1.0, min(1.0, float(brow_tilt)))
    sm = max(-1.0, min(1.0, float(smile)))
    jw = max(0.0, min(1.0, float(jaw_open)))
    brow0, ulid0, llid0, piv_l, comm0, jaw0 = fl.moving_rest()
    out = {k: [] for k in ('brow_servos', 'brow_linkages', 'lid_servos', 'lid_linkages',
                           'smile_servos', 'smile_linkages', 'jaw_servos', 'jaw_linkages')}
    dy = 0.0024 * blf
    ex = float(piv_l[0])
    for side, sgn in (('left', 1.0), ('right', -1.0)):
        bpiv, rb = np.array([sgn * (ex + 0.001), 0.114, 0.090]), fh.rot('z', -sgn * math.radians(3.0 * btl))
        out[side + '_brow'] = fh.turn_tris(brow0 if sgn > 0 else fl.mirror_x(brow0), rb, bpiv, (0.0, dy, 0.0))
        sb1, hb1, tb1, _ = fh.micro_servo((sgn * (ex - 0.013), 0.136, 0.052), 'x', -0.45 * (blf + 0.5 * btl), 0.0054, sgn=sgn)
        sb2, hb2, tb2, _ = fh.micro_servo((sgn * (ex - 0.001), 0.134, 0.046), 'x', -0.45 * (blf - 0.5 * btl), 0.0054, sgn=sgn)
        out['brow_servos'] += sb1 + sb2
        out['brow_linkages'] += (hb1 + hb2
                                 + fh.rod(tb1, bpiv + rb @ np.array([-sgn * 0.012, -0.001, -0.003]) + (0.0, dy, 0.0), 0.00075)
                                 + fh.rod(tb2, bpiv + rb @ np.array([sgn * 0.006, -0.001, -0.010]) + (0.0, dy, 0.0), 0.00075))

        ps = piv_l * (sgn, 1.0, 1.0)
        ru, rl = fh.rot('x', math.radians(17.5 * bl)), fh.rot('x', math.radians(-13.5 * bl))
        out[side + '_upper_lid'] = fh.turn_tris(ulid0 if sgn > 0 else fl.mirror_x(ulid0), ru, ps)
        out[side + '_lower_lid'] = fh.turn_tris(llid0 if sgn > 0 else fl.mirror_x(llid0), rl, ps)
        slb, slh, tl, _ = fh.micro_servo((sgn * (ex + 0.005), 0.107, 0.048), 'x', 0.55 * bl, 0.0050, sgn=sgn)
        out['lid_servos'] += slb
        out['lid_linkages'] += slh + fh.rod(tl, ps + ru @ np.array([sgn * 0.0036, 0.0074, 0.0068]), 0.00070)

        cpiv, rc = np.array([sgn * 0.0145, 0.0442, 0.1025]), fh.rot('z', sgn * math.radians(19.0 * sm))
        out[side + '_lip_corner'] = fh.turn_tris(comm0 if sgn > 0 else fl.mirror_x(comm0), rc, cpiv, (0.0, 0.0019 * sm, 0.0))
        ssb, ssh, ts, _ = fh.micro_servo((sgn * 0.027, 0.068, 0.050), 'z', sgn * 0.55 * sm, 0.0054, (sgn, 0.0, 0.0))
        out['smile_servos'] += ssb
        out['smile_linkages'] += ssh + fh.rod(ts, cpiv + rc @ np.array([sgn * 0.0050, 0.0016, -0.0054]) + (0.0, 0.0019 * sm, 0.0), 0.00075)

    jpiv, rj = np.array([0.0, 0.066, 0.024]), fh.rot('x', math.radians(10.2 * jw))
    out['mandible'] = fh.turn_tris(jaw0, rj, jpiv)
    for sgn in (1.0, -1.0):
        sjb, sjh, tj, _ = fh.micro_servo((sgn * 0.030, 0.078, 0.038), 'x', 0.58 * jw, 0.0056, sgn=sgn)
        out['jaw_servos'] += sjb
        out['jaw_linkages'] += sjh + fh.rod(tj, jpiv + rj @ np.array([sgn * 0.0402, -0.024, 0.026]), 0.00080)
    return out


def head_parts(gaze=(0.0, 0.0, 0.02), blink=0.0, brow_lift=0.0, brow_tilt=0.0, smile=0.0, jaw_open=0.0, plates=True):
    """Return `{name: [(tri, rgb)]}` of the mechanical head in `head` local frame (m)."""
    out = {'chassis': fl.chassis_static(), 'compute_heatsink': fl.compute_heatsink(),
           **fl.ear_microphones(), **_eyes_posed(gaze)}
    expr = _expressions_posed(blink, brow_lift, brow_tilt, smile, jaw_open)
    if plates:
        return {**out, 'inner_liner': fl.inner_liner(), **fl.static_plates(), **expr}
    return {**out, **{k: v for k, v in expr.items() if k.endswith(('_servos', '_linkages')) or k.endswith('_lip_corner')}}


def neck_skin_tris(nod=0.0, head_yaw=0.0):
    """Return closed C1 `NECK_RINGS` articulated cervical sheath and pearl-white clavicle gorget in `torso` frame."""
    from coaxial.graphics import lit
    from coaxial.graphics.shapes import finer, loft
    np, (nt, hr, *_r) = fl.np_mod(), fl.anchors()
    c, tris, _uv, _m = loft(finer(NECK_RINGS, -0.040, 0.074, 0.004), lit.paint(fh.NECK_SKIN), poles=(-0.044, 0.076))
    yn = c[:, 1].copy()
    if head_yaw:
        wy = np.clip((yn - 0.040) / 0.034, 0.0, 1.0)
        a = np.radians(head_yaw * wy * wy * (3.0 - 2.0 * wy))
        dx, dz = c[:, 0] - hr[0], c[:, 2] - hr[2]
        c[:, 0], c[:, 2] = hr[0] + np.cos(a) * dx + np.sin(a) * dz, hr[2] - np.sin(a) * dx + np.cos(a) * dz
    if nod:
        wn = np.clip((yn + 0.036) / 0.061, 0.0, 1.0)
        a = np.radians(nod * wn * wn * (3.0 - 2.0 * wn))
        cy, cz = c[:, 1].copy(), c[:, 2].copy()
        c[:, 1], c[:, 2] = np.cos(a) * cy - np.sin(a) * cz, np.sin(a) * cy + np.cos(a) * cz
    pts = c + np.asarray(nt, float)
    return [(pts[t], fh.neck_band(yn, t)) for t in tris]


def posed(preset=None, nod=0.0, head_yaw=0.0, plates=True, include_neck=True, **kwargs):
    """Return `{name: [(tri, rgb)]}` in `torso` frame mounted on `neck_mount`'s `carrier`."""
    kw = {**PRESETS[preset or 'neutral'], **kwargs}
    nt, _hr, ht, *_r = fl.anchors()
    ry, rx = fh.rot('y', math.radians(float(head_yaw))), fh.rot('x', math.radians(float(nod)))
    out = {}
    for k, ts in head_parts(plates=plates, **kw).items():
        tr = fh.turn_tris(ts, ry, (0.0, 0.0, 0.0), ht)
        out[k] = fh.turn_tris(tr, rx, nt) if nod else tr
    if include_neck:
        from tools.render import robot_worn as rw
        nmp = nm.posed(nod=nod, turn=head_yaw)
        for k in ('crown', 'head drive', 'head ring', 'carrier'):
            out['mount_' + k.replace(' ', '_')] = rw.apply_livery(nmp[k])
        out['neck_skin'] = neck_skin_tris(nod=nod, head_yaw=head_yaw)
    return out


def _normals(raw, crease=52.0):
    from tools.render import robot_worn as rw
    np = fl.np_mod()
    cu, inv = np.unique(np.round(raw.reshape(-1, 3), 6), axis=0, return_inverse=True)
    return rw._smooth_normals(cu, inv.reshape(-1, 3), crease)[2]


def lit_mesh(parts_dict):
    """Pack `{name: [(tri, rgb)]}` into `(positions, normals, uv, materials, index)` for `gpu.LitRaster`."""
    from coaxial.graphics import lit
    np = fl.np_mod()
    pl, nl_, ul, ml, il, base = [], [], [], [], [], 0
    for key, tris in parts_dict.items():
        if not tris:
            continue
        raw, cols = np.array([t for t, _c in tris], np.float64), np.array([c for _t, c in tris], np.uint32)
        vn, fc = _normals(raw, 175.0 if key == 'neck_skin' else 62.0), raw.reshape(-1, 3)
        matte = (cols == fh.OPTIC_DARK).all(1) | (cols == fl.BAFFLE_DARK).all(1) | (cols == fh.BROW_DARK).all(1) | (
            (key == 'neck_skin') & ~(cols == fh.SHELL_CF).all(1))
        flags = (np.uint32(lit.PAINTED) | np.where(matte, np.uint32(lit.WOVEN), np.uint32(lit.HARD))
                 | np.where((cols == fl.CHASSIS_CF).all(1), np.uint32(lit.TWILL), np.uint32(0))
                 | (cols[:, 0] << np.uint32(16)) | (cols[:, 1] << np.uint32(8)) | cols[:, 2])
        pl.append(fc.astype(np.float32))
        nl_.append(vn.astype(np.float32))
        ul.append(np.zeros((len(fc), 2), np.float32))
        ml.append(np.repeat(flags, 3))
        il.append(np.arange(len(fc), dtype=np.uint32).reshape(-1, 3) + np.uint32(base))
        base += len(fc)
    return np.vstack(pl), np.vstack(nl_), np.vstack(ul), np.concatenate(ml), np.vstack(il)


def render_panel(parts_dict, yaw=0.0, pitch=6.0, width=640, height=840, centre=(0.0, 0.562, 0.016), scale=2320.0, distance=0.88, raster=None):
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
    bg = (1.0 - gy) * np.array([28.0, 31.0, 38.0], np.float32) + gy * np.array([14.0, 16.0, 21.0], np.float32)
    ssaa = np.where((depth > 0.0)[..., None], rgba[..., :3].astype(np.float32), bg).reshape(int(height), 2, int(width), 2, 3).mean((1, 3))
    return Image.fromarray(np.clip(np.rint(ssaa), 0, 255).astype(np.uint8))


def render_sheets(out_dir):
    """Render `face_artificial_ortho.png` (with High-Crown 3/4) and `face_artificial_expressions.png` at 2x SSAA."""
    from PIL import Image, ImageDraw, ImageFont
    from coaxial.graphics import gpu
    os.makedirs(out_dir, exist_ok=True)
    lr = gpu.LitRaster()
    try:
        font = ImageFont.load_default(size=20)
    except TypeError:
        font = ImageFont.load_default()
    specs = [('FRONT ORTHO', posed('neutral'), 0.0, 4.0),
             ('3/4 STUDIO', posed('neutral'), 30.0, 8.0),
             ('HIGH-CROWN 3/4', posed('neutral'), 26.0, 26.0),
             ('SIDE EAR-MIC PROFILE', posed('neutral'), 88.0, 4.0),
             ('INTERNAL SERVOS & GIMBALS', posed('curious', plates=False), 26.0, 12.0)]
    pw, ph, top = 600, 820, 52
    ortho = Image.new('RGB', (pw * len(specs), ph + top), (16, 18, 24))
    po = ImageDraw.Draw(ortho)
    for i, (lbl, pts, yw, pt) in enumerate(specs):
        ortho.paste(render_panel(pts, yw, pt, pw, ph, raster=lr), (i * pw, top))
        po.text((i * pw + 20, 16), lbl, fill=(230, 234, 242), font=font)
    op = os.path.join(out_dir, 'face_artificial_ortho.png')
    ortho.save(op)

    ew, eh, names = 560, 760, ('neutral', 'smile', 'curious', 'focused', 'speaking')
    expr = Image.new('RGB', (ew * len(names), eh + top), (16, 18, 24))
    pe = ImageDraw.Draw(expr)
    for i, nm_ in enumerate(names):
        expr.paste(render_panel(posed(nm_), 20.0, 7.0, ew, eh, scale=2160.0, raster=lr), (i * ew, top))
        pe.text((i * ew + 20, 16), nm_.upper(), fill=(230, 234, 242), font=font)
    ep = os.path.join(out_dir, 'face_artificial_expressions.png')
    expr.save(ep)
    return op, ep


def checked():
    """Verify zero crossings across presets and head yaws, C1 neck clearance, stereo baseline and CSG manifolds."""
    from machine.eyes import gimbal, stereo
    from tools.render import clips, solids
    np, pc = fl.np_mod(), {}
    for nm_ in PRESETS:
        p = head_parts(**PRESETS[nm_], plates=True)
        st = [t for k in STATIC_PLATE_KEYS for t in p[k]]
        mv = [t for k in ('left_brow', 'right_brow', 'left_upper_lid', 'right_upper_lid',
                          'left_lower_lid', 'right_lower_lid', 'left_lip_corner', 'right_lip_corner', 'mandible') for t in p[k]]
        ey = [t for k in ('left_camera', 'right_camera', 'left_gimbal', 'right_gimbal', 'gaze_linkages') for t in p[k]]
        pl = st + mv
        pc[nm_] = int(clips.crossed(mv, st) + clips.crossed(p['left_upper_lid'] + p['right_upper_lid'], p['left_lower_lid'] + p['right_lower_lid'])
                      + clips.crossed(p['left_lip_corner'] + p['right_lip_corner'], p['mandible'])
                      + clips.crossed(ey, pl + p['compute_heatsink'])
                      + clips.crossed(p['left_ear_mic'] + p['right_ear_mic'], pl)
                      + clips.crossed(p['compute_heatsink'], pl)
                      + clips.crossed(ey, p['chassis'] + p['inner_liner']))
    bf = head_parts(blink=1.0, plates=True)
    lshut = int(clips.crossed(bf['left_upper_lid'] + bf['right_upper_lid'], bf['left_lower_lid'] + bf['right_lower_lid']))
    nc = mc = 0
    for pr, yw, nd in (('neutral', 0.0, 0.0), ('speaking', 0.0, 0.0), ('neutral', 60.0, 0.0), ('speaking', -60.0, 25.0)):
        q = posed(pr, nod=nd, head_yaw=yw, plates=True, include_neck=True)
        ha = [t for k, ts in q.items() if not k.startswith('mount_') and k != 'neck_skin' for t in ts]
        nc += clips.crossed(ha, q['neck_skin'])
        mc += clips.crossed(ha, [t for k, ts in q.items() if k.startswith('mount_') for t in ts])
    neu = head_parts(**PRESETS['neutral'], plates=True)
    cl, cr = (np.vstack([t for t, _c in neu[k]]).mean(0) for k in ('left_camera', 'right_camera'))
    berr = round(1e3 * abs(float(cl[0] - cr[0]) - float(stereo.BASELINE_M)), 2)
    crown_mm = round(1e3 * float(np.vstack([t for t, _c in neu['cranium_dome']])[:, 1].max()), 2)
    oe = {k: int(solids.printable([1e3 * t for t, _c in neu[k]])['open'])
          for k in ('cranium_frontal', 'cranium_dome', 'cranium_occipital', 'left_temporal', 'left_mastoid',
                    'nasal_bridge', 'left_brow', 'left_maxilla', 'left_orbital_pillar', 'philtrum', 'upper_lip',
                    'compute_heatsink', 'inner_liner')}
    return {'ok': sum(pc.values()) == 0 and lshut == 0 and nc == 0 and mc == 0 and berr < 1.0 and sum(oe.values()) == 0 and abs(float(gimbal.DOWN_DEG) - 15.0) < 1e-6,
            'preset_cross': pc, 'lid_shut_cross': lshut, 'neck_cross': int(nc), 'mount_cross': int(mc),
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
