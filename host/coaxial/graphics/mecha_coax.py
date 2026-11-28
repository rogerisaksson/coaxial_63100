"""Decimated CAD mechatronics for every coaxial humanoid drive, clamp and leaf linkage."""
import functools
import hashlib
import os
from typing import Any

from coaxial.graphics import drums, guards, head_coax, plated, sneaker
from coaxial.graphics.lit import HARD, PAINTED
from coaxial.graphics.livery import GEAR, PAD, SERVO, TUBE, VOID
from coaxial.graphics.panels import OPEN
from coaxial.graphics.shapes import turn_about
from machine.body import figure
from machine.body.figure import CARRY_DEG, FOREARM, HIP_DROP, HIP_HALF, SHANK, THIGH, UPPER_ARM
from machine.body.guards import NEAR_M

_Z0 = (0.0, 0.0, 0.0)
_GRID_M = 0.0068
_NECK_DROP = frozenset(('neck drive', 'neck ring', 'neck plate', 'cheek', 'shoe', 'neck hose',
                        'floor pipe', 'neck leads', 'crown', 'head drive', 'head ring', 'carrier'))
_NECK_LIM = frozenset(('block', 'pipes', 'head hose'))


def _sig():
    """Return content digest over coaxial CAD mount modules and this adapter."""
    rdir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'tools', 'render')
    h = hashlib.sha256()
    pf = ('clamped', 'covered', 'elbow_', 'foot', 'hip_', 'knee_', 'leaves', 'leg_clamped', 'neck_',
          'robot_worn', 'shoulder_', 'trunk_clamped', 'trunk_laid', 'trunk_rods')
    for path in [os.path.abspath(__file__)] + [os.path.join(rdir, n) for n in sorted(os.listdir(rdir))
                                               if n.endswith('.py') and n.startswith(pf)]:
        if os.path.isfile(path):
            h.update(os.path.basename(path).encode('utf-8'))
            with open(path, 'rb') as f:
                h.update(f.read())
    return h.hexdigest()[:16]


def _cluster(tris, grid_m=_GRID_M, force_ink=None):
    """Apply coaxial livery and weld `(tri, rgb)` triangles while preserving sharp creases."""
    from coaxial.model.blocks import numpy as np
    from tools.render import robot_worn as rw
    if not tris:
        return np.zeros((0, 3)), np.zeros((0, 3), int), np.zeros((0, 2)), np.zeros(0, np.uint32)
    painted = rw.apply_livery(tris)
    tri_arr = np.array([t for t, _c in painted], float)
    fn = np.cross(tri_arr[:, 1] - tri_arr[:, 0], tri_arr[:, 2] - tri_arr[:, 0])
    fn /= np.maximum(np.linalg.norm(fn, axis=1, keepdims=True), 1e-12)
    nb = np.repeat(np.rint(fn * 1.1).astype(np.int64), 3, axis=0)
    raw = tri_arr.reshape(-1, 3)
    cols = np.repeat(np.array([c for _t, c in painted], np.int64), 3, axis=0)
    uniq, inv = np.unique(np.c_[np.rint(raw / grid_m).astype(np.int64), cols, nb], axis=0, return_inverse=True)
    corners = np.zeros((len(uniq), 3), float)
    np.add.at(corners, inv, raw)
    corners /= np.maximum(np.bincount(inv, minlength=len(uniq))[:, None], 1)
    idx = inv.reshape(-1, 3)
    idx = idx[(idx[:, 0] != idx[:, 1]) & (idx[:, 1] != idx[:, 2]) & (idx[:, 2] != idx[:, 0])]
    idx = idx[np.linalg.norm(np.cross(corners[idx[:, 1]] - corners[idx[:, 0]],
                                      corners[idx[:, 2]] - corners[idx[:, 0]]), axis=1) > 1e-10]
    used, tri = np.unique(idx, return_inverse=True)
    corners, rgb = corners[used], uniq[used, 3:6]
    if force_ink is not None:
        ink = np.full(len(corners), force_ink, np.uint32)
    else:
        pal = {(255, 48, 200): SERVO, (196, 36, 152): GEAR, (64, 66, 68): TUBE, (24, 24, 26): VOID}
        ink = np.array([pal.get((int(r), int(g), int(b)), PAINTED | HARD | (int(r) << 16) | (int(g) << 8) | int(b))
                        for r, g, b in rgb], np.uint32)
    return corners, tri.reshape(-1, 3), np.zeros((len(corners), 2), float), ink


def _mirror(mesh):
    """Mirror `(corners, triangles, uv, materials)` across the sagittal midline `x = 0`."""
    c, t, u, m = mesh
    return c * (-1.0, 1.0, 1.0), t[:, (0, 2, 1)].copy(), u.copy(), m.copy()


def _rod_local(tris, a, b, x_ref=(1.0, 0.0, 0.0)):
    """Express `tris` in the orthonormal rod frame anchored at `a` with `+y` toward `b`."""
    from coaxial.model.blocks import numpy as np
    a_v, b_v, xr = np.asarray(a, float), np.asarray(b, float), np.asarray(x_ref, float)
    u = (b_v - a_v) / max(float(np.linalg.norm(b_v - a_v)), 1e-9)
    x = xr - float(xr @ u) * u
    x /= max(float(np.linalg.norm(x)), 1e-9)
    rot = np.column_stack([x, u, np.cross(x, u)])
    return [((t - a_v) @ rot, col) for t, col in tris]


def _elbow_frame():
    """Return `(turn_p, shift_p)` mapping `elbow_mount` coordinates to `left_upper_arm` elbow frame."""
    from coaxial.model.blocks import numpy as np
    from machine.body import drives, skeleton
    names = [p for p, *_ in drives.along('left_elbow')]
    xs = [(float(c[2][0]), c[5]) for c in skeleton.cans() if c[0] == 'left_elbow']
    way = 1.0 if xs[names.index('gear')][0] > sum(x for x, _h in xs) / len(xs) else -1.0
    mid = 0.5 * (min(way * x - h for x, h in xs) + max(way * x + h for x, h in xs))
    return np.diag([way, 1.0, -1.0]), np.array([mid, 0.0, 0.0])


def _build_trunk_and_torso():
    """Build decimated trunk cardan, dorsal inverter bay, neck and pectoral sub-assemblies."""
    from coaxial.model.blocks import numpy as np
    from tools.render import neck_hosed as nh, neck_mount as nm, robot_built as rb, robot_worn as rw
    from tools.render import shoulder_mount as sm, shoulder_rolled as sr
    from tools.render import trunk_clamped as tc, trunk_laid as tl, trunk_rods as tr
    seg_at = {s[0]: np.asarray(s[3], float) for s in figure.SEGMENTS}
    links_tr = tc.links()
    pair_l = [(t, c) for t, c in links_tr['chest']['pair still'] if float(t[:, 0].mean()) >= 0.0]
    pair_r = [(t, c) for t, c in links_tr['chest']['pair still'] if float(t[:, 0].mean()) < 0.0]
    chest_rest = ([t for k, v in links_tr['chest'].items() if k != 'pair still' for t in v]
                  + [t for v in links_tr['left crank'].values() for t in v]
                  + [t for v in links_tr['right crank'].values() for t in v])
    rods_tr = tr.rods(0.0, 0.0)
    cranks_tr: Any = tl.cranked(0.0, 0.0)
    rod_l = _rod_local([t for (r, _p), v in rods_tr.items() if r == 'left rod' for t in v],
                       cranks_tr[0][2], cranks_tr[0][1])
    nm_still, nh_all = nm.still(low=tc.seat()), nh.posed()
    nm_all = dict(nm.posed(low=tc.seat()), **nh_all)
    bay_trim = [(t, c) for n, ts in nm_all.items() if not n.endswith(' shoulder') and n not in _NECK_DROP
                for t, c in ts if n not in _NECK_LIM or float(t[:, 1].max()) < rw.NECK_Y[0]]
    bay_bare = ([t for k, v in nm_still.items() if k != 'neck drive' and not k.endswith(' shoulder') for t in v]
                + [t for k, v in nh_all.items() if k in ('bay leads', 'shoulder bay', 'shoulder bay guard',
                                                         'shoulder bay lid', 'pipes') for t in v])
    nod_all, turn_all = nm.nodding(), nm.turning()
    n_off, h_off = seg_at['neck'], seg_at['neck'] + seg_at['head']
    sm_p, turn_ua, ua_off = sm.posed(0.0, 0.0), turn_about('z', 6.0), seg_at['left_upper_arm']
    clevis = rb._clip_x_min([((t * (-1.0, 1.0, -1.0)) @ turn_ua.T + ua_off, c)
                             for k in ('crossbar', 'clevis', 'span') for t, c in sm_p[k]], 0.0415)
    pec_l = sr.chest(0.0, False)['pectoral'] + clevis
    st1_yoke = [(t * (-1.0, 1.0, -1.0), c) for k in ('yoke', 'arbor', 'crossbar foot', 'studs', 'hub plate',
                                                      'output ring', 'block', 'coil', 'bus') for t, c in sm_p[k]]
    return {
        'trunk_pelvis': _cluster([t for v in links_tr['pelvis'].values() for t in v]),
        'trunk_cross': _cluster([t for v in links_tr['cross'].values() for t in v]),
        'drive_spine': _cluster(pair_l, force_ink=SERVO),
        'drive_spine_roll': _cluster(pair_r, force_ink=SERVO),
        'trunk_chest': _cluster(chest_rest),
        'rod_spine': _cluster(rod_l, 0.0040),
        'drive_neck': _cluster(nm_still['neck drive'], force_ink=SERVO),
        'torso_bay_trim': _cluster(bay_trim),
        'torso_bay_bare': _cluster(bay_bare),
        'neck_nod': _cluster([(t - n_off, c) for k, v in nod_all.items() if k != 'head drive' for t, c in v], 0.0040),
        'drive_head': _cluster([(t - n_off, c) for t, c in nod_all['head drive']], 0.0040, force_ink=SERVO),
        'head_turn': _cluster([(t - h_off, c) for v in turn_all.values() for t, c in v], 0.0040),
        'pectoral_l': _cluster(pec_l),
        'stage_ua_l': _cluster(st1_yoke),
    }


def _build_arm_and_leg():
    """Build decimated left arm, coaxial elbow, 3-axis hip, twin-calf knee, leaves and keel foot."""
    from coaxial.model.blocks import numpy as np
    from machine.body import skeleton
    from tools.render import elbow_mount as em, hip_clamped as hc, knee_clamped as kc, knee_loads as kl
    from tools.render import leaves, leg_clamped as lc, mounting as m, shoulder_mount as sm, shoulder_rolled as sr
    sm_turn, (turn_p, shift_p) = sm.turning(), _elbow_frame()
    still_parts, _gear, arm_in, arm_out = em._stack()
    over = max(r for *_x, r in still_parts) + 0.002
    face_t, face_s = over + em.SADDLE_M + em.SEAT_M, -(over + em.ROTOR_AIR + em.ROTOR_M + em.SEAT_M) - em.FLANGE_M
    at_x = float(shift_p[0] * turn_p[0, 0])
    ua_tube = m._prism(m._circle(arm_in, 32, (at_x, 0.0)), face_t + em.FLANGE_M - UPPER_ARM,
                       -sr.laid()['down'] - em.FLANGE_M, em.CARBON)
    sh_drv = [(t * (-1.0, 1.0, -1.0), c) for t, c in sm_turn['drive']]
    sh_rest = [(t * (-1.0, 1.0, -1.0), c) for k, v in sm_turn.items() if k != 'drive' for t, c in v] + ua_tube
    det_p = int(np.linalg.det(turn_p))
    placed_em = lambda ts: [(((t + shift_p) @ turn_p)[::det_p], c) for t, c in ts]
    em_all, em_cab, em_still, em_turn = em.bodies(flex=0.0, hosed=False)
    el_drv = [tc for tc in placed_em(em_still) if tc[1] in (em.SERVO, em.GEAR, em.BOARD)]
    el_sta = [tc for tc in placed_em(em_still) if tc[1] not in (em.SERVO, em.GEAR, em.BOARD)] + placed_em(em_cab)
    el_rot = placed_em([tc for tc in em_all[len(em_still):len(em_all) - len(em_turn)]] + em_turn)
    r_carry = turn_about('z', CARRY_DEG)
    p_top, p_bot = r_carry.T @ np.array([at_x, face_s, 0.0]), np.array([0.0, -FOREARM + 0.004, 0.0])
    u_fa = (p_top - p_bot) / np.linalg.norm(p_top - p_bot)
    x_fa = np.cross(u_fa, (0.0, 0.0, 1.0))
    x_fa /= np.linalg.norm(x_fa)
    rot_fa = np.column_stack([x_fa, u_fa, np.cross(x_fa, u_fa)])
    tube_fa = [(t @ rot_fa.T + p_bot, c) for t, c in m._prism(
        m._circle(arm_out, 32), 0.0, float(np.linalg.norm(p_top - p_bot)), em.CARBON)]
    tube_fa += m._prism(m._circle(arm_out + 0.0022, 32), -FOREARM + 0.002, -FOREARM + 0.014, em.CARBON)
    links_h, (_kp, _kc_cab, kc_still, kc_turn), sh_all = hc.links(), kc.bodies(flex=0.0, ahead=kl.stated()), lc.shank()
    foot_keys = {'foot', 'plate', 'soles', 'heel bar'}
    shank_fixed = ([t for k, v in kc_turn.items() if k not in ('pitch drive', 'roll drive') for t in v]
                   + [t for k, v in sh_all.items() if k not in foot_keys and not k.startswith('leaf ') for t in v])
    foot_bare = [(t + (0.0, SHANK, 0.0), c) for k in ('foot', 'plate', 'soles') for t, c in sh_all[k]]
    heel_tris = [(t + (0.0, SHANK, 0.0), c) for t, c in sh_all.get('heel bar', ())]
    if not heel_tris:
        balls = [skeleton._rod('left_', k)[4] for k in leaves.KINDS]
        (_bx, yh, zh), far = balls[0], lc.ARM_X - lc.AIR
        from tools.render import clamped as cl, foot as ft_mod, knee_mount as km
        horn = leaves.hull(leaves.ring((yh, zh), lc.HORN_M) + [(-SHANK + s * lc.BOSS_M, z) for s in (1.0, -1.0)
                                                               for z in (-skeleton.CROSS[2], ft_mod.PILLAR_M - skeleton.CROSS[2])])
        heel_raw = (em._rounded(cl.filleted(horn), -lc.HORN_X, lc.HORN_X, lc.STEEL, axis=0, r=0.001, steps=1)
                    + km._rod((-far, yh, zh), (far, yh, zh), leaves.SHAFT_M, lc.STEEL))
        heel_tris = [(t + (0.0, SHANK, 0.0), c) for t, c in heel_raw]
    leaf_a = _rod_local(sh_all['leaf ankle'], skeleton._rod('left_', 'ankle')[4], skeleton._rod('left_', 'ankle')[2])
    leaf_r = _rod_local(sh_all['leaf ankle_roll'], skeleton._rod('left_', 'ankle_roll')[4],
                        skeleton._rod('left_', 'ankle_roll')[2])
    return {
        'drive_shoulder_l': _cluster(sh_drv, force_ink=SERVO),
        'shoulder_l': _cluster(sh_rest),
        'drive_elbow_l': _cluster(el_drv, force_ink=SERVO),
        'elbow_stator_l': _cluster(el_sta),
        'elbow_rotor_l': _cluster(el_rot),
        'forearm_l': _cluster(tube_fa),
        'drive_hip_yaw_l': _cluster(links_h['pelvis']['yaw still'], force_ink=SERVO),
        'hip_pelvis_l': _cluster([t for k, v in links_h['pelvis'].items() if k != 'yaw still' for t in v]),
        'drive_hip_roll_l': _cluster(links_h['yaw']['roll still'], force_ink=SERVO),
        'hip_yaw_l': _cluster([t for k, v in links_h['yaw'].items() if k != 'roll still' for t in v]),
        'drive_hip_l': _cluster(links_h['roll']['pitch still'], force_ink=SERVO),
        'hip_roll_l': _cluster([t for k, v in links_h['roll'].items() if k != 'pitch still' for t in v]),
        'thigh_l': _cluster([t for v in links_h['thigh'].values() for t in v] + hc.slung()),
        'drive_knee_l': _cluster(kc_still['knee drive'], force_ink=SERVO),
        'knee_stator_l': _cluster([t for k, v in kc_still.items() if k != 'knee drive' for t in v]),
        'drive_ankle_l': _cluster(kc_turn['pitch drive'], force_ink=SERVO),
        'drive_ankle_roll_l': _cluster(kc_turn['roll drive'], force_ink=SERVO),
        'shank_l': _cluster(shank_fixed),
        'leaf_ankle_l': _cluster(leaf_a, 0.0040),
        'leaf_ankle_roll_l': _cluster(leaf_r, 0.0040),
        'foot_bare_l': _cluster(foot_bare + heel_tris, 0.0045),
    }


@functools.lru_cache(maxsize=1)
def meshes():
    """Return `{key: (corners, triangles, uv, materials)}` for all Robot 2.0 CAD links, cached on disk."""
    from coaxial.model.blocks import numpy as np
    path = os.path.join(head_coax._CACHE_DIR, 'mecha_coax_%s.npz' % _sig())
    if os.path.isfile(path):
        with np.load(path, allow_pickle=False) as data:
            return {k[:-2]: (data[k], data[k[:-2] + '_t'], data[k[:-2] + '_u'], data[k[:-2] + '_m'])
                    for k in data.files if k.endswith('_c')}
    got = {**_build_trunk_and_torso(), **_build_arm_and_leg()}
    os.makedirs(head_coax._CACHE_DIR, exist_ok=True)
    arrays: dict[str, Any] = {k + suf: arr for k, vals in got.items() for suf, arr in zip(('_c', '_t', '_u', '_m'), vals)}
    tmp = '%s.%d.npz' % (path, os.getpid())
    np.savez(tmp, **arrays)
    os.replace(tmp, path)
    return got


@functools.lru_cache(maxsize=1)
def _anchors():
    """Return `{rod_part_name: (seg_a, pt_a, seg_b, pt_b)}` for dynamic PAHT-CF leaf linkages."""
    from coaxial.model.blocks import numpy as np
    from machine.body import skeleton
    from tools.render import trunk_laid as tl
    j_tr = np.asarray(tl.joint()[0], float)
    cranks: Any = tl.cranked(0.0, 0.0)
    off_t = np.array([0.0, tl.RAISED_M, 0.0])
    out = {
        'coaxrod_spine': ('pelvis', j_tr + cranks[0][2], 'torso', off_t + cranks[0][1]),
        'coaxrod_spine_roll': ('pelvis', j_tr + cranks[1][2], 'torso', off_t + cranks[1][1]),
    }
    for side, sgn in (('left', 1.0), ('right', -1.0)):
        for kind in ('ankle', 'ankle_roll'):
            _seg, _hub, pin, _bend, ball = skeleton._rod('left_', kind)
            b_foot = np.array([sgn * ball[0], ball[1] + SHANK, ball[2]])
            p_shank = np.array([sgn * pin[0], pin[1], pin[2]])
            out['coaxrod_%s_%s' % (side, kind)] = (side + '_foot', b_foot, side + '_shank', p_shank)
    return out


def posed(parts_, placed, frames):
    """Fill orthonormal `(turn, spot)` frames for every `'coaxrod_*'` leaf-spring cartridge."""
    from coaxial.model.blocks import numpy as np
    anchors = _anchors()
    for i, (name, parent, *_rest) in enumerate(parts_):
        if parent != '*' or name not in anchors:
            continue
        seg_a, pt_a, seg_b, pt_b = anchors[name]
        ta, sa = placed[seg_a]
        tb, sb = placed[seg_b]
        a, b = sa + ta @ pt_a, sb + tb @ pt_b
        u = (b - a) / max(float(np.linalg.norm(b - a)), 1e-9)
        xr = ta[:, 0]
        x = xr - float(xr @ u) * u
        x /= max(float(np.linalg.norm(x)), 1e-9)
        frames[i] = (np.column_stack([x, u, np.cross(x, u)]), a)


def _hands_and_shoes(bare, liked):
    """Return hand and sneaker part tuples for plated or dressed coaxial views."""
    from coaxial.model.blocks import numpy as np
    if bare:
        return []
    out: list[Any] = []
    liked_list: list[Any] = list(liked or ())
    body = {p[1]: p for p in liked_list if p[0].startswith('form_')}
    hands: list[Any] = [body[s] for s in ('left_hand', 'right_hand') if s in body]
    if not hands:
        hands = [p for p in liked_list if p[0] in ('like_left_hand', 'like_right_hand')]
    for name, parent, *rest in hands:
        joints, offset = (rest[0], rest[1]) if len(rest) == 3 else ((), rest[0])
        c, t, u, m = rest[-1]
        t = t[~(m[t] == OPEN).any(axis=1)]
        used, inv = np.unique(t, return_inverse=True)
        out.append((name, parent, joints, offset, 0.0,
                    (c[used], inv.reshape(-1, 3), u[used], np.full(len(used), TUBE, np.uint32))))
    for side in ('left', 'right'):
        for seg, (c, t, u, _m) in sneaker.feet(side).items():
            out.append(('shell_' + seg, seg, (), _Z0, 0.0, (c, t, u, np.full(len(c), TUBE, np.uint32))))
        for sname, par, off, ms in sneaker.soles(side):
            out.append((sname, par, (), off, 0.0, ms))
    return out


def _guards_and_seats(out, liked):
    """Return buttock honeycomb seat nets and coaxial-cleared honeycomb limb and torso guards."""
    from coaxial.graphics import seat
    from coaxial.model.blocks import numpy as np
    placed_r, frames_r = plated._rested(out, pose_fn=posed)
    rested = [(p[0], p[1], p[5][0] @ fr[0].T + fr[1], p[5][1]) for p, fr in zip(out, frames_r)
              if fr is not None and len(p[5][0])]
    pelvis_r = [(c, t) for _n, par, c, t in rested if par == 'pelvis']
    extra: list[Any] = []
    for sname, sgn in (('left', 1.0), ('right', -1.0)):
        j_spot = placed_r[sname + '_hip_yaw'][1]
        drv = ([(n, c, t) for n, par, c, t in rested if par == sname + '_hip_yaw'], j_spot)
        extra.append((sname + '_seat', sname + '_hip_yaw', (), tuple(-j_spot), 0.0, seat.plate(sgn, drv, pelvis_r)))
        rc, rt, ru, rm = seat.rim(sgn, drv, pelvis_r)
        extra.append(('rim_' + sname + '_seat', sname + '_hip_yaw', (), tuple(-j_spot), 0.0,
                      (rc, rt, ru, rm if (rm >= PAINTED).all() else np.full(len(rc), PAD))))
    body = {p[1]: p[-1][0] for p in (liked or ()) if p[0].startswith('form_')
            and (p[1] == 'torso' or p[1].partition('_')[2] in guards.LIMB)}
    if body:
        hard = [c for n, _par, c, _t in rested if n.startswith(('drive_', 'coax_', 'coaxrod_', 'stage_'))]
        surfaces = {}
        for seg, corners in body.items():
            turn, spot = placed_r[seg]
            low, high = float(corners[:, 1].min()) - plated.DRUM_M, float(corners[:, 1].max()) + plated.DRUM_M
            by = [at[(at[:, 1] > low) & (at[:, 1] < high) & (np.hypot(at[:, 0], at[:, 2]) < NEAR_M)]
                  for at in ((w - spot) @ turn for w in hard)]
            surfaces[seg] = (corners, np.vstack([np.zeros((0, 3)), *by]))
        extra += [(name, parent, (), offset, 0.0, mesh) for name, parent, offset, mesh in guards.laid(surfaces)]
    return extra


def parts(bare=False, dressed=False, liked: Any = None):
    """Return `[(name, parent, joints, offset, rest, mesh)]` of the full Robot 2.0 build."""
    from coaxial.model.blocks import numpy as np
    from tools.render import trunk_laid as tl
    m = meshes()
    empty: Any = (np.zeros((0, 3)), np.zeros((0, 3), int), np.zeros((0, 2)), np.zeros(0, np.uint32))
    seg_meshes: dict[str, Any] = {s[0]: empty for s in figure.SEGMENTS}
    if bare:
        seg_meshes['head'] = head_coax.bare_mesh()
        seg_meshes['left_foot'] = m['foot_bare_l']
        seg_meshes['right_foot'] = _mirror(m['foot_bare_l'])
    out: list[Any] = [(s[0], s[1], s[2], s[3], s[4], seg_meshes[s[0]]) for s in figure.SEGMENTS]
    out += [(n, p, j, at, 0.0, empty) for n, p, j, at, _ms in drums.stages()]
    j_tr, off_tr = tuple(float(v) for v in tl.joint()[0]), (0.0, tl.RAISED_M, 0.0)
    out += [
        ('coax_trunk_pelvis', 'pelvis', (), j_tr, 0.0, m['trunk_pelvis']),
        ('coax_trunk_cross', 'spine', (), off_tr, 0.0, m['trunk_cross']),
        ('drive_spine', 'torso', (), off_tr, 0.0, m['drive_spine']),
        ('drive_spine_roll', 'torso', (), off_tr, 0.0, m['drive_spine_roll']),
        ('coax_trunk_chest', 'torso', (), off_tr, 0.0, m['trunk_chest']),
        ('coaxrod_spine', '*', (), _Z0, 0.0, m['rod_spine']),
        ('coaxrod_spine_roll', '*', (), _Z0, 0.0, _mirror(m['rod_spine'])),
        ('coax_torso_bay', 'torso', (), _Z0, 0.0, m['torso_bay_bare'] if bare else m['torso_bay_trim']),
        ('drive_neck', 'torso', (), _Z0, 0.0, m['drive_neck'] if bare else empty),
    ]
    if bare:
        out += [('coax_neck_nod', 'neck', (), _Z0, 0.0, m['neck_nod']),
                ('drive_head', 'neck', (), _Z0, 0.0, m['drive_head']),
                ('coax_head_turn', 'head', (), _Z0, 0.0, m['head_turn'])]
    else:
        at = {s: np.array(p) for s, (p, _t) in figure.frames({}, _Z0, figure.IDENT).items()}
        out += head_coax.terminal_parts(at['torso'], at['neck'], at['head'], dressed=bool(dressed and liked))
    seg_map = {s[0]: s for s in figure.SEGMENTS}
    for side, sgn in (('left', 1.0), ('right', -1.0)):
        pick = (lambda k, sgn=sgn: m[k] if sgn > 0.0 else _mirror(m[k]))
        ua_seg, hip_at = seg_map[side + '_upper_arm'], (HIP_HALF * sgn, -HIP_DROP, 0.0)
        el_at, kn_at = (0.0, -UPPER_ARM, 0.0), (0.0, -THIGH, 0.0)
        ua, fa, hy, hr, th, sh = (side + s for s in ('_upper_arm', '_forearm', '_hip_yaw', '_hip_roll', '_thigh', '_shank'))
        out += [('stage_' + ua, 'torso', (), ua_seg[3], ua_seg[4], pick('stage_ua_l')),
                ('coax_' + side + '_elbow', ua, ((side + '_elbow', 'x', -1),), el_at, 0.0, pick('elbow_rotor_l'))]
        for nm, par, off, key in (
            ('coax_%s_pectoral', 'torso', _Z0, 'pectoral_l'), ('drive_%s_shoulder', ua, _Z0, 'drive_shoulder_l'),
            ('coax_%s_shoulder', ua, _Z0, 'shoulder_l'), ('drive_%s_elbow', ua, el_at, 'drive_elbow_l'),
            ('coax_%s_elbow_stator', ua, el_at, 'elbow_stator_l'), ('coax_%s_forearm', fa, _Z0, 'forearm_l'),
            ('drive_%s_hip_yaw', 'pelvis', hip_at, 'drive_hip_yaw_l'), ('coax_%s_hip_pelvis', 'pelvis', hip_at, 'hip_pelvis_l'),
            ('drive_%s_hip_roll', hy, _Z0, 'drive_hip_roll_l'), ('coax_%s_hip_yaw', hy, _Z0, 'hip_yaw_l'),
            ('drive_%s_hip', hr, _Z0, 'drive_hip_l'), ('coax_%s_hip_roll', hr, _Z0, 'hip_roll_l'),
            ('coax_%s_thigh', th, _Z0, 'thigh_l'), ('drive_%s_knee', th, kn_at, 'drive_knee_l'),
            ('coax_%s_knee_stator', th, kn_at, 'knee_stator_l'), ('drive_%s_ankle', sh, _Z0, 'drive_ankle_l'),
            ('drive_%s_ankle_roll', sh, _Z0, 'drive_ankle_roll_l'), ('coax_%s_shank', sh, _Z0, 'shank_l'),
            ('coaxrod_%s_ankle', '*', _Z0, 'leaf_ankle_l'), ('coaxrod_%s_ankle_roll', '*', _Z0, 'leaf_ankle_roll_l'),
        ):
            out.append((nm % side, par, (), off, 0.0, pick(key)))
    out += _hands_and_shoes(bare, liked)
    if not bare:
        out += _guards_and_seats(out, liked)
    if dressed and liked and not bare:
        return plated.dressed(out, liked, pose_fn=posed)
    return out
