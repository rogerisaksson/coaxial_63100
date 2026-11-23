"""The build whole: every mount on the figure, bare or dressed, with head, face and hands.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/robot_built.py OUT_DIR [--px N] [--dressed]

The sketches lie each in a frame of its own: the leg's and the hip's in the knee's (`leg_worn`, `leg_shaped`), the arm's
and the shoulder's in the elbow's (`arm_worn`), the neck's, the head's and the bay's in the torso's (`neck_mount`,
`neck_hosed`), the trunk's in its joint's (`trunk_clamped`, `trunk_shaped`, `trunk_worn`). The page's body (`gynoid.body`
with `likeness.FOUND` when present) supplies the head, face, hair, 5-fingered carbon hands, and posed garments (`tee`,
`sleeves`, `jeans`, `shoes`) over the mechanism. `whole(roll, dressed, turn, flex)` draws all regions bare or clothed;
`clasped()` verifies front and back hand-clasp reach and clearance.
"""
import argparse
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import mounting as m  # noqa: E402

#: Legacy fallback colour when `likeness.FOUND` is absent, and raw segment names for hands and head.
GHOST = (62, 84, 116)
UNDRAWN = ('left_hand', 'left_fingers', 'right_hand', 'right_fingers')
HEAD_LIKED = ('like_head', 'like_hair', 'like_fall')
HEAD_SHELL = ('shell_head', 'shell_jaw', 'shell_nose', 'shell_lips', 'shell_left_ear', 'shell_right_ear',
              'shell_left_eye', 'shell_right_eye')
CLOTH = ('like_torso', 'like_left_upper_arm', 'like_right_upper_arm', 'like_pelvis', 'like_left_thigh',
         'like_right_thigh', 'like_left_shank', 'like_right_shank')
SHOES = ('shell_left_foot', 'shell_left_toes', 'shell_right_foot', 'shell_right_toes')
#: Hand-clasp poses (Stage-2 shoulder pitch `turn`, Stage-1 pectoral `roll`, elbow `flex`, deg): front & behind.
CLASP_FRONT = (64.0, -25.75, 0.0)
CLASP_BACK = (-38.0, -18.0, 8.0)
#: The whole's views: (name, yaw deg, pitch deg); the regions drawn close: {name: (segments, half width m, views)}.
VIEWS = (('ahead', 20.0, 6.0), ('side', 90.0, 4.0), ('behind', 200.0, 6.0))
CLOSE = {'hip': (('left_thigh', 'pelvis'), 0.20, (('ahead', 35.0, 10.0), ('behind', 215.0, 10.0))),
         'knee': (('left_shank', 'left_shank'), 0.20, (('ahead', 35.0, 10.0), ('behind', 215.0, 10.0))),
         'foot': (('left_foot', 'left_foot'), 0.18, (('side', 90.0, 8.0), ('behind', 215.0, 10.0))),
         'shoulder': (('left_upper_arm', 'left_upper_arm'), 0.18, (('ahead', 35.0, 10.0), ('behind', 215.0, 10.0))),
         'elbow': (('left_forearm', 'left_forearm'), 0.16, (('ahead', 35.0, 10.0), ('behind', 215.0, 10.0))),
         'neck': (('neck', 'neck'), 0.20, (('ahead', 25.0, 10.0), ('behind', 205.0, 10.0))),
         'trunk': (('torso', 'neck'), 0.26, (('ahead', 35.0, 10.0), ('side', 90.0, 4.0), ('behind', 215.0, 10.0)))}


@functools.lru_cache(maxsize=None)
def _figure(dressed=False):
    """(the page's plated body with likeness when present, part names, rest frames, rest corners)."""
    import numpy as np
    from coaxial.graphics import gynoid, likeness
    body = gynoid.body(dressed, None, likeness.FOUND, True)
    names = [part[0] for part in body.parts]
    root = ((0.0, 0.0, 0.0), np.eye(3))
    frames = body._frames({}, root)
    pos, _n = body.pose({}, root=root, _frames=frames)
    return body, names, frames, pos


def _put(tris, segment, pivot):
    """`tris` of a sketch in `segment`'s frame at `pivot`'s origin, laid where the figure stands them."""
    _body, names, frames, _pos = _figure(False)
    turn, spot = frames[names.index(segment)][0], frames[names.index(pivot)][1]
    return [(tri @ turn.T + spot, colour) for tri, colour in tris]


def _mirrored(tris):
    """`tris` on the robot's other side: mirrored in its middle, their faces kept outward."""
    return [((tri * (-1.0, 1.0, 1.0))[::-1], colour) for tri, colour in tris]


def _extract(body, names, pos, wanted):
    """[(tri, rgb)] in root frame from `body` for every part name in `wanted` found in `names`."""
    import numpy as np
    from tools.render import leg_worn as lw
    owner = np.repeat(np.arange(len(names)), [len(part[5][1]) for part in body.parts])
    index = body.index.astype(int)
    return [t for n in wanted if n in names
            for t in lw._tris(pos, index[owner == names.index(n)], body.materials)]


def _pose_arm_local(tris, turn=0.0, roll=0.0):
    """Transform `tris` in left `arm_worn` elbow frame by Stage-2 pitch `turn` deg and Stage-1 `roll` deg."""
    import numpy as np
    from tools.render import arm_worn as aw, knee_mount as km, shoulder_rolled as sr
    if not (turn or roll):
        return tris
    _p, shoulder, _w = aw.laid(False, ())
    out = tris
    if turn:
        rx = km._rx(-turn)
        out = [((t - shoulder) @ rx.T + shoulder, col) for t, col in out]
    if roll:
        piv_a = shoulder + np.array([-sr.X_ABD, 0.0, 0.0])
        c_r, s_r = math.cos(math.radians(roll)), math.sin(math.radians(roll))
        rz = np.array([[c_r, -s_r, 0.0], [s_r, c_r, 0.0], [0.0, 0.0, 1.0]])
        out = [((t - piv_a) @ rz.T + piv_a, col) for t, col in out]
    return out


def _left_hand_local(flex=0.0, turn=0.0, roll=0.0):
    """[(tri, rgb)] of the left 5-fingered hand coaxial on `forearm tube` in `arm_worn` elbow frame, posed."""
    import numpy as np
    from tools.render import arm_worn as aw, knee_mount as km
    body, names, frames, pos = _figure(False)
    part_names = ('form_left_hand',) if 'form_left_hand' in names else ('left_hand', 'left_fingers')
    raw = _extract(body, names, pos, part_names)
    turn_fa, spot_fa = frames[names.index('left_forearm')]
    turn_p, shift_p, bend_p = aw.placing()
    off = np.array([float(shift_p[0] * turn_p[0, 0]), 0.010, 0.0])
    coax = [((t - spot_fa) @ turn_fa + off, col) for t, col in raw]
    bent = km._turn(coax, bend_p * flex) if flex else coax
    return _pose_arm_local(bent, turn, roll)


def _head_and_hands(roll=0.0, turn=0.0, flex=0.0):
    """{'head', 'hands': [(tri, rgb)]} in root frame with complete face, hair and coaxial posed hands."""
    from coaxial.graphics import likeness
    body, names, _frames, pos = _figure(False)
    head = _extract(body, names, pos, HEAD_LIKED if likeness.FOUND else HEAD_SHELL)
    left_hand = _put(_left_hand_local(flex, turn, roll), 'left_upper_arm', 'left_forearm')
    return {'head': head, 'hands': left_hand + _mirrored(left_hand)}


def _smooth_cloth(body_d, names_d, pos_d, norm_d, obs_tris):
    """[(tri, rgb)] of CLOTH in `body_d`, gently offset outward along normals past `obs_tris` without steps."""
    import numpy as np
    from scipy.spatial import KDTree
    from tools.render import leg_worn as lw
    pts = np.vstack([t for t, _c in obs_tris])
    _, idx = np.unique(np.floor(pts / 0.006).astype(np.int64), axis=0, return_index=True)
    tree = KDTree(pts[idx])
    out = []
    for name in CLOTH:
        if name not in names_d:
            continue
        k = names_d.index(name)
        lo, hi = body_d.spans[k]
        v = pos_d[lo:hi].copy()
        n = norm_d[lo:hi]
        n = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
        tris = body_d.parts[k][5][1]
        dists, hits = tree.query(v, k=6, distance_upper_bound=0.045)
        valid = np.isfinite(dists)
        safe = np.where(valid, hits, 0)
        delta = pts[idx[safe]] - v[:, None, :]
        proj = (delta * n[:, None, :]).sum(axis=-1)
        tang = np.linalg.norm(delta - proj[..., None] * n[:, None, :], axis=-1)
        need = np.where(valid & (tang < 0.018), np.clip(proj + 0.005, 0.0, 0.030), 0.0).max(axis=1)
        e0 = np.concatenate([tris[:, 0], tris[:, 1], tris[:, 2], tris[:, 1], tris[:, 2], tris[:, 0]])
        e1 = np.concatenate([tris[:, 1], tris[:, 2], tris[:, 0], tris[:, 0], tris[:, 1], tris[:, 2]])
        d = need.copy()
        for _ in range(6):
            acc, cnt = np.zeros(len(v)), np.zeros(len(v))
            np.add.at(acc, e0, d[e1])
            np.add.at(cnt, e0, 1.0)
            nbr = acc / np.maximum(cnt, 1.0)
            d = np.maximum(need, 0.30 * d + 0.70 * nbr)
        v += d[:, None] * n
        mats = body_d.materials[lo:hi]
        out += [(v[t], lw._rgb(mats[t[0]])) for t in tris]
    return out


def _posed_arm_cad(flex, turn, roll, bare_guards=True):
    """[(tri, rgb)] of the left arm CAD in `arm_worn` elbow frame, posed at `(flex, turn, roll)`."""
    from tools.render import arm_worn as aw, knee_mount as km
    turn_p, _s, bend_p = aw.placing()
    arm_d = aw.arm(flex, roll, turn)
    if bare_guards:
        gf = km._turn(aw._guarded()['left_forearm'], bend_p * flex) if flex else aw._guarded()['left_forearm']
        ga = _pose_arm_local(aw._guarded()['left_upper_arm'] + gf, turn, roll)
        return [t for part in arm_d.values() for t in part] + ga
    low = [t for k in ('elbow', 'hose', 'forearm tube') for t in arm_d[k]]
    low += [(t, c) for t, c in arm_d['upper arm tube'] if t[:, 1].max() < -0.05]
    gf = km._turn(aw._guarded()['left_forearm'], bend_p * flex) if flex else aw._guarded()['left_forearm']
    return low + _pose_arm_local(gf, turn, roll), [t for part in arm_d.values() for t in part] + _pose_arm_local(
        aw._guarded()['left_upper_arm'] + gf, turn, roll)


@functools.lru_cache(maxsize=None)
def whole(roll=0.0, dressed=False, turn=0.0, flex=0.0):
    """{name: tris} of the build in root frame (`pelvis`), standing (`turn` deg shoulder pitch, `roll` deg pectoral
    abduction, `flex` deg elbow bend), bare or `dressed`, complete with head, face, hair and coaxial hands."""
    import numpy as np
    from coaxial.graphics import likeness
    from tools.render import leg_clamped as lc, leg_shaped as ls, leg_worn as lw
    from tools.render import neck_hosed as nh, neck_mount as nm, shoulder_rolled as sr
    from tools.render import trunk_clamped as tc, trunk_laid as tl, trunk_shaped as ts, trunk_worn as tw
    _b, _nb, _fb, _pb = _figure(False)
    foot_y = float(_fb[_nb.index('left_foot')][1][1])
    leg = _put(ls.posed(lc.stated(), 0.0) + [t for part in lw.guards(lc.stated()).values() for t in part],
               'left_thigh', 'left_shank')
    pec = sr.chest(roll, False)['pectoral'] + sr.chest(roll, True)['pectoral']
    neck = [t for name, part in dict(nm.posed(low=tc.seat()), **nh.posed()).items() if not name.endswith(' shoulder')
            for t in part]
    at = np.asarray(tl.joint()[0], float)
    trunk_rel = [t for (link, name), part in ts.posed().items() if link != 'hips' and name != 'tube' for t in part]
    trunk = [(tri + at, colour) for tri, colour in trunk_rel]
    guards_t = [(tri + at, colour) for tri, colour in tw.guards()]
    if dressed and likeness.FOUND:
        body_d, names_d, _fd, _pd = _figure(True)
        root = ((0.0, 0.0, 0.0), np.eye(3))
        angles = {'arms_out': roll, 'left_shoulder': turn, 'right_shoulder': turn,
                  'left_elbow': flex, 'right_elbow': flex}
        pos_d, norm_d = body_d.pose(angles, root=root)
        arm_low_l, arm_full_l = _posed_arm_cad(flex, turn, roll, bare_guards=False)
        arm_full_root = _put(arm_full_l, 'left_upper_arm', 'left_forearm')
        obs = (_put(pec, 'torso', 'torso') + _put(trunk + guards_t, 'pelvis', 'pelvis')
               + leg + _mirrored(leg) + arm_full_root + _mirrored(arm_full_root))
        cloth = _smooth_cloth(body_d, names_d, pos_d, norm_d, obs)
        shoes = _extract(body_d, names_d, pos_d, SHOES)
        leg_low = [t for t in leg if t[0][:, 1].min() < foot_y + 0.025]
        neck_up = [(t, c) for t, c in neck if t[:, 1].max() >= 0.355]
        arm = _put(arm_low_l, 'left_upper_arm', 'left_forearm')
        out = {'left arm': arm, 'right arm': _mirrored(arm), 'left ankle': leg_low, 'right ankle': _mirrored(leg_low),
               'shoes': shoes, 'garments': cloth, 'neck': _put(neck_up, 'torso', 'torso')}
    else:
        arm = _put(_posed_arm_cad(flex, turn, roll, bare_guards=True), 'left_upper_arm', 'left_forearm')
        out = {'left leg': leg, 'right leg': _mirrored(leg), 'left arm': arm, 'right arm': _mirrored(arm),
               'pectoral': _put(pec, 'torso', 'torso'), 'neck': _put(neck, 'torso', 'torso'),
               'trunk': _put(trunk, 'pelvis', 'pelvis'), 'trunk guards': _put(guards_t, 'pelvis', 'pelvis')}
    out.update(_head_and_hands(roll, turn, flex))
    return out


def clasped():
    """{'front_gap', 'back_gap', 'front_lap', 'back_lap', 'front_z', 'back_z', 'in_travel', 'crossing'}: verify both
    front and back hand-clasp poses stay within shoulder/elbow travel, bring left & right hands to touch at x = 0, and
    cross no body, pectoral or contralateral arm triangles."""
    import numpy as np
    from machine.body import drives
    from machine.body.figure import THIGH
    from tools.render import arm_worn as aw, clips, elbow_mount as em, hip_clamped as hc, knee_mount as km
    from tools.render import shoulder_laid as sl, shoulder_mount as sm, shoulder_rolled as sr
    from tools.render import trunk_clamped as tc, trunk_laid as tl
    body, names, frames, pos = _figure(False)
    wanted_body = [n for n in names if n in ('guard_torso', 'guard_left_thigh', 'guard_right_thigh',
                                             'left_seat', 'right_seat')
                   or any(n.startswith(p) for p in ('drive_spine', 'gear_spine', 'inv_spine', 'drive_waist',
                                                    'drive_left_hip', 'drive_right_hip', 'gear_left_hip',
                                                    'gear_right_hip', 'hung_torso', 'hung_pelvis',
                                                    'hung_left_thigh', 'hung_right_thigh'))]
    fig_obs = _extract(body, names, pos, wanted_body)
    at = np.asarray(tl.joint()[0], float)
    trunk_obs = _put([(t + at, c) for (link, name), part in tc.posed().items()
                      if link != 'hips' and name != 'tube' for t, c in part], 'pelvis', 'pelvis')
    hip = [((t + (0.0, THIGH, 0.0)), c) for group in ('thigh', 'roll', 'yaw', 'pelvis')
           for ts in hc.links()[group].values() for t, c in ts]
    hip_obs = _put(hip, 'left_thigh', 'left_shank')
    body_obs = fig_obs + trunk_obs + hip_obs + _mirrored(hip_obs)
    turn_p, shift_p, bend_p = aw.placing()
    _p, shoulder, wrist = aw.laid(False, ())
    still_parts, _gear, arm_in, arm_out = em._stack()
    over = max(r for *_x, r in still_parts) + 0.002
    face_t, face_s = over + em.SADDLE_M + em.SEAT_M, -(over + em.ROTOR_AIR + em.ROTOR_M + em.SEAT_M)
    top, spot_x = float(shoulder[1]) - sr.laid()['down'] - em.FLANGE_M, (float(shift_p[0] * turn_p[0, 0]), 0.0)
    ua = m._prism(m._circle(arm_in, 32, spot_x), face_t + em.FLANGE_M, top, em.CARBON)
    fa = m._prism(m._circle(arm_out + 0.0022, 32, spot_x), float(wrist[1]) + 0.002, face_s - em.FLANGE_M, em.CARBON)
    eb = [((t + shift_p) @ turn_p, c) for t, c in em._drum(over + 0.004, face_s, face_t, em.SERVO)]
    turn_ua, _spot_ua = frames[names.index('left_upper_arm')]
    turn_fa, spot_fa = frames[names.index('left_forearm')]
    gua = [((t - spot_fa) @ turn_ua, c) for t, c in _extract(body, names, pos, ('guard_left_upper_arm',))]
    gfa = [((t - spot_fa) @ turn_fa + np.array([spot_x[0], 0.0, 0.0]), c)
           for t, c in _extract(body, names, pos, ('guard_left_forearm',))]
    lo_t, hi_t = sl.travel()
    lo_r, hi_r = sr.TRAVEL
    lo_e, hi_e = drives.TRAVEL['elbow']
    pec_still = _put(sr._pectoral_still() + _mirrored(sr._pectoral_still()), 'torso', 'torso')
    out = {'in_travel': True, 'crossing': 0}
    for tag, (turn, roll, flex) in (('front', CLASP_FRONT), ('back', CLASP_BACK)):
        out['in_travel'] &= (lo_t <= turn <= hi_t and lo_r <= roll <= hi_r and lo_e <= flex <= hi_e)
        lh_root = _put(_left_hand_local(flex, turn, roll), 'left_upper_arm', 'left_forearm')
        xs = np.vstack([t for t, _c in lh_root])[:, 0]
        zs = np.vstack([t for t, _c in lh_root])[:, 2]
        x_min = float(xs.min())
        out[tag + '_gap'] = round(max(0.0, 2e3 * x_min), 2)
        out[tag + '_lap'] = round(max(0.0, -2e3 * x_min), 2)
        out[tag + '_z'] = (round(float(zs.min()), 3), round(float(zs.max()), 3))
        distal = km._turn(fa + gfa, bend_p * flex) if flex else (fa + gfa)
        sm_p = sm.posed(turn, roll)
        sh_stage2 = [(t * (-1.0, 1.0, -1.0) + shoulder, c) for k, v in sm_p.items()
                     if k not in ('crossbar', 'clevis', 'span', 'pectoral', 'yoke') for t, c in v]
        sh_yoke = _put([(t * (-1.0, 1.0, -1.0) + shoulder, c) for t, c in sm_p['yoke']],
                       'left_upper_arm', 'left_forearm')
        l_arm = _put(_pose_arm_local(ua + gua + eb + distal, turn, roll) + sh_stage2,
                     'left_upper_arm', 'left_forearm') + lh_root
        l_move = l_arm + sh_yoke
        pec = _put(sr.chest(roll, False)['pectoral'] + sr.chest(roll, True)['pectoral'], 'torso', 'torso')
        out['crossing'] += (clips.crossed(l_move, body_obs) + clips.crossed(l_arm, pec)
                            + clips.crossed(sh_yoke, pec_still) + clips.crossed(l_move, _mirrored(l_move)))
    return out


def sheets(out, px=2400, dressed=False):
    """[path]: the whole from each of VIEWS and each region of CLOSE from its own, a PNG each in `out`, `px` a side."""
    import numpy as np
    _body, names, frames, _pos = _figure(False)
    tris = [t for part in whole(dressed=dressed).values() for t in part]
    corners = np.vstack([t for t, _c in tris])
    low, high = corners.min(axis=0), corners.max(axis=0)
    shots = [('whole from ' + name, 0.5 * (low + high), 0.52 * float((high - low).max()), yaw, pitch)
             for name, yaw, pitch in VIEWS]
    for region, ((a, b), half, views) in CLOSE.items():
        centre = 0.5 * (frames[names.index(a)][1] + frames[names.index(b)][1])
        shots += [('%s from %s' % (region, name), centre, half, yaw, pitch) for name, yaw, pitch in views]
    os.makedirs(out, exist_ok=True)
    paths = []
    for title, centre, half, yaw, pitch in shots:
        near = [(t, c) for t, c in tris if np.abs(t.mean(axis=0) - centre).max() < 1.6 * half]
        path = os.path.join(out, title.replace(' ', '_') + '.png')
        m.draw(near, yaw=yaw, pitch=pitch, px=px, half=half, centre=tuple(centre)).save(path)
        paths.append(path)
    return paths


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    parser.add_argument('--px', type=int, default=2400)
    parser.add_argument('--dressed', action='store_true')
    args = parser.parse_args(argv)
    for path in sheets(args.out, args.px, args.dressed):
        print(path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
