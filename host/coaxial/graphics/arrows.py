"""3D movement pattern & telemetry arrows on the floor/surface and in space."""
import math

from coaxial.graphics.callouts import line
from coaxial.graphics.lit import project
from machine.body import figure
from machine.law import se2
from machine.body.figure import quat

FRAME_INK, FILL_INK = (72, 216, 255), (88, 255, 144)
STOP_INK, LAND_INK = (255, 112, 96), (255, 184, 72)

_TWISTS = (
    (0.22, 0.0, 'sway'), (0.28, 0.0, 'apa'), (0.30, 0.0, 'ice_walk', 'shuffle'), (0.32, 0.0, 'step'),
    (0.35, 0.0, 'init_walk', 'init', 'trip', 'mincing'), (0.45, 0.0, 'stairs_down', 'descent'),
    (0.55, 0.0, 'stairs_walk', 'climb', 'slope_walk', 'incline'), (0.85, 0.0, 'ballistic_walk', 'stride'),
    (1.25, 0.0, 'jog', 'run'), (0.69, 0.55, 'turn_left', 'circle_walk'), (0.69, -0.55, 'turn_right'),
    (-0.35, 0.55, 'turn_back_left'), (-0.35, -0.55, 'turn_back_right'),
)
GAIT_TWIST = {**{k: (vx, 0.0, wz) for vx, wz, *ks in _TWISTS for k in ks}, **se2.NOMINAL_TWIST}

_JUMP_UP = {'hop_up_chair', 'crouch_load', 'propulsion', 'flight_tuck', 'seat_land'}
_JUMP_DN = {'hop_down_chair', 'step_off', 'drop', 'absorb'}
JUMP_DOING = {'hop', 'load', 'throw', 'land'} | _JUMP_UP | _JUMP_DN
_SEAT_BED = {'sit_chair', 'approach', 'seated', 'lie_bed', 'sit_edge', 'recline', 'bed_rest'}
_FLOOR = {'lie_floor', 'kneel', 'lower', 'floor_rest'}
FALL_DOING = {'controlled_fall', 'cushion', 'parry_shove', 'catch', 'restore'} | _SEAT_BED | _FLOOR
RISE_DOING = {'rise_chair', 'lean', 'thrust', 'rise_bed', 'sit_up', 'swing_out',
              'rise_floor', 'push_arms', 'half_kneel'}
STOP_DOING = {'brake', 'touch'}
_SURF_KINDS = {'slope', 'stairs', 'snow', 'ice', 'rug', 'slip', 'sill', 'board', 'brick'}


def surface_at(x, z, props=(), include_furniture=False):
    """(y, normal_xyz) of the walkable surface at world (x, z), conforming to props."""
    best_y, best_n = 0.0, (0.0, 1.0, 0.0)
    kinds = _SURF_KINDS | ({'chair', 'bed'} if include_furniture else set())
    for prop in (props or ()):
        if len(prop) < 4:
            continue
        kind, (cx, cy, cz), (hx, hy, hz), ((r00, r01, r02), (r10, r11, r12), (r20, r21, r22)) = prop[:4]
        nx, ny, nz = float(r01), float(r11), float(r21)
        if abs(ny) < 0.2:
            continue
        py = cy + hy * ny - (nx * (x - cx - hy * nx) + nz * (z - cz - hy * nz)) / ny
        ux, uy, uz = x - cx, py - cy, z - cz
        if abs(ux * r00 + uy * r10 + uz * r20) > hx + 0.02 or abs(ux * r02 + uy * r12 + uz * r22) > hz + 0.02:
            continue
        if kind == 'hole':
            best_y, best_n = min(best_y, float(cy - hy)), (0.0, 1.0, 0.0)
        elif kind in kinds and not (kind == 'chair' and hy > 0.10) and not (kind == 'bed' and hy > 0.20) and py >= best_y - 1e-4:
            best_y, best_n = float(py), (nx, ny, nz)
    return best_y + 0.004, best_n


def feet_origin(now, props=()):
    """World (x, y, z) on the surface midway between her left and right feet."""
    where = now.get('where', (0.0, 0.90, 0.0))
    try:
        fr = figure.frames(now.get('angles') or {}, where, quat(*now.get('turn', (1.0, 0.0, 0.0, 0.0))))
        la, ra = fr['left_ankle'][0], fr['right_ankle'][0]
        mx, mz = 0.5 * (float(la[0]) + float(ra[0])), 0.5 * (float(la[2]) + float(ra[2]))
    except Exception:
        mx, mz = float(where[0]), float(where[2])
    return (mx, surface_at(mx, mz, props)[0], mz)


def pelvis_yaw(now, fallback=0.0):
    """Physical horizontal pelvis yaw (rad) from `now` dict or `(qw, qx, qy, qz)` quaternion."""
    hd = float(now.get('heading', fallback)) if isinstance(now, dict) else float(fallback)
    turn = now.get('turn') if isinstance(now, dict) else now
    if turn is not None and len(turn) >= 4:
        qw, qx, qy, qz = float(turn[0]), float(turn[1]), float(turn[2]), float(turn[3])
        n2 = qw * qw + qx * qx + qy * qy + qz * qz
        if n2 > 1e-12:
            s = 2.0 / n2
            return math.atan2(s * (qx * qz + qw * qy), 1.0 - s * (qx * qx + qy * qy))
    return hd


def _path_sampler(origin, heading, vx_set, vy_set, wz_set, props, length_scale=1.0):
    ox, _oy, oz = origin
    spd = math.hypot(vx_set, vy_set)
    spin = spd <= 1e-3 and abs(wz_set) > 0.03
    length = 0.432 * length_scale if spin else max(0.38, min(1.35, (0.35 + 0.72 * spd) * length_scale))
    ang0 = heading + (math.atan2(vy_set, vx_set) if spd > 1e-3 else 0.0)
    turn_gain = 0.95 + 0.25 * min(1.0, spd / 0.70)
    turn = ((1.35 if wz_set > 0 else -1.35) if spin
            else (max(-1.1, min(1.1, wz_set * (-1.0 if vx_set < -0.02 else 1.0) * turn_gain)) if abs(wz_set) > 0.03 else 0.0))

    def sample(u):
        s, ang = u * length, ang0 + u * turn
        px, pz = ((ox + s * math.sin(ang0), oz + s * math.cos(ang0)) if abs(turn) < 1e-3 else (
            ox + length / turn * (math.cos(ang0) - math.cos(ang)), oz - length / turn * (math.sin(ang0) - math.sin(ang))))
        return (px, pz), (math.sin(ang), math.cos(ang)), (math.cos(ang), -math.sin(ang))

    pt3 = lambda px, pz: (px, surface_at(px, pz, props)[0], pz)
    width_at = lambda u: 0.055 if u <= 0.70 else max(0.0, 0.135 * (1.0 - (u - 0.70) / 0.30))
    return sample, pt3, width_at, 0.055, 0.135


def _surface_path_arrow(origin, heading, vx_set, vy_set, wz_set, fill, props, length_scale=1.0):
    sample, pt3, width_at, w_s, w_h = _path_sampler(origin, heading, vx_set, vy_set, wz_set, props, length_scale)

    def span(u, w):
        (px, pz), _, (lx, lz) = sample(u)
        return pt3(px + w * lx, pz + w * lz), pt3(px - w * lx, pz - w * lz)

    pts = [span(u, w) for u, w in ([(0.70 * k / 11, w_s) for k in range(12)] + [(0.70, w_h)]
                                   + [(0.70 + 0.30 * k / 6, width_at(0.70 + 0.30 * k / 6)) for k in range(1, 7)])]
    frame_segs = [pts[0], *(seg for (pl, pr), (cl, cr) in zip(pts[:-1], pts[1:]) for seg in ((pl, cl), (pr, cr)))]
    fill_segs, u_fill = [], max(0.0, min(1.0, float(fill)))
    if u_fill > 0.03:
        n_f = max(1, int(math.ceil(u_fill * 16)))
        for frac in (-0.72, -0.36, 0.0, 0.36, 0.72):
            line_pts = [span(u_fill * k / n_f, frac * width_at(u_fill * k / n_f) * 0.85)[0] for k in range(n_f + 1)]
            fill_segs.extend(zip(line_pts[:-1], line_pts[1:]))
        fill_segs.extend(span(u_fill * k / n_f, width_at(u_fill * k / n_f) * 0.88) for k in range(1, n_f + 1))
    return frame_segs, fill_segs


def _ring_segments(cx, cz, r, props, n=8, include_furniture=False, cross=True):
    pt = lambda x, z: (x, surface_at(x, z, props, include_furniture)[0], z)
    pts = [pt(cx + r * math.cos(math.tau * k / n), cz + r * math.sin(math.tau * k / n)) for k in range(n)]
    segs = [(pts[k], pts[(k + 1) % n]) for k in range(n)]
    if cross:
        y_c, rc = pt(cx, cz)[1], r * 0.75
        segs += [((cx - rc, y_c, cz), (cx + rc, y_c, cz)), ((cx, y_c, cz - rc), (cx, y_c, cz + rc))]
    return segs


def _vertical_arrow(origin, h_set, fill, direction=1.0, fwd=0.0, heading=0.0, props=()):
    ox, oy, oz = origin
    sh, ch = math.sin(heading), math.cos(heading)
    y0, y1 = (oy, oy + h_set) if direction > 0 else (oy + h_set, oy)
    end_y = surface_at(ox + fwd * sh, oz + fwd * ch, props, True)[0] if abs(fwd) > 0.05 else 0.0
    width_at = lambda u: 0.045 if u <= 0.72 else max(0.0, 0.115 * (1.0 - (u - 0.72) / 0.28))

    def span(u, w, ax=ch, az=-sh):
        fy = ((1.0 - u) ** 2 * oy + 2.0 * (1.0 - u) * u * (max(oy, end_y) + h_set) + u * u * end_y
              if abs(fwd) > 0.05 else y0 + u * (y1 - y0))
        cx, cz = ox + u * fwd * sh, oz + u * fwd * ch
        return (cx + w * ax, fy, cz + w * az), (cx - w * ax, fy, cz - w * az)

    frame_segs = _ring_segments(ox, oz, 0.12, props, 8, False, False)
    stations = [(k / 12, width_at(k / 12)) for k in range(9)] + [(8 / 12, 0.115)] + [(k / 12, width_at(k / 12)) for k in range(9, 13)]
    for ax, az in ((ch, -sh), (sh, ch)):
        pts = [span(u, w, ax, az) for u, w in stations]
        frame_segs.extend(seg for (pl, pr), (cl, cr) in zip(pts[:-1], pts[1:]) for seg in ((pl, cl), (pr, cr)))
    fill_segs, u_fill = [], max(0.0, min(1.0, float(fill)))
    if u_fill > 0.02:
        n_f = max(2, int(math.ceil(u_fill * 12)))
        for k in range(n_f):
            u_a, u_b = u_fill * k / n_f, u_fill * (k + 1) / n_f
            wa, wb = width_at(u_a) * 0.82, width_at(u_b) * 0.82
            fill_segs += [(span(u_a, f * wa)[0], span(u_b, f * wb)[0]) for f in (-0.65, 0.0, 0.65)] + [span(u_b, -wb)]
    return frame_segs, fill_segs


def _spec(kind, origin=None, normal=None, setpoint=(0.0, 0.0, 0.0), actual=(0.0, 0.0, 0.0),
          fill=0.0, landing=None, frame=(), fill_segs=(), target=(), heading=0.0, pelvis_yaw=0.0):
    return {'kind': kind, 'origin': origin, 'normal': normal, 'setpoint': setpoint, 'actual': actual, 'fill': fill,
            'heading': heading, 'pelvis_yaw': pelvis_yaw, 'landing': landing,
            'frame_segments': list(frame), 'fill_segments': list(fill_segs), 'target_segments': list(target)}


def _jump_arrow(doing, _now, origin, normal, heading, sh, ch, spd_phys, props, tw_ord):
    spd_f = max(tw_ord[0], spd_phys)
    fwd = 0.45 if doing in _JUMP_UP else (0.38 if doing in _JUMP_DN else (min(0.55, spd_f * 0.55) if spd_f > 0.08 else 0.0))
    vy_j = 2.2 if doing in (_JUMP_UP | {'throw'}) - {'crouch_load', 'seat_land'} else (1.2 if doing in ('load', 'crouch_load') else 1.6)
    fill, l_pt = max(0.25, min(1.0, vy_j / 2.2)), (origin[0] + fwd * sh, origin[1], origin[2] + fwd * ch)
    has_fwd = abs(fwd) > 0.05
    l_pt = (l_pt[0], surface_at(l_pt[0], l_pt[2], props, True)[0], l_pt[2]) if has_fwd else l_pt
    t_segs = _ring_segments(l_pt[0], l_pt[2], 0.16, props, 8, True) if has_fwd else []
    return _spec('jump', origin, normal, (fwd, 0.85, 0.0), (tw_ord[0], vy_j, tw_ord[2]), fill, l_pt,
                 *_vertical_arrow(origin, 0.85, fill, 1.0, fwd, heading, props), t_segs, heading=heading, pelvis_yaw=heading)


def _fall_arrow(doing, now, origin, _normal, heading, sh, ch, _spd_phys, props, tw_ord):
    where, vel_xz = now.get('where', (0.0, 0.90, 0.0)), now.get('velocity', (0.0, 0.0))
    ty = 0.45 if doing in _SEAT_BED else (0.22 if (doing in _FLOOR or now.get('fallen')) else 0.58)
    tf = math.sqrt(2.0 * max(0.04, float(where[1]) - ty) / 9.81)
    po = -0.18 if doing in _SEAT_BED else (0.22 if doing in _FLOOR else 0.25)
    lx, lz = float(where[0]) + float(vel_xz[0]) * tf + po * sh, float(where[2]) + float(vel_xz[1]) * tf + po * ch
    ly, l_norm = surface_at(lx, lz, props, True)
    l_pt, fill = (lx, ly, lz), max(0.25, min(1.0, (0.90 - float(where[1])) / max(0.12, 0.90 - ty)))
    f_segs, fl_segs = _surface_path_arrow(
        origin, heading, (lx - origin[0]) * sh + (lz - origin[2]) * ch, (lx - origin[0]) * ch - (lz - origin[2]) * sh, 0.0, fill, props)
    rings = _ring_segments(lx, lz, 0.20, props, 8, True, True) + _ring_segments(lx, lz, 0.10, props, 8, True, False)
    return _spec('fall', origin, l_norm, l_pt, tw_ord, fill, l_pt, [*f_segs, (tuple(map(float, where)), l_pt)], fl_segs,
                 rings, heading=heading, pelvis_yaw=heading)


def _vert_posture_arrow(doing, _now, origin, normal, heading, _sh, _ch, _spd_phys, props, _tw_ord):
    vy_p = 0.65 if doing in RISE_DOING else -0.65
    return _spec('posture', origin, normal, (0.0, vy_p, 0.0), (0.0, vy_p, 0.0), 1.0, origin,
                 *_vertical_arrow(origin, 0.65, 1.0, 1.0 if vy_p > 0 else -1.0, 0.0, heading, props),
                 heading=heading, pelvis_yaw=heading)


def _surf_posture_arrow(doing, _now, origin, normal, heading, _sh, _ch, _spd_phys, props, _tw_ord):
    vec = (0.0, 0.45, 0.0) if doing in ('one_sole', 'weight_shift', 'over') else (0.45, 0.0, 0.0)
    return _spec('posture', origin, normal, vec, vec, 1.0, origin,
                 *_surface_path_arrow(origin, heading, vec[0], vec[1], 0.0, 1.0, props),
                 heading=heading, pelvis_yaw=heading)


_SPECIAL_ARROWS = (
    (lambda d, now: d in JUMP_DOING, _jump_arrow),
    (lambda d, now: d in FALL_DOING or bool(now.get('fallen')), _fall_arrow),
    (lambda d, _now: d in ('crouch', 'down') or d in RISE_DOING, _vert_posture_arrow),
    (lambda d, _now: d in ('one_sole', 'weight_shift', 'over', 'bow', 'forward'), _surf_posture_arrow),
)


def _describe_special(doing, now, origin, normal, heading, sh, ch, spd_phys, props, tw_ord):
    """Dispatch jump, fall/parry/sit/lie, and postural arrow specifications from `_SPECIAL_ARROWS`."""
    fn = next((f for pred, f in _SPECIAL_ARROWS if pred(doing, now)), None)
    return fn(doing, now, origin, normal, heading, sh, ch, spd_phys, props, tw_ord) if fn else None


def _actual_twist(now, se2_st, sh, ch, doing, any_key):
    """Resolve body-frame actual twist `(vx, vy, wz)` from physics telemetry or SE(2) state."""
    vel_xz, tw_now, spd_now = now.get('velocity', (0.0, 0.0)), now.get('twist_actual'), float(now.get('speed', 0.0))
    has_vel = 'velocity' in now or 'speed' in now
    v_mag = math.hypot(float(vel_xz[0]), float(vel_xz[1]))
    vx_v = float(vel_xz[0]) * sh + float(vel_xz[1]) * ch if v_mag > 1e-4 else spd_now
    vy_v = float(vel_xz[0]) * ch - float(vel_xz[1]) * sh if v_mag > 1e-4 else 0.0
    st_act = ((tuple(map(float, se2_st['actual'])) if 'actual' in se2_st
               else (float(se2_st.get('v_x', 0.0)), float(se2_st.get('v_y', 0.0)), float(se2_st.get('omega_z', 0.0))))
              if se2_st is not None else tuple(map(float, (now.get('twist') or (0.0, 0.0, 0.0))[:3])))
    wz_fb = float(tw_now[2]) if (tw_now is not None and abs(tw_now[2]) > 1e-4) else st_act[2]
    candidates = (
        (v_mag > 1e-4 or abs(spd_now) > 1e-4, (vx_v, vy_v, wz_fb)),
        (tw_now is not None and any(abs(v) > 1e-4 for v in tw_now[:3]),
         tuple(map(float, tw_now[:3])) if tw_now is not None else (0.0, 0.0, 0.0)),
        (has_vel, (0.0, 0.0, wz_fb)),
        (True, st_act),
    )
    tw_act = next(val for cond, val in candidates if cond)
    if se2_st is not None and (tw_now is not None or has_vel):
        se2_st['actual'] = tw_act
    if math.hypot(tw_act[0], tw_act[1]) <= 0.01 and abs(tw_act[2]) <= 0.01 and doing in GAIT_TWIST and not any_key and tw_now is None and not has_vel:
        tw_act = GAIT_TWIST[doing]
    return tw_act


def describe(now, se2_st=None):
    """Compute 3D movement pattern indicator specification for `now` and `se2_st`."""
    if not now:
        return _spec('idle')
    props = now.get('props') or ()
    origin = feet_origin(now, props)
    _, normal = surface_at(origin[0], origin[2], props)
    raw_hd, doing, vel_xz = float(now.get('heading', 0.0)), str(now.get('doing', 'rest')), now.get('velocity', (0.0, 0.0))
    entry = str(now.get('entry') or '')
    intent = entry if doing in ('brake', 'stand', 'rest', 'init_walk') and entry in (JUMP_DOING | FALL_DOING | RISE_DOING | set(GAIT_TWIST)) else doing
    pyaw = pelvis_yaw(now, raw_hd)
    set_hd = se2.wrap_angle(pyaw + 0.35 * se2.wrap_angle(raw_hd - pyaw))
    sh, ch, spd_phys = math.sin(pyaw), math.cos(pyaw), math.hypot(float(vel_xz[0]), float(vel_xz[1]))
    keys = (se2_st or {}).get('keys') or {}
    any_key = any(keys.get(k, False) for k in ('w', 's', 'a', 'd', 'q', 'e'))
    tw_act = _actual_twist(now, se2_st, sh, ch, intent, any_key)
    vx_act, vy_act, wz_act = tw_act
    act_mag = math.hypot(vx_act, vy_act)
    special = _describe_special(intent, now, origin, normal, pyaw, sh, ch, spd_phys, props, tw_act)
    if special is not None:
        return special
    if intent in STOP_DOING:
        vx_dir, stop_dist = (vx_act if abs(vx_act) > 0.02 else 0.35), max(0.32, min(0.75, 0.32 + 0.45 * max(act_mag, spd_phys)))
        fill, sgn = max(0.15, min(1.0, 1.0 - act_mag / se2.WALK_SPEED)), (1.0 if vx_dir >= 0 else -1.0)
        sx, sz = origin[0] + stop_dist * sh * sgn, origin[2] + stop_dist * ch * sgn
        sy, lx_v, lz_v = surface_at(sx, sz, props)[0], ch * 0.22, -sh * 0.22
        t_segs = [((sx - lx_v, sy + dy, sz - lz_v), (sx + lx_v, sy + dy, sz + lz_v)) for dy in (0.0, 0.02)] + _ring_segments(sx, sz, 0.12, props, 8, False, False)
        return _spec('stop', origin, normal, (0.0, 0.0, 0.0), tw_act, fill, (sx, sy, sz),
                     *_surface_path_arrow(origin, pyaw, vx_dir, vy_act, 0.0, fill, props, 0.75), t_segs,
                     heading=pyaw, pelvis_yaw=pyaw)
    tw_cmd = now.get('twist')
    has_cmd = tw_cmd is not None and (math.hypot(float(tw_cmd[0]), float(tw_cmd[1])) > 0.02 or abs(float(tw_cmd[2])) > 0.03)
    vx_set, vy_set, wz_set = next(val for cond, val in (
        (any_key, (se2_st or {}).get('setpoint') or (0.0, 0.0, 0.0)),
        (has_cmd, (float(tw_cmd[0]), float(tw_cmd[1]), float(tw_cmd[2])) if tw_cmd is not None else (0.0, 0.0, 0.0)),
        (intent in GAIT_TWIST, GAIT_TWIST.get(intent, (0.0, 0.0, 0.0))),
        (True, (0.0, 0.0, 0.0)),
    ) if cond)
    set_mag = math.hypot(vx_set, vy_set)
    if set_mag > 0.02 or abs(wz_set) > 0.03:
        wz_way = wz_set * (-1.0 if vx_set < -0.02 else 1.0)
        lin_s = max(0.0, (vx_act * vx_set + vy_act * vy_set) / (set_mag * set_mag)) if set_mag > 0.02 else 0.0
        yaw_s = max(0.0, (wz_act * wz_way) / (wz_way * wz_way)) if abs(wz_way) > 0.03 else 0.0
        fill = min(1.0, 0.5 * (lin_s + yaw_s) if (set_mag > 0.02 and abs(wz_set) > 0.03) else (lin_s if set_mag > 0.02 else yaw_s))
        return _spec('locomotion', origin, normal, (vx_set, vy_set, wz_set), tw_act, fill, None,
                     *_surface_path_arrow(origin, set_hd, vx_set, vy_set, wz_set, fill, props),
                     heading=set_hd, pelvis_yaw=pyaw)
    return _spec('idle', origin, normal, heading=pyaw, pelvis_yaw=pyaw)


def _rasterize(segments, m, cam, centre, travel):
    from coaxial.model.blocks import numpy as np
    if not segments:
        return None
    pts = np.array([[p[0] - travel[0], p[1], p[2] - travel[1]]
                    for seg in segments for p in seg], dtype=float)
    sx, sy, w = project(pts, m, cam, centre)
    dots = np.zeros((int(cam['height']), int(cam['width'])), bool)
    for k in range(0, len(pts), 2):
        if w[k] > 0.0 and w[k + 1] > 0.0:
            line(dots, (sx[k], sy[k]), (sx[k + 1], sy[k + 1]))
    return dots if dots.any() else None


def layers(arrows_in, m, cam, centre, travel=(0.0, 0.0)):
    """[(dots, ink)] for `lit.braille`: setpoint frame, stop/landing target, and actual fill."""
    if not isinstance(arrows_in, (tuple, dict)):
        return []
    spec = (describe(arrows_in[0], arrows_in[1]) if isinstance(arrows_in, tuple)
            else (arrows_in if 'frame_segments' in arrows_in else describe(arrows_in, None)))
    kind = spec.get('kind', 'idle')
    out = []
    for key, ink in (('frame_segments', FRAME_INK), ('target_segments', LAND_INK), ('fill_segments', FILL_INK)):
        dots = _rasterize(spec.get(key), m, cam, centre, travel)
        if dots is not None:
            out.append((dots, STOP_INK if kind == 'stop' else ink))
    return out

