"""3D movement pattern & telemetry arrows on the floor/surface and in space.

Draws the commanded setpoint frame (börvärdespil) conforming to the surface under and ahead of
her feet (floor, snow, inclined slope, ice, stairs, rug, hole) starting between her feet,
filled internally by the achieved actual value (ärvärde). Also renders 3D upward/parabolic
jump arrows, commanded stop bars, and estimated touch-down landing zones for falls, shoves,
sitting, and lying down.
"""
import math

from coaxial.graphics.callouts import line
from coaxial.graphics.lit import project
from machine import figure, se2
from machine.figure import quat

#: Inks for the setpoint arrow frame (börvärde), actual fill (ärvärde), and stop/landing targets.
FRAME_INK = (72, 216, 255)
FILL_INK = (88, 255, 144)
STOP_INK = (255, 112, 96)
LAND_INK = (255, 184, 72)

#: Nominal reference twists (vx, vy, omega_z) in body frame for discrete gait entries.
GAIT_TWIST = {
    '#202': (0.22, 0.0, 0.0),
    'apa': (0.28, 0.0, 0.0),
    '#203': (0.32, 0.0, 0.0),
    'init_walk': (0.35, 0.0, 0.0),
    'init': (0.35, 0.0, 0.0),
    '#204': (0.69, 0.0, 0.0),
    'ballistic_walk': (0.85, 0.0, 0.0),
    'stride': (0.85, 0.0, 0.0),
    '#205': (-0.35, 0.0, 0.0),
    'trip': (0.35, 0.0, 0.0),
    'mincing': (0.35, 0.0, 0.0),
    'jog': (1.25, 0.0, 0.0),
    'run': (1.25, 0.0, 0.0),
    'stairs_walk': (0.55, 0.0, 0.0),
    'climb': (0.55, 0.0, 0.0),
    'stairs_down': (0.45, 0.0, 0.0),
    'descent': (0.45, 0.0, 0.0),
    'ice_walk': (0.30, 0.0, 0.0),
    'shuffle': (0.30, 0.0, 0.0),
    'slope_walk': (0.55, 0.0, 0.0),
    'incline': (0.55, 0.0, 0.0),
    'strafe_left': (0.0, 0.50, 0.0),
    'strafe_right': (0.0, -0.50, 0.0),
    'turn_left': (0.69, 0.0, 0.55),
    'turn_right': (0.69, 0.0, -0.55),
    'circle_walk': (0.69, 0.0, 0.55),
    'turn_back_left': (-0.35, 0.0, 0.55),
    'turn_back_right': (-0.35, 0.0, -0.55),
}

JUMP_DOING = {'hop', 'load', 'throw', 'land', 'hop_up_chair', 'crouch_load',
              'propulsion', 'flight_tuck', 'seat_land', 'hop_down_chair',
              'step_off', 'drop', 'absorb'}
FALL_DOING = {'controlled_fall', 'cushion', 'parry_shove', 'catch', 'restore',
              'lie_floor', 'kneel', 'lower', 'floor_rest', 'lie_bed', 'sit_edge',
              'recline', 'bed_rest', 'sit_chair', 'approach', 'seated'}
RISE_DOING = {'rise_chair', 'lean', 'thrust', 'rise_bed', 'sit_up', 'swing_out',
              'rise_floor', 'push_arms', 'half_kneel'}
STOP_DOING = {'brake', 'touch'}


def surface_at(x, z, props=(), include_furniture=False):
    """(y, normal_xyz) of the walkable surface at world (x, z), conforming to props."""
    best_y = 0.0
    best_n = (0.0, 1.0, 0.0)
    surf_kinds = {'slope', 'stairs', 'snow', 'ice', 'rug', 'slip', 'sill', 'board', 'brick'}
    if include_furniture:
        surf_kinds = surf_kinds | {'chair', 'bed'}
    for prop in (props or ()):
        if len(prop) < 4:
            continue
        kind, (cx, cy, cz), (hx, hy, hz), rot = prop[:4]
        dx, dz = x - cx, z - cz
        # Project horizontal offset onto prop local X and Z axes
        r00, r01, r02 = rot[0]
        r10, r11, r12 = rot[1]
        r20, r21, r22 = rot[2]
        # Top face center p0 = center + hy * n, where n = (r01, r11, r21)
        nx, ny, nz = float(r01), float(r11), float(r21)
        if abs(ny) < 0.2:
            continue
        p0x, p0y, p0z = cx + hy * nx, cy + hy * ny, cz + hy * nz
        plane_y = p0y - (nx * (x - p0x) + nz * (z - p0z)) / ny
        ux, uy, uz = x - cx, plane_y - cy, z - cz
        lx = ux * r00 + uy * r10 + uz * r20
        lz = ux * r02 + uy * r12 + uz * r22
        if abs(lx) <= hx + 0.02 and abs(lz) <= hz + 0.02:
            if kind == 'hole':
                best_y = min(best_y, float(cy - hy))
                best_n = (0.0, 1.0, 0.0)
            elif kind in surf_kinds:
                if kind == 'chair' and hy > 0.10:
                    continue  # skip chair backrest
                if kind == 'bed' and hy > 0.20:
                    continue  # skip bed headboard
                if plane_y >= best_y - 1e-4:
                    best_y = float(plane_y)
                    best_n = (nx, ny, nz)
    return best_y + 0.004, best_n


def feet_origin(now, props=()):
    """World (x, y, z) on the surface midway between her left and right feet."""
    where = now.get('where', (0.0, 0.90, 0.0))
    angles = now.get('angles') or {}
    turn = now.get('turn', (1.0, 0.0, 0.0, 0.0))
    try:
        fr = figure.frames(angles, where, quat(*turn))
        la = fr['left_ankle'][0]
        ra = fr['right_ankle'][0]
        mx = 0.5 * (float(la[0]) + float(ra[0]))
        mz = 0.5 * (float(la[2]) + float(ra[2]))
    except Exception:
        mx, mz = float(where[0]), float(where[2])
    sy, _n = surface_at(mx, mz, props)
    return (mx, sy, mz)


def _surface_path_arrow(origin, heading, vx_set, vy_set, wz_set, fill, props, length_scale=1.0):
    """Build surface-conforming 3D arrow frame and inner actual-value fill segments."""
    ox, _oy, oz = origin
    spd_set = math.hypot(vx_set, vy_set)
    total_len = max(0.38, min(1.35, (0.35 + 0.72 * spd_set) * length_scale))
    shaft_len = 0.70 * total_len
    w_shaft = 0.055
    w_head = 0.135
    base_dir = math.atan2(vy_set, vx_set) if spd_set > 1e-4 else (math.pi / 2.0 if wz_set > 0 else -math.pi / 2.0)
    # Curvature along path for composite turns (WA/WD/SA/SD)
    turn_sign = -1.0 if vx_set < -0.02 else 1.0
    total_turn = max(-1.1, min(1.1, wz_set * turn_sign * 1.15)) if abs(wz_set) > 0.03 else 0.0
    n_samples = 16

    def sample(u):
        s = u * total_len
        if abs(total_turn) < 1e-3:
            ang = heading + base_dir
            px = ox + s * math.sin(ang)
            pz = oz + s * math.cos(ang)
            tang = ang
        else:
            r = total_len / total_turn
            ang0 = heading + (0.0 if vx_set >= -0.02 else math.pi)
            ang = ang0 + u * total_turn
            # Arc in world (x, z) where +heading is from +Z towards +X
            cx = ox + r * math.cos(ang0)
            cz = oz - r * math.sin(ang0)
            px = cx - r * math.cos(ang)
            pz = cz + r * math.sin(ang)
            tang = ang
        tx, tz = math.sin(tang), math.cos(tang)
        # Left unit vector in world (x, z): +heading + pi/2 -> (cos(tang), -sin(tang))
        lx, lz = math.cos(tang), -math.sin(tang)
        return (px, pz), (tx, tz), (lx, lz)

    def pt3(px, pz):
        py, _ = surface_at(px, pz, props)
        return (px, py, pz)

    def width_at(u):
        if u <= 0.70:
            return w_shaft
        return max(0.0, w_head * (1.0 - (u - 0.70) / 0.30))

    frame_segs = []
    # Base segment between feet
    (p0x, p0z), _, (l0x, l0z) = sample(0.0)
    b_left = pt3(p0x + w_shaft * l0x, p0z + w_shaft * l0z)
    b_right = pt3(p0x - w_shaft * l0x, p0z - w_shaft * l0z)
    frame_segs.append((b_left, b_right))

    # Shaft side rails (u = 0 .. 0.70)
    n_shaft = 11
    prev_l, prev_r = b_left, b_right
    for k in range(1, n_shaft + 1):
        u = 0.70 * k / n_shaft
        (px, pz), _, (lx, lz) = sample(u)
        cur_l = pt3(px + w_shaft * lx, pz + w_shaft * lz)
        cur_r = pt3(px - w_shaft * lx, pz - w_shaft * lz)
        frame_segs.append((prev_l, cur_l))
        frame_segs.append((prev_r, cur_r))
        prev_l, prev_r = cur_l, cur_r

    # Arrowhead flares and outer edges (u = 0.70 .. 1.0)
    (hx, hz), _, (hlx, hlz) = sample(0.70)
    wing_l = pt3(hx + w_head * hlx, hz + w_head * hlz)
    wing_r = pt3(hx - w_head * hlx, hz - w_head * hlz)
    frame_segs.append((prev_l, wing_l))
    frame_segs.append((prev_r, wing_r))

    n_head = 6
    prev_wl, prev_wr = wing_l, wing_r
    for k in range(1, n_head + 1):
        u = 0.70 + 0.30 * k / n_head
        w_u = width_at(u)
        (px, pz), _, (lx, lz) = sample(u)
        cur_wl = pt3(px + w_u * lx, pz + w_u * lz)
        cur_wr = pt3(px - w_u * lx, pz - w_u * lz)
        frame_segs.append((prev_wl, cur_wl))
        frame_segs.append((prev_wr, cur_wr))
        prev_wl, prev_wr = cur_wl, cur_wr

    # Actual value interior fill (ärvärde) up to u = fill
    fill_segs = []
    u_fill = max(0.0, min(1.0, float(fill)))
    if u_fill > 0.03:
        n_fill = max(1, int(math.ceil(u_fill * n_samples)))
        # Longitudinal interior ribs + transverse crossbars
        for frac in (-0.72, -0.36, 0.0, 0.36, 0.72):
            pts = []
            for k in range(n_fill + 1):
                u = u_fill * k / n_fill
                w_u = width_at(u) * 0.85
                (px, pz), _, (lx, lz) = sample(u)
                pts.append(pt3(px + frac * w_u * lx, pz + frac * w_u * lz))
            for a, b in zip(pts[:-1], pts[1:]):
                fill_segs.append((a, b))
        for k in range(1, n_fill + 1):
            u = u_fill * k / n_fill
            w_u = width_at(u) * 0.88
            (px, pz), _, (lx, lz) = sample(u)
            fill_segs.append((pt3(px + w_u * lx, pz + w_u * lz),
                              pt3(px - w_u * lx, pz - w_u * lz)))
    return frame_segs, fill_segs


def _ring_segments(cx, cz, radius, props, n=8, include_furniture=False, cross=True):
    """Surface-conforming polygon ring (and optional crosshairs) at world (cx, cz)."""
    pts = []
    for k in range(n):
        ang = math.tau * k / n
        px, pz = cx + radius * math.cos(ang), cz + radius * math.sin(ang)
        py, _ = surface_at(px, pz, props, include_furniture=include_furniture)
        pts.append((px, py, pz))
    segs = [(pts[k], pts[(k + 1) % n]) for k in range(n)]
    if cross:
        y_c, _ = surface_at(cx, cz, props, include_furniture=include_furniture)
        segs.append(((cx - radius * 0.75, y_c, cz), (cx + radius * 0.75, y_c, cz)))
        segs.append(((cx, y_c, cz - radius * 0.75), (cx, y_c, cz + radius * 0.75)))
    return segs


def _vertical_arrow(origin, height_set, fill, direction=1.0, fwd_offset=0.0, heading=0.0, props=()):
    """3D vertical or arched arrow frame and actual fill for jumps, crouches, and rises."""
    ox, oy, oz = origin
    sh, ch = math.sin(heading), math.cos(heading)
    lx, lz = ch, -sh
    w_s, w_h = 0.045, 0.115
    y0 = oy if direction > 0 else oy + height_set
    y1 = oy + height_set if direction > 0 else oy

    def center_at(u):
        # Parabolic arch if fwd_offset != 0
        fx = ox + u * fwd_offset * sh
        fz = oz + u * fwd_offset * ch
        if abs(fwd_offset) > 0.05:
            end_y, _ = surface_at(ox + fwd_offset * sh, oz + fwd_offset * ch, props, include_furniture=True)
            peak_y = max(oy, end_y) + height_set
            fy = (1.0 - u) * (1.0 - u) * oy + 2.0 * (1.0 - u) * u * peak_y + u * u * end_y
        else:
            fy = y0 + u * (y1 - y0)
        return (fx, fy, fz)

    def width_at(u):
        return w_s if u <= 0.72 else max(0.0, w_h * (1.0 - (u - 0.72) / 0.28))

    frame_segs = _ring_segments(ox, oz, 0.12, props, n=8, cross=False)
    n_pts = 12
    for axis_x, axis_z in ((lx, lz), (sh, ch)):
        prev_l = prev_r = None
        for k in range(n_pts + 1):
            u = k / n_pts
            cx, cy, cz = center_at(u)
            wu = width_at(u)
            cur_l = (cx + wu * axis_x, cy, cz + wu * axis_z)
            cur_r = (cx - wu * axis_x, cy, cz - wu * axis_z)
            if prev_l is not None:
                frame_segs.append((prev_l, cur_l))
                frame_segs.append((prev_r, cur_r))
            if k == int(0.72 * n_pts):
                wl = (cx + w_h * axis_x, cy, cz + w_h * axis_z)
                wr = (cx - w_h * axis_x, cy, cz - w_h * axis_z)
                frame_segs.append((cur_l, wl))
                frame_segs.append((cur_r, wr))
                cur_l, cur_r = wl, wr
            prev_l, prev_r = cur_l, cur_r

    fill_segs = []
    u_fill = max(0.0, min(1.0, float(fill)))
    if u_fill > 0.02:
        n_f = max(2, int(math.ceil(u_fill * n_pts)))
        for k in range(n_f):
            u_a, u_b = u_fill * k / n_f, u_fill * (k + 1) / n_f
            ca, cb = center_at(u_a), center_at(u_b)
            wa = width_at(u_a) * 0.82
            wb = width_at(u_b) * 0.82
            for frac in (-0.65, 0.0, 0.65):
                fill_segs.append(((ca[0] + frac * wa * lx, ca[1], ca[2] + frac * wa * lz),
                                  (cb[0] + frac * wb * lx, cb[1], cb[2] + frac * wb * lz)))
            fill_segs.append(((cb[0] - wb * lx, cb[1], cb[2] - wb * lz),
                              (cb[0] + wb * lx, cb[1], cb[2] + wb * lz)))
    return frame_segs, fill_segs


def _describe_special(doing, now, origin, normal, heading, props, tw_ord):
    """Handle jump, fall/parry/sit/lie, stop/brake, and postural arrow specifications."""
    vx_ord, vy_ord, wz_ord = tw_ord
    sh, ch = math.sin(heading), math.cos(heading)
    where = now.get('where', (0.0, 0.90, 0.0))
    vel_xz = now.get('velocity', (0.0, 0.0))
    spd_phys = math.hypot(float(vel_xz[0]), float(vel_xz[1]))
    if doing in JUMP_DOING:
        spd_f = max(vx_ord, spd_phys)
        fwd = 0.45 if doing in ('hop_up_chair', 'crouch_load', 'propulsion', 'flight_tuck', 'seat_land') else (
            0.38 if doing in ('hop_down_chair', 'step_off', 'drop', 'absorb')
            else (min(0.55, spd_f * 0.55) if spd_f > 0.08 else 0.0))
        vy_jump = 2.2 if doing in ('throw', 'propulsion', 'flight_tuck', 'hop_up_chair') else (
            1.2 if doing in ('load', 'crouch_load') else 1.6)
        fill = max(0.25, min(1.0, vy_jump / 2.2))
        f_segs, fl_segs = _vertical_arrow(origin, 0.85, fill, 1.0, fwd, heading, props)
        land_pt = (origin[0] + fwd * sh, origin[1], origin[2] + fwd * ch)
        t_segs = []
        if abs(fwd) > 0.05:
            ly, _ = surface_at(land_pt[0], land_pt[2], props, include_furniture=True)
            land_pt = (land_pt[0], ly, land_pt[2])
            t_segs = _ring_segments(land_pt[0], land_pt[2], 0.16, props, include_furniture=True)
        return {'kind': 'jump', 'origin': origin, 'normal': normal, 'setpoint': (fwd, 0.85, 0.0),
                'actual': (vx_ord, vy_jump, wz_ord), 'fill': fill, 'landing': land_pt,
                'frame_segments': f_segs, 'fill_segments': fl_segs, 'target_segments': t_segs}
    if doing in FALL_DOING or now.get('fallen'):
        seat_bed = ('sit_chair', 'approach', 'seated', 'lie_bed', 'sit_edge', 'recline', 'bed_rest')
        flr = ('lie_floor', 'kneel', 'lower', 'floor_rest')
        target_y = 0.45 if doing in seat_bed else (0.22 if (doing in flr or now.get('fallen')) else 0.58)
        t_fall = math.sqrt(2.0 * max(0.04, float(where[1]) - target_y) / 9.81)
        pitch_off = -0.18 if doing in seat_bed else (0.22 if doing in flr else 0.25)
        lx = float(where[0]) + float(vel_xz[0]) * t_fall + pitch_off * sh
        lz = float(where[2]) + float(vel_xz[1]) * t_fall + pitch_off * ch
        ly, l_norm = surface_at(lx, lz, props, include_furniture=True)
        land_pt = (lx, ly, lz)
        t_segs = (_ring_segments(lx, lz, 0.20, props, 8, True, True)
                  + _ring_segments(lx, lz, 0.10, props, 8, True, False))
        fill = max(0.25, min(1.0, (0.90 - float(where[1])) / max(0.12, 0.90 - target_y)))
        dx_b, dy_b = (lx - origin[0]) * sh + (lz - origin[2]) * ch, (lx - origin[0]) * ch - (lz - origin[2]) * sh
        f_segs, fl_segs = _surface_path_arrow(origin, heading, dx_b, dy_b, 0.0, fill, props)
        f_segs.append(((float(where[0]), float(where[1]), float(where[2])), land_pt))
        return {'kind': 'fall', 'origin': origin, 'normal': l_norm, 'setpoint': land_pt,
                'actual': tw_ord, 'fill': fill, 'landing': land_pt,
                'frame_segments': f_segs, 'fill_segments': fl_segs, 'target_segments': t_segs}
    if doing in ('crouch', 'down') or doing in RISE_DOING:
        is_rise = doing in RISE_DOING
        vy_p = 0.65 if is_rise else -0.65
        f_segs, fl_segs = _vertical_arrow(origin, 0.65, 1.0, 1.0 if is_rise else -1.0, 0.0, heading, props)
        return {'kind': 'posture', 'origin': origin, 'normal': normal, 'setpoint': (0.0, vy_p, 0.0),
                'actual': (0.0, vy_p, 0.0), 'fill': 1.0, 'landing': origin,
                'frame_segments': f_segs, 'fill_segments': fl_segs, 'target_segments': []}
    if doing in ('one_sole', 'weight_shift', 'over', 'bow', 'forward'):
        vec = (0.0, 0.45, 0.0) if doing in ('one_sole', 'weight_shift', 'over') else (0.45, 0.0, 0.0)
        f_segs, fl_segs = _surface_path_arrow(origin, heading, vec[0], vec[1], 0.0, 1.0, props)
        return {'kind': 'posture', 'origin': origin, 'normal': normal, 'setpoint': vec,
                'actual': vec, 'fill': 1.0, 'landing': origin,
                'frame_segments': f_segs, 'fill_segments': fl_segs, 'target_segments': []}
    return None


def describe(now, se2_st=None):
    """Compute 3D movement pattern indicator specification for `now` and `se2_st`.

    Setpoint frame (börvärdespil) shows target velocity for the active command (WASD walk vs
    Shift+WASD sprint); interior fill (ärvärde) ramps up as the robot's velocity reaches setpoint.
    """
    if not now:
        return {'kind': 'idle', 'fill': 0.0, 'frame_segments': [], 'fill_segments': [], 'target_segments': []}
    props = now.get('props') or ()
    origin = feet_origin(now, props)
    _, normal = surface_at(origin[0], origin[2], props)
    heading = float(now.get('heading', 0.0))
    doing = str(now.get('doing', 'rest'))
    vel_xz = now.get('velocity', (0.0, 0.0))
    sh, ch = math.sin(heading), math.cos(heading)
    keys = (se2_st or {}).get('keys') or {}
    any_key = any(keys.get(k, False) for k in ('w', 's', 'a', 'd'))
    if se2_st is not None and 'actual' in se2_st:
        tw_act = tuple(float(v) for v in se2_st['actual'])
    elif se2_st is not None:
        tw_act = (float(se2_st.get('v_x', 0.0)), float(se2_st.get('v_y', 0.0)), float(se2_st.get('omega_z', 0.0)))
    elif 'twist_actual' in now:
        tw_act = tuple(float(v) for v in (now.get('twist_actual') or (0.0, 0.0, 0.0)))
    else:
        tw_raw = now.get('twist') or (0.0, 0.0, 0.0)
        tw_act = (float(tw_raw[0]), float(tw_raw[1]), float(tw_raw[2]))
    if math.hypot(tw_act[0], tw_act[1]) <= 0.01 and abs(tw_act[2]) <= 0.01 and doing in GAIT_TWIST and not any_key and 'twist_actual' not in now:
        tw_act = GAIT_TWIST[doing]
    vx_act, vy_act, wz_act = tw_act
    act_mag = math.hypot(vx_act, vy_act)

    special = _describe_special(doing, now, origin, normal, heading, props, tw_act)
    if special is not None:
        return special

    spd_phys = math.hypot(float(vel_xz[0]), float(vel_xz[1]))
    if doing in STOP_DOING or (not any_key and doing in ('#201', 'rest') and (act_mag > 0.04 or spd_phys > 0.08)):
        vx_dir = vx_act if abs(vx_act) > 0.02 else 0.35
        stop_dist = max(0.32, min(0.75, 0.32 + 0.45 * max(act_mag, spd_phys)))
        fill = max(0.15, min(1.0, 1.0 - act_mag / se2.WALK_SPEED))
        f_segs, fl_segs = _surface_path_arrow(origin, heading, vx_dir, vy_act, 0.0, fill, props, length_scale=0.75)
        sx = origin[0] + stop_dist * sh * (1.0 if vx_dir >= 0 else -1.0)
        sz = origin[2] + stop_dist * ch * (1.0 if vx_dir >= 0 else -1.0)
        sy, _ = surface_at(sx, sz, props)
        lx_v, lz_v = ch * 0.22, -sh * 0.22
        t_segs = [((sx - lx_v, sy, sz - lz_v), (sx + lx_v, sy, sz + lz_v)),
                  ((sx - lx_v, sy + 0.02, sz - lz_v), (sx + lx_v, sy + 0.02, sz + lz_v))]
        t_segs += _ring_segments(sx, sz, 0.12, props, n=8, cross=False)
        return {'kind': 'stop', 'origin': origin, 'normal': normal, 'setpoint': (0.0, 0.0, 0.0),
                'actual': tw_act, 'fill': fill, 'landing': (sx, sy, sz),
                'frame_segments': f_segs, 'fill_segments': fl_segs, 'target_segments': t_segs}

    vx_set, vy_set, wz_set = 0.0, 0.0, 0.0
    if any_key:
        vx_set, vy_set, wz_set = (se2_st.get('setpoint') if se2_st and 'setpoint' in se2_st
                                  else se2.setpoint_for_keys(keys))
    elif doing in GAIT_TWIST:
        vx_set, vy_set, wz_set = GAIT_TWIST[doing]
    elif act_mag > 0.02 or abs(wz_act) > 0.03:
        vx_set, vy_set, wz_set = vx_act, vy_act, wz_act

    set_mag = math.hypot(vx_set, vy_set)
    if set_mag > 0.02 or abs(wz_set) > 0.03:
        if set_mag > 0.02 and abs(wz_set) > 0.03:
            fill = min(1.0, 0.5 * (act_mag / set_mag + abs(wz_act) / abs(wz_set)))
        elif set_mag > 0.02:
            fill = min(1.0, act_mag / set_mag)
        else:
            fill = min(1.0, abs(wz_act) / max(0.05, abs(wz_set)))
        fill = max(0.0, fill)
        f_segs, fl_segs = _surface_path_arrow(origin, heading, vx_set, vy_set, wz_set, fill, props)
        return {'kind': 'locomotion', 'origin': origin, 'normal': normal,
                'setpoint': (vx_set, vy_set, wz_set), 'actual': tw_act, 'fill': fill, 'landing': None,
                'frame_segments': f_segs, 'fill_segments': fl_segs, 'target_segments': []}

    return {'kind': 'idle', 'origin': origin, 'normal': normal, 'setpoint': (0.0, 0.0, 0.0),
            'actual': (0.0, 0.0, 0.0), 'fill': 0.0, 'landing': None,
            'frame_segments': [], 'fill_segments': [], 'target_segments': []}


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
    if arrows_in is None:
        return []
    if isinstance(arrows_in, tuple) and len(arrows_in) == 2:
        spec = describe(arrows_in[0], arrows_in[1])
    elif isinstance(arrows_in, dict) and 'frame_segments' in arrows_in:
        spec = arrows_in
    elif isinstance(arrows_in, dict):
        spec = describe(arrows_in, None)
    else:
        return []
    out = []
    kind = spec.get('kind', 'idle')
    f_dots = _rasterize(spec.get('frame_segments'), m, cam, centre, travel)
    if f_dots is not None:
        out.append((f_dots, STOP_INK if kind == 'stop' else FRAME_INK))
    t_dots = _rasterize(spec.get('target_segments'), m, cam, centre, travel)
    if t_dots is not None:
        out.append((t_dots, STOP_INK if kind == 'stop' else LAND_INK))
    fl_dots = _rasterize(spec.get('fill_segments'), m, cam, centre, travel)
    if fl_dots is not None:
        out.append((fl_dots, STOP_INK if kind == 'stop' else FILL_INK))
    return out
