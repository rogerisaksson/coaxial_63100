"""SE(2) body-relative 3rd-person locomotion controller: input polling, twist ramp, kinematics."""
import math

from machine import ortho

WALK_SPEED, RUN_SPEED, RAMP_ACCEL, DECAY_RATE = 0.69, 1.40, 1.95, 3.20
BACK_SPEED, BACK_SPRINT_SPEED = 0.35, 0.60
STRAFE_SPEED, STRAFE_SPRINT_SPEED, STRAFE_DECAY = 0.50, 0.80, 3.20
YAW_RATE, YAW_SPRINT_RATE, HOLD_WINDOW_S = 0.70, 1.15, 0.65

WASD_GAITS = ('#204', '#205', 'jog', 'strafe_left', 'strafe_right',
              'turn_left', 'turn_right', 'turn_back_left', 'turn_back_right', 'init_walk')

NOMINAL_TWIST = {
    '#204': (WALK_SPEED, 0.0, 0.0), 'walk': (WALK_SPEED, 0.0, 0.0),
    'init_walk': (WALK_SPEED, 0.0, 0.0), 'trip': (WALK_SPEED, 0.0, 0.0),
    'stairs_walk': (0.50, 0.0, 0.0), 'stairs_down': (0.50, 0.0, 0.0),
    'ice_walk': (0.35, 0.0, 0.0), 'slope_walk': (0.55, 0.0, 0.0),
    'ballistic_walk': (0.75, 0.0, 0.0), 'jog': (RUN_SPEED, 0.0, 0.0),
    '#205': (-BACK_SPEED, 0.0, 0.0),
    'strafe_left': (0.0, STRAFE_SPEED, 0.0), 'strafe_right': (0.0, -STRAFE_SPEED, 0.0),
    'turn_left': (WALK_SPEED, 0.0, YAW_RATE), 'turn_right': (WALK_SPEED, 0.0, -YAW_RATE),
    'circle_walk': (0.55, 0.0, 0.45),
    'turn_back_left': (-BACK_SPEED, 0.0, YAW_RATE), 'turn_back_right': (-BACK_SPEED, 0.0, -YAW_RATE),
}

_CHORDS = ('wa', 'wd', 'sa', 'sd', 'wq', 'we', 'sq', 'se', 'aq', 'ae', 'dq', 'de', 'ws', 'ad', 'qe')
_CHORD_EXPAND = {k: (c[0], c[1]) for c in _CHORDS for k in (c, c[::-1])}


def state(yaw=0.0, x=0.0, y=0.0):
    return {
        'v_x': 0.0, 'v_y': 0.0, 'omega_z': 0.0,
        'setpoint': (0.0, 0.0, 0.0), 'actual': (0.0, 0.0, 0.0),
        'yaw': float(yaw), 'x': float(x), 'y': float(y), 'v_wx': 0.0, 'v_wy': 0.0,
        'w_held': 0.0, 's_held': 0.0,
        'keys': {'w': False, 's': False, 'a': False, 'd': False, 'q': False, 'e': False, 'shift': False},
        'last_seen': {}, 'gait': None,
    }


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _approach(current, target, rate, dt):
    delta, limit = target - current, rate * dt
    return target if abs(delta) <= limit else current + math.copysign(limit, delta)


def _extract_tokens(raw):
    tokens = set()
    if raw is None:
        return tokens
    items = [raw] if isinstance(raw, str) else ([k for k, v in raw.items() if v] if isinstance(raw, dict) else list(raw))
    for item in items:
        if isinstance(item, str):
            s = item.strip()
            low = s.lower()
            if ((s.isupper() and (low in _CHORD_EXPAND or low in ('w', 's', 'a', 'd', 'q', 'e')))
                    or low in ('shift', 'sprint')):
                tokens.add('shift')
            if low in _CHORD_EXPAND:
                tokens.update(_CHORD_EXPAND[low])
            elif low in ('w', 's', 'a', 'd', 'q', 'e'):
                tokens.add(low)
    return tokens


def poll_input(raw, st=None, now=None, hold_window=HOLD_WINDOW_S, held=None):
    tokens = _extract_tokens(raw)
    if held is not None and (held or not tokens):
        active = _extract_tokens(held) | tokens
        if st is not None:
            st['last_seen'] = {tok: float(now) for tok in active} if (active and now is not None) else {}
    elif st is not None and now is not None:
        last_seen = st['last_seen']
        for k1, k2 in (('w', 's'), ('s', 'w'), ('a', 'd'), ('d', 'a'), ('q', 'e'), ('e', 'q')):
            if k1 in tokens and k2 not in tokens:
                last_seen.pop(k2, None)
        if tokens and 'shift' not in tokens and (tokens & {'w', 'a', 's', 'd', 'q', 'e'}):
            last_seen.pop('shift', None)
        if tokens & {'a', 'd', 'q', 'e'}:
            for lk in ('w', 's', 'a', 'd'):
                if lk in last_seen and (now - last_seen[lk]) <= hold_window:
                    last_seen[lk] = float(now)
        if tokens & {'w', 's'}:
            for ak in ('a', 'd', 'q', 'e'):
                if ak in last_seen and (now - last_seen[ak]) <= hold_window:
                    last_seen[ak] = float(now)
        for tok in tokens:
            last_seen[tok] = float(now)
        active = {k for k, ts in list(last_seen.items()) if (now - ts) <= hold_window}
    else:
        active = tokens
        if st is not None and not tokens:
            st['last_seen'].clear()
    w, s = 'w' in active and 's' not in active, 's' in active and 'w' not in active
    a, d = 'a' in active and 'd' not in active, 'd' in active and 'a' not in active
    q, e = 'q' in active and 'e' not in active, 'e' in active and 'q' not in active
    keys = {'w': w, 's': s, 'a': a, 'd': d, 'q': q, 'e': e,
            'shift': 'shift' in active and (w or s or a or d or q or e)}
    if st is not None:
        st['keys'] = keys
    return keys


def setpoint_for_keys(keys):
    w, s, a, d, q, e, shift = (bool(keys.get(k)) for k in ('w', 's', 'a', 'd', 'q', 'e', 'shift'))
    vx_set = ((RUN_SPEED if shift else WALK_SPEED) if (w and not s)
              else (-(BACK_SPRINT_SPEED if shift else BACK_SPEED) if (s and not w) else 0.0))
    longitudinal = w ^ s
    rate_z = YAW_SPRINT_RATE if shift else YAW_RATE
    wz_qe = rate_z if (q and not e) else (-rate_z if (e and not q) else 0.0)
    if not longitudinal and (a ^ d):
        spd_y = STRAFE_SPRINT_SPEED if shift else STRAFE_SPEED
        return (vx_set, spd_y if a else -spd_y, wz_qe)
    if longitudinal and (a ^ d):
        return (vx_set, 0.0, wz_qe if (q ^ e) else (rate_z if a else -rate_z))
    return (vx_set, 0.0, wz_qe)


def step_twist(st, keys, dt):
    dt = max(0.0, float(dt))
    w, s, a, d, q, e, shift = (bool(keys.get(k)) for k in ('w', 's', 'a', 'd', 'q', 'e', 'shift'))
    st['keys'] = {'w': w, 's': s, 'a': a, 'd': d, 'q': q, 'e': e, 'shift': shift}
    vx_set, vy_set, wz_set = setpoint_for_keys(st['keys'])
    st['setpoint'] = (vx_set, vy_set, wz_set)
    vx_act, vy_act, wz_act = st.get('actual', (0.0, 0.0, 0.0))
    vx_act = 0.0 if vx_set * vx_act < 0.0 else vx_act
    vy_act = 0.0 if vy_set * vy_act < 0.0 else vy_act
    wz_act = 0.0 if wz_set * wz_act < 0.0 else wz_act
    st['actual'] = (
        _approach(vx_act, vx_set, RAMP_ACCEL if abs(vx_set) >= abs(vx_act) else DECAY_RATE, dt),
        _approach(vy_act, vy_set, 2.40 if abs(vy_set) >= abs(vy_act) else STRAFE_DECAY, dt),
        _approach(wz_act, wz_set, 2.50 if abs(wz_set) >= abs(wz_act) else STRAFE_DECAY, dt),
    )
    if w and not s:
        st['w_held'], st['s_held'] = st['w_held'] + dt, 0.0
        st['v_x'] = (min(RUN_SPEED, _approach(max(WALK_SPEED, st['v_x']), RUN_SPEED, RAMP_ACCEL, dt)) if shift
                     else (WALK_SPEED if st['v_x'] <= WALK_SPEED else _approach(st['v_x'], WALK_SPEED, DECAY_RATE, dt)))
    elif s and not w:
        st['s_held'], st['w_held'] = st['s_held'] + dt, 0.0
        st['v_x'] = max(-BACK_SPRINT_SPEED, _approach(min(-BACK_SPEED, st['v_x']), -BACK_SPRINT_SPEED, RAMP_ACCEL, dt)) if shift else -BACK_SPEED
    else:
        st['w_held'] = st['s_held'] = 0.0
        st['v_x'] = _approach(st['v_x'], 0.0, DECAY_RATE, dt)
    if not (w ^ s) and (a ^ d):
        st['v_x'], st['omega_z'] = 0.0, wz_set
        st['v_y'] = _approach(max(STRAFE_SPEED, st['v_y']) if a else min(-STRAFE_SPEED, st['v_y']), vy_set, STRAFE_DECAY, dt) if shift else vy_set
    elif (w ^ s) and ((a ^ d) or (q ^ e)):
        st['v_y'], st['omega_z'] = 0.0, wz_set
    elif not (w ^ s) and (q ^ e):
        st['v_x'], st['v_y'], st['omega_z'] = 0.0, _approach(st['v_y'], 0.0, STRAFE_DECAY, dt), wz_set
    else:
        st['v_y'], st['omega_z'] = _approach(st['v_y'], 0.0, STRAFE_DECAY, dt), 0.0
    st['gait'] = gait_for_twist(st)
    return (st['v_x'], st['v_y'], st['omega_z'])


def body_to_world(v_x, v_y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return (v_x * c - v_y * s, v_x * s + v_y * c)


def body_to_mujoco(v_x, v_y, heading):
    c, s = math.cos(heading), math.sin(heading)
    return (v_x * s + v_y * c, v_x * c - v_y * s)


def step_kinematics(st, dt):
    dt = max(0.0, float(dt))
    st['yaw'] = wrap_angle(st['yaw'] + st['omega_z'] * dt)
    v_wx, v_wy = body_to_world(st['v_x'], st['v_y'], st['yaw'])
    st['v_wx'], st['v_wy'] = v_wx, v_wy
    st['x'] += v_wx * dt
    st['y'] += v_wy * dt
    return (st['x'], st['y'], st['yaw'])


def gait_for_twist(st, v_y=0.0, omega_z=0.0, shift=False):
    if not isinstance(st, dict):
        st = {'v_x': float(st), 'v_y': float(v_y), 'omega_z': float(omega_z),
              'keys': None, 'shift': bool(shift)}
    keys = st.get('keys')
    if keys is not None:
        w, s, a, d, q, e = (bool(keys.get(k)) for k in ('w', 's', 'a', 'd', 'q', 'e'))
        w, s, a, d, q, e = w and not s, s and not w, a and not d, d and not a, q and not e, e and not q
        if w and (a or d or q or e):
            return 'turn_left' if (q if (q or e) else a) else 'turn_right'
        if s and (a or d or q or e):
            return 'turn_back_left' if (q if (q or e) else a) else 'turn_back_right'
        if w:
            return 'jog' if st.get('v_x', 0.0) > 0.96 else '#204'
        if s:
            return '#205'
        if a or d:
            return 'strafe_left' if a else 'strafe_right'
        if q or e:
            return 'turn_left' if q else 'turn_right'
        if 'setpoint' in st:
            return None
    vx, vy, wz = st.get('v_x', 0.0), st.get('v_y', 0.0), st.get('omega_z', 0.0)
    if abs(vy) > 0.05:
        return 'strafe_left' if vy > 0 else 'strafe_right'
    if vx > 0.05 and abs(wz) > 0.05:
        return 'turn_left' if wz > 0 else 'turn_right'
    if vx < -0.05 and abs(wz) > 0.05:
        return 'turn_back_left' if wz > 0 else 'turn_back_right'
    if abs(vx) <= 0.05 and abs(wz) > 0.05:
        return 'turn_left' if wz > 0 else 'turn_right'
    fast = vx > (0.96 if 'setpoint' in st else WALK_SPEED + 0.05) or (st.get('shift') and vx > 0.05)
    return 'jog' if fast else ('#204' if vx > 0.05 else ('#205' if vx < -0.05 else None))


def step_controller(st, raw_input, dt, now=None, hold_window=HOLD_WINDOW_S, held=None):
    step_twist(st, poll_input(raw_input, st=st, now=now, hold_window=hold_window, held=held), dt)
    step_kinematics(st, dt)
    return st


def _modulate_cpg(g, vx, vy, wz, out, vxp=0.0):
    from machine import latent, modal
    if g['st'][latent.NOW][latent.GAIN] < 0.05:
        if vx > WALK_SPEED + 0.01 and abs(wz) < 0.05 and abs(vy) < 0.01:
            g['st'][latent.TO][latent.AMP][modal.DRIVE] = max(float(g['st'][latent.TO][latent.AMP][modal.DRIVE]), 0.60)
        return
    spin = abs(vx) < 0.05 and abs(vy) < 0.05 and abs(wz) > 0.05
    ts_raw = (0.28 if abs(vy) > 0.05 else 0.40) * max(-1.0, min(1.0, wz / YAW_RATE)) if abs(wz) > 0.03 else 0.0
    ts = g['_turn_dc'] = g.get('_turn_dc', 0.0) + 0.18 * (ts_raw - g.get('_turn_dc', 0.0))
    if out is not None and (abs(wz) > 0.03 or abs(ts) > 0.01):
        if spin:
            bt_d, w_d = float(g['st'][latent.THETA] + g['st'][latent.NOW][latent.PHASE][modal.DRIVE]), float(out[latent.TURNS])
            drv = 0.035 if abs(vxp) < 0.22 else max(0.035, float(out[latent.SWAY][modal.DRIVE]) * min(1.0, abs(vxp) / 0.50))
            out[latent.SWAY][modal.DRIVE], out[latent.REF][modal.DRIVE] = drv, drv * math.sin(bt_d)
            out[latent.RATE][modal.DRIVE], out[latent.ACC][modal.DRIVE] = drv * w_d * math.cos(bt_d), -drv * w_d * w_d * math.sin(bt_d)
            out[latent.SWAY][modal.LIFT] = max(float(out[latent.SWAY][modal.LIFT]), 0.95)
        amp_t, bt, w = (abs(ts) if abs(vy) > 0.05 else 0.14), float(g['st'][latent.THETA] + g['st'][latent.NOW][latent.PHASE][modal.TURN]), float(out[latent.TURNS])
        out[latent.SWAY][modal.TURN], out[latent.REF][modal.TURN] = amp_t, ts + amp_t * math.sin(bt)
        out[latent.RATE][modal.TURN], out[latent.ACC][modal.TURN] = amp_t * w * math.cos(bt), -amp_t * w * w * math.sin(bt)
    if abs(vx) > WALK_SPEED + 0.01 or abs(vy) > STRAFE_SPEED + 0.01 or vx < -(BACK_SPEED + 0.01) or abs(wz) > 0.05:
        hz_m = 0.98 if (spin or abs(wz) > YAW_RATE + 0.01 or abs(vx) > WALK_SPEED + 0.01) else 0.95
        g['st'][latent.TO][latent.HZ] = max(float(g['st'][latent.TO][latent.HZ]), hz_m)
        g['st'][latent.NOW][latent.HZ] = max(float(g['st'][latent.NOW][latent.HZ]), hz_m)
        if out is not None:
            out[latent.TURNS] = math.tau * float(g['st'][latent.NOW][latent.HZ])
            if vx > WALK_SPEED + 0.01 and abs(wz) < 0.05 and abs(vy) < 0.01:
                out[latent.SWAY][modal.DRIVE] = min(0.68, float(out[latent.SWAY][modal.DRIVE]) * 1.08)
                out[latent.SWAY][modal.ARMS] = min(1.00, float(out[latent.SWAY][modal.ARMS]) * 1.08)
            elif vx < -(BACK_SPEED + 0.01) and abs(vy) < 0.01:
                out[latent.SWAY][modal.DRIVE] = min(0.28, float(out[latent.SWAY][modal.DRIVE]) * 1.25)
            if abs(vy) > STRAFE_SPEED + 0.01:
                out[latent.REF][modal.LATERAL] = math.copysign(0.105, vy)


def step_physical_twist(g, dt, out=None):
    from machine import floor, latent, modal, sprung
    y, f0, cur_entry = g['y'], g.setdefault('floor0', float(g['fr'][modal.FLOOR])), g.get('active_entry')
    d, hd = g['world'].data, float(g['heading'])
    sh, ch = math.sin(hd), math.cos(hd)
    if cur_entry in ('hop_up_chair', 'hop_down_chair') and min(float(y[modal.LOWEST][0]), float(y[modal.LOWEST][1])) > 0.025:
        pos = (float(d.qpos[0]), float(d.qpos[2]))
        if cur_entry == 'hop_up_chair' and not g.get('_chair_elevated'):
            g['_chair_elevated'] = True
            floor.mocap(g['world'], 'chair', floor._on(pos, hd, 0.0, 0.020 - floor.CHAIR_SEAT_H, -0.15), hd + math.pi)
        elif cur_entry == 'hop_down_chair' and max(g['loads']) == 0.0 and not g.get('_chair_cleared'):
            g['_chair_cleared'], g['_chair_elevated'] = True, False
            floor.mocap(g['world'], 'chair', floor._on(pos, hd, 0.0, 0.002 - floor.CHAIR_SEAT_H, -0.45), hd + math.pi)
    sy = min(float(y[modal.SOLE][1]), float(y[modal.SOLE][4]))
    g['fr'][modal.FLOOR] += 15.0 * dt * ((min(f0, sy) if sy < f0 - 0.006 else f0) - float(g['fr'][modal.FLOOR]))
    cur_doing = sprung.doing(g)
    twist_mode = bool(g.get('_twist_active') or g.get('_twist_stop_until', 0.0) > g['t'] or cur_entry in WASD_GAITS or cur_doing == 'brake')
    is_hop = cur_doing in ('load', 'throw', 'land') and cur_entry not in ('controlled_fall', 'hop_up_chair', 'hop_down_chair')
    vx, vy, wz = g.get('twist', (0.0, 0.0, 0.0))
    tx, ty, tz = (vx, vy, wz) if g.get('_twist_active') else NOMINAL_TWIST.get(cur_entry, (0.0, 0.0, 0.0))
    vxp, vyp, wzp = float(d.qvel[0]) * sh + float(d.qvel[2]) * ch, float(d.qvel[0]) * ch - float(d.qvel[2]) * sh, float(d.qvel[4])
    dt_ld = (g['t'] - g['_land_touch_t']) if g.get('_land_touch_t') is not None else -1.0
    if out is not None and is_hop and not (g.get('_leap') and dt_ld > 0.06):
        out[latent.SWAY][:] = 0.0
        for m in (modal.DRIVE, modal.LIFT, modal.LATERAL, modal.TURN, modal.LIST):
            out[latent.REF][m] = out[latent.RATE][m] = out[latent.ACC][m] = 0.0
        out[latent.REF][modal.WIDTH] = min(float(out[latent.REF][modal.WIDTH]), 0.04)
        out[latent.RATE][modal.WIDTH] = out[latent.ACC][modal.WIDTH] = 0.0
        if max(g['loads']) <= 10.0 and cur_doing in ('throw', 'land'):
            out[latent.REF][modal.AXIAL] = min(1.10, max(0.20, 0.70 + 0.40 * float(d.qvel[1])))
            out[latent.KS][modal.LIFT], out[latent.DS][modal.LIFT] = 680.0, 60.0
    ax, ay, az = g.get('twist_actual', (0.0, 0.0, 0.0))
    kv, ky, kz = min(1.0, dt / 0.06), min(1.0, dt / (0.07 if abs(ty) > 0.01 else 0.10)), min(1.0, dt / (0.07 if abs(tz) > 0.01 else 0.10))
    g['twist_actual'] = (
        ax + kv * (vxp - ax),
        ay + ky * ((vyp if abs(ty) > 0.01 or g['world'].push_until > g['t'] else 0.0) - ay),
        az + kz * ((wzp if abs(tz) > 0.01 else 0.0) - az),
    )
    if g.get('_jump_until', 0.0) > g['t']:
        return
    q = g['world'].data.qpos[3:7]
    pelvis_yaw = 2.0 * math.atan2(float(q[2]), float(q[0]))
    if abs(tz) > 1e-6 and g.get('target_heading') is None:
        l0, l1 = float(g['loads'][0]), float(g['loads'][1])
        out_st = 0.45 if ((tz > 0.0 and l0 > l1 + 60.0) or (tz < 0.0 and l1 > l0 + 60.0)) else 1.05
        d_yaw = wrap_angle(pelvis_yaw - g['heading'])
        if d_yaw * math.copysign(1.0, tz) >= -0.10:
            g['heading'] = wrap_angle(g['heading'] + out_st * tz * dt)
        if d_yaw * tz > 0.0:
            g['heading'] = wrap_angle(g['heading'] + 10.0 * d_yaw * dt)
    if twist_mode and g.get('target_heading') is None and cur_doing not in sprung.STUNT_PHASES:
        if tx >= 0.05 and abs(ty) < 0.05 and math.hypot(float(y[modal.VCOM][0]), float(y[modal.VCOM][2])) > 0.32 and vxp > 0.15:
            err_v = wrap_angle(math.atan2(float(y[modal.VCOM][0]), float(y[modal.VCOM][2])) - g['heading'])
            if abs(err_v) > 0.32:
                g['heading'] = wrap_angle(g['heading'] + 12.0 * (err_v - math.copysign(0.32, err_v)) * dt)
        err_yaw = wrap_angle(pelvis_yaw - g['heading'])
        if abs(err_yaw) > 0.10:
            g['heading'] = wrap_angle(pelvis_yaw - math.copysign(0.10, err_yaw))
    sh_n, ch_n = math.sin(g['heading']), math.cos(g['heading'])
    g['fr'][modal.HEADING] = g['heading']
    g['fr'][modal.ROT][:] = (sh_n, 0.0, ch_n, ch_n, 0.0, -sh_n, 0.0, 1.0, 0.0)
    if g.get('_twist_active'):
        _modulate_cpg(g, vx, vy, wz, out, vxp)
    if out is not None and not is_hop:
        from machine import worker
        worker.check_parry(g, out, cur_doing, cur_entry, sh_n, ch_n)


def boost_stumble(g, out):
    from machine import latent, modal
    trip = g['t'] < g.get('stumble_until', 0.0)
    hz0, gain0 = float(g['st'][latent.NOW][latent.HZ]), float(g['st'][latent.NOW][latent.GAIN])
    if (trip and hz0 < 1.15) or (not trip and (hz0 < 0.85 or gain0 < 0.1)):
        g['st'][latent.THETA] = (float(g['st'][latent.THETA]) + math.tau * ((1.15 if trip else 1.0) - hz0 * gain0) * 0.001 * g['every']) % math.tau
    w = max(float(out[latent.TURNS]), math.tau * (1.15 if trip else 1.0))
    out[latent.TURNS], out[latent.HELD] = w, 2.0
    lat_ph = -1.885 if (g.get('active_entry') in ('#205', 'turn_back_left', 'turn_back_right') or g.get('twist', (0.0,))[0] < -0.05) else 1.885
    if trip or abs(float(g['st'][latent.NOW][latent.PHASE][modal.LATERAL])) < 0.1:
        for part in (latent.NOW, latent.TO):
            if abs(float(g['st'][part][latent.PHASE][modal.LATERAL])) < 0.1:
                g['st'][part][latent.PHASE][modal.LATERAL] = g['st'][part][latent.PHASE][modal.LIST] = lat_ph
    psi_d = float(g['st'][latent.THETA] + g['st'][latent.NOW][latent.PHASE][modal.DRIVE])
    psi_l = float(g['st'][latent.THETA] + g['st'][latent.NOW][latent.PHASE][modal.LIFT])
    psi_lat = float(g['st'][latent.THETA] + g['st'][latent.NOW][latent.PHASE][modal.LATERAL])
    out[latent.PSI][modal.DRIVE], out[latent.PSI][modal.LIFT], out[latent.PSI][modal.LATERAL] = psi_d, psi_l, psi_lat
    drv = min(0.50 if trip else 0.70, max(0.32, float(out[latent.SWAY][modal.DRIVE])))
    out[latent.SWAY][modal.DRIVE], out[latent.REF][modal.DRIVE] = drv, drv * math.sin(psi_d)
    out[latent.RATE][modal.DRIVE], out[latent.ACC][modal.DRIVE] = drv * w * math.cos(psi_d), -drv * w * w * math.sin(psi_d)
    out[latent.SWAY][modal.LATERAL] = max(0.28, float(out[latent.SWAY][modal.LATERAL]))
    sw0, l_tgt = float(out[latent.SWAY][modal.LIFT]), (2.6 if trip else 1.15)
    if sw0 < l_tgt:
        d_sw = l_tgt - sw0
        out[latent.SWAY][modal.LIFT] = l_tgt
        out[latent.REF][modal.LIFT] += d_sw * math.sin(psi_l)
        out[latent.RATE][modal.LIFT] += d_sw * w * math.cos(psi_l)
        out[latent.ACC][modal.LIFT] -= d_sw * w * w * math.sin(psi_l)
    if g.get('_twist_active') or g.get('_twist_stop_until', 0.0) > g['t']:
        out[latent.REF][modal.WIDTH] = max(float(out[latent.REF][modal.WIDTH]), 0.16)


def align_chair_hop(g, name):
    if name in ('hop_up_chair', 'hop_down_chair'):
        from machine import floor
        d, hd = g['world'].data, float(g['heading'])
        h0 = 0.020 if (name == 'hop_down_chair' and g.get('_chair_elevated')) else 0.002
        g['_chair_elevated'], g['_chair_cleared'] = (h0 > 0.01), False
        floor.mocap(g['world'], 'chair', floor._on((float(d.qpos[0]), float(d.qpos[2])), hd, 0.0, h0 - floor.CHAIR_SEAT_H, -0.15), hd + math.pi)


def heard_twist(g, gym, tw, has_say, said_fn, dropped_fn, heard_say_fn, walk_on):
    from machine import latent
    pvx, pvy, _ = g.get('twist', (0.0, 0.0, 0.0))
    g['twist'] = (0.0, 0.0, 0.0) if tw is None else (float(tw[0]), float(tw[1]), float(tw[2]))
    vx, vy, wz = g['twist']
    by, jumping = {e[0]: e for e in gym}, g.get('_jump_until', 0.0) > g['t']
    decay = (not has_say and math.hypot(vx, vy) < math.hypot(pvx, pvy) - 1e-4
             and abs(vx) < WALK_SPEED - 0.01 and abs(vy) < STRAFE_SPEED - 0.01 and abs(wz) < 0.01)
    if (abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01) and not decay:
        label = gait_for_twist({'v_x': vx, 'v_y': vy, 'omega_z': wz, 'setpoint': ()}) or ('#205' if vx < -0.01 else '#204')
        g['_twist_active'], g['_twist_stop_until'] = True, g['t'] + 2.5
        if jumping:
            if not has_say and label in by and g.get('active_entry') != label:
                heard_say_fn(g, gym, by[label])
            return
        idle = float(g['st'][55]) < 1e-6 and not bool(len(g['tab']) and (g['tab'][:, -2] >= g['t']).any())
        if abs(vx) < 0.05 and abs(vy) < 0.05 and abs(wz) > 0.05 and any(n == 'init_walk' for _, n in g.get('labels', ())):
            dropped_fn(g)
            if label in by:
                said_fn(g, *by[label], at=g['t'] + 0.001)
            g['active_entry'] = label
        elif not has_say and label in by and (g.get('active_entry') != label or idle):
            heard_say_fn(g, gym, by[label])
            g['active_entry'] = label
    elif g.get('_twist_active') or g.get('active_entry') in WASD_GAITS:
        g['_twist_active'], g['_twist_stop_until'] = False, g['t'] + 2.5
        if jumping:
            g['tab'] = g['tab'][g['tab'][:, latent.AT] <= g['_jump_until'] + 0.01]
            g['labels'] = [(w, n) for w, n in g['labels'] if w <= g['_jump_until'] + 0.01]
            if '#201' in by:
                said_fn(g, *by['#201'], at=g['_jump_until'] + 0.02, reset=True)
            g['active_entry'] = '#201'
            return
        if not has_say and '#201' in by:
            heard_say_fn(g, gym, by['#201'])


WALK_ON = ('(stream_inject (sequence_id 204) (blend_duration 650ms) (latent_targets (mode_axial_compression 0.065)'
           ' (mode_pitch_attitude 0.01) (mode_sagittal_shift 0.0) (mode_lateral_shift 0.0) (mode_pelvis_turn 0.0)'
           ' (mode_stance_width 0.0)) (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)'
           ' (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 300) (stiffness mode_sagittal_shift 0)'
           ' (damping mode_sagittal_shift 260) (stiffness mode_foot_rock 2400) (damping mode_foot_rock 240))'
           ' (cpg_config (enable true) (frequency 0.94Hz) (amplitude mode_lateral_shift 0.28)'
           ' (amplitude mode_sagittal_drive 0.605) (amplitude mode_limb_lift 1.05) (amplitude mode_foot_rock 0.62)'
           ' (amplitude mode_pelvis_list 0.35) (amplitude mode_pelvis_turn 0.40) (amplitude mode_arm_swing 0.88)'
           ' (amplitude mode_stance_width 0.0) (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)'
           ' (phase_offset mode_pelvis_list mode_sagittal_drive 108deg) (phase_offset mode_pelvis_turn mode_sagittal_drive 0deg)'
           ' (phase_offset mode_arm_swing mode_sagittal_drive 0deg) (phase_offset mode_foot_rock mode_sagittal_drive 0deg))'
           ' (task_constraints (maintain_zmp (polygon dynamic_footprint))))')
_WALK_ON_WIRE = ortho.compiled(WALK_ON)[0]['']


def stride_hop(g, by, t0, spd, back_now, fwd_gaits, back_gaits, said_fn):
    from machine import flight
    return flight.stride_hop(g, by, t0, spd, back_now, fwd_gaits, back_gaits, said_fn, _WALK_ON_WIRE)
