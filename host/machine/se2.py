"""SE(2) body-relative 3rd-person locomotion controller: input polling, twist ramp, kinematics."""
import math

from machine import ortho

WALK_SPEED, RUN_SPEED, RAMP_ACCEL, DECAY_RATE = 0.69, 1.40, 1.80, 2.00
BACK_SPEED, BACK_SPRINT_SPEED = 0.35, 0.60
STRAFE_SPEED, STRAFE_SPRINT_SPEED, STRAFE_DECAY = 0.50, 0.80, 3.20
YAW_RATE, YAW_SPRINT_RATE, HOLD_WINDOW_S = 0.70, 0.95, 0.65

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
        _approach(vy_act, vy_set, 0.70 if abs(vy_set) >= abs(vy_act) else STRAFE_DECAY, dt),
        _approach(wz_act, wz_set, 0.75 if abs(wz_set) >= abs(wz_act) else STRAFE_DECAY, dt),
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
    if g['st'][latent.NOW][latent.GAIN] <= 0.45:
        if vx > WALK_SPEED + 0.01 and abs(wz) < 0.05 and abs(vy) < 0.01:
            g['st'][latent.TO][latent.AMP][modal.DRIVE] = max(float(g['st'][latent.TO][latent.AMP][modal.DRIVE]), 0.60)
        return
    spin = abs(vx) < 0.05 and abs(vy) < 0.05 and abs(wz) > 0.05
    if out is not None and (spin or (abs(vy) > 0.05 and abs(wz) > 0.05)):
        if spin:
            out[latent.SWAY][modal.DRIVE] = out[latent.REF][modal.DRIVE] = 0.0
            out[latent.RATE][modal.DRIVE] = out[latent.ACC][modal.DRIVE] = 0.0
            out[latent.SWAY][modal.LIFT] = min(float(out[latent.SWAY][modal.LIFT]), 0.65)
        ts = math.copysign(0.32 if spin else 0.24, wz)
        bt = float(g['st'][latent.THETA] + g['st'][latent.NOW][latent.PHASE][modal.TURN])
        w = float(out[latent.TURNS])
        out[latent.SWAY][modal.TURN], out[latent.REF][modal.TURN] = abs(ts), ts * (1.0 + math.sin(bt))
        out[latent.RATE][modal.TURN], out[latent.ACC][modal.TURN] = ts * w * math.cos(bt), -ts * w * w * math.sin(bt)
    if (abs(vx) > WALK_SPEED + 0.01 or abs(vy) > STRAFE_SPEED + 0.01
            or vx < -(BACK_SPEED + 0.01) or abs(wz) > YAW_RATE + 0.01):
        g['st'][latent.TO][latent.HZ] = max(float(g['st'][latent.TO][latent.HZ]), 0.95)
        g['st'][latent.NOW][latent.HZ] = max(float(g['st'][latent.NOW][latent.HZ]), 0.95)
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
    from machine import floor, modal
    y, f0, cur_entry = g['y'], g.setdefault('floor0', float(g['fr'][modal.FLOOR])), g.get('active_entry')
    d, hd = g['world'].data, float(g['heading'])
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
    vx, vy, wz = g.get('twist', (0.0, 0.0, 0.0))
    tx, ty, tz = (vx, vy, wz) if g.get('_twist_active') else NOMINAL_TWIST.get(cur_entry, (0.0, 0.0, 0.0))
    d, hd = g['world'].data, float(g['heading'])
    sh, ch = math.sin(hd), math.cos(hd)
    vxp, vyp, wzp = float(d.qvel[0]) * sh + float(d.qvel[2]) * ch, float(d.qvel[0]) * ch - float(d.qvel[2]) * sh, float(d.qvel[4])
    if abs(tx) < 0.05 and abs(ty) > 0.05 and vxp > 0.25:
        d.qvel[0], d.qvel[2] = float(d.qvel[0]) - 0.05 * vxp * sh, float(d.qvel[2]) - 0.05 * vxp * ch
    ax, ay, az = g.get('twist_actual', (0.0, 0.0, 0.0))
    kv, ky, kz = min(1.0, dt / 0.12), min(1.0, dt / (0.15 if abs(ty) > 0.01 else 0.45)), min(1.0, dt / (0.15 if abs(tz) > 0.01 else 0.45))
    g['twist_actual'] = (
        ax + kv * (vxp - ax),
        ay + ky * ((vyp if abs(ty) > 0.01 or g['world'].push_until > g['t'] else 0.0) - ay),
        az + kz * ((wzp if abs(tz) > 0.01 else 0.0) - az),
    )
    if g.get('_jump_until', 0.0) > g['t']:
        return
    if abs(tz) > 1e-6 and g.get('target_heading') is None:
        q = g['world'].data.qpos[3:7]
        pelvis_yaw = 2.0 * math.atan2(float(q[2]), float(q[0]))
        d_yaw = wrap_angle(pelvis_yaw - g['heading'])
        lead = 0.42 if (abs(tx) < 0.05 and abs(ty) < 0.05) else (0.30 if abs(ty) > 0.05 else 0.22)
        if d_yaw * tz >= -0.02 and abs(d_yaw) < 0.14:
            g['heading'] = wrap_angle(g['heading'] + lead * tz * dt)
        if d_yaw * tz > 0.0:
            gain = 5.5 if abs(tz) > YAW_RATE + 0.01 else 4.5
            g['heading'] = wrap_angle(g['heading'] + gain * d_yaw * dt)
    if g.get('_twist_active'):
        _modulate_cpg(g, vx, vy, wz, out, vxp)


def boost_stumble(g, out):
    from machine import latent, modal
    sw0 = float(out[latent.SWAY][modal.LIFT])
    if sw0 < 2.6:
        d_sw, bt, w = 2.6 - sw0, float(g['st'][latent.THETA] + g['st'][latent.NOW][latent.PHASE][modal.LIFT]), float(out[latent.TURNS])
        out[latent.SWAY][modal.LIFT] = 2.6
        out[latent.REF][modal.LIFT] += d_sw * math.sin(bt)
        out[latent.RATE][modal.LIFT] += d_sw * w * math.cos(bt)
        out[latent.ACC][modal.LIFT] -= d_sw * w * w * math.sin(bt)
    out[latent.SWAY][modal.DRIVE] = min(float(out[latent.SWAY][modal.DRIVE]), 0.50)


def align_chair_hop(g, name):
    if name in ('hop_up_chair', 'hop_down_chair'):
        from machine import floor
        d, hd = g['world'].data, float(g['heading'])
        h0 = 0.020 if (name == 'hop_down_chair' and g.get('_chair_elevated')) else 0.002
        g['_chair_elevated'], g['_chair_cleared'] = (h0 > 0.01), False
        floor.mocap(g['world'], 'chair', floor._on((float(d.qpos[0]), float(d.qpos[2])), hd, 0.0, h0 - floor.CHAIR_SEAT_H, -0.15), hd + math.pi)


def heard_twist(g, gym, tw, has_say, said_fn, dropped_fn, heard_say_fn, walk_on):
    from machine import latent
    prev_vx, prev_vy, _ = g.get('twist', (0.0, 0.0, 0.0))
    g['twist'] = (0.0, 0.0, 0.0) if tw is None else (float(tw[0]), float(tw[1]), float(tw[2]))
    vx, vy, wz = g['twist']
    by = {e[0]: e for e in gym}
    jumping = g.get('_jump_until', 0.0) > g['t']
    decaying_to_zero = (not has_say and math.hypot(vx, vy) < math.hypot(prev_vx, prev_vy) - 1e-4
                        and abs(vx) < WALK_SPEED - 0.01 and abs(vy) < STRAFE_SPEED - 0.01 and abs(wz) < 0.01)
    if (abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01) and not decaying_to_zero:
        label = gait_for_twist({'v_x': vx, 'v_y': vy, 'omega_z': wz, 'setpoint': ()}) or '#204'
        g['_twist_active'] = True
        if jumping:
            g['active_entry'] = label
            return
        idle_rest = float(g['st'][55]) < 1e-6 and not bool(len(g['tab']) and (g['tab'][:, -2] >= g['t']).any())
        if not has_say and label in by and (g.get('active_entry') != label or idle_rest):
            heard_say_fn(g, gym, by[label])
            g['active_entry'] = label
    elif g.get('_twist_active') or g.get('active_entry') in WASD_GAITS:
        g['_twist_active'] = False
        if jumping:
            g['tab'] = g['tab'][g['tab'][:, latent.AT] <= g['_jump_until'] + 0.01]
            g['labels'] = [(w, n) for w, n in g['labels'] if w <= g['_jump_until'] + 0.01]
            if '#201' in by:
                said_fn(g, *by['#201'], at=g['_jump_until'] + 0.02, reset=True)
            g['active_entry'] = '#201'
            return
        if not has_say and '#201' in by:
            heard_say_fn(g, gym, by['#201'])


WALK_ON = ('(stream_inject (sequence_id 204) (blend_duration 650ms)'
           ' (latent_targets (mode_axial_compression 0.065) (mode_pitch_attitude 0.01)'
           ' (mode_sagittal_shift 0.0) (mode_lateral_shift 0.0) (mode_pelvis_turn 0.0) (mode_stance_width 0.0))'
           ' (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)'
           ' (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 300)'
           ' (stiffness mode_sagittal_shift 0) (damping mode_sagittal_shift 260)'
           ' (stiffness mode_foot_rock 2400) (damping mode_foot_rock 240))'
           ' (cpg_config (enable true) (frequency 0.94Hz)'
           ' (amplitude mode_lateral_shift 0.28) (amplitude mode_sagittal_drive 0.605)'
           ' (amplitude mode_limb_lift 1.05) (amplitude mode_foot_rock 0.62)'
           ' (amplitude mode_pelvis_list 0.35) (amplitude mode_pelvis_turn 0.40)'
           ' (amplitude mode_arm_swing 0.88) (amplitude mode_stance_width 0.0)'
           ' (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)'
           ' (phase_offset mode_pelvis_list mode_sagittal_drive 108deg)'
           ' (phase_offset mode_pelvis_turn mode_sagittal_drive 0deg)'
           ' (phase_offset mode_arm_swing mode_sagittal_drive 0deg)'
           ' (phase_offset mode_foot_rock mode_sagittal_drive 0deg))'
           ' (task_constraints (maintain_zmp (polygon dynamic_footprint))))')
SPRINT_APA = WALK_ON.replace('(blend_duration 650ms)', '(blend_duration 280ms)').replace('0.94Hz', '0.96Hz')
_HOP_S = ('(stream_inject (sequence_id %d) (blend_duration %dms)'
          ' (latent_targets (mode_axial_compression %s) (mode_pitch_attitude 0.0))'
          ' (impedance (stiffness mode_axial_compression %d) (damping mode_axial_compression %d))'
          ' (cpg_config (amplitude mode_sagittal_drive %s) (amplitude mode_limb_lift %s)))')
_LOAD_FAST, _LOAD_SLOW, _THROW_FAST, _THROW_SLOW, _LAND_WIRE = (
    ortho.compiled(_HOP_S % p)[0][''] for p in (
        (291, 180, '0.14', 3800, 650, '0.32', '0.50'),
        (291, 180, '0.14', 3800, 650, '0.32', '0.45'),
        (292, 140, '-0.06', 3800, 150, '0.36', '0.40'),
        (292, 140, '-0.08', 4100, 150, '0.36', '0.35'),
        (293, 250, '0.04', 3000, 600, '0.50', '0.95')))
_WALK_ON_WIRE = ortho.compiled(WALK_ON)[0]['']
_SPRINT_APA_WIRE = ortho.compiled(SPRINT_APA)[0]['']


def stride_hop(g, by, t0, spd, back_now, fwd_gaits, back_gaits, said_fn):
    prev = (g.get('active_entry') if g.get('active_entry') in (fwd_gaits + back_gaits)
            else ('#205' if back_now else '#204'))
    fast = prev == 'jog' or spd > 0.65
    for lbl, dt_at, wire in (
        ('load', 0.0, _LOAD_FAST if fast else _LOAD_SLOW),
        ('throw', 0.20, _THROW_FAST if fast else _THROW_SLOW),
        ('land', 0.38, _LAND_WIRE),
    ):
        said_fn(g, lbl, wire, (lbl,), at=t0 + dt_at)
    t_res = t0 + 0.70
    g['_jump_until'], g['active_entry'] = t_res, prev
    if prev in ('#204', 'jog'):
        said_fn(g, prev, _WALK_ON_WIRE, (prev,), at=t_res)
    else:
        said_fn(g, *by[prev], at=t_res)


def reset_for_scenario(g, scen, seen_fn, dropped_fn, tilt_fn, doing_fn, wep_s):
    import numpy as np
    from machine import figure, latent, modal
    world, body = g['world'], g['body']
    d = world.data
    if (math.hypot(float(d.qpos[0]), float(d.qpos[2])) > 0.08 or abs(float(d.qpos[1]) - g['rest_y']) > 0.02
            or tilt_fn(g) > 2.0 or doing_fn(g) != 'rest' or len(g['tab']) > 0):
        world.reset(figure.stand(), where=(0.0, g['rest_y'], 0.0))
        s = g['s'] = seen_fn(body, world, g['limbs'])
        hw = 0.5 * float(g['fr'][modal.WEIGHT])
        g['loads'], g['heading'], g['target_heading'] = [hw, hw], 0.0, None
        modal.sensed(s, g['loads'], g['y'])
        g['fr'][modal.HEADING], g['fr'][modal.FLOOR] = 0.0, g.get('floor0', 0.0)
        g['fr'][modal.ROT] = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]).ravel()
        g['st'] = latent.state(g['rest'])
        g['out'] = latent.step(g['st'].copy(), g['tab'][:0], g['t'], 0.001 * g['every'], np.zeros(latent.OUT))
        g['a'][:], g['a'][modal.STANCE], g['ref_q'][:] = 0.0, 1.0, d.qpos[body.qact]
        g['ref_v'][:], g['hold'][:], g['left'], g['stumble_until'] = 0.0, 0.0, wep_s, 0.0
        g['eso'][:] = [d.qpos[body.qact], np.zeros(len(body.qact)), np.zeros(len(body.qact))]
        g['_twist_active'], g['twist'], g['twist_actual'], g['active_entry'], g['n'] = False, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), None, 0
        g['_chair_elevated'], g['_chair_cleared'], g['_look_yaw'] = False, False, 0.0
        if g.get('waist_idx') is not None:
            g['posture'][g['waist_idx']] = 0.0
        if g.get('head_idx') is not None:
            g['posture'][g['head_idx']] = 0.0
        dropped_fn(g)
    g['scenario'] = scen
    world.scenario(scen)




