"""SE(2) body-relative 3rd-person locomotion controller: input polling, twist ramp, kinematics."""
import math

#: Normal walking speed (m/s) when holding W without Shift (matches #204 walk speed).
WALK_SPEED = 0.69
#: Maximum forward sprinting speed (m/s) when holding Shift+W.
RUN_SPEED = 1.40
#: Longitudinal ramp acceleration (m/s^2) toward ordered setpoint.
RAMP_ACCEL = 0.95
#: Longitudinal deceleration (m/s^2) when W or S is released.
DECAY_RATE = 1.80
#: Reverse walking speed magnitude (m/s) when holding S without Shift.
BACK_SPEED = 0.35
#: Reverse sprinting speed magnitude (m/s) when holding Shift+S.
BACK_SPRINT_SPEED = 0.60
#: Pure lateral strafing speed (m/s) when A or D is pressed without Shift.
STRAFE_SPEED = 0.50
#: Pure lateral sprint-strafing speed (m/s) when Shift+A or Shift+D is pressed.
STRAFE_SPRINT_SPEED = 0.80
#: Lateral velocity decay rate (m/s^2) when A/D is released.
STRAFE_DECAY = 3.20
#: Angular yaw velocity magnitude (rad/s) during composite curvilinear arcs (WA/WD/SA/SD).
YAW_RATE = 0.55
#: Angular yaw velocity magnitude (rad/s) during sprint arcs (Shift+WA/WD/SA/SD).
YAW_SPRINT_RATE = 0.75
#: Default hold persistence window (s) for discrete terminal key-repeat streams (>560ms Windows typematic delay).
HOLD_WINDOW_S = 0.65

WASD_GAITS = ('#204', '#205', 'jog', 'strafe_left', 'strafe_right',
              'turn_left', 'turn_right', 'turn_back_left', 'turn_back_right', 'init_walk')

_CHORD_EXPAND = {
    'wa': ('w', 'a'),
    'wd': ('w', 'd'),
    'sa': ('s', 'a'),
    'sd': ('s', 'd'),
}


def state(yaw=0.0, x=0.0, y=0.0):
    """Fresh SE(2) controller state dictionary in the robot's local body frame."""
    return {
        'v_x': 0.0,
        'v_y': 0.0,
        'omega_z': 0.0,
        'setpoint': (0.0, 0.0, 0.0),
        'yaw': float(yaw),
        'x': float(x),
        'y': float(y),
        'v_wx': 0.0,
        'v_wy': 0.0,
        'w_held': 0.0,
        's_held': 0.0,
        'keys': {'w': False, 's': False, 'a': False, 'd': False, 'shift': False},
        'last_seen': {},
        'gait': None,
    }


def wrap_angle(angle):
    """Wrap angle in radians into [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _approach(current, target, rate, dt):
    """Slew `current` toward `target` by at most `rate * dt`."""
    delta = target - current
    limit = rate * dt
    if abs(delta) <= limit:
        return target
    return current + math.copysign(limit, delta)


def _extract_tokens(raw):
    """Extract normalized key tokens from a string, iterable, or mapping."""
    tokens = set()
    if raw is None:
        return tokens
    if isinstance(raw, str):
        items = [raw]
    elif isinstance(raw, dict):
        items = [k for k, v in raw.items() if v]
    else:
        items = list(raw)
    for item in items:
        if not isinstance(item, str):
            continue
        stripped = item.strip()
        low = stripped.lower()
        if stripped in ('W', 'A', 'S', 'D', 'WA', 'WD', 'SA', 'SD') or low in ('shift', 'sprint'):
            tokens.add('shift')
        if low in _CHORD_EXPAND:
            tokens.update(_CHORD_EXPAND[low])
        elif low in ('w', 's', 'a', 'd'):
            tokens.add(low)
    return tokens


def poll_input(raw, st=None, now=None, hold_window=HOLD_WINDOW_S, held=None):
    """Phase 1 (Input Polling): normalize active WASD state, independent of camera orientation.

    Supports direct hardware key state (`held` as a set of currently depressed physical keys),
    direct boolean state (`raw` when `now is None`), or discrete terminal key-repeat timestamps
    when `now` is provided. Opposing axes (`W+S` or `A+D`) cancel cleanly.
    """
    tokens = _extract_tokens(raw)
    if held is not None:
        active = _extract_tokens(held) | tokens
        if st is not None:
            st['last_seen'] = {tok: float(now) for tok in active} if (active and now is not None) else {}
    elif st is not None and now is not None:
        last_seen = st['last_seen']
        if 'w' in tokens and 's' not in tokens:
            last_seen.pop('s', None)
        elif 's' in tokens and 'w' not in tokens:
            last_seen.pop('w', None)
        if 'a' in tokens and 'd' not in tokens:
            last_seen.pop('d', None)
        elif 'd' in tokens and 'a' not in tokens:
            last_seen.pop('a', None)
        if tokens and 'shift' not in tokens:
            last_seen.pop('shift', None)
        for tok in tokens:
            last_seen[tok] = float(now)
        active = {
            k for k, ts in list(last_seen.items())
            if (now - ts) <= hold_window
        }
    else:
        active = tokens
        if st is not None and not tokens:
            st['last_seen'].clear()

    w = 'w' in active and 's' not in active
    s = 's' in active and 'w' not in active
    a = 'a' in active and 'd' not in active
    d = 'd' in active and 'a' not in active
    shift = 'shift' in active and (w or s or a or d)
    keys = {'w': w, 's': s, 'a': a, 'd': d, 'shift': shift}
    if st is not None:
        st['keys'] = keys
    return keys


def setpoint_for_keys(keys):
    """Target velocity setpoint (vx_set, vy_set, wz_set) for `keys` (WASD walk vs Shift+WASD sprint)."""
    w = bool(keys.get('w'))
    s = bool(keys.get('s'))
    a = bool(keys.get('a'))
    d = bool(keys.get('d'))
    shift = bool(keys.get('shift'))
    if w and not s:
        vx_set = RUN_SPEED if shift else WALK_SPEED
    elif s and not w:
        vx_set = -(BACK_SPRINT_SPEED if shift else BACK_SPEED)
    else:
        vx_set = 0.0
    longitudinal = (w and not s) or (s and not w)
    if not longitudinal and (a or d):
        spd_y = STRAFE_SPRINT_SPEED if shift else STRAFE_SPEED
        vy_set = spd_y if a else -spd_y
        wz_set = 0.0
    elif longitudinal and (a or d):
        vy_set = 0.0
        rate_z = YAW_SPRINT_RATE if shift else YAW_RATE
        wz_set = rate_z if a else -rate_z
    else:
        vy_set = 0.0
        wz_set = 0.0
    return (vx_set, vy_set, wz_set)


def step_twist(st, keys, dt):
    """Phase 2 (Twist Generation): update ordered body-frame twist V_b = [v_x, v_y, omega_z]^T over `dt`.

    - Without Shift: WASD targets normal walk/strafe/back/turn setpoints.
    - With Shift (Shift+WASD): targets sprint setpoints (RUN_SPEED, STRAFE_SPRINT_SPEED, etc.).
    - Releasing WASD smoothly decays ordered velocities toward 0.
    """
    dt = max(0.0, float(dt))
    w = bool(keys.get('w'))
    s = bool(keys.get('s'))
    a = bool(keys.get('a'))
    d = bool(keys.get('d'))
    shift = bool(keys.get('shift'))
    st['keys'] = {'w': w, 's': s, 'a': a, 'd': d, 'shift': shift}
    vx_set, vy_set, wz_set = setpoint_for_keys(st['keys'])
    st['setpoint'] = (vx_set, vy_set, wz_set)

    # 1. Longitudinal velocity v_x
    if w and not s:
        st['w_held'] += dt
        st['s_held'] = 0.0
        if shift:
            base = max(WALK_SPEED, st['v_x'])
            st['v_x'] = min(RUN_SPEED, _approach(base, RUN_SPEED, RAMP_ACCEL, dt))
        else:
            st['v_x'] = WALK_SPEED if st['v_x'] <= WALK_SPEED else _approach(st['v_x'], WALK_SPEED, DECAY_RATE, dt)
    elif s and not w:
        st['s_held'] += dt
        st['w_held'] = 0.0
        if shift:
            base = min(-BACK_SPEED, st['v_x'])
            st['v_x'] = max(-BACK_SPRINT_SPEED, _approach(base, -BACK_SPRINT_SPEED, RAMP_ACCEL, dt))
        else:
            st['v_x'] = -BACK_SPEED
    else:
        st['w_held'] = 0.0
        st['s_held'] = 0.0
        st['v_x'] = _approach(st['v_x'], 0.0, DECAY_RATE, dt)

    # 2 & 3. Lateral strafe v_y vs Curvilinear yaw rate omega_z
    longitudinal_active = w or s
    if not longitudinal_active and (a or d):
        if shift:
            base_y = (max(STRAFE_SPEED, st['v_y']) if a else min(-STRAFE_SPEED, st['v_y']))
            st['v_y'] = _approach(base_y, vy_set, STRAFE_DECAY, dt)
        else:
            st['v_y'] = vy_set
        st['omega_z'] = 0.0
    elif longitudinal_active and (a or d):
        st['v_y'] = 0.0
        st['omega_z'] = wz_set
    else:
        st['v_y'] = _approach(st['v_y'], 0.0, STRAFE_DECAY, dt)
        st['omega_z'] = 0.0

    st['gait'] = gait_for_twist(st)
    return (st['v_x'], st['v_y'], st['omega_z'])


def body_to_world(v_x, v_y, yaw):
    """Transform body-frame linear velocity [v_x, v_y] into 2D world frame using robot `yaw`."""
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (v_x * c - v_y * s, v_x * s + v_y * c)


def body_to_mujoco(v_x, v_y, heading):
    """Transform body-frame [v_x, v_y] into MuJoCo horizontal (X, Z) frame at `heading`."""
    c = math.cos(heading)
    s = math.sin(heading)
    vx_world = v_x * s + v_y * c
    vz_world = v_x * c - v_y * s
    return (vx_world, vz_world)


def step_kinematics(st, dt):
    """Phase 3 (Kinematics Integration): integrate omega_z into yaw and transform [v_x, v_y] to world."""
    dt = max(0.0, float(dt))
    st['yaw'] = wrap_angle(st['yaw'] + st['omega_z'] * dt)
    v_wx, v_wy = body_to_world(st['v_x'], st['v_y'], st['yaw'])
    st['v_wx'] = v_wx
    st['v_wy'] = v_wy
    st['x'] += v_wx * dt
    st['y'] += v_wy * dt
    return (st['x'], st['y'], st['yaw'])


def gait_for_twist(st):
    """Select the orthokinematic gait entry name corresponding to current SE(2) twist state."""
    keys = st.get('keys')
    if keys is not None:
        w, s, a, d = keys.get('w'), keys.get('s'), keys.get('a'), keys.get('d')
        shift = keys.get('shift', False)
        if w and a:
            return 'turn_left'
        if w and d:
            return 'turn_right'
        if s and a:
            return 'turn_back_left'
        if s and d:
            return 'turn_back_right'
        if w:
            return 'jog' if (shift or st.get('v_x', 0.0) > WALK_SPEED + 0.05) else '#204'
        if s:
            return '#205'
        if a:
            return 'strafe_left'
        if d:
            return 'strafe_right'
        if 'setpoint' in st:
            return None
    vx = st.get('v_x', 0.0)
    vy = st.get('v_y', 0.0)
    wz = st.get('omega_z', 0.0)
    if abs(vy) > 0.05:
        return 'strafe_left' if vy > 0 else 'strafe_right'
    if vx > 0.05 and abs(wz) > 0.05:
        return 'turn_left' if wz > 0 else 'turn_right'
    if vx < -0.05 and abs(wz) > 0.05:
        return 'turn_back_left' if wz > 0 else 'turn_back_right'
    if vx > WALK_SPEED + 0.05:
        return 'jog'
    if vx > 0.05:
        return '#204'
    if vx < -0.05:
        return '#205'
    return None


def step_controller(st, raw_input, dt, now=None, hold_window=HOLD_WINDOW_S, held=None):
    """Run all three decoupled SE(2) phases (poll_input -> step_twist -> step_kinematics)."""
    keys = poll_input(raw_input, st=st, now=now, hold_window=hold_window, held=held)
    step_twist(st, keys, dt)
    step_kinematics(st, dt)
    return st


def step_physical_twist(g, dt):
    """Steer modal reference heading and CPG sprint parameters physically without modifying qpos/qvel."""
    if not g.get('_twist_active'):
        return
    from machine import latent, modal
    vx, vy, wz = g.get('twist', (0.0, 0.0, 0.0))
    if abs(wz) > 1e-6:
        q = g['world'].data.qpos[3:7]
        pyaw = 2.0 * math.atan2(float(q[2]), float(q[0]))
        d_yaw = wrap_angle(pyaw - g['heading'])
        if d_yaw * wz > 0.0:
            gain = 5.0 if abs(wz) > YAW_RATE + 0.05 else 3.5
            g['heading'] = wrap_angle(g['heading'] + gain * d_yaw * dt)
    sprinting = (abs(vx) > WALK_SPEED + 0.05 or abs(vy) > STRAFE_SPEED + 0.05
                 or vx < -(BACK_SPEED + 0.05) or abs(wz) > YAW_RATE + 0.05)
    if sprinting and g['st'][latent.NOW][latent.GAIN] > 0.5:
        g['st'][latent.NOW][latent.HZ] = max(float(g['st'][latent.NOW][latent.HZ]), 0.96)
        if abs(vy) > STRAFE_SPEED + 0.05:
            g['st'][latent.NOW][latent.Z][modal.LATERAL] = math.copysign(0.11, vy)
        if abs(wz) > YAW_RATE + 0.05:
            g['st'][latent.NOW][latent.Z][modal.TURN] = math.copysign(0.32, wz)


def heard_twist(g, gym, tw, has_say, said_fn, dropped_fn, heard_say_fn, walk_on):
    """Update `g['twist']` and manage physical CPG gait lifecycle when a twist command arrives."""
    prev_vx, prev_vy, _ = g.get('twist', (0.0, 0.0, 0.0))
    g['twist'] = (0.0, 0.0, 0.0) if tw is None else (float(tw[0]), float(tw[1]), float(tw[2]))
    vx, vy, wz = g['twist']
    by = {e[0]: e for e in gym}
    mag = math.hypot(vx, vy)
    prev_mag = math.hypot(prev_vx, prev_vy)
    decaying_to_zero = (
        not has_say and mag < prev_mag - 1e-4
        and abs(vx) < WALK_SPEED - 0.01 and abs(vy) < STRAFE_SPEED - 0.01 and abs(wz) < 0.01
    )
    if (abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01) and not decaying_to_zero:
        label = gait_for_twist({'v_x': vx, 'v_y': vy, 'omega_z': wz}) or '#204'
        g['_twist_active'] = True
        has_future = bool(len(g['tab']) and (g['tab'][:, -2] >= g['t']).any())
        idle_rest = float(g['st'][55]) < 1e-6 and not has_future
        if not has_say and label in by and (g.get('active_entry') != label or idle_rest):
            heard_say_fn(g, gym, by[label])
            g['active_entry'] = label
    elif g.get('_twist_active') or g.get('active_entry') in WASD_GAITS:
        g['_twist_active'] = False
        if not has_say and '#201' in by:
            heard_say_fn(g, gym, by['#201'])



