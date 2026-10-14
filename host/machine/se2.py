"""SE(2) body-relative 3rd-person locomotion controller: input polling, twist ramp, kinematics."""
import math

#: Initial walking speed (m/s) immediately upon pressing W (matches #204 walk speed).
WALK_SPEED = 0.69
#: Maximum forward running speed (m/s) when holding W over time or holding Shift+W.
RUN_SPEED = 1.40
#: Longitudinal ramp acceleration (m/s^2) while W is held continuously.
RAMP_ACCEL = 0.45
#: Longitudinal deceleration (m/s^2) when W or S is released.
DECAY_RATE = 1.80
#: Reverse walking speed magnitude (m/s) when holding S (clamped lower than forward walk).
BACK_SPEED = 0.35
#: Pure lateral strafing speed (m/s) when A or D is pressed in isolation (clamped to walk speed).
STRAFE_SPEED = 0.50
#: Lateral velocity decay rate (m/s^2) when A/D is released.
STRAFE_DECAY = 3.20
#: Angular yaw velocity magnitude (rad/s) during composite curvilinear arcs (WA/WD/SA/SD).
YAW_RATE = 0.55
#: Default hold persistence window (s) for discrete terminal key-repeat streams.
HOLD_WINDOW_S = 0.25

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
        low = item.strip().lower()
        if item in ('W', 'A', 'S', 'D', 'WA', 'WD', 'SA', 'SD') or low == 'shift':
            tokens.add('shift')
        if low in _CHORD_EXPAND:
            tokens.update(_CHORD_EXPAND[low])
        elif low in ('w', 's', 'a', 'd'):
            tokens.add(low)
    return tokens


def poll_input(raw, st=None, now=None, hold_window=HOLD_WINDOW_S):
    """Phase 1 (Input Polling): normalize active WASD state, independent of camera orientation.

    Supports either direct boolean state (`raw` as a set/dict of currently held keys when
    `now is None`) or discrete terminal key-repeat timestamps when `now` is provided.
    Opposing axes (`W+S` or `A+D`) cancel cleanly.
    """
    tokens = _extract_tokens(raw)
    if st is not None and now is not None:
        last_seen = st['last_seen']
        if 'w' in tokens and 's' not in tokens:
            last_seen.pop('s', None)
        elif 's' in tokens and 'w' not in tokens:
            last_seen.pop('w', None)
        if 'a' in tokens and 'd' not in tokens:
            last_seen.pop('d', None)
        elif 'd' in tokens and 'a' not in tokens:
            last_seen.pop('a', None)
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
    shift = 'shift' in active
    keys = {'w': w, 's': s, 'a': a, 'd': d, 'shift': shift}
    if st is not None:
        st['keys'] = keys
    return keys


def step_twist(st, keys, dt):
    """Phase 2 (Twist Generation): update body-frame twist V_b = [v_x, v_y, omega_z]^T over `dt`.

    1. Longitudinal (W / S):
       - Holding W starts at WALK_SPEED and ramps continuously with hold duration up to RUN_SPEED.
       - Releasing W smoothly decays v_x -> 0.
       - Holding S drives v_x negatively, clamped to -BACK_SPEED.
    2. Pure Strafe (isolated A / D when neither W nor S is active):
       - Generates pure lateral velocity v_y (+STRAFE_SPEED for A, -STRAFE_SPEED for D).
       - Strictly zero yaw rotation (omega_z = 0).
    3. Curvilinear / Arc Trajectory (composite WA / WD / SA / SD):
       - Suppresses lateral strafing (v_y -> 0).
       - Maps A/D to angular yaw velocity (+YAW_RATE for A, -YAW_RATE for D).
    """
    dt = max(0.0, float(dt))
    w = bool(keys.get('w'))
    s = bool(keys.get('s'))
    a = bool(keys.get('a'))
    d = bool(keys.get('d'))
    shift = bool(keys.get('shift'))
    st['keys'] = {'w': w, 's': s, 'a': a, 'd': d, 'shift': shift}

    # 1. Longitudinal velocity v_x
    if w and not s:
        st['w_held'] += dt
        st['s_held'] = 0.0
        base = RUN_SPEED if shift else WALK_SPEED
        st['v_x'] = min(RUN_SPEED, base + RAMP_ACCEL * st['w_held'])
    elif s and not w:
        st['s_held'] += dt
        st['w_held'] = 0.0
        st['v_x'] = -BACK_SPEED
    else:
        st['w_held'] = 0.0
        st['s_held'] = 0.0
        st['v_x'] = _approach(st['v_x'], 0.0, DECAY_RATE, dt)

    # 2 & 3. Lateral strafe v_y vs Curvilinear yaw rate omega_z
    longitudinal_active = w or s
    if not longitudinal_active and (a or d):
        # Isolated A / D -> Pure lateral strafe, strictly zero yaw rotation
        st['v_y'] = STRAFE_SPEED if a else -STRAFE_SPEED
        st['omega_z'] = 0.0
    elif longitudinal_active and (a or d):
        # Composite WA / WD / SA / SD -> Suppress lateral strafe, map A/D to yaw rate omega_z
        st['v_y'] = 0.0
        st['omega_z'] = YAW_RATE if a else -YAW_RATE
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
    """Transform body-frame [v_x, v_y] into MuJoCo horizontal (X, Z) frame at `heading`.

    Matches `modal.frame`'s `ROT` matrix where `ALONG = (sin(h), 0, cos(h))` (+Z at h=0)
    and `LEFT = (cos(h), 0, -sin(h))` (+X at h=0).
    """
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
    keys = st.get('keys') or {}
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
        return 'jog' if shift else '#204'
    if s:
        return '#205'
    if a:
        return 'strafe_left'
    if d:
        return 'strafe_right'
    if st['v_x'] > 0.05:
        return '#204'
    if st['v_x'] < -0.05:
        return '#205'
    if st['v_y'] > 0.05:
        return 'strafe_left'
    if st['v_y'] < -0.05:
        return 'strafe_right'
    return None


def step_controller(st, raw_input, dt, now=None, hold_window=HOLD_WINDOW_S):
    """Run all three decoupled SE(2) phases (poll_input -> step_twist -> step_kinematics)."""
    keys = poll_input(raw_input, st=st, now=now, hold_window=hold_window)
    step_twist(st, keys, dt)
    step_kinematics(st, dt)
    return st
