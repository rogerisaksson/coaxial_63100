"""Her twist on the loop: the gait it walks her by, her heading turned by it, her rhythm by it."""
import math

from machine import ortho

#: Her twists' speeds: a walk, a sprint, a walk back, a sidestep, m/s; a turn, rad/s.
WALK_SPEED, RUN_SPEED, BACK_SPEED, STRAFE_SPEED, YAW_RATE = 0.69, 1.40, 0.35, 0.50, 0.70
#: Her walk's stopping distance, m: told to stand she goes on 0.70 m and is still 2.7 s later,
#: 0.81 m/s felt whatever speed was asked - she has the one walk.
STOP_M = 0.70

WASD_GAITS = ('walk', 'walk_back', 'jog', 'strafe_left', 'strafe_right',
              'turn_left', 'turn_right', 'turn_back_left', 'turn_back_right', 'spin_left', 'spin_right', 'init_walk')

NOMINAL_TWIST = {
    'walk': (WALK_SPEED, 0.0, 0.0), 'init_walk': (WALK_SPEED, 0.0, 0.0), 'trip': (WALK_SPEED, 0.0, 0.0),
    'stairs_walk': (0.50, 0.0, 0.0), 'stairs_down': (0.50, 0.0, 0.0),
    'ice_walk': (0.35, 0.0, 0.0), 'slope_walk': (0.55, 0.0, 0.0),
    'ballistic_walk': (0.75, 0.0, 0.0), 'jog': (RUN_SPEED, 0.0, 0.0),
    'walk_back': (-BACK_SPEED, 0.0, 0.0),
    'strafe_left': (0.0, STRAFE_SPEED, 0.0), 'strafe_right': (0.0, -STRAFE_SPEED, 0.0),
    'turn_left': (WALK_SPEED, 0.0, YAW_RATE), 'turn_right': (WALK_SPEED, 0.0, -YAW_RATE),
    'spin_left': (0.0, 0.0, YAW_RATE), 'spin_right': (0.0, 0.0, -YAW_RATE),
    'circle_walk': (0.55, 0.0, 0.45),
    'turn_back_left': (-BACK_SPEED, 0.0, YAW_RATE), 'turn_back_right': (-BACK_SPEED, 0.0, -YAW_RATE),
}
LEAK_S, STANDING_KP, WEP_HOLD_S, WEP_S, WEP_FILL, KNEE_SOFT, KNEE_NM = 0.2, 0.1, 1.0, 8.0, 0.8, 0.1, 40.0
STUNT_MODES = {'sit_chair', 'rise_chair', 'lie_bed', 'rise_bed', 'lie_floor', 'rise_floor', 'hop_down_chair',
               'hop_up_chair', 'stairs_down', 'parry_shove', 'controlled_fall'}
STUNT_PHASES = {'approach', 'lower', 'seated', 'lean_forward', 'thrust', 'sit_edge', 'recline', 'bed_rest', 'sit_up',
                'swing_out', 'kneel', 'floor_rest', 'push_arms', 'half_kneel', 'cushion', 'land', 'step_off', 'drop',
                'absorb', 'crouch_load', 'propulsion', 'flight_tuck', 'seat_land', 'descent'}
_FWD_GAITS = ('walk', 'trip', 'jog', 'ballistic_walk', 'stairs_walk', 'stairs_down', 'ice_walk', 'slope_walk',
              'strafe_left', 'strafe_right', 'turn_left', 'turn_right', 'spin_left', 'spin_right', 'circle_walk')
_BACK_GAITS = ('walk_back', 'turn_back_left', 'turn_back_right')
RESET_MODES = ('ballistic_walk', 'jog', 'stand')
INIT_GAITS = ('walk', 'trip', 'stairs_walk', 'stairs_down', 'ice_walk', 'slope_walk',
              'turn_left', 'turn_right', 'circle_walk')
RISE_OF = {'seated': 'rise_chair', 'bed_rest': 'rise_bed', 'floor_rest': 'rise_floor'}
HOLD_PHASES = ('hold', 'load', 'throw', 'land', 'step_off', 'drop', 'absorb',
               'crouch_load', 'propulsion', 'flight_tuck', 'seat_land')


def init_lead(g):
    """Continuous pre-sway lead horizon (s) from angular rate `asked[2]` and twist activation."""
    wz = abs(float(g.get('asked', (0.0, 0.0, 0.0))[2]))
    w_turn = min(1.0, max(0.0, (wz - 0.04) / 0.01))
    w_tw = float(bool(g.get('_twist_active')))
    return (1.0 - w_turn) * (0.65 - 0.37 * w_tw) + 0.12 * w_turn


def brake_lead(spd, back_to=0.0):
    """Continuous braking transition horizon (s) from speed `spd` (m/s) and direction `back_to` in [0, 1]."""
    w_fast = min(1.0, max(0.0, (float(spd) - 0.55) / 1e-6))
    b = float(back_to)
    return b * (0.38 + 0.14 * w_fast) + (1.0 - b) * (0.45 + 0.25 * w_fast)


def stand_lead(spd):
    """Continuous braking-to-stand settling and follow-up horizons `(ts, tg)` (s) from speed `spd` (m/s)."""
    w_fast = min(1.0, max(0.0, (float(spd) - 0.45) / 1e-6))
    ts = 0.55 + 0.65 * w_fast
    return ts, 2.0 * ts


def rise_lead(wire):
    """Continuous rise-to-walk horizon (s) derived directly from a rise stunt's compiled wire table."""
    import numpy as np
    from machine import latent
    rows = np.frombuffer(wire, latent.WIRE)
    a1, a3 = 0.001 * float(rows[-1]['at']), 0.001 * float(rows[-3]['at'])
    return round(0.75 - 0.3375 * a1 + 3.1375 * a3, 4)


def said(g, label, wire, names=(), at=None, reset=False):
    import numpy as np
    from machine import latent
    new = latent.table(wire, g['t'] + 0.001 if at is None else at, g['rest'])
    if reset and len(new):
        new[0, latent.VALS] = np.where(new[0, latent.SAID] > 0.5, new[0, latent.VALS], g['rest'])
        new[0, latent.SAID] = 1.0
    t_eat = g['t'] - 0.001 * ((g['n'] - 1) % g['every'] + 1) + 1e-6 if g.get('n', 0) > 0 else -1.0
    g['tab'] = np.concatenate([g['tab'][g['tab'][:, latent.AT] > t_eat], new])
    g['labels'] = g['labels'][-16:] + [(float(w), names[i] if i < len(names) else label) for i, w in enumerate(new[:, latent.AT])]


def _dropped(g):
    g['tab'], g['labels'], g['_jump_until'], g['_held_s'] = (
        g['tab'][:0], [(w, n) for w, n in g['labels'] if w <= g['t']], 0.0, 0.0)
    g['_jump_t0'], g['_jump_throw'], g['_leap'] = 0.0, 0.0, False


def doing(g):
    """What she is doing: the name of her table's row that last came by her law's last pass, 'rest'
    with none. Read at the world's own passes between, a row her law holds came and went every
    pass, 596 flips in a stop."""
    come = [(w, n) for w, n in g['labels'] if w <= g.get('t_law', g['t']) + 1e-9]
    return max(come)[1] if come else 'rest'


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


_INF = math.inf
#: Twist-space polytope table: (vx_min, vx_max, vy_min, vy_max, wz_min, wz_max, gait_name).
GAIT_BOXES = (
    (-_INF, _INF, 0.05, _INF, -_INF, _INF, 'strafe_left'),
    (-_INF, _INF, -_INF, -0.05, -_INF, _INF, 'strafe_right'),
    (-0.05, _INF, -0.05, 0.05, 0.05, _INF, 'turn_left'),
    (-0.05, _INF, -0.05, 0.05, -_INF, -0.05, 'turn_right'),
    (-_INF, -0.05, -0.05, 0.05, 0.05, _INF, 'turn_back_left'),
    (-_INF, -0.05, -0.05, 0.05, -_INF, -0.05, 'turn_back_right'),
    (0.96, _INF, -0.05, 0.05, -0.05, 0.05, 'jog'),
    (0.05, 0.96, -0.05, 0.05, -0.05, 0.05, 'walk'),
    (-_INF, -0.05, -0.05, 0.05, -0.05, 0.05, 'walk_back'),
)


def gait_for_twist(vx, vy, wz):
    """The gym's entry a twist walks her by, queried from `GAIT_BOXES`; None at rest."""
    moving = max(abs(vx), abs(vy), abs(wz)) > 0.05
    return next((name for x0, x1, y0, y1, z0, z1, name in GAIT_BOXES
                 if moving and x0 <= vx <= x1 and y0 <= vy <= y1 and z0 <= wz <= z1), None)


#: The twist step's memory, one array (`twist_laid`): which way her sway led last (nan till it
#: has), her look's yaw, her turn's offset. Its words with `g` each pass (`words_laid`): her
#: heading, the heading told and whether one is, her twist told and as made, whether a twist is
#: active and until when a stop holds, her time; out her heading, her twist as made, her backing
#: and its yaw, her clearance.
LAST_WAY, LOOK_YAW, TURN_DC, TWISTED = 0, 1, 2, 3
W_HEADING, W_TARGET, W_HAS, W_TWIST, W_ACTUAL, W_ACTIVE, W_STOP, W_T = 0, 1, 2, slice(3, 6), slice(6, 9), 9, 10, 11
W_BACK, W_BACKING, W_CLEAR, WORDS = 12, 13, 14, 15
#: The floor under her follows her bearing soles at FLOOR_HZ.
FLOOR_HZ = 15.0


def twist_laid():
    """The step's memory at rest, (TWISTED,): no way led yet, no look, no turn."""
    import numpy as np
    return np.array([math.nan, 0.0, 0.0])


def words_laid():
    """The step's words, (WORDS,), zeros."""
    import numpy as np
    return np.zeros(WORDS)


def _modulate_cpg(st, out, held, prior, state, tw_on, vx, vy, wz, vxp):
    from machine import latent, modal
    gain = float(st[latent.NOW][latent.GAIN])
    amp_d = abs(float(st[latent.TO][latent.AMP][modal.DRIVE]))
    w_gait = min(1.0, max(0.0, (max(amp_d, abs(vy), abs(wz)) - 0.02) / 0.02))
    w_on = tw_on * min(1.0, max(0.0, (gain - 0.02) / 0.03)) * w_gait
    w_rest = tw_on * (1.0 - min(1.0, max(0.0, (gain - 0.02) / 0.03))) * float(0.05 < amp_d < 0.30)
    w_fast_fwd = (min(1.0, max(0.0, (vx - WALK_SPEED) / 0.01))
                  * min(1.0, max(0.0, (0.05 - abs(wz)) / 0.01))
                  * min(1.0, max(0.0, (0.01 - abs(vy)) / 0.01)))
    st[latent.TO][latent.AMP][modal.DRIVE] += w_rest * w_fast_fwd * max(
        0.0, 0.60 - float(st[latent.TO][latent.AMP][modal.DRIVE]))
    w_vy = min(1.0, max(0.0, (abs(vy) - 0.04) / 0.01))
    w_wz = min(1.0, max(0.0, (abs(wz) - 0.025) / 0.005))
    w_wz5 = min(1.0, max(0.0, (abs(wz) - 0.04) / 0.01))
    w_spin = w_on * w_wz5 * min(1.0, max(0.0, (0.05 - abs(vx)) / 0.01)) * (1.0 - w_vy)
    ts_raw = w_on * w_wz * (0.40 - 0.12 * w_vy) * max(-1.0, min(1.0, wz / YAW_RATE))
    ts = state[TURN_DC] = float(state[TURN_DC]) + 0.18 * (ts_raw - float(state[TURN_DC]))
    w_turn = w_on * max(w_wz, min(1.0, max(0.0, (abs(ts) - 0.008) / 0.004)))
    bt_d, w = float(st[latent.THETA] + st[latent.NOW][latent.PHASE][modal.DRIVE]), float(out[latent.TURNS])
    w_vxp = min(1.0, max(0.0, (abs(vxp) - 0.75) / 0.05))
    drv = (1.0 - w_vxp) * 0.035 + w_vxp * max(0.035, 0.35 * (abs(vxp) - 0.15))
    out[latent.SWAY][modal.DRIVE] += w_spin * (drv - float(out[latent.SWAY][modal.DRIVE]))
    out[latent.REF][modal.DRIVE] += w_spin * (drv * math.sin(bt_d) - float(out[latent.REF][modal.DRIVE]))
    out[latent.RATE][modal.DRIVE] += w_spin * (drv * w * math.cos(bt_d) - float(out[latent.RATE][modal.DRIVE]))
    out[latent.ACC][modal.DRIVE] += w_spin * (-drv * w * w * math.sin(bt_d) - float(out[latent.ACC][modal.DRIVE]))
    out[latent.SWAY][modal.LIFT] += w_spin * max(0.0, 0.95 - float(out[latent.SWAY][modal.LIFT]))
    amp_t = w_vy * abs(ts) + (1.0 - w_vy) * 0.14
    bt = float(st[latent.THETA] + st[latent.NOW][latent.PHASE][modal.TURN])
    out[latent.SWAY][modal.TURN] += w_turn * (amp_t - float(out[latent.SWAY][modal.TURN]))
    out[latent.REF][modal.TURN] += w_turn * (ts + amp_t * math.sin(bt) - float(out[latent.REF][modal.TURN]))
    out[latent.RATE][modal.TURN] += w_turn * (amp_t * w * math.cos(bt) - float(out[latent.RATE][modal.TURN]))
    out[latent.ACC][modal.TURN] += w_turn * (-amp_t * w * w * math.sin(bt) - float(out[latent.ACC][modal.TURN]))
    w_slip = (min(1.0, max(0.0, (0.25 - min(held)) / 0.02))
              * min(1.0, max(0.0, (min(prior) - 0.25) / 0.02))
              * min(1.0, max(0.0, (abs(vx) - 0.04) / 0.01)))
    w_vx_hi = min(1.0, max(0.0, (abs(vx) - WALK_SPEED) / 0.01))
    w_vy_hi = min(1.0, max(0.0, (abs(vy) - STRAFE_SPEED) / 0.01))
    w_back_hi = min(1.0, max(0.0, (-vx - BACK_SPEED) / 0.01)) * min(1.0, max(0.0, (0.01 - abs(vy)) / 0.01))
    w_wz_hi = min(1.0, max(0.0, (abs(wz) - YAW_RATE) / 0.01))
    w_hz = w_on * max(w_slip, w_vx_hi, w_vy_hi, w_back_hi, w_wz5)
    hz_m = w_on * max(1.05 * w_slip, 0.98 * max(w_spin, w_wz_hi, w_vx_hi), 0.95 * w_hz)
    st[latent.TO][latent.HZ] = max(float(st[latent.TO][latent.HZ]), hz_m)
    st[latent.NOW][latent.HZ] = max(float(st[latent.NOW][latent.HZ]), hz_m)
    out[latent.TURNS] += w_hz * (math.tau * float(st[latent.NOW][latent.HZ]) - float(out[latent.TURNS]))
    w_boost_fwd = w_hz * w_fast_fwd
    out[latent.SWAY][modal.DRIVE] += w_boost_fwd * (min(0.68, float(out[latent.SWAY][modal.DRIVE]) * 1.08) - float(out[latent.SWAY][modal.DRIVE]))
    out[latent.SWAY][modal.ARMS] += w_boost_fwd * (min(1.00, float(out[latent.SWAY][modal.ARMS]) * 1.08) - float(out[latent.SWAY][modal.ARMS]))
    w_boost_back = w_hz * w_back_hi * (1.0 - w_boost_fwd)
    out[latent.SWAY][modal.DRIVE] += w_boost_back * (min(0.28, float(out[latent.SWAY][modal.DRIVE]) * 1.25) - float(out[latent.SWAY][modal.DRIVE]))
    out[latent.REF][modal.LATERAL] += (w_hz * w_vy_hi) * (math.copysign(0.105, vy) - float(out[latent.REF][modal.LATERAL]))


def floored(y, fr, dt):
    """The floor under her where her bearing soles' lowest corners stand, in her estimate's own
    height - held where she was laid it read her soles 22 mm over it as her estimate drifted up -,
    and its pitch under each sole: that sole's own while its heel's cell and its ball's both
    press, as far as they press alike. One centre of pressure read inside its edges took a
    hole's edge under a sole's middle for a floor 10 deg up."""
    import numpy as np
    from machine import leap, modal
    over = np.maximum(y[modal.LOAD] - leap.TOUCH_N, 0.0)
    bear = y[modal.LOAD] * (over / (over + 1.0))
    total = float(bear.sum())
    k = (total / (total + 1e-9)) * min(1.0, FLOOR_HZ * dt)
    under = float(bear @ y[modal.LOWEST]) / max(total, 1e-9)
    fr[modal.FLOOR] += k * (under - float(fr[modal.FLOOR]))
    at = (bear @ y[modal.SOLE].reshape(2, 3)[:, [0, 2]]) / max(total, 1e-9)
    fr[modal.FLOOR_AT] += k * (at - fr[modal.FLOOR_AT])
    cells = y[modal.CELLS].reshape(2, 2)
    c_min = cells.min(axis=1)
    c_over = np.maximum(c_min - leap.TOUCH_N, 0.0)
    flat = (c_over * c_over / (c_over * c_over + 1.0)) * c_min / np.maximum(cells.max(axis=1), 1e-9)
    fr[modal.GROUND] += min(1.0, FLOOR_HZ * dt) * flat * (y[modal.TILT] - fr[modal.GROUND])


def step_physical_twist(g, dt, out):
    """A pass of her twist on the loop: `twisted` in C on her arrays (`g['twist_step']`,
    `machine.motion.laid`), its words from and to `g`."""
    from machine import modal
    g.setdefault('floor0', float(g['fr'][modal.FLOOR]))
    w = g['words']
    w[W_HEADING], w[W_TARGET], w[W_HAS] = g['heading'], g.get('target_heading', 0.0), g.get('has_heading', 0.0)
    w[W_TWIST], w[W_ACTUAL] = g.get('twist', (0.0, 0.0, 0.0)), g.get('twist_actual', (0.0, 0.0, 0.0))
    w[W_ACTIVE], w[W_STOP], w[W_T] = float(g.get('_twist_active', False)), g.get('_twist_stop_until', 0.0), g['t']
    g['twist_step'](dt, g['est']['vel'], g['held'], g['grip_prior'])
    g['heading'], g['twist_actual'] = float(w[W_HEADING]), tuple(w[W_ACTUAL].tolist())
    g['_w_back'], g['_backing_yaw'], g['_twist_clear'] = float(w[W_BACK]), float(w[W_BACKING]), float(w[W_CLEAR])


def twisted(y_in, vel, y, fr, a, st, out, posture, held, prior, state, words, dt, waist, head):
    """A pass of her twist on her edge's array `y_in` and her estimate's velocity `vel`, modal's
    `y`, `fr` and `a`, her latent `st` and its `out`, her `posture` (her `waist` and `head` in it),
    her soles' grip `held` and her `prior` of it: the floor under her (`floored`), her heading
    turned by the twist told and by her pelvis, her rhythm flipped where her sway leads the other
    way, her gait modulated (`_modulate_cpg`), her look into a turn; `state` (`twist_laid`) and
    `words` (`words_laid`) stepped in place. In C bit for bit (`step_physical_twist`)."""
    import numpy as np
    from machine import edge, latent, leap, modal, stride
    has_thd = float(words[W_HAS])
    heading = wrap_angle(float(words[W_HEADING]) + has_thd * max(-1.6 * dt, min(1.6 * dt, wrap_angle(float(words[W_TARGET]) - float(words[W_HEADING])))))
    sh, ch = math.sin(heading), math.cos(heading)
    floored(y, fr, dt)
    vx, vy, wz = (float(v) for v in words[W_TWIST])
    vxp, vyp, wzp = float(vel[0]) * sh + float(vel[2]) * ch, float(vel[0]) * ch - float(vel[2]) * sh, float(y_in[edge.GYRO][1])
    ax, ay, az = (float(v) for v in words[W_ACTUAL])
    w_ty, w_tz = min(1.0, max(0.0, (abs(vy) - 0.005) / 0.005)), min(1.0, max(0.0, (abs(wz) - 0.005) / 0.005))
    kv, ky, kz = min(1.0, dt / 0.06), min(1.0, dt / (0.10 - 0.03 * w_ty)), min(1.0, dt / (0.10 - 0.03 * w_tz))
    words[W_ACTUAL] = (ax + kv * (vxp - ax), ay + ky * (w_ty * vyp - ay), az + kz * (w_tz * wzp - az))
    pelvis_yaw = edge.yaw(y_in)
    unborne = 1.0 - float(a[modal.BORNE])
    w_yaw = min(1.0, abs(wz) / 1e-4) * (1.0 - has_thd) * unborne
    l0, l1 = float(y[modal.LOAD][0]), float(y[modal.LOAD][1])
    tz_sgn = math.copysign(1.0, wz)
    out_st = 0.75 - 0.30 * math.tanh(3.0 * (l0 - l1) * tz_sgn / max(l0 + l1, leap.TOUCH_N))
    d_yaw = wrap_angle(pelvis_yaw - heading)
    w_step_yaw = float(d_yaw * tz_sgn >= -0.10)
    w_lead_yaw = float(d_yaw * wz > 0.0)
    heading = wrap_angle(heading + w_yaw * (w_step_yaw * out_st * wz + w_lead_yaw * 10.0 * d_yaw) * dt)
    v_horz = math.hypot(float(y[modal.VCOM][0]), float(y[modal.VCOM][2]))
    w_align = (w_yaw * float(vx >= 0.05) * float(abs(vy) < 0.05) * float(v_horz > 0.32) * float(vxp > 0.15))
    err_v = wrap_angle(math.atan2(float(y[modal.VCOM][0]), float(y[modal.VCOM][2])) - heading)
    err_v_ex = math.copysign(max(0.0, abs(err_v) - 0.32), err_v)
    heading = wrap_angle(heading + w_align * 12.0 * err_v_ex * dt)
    err_yaw = wrap_angle(pelvis_yaw - heading)
    err_yaw_ex = math.copysign(max(0.0, abs(err_yaw) - 0.10), err_yaw)
    heading = wrap_angle(heading + w_yaw * err_yaw_ex)
    sh_n, ch_n = math.sin(heading), math.cos(heading)
    fr[modal.HEADING] = heading
    fr[modal.ROT][:] = (sh_n, 0.0, ch_n, ch_n, 0.0, -sh_n, 0.0, 1.0, 0.0)
    l_psi, d_psi = float(out[latent.PSI][modal.LATERAL]), float(out[latent.PSI][modal.DRIVE])
    l_sgn = math.copysign(1.0, float(out[latent.SWAY][modal.LATERAL]))
    cw = math.copysign(1.0, math.sin(l_psi - d_psi) * l_sgn + 0.5)
    last_way = float(np.nan_to_num(state[LAST_WAY], nan=cw))
    w_flip = float(cw * last_way < 0.0) * max(float(st[latent.NOW][latent.GAIN] > 0.05),
                                              float(abs(float(out[latent.SWAY][modal.DRIVE])) > 0.02))
    old_psi = out[latent.PSI].copy()
    st[latent.THETA] = (float(st[latent.THETA]) + w_flip * math.pi) % math.tau
    out[latent.PSI] = st[latent.THETA] + st[latent.NOW][latent.PHASE]
    sway, turns = out[latent.SWAY], float(out[latent.TURNS])
    out[latent.REF] += (-2.0 * w_flip * sway) * [math.sin(float(p)) for p in old_psi]
    out[latent.RATE] += (-2.0 * w_flip * turns * sway) * [math.cos(float(p)) for p in old_psi]
    out[latent.ACC] *= 1.0 - 2.0 * w_flip
    state[LAST_WAY] = cw
    _modulate_cpg(st, out, held, prior, state, float(words[W_ACTIVE]), vx, vy, wz, vxp)
    drive_sway = abs(float(out[latent.SWAY][modal.DRIVE]))
    active = min(1.0, max(0.0, (drive_sway - stride.STIRS) / 0.04)) * unborne
    w_back = float(active > 0.1) * max(float(cw < 0.0), float(vx < -0.05))
    tl_back = math.copysign(1.0, wz + 0.05)
    tl_fwd = float(abs(wz) > 0.03) * max(-0.35, min(0.35, 0.35 * wz / YAW_RATE))
    tl = active * (w_back * tl_back + (1.0 - w_back) * tl_fwd)
    look = state[LOOK_YAW] = float(state[LOOK_YAW]) + min(1.0, (5.0 - 0.5 * w_back) * dt) * (tl - float(state[LOOK_YAW]))
    posture[waist] = 0.38 * look * unborne
    posture[head] = 0.72 * look * unborne
    words[W_HEADING] = heading
    words[W_BACK] = w_back * float(abs(look) > 1e-3)
    words[W_BACKING] = posture[waist]
    tw_on = max(float(words[W_ACTIVE]), float(words[W_STOP] > words[W_T]))
    words[W_CLEAR] = 0.115 * tw_on * unborne * float(out[latent.HELD] > 0.5)


def heard_twist(g, gym, tw, has_say, heard_say_fn):
    from machine import touch
    pvx, pvy, _ = g.get('twist', (0.0, 0.0, 0.0))
    g['asked'] = (0.0, 0.0, 0.0) if tw is None else (float(tw[0]), float(tw[1]), float(tw[2]))
    vx, vy, wz = g['twist'] = touch.against(g, g['asked'])
    by = {e[0]: e for e in gym}
    decay = (not has_say and math.hypot(vx, vy) < math.hypot(pvx, pvy) - 1e-4
             and -(BACK_SPEED - 0.01) < vx < WALK_SPEED - 0.01 and abs(vy) < STRAFE_SPEED - 0.01 and abs(wz) < 0.01)
    active = max(abs(vx), abs(vy), abs(wz)) > 0.01 and not decay
    was_on = bool(g.get('_twist_active') or g.get('active_entry') in WASD_GAITS or doing(g) in WASD_GAITS)
    target = (gait_for_twist(vx, vy, wz) or ('walk_back' if vx < 0.0 else 'walk')) if active else ('stand' if was_on else None)
    if target is not None:
        g['_twist_active'], g['_twist_stop_until'], g['_halt_at'] = active, g['t'] + 2.5, None
    if not has_say and target in by:
        heard_say_fn(g, gym, by[target])


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
    from machine import leap
    return leap.stride_hop(g, by, t0, spd, back_now, fwd_gaits, back_gaits, said_fn, _WALK_ON_WIRE)
