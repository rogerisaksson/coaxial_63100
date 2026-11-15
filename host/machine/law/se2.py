"""Its twist on the loop: the gait it walks it by, its heading turned by it, its rhythm by it."""
import functools
import math

from machine.body import gait
from machine.law import rhythm
from machine.texts import ortho

_G = gait.table('twist')
#: Its twists' speeds (its build's, `machine.body.gait`): a walk, a sprint, a walk back, a sidestep,
#: m/s; a turn, rad/s.
WALK_SPEED, RUN_SPEED, BACK_SPEED, STRAFE_SPEED, YAW_RATE = (_G[k] for k in ('walk_m_s', 'run_m_s', 'back_m_s', 'strafe_m_s', 'yaw_rad_s'))
#: Told faster than this it goes over from its walk to its march (the gym's jog): midway, m/s.
MARCH_FROM = 0.5 * (WALK_SPEED + RUN_SPEED)
#: Its walk's, walk back's and sidestep's stopping distances and half its walk's stop's spread, m.
STOP_M, BACK_STOP_M, SIDE_STOP_M, STOP_SPREAD_M = (_G[k] for k in ('stop_m', 'back_stop_m', 'side_stop_m', 'stop_spread_m'))
#: Its walk's rhythm, Hz, its brake's and stand's blends, s, and the speed under which a brake has halted it, m/s.
WALK_HZ, STAND_S, HALT_M_S = _G['walk_hz'], _G['stand_s'], 0.25
BRAKE_S = 1.0 / WALK_HZ

WASD_GAITS = ('walk', 'walk_back', 'jog', 'strafe_left', 'strafe_right',
              'turn_left', 'turn_right', 'turn_back_left', 'turn_back_right', 'spin_left', 'spin_right', 'init_walk')
SE2_GAITS = ('walk', 'jog', 'turn_left', 'turn_right', 'spin_left', 'spin_right')

NOMINAL_TWIST = {
    'walk': (WALK_SPEED, 0.0, 0.0), 'init_walk': (WALK_SPEED, 0.0, 0.0), 'trip': (WALK_SPEED, 0.0, 0.0),
    'stairs_walk': (_G['stairs_m_s'], 0.0, 0.0), 'stairs_down': (_G['stairs_m_s'], 0.0, 0.0),
    'ice_walk': (_G['ice_m_s'], 0.0, 0.0), 'slope_walk': (_G['slope_m_s'], 0.0, 0.0),
    'ballistic_walk': (_G['ballistic_m_s'], 0.0, 0.0), 'jog': (RUN_SPEED, 0.0, 0.0),
    'walk_back': (-BACK_SPEED, 0.0, 0.0),
    'strafe_left': (0.0, STRAFE_SPEED, 0.0), 'strafe_right': (0.0, -STRAFE_SPEED, 0.0),
    'turn_left': (WALK_SPEED, 0.0, YAW_RATE), 'turn_right': (WALK_SPEED, 0.0, -YAW_RATE),
    'spin_left': (0.0, 0.0, YAW_RATE), 'spin_right': (0.0, 0.0, -YAW_RATE),
    'circle_walk': (_G['circle_m_s'], 0.0, _G['circle_rad_s']),
    'turn_back_left': (-BACK_SPEED, 0.0, YAW_RATE), 'turn_back_right': (-BACK_SPEED, 0.0, -YAW_RATE),
}
LEAK_S, STANDING_KP, WEP_HOLD_S, WEP_S, WEP_FILL, KNEE_SOFT, KNEE_NM = 0.2, 0.1, 1.0, 8.0, 0.8, 0.1, 40.0
STUNT_MODES = {'sit_chair', 'rise_chair', 'lie_bed', 'leg_bed', 'rise_bed', 'lie_floor', 'rise_floor', 'hop_down_chair',
               'hop_up_chair', 'stairs_down', 'parry_shove', 'controlled_fall'}
STUNT_PHASES = {'approach', 'lower', 'seated', 'lean_forward', 'thrust', 'sit_edge', 'recline', 'bed_rest', 'sit_up',
                'swing_out', 'kneel', 'tip', 'floor_rest', 'push_arms', 'half_kneel', 'cushion', 'land', 'step_off', 'drop',
                'absorb', 'crouch_load', 'propulsion', 'flight_tuck', 'seat_land', 'descent'}
#: The doings that lie on the ground on purpose: the fall judge leaves them be (`referee.fallen`).
GROUNDED = frozenset({'kneel', 'tip', 'floor_rest'})
#: The doings that rest limp: its drives' clamps at zero over the bus, the boards' damping all that
#: holds (`motion.drives`) - sprung at the rest's own stiffness it churned on the floor, 200-1400
#: deg/s, pitch rolling full circle.
LIMP = frozenset({'floor_rest'})
#: Its back among what bears it (`ortho.MANIFOLDS`): lain on it, its row blended, it rests limp (`limp`).
DORSAL = ortho.MANIFOLDS.index('dorsal')
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


def braked(spd):
    """Its brake's blend from speed `spd` (m/s), s: its walk's stride at its rate, no shorter than its stand's."""
    return max(STAND_S, BRAKE_S * min(1.0, abs(float(spd)) / WALK_SPEED))


def stand_lead(spd):
    """Its brake's horizons `(ts, tg)` (s) from speed `spd`: it is told to stand once it has braked it
    (`braked`); what follows twice that."""
    ts = braked(spd)
    return ts, 2.0 * ts


def rise_lead(wire):
    """A rise stunt's own length, s: its last row, its rest given back once it stands."""
    import numpy as np
    from machine.law import latent
    return float(np.frombuffer(wire, latent.WIRE)[-1]['at']) / 1e3


def said(g, label, wire, names=(), at=None, reset=False):
    import numpy as np
    from machine.law import latent
    new = latent.table(wire, g['t'] + 0.001 if at is None else at, g['rest'])
    if reset and len(new):
        new[0, latent.VALS] = np.where(new[0, latent.SAID] > 0.5, new[0, latent.VALS], g['rest'])
        new[0, latent.SAID] = 1.0
    t_eat = g['t'] - 0.001 * ((g['n'] - 1) % g['every'] + 1) + 1e-6 if g.get('n', 0) > 0 else -1.0
    g['tab'] = np.concatenate([g['tab'][g['tab'][:, latent.AT] > t_eat], new])
    now = g.get('t_law', g['t']) + 1e-9
    past = [p for p in g['labels'] if p[0] <= now]
    ahead = [p for p in g['labels'] if p[0] > max(now, t_eat)]
    kept = ([max(past)] + ahead) if past else ahead
    g['labels'] = kept + [(float(w), names[i] if i < len(names) else label) for i, w in enumerate(new[:, latent.AT])]


def _dropped(g):
    g['tab'], g['labels'], g['_jump_until'], g['_held_s'] = (
        g['tab'][:0], [(w, n) for w, n in g['labels'] if w <= g['t']], 0.0, 0.0)
    g['_jump_t0'], g['_jump_throw'], g['_leap'], g['_jog_until'] = 0.0, 0.0, False, 0.0


def doing(g):
    """What it is doing: the name of its table's row that last came by its law's last pass, 'rest'
    with none; cached on `(labels, t_law)` (`g['_doing']`)."""
    labels, now = g['labels'], g.get('t_law', g['t'])
    kept = g.get('_doing')
    if kept is None or kept[0] is not labels or kept[1] != now:
        come = [(w, n) for w, n in labels if w <= now + 1e-9]
        kept = g['_doing'] = (labels, now, max(come)[1] if come else 'rest')
    return kept[2]


@functools.cache
def _mods():
    """machine.law's latent, modal and stride, imported once at a first pass."""
    from machine.law import latent, modal, stride
    return latent, modal, stride


def limp(g):
    """How far it rests limp, 0 to 1: a doing in LIMP or its back told to bear it, eased to 1 after
    its row's blend (`motion.drives`), woken smoothly when a twist is told outside a program."""
    latent = _mods()[0]
    blend, now = max(float(g['st'][latent.BLEND]), 1e-3), g.get('t_law', g['t'])
    eased = min(1.0, max(0.0, (now - float(g['st'][latent.T0]) - blend) / blend))
    told = float(g.get('played') is None and any(float(v) != 0.0 for v in g.get('twist', ())))
    way, at, was = g.get('_wake', (0.0, now, 0.0))
    woken = was + (way - was) * min(1.0, max(0.0, (now - at) / blend))
    if told != way:
        g['_wake'] = (told, now, woken)
    return eased * max(float(doing(g) in LIMP), float(float(g['told'][DORSAL]) >= 0.5)) * (1.0 - woken)


def closing(vx, vz, dx, dz):
    """(the gap to a stop `dx, dz` off it, its closing speed on it going `vx, vz`, m/s - none receding:
    a spin's drift away from it is no approach)."""
    gap = math.hypot(dx, dz)
    return gap, max(0.0, (vx * dx + vz * dz) / max(gap, 1e-9))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


_INF, _SV, _SW = math.inf, rhythm.R['backing_m_s'], rhythm.R['still_rad_s']
GAIT_BOXES = (
    (-_INF, _INF, _SV, _INF, -_INF, _INF, 'strafe_left'),
    (-_INF, _INF, -_INF, -_SV, -_INF, _INF, 'strafe_right'),
    (-_SV, _SV, -_SV, _SV, _SW, _INF, 'spin_left'),
    (-_SV, _SV, -_SV, _SV, -_INF, -_SW, 'spin_right'),
    (-_SV, _INF, -_SV, _SV, _SW, _INF, 'turn_left'),
    (-_SV, _INF, -_SV, _SV, -_INF, -_SW, 'turn_right'),
    (-_INF, -_SV, -_SV, _SV, _SW, _INF, 'turn_back_left'),
    (-_INF, -_SV, -_SV, _SV, -_INF, -_SW, 'turn_back_right'),
    (MARCH_FROM, _INF, -_SV, _SV, -_SW, _SW, 'jog'),
    (_SV, MARCH_FROM, -_SV, _SV, -_SW, _SW, 'walk'),
    (-_INF, -_SV, -_SV, _SV, -_SW, _SW, 'walk_back'),
)


def gait_for_twist(vx, vy, wz):
    """The gym's entry a twist walks it by, queried from `GAIT_BOXES`; None at rest."""
    moving = max(abs(vx) / _SV, abs(vy) / _SV, abs(wz) / _SW) > 1.0
    return next((name for x0, x1, y0, y1, z0, z1, name in GAIT_BOXES
                 if moving and x0 <= vx <= x1 and y0 <= vy <= y1 and z0 <= wz <= z1), None)


#: The twist step's memory, one array (`twist_laid`): which way its sway led last (nan till it
#: has), its look's yaw, its turn's offset. Its words with `g` each pass (`words_laid`): its
#: heading, the heading told and whether one is, its twist told and as made, whether a twist is
#: active and until when a stop holds, its time; out its heading, its twist as made, its backing
#: and its yaw, its clearance.
LAST_WAY, LOOK_YAW, TURN_DC, TWISTED = 0, 1, 2, 3
W_HEADING, W_TARGET, W_HAS, W_TWIST, W_ACTUAL, W_ACTIVE, W_STOP, W_T = 0, 1, 2, slice(3, 6), slice(6, 9), 9, 10, 11
W_BACK, W_BACKING, W_CLEAR, WORDS = 12, 13, 14, 15
#: Its heading's and its look's numbers under a twist, its build's (`[gait.steer]`), in the order its
#: step's C twin lays them (wbc_twist_t).
STEER_NAMES = ('slew_rad_s', 'side_told_m_s', 'turn_told_rad_s', 'along_s', 'across_s', 'told_s', 'stance_turn',
               'stance_turn_load', 'stance_turn_k', 'lag_rad', 'lead_1_s', 'align_m_s', 'align_side_m_s',
               'align_speed_m_s', 'align_own_m_s', 'align_rad', 'align_1_s', 'slack_rad', 'flip_gain', 'flip_drive',
               'look_drive', 'look_on', 'look_back_rad_s', 'look_turn_rad_s', 'look_rad', 'look_1_s', 'look_back_1_s',
               'waist_share', 'head_share', 'clear_m')
STEER = gait.table('steer')
#: The floor under it follows its bearing soles at FLOOR_HZ.
FLOOR_HZ = 15.0


def twist_laid():
    """The step's memory at rest, (TWISTED,): no way led yet, no look, no turn."""
    import numpy as np
    return np.array([math.nan, 0.0, 0.0])


def words_laid():
    """The step's words, (WORDS,), zeros."""
    import numpy as np
    return np.zeros(WORDS)


def floored(y, fr, dt):
    """The floor under it where its bearing soles' lowest corners stand, in its estimate's own
    height - held where it was laid it read its soles 22 mm over it as its estimate drifted up -,
    and its pitch under each sole: that sole's own while its heel's cell and its ball's both
    press, as far as they press alike. One centre of pressure read inside its edges took a
    hole's edge under a sole's middle for a floor 10 deg up."""
    import numpy as np
    from machine.moves import leap
    from machine.law import modal
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
    """A pass of its twist on the loop: `twisted` in C on its arrays (`g['twist_step']`,
    `machine.law.motion.laid`), its words from and to `g`."""
    modal = _mods()[1]
    g.setdefault('floor0', float(g['fr'][modal.FLOOR]))
    w = g['words']
    w[W_HEADING], w[W_TARGET], w[W_HAS] = g['heading'], g.get('target_heading', 0.0), g.get('has_heading', 0.0)
    w[W_TWIST], w[W_ACTUAL] = g.get('twist', (0.0, 0.0, 0.0)), g.get('twist_actual', (0.0, 0.0, 0.0))
    w[W_ACTIVE], w[W_STOP], w[W_T] = float(g.get('_twist_active', False)), g.get('_twist_stop_until', 0.0), g['t']
    g['twist_step'](dt, g['est']['vel'], g['held'], g['grip_prior'])
    g['heading'], g['twist_actual'] = float(w[W_HEADING]), tuple(w[W_ACTUAL].tolist())
    g['_w_back'], g['_backing_yaw'], g['_twist_clear'] = float(w[W_BACK]), float(w[W_BACKING]), float(w[W_CLEAR])


def twisted(y_in, vel, y, fr, a, st, out, posture, held, prior, state, words, dt, waist, head):
    """A pass of its twist on its edge's array `y_in` and its estimate's velocity `vel`, modal's
    `y`, `fr` and `a`, its latent `st` and its `out`, its `posture` (its `waist` and `head` in it),
    its soles' grip `held` and its `prior` of it: the floor under it (`floored`), its heading
    turned by the twist told and by its pelvis, its rhythm flipped where its sway leads the other
    way, its gait modulated (`rhythm.modulated`), its look into a turn; `state` (`twist_laid`) and
    `words` (`words_laid`) stepped in place. In C bit for bit (`step_physical_twist`)."""
    import numpy as np
    from machine.senses import edge
    from machine.law import latent, modal, stride
    from machine.moves import leap
    s, back = STEER, rhythm.R['backing_m_s']
    has_thd = float(words[W_HAS])
    heading = wrap_angle(float(words[W_HEADING]) + has_thd * max(-s['slew_rad_s'] * dt, min(s['slew_rad_s'] * dt, wrap_angle(float(words[W_TARGET]) - float(words[W_HEADING])))))
    sh, ch = math.sin(heading), math.cos(heading)
    floored(y, fr, dt)
    vx, vy, wz = (float(v) for v in words[W_TWIST])
    vxp, vyp, wzp = float(vel[0]) * sh + float(vel[2]) * ch, float(vel[0]) * ch - float(vel[2]) * sh, float(y_in[edge.GYRO][1])
    ax, ay, az = (float(v) for v in words[W_ACTUAL])
    w_ty = min(1.0, max(0.0, (abs(vy) - s['side_told_m_s']) / s['side_told_m_s']))
    w_tz = min(1.0, max(0.0, (abs(wz) - s['turn_told_rad_s']) / s['turn_told_rad_s']))
    kv, ky = min(1.0, dt / s['along_s']), min(1.0, dt / (s['across_s'] - s['told_s'] * w_ty))
    kz = min(1.0, dt / (s['across_s'] - s['told_s'] * w_tz))
    words[W_ACTUAL] = (ax + kv * (vxp - ax), ay + ky * (w_ty * vyp - ay), az + kz * (w_tz * wzp - az))
    pelvis_yaw = edge.yaw(y_in)
    unborne = 1.0 - float(a[modal.BORNE])
    w_yaw = min(1.0, abs(wz) / 1e-4) * (1.0 - has_thd) * unborne
    l0, l1 = float(y[modal.LOAD][0]), float(y[modal.LOAD][1])
    wz_way = (1.0 - 2.0 * float(vx < -back)) * wz
    tz_sgn = math.copysign(1.0, wz_way)
    out_st = s['stance_turn'] - s['stance_turn_load'] * math.tanh(s['stance_turn_k'] * (l0 - l1) * tz_sgn / max(l0 + l1, leap.TOUCH_N))
    d_yaw = wrap_angle(pelvis_yaw - heading)
    w_step_yaw = float(d_yaw * tz_sgn >= -s['lag_rad'])
    w_lead_yaw = float(d_yaw * wz_way > 0.0)
    heading = wrap_angle(heading + w_yaw * (w_step_yaw * out_st * wz_way + w_lead_yaw * s['lead_1_s'] * d_yaw) * dt)
    v_horz = math.hypot(float(y[modal.VCOM][0]), float(y[modal.VCOM][2]))
    w_align = (w_yaw * float(vx >= s['align_m_s']) * float(abs(vy) < s['align_side_m_s']) * float(v_horz > s['align_speed_m_s'])
               * float(vxp > s['align_own_m_s']))
    err_v = wrap_angle(math.atan2(float(y[modal.VCOM][0]), float(y[modal.VCOM][2])) - heading)
    err_v_ex = math.copysign(max(0.0, abs(err_v) - s['align_rad']), err_v)
    heading = wrap_angle(heading + w_align * s['align_1_s'] * err_v_ex * dt)
    err_yaw = wrap_angle(pelvis_yaw - heading)
    err_yaw_ex = math.copysign(max(0.0, abs(err_yaw) - s['slack_rad']), err_yaw)
    heading = wrap_angle(heading + w_yaw * err_yaw_ex)
    sh_n, ch_n = math.sin(heading), math.cos(heading)
    fr[modal.HEADING] = heading
    fr[modal.ROT][:] = (sh_n, 0.0, ch_n, ch_n, 0.0, -sh_n, 0.0, 1.0, 0.0)
    # each standing sole's turn of its way since it was planted, naught for one in the air (`stride.meets`)
    fr[modal.PLANTED] = a[modal.STANCE] * (fr[modal.PLANTED] + wrap_angle(heading - float(words[W_HEADING])))
    l_psi, d_psi = float(out[latent.PSI][modal.LATERAL]), float(out[latent.PSI][modal.DRIVE])
    l_sgn = math.copysign(1.0, float(out[latent.SWAY][modal.LATERAL]))
    cw = math.copysign(1.0, math.sin(l_psi - d_psi) * l_sgn + 0.5)
    last_way = float(np.nan_to_num(state[LAST_WAY], nan=cw))
    w_flip = float(cw * last_way < 0.0) * max(float(st[latent.NOW][latent.GAIN] > s['flip_gain']),
                                              float(abs(float(out[latent.SWAY][modal.DRIVE])) > s['flip_drive']))
    old_psi = out[latent.PSI].copy()
    st[latent.THETA] = (float(st[latent.THETA]) + w_flip * math.pi) % math.tau
    out[latent.PSI] = st[latent.THETA] + st[latent.NOW][latent.PHASE]
    sway, turns = out[latent.SWAY], float(out[latent.TURNS])
    out[latent.REF] += (-2.0 * w_flip * sway) * [math.sin(float(p)) for p in old_psi]
    out[latent.RATE] += (-2.0 * w_flip * turns * sway) * [math.cos(float(p)) for p in old_psi]
    out[latent.ACC] *= 1.0 - 2.0 * w_flip
    state[LAST_WAY] = cw
    rhythm.modulated(st, out, held, prior, state, float(words[W_ACTIVE]), vx, vy, wz, vxp)
    lat_on = (1.0 - has_thd) * float(abs(vy) > rhythm.R['side_m_s'])
    drv_abs = abs(float(out[latent.SWAY][modal.DRIVE]))
    drive_sway = max(drv_abs, lat_on * abs(float(out[latent.SWAY][modal.LATERAL])))
    active = min(1.0, max(0.0, (drive_sway - stride.STIRS) / s['look_drive'])) * unborne
    w_back = float(active > s['look_on']) * max(float(cw < 0.0) * float(drv_abs > stride.STIRS), float(vx < -back))
    tl_back = math.copysign(1.0, wz + s['look_back_rad_s'])
    tl_fwd = float(abs(wz) > s['look_turn_rad_s']) * ((s['look_rad'] * wz) / YAW_RATE)
    tl_side = lat_on * ((s['look_rad'] * vy) / STRAFE_SPEED)
    tl_way = max(-s['look_rad'], min(s['look_rad'], tl_fwd + tl_side))
    tl = active * (w_back * tl_back + (1.0 - w_back) * tl_way)
    look = state[LOOK_YAW] = float(state[LOOK_YAW]) + min(1.0, (s['look_1_s'] - s['look_back_1_s'] * w_back) * dt) * (tl - float(state[LOOK_YAW]))
    posture[waist] = s['waist_share'] * look * unborne
    posture[head] = s['head_share'] * look * unborne
    words[W_HEADING] = heading
    words[W_BACK] = w_back * float(abs(look) > 1e-3)
    words[W_BACKING] = posture[waist]
    tw_on = max(float(words[W_ACTIVE]), float(words[W_STOP] > words[W_T]))
    words[W_CLEAR] = s['clear_m'] * tw_on * unborne * float(out[latent.HELD] > 0.5)


def heard_twist(g, gym, tw, has_say, heard_say_fn):
    from machine.senses import touch
    latent, modal, stride = _mods()
    pvx, pvy, _ = g.get('twist', (0.0, 0.0, 0.0))
    was_tw = bool(g.get('_twist_active'))
    g['asked'] = (0.0, 0.0, 0.0) if tw is None else (float(tw[0]), float(tw[1]), float(tw[2]))
    vx, vy, wz = touch.against(g, g['asked'])
    by, cur_d = {e[0]: e for e in gym}, doing(g)
    # a twist less than the last is a key let go, not its errand's own braking: read so, F's run to the door slowed
    # 0.69 -> 0.53 m/s for a corner, stood, braked to nothing there, backed 0.6 m and fell at 6.1 s
    decay = (not has_say and not g.get('_errand') and math.hypot(vx, vy) < math.hypot(pvx, pvy) - 1e-4
             and -(BACK_SPEED - 0.01) < vx < WALK_SPEED - 0.01 and abs(vy) < STRAFE_SPEED - 0.01 and abs(wz) < 0.01)
    active = max(abs(vx), abs(vy), abs(wz)) > 0.01 and not decay
    was_on = bool(was_tw or g.get('active_entry') in WASD_GAITS or cur_d in WASD_GAITS)
    target = (gait_for_twist(vx, vy, wz) or ('walk_back' if vx < 0.0 else 'walk')) if active else ('stand' if was_on else None)
    ae = g.get('active_entry') or (cur_d if cur_d in SE2_GAITS else None)
    striding = (abs(float(g['out'][latent.SWAY][modal.DRIVE])) > stride.STIRS
                or abs(float(g['out'][latent.SWAY][modal.LATERAL])) > 0.08 or float(g['st'][latent.NOW][latent.GAIN]) > 0.02)
    lat_spin = ('strafe_left', 'strafe_right', 'spin_left', 'spin_right')
    if (active and not has_say and g.get('_errand') is None and striding and cur_d not in ('brake', 'stand', 'init_walk')
            and g.get('_jump_until', 0.0) <= g['t'] and ae in SE2_GAITS and target in SE2_GAITS):
        g['governed'] = g['governed'] if was_tw else NOMINAL_TWIST.get(str(ae), g['asked'])
        g['active_entry'], g['_twist_active'], g['_twist_stop_until'] = ae, True, g['t'] + 2.5
        if ae == 'jog':
            g['_jog_until'] = g['t'] + 0.5 * BRAKE_S
        g['twist'] = touch.against(g, g['governed'])
        return
    hold_tw = (not has_say and g.get('_errand') is None
               and ((cur_d == 'brake' and target in lat_spin) or cur_d in RISE_OF
                    or (g.get('_jump_until', 0.0) > g['t'] and not g.get('_leap'))))
    g['governed'], g['twist'] = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)) if hold_tw else (g['asked'], (vx, vy, wz))
    if target is not None:
        g['_twist_active'], g['_twist_stop_until'] = active, g['t'] + 2.5
    if not has_say and target in by:
        heard_say_fn(g, gym, by[target])


_LOOP_G, _MODES_G = gait.table('loop'), gait.table('modes')
_M_TOT = float(gait._G['mass_kg']) * gait.MASSY
_M_LEG = float(gait._G['leg_kg']) * gait.MASSY
_K_WALK, _D_WALK = round(_LOOP_G['height_1_s2'] * _M_TOT, -2), round(_LOOP_G['height_1_s'] * _M_TOT, -1)
_K_ROCK, _D_ROCK = round(_MODES_G['limb_1_s2'] * _M_LEG, -2), round(_MODES_G['limb_1_s'] * _M_LEG, -1)
WALK_ON = (
    '(stream_inject (sequence_id 204) (blend_duration %dms) (latent_targets %s) (impedance %s)'
    ' (cpg_config (enable true) (frequency %.4gHz) %s %s)'
    ' (task_constraints (maintain_zmp (polygon dynamic_footprint))))' % (
        round(750.0 / WALK_HZ),
        ' '.join('(mode_%s %g)' % kv for kv in (
            ('axial_compression', 0.03), ('pitch_attitude', 0.01), ('sagittal_shift', 0),
            ('lateral_shift', 0), ('pelvis_turn', 0), ('stance_width', 0))),
        ' '.join('(stiffness mode_%s %g) (damping mode_%s %g)' % (m, k, m, d) for m, k, d in (
            ('axial_compression', _K_WALK, _D_WALK), ('lateral_shift', _K_WALK, 0.5 * _D_WALK),
            ('sagittal_shift', 0, round(0.4 * _D_WALK, -1)), ('foot_rock', _K_ROCK, _D_ROCK))),
        WALK_HZ,
        ' '.join('(amplitude mode_%s %g)' % kv for kv in (
            ('lateral_shift', 0.28), ('sagittal_drive', 0.85), ('limb_lift', 1.05), ('foot_rock', 0.6),
            ('pelvis_list', 0.4), ('pelvis_turn', 0.4), ('arm_swing', 0.8), ('stance_width', 0))),
        ' '.join('(phase_offset mode_%s mode_sagittal_drive %ddeg)' % kv for kv in (
            ('lateral_shift', 108), ('pelvis_list', 108), ('pelvis_turn', 0),
            ('arm_swing', 0), ('foot_rock', 0)))))
_WALK_ON_WIRE = ortho.compiled(WALK_ON)[0]['']


def stride_hop(g, by, t0, spd, back_now, fwd_gaits, back_gaits, said_fn):
    from machine.moves import leap
    return leap.stride_hop(g, by, t0, spd, back_now, fwd_gaits, back_gaits, said_fn, _WALK_ON_WIRE)
