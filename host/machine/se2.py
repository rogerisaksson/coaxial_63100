"""Her twist on the loop: the gait it walks her by, her heading turned by it, her rhythm by it."""
import math

from machine import ortho

#: Her twists' speeds: a walk, a sprint, a walk back, a sidestep, m/s; a turn, rad/s.
WALK_SPEED, RUN_SPEED, BACK_SPEED, STRAFE_SPEED, YAW_RATE = 0.69, 1.40, 0.35, 0.50, 0.70

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


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def gait_for_twist(vx, vy, wz):
    """The gym's entry a twist walks her by: a sidestep, an arc or a turn in place, an arc back,
    a jog past 0.96 m/s, a walk, a walk back; None at rest."""
    if abs(vy) > 0.05:
        return 'strafe_left' if vy > 0 else 'strafe_right'
    if abs(wz) > 0.05:
        return ('turn_left' if wz > 0 else 'turn_right') if vx >= -0.05 else ('turn_back_left' if wz > 0 else 'turn_back_right')
    return 'jog' if vx > 0.96 else ('#204' if vx > 0.05 else ('#205' if vx < -0.05 else None))


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


#: The floor under her follows her bearing soles at FLOOR_HZ.
FLOOR_HZ = 15.0


def floored(g, y, dt):
    """The floor under her where her bearing soles' lowest corners stand, in her estimate's own
    height - held where she was laid it read her soles 22 mm over it as her estimate drifted up -,
    and its pitch under each sole: that sole's own while its heel's cell and its ball's both
    press, as far as they press alike. One centre of pressure read inside its edges took a
    hole's edge under a sole's middle for a floor 10 deg up."""
    import numpy as np
    from machine import edge, flight, modal
    bear = np.array(g['loads']) * (np.array(g['loads']) >= flight.TOUCH_N)
    under = float(bear @ y[modal.LOWEST]) / max(float(bear.sum()), 1e-9)
    k = float(bear.sum() > 0.0) * min(1.0, FLOOR_HZ * dt)
    g['fr'][modal.FLOOR] += k * (under - float(g['fr'][modal.FLOOR]))
    at = bear @ y[modal.SOLE].reshape(2, 3)[:, [0, 2]] / max(float(bear.sum()), 1e-9)
    g['fr'][modal.FLOOR_AT] += k * (at - g['fr'][modal.FLOOR_AT])
    cells = g['sense'][edge.CELLS].reshape(2, 2)
    flat = (cells.min(axis=1) >= flight.TOUCH_N) * cells.min(axis=1) / np.maximum(cells.max(axis=1), 1e-9)
    g['fr'][modal.GROUND] += min(1.0, FLOOR_HZ * dt) * flat * (y[modal.TILT] - g['fr'][modal.GROUND])


def step_physical_twist(g, dt, out=None):
    from machine import edge, latent, modal, sprung
    y, cur_entry = g['y'], g.get('active_entry')
    g.setdefault('floor0', float(g['fr'][modal.FLOOR]))
    v, hd = g['est']['vel'], float(g['heading'])
    sh, ch = math.sin(hd), math.cos(hd)
    floored(g, y, dt)
    cur_doing = sprung.doing(g)
    twist_mode = bool(g.get('_twist_active') or g.get('_twist_stop_until', 0.0) > g['t'] or cur_entry in WASD_GAITS or cur_doing == 'brake')
    is_hop = cur_doing in ('load', 'throw', 'land') and cur_entry not in ('controlled_fall', 'hop_up_chair', 'hop_down_chair')
    vx, vy, wz = g.get('twist', (0.0, 0.0, 0.0))
    tx, ty, tz = (vx, vy, wz) if g.get('_twist_active') else NOMINAL_TWIST.get(cur_entry, (0.0, 0.0, 0.0))
    vxp, vyp, wzp = float(v[0]) * sh + float(v[2]) * ch, float(v[0]) * ch - float(v[2]) * sh, float(g['sense'][edge.GYRO][1])
    dt_ld = (g['t'] - g['_land_touch_t']) if g.get('_land_touch_t') is not None else -1.0
    if out is not None and is_hop and not (g.get('_leap') and dt_ld > 0.06):
        out[latent.SWAY][:] = 0.0
        for m in (modal.DRIVE, modal.LIFT, modal.LATERAL, modal.TURN, modal.LIST):
            out[latent.REF][m] = out[latent.RATE][m] = out[latent.ACC][m] = 0.0
        out[latent.REF][modal.WIDTH] = min(float(out[latent.REF][modal.WIDTH]), 0.04)
        out[latent.RATE][modal.WIDTH] = out[latent.ACC][modal.WIDTH] = 0.0
        if max(g['loads']) <= 10.0 and cur_doing in ('throw', 'land'):
            out[latent.REF][modal.AXIAL] = min(1.10, max(0.20, 0.70 + 0.40 * float(v[1])))
            out[latent.KS][modal.LIFT], out[latent.DS][modal.LIFT] = 680.0, 60.0
    ax, ay, az = g.get('twist_actual', (0.0, 0.0, 0.0))
    kv, ky, kz = min(1.0, dt / 0.06), min(1.0, dt / (0.07 if abs(ty) > 0.01 else 0.10)), min(1.0, dt / (0.07 if abs(tz) > 0.01 else 0.10))
    g['twist_actual'] = (
        ax + kv * (vxp - ax),
        ay + ky * ((vyp if abs(ty) > 0.01 else 0.0) - ay),
        az + kz * ((wzp if abs(tz) > 0.01 else 0.0) - az),
    )
    if g.get('_jump_until', 0.0) > g['t']:
        return
    pelvis_yaw = edge.yaw(g['sense'])
    if abs(tz) > 1e-6 and g.get('target_heading') is None:
        l0, l1 = float(g['loads'][0]), float(g['loads'][1])
        out_st = 0.45 if ((tz > 0.0 and l0 > l1 + 60.0) or (tz < 0.0 and l1 > l0 + 60.0)) else 1.05
        d_yaw = wrap_angle(pelvis_yaw - g['heading'])
        if d_yaw * math.copysign(1.0, tz) >= -0.10:
            g['heading'] = wrap_angle(g['heading'] + out_st * tz * dt)
        if d_yaw * tz > 0.0:
            g['heading'] = wrap_angle(g['heading'] + 10.0 * d_yaw * dt)
    # turning, her heading goes with her body; walking on, it is where she was told: held to her
    # pelvis and her going, a yaw of 0.22 rad off the floor after a sill turned her way and she
    # crabbed down; after her going alone, the hole, the wall and the tree went with 5 shoves
    if twist_mode and abs(tz) > 1e-6 and g.get('target_heading') is None and cur_doing not in sprung.STUNT_PHASES:
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


def heard_twist(g, gym, tw, has_say, said_fn, dropped_fn, heard_say_fn, walk_on):
    from machine import latent
    pvx, pvy, _ = g.get('twist', (0.0, 0.0, 0.0))
    g['asked'] = (0.0, 0.0, 0.0) if tw is None else (float(tw[0]), float(tw[1]), float(tw[2]))
    from machine import touch
    g['twist'] = touch.against(g, g['asked'])
    vx, vy, wz = g['twist']
    by, jumping = {e[0]: e for e in gym}, g.get('_jump_until', 0.0) > g['t']
    decay = (not has_say and math.hypot(vx, vy) < math.hypot(pvx, pvy) - 1e-4
             and abs(vx) < WALK_SPEED - 0.01 and abs(vy) < STRAFE_SPEED - 0.01 and abs(wz) < 0.01)
    if (abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01) and not decay:
        label = gait_for_twist(vx, vy, wz) or ('#205' if vx < -0.01 else '#204')
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
