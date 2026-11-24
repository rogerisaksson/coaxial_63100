"""Pre-dock checkpoint scenario evaluators for parallel motor engram Monte Carlo sweeps."""
import math
import os


def worker_init():
    """Pin single-threaded BLAS and deterministic kernels in each worker subprocess."""
    from tools.dev import arithmetic
    os.environ['OPENBLAS_NUM_THREADS'] = '1'
    arithmetic.hold()

SCENES = {
    'chair_dock': {'task': 'chair', 'world_scene': 'chair_behind', 'room': 0.0, 'stunt': 'stand_to_sit'},
    'chair_low_dock': {'task': 'chair', 'world_scene': 'chair_low_behind', 'room': 0.0, 'stunt': 'stand_to_sit'},
    'lodge_chair_dock': {'task': 'chair', 'world_scene': 'lodge', 'room': 1.0, 'stunt': 'stand_to_sit'},
    'bed_dock': {'task': 'bed', 'world_scene': 'bed_foot_behind', 'room': 0.0, 'stunt': 'lying_down_from_sitting'},
    'crouch_dock': {'task': 'crouch', 'world_scene': 'flat', 'room': 0.0, 'stunt': 'crouching_down'},
    'turn_dock': {'task': 'turn', 'world_scene': 'flat', 'room': 0.0, 'stunt': 'left_turn'},
}


def chair_geom(m, scene_name):
    """Return `(geom_id, geom_name)` for the target chair seat in `scene_name`."""
    explicit = {'chair_dock': 'chair_seat', 'chair_low_dock': 'chair_low_seat'}.get(scene_name)
    if explicit is not None:
        return m.geom(explicit).id, explicit
    for g in range(m.ngeom):
        name = m.geom(g).name or ''
        if 'chair' in name and 'seat' in name:
            return g, name
    for g in range(m.ngeom):
        name = m.geom(g).name or ''
        if 'chair' in name:
            return g, name
    raise KeyError('no chair geom found for %s' % scene_name)


def apply_wobble(world, wobble):
    """Apply yaw perturbation `wobble` rad to the robot pelvis in `world` before branching."""
    if not wobble:
        return
    d = world.data
    q = d.qpos[3:7].copy()
    cw, sw = math.cos(0.5 * wobble), math.sin(0.5 * wobble)
    d.qpos[3:7] = (q[0] * cw - q[2] * sw, q[1] * cw + q[3] * sw, q[2] * cw + q[0] * sw, q[3] * cw - q[1] * sw)
    world._mj.mj_forward(world.model, d)


def setup_checkpoint(scene_name, sight=False):
    """Initialize a session for `scene_name`, advance to pre-dock state, and return `(s, snap_or_none)`."""
    from machine.process import checkpoint, worker
    from machine.world import physics
    from tools.sim.player import STOP, WALK

    spec = SCENES[scene_name]
    physics.ROOM = float(spec['room'])
    physics.CAMERA = physics.SIGHT = float(bool(sight))
    s = worker.session()
    if sight:
        worker.readied(s)
    if not spec['room']:
        worker.apply(s, {'scenario': spec['world_scene']})
    if scene_name == 'lodge_chair_dock':
        worker.apply(s, {'heading': 1.0, 'twist': WALK})
        for _ in range(600):
            worker.stepped(s)
        worker.apply(s, {'twist': STOP})
        for _ in range(300):
            worker.stepped(s)
    elif scene_name == 'turn_dock':
        worker.apply(s, {'say': 'walk'})
        for _ in range(950):
            worker.stepped(s)
    else:
        for _ in range(400):
            worker.stepped(s)
    return s, (None if sight else checkpoint.capture(s))


def record_step(g, w, hands_id, rows, track, told_at, prev_at=None, jump=0.0, track_jump=False):
    """Append hand positions, throttled look rows, and optional move-entry handover jump."""
    import numpy as np
    from machine.senses import edge
    from tools.sim import look, scenarios
    if track_jump:
        p = g.get('played')
        if p is not None and p.get('at') != prev_at and p.get('at') is not None:
            prev_at = p['at']
            jump = max(jump, float(np.max(np.abs(p['cmd'] - np.degrees(g['sense'][edge.Q])))))
    track.append(w.data.xpos[hands_id].ravel().tolist())
    if rows is not None and g['t'] - told_at >= 1.0 / look.RATE_HZ:
        told_at = g['t']
        rows.append(scenarios._row(g, w))
    return told_at, prev_at, jump


def prep_trial(s, combo, wobble):
    """Apply yaw wobble, hot-swap `combo`, and return `(g, w, m, d, hands_id)`."""
    from machine.process import engrams
    g = s['g']
    w = g['world']
    apply_wobble(w, wobble)
    for name in combo:
        if name:
            engrams.swap(g, name, s=s)
    m = w.model
    return g, w, m, w.data, [m.body('left_hand').id, m.body('right_hand').id]


def hand_jerk(track):
    """Compute 99th-percentile smoothed hand jerk magnitude (`m/s^3`) from 1 kHz `(left, right)` positions."""
    import numpy as np
    if len(track) < 100:
        return 0.0
    arr = np.asarray(track, float)
    w = 20
    best = 0.0
    for sl in (slice(0, 3), slice(3, 6)):
        p = arr[:, sl]
        sm = np.cumsum(p, axis=0)[w:] / w - np.cumsum(p, axis=0)[:-w] / w
        j = np.linalg.norm(np.diff(sm, n=3, axis=0), axis=1) / (0.001 ** 3)
        best = max(best, float(np.percentile(j, 99)))
    return best


def _stunt_far_from_rows(rows, stunt_key, t0, t1):
    """Evaluate dynamic-similarity `stunts.far` distance against reference take `stunt_key`."""
    from tools.sim import staged, stunts
    ref = stunts.reference().get(stunt_key)
    if not ref or len(rows) < 6 or t1 <= t0 + 0.1:
        return math.nan
    sig = staged.STUNTS[stunt_key][4]
    moved = stunts.moved(rows, t0, t1)
    return float(stunts.far(moved, ref, sig))


def eval_chair(s, scene_name, combo, wobble):
    """Run one pre-dock chair trial (`mount` x `dismount`) and return metrics plus world verdict."""
    import numpy as np
    from machine.body import figure
    from machine.law import modal
    from machine.process import sprung, worker

    g, w, m, d, hands_id = prep_trial(s, combo, wobble)
    gid, _gname = chair_geom(m, scene_name)
    bid = int(m.geom_bodyid[gid])
    feet_ids = {m.body(n).id for n in ('left_foot', 'left_toes', 'right_foot', 'right_toes')}
    seat_parts = {m.body(n).id for n in ('pelvis', 'left_thigh', 'right_thigh')}
    hand_ids = {m.body(n).id for n in ('left_hand', 'left_fingers', 'right_hand', 'right_fingers')}
    pelvis_id = m.body('pelvis').id
    robot_dofs = [int(m.jnt_dofadr[m.joint(jn).id]) for jn in figure.JOINTS]
    wt, f6 = float(g['fr'][modal.WEIGHT]), np.zeros(6)

    t0 = float(g['t'])
    worker.apply(s, {'say': 'sit_chair'})
    hold_s, max_sit_s, max_rise_s = 2.5, (14.0 if scene_name != 'lodge_chair_dock' else 18.0), 10.0
    t_hold = None
    ch_tilt_max = qvel_hold = soles_max = handover_jump = 0.0
    pel_front_min = seat_up_min = math.inf
    rows, track, told_at, prev_played_at = [], [], -math.inf, None

    while g['t'] < t0 + max_sit_s and s['fallen_at'] is None and s['epoch'] == 0:
        worker.stepped(s)
        told_at, prev_played_at, handover_jump = record_step(
            g, w, hands_id, rows, track, told_at, prev_played_at, handover_jump, True)
        sR = d.geom_xmat[gid].reshape(3, 3)
        ch_tilt_max = max(ch_tilt_max, math.degrees(math.acos(max(-1.0, min(1.0, float(sR[1, 1]))))))
        if t_hold is None and sprung.doing(g) == 'seated' and g.get('played') is None:
            t_hold = float(g['t'])
        if t_hold is not None:
            if g['t'] >= t_hold + hold_s:
                break
            if g['n'] % 10 == 0:
                pel = sR.T @ (d.xpos[pelvis_id] - d.geom_xpos[gid])
                pel_front_min = min(pel_front_min, float(pel[2] + m.geom_size[gid][2]))
                soles_n = seat_up = 0.0
                for i in range(d.ncon):
                    c = d.contact[i]
                    b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
                    if ((b1 in feet_ids and b2 not in seat_parts and b2 not in feet_ids and b2 not in hand_ids)
                            or (b2 in feet_ids and b1 not in seat_parts and b1 not in feet_ids and b1 not in hand_ids)):
                        w._mj.mj_contactForce(m, d, i, f6)
                        soles_n += abs(float(f6[0]))
                    if (b1 in seat_parts and b2 == bid) or (b2 in seat_parts and b1 == bid):
                        w._mj.mj_contactForce(m, d, i, f6)
                        seat_up += abs(float(f6[0]) * float(c.frame[1]))
                soles_max = max(soles_max, soles_n)
                seat_up_min = min(seat_up_min, seat_up / wt)
                if g['t'] >= t_hold + 0.5:
                    qvel_hold = max(qvel_hold, float(np.abs(d.qvel[robot_dofs]).max()))

    t_sit_end = float(g['t'])
    rose_ok = False
    if s['fallen_at'] is None and t_hold is not None:
        worker.apply(s, {'say': 'rise_chair'})
        t_r0 = float(g['t'])
        while g['t'] < t_r0 + max_rise_s and s['fallen_at'] is None and s['epoch'] == 0:
            worker.stepped(s)
            _, prev_played_at, handover_jump = record_step(
                g, w, hands_id, None, track, told_at, prev_played_at, handover_jump, True)
            sR = d.geom_xmat[gid].reshape(3, 3)
            ch_tilt_max = max(ch_tilt_max, math.degrees(math.acos(max(-1.0, min(1.0, float(sR[1, 1]))))))
            if sprung.doing(g) in ('stand', 'rest') and g.get('played') is None:
                rise_soles = sum(g['loads']) / wt
                if g['t'] >= t_r0 + 1.0 and rise_soles >= 0.85:
                    rose_ok = True
                    break

    reasons = []
    if s['fallen_at'] is not None or s['epoch'] != 0:
        reasons.append('fell')
    if t_hold is None:
        reasons.append('no_hold')
    if pel_front_min < 0.18:
        reasons.append('shallow_seat')
    if soles_max >= 1.5:
        reasons.append('foot_load')
    if seat_up_min < 0.85:
        reasons.append('low_seat_load')
    if ch_tilt_max >= 6.0:
        reasons.append('chair_tilt')
    if qvel_hold >= 1.8:
        reasons.append('spasm')
    if not rose_ok:
        reasons.append('rise_failed')
    far_sit = _stunt_far_from_rows(rows, 'stand_to_sit', t0, t_sit_end)
    score = round(100.0 * math.exp(-0.08 * max(0.0, far_sit)) if (not reasons and not math.isnan(far_sit)) else 0.0, 2)
    return {
        'ok': not reasons, 'reasons': reasons, 'stunt_far': far_sit, 'score': score,
        'handover_deg': handover_jump, 'jerk': hand_jerk(track),
        'seat_depth_m': round(float(pel_front_min if pel_front_min < 9.0 else -1.0), 3),
        'chair_tilt_deg': round(ch_tilt_max, 2),
    }


def eval_bed(s, combo, wobble):
    """Run one pre-dock bed trial and grade bed bearing, alignment, jerk, and mocap similarity."""
    import numpy as np
    from machine.law import modal
    from machine.process import worker
    from machine.senses import edge
    from tools.sim import scenarios

    g, w, m, d, hands_id = prep_trial(s, combo, wobble)
    robot_own = np.array([edge._rooted(m, b, w.pelvis) for b in range(m.nbody)], int)
    robot_own[[m.body(n).id for n in edge.INSOLES]] = 2
    kinds = {m.body(n).id: k for n, k in scenarios.BEARS.items()}
    force = np.zeros(6)

    t0 = float(g['t'])
    worker.apply(s, {'say': 'lie_bed'})
    bore_bed, ended_bed, ended_soles = 0.0, 0.0, 0.0
    rows, track, told_at = [], [], -math.inf
    handover_jump, prev_played_at = 0.0, None
    while g['t'] < t0 + 15.0 and s['fallen_at'] is None and s['epoch'] == 0:
        worker.stepped(s)
        told_at, prev_played_at, handover_jump = record_step(
            g, w, hands_id, rows, track, told_at, prev_played_at, handover_jump, True)
        if g['n'] % 10 == 0:
            now, _ = scenarios.borne(w, robot_own, kinds, force)
            ended_bed, ended_soles = now.get('bed', 0.0), now.get('soles', 0.0)
            if ended_bed > ended_soles:
                bore_bed += 0.01
    qw, qx, qy, qz = (float(v) for v in d.qpos[3:7])
    rolled = abs(math.degrees(math.atan2(2.0 * (qw * qz + qx * qy),
                                         math.hypot(1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qz - qw * qy)))))
    reasons = []
    if s['fallen_at'] is not None or s['epoch'] != 0:
        reasons.append('fell')
    if bore_bed < scenarios.HELD_S:
        reasons.append('insufficient_bed_hold')
    if ended_bed <= ended_soles:
        reasons.append('not_lain_on_bed')
    if rolled > math.degrees(modal.LIST_RAD):
        reasons.append('excess_roll')
    far_bed = _stunt_far_from_rows(rows, 'lying_down_from_sitting', t0, float(g['t']))
    score = round(100.0 * math.exp(-0.08 * max(0.0, far_bed)) if (not reasons and not math.isnan(far_bed)) else 0.0, 2)
    return {
        'ok': not reasons, 'reasons': reasons, 'stunt_far': far_bed, 'score': score,
        'handover_deg': handover_jump, 'jerk': hand_jerk(track),
        'bed_bore_s': round(bore_bed, 2), 'rolled_deg': round(rolled, 1),
    }


def eval_crouch(s, combo, wobble):
    """Run one crouch-down -> crouch-up combinatorial trial against mocap crouch stunts."""
    from machine.process import engrams, worker

    g, w, _m, d, hands_id = prep_trial(s, combo, wobble)
    down_slot = engrams.get(combo[0]).slot
    up_slot = engrams.get(combo[1]).slot
    y0 = y_min = float(d.qpos[1])
    rows, track, told_at, t0 = [], [], -math.inf, float(g['t'])

    worker.apply(s, {'say': down_slot})
    while g['t'] < t0 + 3.6 and s['fallen_at'] is None and s['epoch'] == 0:
        worker.stepped(s)
        y_min = min(y_min, float(d.qpos[1]))
        told_at, _, _ = record_step(g, w, hands_id, rows, track, told_at)
    t_mid = float(g['t'])
    if s['fallen_at'] is None and s['epoch'] == 0:
        worker.apply(s, {'say': up_slot})
        while g['t'] < t_mid + 4.0 and s['fallen_at'] is None and s['epoch'] == 0:
            worker.stepped(s)
            told_at, _, _ = record_step(g, w, hands_id, rows, track, told_at)

    drop = y0 - y_min
    recovered = float(d.qpos[1]) >= 0.88 * y0
    reasons = []
    if s['fallen_at'] is not None or s['epoch'] != 0:
        reasons.append('fell')
    if drop < 0.10:
        reasons.append('shallow_crouch')
    if not recovered:
        reasons.append('incomplete_rise')
    far_crouch = _stunt_far_from_rows(rows, 'crouching_down', t0, t_mid)
    score = round(100.0 * math.exp(-0.20 * max(0.0, far_crouch)) if (not reasons and not math.isnan(far_crouch)) else 0.0, 2)
    return {
        'ok': not reasons, 'reasons': reasons, 'stunt_far': far_crouch, 'score': score,
        'handover_deg': 0.0, 'jerk': hand_jerk(track), 'drop_m': round(drop, 3),
    }


def eval_turn(s, combo, wobble):
    """Run one turn-left combinatorial trial against `left_turn` mocap take."""
    from machine.process import engrams, worker

    g, w, _m, _d, hands_id = prep_trial(s, combo, wobble)
    slot = engrams.get(combo[0]).slot
    yaw0, t0 = float(g['heading']), float(g['t'])
    rows, track, told_at = [], [], -math.inf

    worker.apply(s, {'say': slot})
    while g['t'] < t0 + 2.5 and s['fallen_at'] is None and s['epoch'] == 0:
        worker.stepped(s)
        told_at, _, _ = record_step(g, w, hands_id, rows, track, told_at)
    dyaw = math.degrees(float(g['heading']) - yaw0)
    reasons = []
    if s['fallen_at'] is not None or s['epoch'] != 0:
        reasons.append('fell')
    if abs(dyaw) < 15.0:
        reasons.append('insufficient_turn')
    far_turn = _stunt_far_from_rows(rows, 'left_turn', t0, float(g['t']))
    score = round(100.0 * math.exp(-0.08 * max(0.0, far_turn)) if (not reasons and not math.isnan(far_turn)) else 0.0, 2)
    return {
        'ok': not reasons, 'reasons': reasons, 'stunt_far': far_turn, 'score': score,
        'handover_deg': 0.0, 'jerk': hand_jerk(track), 'dyaw_deg': round(dyaw, 1),
    }
