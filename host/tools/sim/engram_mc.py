#!/usr/bin/env python3
"""Parallel Monte Carlo combinatorics over hot-swappable motor engrams from pre-dock checkpoints.

    python tools/sim/engram_mc.py --list
    python tools/sim/engram_mc.py chair_dock crouch_dock --samples 3 --jobs 16
    python tools/sim/engram_mc.py --quick --samples 2

Each scenario captures a bit-exact simulation checkpoint (`machine.process.checkpoint`) immediately
before docking or maneuver transition, then fans combinations from `machine.process.engrams` across
worker processes (`ProcessPoolExecutor`), restoring the pre-dock checkpoint per trial and grading
world-truth stability alongside dynamic-similarity distance (`tools.sim.stunts`) to `stunts/`.
"""
import argparse
import concurrent.futures
import itertools
import json
import math
import os
import random
import sys

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OPENBLAS_CORETYPE', 'Nehalem')

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

SCENES = {
    'chair_dock': {'task': 'chair', 'world_scene': 'chair_behind', 'room': 0.0, 'stunt': 'stand_to_sit'},
    'chair_low_dock': {'task': 'chair', 'world_scene': 'chair_low_behind', 'room': 0.0, 'stunt': 'stand_to_sit'},
    'lodge_chair_dock': {'task': 'chair', 'world_scene': 'lodge', 'room': 1.0, 'stunt': 'stand_to_sit'},
    'bed_dock': {'task': 'bed', 'world_scene': 'bed_foot_behind', 'room': 0.0, 'stunt': 'lying_down_from_sitting'},
    'crouch_dock': {'task': 'crouch', 'world_scene': 'flat', 'room': 0.0, 'stunt': 'crouching_down'},
    'turn_dock': {'task': 'turn', 'world_scene': 'flat', 'room': 0.0, 'stunt': 'left_turn'},
}


def _worker_init():
    """Pin single-threaded BLAS in each worker subprocess so parallel pools scale linearly."""
    os.environ['OPENBLAS_NUM_THREADS'] = '1'
    os.environ['OPENBLAS_CORETYPE'] = 'Nehalem'


def _chair_geom(m, scene_name):
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


def _apply_wobble(world, wobble):
    """Apply yaw perturbation `wobble` rad to the robot pelvis in `world` before branching."""
    if not wobble:
        return
    d = world.data
    q = d.qpos[3:7].copy()
    cw, sw = math.cos(0.5 * wobble), math.sin(0.5 * wobble)
    d.qpos[3:7] = (q[0] * cw - q[2] * sw, q[1] * cw + q[3] * sw, q[2] * cw + q[0] * sw, q[3] * cw - q[1] * sw)
    world._mj.mj_forward(world.model, d)


def _setup_checkpoint(scene_name):
    """Initialize a session for `scene_name` and capture the bit-exact pre-dock checkpoint."""
    from machine.process import checkpoint, worker
    from machine.world import physics
    from tools.sim.player import STOP, WALK

    spec = SCENES[scene_name]
    physics.ROOM, physics.SIGHT = float(spec['room']), 0.0
    s = worker.session()
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
    return s, checkpoint.capture(s)


def _hand_jerk_from_track(track):
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


def _eval_chair_trial(s, scene_name, combo, wobble):
    """Run one pre-dock chair trial (`mount` x `dismount`) and return metrics plus world verdict."""
    import numpy as np
    from machine.body import figure
    from machine.law import modal
    from machine.process import engrams, sprung, worker
    from machine.senses import edge
    from tools.sim import look, scenarios

    g = s['g']
    w = g['world']
    m, d = w.model, w.data
    _apply_wobble(w, wobble)
    for name in combo:
        engrams.swap(g, name, s=s)
    gid, _gname = _chair_geom(m, scene_name)
    bid = int(m.geom_bodyid[gid])
    feet_ids = {m.body(n).id for n in ('left_foot', 'left_toes', 'right_foot', 'right_toes')}
    seat_parts = {m.body(n).id for n in ('pelvis', 'left_thigh', 'right_thigh')}
    hand_ids = {m.body(n).id for n in ('left_hand', 'left_fingers', 'right_hand', 'right_fingers')}
    hands_id = [m.body('left_hand').id, m.body('right_hand').id]
    pelvis_id = m.body('pelvis').id
    robot_dofs = [int(m.jnt_dofadr[m.joint(jn).id]) for jn in figure.JOINTS]
    wt = float(g['fr'][modal.WEIGHT])
    f6 = np.zeros(6)

    t0 = float(g['t'])
    worker.apply(s, {'say': 'sit_chair'})
    hold_s, max_sit_s, max_rise_s = 2.5, (14.0 if scene_name != 'lodge_chair_dock' else 18.0), 10.0
    t_hold = None
    ch_tilt_max = pel_front_min = qvel_hold = 0.0
    pel_front_min = math.inf
    soles_max = 0.0
    seat_up_min = math.inf
    rows, track, told_at = [], [], -math.inf
    handover_jump = 0.0
    prev_played_at = None

    while g['t'] < t0 + max_sit_s and s['fallen_at'] is None and s['epoch'] == 0:
        worker.stepped(s)
        p = g.get('played')
        if p is not None and p.get('at') != prev_played_at and p.get('at') is not None:
            prev_played_at = p['at']
            handover_jump = max(handover_jump, float(np.max(np.abs(p['cmd'] - np.degrees(g['sense'][edge.Q])))))
        track.append(d.xpos[hands_id].ravel().tolist())
        if g['t'] - told_at >= 1.0 / look.RATE_HZ:
            told_at = g['t']
            rows.append(scenarios._row(g, w))
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
    rise_soles = 0.0
    if s['fallen_at'] is None and t_hold is not None:
        worker.apply(s, {'say': 'rise_chair'})
        t_r0 = float(g['t'])
        while g['t'] < t_r0 + max_rise_s and s['fallen_at'] is None and s['epoch'] == 0:
            worker.stepped(s)
            p = g.get('played')
            if p is not None and p.get('at') != prev_played_at and p.get('at') is not None:
                prev_played_at = p['at']
                handover_jump = max(handover_jump, float(np.max(np.abs(p['cmd'] - np.degrees(g['sense'][edge.Q])))))
            track.append(d.xpos[hands_id].ravel().tolist())
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
    return {
        'ok': not reasons, 'reasons': reasons, 'stunt_far': far_sit,
        'handover_deg': handover_jump, 'jerk': _hand_jerk_from_track(track),
        'seat_depth_m': round(float(pel_front_min if pel_front_min < 9.0 else -1.0), 3),
        'chair_tilt_deg': round(ch_tilt_max, 2),
    }


def _eval_bed_trial(s, combo, wobble):
    """Run one pre-dock bed trial and grade bed bearing, alignment, jerk, and mocap similarity."""
    import numpy as np
    from machine.law import modal
    from machine.process import engrams, worker
    from machine.senses import edge
    from tools.sim import look, scenarios

    g = s['g']
    w = g['world']
    m, d = w.model, w.data
    _apply_wobble(w, wobble)
    for name in combo:
        engrams.swap(g, name, s=s)
    robot_own = np.array([edge._rooted(m, b, w.pelvis) for b in range(m.nbody)], int)
    robot_own[[m.body(n).id for n in edge.INSOLES]] = 2
    kinds = {m.body(n).id: k for n, k in scenarios.BEARS.items()}
    hands_id = [m.body('left_hand').id, m.body('right_hand').id]
    force = np.zeros(6)

    t0 = float(g['t'])
    worker.apply(s, {'say': 'lie_bed'})
    bore_bed, ended_bed, ended_soles = 0.0, 0.0, 0.0
    rows, track, told_at = [], [], -math.inf
    handover_jump, prev_played_at = 0.0, None
    while g['t'] < t0 + 15.0 and s['fallen_at'] is None and s['epoch'] == 0:
        worker.stepped(s)
        p = g.get('played')
        if p is not None and p.get('at') != prev_played_at and p.get('at') is not None:
            prev_played_at = p['at']
            handover_jump = max(handover_jump, float(np.max(np.abs(p['cmd'] - np.degrees(g['sense'][edge.Q])))))
        track.append(d.xpos[hands_id].ravel().tolist())
        if g['t'] - told_at >= 1.0 / look.RATE_HZ:
            told_at = g['t']
            rows.append(scenarios._row(g, w))
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
    return {
        'ok': not reasons, 'reasons': reasons, 'stunt_far': far_bed,
        'handover_deg': handover_jump, 'jerk': _hand_jerk_from_track(track),
        'bed_bore_s': round(bore_bed, 2), 'rolled_deg': round(rolled, 1),
    }


def _eval_crouch_trial(s, combo, wobble):
    """Run one crouch-down -> crouch-up combinatorial trial against mocap crouch stunts."""
    from machine.process import engrams, worker
    from tools.sim import look, scenarios

    g = s['g']
    w = g['world']
    m, d = w.model, w.data
    _apply_wobble(w, wobble)
    for name in combo:
        engrams.swap(g, name, s=s)
    down_slot = engrams.CATALOG[combo[0]].slot
    up_slot = engrams.CATALOG[combo[1]].slot
    hands_id = [m.body('left_hand').id, m.body('right_hand').id]
    y0 = float(d.qpos[1])
    y_min = y0
    rows, track, told_at = [], [], -math.inf
    t0 = float(g['t'])

    worker.apply(s, {'say': down_slot})
    while g['t'] < t0 + 3.6 and s['fallen_at'] is None and s['epoch'] == 0:
        worker.stepped(s)
        y_min = min(y_min, float(d.qpos[1]))
        track.append(d.xpos[hands_id].ravel().tolist())
        if g['t'] - told_at >= 1.0 / look.RATE_HZ:
            told_at = g['t']
            rows.append(scenarios._row(g, w))
    t_mid = float(g['t'])
    if s['fallen_at'] is None and s['epoch'] == 0:
        worker.apply(s, {'say': up_slot})
        while g['t'] < t_mid + 4.0 and s['fallen_at'] is None and s['epoch'] == 0:
            worker.stepped(s)
            track.append(d.xpos[hands_id].ravel().tolist())
            if g['t'] - told_at >= 1.0 / look.RATE_HZ:
                told_at = g['t']
                rows.append(scenarios._row(g, w))

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
    return {
        'ok': not reasons, 'reasons': reasons, 'stunt_far': far_crouch,
        'handover_deg': 0.0, 'jerk': _hand_jerk_from_track(track), 'drop_m': round(drop, 3),
    }


def _eval_turn_trial(s, combo, wobble):
    """Run one turn-left combinatorial trial against `left_turn` mocap take."""
    from machine.process import engrams, worker
    from tools.sim import look, scenarios

    g = s['g']
    w = g['world']
    m, d = w.model, w.data
    _apply_wobble(w, wobble)
    for name in combo:
        engrams.swap(g, name, s=s)
    slot = engrams.CATALOG[combo[0]].slot
    hands_id = [m.body('left_hand').id, m.body('right_hand').id]
    yaw0 = float(g['heading'])
    t0 = float(g['t'])
    rows, track, told_at = [], [], -math.inf

    worker.apply(s, {'say': slot})
    while g['t'] < t0 + 2.5 and s['fallen_at'] is None and s['epoch'] == 0:
        worker.stepped(s)
        track.append(d.xpos[hands_id].ravel().tolist())
        if g['t'] - told_at >= 1.0 / look.RATE_HZ:
            told_at = g['t']
            rows.append(scenarios._row(g, w))
    dyaw = math.degrees(float(g['heading']) - yaw0)
    reasons = []
    if s['fallen_at'] is not None or s['epoch'] != 0:
        reasons.append('fell')
    if abs(dyaw) < 15.0:
        reasons.append('insufficient_turn')
    far_turn = _stunt_far_from_rows(rows, 'left_turn', t0, float(g['t']))
    return {
        'ok': not reasons, 'reasons': reasons, 'stunt_far': far_turn,
        'handover_deg': 0.0, 'jerk': _hand_jerk_from_track(track), 'dyaw_deg': round(dyaw, 1),
    }


def _run_shard(args):
    """Worker entrypoint: capture pre-dock checkpoint once, then run assigned combos x wobbles."""
    scene_name, combos, wobbles = args
    from machine.process import checkpoint, engrams
    from machine.world import physics

    prev_room, prev_sight = physics.ROOM, physics.SIGHT
    s, snap = _setup_checkpoint(scene_name)
    out = []
    try:
        task = SCENES[scene_name]['task']
        for combo in combos:
            for wobble in wobbles:
                engrams.unload_all(s['g'], s=s)
                checkpoint.restore(s, snap)
                if task == 'chair':
                    res = _eval_chair_trial(s, scene_name, combo, wobble)
                elif task == 'bed':
                    res = _eval_bed_trial(s, combo, wobble)
                elif task == 'crouch':
                    res = _eval_crouch_trial(s, combo, wobble)
                elif task == 'turn':
                    res = _eval_turn_trial(s, combo, wobble)
                else:
                    raise ValueError('unsupported task %r' % (task,))
                out.append({'scene': scene_name, 'combo': list(combo), 'wobble': wobble, **res})
    finally:
        engrams.unload_all(s['g'], s=s)
        physics.ROOM, physics.SIGHT = prev_room, prev_sight
    return out


def combinations_for(scene_name, quick=False):
    """Build combinatorial candidate engram tuples for `scene_name`."""
    from machine.process import engrams
    task = SCENES[scene_name]['task']
    slots = engrams.suites()[task]
    lists = [vals[:2] if quick else vals for vals in slots.values()]
    return list(itertools.product(*lists))


def _wobble_samples(samples, seed):
    """Generate deterministic symmetric yaw perturbations centered on 0.0 rad."""
    if samples <= 1:
        return (0.0,)
    if samples == 2:
        return (0.0, 0.025)
    if samples == 3:
        return (-0.03, 0.0, 0.03)
    rng = random.Random(seed)
    return tuple([0.0] + [round(rng.uniform(-0.04, 0.04), 4) for _ in range(samples - 1)])


def run_sweep(scenes=None, quick=False, samples=3, seed=7, workers=None, chunk_size=2):
    """Run parallel Monte Carlo engram combinatorics across `scenes` and aggregate summary rows."""
    scenes = tuple(scenes or SCENES)
    wobbles = _wobble_samples(samples, seed)
    tasks = []
    for scene in scenes:
        combos = combinations_for(scene, quick=quick)
        for i in range(0, len(combos), chunk_size):
            tasks.append((scene, combos[i:i + chunk_size], wobbles))
    if not tasks:
        return {'trials': [], 'summary': []}
    n_workers = workers or max(1, min(len(tasks), (os.cpu_count() or 4) - 2))
    trials = []
    if n_workers <= 1 or len(tasks) == 1:
        for task in tasks:
            trials.extend(_run_shard(task))
    else:
        with concurrent.futures.ProcessPoolExecutor(max_workers=n_workers, initializer=_worker_init) as pool:
            for shard_out in pool.map(_run_shard, tasks):
                trials.extend(shard_out)

    grouped = {}
    for row in trials:
        key = (row['scene'], tuple(row['combo']))
        grouped.setdefault(key, []).append(row)

    summary = []
    for (scene, combo), items in grouped.items():
        n = len(items)
        passed = sum(1 for x in items if x['ok'])
        fars = [x['stunt_far'] for x in items if not math.isnan(x['stunt_far'])]
        reasons = {}
        for x in items:
            for r in x['reasons']:
                reasons[r] = reasons.get(r, 0) + 1
        summary.append({
            'scene': scene,
            'combo': list(combo),
            'passed': passed,
            'total': n,
            'pass_rate': passed / n,
            'mean_stunt_far': round(sum(fars) / len(fars), 3) if fars else math.nan,
            'max_handover_deg': round(max(x['handover_deg'] for x in items), 3),
            'max_jerk': round(max(x['jerk'] for x in items), 1),
            'reasons': reasons,
        })
    summary.sort(key=lambda r: (r['scene'], -r['pass_rate'],
                                999.0 if math.isnan(r['mean_stunt_far']) else r['mean_stunt_far'],
                                r['max_jerk']))
    return {'trials': trials, 'summary': summary}


def _print_list():
    """Print prepared candidate engram catalogs and combinatorial scenario counts."""
    from machine.process import engrams
    print('PREPARED MOTOR ENGRAMS (stunts/ mapping):')
    for name, eng in engrams.catalog().items():
        print('  %-24s slot=%-16s kind=%-7s stunt=%-25s %s' % (
            name, eng.slot, eng.kind, eng.stunt_ref or '-', eng.description))
    print('\nSCENARIO COMBINATORICS:')
    for sc in SCENES:
        print('  %-18s task=%-7s stunt=%-25s combos=%d' % (
            sc, SCENES[sc]['task'], SCENES[sc]['stunt'], len(combinations_for(sc))))


def main(argv=None):
    """CLI entrypoint for listing engrams or executing parallel Monte Carlo sweeps."""
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    p.add_argument('scenes', nargs='*', help='scenes to sweep (all if omitted)')
    p.add_argument('--list', action='store_true', help='print prepared engram catalog and exit')
    p.add_argument('--quick', action='store_true', help='run compact candidate subset')
    p.add_argument('--samples', type=int, default=3, help='wobble perturbations per combination')
    p.add_argument('--seed', type=int, default=7, help='PRNG seed for perturbation draws')
    p.add_argument('--jobs', type=int, default=0, help='parallel worker processes (0 = auto parallel)')
    p.add_argument('--chunk', type=int, default=2, help='combinations per checkpointed worker shard')
    p.add_argument('--json', help='write full trial and summary JSON to file')
    args = p.parse_args(argv)
    if args.list:
        _print_list()
        return 0
    selected = args.scenes or list(SCENES)
    bad = [s for s in selected if s not in SCENES]
    if bad:
        p.error('unknown scenes: %s (choices: %s)' % (', '.join(bad), ', '.join(SCENES)))
    got = run_sweep(scenes=selected, quick=args.quick, samples=args.samples, seed=args.seed,
                    workers=args.jobs or None, chunk_size=max(1, args.chunk))
    print('%-18s %-44s %6s %8s %8s %9s %s' % (
        'scene', 'engram_combo', 'pass', 'stunt_far', 'jump_deg', 'max_jerk', 'failures'))
    for r in got['summary']:
        far_s = 'nan' if math.isnan(r['mean_stunt_far']) else '%.2f' % r['mean_stunt_far']
        fail_s = ','.join('%s:%d' % kv for kv in sorted(r['reasons'].items())) or '-'
        print('%-18s %-44s %2d/%-3d %8s %8.3f %9.0f %s' % (
            r['scene'], ' + '.join(r['combo']), r['passed'], r['total'], far_s,
            r['max_handover_deg'], r['max_jerk'], fail_s))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(got, f, indent=2)
    return 0


if __name__ == '__main__':
    sys.exit(main())
