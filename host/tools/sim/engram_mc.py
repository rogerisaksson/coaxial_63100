#!/usr/bin/env python3
"""Parallel Monte Carlo combinatorics over hot-swappable motor engrams from pre-dock checkpoints.

    python tools/sim/engram_mc.py --list
    python tools/sim/engram_mc.py crouching_down --engrams crouch_down_mocap,crouch_down_calm --jobs 16
    python tools/sim/engram_mc.py chair_dock bed_dock crouch_dock turn_dock --samples 3 --jobs 24
    python tools/sim/engram_mc.py --all-stunts --samples 1 --jobs 24

Each scenario captures a bit-exact simulation checkpoint (`machine.process.checkpoint`) right before
docking or stunt onset, then fans engram bundles across worker processes (`ProcessPoolExecutor`),
restoring the pre-dock checkpoint per trial and grading world stability plus mocap similarity.
"""
import argparse
import concurrent.futures
import itertools
import json
import math
import multiprocessing
import os
import random
import sys

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OPENBLAS_CORETYPE', 'Nehalem')

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.dev import arithmetic
from tools.sim.engram_scenes import (SCENES, apply_wobble, eval_bed, eval_chair, eval_crouch,
                                     eval_turn, hand_jerk, setup_checkpoint, worker_init)

arithmetic.hold()


def _setup_stunt_checkpoint(stunt_key):
    """Place `stunt_key`'s scene, advance up to its final maneuver command, and capture checkpoint."""
    from machine.process import checkpoint, worker
    from machine.world import physics
    from tools.sim import scenarios, staged, stunts

    _take, scene, plan, _start, _sig = staged.STUNTS[stunt_key]
    physics.ROOM, physics.SIGHT = float(scene == 'lodge'), 0.0
    s = worker.session()
    scenarios.placed(s, scene)
    ordered = sorted(((float(at), stunts.told(c)) for at, c in plan), key=lambda x: x[0])
    split_t = max(0.0, ordered[-1][0] - 0.01) if ordered else 0.4
    pre = [(at, cmd) for at, cmd in ordered if at < split_t]
    idx = 0
    while s['g']['t'] < split_t - 1e-9 and s['fallen_at'] is None:
        while idx < len(pre) and s['g']['t'] >= pre[idx][0]:
            worker.apply(s, pre[idx][1])
            idx += 1
        worker.stepped(s)
    return s, checkpoint.capture(s), split_t


def _eval_stunt_trial(s, stunt_key, combo, wobble, split_t):
    """Run `stunt_key` from pre-maneuver checkpoint with `combo` hot-swapped live."""
    import numpy as np
    from machine.process import engrams, worker
    from machine.senses import edge
    from tools.sim import look, scenarios, staged, stunt_similarity, stunts

    g = s['g']
    w = g['world']
    m, d = w.model, w.data
    apply_wobble(w, wobble)
    swap_jump = 0.0
    for ename in combo:
        if ename:
            info = engrams.swap(g, ename, s=s)
            swap_jump = max(swap_jump, info['pose_jump_deg'])

    _take, _scene, plan, start, sig = staged.STUNTS[stunt_key]
    ref = stunts.reference()[stunt_key]
    end_t = (staged.PLAYED_S if start is None else start) + ref['seconds'] + scenarios.STILL_S
    active_names = tuple(x for x in combo if x)
    ordered = sorted(((float(at), stunt_similarity._patched_command(c, active_names, g))
                      for at, c in plan if float(at) >= split_t - 1e-9), key=lambda x: x[0])
    told_names = [c['say'] for _at, c in plan if 'say' in c]
    refused = sorted(set(told_names) - {e[0] for e in g['gym']})
    if refused:
        return {'ok': False, 'reasons': ['no_text:' + refused[0]], 'stunt_far': math.nan,
                'score': 0.0, 'handover_deg': swap_jump, 'jerk': 0.0}

    head_id = m.body('head').id
    hands_id = [m.body('left_hand').id, m.body('right_hand').id]
    k, began, prev_at, fell, hit = 0, None, None, None, None
    rows, track, told_at, scale_min = [], [], -math.inf, 1.0
    while g['t'] < end_t and s['epoch'] == 0:
        while k < len(ordered) and g['t'] >= ordered[k][0]:
            worker.apply(s, ordered[k][1])
            k += 1
        worker.stepped(s)
        p = g.get('played')
        if p is not None and p.get('at') is not None and p.get('at') != prev_at:
            prev_at = p['at']
            if began is None:
                began = p['at']
            swap_jump = max(swap_jump, float(np.max(np.abs(p['cmd'] - np.degrees(g['sense'][edge.Q])))))
        if g['n'] % g['every'] == 1 and 'got' in g:
            scale_min = min(scale_min, float(g['got'].get('scale', 1.0)))
        if s['fallen_at'] is not None and fell is None:
            fell = float(g['t'])
        if fell is not None and hit is None and float(d.xpos[head_id][1]) < 0.25:
            hit = float(np.linalg.norm(d.cvel[head_id][3:]))
        track.append(d.xpos[hands_id].ravel().tolist())
        if g['t'] - told_at >= 1.0 / look.RATE_HZ:
            told_at = g['t']
            rows.append(scenarios._row(g, w))

    cut = began or 0.0
    if start is not None and cut >= float(scenarios.stunt_window(stunt_key)[0] or 0.0):
        rows = [r for r in rows if float(r['t']) <= cut]
    moved = stunts.moved(rows, *scenarios.stunt_window(stunt_key, began or math.inf))
    far_sig = float(stunts.far(moved, ref, sig))
    far_all = float(stunts.far(moved, ref, stunt_similarity._all_measures()))
    stood = fell is None
    jerk_99 = hand_jerk(track)
    score = stunt_similarity.score_stunt(far_sig, far_all, stood, hit, scale_min, jerk_99, None)
    head_ok = hit is None or hit < scenarios.FALL_M_S
    met = bool(stunts.met(moved, ref, sig) and head_ok)
    reasons = stunt_similarity.diagnose(moved, ref, sig, stood, hit, scale_min, jerk_99, None)
    return {'ok': met, 'reasons': reasons, 'stunt_far': round(far_sig, 3),
            'score': score, 'handover_deg': round(swap_jump, 3), 'jerk': round(jerk_99, 1)}


def _dispatch_trial(s, scene_name, task, combo, wobble, split_t):
    """Run trial evaluation for `task` on session `s`."""
    if task == 'chair':
        return eval_chair(s, scene_name, combo, wobble)
    if task == 'bed':
        return eval_bed(s, combo, wobble)
    if task == 'crouch':
        return eval_crouch(s, combo, wobble)
    if task == 'turn':
        return eval_turn(s, combo, wobble)
    return _eval_stunt_trial(s, scene_name, combo, wobble, split_t)


def _run_shard(args):
    """Worker entrypoint: run combos x wobbles from pre-dock checkpoint (or fresh camera if `sight`)."""
    scene_name, combos, wobbles, extras, spec_list, sight = args
    from machine.process import checkpoint, engrams
    from machine.world import physics

    if extras:
        engrams.register(extras)
    if spec_list:
        engrams.load_specs(spec_list)
    prev_room, prev_camera, prev_sight = physics.ROOM, physics.CAMERA, physics.SIGHT
    is_scene = scene_name in SCENES
    task = SCENES[scene_name]['task'] if is_scene else 'stunt'
    out = []
    if sight and is_scene:
        try:
            for combo in combos:
                for wobble in wobbles:
                    s, _ = setup_checkpoint(scene_name, sight=True)
                    try:
                        res = _dispatch_trial(s, scene_name, task, combo, wobble, 0.0)
                        out.append({'scene': scene_name, 'combo': [c for c in combo if c] or ['stock'],
                                    'wobble': wobble, **res})
                    finally:
                        engrams.unload_all(s['g'], s=s)
                        if s['g'].get('keep') is not None:
                            s['g']['keep'].close()
        finally:
            physics.ROOM, physics.CAMERA, physics.SIGHT = prev_room, prev_camera, prev_sight
        return out
    if is_scene:
        s, snap = setup_checkpoint(scene_name, sight=False)
        split_t = 0.0
    else:
        s, snap, split_t = _setup_stunt_checkpoint(scene_name)
    try:
        for combo in combos:
            for wobble in wobbles:
                engrams.unload_all(s['g'], s=s)
                checkpoint.restore(s, snap)
                res = _dispatch_trial(s, scene_name, task, combo, wobble, split_t)
                out.append({'scene': scene_name, 'combo': [c for c in combo if c] or ['stock'],
                            'wobble': wobble, **res})
    finally:
        engrams.unload_all(s['g'], s=s)
        physics.ROOM, physics.CAMERA, physics.SIGHT = prev_room, prev_camera, prev_sight
    return out


def _parse_engram_groups(engrams_arg):
    """Parse `--engrams` into Cartesian combo tuples (`A,B;C,D` -> `(A,C)..`, `A,B` -> `(A,), (B,)`)."""
    if not engrams_arg:
        return None
    if ';' in engrams_arg:
        groups = [[x.strip() for x in grp.split(',') if x.strip()] for grp in engrams_arg.split(';') if grp.strip()]
        return list(itertools.product(*groups))
    return [(x.strip(),) for x in engrams_arg.split(',') if x.strip()]


def combinations_for(scene_name, quick=False, explicit=None, explore=0, seed=7):
    """Build combinatorial candidate engram tuples and optional parametric mutations for `scene_name`."""
    if explicit is not None:
        return explicit, []
    from machine.process import engrams
    if scene_name in SCENES:
        task = SCENES[scene_name]['task']
        slots = engrams.suites()[task]
        lists = [list(vals[:2] if quick else vals) for vals in slots.values()]
        extras, extras_combos = [], []
        if explore > 0:
            rng = random.Random(seed + sum(ord(c) for c in scene_name))
            slot_names = list(slots)
            for k in range(explore):
                combo_var = []
                for sname, vals in zip(slot_names, lists):
                    eng = engrams.variant(rng.choice(vals), '%s_var%d_%s' % (scene_name, k, sname), rng)
                    extras.append(eng)
                    combo_var.append(eng.name)
                engrams.register(extras[-len(slot_names):])
                extras_combos.append(tuple(combo_var))
        return list(itertools.product(*lists)) + extras_combos, extras
    matched = [(eng.name,) for eng in engrams.catalog().values() if eng.stunt_ref == scene_name]
    bundle = engrams.stunt_bundle(scene_name)
    base = [()] + matched + ([bundle] if bundle and bundle not in matched else [])
    return (base[:3] if quick else base), []


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


def run_sweep(scenes=None, quick=False, samples=3, seed=7, workers=None, chunk_size=2,
              engrams_arg=None, spec_list=None, explore=0, sight=False):
    """Run parallel Monte Carlo engram combinatorics across `scenes` or stunts."""
    from machine.process import engrams
    if spec_list:
        engrams.load_specs(spec_list)
    explicit = _parse_engram_groups(engrams_arg)
    scenes = tuple(scenes or SCENES)
    wobbles = _wobble_samples(samples, seed)
    tasks = []
    for scene in scenes:
        combos, extras = combinations_for(scene, quick=quick, explicit=explicit, explore=explore, seed=seed)
        for i in range(0, len(combos), chunk_size):
            tasks.append((scene, combos[i:i + chunk_size], wobbles, extras, spec_list, bool(sight)))
    if not tasks:
        return {'trials': [], 'summary': []}
    n_workers = workers or max(1, min(len(tasks), (os.cpu_count() or 4) - 2))
    trials = []
    if n_workers <= 1 or len(tasks) == 1:
        for task in tasks:
            trials.extend(_run_shard(task))
    else:
        ctx = multiprocessing.get_context('spawn')
        with concurrent.futures.ProcessPoolExecutor(max_workers=n_workers, mp_context=ctx,
                                                    initializer=worker_init) as pool:
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
        scores = [x['score'] for x in items if 'score' in x and not math.isnan(x['score'])]
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
            'mean_score': round(sum(scores) / len(scores), 2) if scores else math.nan,
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
            sc, SCENES[sc]['task'], SCENES[sc]['stunt'], len(combinations_for(sc)[0])))


def main(argv=None):
    """CLI entrypoint for listing engrams or executing parallel Monte Carlo sweeps."""
    from tools.sim import staged
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    p.add_argument('scenes', nargs='*', help='scenes or stunt keys to sweep (default: pre-dock SCENES)')
    p.add_argument('--all-stunts', action='store_true', help='sweep all stunts in parallel')
    p.add_argument('--engrams', help='engram list (`A,B`) or `;`-separated slot product (`A,B;C,D`)')
    p.add_argument('--spec', help='JSON file defining candidate engram specs')
    p.add_argument('--list', action='store_true', help='print prepared engram catalog and exit')
    p.add_argument('--quick', action='store_true', help='run compact candidate subset')
    p.add_argument('--samples', type=int, default=3, help='wobble perturbations per combination')
    p.add_argument('--seed', type=int, default=7, help='PRNG seed for perturbation draws')
    p.add_argument('--explore', type=int, default=0, help='extra parametric variant combinations per scene')
    p.add_argument('--sight', action='store_true', help='evaluate with full stereo vision (SIGHT=1)')
    p.add_argument('--jobs', type=int, default=0, help='parallel worker processes (0 = auto parallel)')
    p.add_argument('--chunk', type=int, default=2, help='combinations per checkpointed worker shard')
    p.add_argument('--json', help='write full trial and summary JSON to file')
    args = p.parse_args(argv)
    if args.list:
        _print_list()
        return 0
    spec_list = None
    if args.spec:
        with open(args.spec, encoding='utf-8') as f:
            raw = json.load(f)
        spec_list = raw.get('engrams', raw) if isinstance(raw, dict) else raw
    valid = set(SCENES) | set(staged.STUNTS)
    selected = list(staged.STUNTS) if args.all_stunts else (args.scenes or list(SCENES))
    bad = [s for s in selected if s not in valid]
    if bad:
        p.error('unknown scenes/stunts: %s' % ', '.join(bad))
    got = run_sweep(scenes=selected, quick=args.quick, samples=args.samples, seed=args.seed,
                    workers=args.jobs or None, chunk_size=max(1, args.chunk),
                    engrams_arg=args.engrams, spec_list=spec_list,
                    explore=max(0, args.explore), sight=args.sight)
    print('%-20s %-40s %5s %6s %7s %7s %8s %s' % (
        'scene', 'engram_combo', 'pass', 'score', 'far', 'jump', 'jerk', 'failures'))
    for r in got['summary']:
        far_s = 'nan' if math.isnan(r['mean_stunt_far']) else '%.2f' % r['mean_stunt_far']
        sc_s = '-' if math.isnan(r['mean_score']) else '%6.1f' % r['mean_score']
        fail_s = ','.join('%s:%d' % kv for kv in sorted(r['reasons'].items())) or '-'
        print('%-20s %-40s %2d/%-2d %6s %7s %7.3f %8.0f %s' % (
            r['scene'], ' + '.join(r['combo']), r['passed'], r['total'], sc_s, far_s,
            r['max_handover_deg'], r['max_jerk'], fail_s))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(got, f, indent=2)
    return 0


if __name__ == '__main__':
    sys.exit(main())
