#!/usr/bin/env python3
"""Anatomical and kinematic similarity analyzer comparing robot trajectories to mocap takes.

    python tools/sim/stunt_similarity.py --jobs 24
    python tools/sim/stunt_similarity.py crouching_down --engrams crouch_down_calm
    python tools/sim/stunt_similarity.py --bundle best --against baseline.json --jobs 24

Scores each stunt (0..100) from dynamic-similarity distance against `stunts/` (`far_sig`, `far_all`),
standing survival, actuator torque headroom (`scale_min`), and hand/torso smoothness, with
per-measure anatomical bottleneck diagnostics and parallel A/B diffs.
"""
import argparse
import concurrent.futures
import json
import math
import multiprocessing
import os
import sys

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OPENBLAS_CORETYPE', 'Nehalem')

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

ALL_MEASURES = None


def _all_measures():
    """Return tuple of all dynamic-similarity measure names in `stunts.MOVES`."""
    global ALL_MEASURES
    if ALL_MEASURES is None:
        from tools.sim import stunts
        ALL_MEASURES = tuple(m[0] for m in stunts.MOVES)
    return ALL_MEASURES


def score_stunt(far_sig, far_all, stood, head_hit, scale_min, jerk_99, refused):
    """Compute bounded 0..100 anatomical similarity score for one evaluated stunt."""
    from tools.sim import scenarios
    if refused or math.isnan(far_sig):
        return 0.0
    head_ok = head_hit is None or float(head_hit) < scenarios.FALL_M_S
    sig_pts = 50.0 * math.exp(-0.35 * max(0.0, float(far_sig)))
    body_pts = 25.0 * math.exp(-0.08 * max(0.0, float(far_all)))
    if stood and head_ok:
        limit_pts = 15.0 * min(1.0, max(0.0, float(scale_min) / 0.75))
    else:
        limit_pts = 5.0 if head_ok else 0.0
    j_val = 0.0 if math.isnan(jerk_99) else float(jerk_99)
    smooth_pts = 10.0 / (1.0 + max(0.0, (j_val - 2500.0) / 8000.0))
    return round(sig_pts + body_pts + limit_pts + smooth_pts, 2)


def diagnose(moved, ref, signature, stood, head_hit, scale_min, jerk_99, refused):
    """Identify sorted anatomical and control bottlenecks explaining gaps to the mocap take."""
    from tools.sim import scenarios, stunts
    tags = []
    if refused:
        return ['no_text:' + str(refused)]
    if not stood:
        if head_hit is not None and float(head_hit) >= scenarios.FALL_M_S:
            tags.append('fell(%.1fm/s)' % float(head_hit))
        else:
            tags.append('fell')
    if float(scale_min) < 0.75:
        tags.append('torque_clip(%.2f)' % float(scale_min))
    if not math.isnan(jerk_99) and float(jerk_99) > 10000.0:
        tags.append('high_jerk(%.0f)' % float(jerk_99))
    gaps = []
    for name, unit, side in stunts.MOVES:
        if name not in signature + stunts.SAFE:
            continue
        a = moved.get(name, math.nan)
        b = ref[name]
        tol = stunts.WITHIN[unit]
        if math.isnan(a):
            gaps.append((9.9, name + ':nan'))
            continue
        err = (b - a) / tol if side > 0 else (a - b) / tol
        over = (max(0.0, b - a) / tol - 1.0) if side > 0 else (abs(a - b) / tol - 1.0)
        if over > 0.0:
            gaps.append((over, '%s(%+.1fw)' % (name, err)))
    gaps.sort(reverse=True)
    return tags + [label for _ov, label in gaps[:3]]


def _patched_command(cmd, engram_names, g):
    """Dispatch `cmd` using an active hot-swapped engram entry when matching `take` or `pattern`."""
    from machine.process import engrams
    from tools.sim import stunts
    by_slot = {}
    for ename in engram_names:
        eng = engrams.get(ename)
        by_slot[eng.slot] = eng
    take_key = cmd.get('take')
    if take_key and take_key in by_slot:
        entry = next((e for e in g['gym'] if e[0] == take_key), None)
        if entry is not None:
            return {'entries': [entry]}
    pat_key = cmd.get('pattern')
    if pat_key and pat_key in by_slot:
        entry = next((e for e in g['gym'] if e[0] == pat_key), None)
        if entry is not None:
            return {'entries': [entry]}
    return stunts.told(cmd)


def evaluate_stunt(stunt_key, engram_names=(), spec_list=None, wobble=0.0):
    """Run `stunt_key` in MuJoCo with `engram_names` swapped in and return similarity metrics."""
    import numpy as np
    from machine.process import engrams, worker
    from machine.world import physics
    from tools.sim import look, scenarios, staged, stunts

    if spec_list:
        engrams.load_specs(spec_list)
    _take, scene, plan, start, signature = staged.STUNTS[stunt_key]
    ref = stunts.reference()[stunt_key]
    seconds = (staged.PLAYED_S if start is None else start) + ref['seconds'] + scenarios.STILL_S

    prev_room, prev_sight = physics.ROOM, physics.SIGHT
    physics.ROOM, physics.SIGHT = float(scene == 'lodge'), 0.0
    try:
        s = worker.session()
        g = s['g']
        for ename in engram_names:
            engrams.swap(g, ename, s=s)
        told_names = [c['say'] for _at, c in plan if 'say' in c]
        refused = sorted(set(told_names) - {e[0] for e in g['gym']})
        if refused:
            return {
                'stunt': stunt_key, 'engrams': list(engram_names), 'refused': refused[0],
                'met': False, 'stood': False, 'far_sig': math.nan, 'far_all': math.nan,
                'scale_min': 1.0, 'jerk_99': math.nan, 'score': 0.0,
                'bottlenecks': ['no_text:' + refused[0]], 'moved': {},
            }
        scenarios.placed(s, scene)
        w = g['world']
        m, d = w.model, w.data
        if wobble:
            q = d.qpos[3:7].copy()
            cw, sw = math.cos(0.5 * wobble), math.sin(0.5 * wobble)
            d.qpos[3:7] = (q[0] * cw - q[2] * sw, q[1] * cw + q[3] * sw,
                           q[2] * cw + q[0] * sw, q[3] * cw - q[1] * sw)
            w._mj.mj_forward(m, d)
        head_id = m.body('head').id
        hands_id = [m.body('left_hand').id, m.body('right_hand').id]
        ordered = sorted(((at, _patched_command(c, engram_names, g)) for at, c in plan), key=lambda x: x[0])
        k, began, fell, hit = 0, None, None, None
        rows, track, told_at = [], [], -math.inf
        scale_min = 1.0

        while g['t'] < seconds and s['epoch'] == 0:
            while k < len(ordered) and g['t'] >= ordered[k][0]:
                worker.apply(s, ordered[k][1])
                k += 1
            worker.stepped(s)
            if began is None and g.get('played') is not None:
                began = g['played'].get('at')
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
        far_sig = float(stunts.far(moved, ref, signature))
        far_all = float(stunts.far(moved, ref, _all_measures()))
        head_ok = hit is None or hit < scenarios.FALL_M_S
        met = bool(stunts.met(moved, ref, signature) and head_ok)
        stood = fell is None
        jerk_99 = 0.0
        if len(track) >= 100:
            arr = np.asarray(track, float)
            w_len = 20
            for sl in (slice(0, 3), slice(3, 6)):
                p = arr[:, sl]
                sm = np.cumsum(p, axis=0)[w_len:] / w_len - np.cumsum(p, axis=0)[:-w_len] / w_len
                j = np.linalg.norm(np.diff(sm, n=3, axis=0), axis=1) / (0.001 ** 3)
                jerk_99 = max(jerk_99, float(np.percentile(j, 99)))
        score = score_stunt(far_sig, far_all, stood, hit, scale_min, jerk_99, None)
        bottlenecks = diagnose(moved, ref, signature, stood, hit, scale_min, jerk_99, None)
        return {
            'stunt': stunt_key, 'engrams': list(engram_names), 'refused': None,
            'met': met, 'stood': stood, 'head_hit': hit,
            'far_sig': round(far_sig, 3), 'far_all': round(far_all, 3),
            'scale_min': round(scale_min, 3), 'jerk_99': round(jerk_99, 1),
            'score': score, 'bottlenecks': bottlenecks,
            'moved': {m_k: round(float(m_v), 4) for m_k, m_v in moved.items()},
        }
    finally:
        physics.ROOM, physics.SIGHT = prev_room, prev_sight


def _worker_eval(args):
    """Run one `(stunt_key, engram_names, spec_list, wobble)` task inside worker pool."""
    return evaluate_stunt(*args)


def analyze(stunt_keys=None, engram_map=None, spec_list=None, wobble=0.0, workers=None):
    """Evaluate all requested stunts in parallel and return sorted per-stunt metrics and totals."""
    from tools.sim import staged
    keys = list(stunt_keys or staged.STUNTS)
    emap = engram_map or {}
    tasks = [(k, tuple(emap.get(k, ())), spec_list, wobble) for k in keys]
    n_workers = workers or max(1, min(len(tasks), (os.cpu_count() or 4) - 2))
    if n_workers <= 1 or len(tasks) == 1:
        rows = [_worker_eval(t) for t in tasks]
    else:
        from tools.sim.engram_scenes import worker_init
        ctx = multiprocessing.get_context('spawn')
        with concurrent.futures.ProcessPoolExecutor(max_workers=n_workers, mp_context=ctx,
                                                    initializer=worker_init) as pool:
            rows = list(pool.map(_worker_eval, tasks))
    rows.sort(key=lambda r: (-r['score'], r['stunt']))
    executable = [r for r in rows if not r['refused']]
    mean_score = round(sum(r['score'] for r in rows) / len(rows), 2) if rows else 0.0
    mean_exec = round(sum(r['score'] for r in executable) / len(executable), 2) if executable else 0.0
    mean_far = round(sum(r['far_sig'] for r in executable) / len(executable), 3) if executable else math.nan
    return {
        'rows': rows,
        'summary': {
            'total': len(rows),
            'executable': len(executable),
            'met': sum(1 for r in rows if r['met']),
            'stood': sum(1 for r in executable if r['stood']),
            'mean_score': mean_score,
            'mean_exec_score': mean_exec,
            'mean_far_sig': mean_far,
        },
    }


def _load_spec_list(spec_path):
    """Read raw engram list from JSON file `spec_path` for broadcast to worker processes."""
    if not spec_path:
        return None
    with open(spec_path, encoding='utf-8') as f:
        raw = json.load(f)
    return raw.get('engrams', raw) if isinstance(raw, dict) else raw


def main(argv=None):
    """CLI entrypoint for anatomical and kinematic stunt similarity analysis."""
    from machine.process import engrams
    from tools.sim import staged
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    p.add_argument('stunts', nargs='*', help='stunts to analyze (all when omitted)')
    p.add_argument('--engrams', help='comma-separated engram names applied to listed stunts')
    p.add_argument('--bundle', choices=('none', 'best'), default='none',
                   help='apply verified per-stunt catalog bundles automatically')
    p.add_argument('--spec', help='JSON file with candidate engram specs and optional bundles dict')
    p.add_argument('--wobble', type=float, default=0.0, help='yaw perturbation in radians')
    p.add_argument('--jobs', type=int, default=0, help='parallel worker processes (0 = auto)')
    p.add_argument('--save', help='save full JSON analysis to path')
    p.add_argument('--against', help='compare against baseline JSON file from --save')
    args = p.parse_args(argv)

    selected = args.stunts or list(staged.STUNTS)
    bad = [k for k in selected if k not in staged.STUNTS]
    if bad:
        p.error('unknown stunts: %s' % ', '.join(bad))
    spec_list = _load_spec_list(args.spec)
    emap = {}
    if args.bundle == 'best':
        for k in selected:
            emap[k] = engrams.stunt_bundle(k)
    if args.spec:
        with open(args.spec, encoding='utf-8') as f:
            raw = json.load(f)
        if isinstance(raw, dict) and isinstance(raw.get('bundles'), dict):
            for k, bnames in raw['bundles'].items():
                if k in selected:
                    emap[k] = tuple(bnames)
    if args.engrams:
        explicit = tuple(x.strip() for x in args.engrams.split(',') if x.strip())
        for k in selected:
            emap[k] = explicit

    got = analyze(selected, emap, spec_list=spec_list, wobble=args.wobble, workers=args.jobs or None)
    print('%-24s %6s %4s %7s %7s %5s %5s %8s %s' % (
        'stunt', 'score', 'met', 'far_sig', 'far_all', 'stood', 'scale', 'jerk_99', 'bottlenecks'))
    for r in got['rows']:
        fs = 'nan' if math.isnan(r['far_sig']) else '%7.2f' % r['far_sig']
        fa = 'nan' if math.isnan(r['far_all']) else '%7.2f' % r['far_all']
        jk = 'nan' if math.isnan(r['jerk_99']) else '%8.0f' % r['jerk_99']
        print('%-24s %6.2f %-4s %7s %7s %-5s %5.2f %8s %s' % (
            r['stunt'], r['score'], 'YES' if r['met'] else '-', fs, fa,
            'YES' if r['stood'] else 'no', r['scale_min'], jk, ', '.join(r['bottlenecks']) or '-'))
    sm = got['summary']
    print('\nsummary: met=%d/%d stood=%d/%d mean_score=%.2f mean_exec_score=%.2f mean_far_sig=%.3f' % (
        sm['met'], sm['total'], sm['stood'], sm['executable'],
        sm['mean_score'], sm['mean_exec_score'], sm['mean_far_sig']))

    if args.save:
        with open(args.save, 'w', encoding='utf-8') as f:
            json.dump(got, f, indent=2)
    if args.against:
        with open(args.against, encoding='utf-8') as f:
            was = json.load(f)
        base_by = {r['stunt']: r for r in was['rows']}
        print('\ndelta vs %s (mean_score %+.2f, mean_far_sig %+.3f):' % (
            args.against, sm['mean_score'] - was['summary']['mean_score'],
            sm['mean_far_sig'] - was['summary']['mean_far_sig']))
        for r in got['rows']:
            b = base_by.get(r['stunt'])
            if b and (abs(r['score'] - b['score']) > 1e-6 or r['met'] != b['met']):
                print('  %-24s score %5.2f -> %5.2f (%+.2f)  far_sig %.2f -> %.2f  met %s -> %s  engrams=%s' % (
                    r['stunt'], b['score'], r['score'], r['score'] - b['score'],
                    b['far_sig'], r['far_sig'], b['met'], r['met'], '+'.join(r['engrams']) or 'stock'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
