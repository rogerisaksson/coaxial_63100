#!/usr/bin/env python3
"""Data-oriented Monte Carlo, CMA-ES and invariant gate toolchain for ballistic capture dynamics.

    python tools/sim/ballistic.py                                       # evaluate default walk
    python tools/sim/ballistic.py --plan ballistic_walk --to 20         # evaluate behaviour
    python tools/sim/ballistic.py --mc 16 --span stride.LEADS=0.2:0.8 #204.amp.2=0.5:0.8
    python tools/sim/ballistic.py --search stride.LEADS=0.2:0.9 #204.amp.2=0.5:0.85

Orchestrates simulations across flat wire rows (`ortho.ROW`, `<4I59f`) and branchless module
constants (`stride`, `poise`, `impedance`, `modal`, `latent`) without coupling to anatomical
or kinematic names. Candidates are scored against SO(3) geodesic mocap alignment (`gvs`),
dimensionless Capture Point invariant gates, and normative human gait bounds (`normal`, `looks`).
"""
import argparse
import itertools
import json
import math
import os
import random
import re
import sys
import time

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np

from machine import figure, impedance, latent, modal, ortho, poise, sprung, stride
from machine.errors import MachineError
from tools import REPO
from tools.dev import background
from tools.sim import cmaes, fbx, gvs, look, looks, normal, strides
from tools.sim import ortho as sim_ortho

MODULES = {'stride': stride, 'poise': poise, 'impedance': impedance, 'modal': modal, 'latent': latent}
BLOCKS = {'z': ortho.Z, 'k': ortho.K, 'd': ortho.D, 'amp': ortho.AMP, 'phase': ortho.PHASE}
SCALARS = {'gain': ortho.GAIN, 'hz': ortho.HZ, 'zmp': ortho.ZMP, 'yaw': ortho.YAW}
HEADERS = {'id': (0, 1.0), 'at': (1, 1e3), 'blend': (2, 1e3), 'flags': (3, 1.0)}
RUN_GB, RUN_S, BATONS = 1.2, 300.0, None
LOG = os.path.join(REPO, 'build', 'ballistic.jsonl')
_WIRE_RE = re.compile(r'^wire\[(\d+),(\d+),(\d+)\]$')
_MOD_IDX_RE = re.compile(r'^([a-z_]+)\.([A-Z0-9_]+)(?:\[(\d+)\])?$')
_REF_CACHE = {}


def _mode_index(token):
    """Resolve an integer index `0..N-1` or mode name (with or without `mode_`) to column offset."""
    if token.isdigit():
        return int(token)
    clean = token[5:] if token.startswith('mode_') else token
    if clean in ortho.MODES:
        return ortho.MODES.index(clean)
    raise KeyError('unknown mode coordinate: %s' % token)


def _wire_coord(plan, key):
    """Resolve a wire coordinate key to `(entry_idx, row_idx, col_idx, scale)` in `plan`.
    Supports `wire[e,r,c]` or `<label>.[<phase>.]<field>[.<mode_or_idx>]`."""
    m = _WIRE_RE.match(key)
    if m:
        return int(m.group(1)), int(m.group(2)), int(m.group(3)), 1.0
    parts = key.split('.')
    labels = [item[1] for item in plan]
    if parts[0] not in labels:
        raise KeyError('entry %s not in plan %s' % (parts[0], labels))
    e_idx = labels.index(parts[0])
    names = plan[e_idx][3]
    rest = parts[1:]
    r_idx = 0
    if len(rest) >= 2 and (rest[0].isdigit() or rest[0] in names):
        r_idx = int(rest[0]) if rest[0].isdigit() else names.index(rest[0])
        rest = rest[1:]
    field = rest[0]
    if field in HEADERS:
        col, scale = HEADERS[field]
        return e_idx, r_idx, col, scale
    if field in SCALARS:
        return e_idx, r_idx, 4 + SCALARS[field], 1.0
    if field in BLOCKS and len(rest) == 2:
        scale = math.pi / 180.0 if field == 'phase' else 1.0
        return e_idx, r_idx, 4 + BLOCKS[field] + _mode_index(rest[1]), scale
    raise KeyError('invalid wire coordinate: %s' % key)


def now(plan, key):
    """Read current float value of `key` from module constants or `plan`'s compiled wire rows."""
    m = _MOD_IDX_RE.match(key)
    if m and m.group(1) in MODULES:
        mod, attr, idx = MODULES[m.group(1)], m.group(2), m.group(3)
        val = getattr(mod, attr)
        return float(val[int(idx)] if idx is not None else val)
    e_idx, r_idx, col, scale = _wire_coord(plan, key)
    rows = ortho.rows(plan[e_idx][2])
    return float(rows[r_idx][col]) / scale


def _set_mod_attr(mod, attr, val):
    """Set `attr` on `mod` and sync any re-exported copies across `MODULES`."""
    setattr(mod, attr, val)
    for other in MODULES.values():
        if other is not mod and hasattr(other, attr):
            setattr(other, attr, val)
    if attr == 'SWING':
        _set_mod_attr(stride, 'PHI_W', math.pi * float(val))


def put(plan, values):
    """Return a new `plan` with `values` `{key: float}` applied to module constants and wire rows."""
    mutated = [list(item) for item in plan]
    for key, val in values.items():
        val = float(val)
        m = _MOD_IDX_RE.match(key)
        if m and m.group(1) in MODULES:
            mod, attr, idx = MODULES[m.group(1)], m.group(2), m.group(3)
            cur = getattr(mod, attr)
            if idx is not None:
                i = int(idx)
                if isinstance(cur, tuple):
                    lst = list(cur)
                    lst[i] = val
                    _set_mod_attr(mod, attr, tuple(lst))
                else:
                    arr = np.array(cur, float, copy=True)
                    arr[i] = val
                    _set_mod_attr(mod, attr, arr)
            else:
                _set_mod_attr(mod, attr, val)
            continue
        e_idx, r_idx, col, scale = _wire_coord(mutated, key)
        rows = [list(r) for r in ortho.rows(mutated[e_idx][2])]
        rows[r_idx][col] = int(round(val * scale)) if col < 4 else float(val * scale)
        mutated[e_idx][2] = b''.join(ortho.ROW.pack(*r) for r in rows)
    return [tuple(item) for item in mutated]


def _snapshot():
    """Snapshot all uppercase constants across `MODULES` for clean restoration."""
    snap = {}
    for m_name, mod in MODULES.items():
        snap[m_name] = {k: (v.copy() if isinstance(v, np.ndarray) else v)
                        for k, v in vars(mod).items() if k.isupper()}
    return snap


def _restore(snap):
    """Restore uppercase constants across `MODULES` from `snap`."""
    for m_name, attrs in snap.items():
        mod = MODULES[m_name]
        for k, v in attrs.items():
            setattr(mod, k, v.copy() if isinstance(v, np.ndarray) else v)


def lhs_candidates(spans, count, seed=7):
    """Latin Hypercube sample of `count` candidate dicts over `spans` `{key: (lo, hi)}`."""
    rng = random.Random(seed)
    cols = {}
    for key, (lo, hi) in spans.items():
        slots = [(k + rng.random()) / count for k in range(count)]
        rng.shuffle(slots)
        cols[key] = [lo + (hi - lo) * u for u in slots]
    return [{k: cols[k][i] for k in spans} for i in range(count)]


def _swing_back_star(init_times, init_l_foot, init_r_foot, leg):
    """Dimensionless backward foot kick metric (`swing_back_star`) at first step onset."""
    if len(init_times) < 5:
        return 0.0
    n_init = min(len(init_times), 60)
    back_l = max(0.0, float(init_l_foot[0] - np.min(init_l_foot[:n_init])))
    back_r = max(0.0, float(init_r_foot[0] - np.min(init_r_foot[:n_init])))
    return max(back_l, back_r) / max(leg, 1e-6)


def evaluate(base_plan, values, to=15.0, ref_path=gvs.REF_DEFAULT, every=None):
    """Run candidate `values` on `base_plan` for `to` s and evaluate all invariant gates."""
    from terminal.views.humanoid_keys import HEADER, row
    snap = _snapshot()
    try:
        plan = put(base_plan, values)
        g = sprung.laid(every or sprung.STACK_EVERY)
        for at, label, wire, names in sorted(plan, key=lambda item: item[0]):
            sprung.said(g, label, wire, names, at)
        first_at = min(item[0] for item in plan)
        active = [(at, r) for at, _l, wire, _n in plan for r in ortho.rows(wire) if not (r[3] & ortho.REST)]
        rests = [at + 1e-3 * r[1] for at, _l, wire, _n in plan for r in ortho.rows(wire) if r[3] & ortho.REST]
        walks = max(at + 1e-3 * (r[1] + r[2]) for at, r in active)
        stop_t = min(walks + to, min(rests) - 0.05 if rests else math.inf)
        world = g['world']
        d, rows, frames_list, rob_times, told, stood = world.data, [], [], [], -1.0, True
        init_times, init_l, init_r = [], [], []
        while g['t'] < stop_t and stood:
            try:
                sprung.passed(g)
            except MachineError:
                stood = False
            stood = stood and sprung.tilt(g) <= sprung.DOWN_DEG and d.qpos[1] >= sprung.DOWN_M
            if g['t'] - told >= 1.0 / look.RATE_HZ:
                told = g['t']
                angles = {j: math.degrees(float(d.qpos[world.qadr[i]])) for i, j in enumerate(figure.JOINTS)}
                stepping = abs(float(g['out'][latent.SWAY][modal.DRIVE])) > stride.STIRS
                now_state = {'t': g['t'], 'stage': 'walk' if stepping else 'stand',
                             'speed': float(d.qvel[2]), 'phase': float(g['st'][latent.THETA]) / math.tau,
                             'loads': tuple(float(v) for v in g['loads']),
                             'where': tuple(d.qpos[0:3].tolist()), 'turn': tuple(d.qpos[3:7].tolist()),
                             'angles': angles, 'set': angles, 'watts': world.drawn()}
                rows.append(dict(zip(HEADER, row(now_state, 60.0)), loose=world.loose(),
                                 feet=world.gap(look.LEFT_FOOT, look.RIGHT_FOOT),
                                 lifted=world.lifted(look.LEFT_FOOT), landed=True,
                                 trunk=world.gap(('torso', 'head', 'upper_arm', 'forearm', 'hand'),
                                                 ('thigh', 'shank')),
                                 ahead=look._ahead(world), down='', laid=None, **look._held(world)))
                q = d.qpos[3:7]
                p_quat = figure.quat(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
                fr = figure.frames(angles, tuple(float(x) for x in d.qpos[0:3]), p_quat)
                if stepping:
                    init_times.append(g['t'])
                    init_l.append(float(fr['left_foot'][0][2]))
                    init_r.append(float(fr['right_foot'][0][2]))
                if g['t'] >= first_at:
                    rob_times.append(g['t'])
                    frames_list.append(fr)
    finally:
        _restore(snap)

    if not stood or len(rob_times) < 60:
        return {'stood': False, 't': round(g['t'], 3), 'cost': 1000.0 - round(g['t'], 3),
                'gps_deg': 99.0, 'off': 99.0, 'gates': [], 'broken': []}

    if ref_path not in _REF_CACHE:
        _REF_CACHE[ref_path] = gvs.extract_take_rotations(ref_path)
    ref_times, ref_joints, ref_rots = _REF_CACHE[ref_path]
    rob_joints = gvs._sim_joints(frames_list)
    rob_com, _v, rob_rots, leg = gvs.anatomical_frames(rob_times, rob_joints)
    gvs_scores, gps_rad = gvs.compute_gvs(
        ref_times, ref_joints['Hips'][:, 2], ref_rots,
        rob_times, rob_com, rob_rots, leg=leg, ref_joints=ref_joints, rob_joints=rob_joints)
    cap_metrics, cap_violations = gvs.capture_gates(rob_times, rob_com, rob_joints, leg=leg, gps_rad=gps_rad)
    swing_back = _swing_back_star(init_times, init_l, init_r, leg)
    cap_metrics['swing_back_star'] = swing_back
    if swing_back > 0.03:
        cap_violations.append(('swing_back_star', swing_back, 0.03))

    walk = strides.steady(rows)
    got = strides.measured(walk) if len(walk) >= 3 * look.RATE_HZ else {'fell': True}
    band = normal.measured(*fbx.joints(walk)) if len(walk) >= 3 * look.RATE_HZ else {}
    off, out_band = normal.off(band) if band else (20.0, [])
    named = normal.named(band) if band else ['fell']
    words = dict(normal.said(band)) if band else {}
    if words.get('stiff', 0.0) >= 0.5:
        cap_violations.append(('stiff', float(words['stiff']), 0.5))
    broken = looks.broken(got) if not got.get('fell') else [('fell', 1.0, 0.0)]
    gate_pen = sum(25.0 * (abs(v - b) / max(abs(b), 0.1) if v == v else 2.0) for _n, v, b in cap_violations)
    cost = float(cap_metrics['gps_deg'] + 4.0 * off + 0.5 * looks.priced(got)
                 + 50.0 * swing_back + gate_pen)
    return {
        'stood': bool(stood), 't': round(float(g['t']), 3), 'cost': round(float(cost), 3),
        'speed': round(float(rob_com[-1] - rob_com[0]) / max(float(rob_times[-1] - rob_times[0]), 1e-3), 3),
        'gps_deg': round(float(cap_metrics['gps_deg']), 2),
        'first_step_star': round(float(cap_metrics['first_step_star']), 3) if not math.isnan(cap_metrics['first_step_star']) else -1.0,
        'step_star': round(float(cap_metrics['step_star']), 3) if not math.isnan(cap_metrics['step_star']) else -1.0,
        'walk_ratio': round(float(cap_metrics['walk_ratio']), 3) if not math.isnan(cap_metrics['walk_ratio']) else -1.0,
        'stall_share': round(float(cap_metrics['stall_share']), 3),
        'init_tau': round(float(cap_metrics['init_tau']), 2),
        'arm_deg': round(float(cap_metrics['arm_deg']), 1) if not math.isnan(cap_metrics['arm_deg']) else -1.0,
        'heel_deg': round(float(cap_metrics['heel_deg']), 1) if not math.isnan(cap_metrics['heel_deg']) else -1.0,
        'swing_back_star': round(float(cap_metrics['swing_back_star']), 4),
        'off': round(float(off), 2), 'named': list(named),
        'out_band': [(str(n), round(float(v), 2), round(float(b), 2)) for n, v, b in out_band],
        'words': [(str(w), round(float(val), 3)) for w, val in (normal.said(band) if band else ())],
        'gates': [(str(n), round(float(v), 3) if v == v else -1.0, round(float(b), 3)) for n, v, b in cap_violations],
        'broken': [(str(n), round(float(v), 2) if v == v else -1.0, float(b)) for n, v, b in broken],
        'gvs_deg': {str(k): round(math.degrees(float(v)), 2) for k, v in gvs_scores.items()},
    }


def run_batch(asked, file_path, to, ref_path, candidates):
    """Evaluate `candidates` in sharded workers across `focus.relay`."""
    import tempfile
    from tools.dev import focus
    n_shards = max(1, min(len(candidates), BATONS or 8))
    got = {}
    with tempfile.TemporaryDirectory() as tmp:
        jobs = []
        for k in range(n_shards):
            shard_path = os.path.join(tmp, 'shard_%d.json' % k)
            items = [[i, candidates[i]] for i in range(k, len(candidates), n_shards)]
            with open(shard_path, 'w', encoding='utf-8') as f:
                json.dump({'asked': asked, 'file': file_path, 'to': to, 'ref': ref_path, 'items': items}, f)
            jobs.append(focus.Job('ballistic.%d' % k,
                                  [sys.executable, '-X', 'utf8', os.path.abspath(__file__), '--one', shard_path],
                                  RUN_GB, RUN_S))
        for job, text, _code, _s in focus.relay(jobs, BATONS):
            line = next((ln for ln in reversed(text.splitlines()) if ln.startswith('{')), None)
            if line:
                for i, r in json.loads(line).get('rows', ()):
                    got[int(i)] = r
            else:
                sys.stderr.write('WARN %s failed:\n%s\n' % (job.name, text[-600:]))
    out = []
    for i in range(len(candidates)):
        r = got.get(i, {'stood': False, 'cost': 1000.0, 'gps_deg': 99.0, 'off': 99.0})
        out.append((float(r['cost']), 1.0 if r.get('stood') else 0.0, float(r.get('gps_deg', 99.0)), r))
    return out


def _show(values, res):
    label = ' '.join('%s=%g' % kv for kv in values.items()) or 'baseline'
    status = 'PASS' if res.get('stood') and not res.get('gates') else ('STOOD' if res.get('stood') else 'DOWN')
    print('[%5s] cost %6.2f | GPS %5.2f deg | off %4.2f | step* %.3f | wr %.2f | tau %4.1f | back* %.3f | v %.2f m/s | %s' % (
        status, res['cost'], res.get('gps_deg', 99.0), res.get('off', 99.0),
        res.get('step_star', -1.0), res.get('walk_ratio', -1.0), res.get('init_tau', 99.0),
        res.get('swing_back_star', 0.0), res.get('speed', 0.0), label))
    if res.get('words'):
        print('    words        : %s (out: %s)' % (
            ', '.join('%s=%.2f' % tuple(w) for w in res['words']),
            ', '.join('%s=%g(%g)' % tuple(o) for o in res.get('out_band', ()))))
    if res.get('gates'):
        print('    gates broken : %s' % ', '.join('%s=%g (bound %g)' % tuple(g) for g in res['gates']))
    if res.get('broken'):
        print('    form broken  : %s' % ', '.join('%s=%g (%g)' % tuple(b) for b in res['broken']))


def main(argv=None):
    global BATONS
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--plan', nargs='*', default=None, help='plan entries (default: sprung.WALK)')
    parser.add_argument('--file', default=None, help='optional Lisp file path')
    parser.add_argument('--to', type=float, default=15.0, help='seconds followed after plan blend (15)')
    parser.add_argument('--ref', default=gvs.REF_DEFAULT, help='reference FBX mocap take')
    parser.add_argument('--set', nargs='*', default=[], metavar='K=V', help='fixed coordinate overrides')
    parser.add_argument('--grid', nargs='*', default=[], metavar='K=V1,V2', help='grid sweep coordinates')
    parser.add_argument('--mc', type=int, default=0, help='number of Latin Hypercube Monte Carlo draws')
    parser.add_argument('--span', nargs='*', default=[], metavar='K=LO:HI', help='parameter spans for --mc')
    parser.add_argument('--search', nargs='*', default=[], metavar='K=LO:HI', help='CMA-ES optimization spans')
    parser.add_argument('--generations', type=int, default=8)
    parser.add_argument('--population', type=int, default=8)
    parser.add_argument('--sigma', type=float, default=0.15)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--log', default=LOG)
    parser.add_argument('--batons', type=int, default=None)
    parser.add_argument('--one', default=None, help='internal worker JSON evaluation')
    args = parser.parse_args(argv)
    BATONS = args.batons
    background.lower()

    if args.one is not None and os.path.isfile(args.one):
        with open(args.one, encoding='utf-8') as f:
            spec = json.load(f)
        s_text = None
        if spec.get('file'):
            with open(spec['file'], encoding='utf-8') as f:
                s_text = f.read()
        s_plan = sim_ortho.planned(spec['asked'], s_text)
        rows = [[i, evaluate(s_plan, cand, to=spec['to'], ref_path=spec['ref'])]
                for i, cand in spec['items']]
        print(json.dumps({'rows': rows}))
        return 0

    text = None
    if args.file:
        with open(args.file, encoding='utf-8') as f:
            text = f.read()
    asked = args.plan or ['%s@%g' % entry for entry in sprung.WALK]
    base_plan = sim_ortho.planned(asked, text)

    if args.one is not None:
        res = evaluate(base_plan, json.loads(args.one), to=args.to, ref_path=args.ref)
        print(json.dumps({'result': res}))
        return 0

    fixed = {k: float(v) for k, v in (a.split('=') for a in args.set)}
    began = time.time()
    os.makedirs(os.path.dirname(args.log), exist_ok=True)
    with open(args.log, 'a', encoding='utf-8') as log:
        if args.search:
            spans = {k: tuple(float(x) for x in v.split(':')) for k, v in (a.split('=') for a in args.search)}
            cost, best_vals = cmaes.search(
                None, spans, args.generations, args.population, log,
                lambda _pool, cands: run_batch(asked, args.file, args.to, args.ref,
                                               [dict(fixed, **c) for c in cands]),
                lambda k: now(base_plan, k), args.sigma, start=fixed)
            print('BEST %.2f %s' % (cost, json.dumps(best_vals)))
            cands = [dict(fixed, **(best_vals or {})), fixed]
        elif args.mc > 0 and args.span:
            spans = {k: tuple(float(x) for x in v.split(':')) for k, v in (a.split('=') for a in args.span)}
            cands = [dict(fixed)] + [dict(fixed, **c) for c in lhs_candidates(spans, args.mc, args.seed)]
        elif args.grid:
            axes = [(k, [float(x) for x in v.split(',')]) for k, v in (a.split('=') for a in args.grid)]
            cands = [dict(fixed, **dict(zip([k for k, _v in axes], combo)))
                     for combo in itertools.product(*[v for _k, v in axes])]
        else:
            cands = [fixed]

        if len(cands) == 1:
            res = evaluate(base_plan, cands[0], to=args.to, ref_path=args.ref)
            _show(cands[0], res)
            return 0 if res.get('stood') and not res.get('gates') else 1

        got = run_batch(asked, args.file, args.to, args.ref, cands)
        ranked = sorted(zip(cands, got), key=lambda item: item[1][0])
        for vals, (_cost, _stood, _gps, res) in ranked:
            log.write(json.dumps({'values': vals, 'result': res}) + '\n')
            _show(vals, res)
    print('%d candidates in %.1f s' % (len(cands), time.time() - began))
    return 0


if __name__ == '__main__':
    sys.exit(main())

