"""A handover row's sums past its landings: the double support's load planning, an ensemble's summary.

    row.update(handovers.planned(A, at, dt, fwd, b10, landed))   # the plan's loads against the sensed, medians
    print(handovers.ensemble(spec, rows))                        # one line for a spec's runs over ENSEMBLE

`A` the run's log as arrays (`handover.ran`), `at` its clean landings (sample, sole), `dt` the log's step, `fwd`
and `b10` samples (0.15 s on, 0.10 s back), `landed` the load share a landing is; `rows` the runs' rows (dicts).
"""
import json
import math
import os
import sys
from typing import Any

import numpy as np

from tools.sim.knobs import expand_variants, merged

#: An ensemble's starts: the walk's drive and its axial target, a run each pair (`handover --ensemble`).
ENSEMBLE = ((0.84, 0.85, 0.86), (0.028, 0.03, 0.032))


def planned(A, at, dt, fwd, b10, landed):
    """The double support's load planning against the sensed loads, medians over the landings `at`: the mean
    |planned - real| share over `fwd` passes from a landing for the landing sole and the trailing one (from `b10`
    passes before), ms after the landing when the plan zeroes the trailing sole and when its load leaves."""
    errs_b, errs_o, zero, left = [], [], [], []
    for i, b in at:
        o, hi = 1 - b, min(len(A['t']), i + fwd)
        errs_b.append(np.abs(A['qp'][i:hi, b] - A['ld'][i:hi, b]).mean())
        errs_o.append(np.abs(A['qp'][i - b10:hi, o] - A['ld'][i - b10:hi, o]).mean())
        gone = np.flatnonzero(A['qp'][i:hi, o] <= 0.02)
        off = np.flatnonzero(A['ld'][i:hi, o] < landed)
        zero.append(1e3 * dt * (gone[0] if len(gone) else fwd))
        left.append(1e3 * dt * (off[0] if len(off) else fwd))
    return {k: round(float(np.median(v)), 3) for k, v in (('plan_err_land', errs_b), ('plan_err_trail', errs_o),
                                                         ('plan_zero_ms', zero), ('trail_off_ms', left)) if v}


def ensemble(spec, rows):
    """One line for `spec`'s ensemble `rows` (dicts): how many stood of how many, when the rest fell, the hip clamp
    passes' median and worst, the speed's, the landing knee's and the plan's load errors' medians, the least grip any
    start felt."""
    med = lambda k: round(float(np.median([r[k] for r in rows if r.get(k) is not None])), 3) if any(r.get(k) is not None for r in rows) else None
    return json.dumps({'spec': spec, 'runs': len(rows), 'stood': sum(bool(r.get('stood')) for r in rows),
                       'fell_at': sorted(round(r['fell'], 1) for r in rows if r.get('fell') is not None),
                       'stumbles': sum(len(r.get('stumbles') or ()) for r in rows),
                       'hip_passes_med': med('hip_passes'), 'hip_passes_max': max((r.get('hip_passes') or 0) for r in rows),
                       'v_med': med('v'), 'land_knee_med': med('land_knee'), 'knee_min_med': med('knee_min'),
                       'plan_err_land_med': med('plan_err_land'), 'plan_err_trail_med': med('plan_err_trail'),
                       'plan_zero_ms_med': med('plan_zero_ms'), 'trail_off_ms_med': med('trail_off_ms'),
                       'grip_min': min((r['grip_min'] for r in rows if r.get('grip_min') is not None), default=None)})


def ran_special(spec: dict[str, Any], play: str, wobble: float, texts: Any) -> dict[str, Any]:
    """Run `'driven'` (page WASD sequence) or `'scenario:<name>'` at initial yaw `wobble`, returning a uniform row."""
    from tools.sim import scenarios
    label = json.dumps(spec, separators=(',', ':')) if any(isinstance(v, dict) for v in spec.values()) else ','.join('%s=%s' % kv for kv in spec.items())
    if play.startswith('scenario:'):
        sname = play.split(':', 1)[1]
        got = scenarios.ran(sname, wobble, **({'texts': texts} if texts is not None else {}))
        return {'spec': label, 'play': play, 'wobble': wobble, 'stood': got['fell_at'] is None and bool(got['met']),
                'met': bool(got['met']), 'fell': got['fell_at'], 'stumbles': [] if got['met'] else ['not_met'], 'spasms': 0}
    from types import SimpleNamespace as SN
    from machine.process import introspect, watch, worker
    from machine.world import physics
    from terminal.ui import bindings
    from terminal.views import humanoid_keys
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    held = (('w', 3.0), ('wa', 2.0), ('wd', 2.0), ('', 3.0), ('s', 2.0), ('', 3.0))
    s = worker.session(texts=texts)
    worker.apply(s, {'scenario': 'flat'})
    if wobble:
        world = s['g']['world']
        q, cw, sw = world.data.qpos[3:7].copy(), math.cos(wobble / 2.0), math.sin(wobble / 2.0)
        world.data.qpos[3:7] = (q[0] * cw - q[2] * sw, q[1] * cw + q[3] * sw, q[2] * cw + q[0] * sw, q[3] * cw - q[1] * sw)
        world._mj.mj_forward(world.model, world.data)
    for _ in range(1000):
        worker.stepped(s)
    sent, told = [], []
    st = {'body': SN(send=lambda **cmd: sent.append(cmd)), 'scanner': None, 'tour': None, 'yaw': 180.0,
          'pad': humanoid_keys.pad(), 'now': worker.told(s)}
    tap, watched = introspect.Tap(), watch.Watch()
    frame, typed_at, min_sr = 0.0, -math.inf, 1.0
    for word, seconds in held:
        t0 = float(s['g']['t'])
        while s['g']['t'] - t0 < seconds and s['fallen_at'] is None:
            for cmd in sent:
                worker.apply(s, cmd)
            sent.clear()
            worker.stepped(s)
            min_sr = min(min_sr, float(s['g']['surety']))
            t = float(s['g']['t'])
            if word and t - typed_at >= 0.033:
                typed_at = t
                bindings.act_on(humanoid_keys.PAD, list(word), st, t)
            if t - frame >= 0.05:
                frame = t
                st['now'] = worker.told(s)
                humanoid_keys.steered(st, t, 0.0)
            snap = tap.passed(s)
            if snap is not None:
                told.extend(watched.step(snap))
    stumbles = [round(e['t'], 2) for e in told if e['event'] in ('stumble', 'fall')]
    return {'spec': label, 'play': play, 'wobble': wobble, 'stood': s['fallen_at'] is None, 'fell': s['fallen_at'],
            'min_sr': round(min_sr, 4), 'stumbles': stumbles, 'spasms': sum(e['event'] == 'spasm' for e in told)}


def run_sweep(cfg: dict[str, Any], args: Any, script_path: str) -> int:
    """Expand a JSON/TOML sweep config into LPT-sorted parallel `focus.relay` jobs across all batons."""
    from tools.dev import focus
    here = [sys.executable, '-X', 'utf8', script_path]
    variants = expand_variants(cfg)
    plays: list[str] = [str(p) for p in (cfg.get('plays') or [args.play])]
    wobbles = [float(w) for w in (cfg.get('wobbles') or [args.wobble])]
    ens = cfg.get('ensemble', args.ensemble)
    starts = [{'drive': d, 'ax': ax} for d, ax in (zip(*ENSEMBLE) if ens == 'diag' else [(d, a) for d in ENSEMBLE[0] for a in ENSEMBLE[1]])] if ens else [{}]
    win = str(cfg.get('window') or args.window)
    items = []
    for vname, vspec in variants:
        for play in plays:
            wt = 30.0 if play == 'driven' else (20.0 if play.startswith('scenario:') else 15.0)
            for idx, st_spec in enumerate(starts if not play.startswith('scenario:') else [{}]):
                full = merged(vspec, st_spec)
                for wob in wobbles:
                    jname = '%s|%s|%d|%g' % (vname, play, idx, wob)
                    cmd = here + ['--spec', json.dumps(full), '--play', play, '--window', win, '--wobble', str(wob)]
                    items.append((vname, wt, focus.Job(jname, cmd, 0.5, 900.0)))
    items.sort(key=lambda it: -it[1])
    got = {}
    for job, text, _code, _sec in focus.relay([it[2] for it in items]):
        lines = [l for l in text.splitlines() if l.startswith('{')]
        got[job.name] = json.loads(lines[-1]) if lines else {'stood': False, 'err': text[-200:]}
    for vname, _vspec in variants:
        sub = [got[it[2].name] for it in items if it[0] == vname]
        bad = [r for r in sub if not r.get('stood') or r.get('stumbles') or 'err' in r]
        if args.rows:
            for r in sub:
                print(json.dumps(r))
        print(json.dumps({'variant': vname, 'runs': len(sub), 'stood': sum(bool(r.get('stood')) for r in sub),
                          'bad': len(bad), 'stumbles': sum(len(r.get('stumbles') or ()) for r in sub),
                          'spasms': sum(r.get('spasms') or 0 for r in sub),
                          'fell_at': sorted(round(r['fell'], 2) for r in sub if r.get('fell') is not None)}))
    return 0


def run_specs(args: Any, script_path: str) -> int:
    """Run `--specs` variants across `focus.relay`, optionally expanded over `ENSEMBLE`."""
    from tools.dev import focus
    here = [sys.executable, '-X', 'utf8', script_path]
    starts = [',drive=%g,ax=%g' % (d, ax) for d in ENSEMBLE[0] for ax in ENSEMBLE[1]] if args.ensemble else ['']
    runs = [(spec, spec + start) for spec in args.specs for start in starts]
    jobs = [focus.Job(run or 'base', here + ['--spec', run, '--play', args.play, '--window', args.window, '--wobble', str(args.wobble)]
                      + (['--entries', args.entries] if args.entries else []) + (['--trace'] if args.trace else [])
                      + (['--trace-at', str(args.trace_at)] if args.trace_at is not None else []) + (['--trace-right'] if args.trace_right else []), 0.5, 900.0)
            for _spec, run in runs]
    rows = {}
    for job, text, _code, seconds in focus.relay(jobs):
        lines = [line for line in text.splitlines() if line.startswith('{')]
        rows[job.name] = (lines[-1] if lines else 'NO ROW: ' + ' | '.join(text.splitlines()[-3:])[-400:]) + ' (%.0f s)' % seconds
        if args.trace:
            traced = [line for line in text.splitlines() if line[:1] in 't-0123456789']
            rows[job.name] = os.linesep.join([job.name + ':'] + traced + [rows[job.name]])
    if args.ensemble:
        for spec in args.specs:
            texts = [rows[run or 'base'] for s, run in runs if s == spec]
            if args.rows:
                for r in (json.loads(t[:t.rfind('}') + 1]) for t in texts if t.startswith('{')):
                    print(json.dumps({k: r.get(k) for k in ('spec', 'stood', 'fell', 'stumbles', 'hip_passes', 'v', 'grip_min')}))
            print(ensemble(spec or 'base', [json.loads(t[:t.rfind('}') + 1]) for t in texts if t.startswith('{')]))
        return 0
    for spec in args.specs:
        print(rows[spec or 'base'])
    return 0
