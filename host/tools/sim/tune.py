#!/usr/bin/env python3
"""The autotuner on her loop: a gait of the suite played, its six classes measured, a step judged.

    python tools/sim/tune.py patch-sneaker-runway             # its reach, its Jacobian, a step judged
    python tools/sim/tune.py patch-sneaker-runway --steps 3   # each from the last kept
    python tools/sim/tune.py patch-sneaker-runway --write     # the kept texts written, converged
    python tools/sim/tune.py sit_chair --reach                # what each literal moves, nothing run
    python tools/sim/tune.py ROOT --one --set=PATH:START:END=V  # one run as JSON, a relay job's

A run is the suite's (`suite.played`): her walk, the root's row laid, 20 s; from SETTLE_S her world
is sampled each pass for the text's telemetry, each class a residual in its dimension (CHANNELS):
the centroidal pivot's shortfall inside her soles' contact hull, her load's below her weight's
floor, her soles' slip over a sole's length, her feet's gap's shortfall, her landing's load past
her form's, her drives' energy a metre over her walk's price. Its Jacobian is a run pair a live
literal (`tune.reach`), its scale times the text's trust region either side, on the relay's batons;
the step the text's (`tune.step`), her form's measures held from growing by it, kept unless
VERDICTS say otherwise, its trust region halved till its model's gain is under the floor. Written
(--write) where the residuals' norm moved less than the text's tol.
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np  # behind the OpenBLAS cap

from tools.sim import look, looks, strides, suite

#: Each class's measure of hers, how it is a residual, and its bound: None the class's invariant
#: (the manifold's), a name the manifold's plantar boundary's, a number her form's or her walk's.
CHANNELS = {'com-wrench-balance': ('pivot', 'short', None), 'normal-force-floor': ('load', 'short', None),
            'tangential-shear-slip': ('slip', 'over', 'support-polygon-x-mm'),
            'spatial-sdf-clearance': ('gap', 'short', None),
            'contact-transient-ringdown': ('landing load', 'past', looks.LANDS),
            'actuator-thermal-passivity': ('energy', 'over', looks.ENERGY_J_M)}
#: A measure as a residual, its bound's share: its mean shortfall under the bound, how far its
#: most is past it, its size.
HOW = {'short': lambda x, b: float(np.mean(np.maximum(0.0, b - np.atleast_1d(x)))) / b,
       'past': lambda x, b: max(0.0, float(np.max(x)) - b) / b, 'over': lambda x, b: float(x) / b}


def norm(run):
    """A run's residuals' norm."""
    return float(np.linalg.norm(run['r']))


def off(run):
    """{measure: how far a run's form is outside its band (`looks.FORM`)}: unmeasured, infinitely."""
    out = {}
    for name, least, most in looks.FORM:
        v = run['form'].get(name)
        out[name] = math.inf if v is None or v != v else max(0.0, (least - v) if least is not None else 0.0,
                                                             (v - most) if most is not None else 0.0)
    return out


def _worse(before, after):
    """The measures of her form `after` moves further outside its band than `before`'s floor, on a
    flat floor: its form is a condition, never a price."""
    b, a = off(before), off(after)
    return sorted(name for name in a if a[name] - b[name] > before['off_floor'][name]) if after['scene'] == 'flat' else []


#: Why a step is not kept: (its test on the runs before and after it, what it says). A floor is
#: how far a measure moves with each live literal a unit of the text's last place off before's.
VERDICTS = ((lambda b, a: 'error' in a, lambda b, a: 'not run: %s' % a.get('error', '')[-160:]),
            (lambda b, a: a['fell'] is not None, lambda b, a: 'she fell at %.2f s' % a['fell']),
            (lambda b, a: bool(_worse(b, a)), lambda b, a: 'her form worse: ' + ', '.join(
                '%s %.3f -> %.3f' % (n, b['form'][n], a['form'][n]) for n in _worse(b, a))),
            (lambda b, a: norm(b) - norm(a) <= b['floor'],
             lambda b, a: '|r| %.4f -> %.4f' % (norm(b), norm(a))))


def judged(before, after):
    """Why the step from run `before` to run `after` is not kept (VERDICTS), or None."""
    return next((why(before, after) for test, why in VERDICTS if test(before, after)), None)


def _hull(points):
    """The convex hull of (x, z) `points`, counter-clockwise (Andrew's monotone chain)."""
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts
    turn = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    chain = []
    for side in (pts, pts[::-1]):
        part = []
        for p in side:
            while len(part) >= 2 and turn(part[-2], part[-1], p) <= 0.0:
                part.pop()
            part.append(p)
        chain += part[:-1]
    return chain


def margin(p, points):
    """How far point `p` lies inside the hull of `points`, m: negative outside; the hull of one or
    two points a point or a segment, everything off it outside."""
    hull = _hull(points)
    if len(hull) < 3:
        a, b = hull[0], hull[-1]
        ab = (b[0] - a[0], b[1] - a[1])
        t = max(0.0, min(1.0, ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / max(ab[0] ** 2 + ab[1] ** 2, 1e-300)))
        return -math.hypot(p[0] - a[0] - t * ab[0], p[1] - a[1] - t * ab[1])
    return min(((p[1] - a[1]) * (b[0] - a[0]) - (p[0] - a[0]) * (b[1] - a[1])) / math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(hull, hull[1:] + hull[:1]))


def _touching(world, hers, force, v6):
    """(the floor's force on her, N, (x, y, z); each contact's (x, z); their mean height; the
    soles' slip speed, m/s, weighed by their normal forces) of her soles' contacts now."""
    m, d, mj = world.model, world.data, world._mj
    f_on, points, ys, slip, normal = np.zeros(3), [], [], 0.0, 0.0
    for i in range(d.ncon):
        c = d.contact[i]
        a, b = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        mine = c.geom1 if hers[a] else c.geom2
        if (hers[a] > 0) == (hers[b] > 0) or hers[m.geom_bodyid[mine]] < 2:
            continue
        mj.mj_contactForce(m, d, i, force)
        frame = np.asarray(c.frame).reshape(3, 3)
        f_on += (frame.T @ force[:3]) * (1.0 if mine == c.geom2 else -1.0)
        points.append((float(c.pos[0]), float(c.pos[2])))
        ys.append(float(c.pos[1]))
        mj.mj_objectVelocity(m, d, mj.mjtObj.mjOBJ_GEOM, mine, v6, 0)
        v = v6[3:] + np.cross(v6[:3], np.asarray(c.pos) - d.geom_xpos[mine])
        slip += abs(float(force[0])) * float(np.linalg.norm(v - float(v @ frame[0]) * frame[0]))
        normal += abs(float(force[0]))
    return f_on, points, (sum(ys) / len(ys) if ys else 0.0), slip / normal if normal else 0.0


def sampler(rec):
    """A pass's sampling into `rec` from SETTLE_S on: the centroidal pivot's margin in her soles'
    contact hull, mm (where her centre of mass's line along the floor's force meets the floor), her
    load over her weight, her soles' slip, mm, her feet's gap, mm, every ROW_S, her drives' joules,
    her pelvis's place first and last."""
    from machine import edge, modal

    def laid(s):
        g = s['g']
        world, t = g['world'], g['t']
        if t < suite.SETTLE_S:
            return
        m, d = world.model, world.data
        if not rec['hers']:
            hers = np.array([edge._rooted(m, b, world.pelvis) for b in range(m.nbody)], int)
            hers[[m.body(n).id for n in edge.INSOLES]] = 2
            rec.update(hers=hers.tolist(), weight=float(g['fr'][modal.WEIGHT]), t=t, first=d.qpos[[0, 2]].tolist())
        f_on, points, floor_y, slip = _touching(world, rec['hers'], rec['force'], rec['v6'])
        dt = t - rec['t']
        rec['load'] = min(rec['load'], float(f_on[1]) / rec['weight'])
        rec['slip'] += 1e3 * slip * dt
        if points and f_on[1] > 0.0:
            c = d.subtree_com[world.pelvis]
            k = (float(c[1]) - floor_y) / float(f_on[1])
            rec['pivot'].append(1e3 * margin((float(c[0]) - k * f_on[0], float(c[2]) - k * f_on[2]), points))
        rec['joules'] += world.drawn() * dt
        if t - rec['gap_t'] >= suite.ROW_S - 1e-9:
            rec.update(gap=min(rec['gap'], 1e3 * world.gap(look.LEFT_FOOT, look.RIGHT_FOOT)), gap_t=t)
        rec.update(t=t, last=d.qpos[[0, 2]].tolist())
    return laid


def residuals(measures, gr, bounds):
    """The text's six residuals of her `measures`, in its order, each in its dimension (CHANNELS),
    `bounds` the manifold's invariants and plantar boundary."""
    out = []
    for name, r in gr.items():
        measure, how, bound = CHANNELS[name]
        out.append(HOW[how](measures[measure], bounds[r.invariant] if bound is None else bounds.get(bound, bound)))
    return np.array(out)


def ran(root, said):
    """{r, measures, form, broken, fell, scene, wall s} of gait `root` played on her walk, the
    suite's texts with `said` ({(path, start, end): value}) at those spans."""
    from machine import dsp, tune, voicing
    from machine.errors import MachineError
    lits = [tune.Literal(p, a, z, None, None, None, None, v) for (p, a, z), v in said.items()]
    got = dsp.read(dsp.SUITE, tune.spliced(tune.texts(), lits, [lit.value for lit in lits]))
    if not isinstance(got.get(root), dict):
        raise MachineError("%s: a run plays a gait of the suite; a stunt's is the gym's" % root)
    row, told = voicing.voiced(got[root], got['nominal-biped-manifold'], suite._her())
    scene = suite.scene_of(got[root])
    rec = {'hers': None, 'force': np.zeros(6), 'v6': np.zeros(6), 'load': math.inf, 'slip': 0.0, 'pivot': [],
           'joules': 0.0, 'gap': math.inf, 'gap_t': -math.inf}
    began = time.time()
    rows, fell = suite.played(row, told, scene, laid=sampler(rec))
    form = strides.measured(rows)
    way = math.hypot(rec['last'][0] - rec['first'][0], rec['last'][1] - rec['first'][1])
    measures = {'pivot': rec['pivot'], 'load': rec['load'], 'slip': rec['slip'], 'gap': rec['gap'],
                'landing load': form['landing load'], 'energy': rec['joules'] / way}
    bounds = dict(got['nominal-biped-manifold']['plantar-boundary'], **got['physical-invariants'])
    pivot = np.array(rec['pivot'])
    return {'r': residuals(measures, tune.grammar(), bounds).tolist(), 'fell': fell, 'scene': scene,
            'measures': dict(measures, pivot=[float(np.median(pivot)), float(np.mean(pivot < bounds['min-zmp-margin-mm']))]),
            'form': {name: form.get(name) for name, _least, _most in looks.FORM},
            'broken': [name for name, _v, _b in looks.broken(form)], 'wall': round(time.time() - began, 1)}


def _said(arg):
    """((path, start, end), value) of a --set PATH:START:END=VALUE."""
    where, value = arg.rsplit('=', 1)
    path, start, end = where.rsplit(':', 2)
    return (path, int(start), int(end)), float(value)


def runs(root, states):
    """[`ran`'s dict, or {error}] of gait `root` at each of `states` ({span: value}), side by side
    on the relay's batons."""
    from machine import tune
    from tools.dev import focus
    jobs = [focus.Job('tune %d' % k, [sys.executable, '-X', 'utf8', os.path.abspath(__file__), root, '--one'] + [
        '--set=%s:%d:%d=%s' % (p, a, z, tune.number(v)) for (p, a, z), v in said.items()], 1.5, 3600.0, None)
            for k, said in enumerate(states)]
    out: list = [{'error': 'not run'}] * len(states)
    for job, text, code, _seconds in focus.relay(jobs):
        found = [ln for ln in (text or '').splitlines() if ln.startswith('{')]
        out[int(job.name.split()[-1])] = json.loads(found[-1]) if found else {'error': (text or '')[-300:], 'code': code}
    return out


def walk_report(root, table, cols, refused):
    """The binding table's lines: * a live literal and how many voices it moves, ! one refused and
    why, the rest inert."""
    from machine import dsp
    lines = ['%s: %d literals walked, %d live, %d refused, %d inert' % (
        root, len(table), len(cols), len(refused), len(table) - len(cols) - len(refused))]
    for i, (lit, cls) in enumerate(table):
        mark = '*' if i in cols else '!' if i in refused else ' '
        where = '%s:%d' % (os.path.basename(lit.path), dsp.text(lit.path).count('\n', 0, lit.start) + 1)
        said = '%d voices' % len(cols[i]) if i in cols else refused.get(i, '')
        lines.append(' %s %-34s %-28s %-24s %-26s %-9s %9g  %-26s %s' % (
            mark, where, lit.symbol[:28], lit.node[:24], lit.key, lit.op or '', lit.value, cls, said[:60]))
    return '\n'.join(lines)


def run_report(gr, before, after, table, live, cur, new):
    """A step's lines: each class's residual before and after, her measures, each literal's move."""
    lines = ['  %-28s %-32s %9s %9s' % ('class', 'telemetry', 'before', 'after')]
    lines += ['  %-28s %-32s %9.4f %9.4f' % (name, r.telemetry, before['r'][k], after['r'][k] if 'r' in after else math.nan)
              for k, (name, r) in enumerate(gr.items())]
    for run, label in ((before, 'before'), (after, 'after')):
        m = run.get('measures', {})
        lines.append('  %s: pivot median %.1f mm, under its bound %.0f %%; load >= %.3f; slip %.0f mm; feet >= %.1f mm; '
                     'landing %.2f; %.0f J/m; form broken: %s' % (
                         label, m.get('pivot', [math.nan, math.nan])[0], 1e2 * m.get('pivot', [math.nan, math.nan])[1],
                         m.get('load', math.nan), m.get('slip', math.nan), m.get('gap', math.nan),
                         m.get('landing load', math.nan), m.get('energy', math.nan), ', '.join(run.get('broken', [])) or '-'))
    lines += ['  %-28s %-24s %-26s %9g -> %g' % (table[i][0].symbol[:28], table[i][0].node[:24], table[i][0].key, cur[j], new[j])
              for j, i in enumerate(live)]
    return '\n'.join(lines)


def tuned(root, steps):
    """`root`'s live literals stepped up to `steps` times from the texts as they are: (the kept
    state {span: value}, whether its last step converged), each step printed with its Jacobian's
    runs, its residuals before and after and why it was kept or not."""
    from machine import dsp, tune
    gr, tx, fr = tune.grammar(), tune.texts(), suite._her()
    table = tune.walked(root, gr, tx)
    cols, refused = tune.reach(root, table, tx, fr)
    print(walk_report(root, table, cols, refused), flush=True)
    live, got, k = sorted(cols), dsp.read(dsp.SUITE, tx), tune.knobs()
    c = {'root': root, 'gr': gr, 'table': table, 'live': live, 'digits': k[('sync-mutated-packages-to-disk', 'float-precision')],
         'scales': np.array([tune.scale(table[i][0], got) for i in live]),
         'span': [(table[i][0].path, table[i][0].start, table[i][0].end) for i in live]}
    c['h'] = k[('compute-adjoint-ast-deltas', 'trust-region-scale')] * c['scales']
    state, converged = {}, False
    before = runs(root, [state])[0]
    print('' if 'r' in before else 'her walk with it not run: %s' % before.get('error', '')[-300:], flush=True)
    for n in range(steps if live and 'r' in before else 0):
        cur = np.array([state.get(c['span'][j], table[i][0].value) for j, i in enumerate(live)])
        J, held, ok = measured_at(c, state, cur, before, n)
        trial, after = stepped(c, state, cur, before, (J, held, ok))
        if after is None:
            converged = bool(ok)
            break
        converged = abs(norm(after) - norm(before)) < k[('universal-autotune-daemon', 'tol')] * norm(before)
        state, before = trial, after
    return state, converged


def measured_at(c, state, cur, before, n):
    """(J, held, ok) of the live literals at `cur`, a run pair a literal its h either side: the
    residuals' Jacobian and her form's (on a flat floor) over the literals `ok` measured - none
    felled her -; `before`'s floors set by two runs, each literal a unit of the text's last place
    off either way."""
    unit = 10.0 ** -c['digits']
    pairs = runs(c['root'], [{**state, c['span'][j]: cur[j] + s * c['h'][j]} for j in range(len(cur)) for s in (1.0, -1.0)]
                 + [{**state, **{c['span'][j]: cur[j] + s * unit for j in range(len(cur))}} for s in (1.0, -1.0)])
    pairs, floor = pairs[:-2], [f for f in pairs[-2:] if 'r' in f]
    before['floor'] = max([abs(norm(f) - norm(before)) for f in floor] or [math.inf])
    offs = [off(f) for f in floor]
    before['off_floor'] = {name: max([abs(o[name] - v) for o in offs if math.isfinite(o[name] - v)] or [math.inf])
                           for name, v in off(before).items()}
    ok = [j for j in range(len(cur)) if all('r' in p and p['fell'] is None for p in pairs[2 * j:2 * j + 2])]
    d = lambda j, of: (of(pairs[2 * j]) - of(pairs[2 * j + 1])) / (2.0 * c['h'][j])
    J = np.array([[d(j, lambda p: p['r'][k]) for j in ok] for k in range(len(c['gr']))]).reshape(len(c['gr']), len(ok))
    held = [[d(j, lambda p: off(p)[name]) for j in ok] for name, _least, _most in looks.FORM] if before['scene'] == 'flat' else []
    print('\nstep %d: %d of %d literals measured (%s), its floor %.4f' % (
        n + 1, len(ok), len(cur), ', '.join('%.0f s' % p.get('wall', math.nan) for p in pairs[:2]), before['floor']), flush=True)
    return J, held, ok


def stepped(c, state, cur, before, measured):
    """(the state a kept step leaves, its run) - or (`state`, None) where none is kept -: the
    text's step on `measured` (J, held, ok), its trust region halved while a step is not kept,
    till the gain its own model predicts is under the floor."""
    from machine import tune
    J, held, ok = measured
    r, shrink = np.array(before['r']), 1.0
    while ok:
        d = tune.step(J, r, c['scales'][ok], held, shrink)
        gain = norm(before) - float(np.linalg.norm(r + J @ d))
        if gain <= before['floor']:
            print('  at %.3g of its trust region its model gains %.4f, under its floor: none kept' % (shrink, gain), flush=True)
            break
        new = cur.copy()
        new[ok] += d
        new = np.array([float(tune.number(v, c['digits'])) for v in new])
        trial = {**state, **{c['span'][j]: new[j] for j in ok}}
        after = runs(c['root'], [trial])[0]
        why = judged(before, after)
        print('  at %.3g of its trust region: %s' % (shrink, 'kept' if why is None else 'not kept, ' + why), flush=True)
        print(run_report(c['gr'], before, after, c['table'], c['live'], cur, new), flush=True)
        if why is None:
            return trial, after
        shrink /= 2.0
    return state, None


def main(argv=None):
    """A root's walk and reach, its steps judged, or one run as JSON (--one)."""
    p = argparse.ArgumentParser(description='The autotuner on her loop.')
    p.add_argument('root', help='a gait of the suite; a stunt she plays with --reach')
    p.add_argument('--one', action='store_true', help="one run as JSON, a relay job's")
    p.add_argument('--set', action='append', default=[], help='PATH:START:END=VALUE, a literal (--one)')
    p.add_argument('--reach', action='store_true', help='what each literal moves of her, nothing run')
    p.add_argument('--steps', type=int, default=1)
    p.add_argument('--write', action='store_true', help='the kept texts written where the norm converged')
    args = p.parse_args(argv)
    from machine import tune
    if args.one:
        print(json.dumps(ran(args.root, dict(_said(s) for s in args.set)), default=float))
        return 0
    if args.reach:
        gr, tx = tune.grammar(), tune.texts()
        table = tune.walked(args.root, gr, tx)
        print(walk_report(args.root, table, *tune.reach(args.root, table, tx, suite._her())))
        return 0
    state, converged = tuned(args.root, args.steps)
    lits = [tune.Literal(p_, a, z, None, None, None, None, v) for (p_, a, z), v in state.items()]
    texts = tune.spliced(tune.texts(), lits, [lit.value for lit in lits],
                         tune.knobs()[('sync-mutated-packages-to-disk', 'float-precision')])
    print('\n%d literals kept moved; %s' % (len(state), 'converged' if converged else 'not converged'))
    if args.write and state and converged:
        print('written: ' + ', '.join(os.path.basename(path) for path in tune.written(texts)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
