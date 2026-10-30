#!/usr/bin/env python3
"""The autotuner on her loop: a root of the suite run, its six classes measured, a step judged.

    python tools/sim/tune.py patch-sneaker-runway             # its reach, its Jacobian, a step judged
    python tools/sim/tune.py sit_chair --steps 3              # a stunt she plays; each from the last kept
    python tools/sim/tune.py compile-omnidirectional-suite    # a schedule to its first refusal
    python tools/sim/tune.py patch-sneaker-runway --write     # converged and held elsewhere: written
    python tools/sim/tune.py sit_chair --reach                # what each literal moves, nothing run
    python tools/sim/tune.py ROOT --one --set=PATH:START:END=V  # one run as JSON, a relay job's

A run is the telemetry's (`telemetry.ran`): a gait, a stunt or a schedule, the text's six classes
sampled over whatever bears her. Its Jacobian is a run pair a live literal (`tune.reach`), its
scale times the text's trust region either side; the step the text's (`tune.step`), her form held
on a gait's flat floor, kept unless VERDICTS say otherwise, its trust region halved till its
model's gain is under the floor. Written (--write) converged by the text's tol and held on every
other root a moved literal reaches (`elsewhere`).
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np  # behind the OpenBLAS cap

from tools.sim import looks, suite, telemetry


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


def _formed(run):
    """Whether her form is a run's condition: her walk's, on a flat floor (`telemetry.ran`'s word)."""
    return run['formed']


def _worse(before, after):
    """The measures of her form `after` moves further outside its band than `before`'s floor, where
    it is a condition, never a price."""
    b, a = off(before), off(after)
    return sorted(name for name in a if a[name] - b[name] > before['off_floor'][name]) if _formed(after) else []


#: Why a step is not kept: (its test on the runs before and after it, what it says), the norm's
#: last. A floor is how far a measure moves with each live literal a unit of the text's last place
#: off before's.
VERDICTS = ((lambda b, a: 'error' in a, lambda b, a: 'not run: %s' % a.get('error', '')[-160:]),
            (lambda b, a: a['fell'] is not None and b['fell'] is None, lambda b, a: 'she fell at %.2f s' % a['fell']),
            (lambda b, a: bool(b.get('met')) and not a.get('met'), lambda b, a: 'its goal no longer met'),
            (lambda b, a: bool(_worse(b, a)), lambda b, a: 'her form worse: ' + ', '.join(
                '%s %.3f -> %.3f' % (n, b['form'][n], a['form'][n]) for n in _worse(b, a))),
            (lambda b, a: norm(b) - norm(a) <= b['floor'],
             lambda b, a: '|r| %.4f -> %.4f' % (norm(b), norm(a))))


def judged(before, after, verdicts=VERDICTS):
    """Why the step from run `before` to run `after` is not kept (`verdicts`), or None."""
    return next((why(before, after) for test, why in verdicts if test(before, after)), None)


def runs(items):
    """[`telemetry.ran`'s dict, or {error}] of each (root, state {span: value}) of `items`, side by side on
    the relay's batons."""
    from machine.texts import tune
    from tools.dev import focus
    jobs = [focus.Job('tune %d' % k, [sys.executable, '-X', 'utf8', os.path.abspath(__file__), root, '--one'] + [
        '--set=%s:%d:%d=%s' % (p, a, z, tune.number(v)) for (p, a, z), v in said.items()], 1.5, 3600.0, None)
            for k, (root, said) in enumerate(items)]
    out: list = [{'error': 'not run'}] * len(items)
    for job, text, code, _seconds in focus.relay(jobs):
        found = [ln for ln in (text or '').splitlines() if ln.startswith('{')]
        out[int(job.name.split()[-1])] = json.loads(found[-1]) if found else {'error': (text or '')[-300:], 'code': code}
    return out


def walk_report(root, table, cols, refused):
    """The binding table's lines: * a live literal and how many voices it moves, ! one refused and
    why, the rest inert."""
    from machine.texts import ortho
    lines = ['%s: %d literals walked, %d live, %d refused, %d inert' % (
        root, len(table), len(cols), len(refused), len(table) - len(cols) - len(refused))]
    for i, (lit, cls) in enumerate(table):
        mark = '*' if i in cols else '!' if i in refused else ' '
        where = '%s:%d' % (os.path.basename(lit.path), ortho.source(lit.path).count('\n', 0, lit.start) + 1)
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
        m, nan = run.get('measures', {}), [math.nan, math.nan]
        lines.append('  %s: pivot median %.1f mm, under its bound %.0f %%; load %.3f-%.3f; cone %.3f; slip %.0f mm; '
                     'feet >= %.1f mm; tank bled %.2f J; %.0f J/m%s; form broken: %s' % (
                         label, m.get('pivot', nan)[0], 1e2 * m.get('pivot', nan)[1], m.get('load', math.nan),
                         m.get('peak', math.nan), m.get('cone', math.nan), m.get('slip', math.nan), m.get('gap', math.nan),
                         m.get('bleed', math.nan), m.get('energy', math.nan),
                         '' if 'met' not in run else '; its goal %s' % ('met' if run['met'] else 'not met'),
                         ', '.join(run.get('broken', [])) or '-'))
    lines += ['  %-28s %-24s %-26s %9g -> %g' % (table[i][0].symbol[:28], table[i][0].node[:24], table[i][0].key, cur[j], new[j])
              for j, i in enumerate(live)]
    return '\n'.join(lines)


def tuned(root, steps, ran=None):
    """(the kept state {span: value}, whether the norm converged) of `root`'s live literals stepped
    up to `steps` times from the texts as they are, each step printed; its runs `ran`'s (`runs`'s
    signature: the relay's where none, windows on her running loop at runtime)."""
    from machine.texts import tune, voicing
    gr, tx, fr = tune.grammar(), tune.texts(), suite._her()
    table = tune.walked(root, gr, tx)
    cols, refused = tune.reach(root, table, tx, fr)
    print(walk_report(root, table, cols, refused), flush=True)
    (got, gym), live, k = tune.evaluated(tx), sorted(cols), tune.knobs()
    row = voicing.walk(fr, gym)
    c = {'root': root, 'gr': gr, 'table': table, 'live': live, 'runs': ran or runs, 'digits': k[('sync-mutated-packages-to-disk', 'float-precision')],
         'scales': np.array([tune.scale(table[i][0], got, row) for i in live]),
         'span': [(table[i][0].path, table[i][0].start, table[i][0].end) for i in live]}
    c['h'] = k[('compute-adjoint-ast-deltas', 'trust-region-scale')] * c['scales']
    state, converged = {}, False
    before = c['runs']([(root, state)])[0]
    print('\nas the texts are: |r| %.4f (%s)' % (norm(before), ', '.join('%.4f' % x for x in before['r'])) if 'r' in before
          else 'not run: %s' % before.get('error', '')[-300:], flush=True)
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


def floors(before, floor):
    """`before`'s floors set from its `floor` runs: how far its norm and each form measure's
    distance outside its band moved in them."""
    floor = [f for f in floor if 'r' in f]
    before['floor'] = max([abs(norm(f) - norm(before)) for f in floor] or [math.inf])
    offs = [off(f) for f in floor]
    before['off_floor'] = {name: max([abs(o[name] - v) for o in offs if math.isfinite(o[name] - v)] or [math.inf])
                           for name, v in off(before).items()}


def measured_at(c, state, cur, before, n):
    """(J, held, ok): the residuals' and her form's Jacobians over the live literals `ok` that felled
    her neither way, a run pair a literal its h either side - a form measure's slope where its pair
    moved it past its floor, else 0 -; `before`'s floors set (`floors`), each literal a unit of the
    text's last place off either way."""
    unit = 10.0 ** -c['digits']
    pairs = c['runs']([(c['root'], {**state, c['span'][j]: cur[j] + s * c['h'][j]}) for j in range(len(cur)) for s in (1.0, -1.0)]
                 + [(c['root'], {**state, **{c['span'][j]: cur[j] + s * unit for j in range(len(cur))}}) for s in (1.0, -1.0)])
    pairs, floor = pairs[:-2], pairs[-2:]
    floors(before, floor)
    ok = [j for j in range(len(cur)) if all('r' in p and p['fell'] is None for p in pairs[2 * j:2 * j + 2])]
    d = lambda j, of: (of(pairs[2 * j]) - of(pairs[2 * j + 1])) / (2.0 * c['h'][j])
    J = np.array([[d(j, lambda p: p['r'][k]) for j in ok] for k in range(len(c['gr']))]).reshape(len(c['gr']), len(ok))
    past = lambda j, name: abs(off(pairs[2 * j])[name] - off(pairs[2 * j + 1])[name]) > before['off_floor'][name]
    held = [[d(j, lambda p: off(p)[name]) if past(j, name) else 0.0 for j in ok]
            for name, _least, _most in looks.FORM] if _formed(before) else []
    print('\nstep %d: %d of %d literals measured (%s), its floor %.4f' % (
        n + 1, len(ok), len(cur), ', '.join('%.0f s' % p.get('wall', math.nan) for p in pairs[:2]), before['floor']), flush=True)
    return J, held, ok


def stepped(c, state, cur, before, measured):
    """(the state a kept step leaves, its run) - or (`state`, None) where none is kept -: the
    text's step on `measured` (J, held, ok), its trust region halved while a step is not kept,
    till the gain its own model predicts is under the floor, the step under the text's last place
    (its deadband) or her run the same bit for bit (a literal her law rounds: the sit's ms)."""
    from machine.texts import tune
    J, held, ok = measured
    r, shrink = np.array(before['r']), 1.0
    while ok:
        d = tune.step(J, r, c['scales'][ok], held, shrink, [before['off_floor'][name] for name, _l, _m in looks.FORM])
        gain = norm(before) - float(np.linalg.norm(r + J @ d))
        if gain <= before['floor']:
            print('  at %.3g of its trust region its model gains %.4f, under its floor: none kept' % (shrink, gain), flush=True)
            break
        new = cur.copy()
        new[ok] += d
        new = np.array([float(tune.number(v, c['digits'])) for v in new])
        if np.array_equal(new, cur):
            print("  at %.3g of its trust region it moves nothing at the text's last place: none kept" % shrink, flush=True)
            break
        trial = {**state, **{c['span'][j]: new[j] for j in ok}}
        after = c['runs']([(c['root'], trial)])[0]
        if after.get('r') == before['r']:
            print('  at %.3g of its trust region her run is the one before it, bit for bit: none kept' % shrink, flush=True)
            break
        why = judged(before, after)
        print('  at %.3g of its trust region: %s' % (shrink, 'kept' if why is None else 'not kept, ' + why), flush=True)
        print(run_report(c['gr'], before, after, c['table'], c['live'], cur, new), flush=True)
        if why is None:
            return trial, after
        shrink /= 2.0
    return state, None


def _voices(name, played, fr):
    """`name`'s voices as `played` (the suite, her gym) has them, or the refusal's words."""
    from machine.texts import tune
    from machine.errors import MachineError
    got, gym = played
    try:
        return tune.voices(name, got, fr, gym)
    except (MachineError, SyntaxError, ValueError) as error:
        return str(error)


def moved(root, state):
    """The other roots `state` moves, by evaluation: each gait and schedule of the suite, each stunt
    she plays, each behaviour of her gym, whose voices differ as her texts are and with `state` - a
    gait on her walk moved by her walk's numbers too."""
    from machine.texts import maneuver, tune
    fr, tx = suite._her(), tune.texts()
    was, now = tune.evaluated(tx), tune.evaluated(tune.given(state, tx))
    names = dict.fromkeys(list(was[0]) + list(maneuver.PLAYED) + [e[0] for e in was[1]] + list(tune.WHOLE))
    return [n for n in names if n != root and _voices(n, was, fr) != _voices(n, now, fr)]


def elsewhere(root, state):
    """[(root, why)] of each other root `state` moves (`moved`) that it breaks: run as the texts are
    and with `state`, judged as a step is but for the norm; one not run as they are, named; the
    bench's scenarios too (`benched`)."""
    from machine.texts import tune
    unit = 10.0 ** -tune.knobs()[('sync-mutated-packages-to-disk', 'float-precision')]
    others = moved(root, state)
    got_runs = runs([(other, said) for other in others for said in (
        {}, state, {span: v + unit for span, v in state.items()}, {span: v - unit for span, v in state.items()})])
    out, unjudged = [], []
    for k, other in enumerate(others):
        before, after = got_runs[4 * k], got_runs[4 * k + 1]
        if 'r' not in before:
            unjudged.append(other)
            continue
        floors(before, got_runs[4 * k + 2:4 * k + 4])
        why = judged(before, after, VERDICTS[:-1])
        out += [(other, why)] * (why is not None)
    print('held elsewhere: %d roots run, %s; not run as the texts are, not judged: %s' % (
        len(others), '; '.join('%s: %s' % o for o in out) or 'none broken', ', '.join(unjudged) or 'none'), flush=True)
    return out + benched(state)


def benched(state):
    """[(scenario@wobble, why)] of the bench's scenarios and shoves (`bench.jobs`) met as her texts
    are and not with `state`: a step on her walk held her suite's roots and broke the wall."""
    from machine.texts import tune
    from tools.sim import bench
    host = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    sets = ['%s:%d:%d=%s' % (p, a, z, tune.number(v)) for (p, a, z), v in state.items()]
    pairs = [(j, w._replace(name='with ' + w.name)) for j, w in zip(bench.jobs(host), bench.jobs(host, sets))
             if not j.name.startswith('replay:')]
    got = bench.ran([job for pair in pairs for job in pair])
    out = [(j.name, 'not met') for j, w in pairs if bench.met(j.name, got[j.name]) and not bench.met(j.name, got[w.name])]
    print('held on the bench: %d of %d met as the texts are, %s' % (
        sum(bench.met(j.name, got[j.name]) for j, _w in pairs), len(pairs),
        ', '.join(name for name, _why in out) + ' not met with it' if out else 'all still met'), flush=True)
    return out


def main(argv=None):
    """A root's walk and reach, its steps judged, or one run as JSON (--one)."""
    p = argparse.ArgumentParser(description='The autotuner on her loop.')
    p.add_argument('root', help='a gait, a schedule or a stunt she plays')
    p.add_argument('--one', action='store_true', help="one run as JSON, a relay job's")
    p.add_argument('--set', action='append', default=[], help='PATH:START:END=VALUE, a literal (--one)')
    p.add_argument('--reach', action='store_true', help='what each literal moves of her, nothing run')
    p.add_argument('--steps', type=int, default=1)
    p.add_argument('--write', action='store_true', help='the kept texts written, converged and held elsewhere')
    args = p.parse_args(argv)
    from machine.texts import tune
    if args.one:
        print(json.dumps(telemetry.ran(args.root, dict(tune.setting(s) for s in args.set)), default=float))
        return 0
    if args.reach:
        gr, tx = tune.grammar(), tune.texts()
        table = tune.walked(args.root, gr, tx)
        print(walk_report(args.root, table, *tune.reach(args.root, table, tx, suite._her())))
        return 0
    kept(args.root, *tuned(args.root, args.steps), args.write)
    return 0


def kept(root, state, converged, write):
    """The kept `state` said; written (`write`) where it converged, holds on every other root it
    moves (`elsewhere`) and on the suites its files reach (`gate.held`) - refused, half its move
    from her texts as they are, as a step's trust region halves, till it rounds to nothing."""
    from machine.texts import tune
    from tools.sim import gate
    digits, tx = tune.knobs()[('sync-mutated-packages-to-disk', 'float-precision')], tune.texts()
    print('\n%d literals kept moved; %s' % (len(state), 'converged' if converged else 'not converged'))
    was, share = {span: float(tx[span[0]][span[1]:span[2]]) for span in state}, 1.0
    while state and converged and write:
        said = {span: float(tune.number(was[span] + share * (v - was[span]), digits)) for span, v in state.items()}
        if said == was:
            print('its move rounds to nothing: none written', flush=True)
            return
        texts = tune.given(said, digits=digits)
        if not elsewhere(root, said) and not gate.held(texts):
            print('written at %g of its move: %s' % (share, ', '.join(os.path.basename(p) for p in tune.written(texts))))
            return
        share /= 2.0


if __name__ == '__main__':
    sys.exit(main())
