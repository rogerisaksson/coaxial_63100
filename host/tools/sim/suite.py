#!/usr/bin/env python3
"""The ortho-DSP suite's patches and the takes' modifiers played on her loop and measured.

Measured by the converter that makes a modifier of a take (`tools/sim/identify.py`): what she
was told beside what she did, against her walk.

    python tools/sim/suite.py                       # the suite's patches, each on its scene, side by side
    python tools/sim/suite.py TAKE.fbx [..]         # takes converted against her walk and played
    python tools/sim/suite.py --schedule            # the suite's 32 bars to its first refusal
    rows, fell = suite.played(row, told, scene)     # her look's rows from SETTLE_S, her fall's s

A patch is voiced on her walk by `machine.dsp`; told her twist and grip with her walk from 1 s,
its row laid LAY_S in, after her walk's own (laid in their tick the two add, her rhythm's gain 2).
Her walk itself is the reference, recorded the same way and identified once a run (REFERENCE): a
take's modifier is its change off her walk, and her run's change off it is how much of the take
she played (SHARE). A patch's world is its stages' (WORLDS); a schedule's morph waits for her
lateral voice's crossing and the double support's lock (`machine.patch`). Several items run as
jobs side by side on the relay's batons (`tools.dev.focus`).
"""
import argparse
import json
import math
import os
import sys

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, HOST)

from coaxial.model.blocks import numpy as np  # behind the OpenBLAS cap

#: Her walk's settling, s, before rows are kept; a patch's row laid LAY_S in; a run's seconds; a
#: row kept every ROW_S.
SETTLE_S, LAY_S, SECONDS, ROW_S = 4.0, 3.0, 20.0, 0.01
#: Her walk identified, a run's: the reference every take and patch is measured against.
REFERENCE = os.path.join(os.path.dirname(HOST), 'build', 'dsp', 'her_walk.json')
#: The patches the converter writes.
PATCHES = os.path.join(os.path.dirname(HOST), 'build', 'patches')
#: A patch's world by its stages: (stage, its key whose sign picks, the scene up, the scene down).
WORLDS = (('modulator-discrete-stairs', 'riser-z-mm', 'stairs', 'stairs_down'),
          ('modulator-continuous-slope', 'grade-angle-rad', 'slope', 'slope_down'),
          ('surface-low-mu-ice', None, 'ice', 'ice'), ('surface-granular-dissipative', None, 'snow', 'snow'),
          ('surface-stochastic-uneven', None, 'rug', 'rug'))
#: What she is measured by: (section, measure) of `identify.identified`.
MEASURES = (('clock', 'bpm'), ('coronal', 'track'), ('coronal', 'lfo'), ('sidechain', 'depth'),
            ('axial', 'thorax'), ('axial', 'arm'), ('axial', 'lag'), ('path', 'speed'), ('path', 'stride'),
            ('path', 'clearance'), ('path', 'height'), ('clock', 'vault-mm'))


def scene_of(g):
    """The scene a gait's stages lay her on, the last that has one: 'flat' where none."""
    found = 'flat'
    for stage, keys in g.get('_trace', []):
        for name, key, up, down in WORLDS:
            if stage == name:
                found = up if key is None or keys[key] >= 0.0 else down
    return found


def played(row, told, scene, seconds=SECONDS, laid=None):
    """(her look's rows from SETTLE_S on, the s she fell at or None): her walk told `told` from
    1 s - its twist and grip -, `row` ({slot: value}) laid LAY_S in; `laid(s)` each pass after,
    a schedule's hand."""
    from machine import gait, ortho, patch, worker
    from tools.sim import scenarios
    s = worker.session()
    scenarios.placed(s, scene)
    g = s['g']
    while g['t'] < 1.0:
        worker.stepped(s)
    p = [math.nan] * ortho.P
    p[ortho.GAIN] = 1.0
    for slot, v in row.items():
        p[slot] = float(v)
    command = {'twist': tuple(float(x) for x in told['twist']),
               'entries': [('patch', ortho.ROW.pack(patch.ROW_ID, int(1e3 * (LAY_S - 1.0)), int(1e3 * gait.BLEND_S), 0, *p),
                            ('patch',))] if row else []}
    command.update({'grip': float(told['grip'])} if 'grip' in told else {})
    worker.apply(s, command)
    rows, last = [], -math.inf
    while g['t'] < seconds and s['fallen_at'] is None:
        worker.stepped(s)
        if laid is not None:
            laid(s)
        if g['t'] >= SETTLE_S and g['t'] - last >= ROW_S - 1e-9:
            last = g['t']
            rows.append(scenarios._row(g, g['world']))
    return rows, s['fallen_at']


def measured(rows):
    """{section: {measure: value}} of her rows by the converter, the scalars alone."""
    from tools.sim import identify
    from tools.sim.fbx import joints
    got = identify.identified(identify.soa(*joints(rows)))
    return {k: {m: (bool(v) if isinstance(v, (bool, np.bool_)) else v if isinstance(v, str) else float(v))
                for m, v in sec.items() if isinstance(v, (float, int, str, bool, np.floating, np.bool_))}
            for k, sec in got.items()}


def reference():
    """Her walk identified (REFERENCE), recorded and kept on its first call of a run."""
    if not os.path.exists(REFERENCE):
        from tools.sim import scenarios
        rows, fell = played({}, {'twist': scenarios.WALK}, 'flat')
        os.makedirs(os.path.dirname(REFERENCE), exist_ok=True)
        with open(REFERENCE, 'w', encoding='utf-8') as f:
            json.dump(dict(measured(rows), fell=fell), f)
    with open(REFERENCE, encoding='utf-8') as f:
        return json.load(f)


def _her():
    """Her statics standing (`modal.frame`)."""
    from machine import worker
    return worker.session()['g']['fr']


def line(item):
    """{item, told, her, ...} of one item: a suite patch by name or a take's FBX, voiced, played,
    measured; refused by name where she cannot."""
    from machine import dsp, voicing
    from machine.errors import MachineError
    got = dsp.read(dsp.SUITE)
    out = {'item': item}
    if item.lower().endswith('.fbx'):
        from tools.sim import identify
        from tools.sim.fbx import take
        ref = reference()
        name = 'take-' + ''.join(ch if ch.isalnum() else '-' for ch in os.path.splitext(os.path.basename(item))[0].lower())
        try:
            told_by = identify.identified(identify.soa(*take(item)))
            text = identify.emitted(name, told_by, _ref_full(ref))
        except (ValueError, KeyError) as error:
            return dict(out, refused='not identified: %s' % error)
        out['take'] = {k: {m: v for m, v in sec.items() if isinstance(v, (float, int, str))} for k, sec in told_by.items()}
        os.makedirs(PATCHES, exist_ok=True)
        path = os.path.join(PATCHES, name + '.lisp')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)
        got = dsp.read(dsp.SUITE + [path])
        g = dsp.value(['->', 'nominal-biped-manifold', [name]], got)
    else:
        g = got[item]
    fr = _her()
    try:
        row, told = voicing.voiced(g, got['nominal-biped-manifold'], fr)
    except MachineError as error:
        return dict(out, refused=str(error))
    scene = scene_of(g)
    rows, fell = played(row, told, scene)
    out.update(scene=scene, row={str(k): v for k, v in row.items()}, twist=list(told['twist']), asked=told.get('asked'),
               fell=fell)
    try:
        out['her'] = measured(rows)
    except (ValueError, KeyError, IndexError) as error:
        out['her'] = None
        out['unmeasured'] = str(error)
    return out


def _ref_full(ref):
    """The reference as `identify.emitted` reads it: its sections' scalars."""
    return {k: v for k, v in ref.items() if isinstance(v, dict)}


def share(take, her, ref):
    """How much of the take's change off her walk she played: 1 all, 0 none, nan where it has none."""
    return (her - ref) / (take - ref) if take != ref else math.nan


def table(lines, ref):
    """The lines side by side: a column a measure, the take's (or what the patch asked) over hers."""
    head = '%-34s %6s %6s' % ('item', 'scene', 'fell') + ''.join('%15s' % m for _s, m in MEASURES)
    out = [head, '%-34s %6s %6s' % ('her walk', 'flat', '-') + ''.join(
        '%15.4g' % ref[s][m] for s, m in MEASURES)]
    for ln in lines:
        name = os.path.basename(ln['item'])[:34]
        if 'refused' in ln:
            out.append('%-34s refused: %s' % (name, ln['refused']))
            continue
        cells = ''
        for s, m in MEASURES:
            her = (ln.get('her') or {}).get(s, {}).get(m, math.nan)
            take = ln.get('take', {}).get(s, {}).get(m, math.nan)
            cells += '%15s' % ('%.4g/%.4g' % (take, her) if take == take else '%.4g' % her)
        out.append('%-34s %6s %6s' % (name, ln.get('scene', ''), '%.1f' % ln['fell'] if ln.get('fell') else '-') + cells)
    return '\n'.join(out)


def schedule():
    """The suite's schedule played to its first refusal on a flat floor - its worlds named, not
    laid -: each patch's row at its bar, each morph's rows from the lock past its crossing.
    [(from bar, to bar, what, laid at s or the refusal)] and her rows."""
    from machine import dsp, patch, voicing, worker
    from machine.errors import MachineError
    got = dsp.read(dsp.SUITE)
    dry, fr = got['nominal-biped-manifold'], _her()
    plan, at, steps = [], LAY_S, []
    for b0, b1, item in dsp.schedule(got['compile-omnidirectional-suite']):
        try:
            if isinstance(item, dsp.Morph):
                wire, span = voicing.morphed(item, dry, fr)
                steps.append(('morph', b0, b1, wire, span))
            else:
                row, told = voicing.voiced(item, dry, fr)
                steps.append(('patch', b0, b1, row, told))
        except MachineError as error:
            plan.append((b0, b1, 'refused', str(error)))
            break
    state = {'k': 0, 'next': None, 'due': at}

    def laid(s):
        from machine import gait, latent, ortho
        g = s['g']
        if state['k'] >= len(steps) or g['t'] < state['due']:
            return
        kind, b0, b1, what, extra = steps[state['k']]
        bar = 1.0 / float(g['st'][latent.NOW][latent.HZ])
        if kind == 'patch':
            p = [math.nan] * ortho.P
            p[ortho.GAIN] = 1.0
            for slot, v in what.items():
                p[slot] = float(v)
            late = int(1e3 * bar / patch.GRID)          # after the rows a twist may say, never in their tick
            worker.apply(s, {'twist': tuple(float(x) for x in extra['twist']),
                             'entries': [('patch', ortho.ROW.pack(patch.ROW_ID, late, int(1e3 * gait.BLEND_S), 0, *p),
                                          ('patch',))]})
            plan.append((b0, b1, 'patch', round(g['t'], 3)))
            state.update(k=state['k'] + 1, due=g['t'] + (b1 - b0) * bar, next=None)
            return
        if state['next'] is None:
            state['next'] = patch.crossing(g['st'], g['t'])
        if g['t'] < state['next']:
            return
        if patch.locked(g['y'], g['fr'], g['a']):
            worker.apply(s, {'entries': [('morph', what, ('morph',))]})
            plan.append((b0, b1, 'morph', round(g['t'], 3)))
            state.update(k=state['k'] + 1, due=g['t'] + extra, next=None)
        elif g['t'] > state['next'] + patch.window(g['st']):
            state['next'] = patch.crossing(g['st'], g['t'] + 1e-3)

    from tools.sim import scenarios
    total = LAY_S + sum((st[2] - st[1]) * 1.4 for st in steps) + 4.0
    rows, fell = played({}, {'twist': scenarios.WALK}, 'flat', total, laid)
    return plan, rows, fell


def main(argv=None):
    """Items side by side on the relay, or one item's line as JSON (--one)."""
    p = argparse.ArgumentParser(description='The ortho-DSP suite and the takes played on her loop.')
    p.add_argument('items', nargs='*')
    p.add_argument('--one', action='store_true', help="one item's line as JSON, a relay job's")
    p.add_argument('--schedule', action='store_true')
    args = p.parse_args(argv)
    if args.schedule:
        plan, rows, fell = schedule()
        for step in plan:
            print(step)
        print('fell', fell, 'rows', len(rows))
        return 0
    if args.one:
        print(json.dumps(line(args.items[0]), default=float))
        return 0
    from machine import dsp
    items = args.items or [k for k, v in dsp.read(dsp.SUITE).items() if k.startswith('patch-') and isinstance(v, dict)]
    reference()
    from tools.dev import focus
    jobs = [focus.Job(os.path.basename(item), [sys.executable, '-X', 'utf8', os.path.abspath(__file__), item, '--one'],
                      1.5, 3600.0, None) for item in items]
    lines = []
    for _job, text, _code, _seconds in focus.relay(jobs):
        found = [ln for ln in (text or '').splitlines() if ln.startswith('{')]
        lines.append(json.loads(found[-1]) if found else {'item': _job.name, 'refused': (text or '')[-300:]})
    print(table(sorted(lines, key=lambda ln: ln['item']), reference()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
