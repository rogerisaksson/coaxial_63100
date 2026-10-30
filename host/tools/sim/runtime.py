#!/usr/bin/env python3
"""The autotuner at runtime: her walk tuned while she walks, her loop in a process of its own.

    python tools/sim/runtime.py                      # her walk: windows, a Jacobian, a step judged
    python tools/sim/runtime.py --steps 3 --write    # converged and held elsewhere: written

Her loop (`loop`) walks linked (`tap.attached`), never waiting on this core: it drains her raw
passes and looks and publishes tables (`tap.table`) of her walk's row compiled from her texts with
a state's literals spliced (`row`), laid at her barrier. A run (`window`) is WINDOW_S of her
passes from SETTLE_S past a table's laying: its six residuals and her form as offline
(`telemetry.scored`); the Jacobian, the step, its verdicts and the write offline's
(`tools.sim.tune.tuned`), its runs these windows one after another.
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.sim import suite, telemetry, tune as tuner

#: Her walk's root; a window's settling past a table's laying and its length, s - whole strides
#: of her walk and its blend's transient past (`gait.BLEND_S`); her rings' seconds.
ROOT, SETTLE_S, WINDOW_S, RING_S = 'walk', 2.0, 6.0, 8.0
#: How long this core sleeps between drains, s; how long a window may take on the wall, s.
NAP_S, WALL_S = 0.005, 600.0


def loop(name):
    """Her loop, linked to `name`: her walk told, each new epoch's too, till the tuner says halt."""
    from machine.process import worker
    from tools.sim import scenarios
    s = worker.session()
    worker.apply(s, {'link': name})
    link, epoch = s['links'][0], None
    while not link.word('halt'):
        if s['epoch'] != epoch:
            worker.apply(s, {'twist': scenarios.WALK})
            epoch = s['epoch']
        worker.stepped(s)
    for found in s['links']:
        found.closed()


def row(c, state):
    """Her walk's row in its reached slots (`c['slots']`) compiled from her texts with `state`'s
    literals ({span: value}) spliced: her gym re-read, her walk voiced (`voicing.walk`)."""
    from machine.texts import tune, voicing
    walked = voicing.walk(c['fr'], tune.evaluated(tune.given(state, c['texts']))[1])
    return {k: float(walked[k]) for k in c['slots']}


def window(c, seq):
    """Her run of the table `seq` (`telemetry.scored`'s dict): her passes from SETTLE_S past its
    laying for WINDOW_S, or where she fell first, her fall."""
    from machine.law import gait
    from machine.process import tap
    raws, looks, began, link = [], [], time.time(), c['link']
    while True:
        if not c['her'].is_alive() or time.time() - began > WALL_S:
            return {'error': 'her loop %s' % ('gone' if not c['her'].is_alive() else 'silent')}
        got, seen = link.drained(0), link.drained(1)
        raws += [x for x in got if x[tap.HE['laid']] == seq]
        looks += list(seen)
        if not raws:
            time.sleep(NAP_S)
            continue
        fell = [x[tap.HE['fell']] for x in raws if math.isfinite(x[tap.HE['fell']])]
        start = raws[0][tap.HE['t']] + gait.BLEND_S + SETTLE_S
        if fell or raws[-1][tap.HE['t']] >= start + WINDOW_S:
            break
        time.sleep(NAP_S)
    kept = [x for x in raws if start <= x[tap.HE['t']] < start + WINDOW_S]
    look = [x for x in looks if start <= x[0] < start + WINDOW_S]
    if fell or not kept:
        return {'r': [math.inf] * len(c['gr']), 'fell': fell[0] if fell else raws[-1][tap.HE['t']], 'formed': True,
                'form': {}, 'wall': round(time.time() - began, 1)}
    out = telemetry.scored(c['got'], tap.measured(tap.cooked(kept), c['weight']), telemetry.rows(look), {'formed': True})
    return dict(out, fell=None, kind='behaviour', scene='flat', wall=round(time.time() - began, 1))


def walking(c):
    """Drained till she walks: her epoch past the suite's settling (`suite.SETTLE_S`), standing - a
    table laid at rest turns her rhythm on under her stand."""
    from machine.process import tap
    began = time.time()
    while time.time() - began < WALL_S and c['her'].is_alive():
        got = c['link'].drained(0)
        c['link'].drained(1)
        if len(got) and got[-1][tap.HE['t']] >= suite.SETTLE_S and not math.isfinite(got[-1][tap.HE['fell']]):
            return
        time.sleep(NAP_S)


def runs(c):
    """The offline tuner's runs (`tools.sim.tune.runs`) as windows on her loop, one after another:
    each state's table published once she walks (`walking`), laid, its window measured."""
    from machine.law import gait
    from machine.process import tap

    def ran(items):
        out = []
        for _root, state in items:
            walking(c)
            c['seq'] += 1
            while not c['link'].published(tap.table(row(c, state), gait.BLEND_S, gait.ROW_ID)):
                time.sleep(NAP_S)
            out.append(window(c, c['seq']))
        return out
    return ran


def main(argv=None):
    """Her loop started linked, her walk tuned on it, the kept state written where asked."""
    import multiprocessing
    from machine.law import modal
    from machine.process import tap
    from machine.texts import tune
    p = argparse.ArgumentParser(description='The autotuner at runtime, on her walk.')
    p.add_argument('--steps', type=int, default=1)
    p.add_argument('--write', action='store_true', help='the kept texts written, converged and held elsewhere')
    args = p.parse_args(argv)
    gr, tx, fr = tune.grammar(), tune.texts(), suite._her()
    table = tune.walked(ROOT, gr, tx)
    cols, _refused = tune.reach(ROOT, table, tx, fr)
    got = tune.evaluated(tx)[0]
    c = {'gr': gr, 'texts': tx, 'fr': fr, 'got': got, 'weight': float(fr[modal.WEIGHT]), 'seq': 0,
         'slots': sorted({key[1] for col in cols.values() for key in col if key[0] == 'slot'}),
         'link': tap.made(RING_S)}
    c['her'] = multiprocessing.get_context('spawn').Process(target=loop, args=(c['link'].name,), daemon=True)
    c['her'].start()
    try:
        state, converged = tuner.tuned(ROOT, args.steps, runs(c))
    finally:
        c['link'].said('halt', 1)
        c['her'].join(10.0)
        c['link'].closed()
    tuner.kept(ROOT, state, converged, args.write)
    return 0


if __name__ == '__main__':
    sys.exit(main())
