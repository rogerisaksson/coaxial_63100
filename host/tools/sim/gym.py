#!/usr/bin/env python3
"""The gym's grinder: the local model proposes, the relay scores, the log keeps the record.

    python tools/sim/gym.py --hours 1                 # unattended, the look suite unless it says
    python tools/sim/gym.py --suite stand --rounds 2
    python tools/sim/gym.py --dry                     # the prompt and one proposal, nothing run

The model reads BRIEF - her kinematics, the knobs it may move (KNOBS, a span and a clause each),
the suites and the log's last results - a decision model picking the suite when one is pulled
(`coaxial_ollama.decide`) - and answers SCHEMA: a hypothesis in a sentence, the
knobs' spans to search, the suite, the search's size, held to RUNS runs at most (the relay's). The
search runs as `gait_montecarlo --search` runs it; its best - cost, held, values - goes to LOG
and into the next prompt. The model and the relay grind; the reasoning and the smoke tests stay
mine (the user, 2026-10-04).
"""
import argparse
import contextlib
import io
import json
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO  # noqa: E402

LOG = os.path.join(REPO, 'build', 'gym.jsonl')

#: The knobs the model may move: (low, high, what it is), each by its module (`tools.sim.knobs`).
KNOBS = {
    'walker.SIDE_K': (0.1, 0.4, "the pelvis's sideways error fed back, 1"),
    'walker.SIDE_D': (0.03, 0.15, "its speed's, s"),
    'walker.SWAY_K': (0.05, 0.2, 'the head moved toward the pendulum bob, of its offset'),
    'capture.MARGIN': (0.01, 0.06, 'the capture point held by the ankle within this of the sole, m'),
    'capture.GAIN': (0.9, 1.3, 'the landing past the capture point, of what it is off'),
    'landing.PARRY_M': (0.0, 0.05, 'shoved: the capture point this far past the feet, m'),
    'landing.PARRY_HURRY': (0.1, 0.4, 'the phase run this much faster catching'),
    'gait.LEAN_DEG': (2.0, 6.0, 'the lean into the first step, deg'),
    'arrival.SHIFT_IN': (0.02, 0.06, 'her weight inside the standing ankle before the step, m'),
    'arrival.LIFT_UP_M': (0.03, 0.07, 'the first foot lifted this high, m'),
    'arrival.FIRST': (0.4, 0.8, "the first stride, of a stride's"),
    'arrival.PULL_UP_M': (0.05, 0.2, 'the pelvis target within this of the pelvis rising, m'),
    'arrival.PRESS_M': (0.0, 0.02, 'a landed foot reaching this far under the floor until it bears, m'),
    'stance.HALT': (0.3, 0.8, "halting, the stride down to this of its own"),
    'stand.STEP_M': (0.02, 0.1, 'standing: out of the support by this she steps, m'),
    'stand.DWELL_S': (0.15, 0.6, 'no second step sooner than this after one, s'),
    'stand.GAIN': (0.0, 1.0, 'the step past the predicted capture point, of what it is out'),
    'stand.STEP_MAX_M': (0.05, 0.3, 'a step no further than this, m'),
    'stand.HANG_S': (0.05, 0.3, 'a foot unloaded this long is no support, s'),
}

#: The suites and what they score; the runs a candidate makes on each (3 a trial).
SUITES = {'look': ('the rises at 3 paces and the walks at 4, the look of the walk', 21),
          'walk': ('the walks alone', 12),
          'faults': ("the floor's events walking, a glitched board, the shove's fall", 33),
          'stand': ('standing: nudges, shoves, bricks taken away, the balance board', 30)}

#: A search's runs at most: the relay's hour. A 6-knob search at 8 x 12 on the faults suite ran
#: 34 jobs on 16 cores and logged nothing in 50 min (2026-10-04).
RUNS = 400

BRIEF = """You tune the balance of a walking humanoid robot simulated in MuJoCo: a 55 kg woman's
build, 27 joints each a drive holding a setpoint. From a squat she rises, steps off and walks;
each foot lands on the capture point (the centre of mass plus its speed over omega, 3.3/s),
the ankle holds it within the sole, a catch or a side step takes over when it runs off; standing,
a point under each sole is her support and a step toward the capture point is her reflex. A
scoreboard scores a candidate's knobs on a suite of trials: the share she stays up (held) and
a cost (lower is better: falls 300 each, the walk's look, the landings' force). The scoreboard
scores chance: near-identical builds 150 apart, so a span searched beats one value.

Answer JSON: {"hypothesis": one sentence, what you expect and why; "knobs": {name: [low, high]}
for 1 to 4 knobs from the list, spans inside the ones given; "suite": one of the suites;
"generations": 2-6; "population": 4-12}. Each search costs generations x population x the
suite's runs, %d at most. Prefer the suite whose numbers are worst, prefer knobs no search has
moved yet, and do not repeat a hypothesis in the record.

Knobs (name: low..high, what it is):
%s

Suites (name: what it scores, runs a candidate):
%s"""

SCHEMA = {'type': 'object',
          'required': ['hypothesis', 'knobs', 'suite', 'generations', 'population'],
          'properties': {'hypothesis': {'type': 'string'},
                         'knobs': {'type': 'object',
                                   'additionalProperties': {'type': 'array',
                                                            'items': {'type': 'number'}}},
                         'suite': {'enum': list(SUITES)},
                         'generations': {'type': 'integer'},
                         'population': {'type': 'integer'}}}


def brief():
    return BRIEF % (RUNS, '\n'.join('- %s: %g..%g, %s' % (n, lo, hi, what)
                                    for n, (lo, hi, what) in KNOBS.items()),
                    '\n'.join('- %s: %s, %d' % (n, what, runs)
                              for n, (what, runs) in SUITES.items()))


def record(n=12):
    """The log's last `n` results as the model reads them."""
    rows = []
    if os.path.exists(LOG):
        with open(LOG, encoding='utf-8') as f:
            rows = [json.loads(line) for line in f if line.strip()]
    if not rows:
        return 'no search yet'
    return '\n'.join('- %s on %s: cost %.1f held %.1f %% at %s' % (
        r['hypothesis'], r['suite'], r['cost'], 100 * r['held'],
        ', '.join('%s=%.4g' % kv for kv in r['values'].items()))
        for r in rows[-n:])


def decided(suite=None):
    """The suite a decision model picks from the record (`coaxial_ollama.decide`, Clef Flash
    when pulled), else `suite`."""
    from coaxial_ollama import decide
    if suite or not decide.available():
        return suite
    try:
        got = decide.decide('the record so far:\n' + record(), {'suite': decide.choice(
            'Which suite should the next search score: where her numbers are worst and no '
            'search has moved them?', {n: what for n, (what, _runs) in SUITES.items()})})
        return got['suite']['choice']
    except (RuntimeError, KeyError):
        return suite


def propose(model, suite=None):
    """The model's proposal, read and held to KNOBS, SUITES and RUNS; None if it gave none."""
    suite = decided(suite)
    asked = 'the record so far:\n%s\n%s' % (
        record(), 'score it on the %s suite.' % suite if suite else 'choose the suite.')
    messages = [{'role': 'system', 'content': brief()}, {'role': 'user', 'content': asked}]
    try:
        reply = model.chat(messages, fmt=SCHEMA, think=False, num_predict=400)['content']
        got = json.loads(reply)
    except Exception:  # noqa: BLE001 - a model failing must not stop the grind
        return None
    knobs = {}
    for name, span in (got.get('knobs') or {}).items():
        if name in KNOBS and len(span) == 2:
            lo, hi = sorted(float(v) for v in span)
            knobs[name] = (max(KNOBS[name][0], lo), min(KNOBS[name][1], hi))
    knobs = {n: s for n, s in knobs.items() if s[1] > s[0]}
    which = suite or got.get('suite')
    if not knobs or which not in SUITES:
        return None
    population = max(4, min(12, int(got.get('population', 8))))
    generations = max(1, min(int(got.get('generations', 4)), RUNS // (population * SUITES[which][1])))
    return {'hypothesis': str(got.get('hypothesis', ''))[:300], 'knobs': knobs, 'suite': which,
            'generations': generations, 'population': population}


def grind(proposal):
    """The proposal searched on the relay (`gait_montecarlo`), its best logged, printed, returned."""
    from tools.sim import gait_montecarlo
    argv = (['--suite', proposal['suite'], '--generations', str(proposal['generations']),
             '--population', str(proposal['population']), '--log',
             os.path.join(REPO, 'build', 'gym_candidates.jsonl'), '--search']
            + ['%s=%g:%g' % (n, lo, hi) for n, (lo, hi) in proposal['knobs'].items()])
    out = io.StringIO()
    began = time.time()
    with contextlib.redirect_stdout(out):
        gait_montecarlo.main(argv)
    gait_montecarlo.suite('all')
    text = out.getvalue()
    best = re.search(r'^BEST ([\d.]+) (\{.*\})', text, re.M)
    kept = re.search(r'cost\s+[\d.]+\s+held\s+([\d.]+) %', text)
    seconds = round(time.time() - began)
    cost = float(best.group(1)) if best else float('inf')
    values = json.loads(best.group(2)) if best else {}
    held = float(kept.group(1)) / 100.0 if kept else float('nan')
    result = dict(proposal, at=time.strftime('%Y-%m-%d %H:%M'), seconds=seconds, cost=cost,
                  values=values, held=held)
    with open(LOG, 'a', encoding='utf-8') as f:
        f.write(json.dumps(result) + '\n')
    print('  cost %.1f held %.1f %% in %d s: %s' % (
        cost, 100.0 * held, seconds, ', '.join('%s=%.4g' % kv for kv in values.items())), flush=True)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--hours', type=float, default=1.0, help='grind this long')
    parser.add_argument('--rounds', type=int, default=0, help='or this many searches')
    parser.add_argument('--suite', choices=sorted(SUITES), help='the suite, or the model chooses')
    parser.add_argument('--dry', action='store_true', help='the prompt and one proposal, no run')
    args = parser.parse_args(argv)
    from coaxial_ollama.client import Chosen
    model = Chosen()
    if args.dry:
        print(brief())
        print('\nrecord:\n' + record())
        print('\nproposal:', json.dumps(propose(model, args.suite), indent=1))
        return 0
    began, rounds = time.time(), 0
    while (time.time() - began < 3600.0 * args.hours) and (not args.rounds or rounds < args.rounds):
        proposal = propose(model, args.suite)
        if proposal is None:
            print('no proposal', flush=True)
            time.sleep(10.0)
            continue
        print('%s: %s on %s, %d x %d over %s' % (
            time.strftime('%H:%M'), proposal['hypothesis'], proposal['suite'],
            proposal['generations'], proposal['population'],
            ', '.join(proposal['knobs'])), flush=True)
        grind(proposal)
        rounds += 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
