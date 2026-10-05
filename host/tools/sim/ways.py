#!/usr/bin/env python3
"""Her ways on the one law over a spread of timings, a group a row: how many she is up through.

    python tools/sim/ways.py                       # every group, 104 trials through the relay
    python tools/sim/ways.py --only run -v         # the groups named so, each trial's segments
    python tools/sim/ways.py hold.HOLD_K=0.8       # under a constant moved

A trial is `go.py --json` on a track or on rows asked ('ask ..', hers following as
`gaits.toward` has it), shoved or not; a group passes it as she is up at its end and what the
group asks of its last segment holds - standing again, the last LAST_S s are judged alone. The
measure of docs/TODO.md's item on her going: nothing here is a price, a fall ends a trial.
"""
import argparse
import json
import os
import sys

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, HOST)

#: Standing again, her last row is asked once more LAST_S before the end: a segment to judge.
LAST_S = 3.0


#: The page's S and F from her walk to the run and down to her stand, (s on, row): a press
#: every 3-8 s (`humanoid_keys.LEVELS`).
STEPPED = ((0.0, 0.5), (6.0, 0.7), (10.0, 0.85), (14.0, 1.0), (22.0, 0.85), (25.0, 0.7),
           (28.0, 0.5), (34.0, 0.0), (40.0, -0.3), (44.0, -0.6), (48.0, -1.0))


def still(result):
    """Standing at its end: hardly moving, three steps at most in its last segment."""
    last = result['segments'][-1]
    return abs(last['speed']) < 0.05 and last['steps'] <= 3


def stood(result):
    """Stood through: hardly moving and no step."""
    last = result['segments'][-1]
    return abs(last['speed']) < 0.03 and last['steps'] == 0


def up(_result):
    """Up at its end, no more asked."""
    return True


#: (group, its seconds, what it asks, [(track, shoves)]): the page's nudge and shove
#: (`events.SHOVES`) from four or eight ways.
GROUPS = [
    ('stand, walk, stand', 20.0, still, [('ask 0:-1 %g:0 %g:-1' % (t, t + 7.0 + 0.37 * i), '')
                                         for i, t in enumerate((1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.3,
                                                                2.6, 2.9, 3.2))]),
    ('slow walks', 20.0, up, [(t, '') for t in (
        '0:-1 2:-1 3:-0.75 20:-0.75', '0:-1 2:-1 3:-0.5 20:-0.5', '0:-1 2:-1 3:-0.25 20:-0.25',
        '0:-1 1:-1 2:0 5:0 6:-0.25 20:-0.25', '0:-1 1:-1 2:0 5:0 6:-0.5 20:-0.5',
        '0:-1 1:-1 2:0 5:0 6:-0.75 20:-0.75')]),
    ('to her jog and back', 30.0, still, [('ask 0:-1 %g:0.5 %g:-1' % (t, t + 12.0 + 0.41 * i), '')
                                          for i, t in enumerate((1.0, 1.25, 1.5, 1.75, 2.0, 2.25,
                                                                 2.5, 2.75))]),
    ('to the run and back', 44.0, still, [('ask 0:-1 %g:1 %g:-1' % (t, t + 17.0 + 0.43 * i), '')
                                          for i, t in enumerate((1.0, 1.15, 1.3, 1.45, 1.6, 1.75,
                                                                 1.9, 2.05, 2.2, 2.35, 2.5, 2.65))]),
    ('turned back on her way', 50.0, still, [('ask ' + t, '') for t in (
        '0:-1 1:1 9.5:0 16:1 30:-1', '0:-1 1:1 11:0.5 19:1 31:-1', '0:-1 1:0.5 4:0 8:1 28:-1',
        '0:-1 1:1 5:-1 12:1 30:-1', '0:-1 1:1 12.5:-1 24:0.5 32:-1', '0:-1 1:0 3:1 6:0 14:-1')]),
    ('the run held', 60.0, up, [('ask 0:-1 1:1', ''), ('ask 0:-1 1.4:1', '')]),
    ("stepped as the page's keys", 66.0, still, [('ask 0:-1 1:0 ' + ' '.join(
        '%g:%g' % (6.0 + 0.19 * i + t, k) for t, k in STEPPED), '') for i in range(12)]),
    ('nudged standing', 10.0, stood, [('0:-1', '4:38:%d' % d) for d in range(0, 360, 45)]),
    ('shoved standing', 12.0, still, [('0:-1', '4:120:%d' % d) for d in range(0, 360, 45)]),
    ('nudged walking', 16.0, up, [('0:-1 1:-1 2:0 16:0', '%g:38:%d' % (t, d))
                                  for t in (7.0, 7.25) for d in (0, 90, 180, 270)]),
    ('shoved walking', 16.0, up, [('0:-1 1:-1 2:0 16:0', '%g:120:%d' % (t, d))
                                  for t in (7.0, 7.25) for d in (0, 90, 180, 270)]),
    ('nudged running', 26.0, up, [('ask 0:-1 1:1', '%g:38:%d' % (t, d))
                                  for t in (18.0, 18.2) for d in (0, 90, 180, 270)]),
    ('shoved running', 26.0, up, [('ask 0:-1 1:1', '%g:120:%d' % (t, d))
                                  for t in (18.0, 18.2) for d in (0, 90, 180, 270)]),
]


def gone(only=(), knobs=()):
    """[(group, [(track, shoves, result or None, passed)])] of GROUPS - those with a word of
    `only` in their name - under `knobs` ['NAME=V'], every trial a job of the relay."""
    from tools.dev import focus
    tool = os.path.join(HOST, 'tools', 'sim', 'go.py')
    groups = [g for g in GROUPS if not only or any(word in g[0] for word in only)]
    jobs = []
    for group, to_s, ask, trials in groups:
        for i, (track, shoves) in enumerate(trials):
            if track.startswith('ask ') and ask is still:
                track += ' %g:%s' % (to_s - LAST_S, track.split(':')[-1])
            how = ['--ask', track[4:]] if track.startswith('ask ') else ['--track', track]
            jobs.append(focus.Job('%s|%d' % (group, i), [
                sys.executable, '-X', 'utf8', tool, '--json', '--to', str(to_s)] + how + (
                    ['--shove', shoves] if shoves else []) + list(knobs), 1.2, 900.0))
    out = {}
    for job, text, _code, _s in focus.relay(jobs):
        line = next((ln[7:] for ln in reversed(text.splitlines()) if ln.startswith('RESULT ')),
                    None)
        out[job.name] = json.loads(line) if line else None
    return [(group, [(track, shoves, r, bool(r) and not r['fell'] and ask(r))
                     for i, (track, shoves) in enumerate(trials)
                     for r in (out['%s|%d' % (group, i)],)])
            for group, _to_s, ask, trials in groups]


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--only', nargs='*', default=(), help='groups with one of these words')
    parser.add_argument('-v', action='store_true', help='every trial, its segments')
    parser.add_argument('knobs', nargs='*', metavar='NAME=V', help='constants moved')
    args = parser.parse_args(argv)
    total = [0, 0]
    for group, trials in gone(args.only, args.knobs):
        off = ['lost' if r is None else '%.1f' % r['fell'] if r['fell'] else 'not as asked'
               for _track, _shoves, r, passed in trials if not passed]
        for track, shoves, r, _passed in trials if args.v else ():
            print('    %-60s %-9s %s' % (track[:60], shoves, 'lost' if r is None else '%s | %s' % (
                'down %.1f' % r['fell'] if r['fell'] else 'up', ' '.join(
                    '%+.2f:%.2f m/s,%d st' % (g['k'][1], g['speed'], g['steps'])
                    for g in r['segments']))))
        passed = sum(1 for trial in trials if trial[3])
        total = [total[0] + passed, total[1] + len(trials)]
        print('%-23s %2d of %2d%s' % (group, passed, len(trials),
                                      '   down or off: ' + ' '.join(off) if off else ''))
    print('RESULT %d of %d' % tuple(total))
    return 0


if __name__ == '__main__':
    sys.exit(main())
