#!/usr/bin/env python3
"""Her whole bench on the relay: every scenario at three wobbles, the 80 N polar, the kept replays.

    python tools/sim/bench.py build/bench.json                         # this tree's
    python tools/sim/bench.py build/now.json --against build/bench.json
    python tools/sim/bench.py build/base.json --host ../base/host      # another tree's code

A line a scenario and wobble (`+` met, `-` not, the second where it fell), the tallies, and with
`--against` what moved: a change is judged on the whole, the dynamics' margins moving where no
single scenario shows it (docs/findings/scenarios.md).
"""
import argparse
import glob
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: Each scenario laid turned by these, rad (`scenarios.ran`); a kept replay laid aside by these, m.
WOBBLES = ('0', '0.0002', '-0.0002')
#: The polar's force, N.
SHOVE_N = '80'
#: A kept replay replayed in its own process on the tree at its third argument, its line as JSON:
#: a relay's job runs in this tree, and `-c` imports from there.
REPLAYED = ('import csv, json, sys; sys.path.insert(0, sys.argv[3]); from tools.sim import replay; '
            'rows = list(csv.DictReader(open(sys.argv[1], '
            "encoding='utf-8'))); got = replay.replayed(replay.entries(rows), rows, wobble=float(sys.argv[2])); "
            "print(json.dumps({'fell': got['fell'], 't': got['t']}))")


def jobs(host, sets=()):
    """The bench's jobs on `host`'s code: (name, argv); its scenarios and shoves on her texts with
    `sets` (PATH:START:END=VALUE) spliced - a replay replays her texts as they are."""
    from tools.dev import focus
    names = json.loads(subprocess.check_output(
        [sys.executable, '-c', 'import json; from tools.sim import scenarios as s; print(json.dumps([list(s.SCENARIOS), '
         '[n for n in s.SHOVES if n.split("_")[1] == "%s"]]))' % SHOVE_N], cwd=host, text=True).strip().splitlines()[-1])
    scenes, shoves = names
    one, said = [sys.executable, '-X', 'utf8', os.path.join(host, 'tools', 'sim', 'scenarios.py'), '--one'], [
        '--set=' + s for s in sets]
    out = [focus.Job('%s@%s' % (n, w), one + [n, '--wobble', w] + said, timeout=900.0) for n in scenes for w in WOBBLES]
    out += [focus.Job('%s@0' % n, one + [n, '--wobble', '0'] + said, timeout=900.0) for n in shoves]
    kept = sorted(glob.glob(os.path.join(host, 'tests', 'replays', '*.csv')))
    out += [focus.Job('replay:%s@%s' % (os.path.basename(p), w), [sys.executable, '-X', 'utf8', '-c', REPLAYED, p, w, host],
                      timeout=900.0) for p in kept for w in WOBBLES]
    return out


def ran(bench):
    """{job's name: its line} of the jobs `bench` on the relay."""
    from tools.dev import focus
    got = {}
    for job, text, _code, _s in focus.relay(bench):
        line = next((ln for ln in reversed((text or '').splitlines()) if ln.startswith('{')), None)
        got[job.name] = json.loads(line) if line else {'error': (text or '').strip()[-400:]}
    return got


def met(name, r):
    """Whether a line met its goal: a replay standing, a scenario its own."""
    return 'error' not in r and (r.get('fell') is None if name.startswith('replay:') else bool(r.get('met')))


def tally(got):
    """'scenes m/n, shoves 80 N stand s walk w of n, replays r/n'."""
    count = {}
    for name, r in got.items():
        kind = 'replay' if name.startswith('replay:') else ('walk' if '_walk_' in name else 'stand')
        kind = kind if name.startswith(('replay:', 'shove_')) else 'scene'
        has, of = count.get(kind, (0, 0))
        count[kind] = (has + met(name, r), of + 1)
    c = lambda k: count.get(k, (0, 0))
    return 'scenes %d/%d, shoves %s N stand %d walk %d of %d, replays %d/%d' % (
        *c('scene'), SHOVE_N, c('stand')[0], c('walk')[0], c('stand')[1] + c('walk')[1], *c('replay'))


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out', help='its lines written here as JSON')
    parser.add_argument('--against', help="another run's JSON")
    parser.add_argument('--host', default=os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                        help="the host tree whose code runs (a worktree's for a baseline)")
    args = parser.parse_args(argv)
    host = os.path.abspath(args.host)
    os.environ['PYTHONPATH'] = host
    got = ran(jobs(host))
    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump(got, f, indent=0, sort_keys=True)
    was = {}
    if args.against:
        with open(args.against, encoding='utf-8') as f:
            was = json.load(f)
    lines = {}
    for name in got:
        key, w = name.rsplit('@', 1)
        lines.setdefault(key, {})[w] = got[name]
    for key in sorted(k for k in lines if not k.startswith('shove_')):
        marks = lambda res: ''.join(('+' if met(key, res['%s@%s' % (key, w)]) else '-') if '%s@%s' % (key, w) in res
                                    else ' ' for w in WOBBLES)
        fell = ' '.join(str(lines[key][w].get('fell_at', lines[key][w].get('fell'))) for w in WOBBLES if w in lines[key])
        print('%-30s %s %s  %s' % (key, marks(got), ('was ' + marks(was)) if was else '', fell))
    print(tally(got))
    if was:
        print('was ' + tally(was))
        print('moved: ' + ', '.join('%s %s' % (n, 'up' if met(n, got[n]) else 'DOWN') for n in sorted(got)
                                    if n in was and met(n, got[n]) != met(n, was[n])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
