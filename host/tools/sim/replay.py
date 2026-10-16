#!/usr/bin/env python3
"""A HUMANOID recording replayed headless: every command it heard at the pass it came at.

    python tools/sim/replay.py --last                 # the newest recording (R on the page)
    python tools/sim/replay.py build/recordings/x.csv
    python tools/sim/replay.py --last --keep tests/replays/name.csv    # kept as a regression

The page's worker logs each command with its epoch (a world laid anew each: a restart, a fall
recovered) and the pass of that epoch it came at (`worker.apply`); a recording's `commands`
column carries them from the epoch's first pass on. Replayed the same, her world is the same
pass for pass: the line is how far her pelvis came from where the recording has it, and whether
she fell. `--keep` writes the recording's commands and her pelvis's track alone.
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO

#: What a kept replay holds of a recording, column by column.
KEPT = ('t', 'epoch', 'stage', 'x', 'y', 'z', 'keys', 'commands')
#: A kept replay stands laid these m to her left too: one that stood on this host fell at 10.6 s
#: on CI's - another platform's floating point, a fall in the margin.
WOBBLES = (0.0, 2e-4, -2e-4)


def entries(rows):
    """[(epoch, pass, t, command)] of a recording's rows, in the order heard."""
    out = []
    for r in rows:
        out += [tuple(e) for e in json.loads(r.get('commands') or '[]')]
    return sorted(out, key=lambda e: (e[0], e[1]))


def replayed(heard, rows=(), to=None, wobble=0.0):
    """{fell: (epoch, t) or None, off: m her pelvis furthest from the rows' (x, y, z) at their
    (epoch, t), t: where it ended, epochs}: `heard` (`entries`) replayed from its first epoch's
    lay to its last command, or `to` s into its last epoch; laid `wobble` m to her left."""
    from machine import physics, worker
    laid = next(c['laid'] for _e, _p, _t, c in heard if 'laid' in c)
    first = heard[0][0]
    physics.ROOM, physics.SIGHT = laid.get('room', 1.0), laid.get('sight', 0.0)
    s = worker.session(laid['every'], laid['walks'])
    world = s['g']['world']
    world.data.qpos[0] += wobble
    world._mj.mj_forward(world.model, world.data)
    if laid.get('gym', s['gym_name']) != s['gym_name']:
        worker.apply(s, {'gym': laid['gym']})
    want = {(int(r['epoch']) - first, round(float(r['t']), 3)): (float(r['x']), float(r['y']), float(r['z']))
            for r in rows if r.get('epoch') not in (None, '')}
    out = {'fell': None, 'off': 0.0}

    def judged():
        g = s['g']
        if s['fallen_at'] is not None and out['fell'] is None:
            out['fell'] = (s['epoch'], s['fallen_at'])
        at = want.get((s['epoch'], round(g['t'], 3)))
        if at is not None:
            here = g['world'].data.qpos[0:3]
            out['off'] = max(out['off'], math.dist(at, (float(here[0]), float(here[1]), float(here[2]))))

    # replayed apart from the recording - wobbled, or by other code - she falls where it did not, or
    # does not where it did: the epoch the recording went on into never comes, so the replay ends
    # as she falls, or once it has run the passes the recording ran
    timed = [(int(r['epoch']) - first, float(r['t'])) for r in rows if r.get('epoch') not in (None, '')]
    last = max(timed) if timed else (0, 0.0)
    spans = {}
    for epoch, t in timed:
        spans[epoch] = max(spans.get(epoch, 0.0), t)
    budget = round(1e3 * sum(spans.values())) if spans else None
    ran = 0
    for epoch, n, _t, command in heard:
        if 'laid' in command:
            continue
        while (s['epoch'], s['passes']) < (epoch - first, n) and out['fell'] is None:
            worker.stepped(s)
            judged()
            ran += 1
        if out['fell'] is not None:
            break
        worker.apply(s, command)
    end = (last[0], to if to is not None else last[1])
    while ((s['epoch'], s['g']['t']) < (end[0], end[1] - 1e-9) and out['fell'] is None
           and (budget is None or ran < budget)):
        worker.stepped(s)
        judged()
        ran += 1
    return dict(out, t=round(s['g']['t'], 3), epochs=s['epoch'] + 1)


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('csv', nargs='?', help='a HUMANOID recording')
    parser.add_argument('--last', action='store_true', help='the newest recording')
    parser.add_argument('--keep', help='its commands and her pelvis track written here')
    args = parser.parse_args(argv)
    found = glob.glob(os.path.join(REPO, 'build', 'recordings', '*.csv')) if args.last else []
    path = args.csv or (max(found, key=os.path.getmtime) if found else None)
    if path is None:
        parser.error('a recording, or --last')
    with open(path, encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    heard = entries(rows)
    if not heard:
        print('%s: no commands recorded - a recording from before they were' % path)
        return 1
    got = replayed(heard, rows)
    print('%s: %d commands over %d epochs, to %.3f s; her pelvis within %.1f mm of the recording; %s'
          % (path, len(heard), got['epochs'], got['t'], 1e3 * got['off'],
             'fell at %.3f s of epoch %d' % (got['fell'][1], got['fell'][0]) if got['fell'] else 'stood'))
    if args.keep:
        with open(args.keep, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(KEPT)
            writer.writerows([r.get(k, '') for k in KEPT] for r in rows)
    return 0


if __name__ == '__main__':
    sys.exit(main())
