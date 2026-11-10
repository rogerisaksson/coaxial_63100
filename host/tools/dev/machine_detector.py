#!/usr/bin/env python3
"""The host a run is on, measured as it starts: its cores, its pace at the relay's width.

    python tools/dev/machine_detector.py           # this host, said
    python tools/dev/machine_detector.py --keep    # tests/work.json laid from this tree's whole runs

A suite's seconds are a host's and a width's: test_wasd's shards sum to 800.8 s here, 32 side by side,
and to 505.0 s on GitHub's runner of two cores - a core beside 31 busy ones runs the same kernel of
python and numpy 1.8 times slower than alone. Seconds written into the tree as numbers (FRESH_S: 900
for that suite, a cut on CI's runner told in its comment) were one host's guesses of another's. So a
run measures the host it is on: the kernel (`work`) on every baton at once, begun as one, its pace
kernels a second (`paced`). What a suite costs is kept as work - its seconds times the pace they were
run at, kernels, no host's - in tests/work.json, the heaviest of its whole runs (`--keep`); a run expects of a
suite its work over this host's pace where the tree's own record holds none, and that record's seconds
by the pace they were run at (`expected`). A suite's shards and its limit follow (`runner._jobs`).
"""
import argparse
import json
import os
import platform
import statistics
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.dev import counts
from tools.dev.focus import physical_cores
from tools.dev.suites import ROOT

#: Each suite's work, kernels, by its name: laid by `--keep`, read where a tree has no record of its own.
PATH = ROOT / 'tests' / 'work.json'

#: The kernel: its turns, the order of the system each solves, the terms python sums in each - a third of
#: a second alone on a core of this year's, a suite's glue and its numpy in the share her law has them.
TURNS, ORDER, TERMS = 12000, 48, 64

#: A baton's process: numpy loaded and the kernel's code run once, 'ready' said, the kernel timed at a word.
BATON = ('import sys\nfrom tools.dev import machine_detector as m\nm.work(%d)\nprint("ready", flush=True)\n'
         'sys.stdin.readline()\nprint(m.work(), flush=True)\n' % (TURNS // 100))


def work(turns=TURNS):
    """s this process takes for `turns` of the kernel: a small solve and a sum in python's own loop each."""
    import numpy as np
    rng = np.random.default_rng(0)
    a, b = rng.standard_normal((ORDER, ORDER)) + ORDER * np.eye(ORDER), rng.standard_normal(ORDER)
    t, total = time.perf_counter(), 0.0
    for k in range(turns):
        total += float(np.linalg.solve(a, b)[k % ORDER]) + sum(0.5 * j for j in range(TERMS))
    return time.perf_counter() - t


def paced(width):
    """Kernels a second a baton does with `width` at once, as the relay's jobs run - a process each, OpenBLAS
    on one thread -: every one stood ready before any begins, all begun as one, the median's."""
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(filter(None, [str(ROOT), os.environ.get('PYTHONPATH')])))
    env.setdefault('OPENBLAS_NUM_THREADS', '1')
    going = [subprocess.Popen([sys.executable, '-X', 'utf8', '-c', BATON], stdin=subprocess.PIPE,
                              stdout=subprocess.PIPE, text=True, env=env, cwd=ROOT) for _ in range(width)]
    for baton in going:
        if baton.stdout is not None:
            baton.stdout.readline()
    for baton in going:
        if baton.stdin is not None:
            baton.stdin.write('\n')
            baton.stdin.flush()
    return 1.0 / statistics.median(float(baton.communicate()[0]) for baton in going)


def host(width=None):
    """This host as a run finds it: its physical cores, its threads, its processor and its system, and its
    pace at `width` batons - the relay's: its physical cores unless told."""
    cores = physical_cores()
    width = width or cores
    return {'cores': cores, 'threads': os.cpu_count() or cores, 'machine': platform.machine(),
            'system': platform.system(), 'width': width, 'pace': paced(width)}


def said(here):
    """A host, in a line."""
    return 'host: %d cores, %d threads, %s %s; %.2f kernels a second a baton, %d at once' % (
        here['cores'], here['threads'], here['machine'], here['system'], here['pace'], here['width'])


def table(path=PATH):
    """{suite: its work, kernels} as the table holds it (tests/work.json unless told); none where it holds none."""
    try:
        with open(path, encoding='utf-8') as f:
            got = json.load(f)
    except (OSError, ValueError):
        return {}
    return got if isinstance(got, dict) else {}


def expected(seconds, paces, pace, work_of=None, suites=()):
    """{suite: s} a run expects on a host of `pace`: the tree's own record `seconds`, each by the pace it was
    run at (`paces`; one with none as it stands), for a suite the record has not, its work (`work_of`,
    the table's unless told) over that pace, and for one of `suites` neither knows the heaviest they know:
    its first whole run is its measure, and expected at 0 s it ran as one job cut at the plain limit."""
    own = {name: s * paces.get(name, pace) / pace for name, s in seconds.items()}
    known = {**{name: kernels / pace for name, kernels in (table() if work_of is None else work_of).items()}, **own}
    return {**dict.fromkeys(suites, max(known.values(), default=0.0)), **known}


def keep(known=None, path=PATH):
    """The table laid (tests/work.json unless told): each suite's work, kernels, off this tree's record of
    whole runs (tests/.counts.json unless told: a suite's seconds and the pace they were run at). A row is
    the most a whole run of its suite has been measured to cost, raised and never lowered: a tree's state
    is in a run's seconds - test_layout 20.7 s a fresh tree's first run and 2.2 s its next, 36 of the gate's
    suites past 1.6 times lighter in one tree's gate than in another's - and a limit is to hold on a fresh
    checkout; laid anew where the file is removed first. A suite the record has not keeps its row - read
    before the file is opened to be written: read after, a run of eighteen suites left a table of 109
    those eighteen. What the record told."""
    known = counts.load() if known is None else known
    seconds, paces = known.get('seconds') or {}, known.get('paces') or {}
    laid = {name: round(seconds[name] * paces[name], 1) for name in sorted(seconds) if name in paces}
    held = table(path)
    held.update({name: max(kernels, held.get(name, 0.0)) for name, kernels in laid.items()})
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        json.dump(held, f, indent=1, sort_keys=True)
        f.write('\n')
    return laid


def main(argv=None):
    """`--keep`: the table laid, how many suites said; else this host measured and said."""
    parser = argparse.ArgumentParser(description=(__doc__ or '').split('\n')[0])
    parser.add_argument('--keep', action='store_true', help="tests/work.json laid from this tree's whole runs")
    parser.add_argument('--jobs', type=int, help='the width measured at: the physical cores unless told')
    args = parser.parse_args(argv)
    if args.keep:
        print('machine_detector: the work of %d suites laid in %s' % (len(keep()), PATH.name))
        return 0
    print(said(host(args.jobs)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
