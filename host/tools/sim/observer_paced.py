#!/usr/bin/env python3
"""The robot's observer's C step timed at every law pass of an errand, function by function.

    python tools/sim/observer_paced.py bed              # each function's calls and us a law pass, alone and with callees
    python tools/sim/observer_paced.py chair --phases   # and the same for each phase of the errand, most passes first

The errand laid as pace.py lays it, blind (`pace.Run`): at each law pass the observer's step runs first on a copy of
the robot's observer in the probe's build (`tools.cores.gmo_probe`), timed, then the robot's own step on the robot's
observer, its wall time read; a pass whose copy ends a byte apart from the robot's observer is counted. The probe
touches nothing the robot reads: the passes' hash is pace.py's for the errand.
"""
import argparse
import collections
import ctypes
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

# numpy loaded before the robot's tools, as pace.py loads it: the robot's bits pace.py's
import numpy as np  # noqa: E402

#: The functions a table lists at most, by their own seconds; a phase's.
LISTED, PHASE_LISTED = 21, 8


class Tally:

    """A phase's sums: law passes, those whose copy ended apart, the robot's own step's seconds, the friction band's
    rounds, and each probe slot's calls, own and inclusive seconds."""

    def __init__(self, slots):
        self.passes = self.apart = 0
        self.wall = self.rounds = 0.0
        self.calls, self.own, self.inclusive = np.zeros(slots), np.zeros(slots), np.zeros(slots)

    def add(self, got, wall, apart, rounds, passes=1):
        calls, own, inclusive = got
        self.passes, self.apart = self.passes + passes, self.apart + int(apart)
        self.wall, self.rounds = self.wall + wall, self.rounds + rounds
        self.calls += calls
        self.own += own
        self.inclusive += inclusive


def paced(name, phases=False):
    """(the run, the probe, {phase: Tally}) of errand `name` run as pace.py runs it, the observer's step timed at
    every law pass on a copy (module docstring); one phase, the errand's, unless `phases`."""
    from tools.cores import gmo, gmo_probe
    from tools.sim import pace
    probe = gmo_probe.Probe()
    r = pace.Run(blind=True)
    obs = r.s['g']['observer']
    c, real, size = obs['c'], obs['step'], ctypes.sizeof(gmo.Gmo)
    copy = gmo.made()
    tallies = collections.defaultdict(lambda: Tally(len(probe.names)))

    def stepped(at, base, q, u, dt):
        ctypes.memmove(ctypes.addressof(copy), ctypes.addressof(c), size)
        probe.step(copy, base, q, u, dt)
        got = probe.read()
        t0 = time.perf_counter()
        real(at, base, q, u, dt)
        wall = time.perf_counter() - t0
        apart = ctypes.string_at(ctypes.addressof(copy), size) != ctypes.string_at(ctypes.addressof(c), size)
        tallies[r.doing() if phases else name].add(got, wall, apart, c.rounds)

    obs['step'] = stepped
    try:
        r.go(0.5)
        pace.ERRANDS[name](r)
    finally:
        r.close()
    return r, probe, tallies


def table(probe, tally, listed):
    """Lines of `tally`'s functions, the most own seconds first, `listed` at most: calls, own and inclusive us a
    law pass, own share of the probe's step."""
    n = max(tally.passes, 1)
    step = tally.inclusive[probe.names.index('wbc_gmo_step')]
    lines = ['  %-18s %8s %9s %12s %7s' % ('function', 'calls', 'own us', 'with callees', 'own %')]
    for k in np.argsort(-tally.own)[:listed]:
        if tally.calls[k]:
            lines.append('  %-18s %8.2f %9.1f %12.1f %6.1f%%' % (
                probe.names[k], tally.calls[k] / n, 1e6 * tally.own[k] / n, 1e6 * tally.inclusive[k] / n,
                100.0 * tally.own[k] / max(step, 1e-12)))
    return lines


def said(name, r, probe, tallies):
    """The run's lines: its passes, the copies apart, the step's time the robot's and the probe's, each table."""
    whole = Tally(len(probe.names))
    for t in tallies.values():
        whole.add((t.calls, t.own, t.inclusive), t.wall, t.apart, t.rounds, t.passes)
    n = max(whole.passes, 1)
    step = whole.inclusive[probe.names.index('wbc_gmo_step')]
    lines = ['observer_paced: %s, %d law passes, %d of them a copy apart; passes %s%s' % (
                 name, whole.passes, whole.apart, r.passed.hexdigest()[:16], r.ended),
             "the robot's own step %.1f us a law pass, the probe's %.1f (its hooks taken off: %.0f + %.0f ticks a "
             'call, %.2f GHz); %.2f friction rounds a pass' % (1e6 * whole.wall / n, 1e6 * step / n, probe.caller,
                                                               probe.within, probe.rate / 1e9, whole.rounds / n)]
    lines += table(probe, whole, LISTED)
    if len(tallies) > 1:
        for phase, t in sorted(tallies.items(), key=lambda kv: -kv[1].passes):
            lines += ["-- %s: %d passes, the robot's step %.1f us, %.2f rounds --" % (
                phase, t.passes, 1e6 * t.wall / max(t.passes, 1), t.rounds / max(t.passes, 1))]
            lines += table(probe, t, PHASE_LISTED)
    return lines


def main(argv=None):
    from tools.sim import pace
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('errand', nargs='?', default='bed', choices=[e for e in pace.ERRANDS if e not in pace.CRAFTS])
    p.add_argument('--phases', action='store_true', help="each phase of the errand's own table too")
    args = p.parse_args(argv)
    for line in said(args.errand, *paced(args.errand, args.phases)):
        print(line, flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
