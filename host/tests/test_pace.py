#!/usr/bin/env python3
"""Does her loop still run as fast as it did? Her errands' pace at a benchmark's priority, against a recorded floor.

    .\run_tests.ps1 -Bench                      # with the board's bench, each suite above normal priority
    python -X utf8 tests/test_pace.py --record  # this host's pace written as its floor

Each errand (`tools.sim.pace`: stand, walk, door, chair, bed) runs as the worker runs her - her lodge, her camera, her
planner and her vision apart -, flat out, alone on the relay at a benchmark's priority (`focus.relay`'s bench: her
processes above normal, the host's other loads under them): its mean pace, her world's seconds a wall second, at least
SLACK of this host's recorded one (BASELINE, the host's own, kept out of the tree). Real time is 1.0; past it with a
margin is the aim, read off each line. Under a plain run's runner, held below normal (`background.lower`), her pace is
not hers: skipped there.
"""
import ctypes
import json
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

BASELINE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.pace.json')

#: The share of a host's recorded mean pace below which an errand has regressed: at a benchmark's priority her pace
#: flickers a few percent between runs, a pass's 10 % lost is caught.
SLACK = 0.85

#: A pace line (`pace.paced`): its errand, its worst second and what she did, its mean, its qpos, where it ended.
LINE = re.compile(r'^(\S+)\s+worst x([\d.]+) \((.*?)\) mean x([\d.]+) \| .*? \| qpos (\w+) at t ([\d.]+)(.*)$')


def paced():
    """{errand: (mean, worst, its line's tail)} of every errand's line on the relay, None for one with no line."""
    from tools.sim import pace
    got = {}
    for name, line in pace.relayed(list(pace.ERRANDS)):
        m = LINE.match(line.strip())
        got[name] = None if m is None else (float(m.group(4)), float(m.group(2)),
                                            'qpos %s at t %s%s' % (m.group(5), m.group(6), m.group(7)))
    return got


def report(now, was):
    """(lines, failures): each errand's mean against `was`, its floor's source; one without a line fails."""
    lines, bad = [], 0
    for name, got in now.items():
        if got is None:
            lines.append('  FAIL  %-6s no line: it did not run to its end' % name)
            bad += 1
            continue
        mean, worst, tail = got
        side = 'past' if mean >= 1.0 else 'under'
        before = was.get(name)
        if not before:
            lines.append('  PASS  %-6s mean x%.2f worst x%.2f, %s real time, no floor (%s)' % (name, mean, worst, side,
                                                                                             tail))
            continue
        ok = mean >= SLACK * before
        bad += not ok
        lines.append('  %s  %-6s mean x%.2f worst x%.2f, %s real time, was x%.2f (%3.0f %%) (%s)'
                     % ('PASS' if ok else 'FAIL', name, mean, worst, side, before, 100.0 * mean / before, tail))
    return lines, bad


def lowered():
    """Whether this process runs below normal priority - a plain run's runner holds its suites there
    (`background.lower`), her pace measured so not hers. False on another system."""
    if sys.platform != 'win32':
        return False
    kernel = ctypes.WinDLL('kernel32')
    kernel.GetCurrentProcess.restype = ctypes.c_void_p
    kernel.GetPriorityClass.argtypes = [ctypes.c_void_p]
    return kernel.GetPriorityClass(kernel.GetCurrentProcess()) in (getattr(subprocess, 'IDLE_PRIORITY_CLASS'),
                                                                   getattr(subprocess, 'BELOW_NORMAL_PRIORITY_CLASS'))


def main():
    from tools.cores.build import find_cc
    if find_cc() is None or lowered():
        print('  SKIP  %s' % ('no host C compiler; setup.ps1 installs one' if find_cc() is None else
                              "held below normal priority: her pace is a benchmark's (run_tests.ps1 -Bench)"))
        print('\n0 passed, 0 failed, 1 skipped')
        return 0
    was = {}
    if os.path.exists(BASELINE):
        with open(BASELINE, encoding='utf-8') as handle:
            was = json.load(handle)
    now = paced()
    lines, bad = report(now, was)
    print('\n'.join(lines))
    if '--record' in sys.argv and not bad:
        with open(BASELINE, 'w', encoding='utf-8', newline='\n') as handle:
            json.dump({name: got[0] for name, got in now.items() if got is not None}, handle, indent=2, sort_keys=True)
            handle.write('\n')
        print('\nfloor written to %s' % os.path.basename(BASELINE))
    print('\n%d passed, %d failed' % (len(now) - bad, bad))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
