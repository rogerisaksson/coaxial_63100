#!/usr/bin/env python3
"""Her drives' pass in one C call (`tools.cores.tick`, wbc_pass_tick) beside its python (`motion.drives_twin`): from the
same state at every STRIDE-th pass lying into a bed, resting limp there, rising and walking, every array, her master
bus and the numbers it writes the same bits."""
import ctypes
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: Her lie into the bed and her rise off it, s; her walk after, passes; a pass compared each STRIDE.
LIE_S, RISE_S, WALK_PASSES, STRIDE = 24.0, 8.0, 6000, 7
#: Her numbers the pass writes on `g`.
NUMBERS = ('surety', 'surety_vec', 'surety_probe', 'left', '_wake')


def _held(g):
    """What her drives' pass writes, as it is: its arrays, her master bus's state and arrays, her numbers."""
    _p, kept, _step = g['drives']
    _k, bus, arrays = g['bus']
    out = [g[name] for name in ('eso', 'ref_q', 'ref_v', 'hold')] + [kept[name] for name in ('sb', 'feed', 'limit')]
    out = [a.copy() for a in out + [g['cmd']['limit']] + list(arrays.values())]
    return out + [ctypes.string_at(ctypes.addressof(bus), ctypes.sizeof(bus)), tuple(g.get(k) for k in NUMBERS)]


def _put(g, was):
    """`g` as `_held` held it."""
    _p, kept, _step = g['drives']
    _k, bus, arrays = g['bus']
    out = [g[name] for name in ('eso', 'ref_q', 'ref_v', 'hold')] + [kept[name] for name in ('sb', 'feed', 'limit')]
    for a, held in zip(out + [g['cmd']['limit']] + list(arrays.values()), was):
        a[...] = held
    ctypes.memmove(ctypes.addressof(bus), was[-2], ctypes.sizeof(bus))
    for k, v in zip(NUMBERS, was[-1]):
        g.pop(k, None) if v is None else g.__setitem__(k, v)


def _apart(a, b):
    """Whether two things held differ: an array's bits, the bus's bytes, a number's repr."""
    if isinstance(a, np.ndarray):
        return a.dtype != b.dtype or a.tobytes() != b.tobytes()
    return a != b if isinstance(a, bytes) else repr(a) != repr(b)


def test_her_drives_pass_in_c_as_in_python(report, _c):
    """Her drives' pass in one C call and in its python from the same state - each pass of the two compared, the C's
    kept -: lying into a bed and resting limp there, rising off it, walking."""
    from machine.law import motion
    from machine.process import worker
    from tools.sim import told
    tally, real = {'n': 0, 'passes': 0, 'apart': 0, 'limp': 0, 'held': 0}, motion.drives

    def beside(g, dt):
        tally['n'] += 1
        if tally['n'] % STRIDE:
            return real(g, dt)
        was = _held(g)
        motion.drives_twin(g, dt)
        twin = _held(g)
        _put(g, was)
        real(g, dt)
        got = _held(g)
        tally['apart'] += any(_apart(a, b) for a, b in zip(twin, got))
        tally['passes'] += 1
        tally['limp'] += bool((g['cmd']['limit'] < g['drives'][1]['limit']).any())
        tally['held'] += bool(g['hold'].any())
    motion.drives = beside
    try:
        r = told.Run('bed_behind')
        r.tell(say='lie_bed')
        r.go(LIE_S)
        r.tell(say='rise_bed')
        r.go(RISE_S)
        s = worker.session(walks=True)
        for _ in range(WALK_PASSES):
            worker.stepped(s)
    finally:
        motion.drives = real
    report.check('her drives pass in C as in python, %(passes)d passes (%(limp)d limp, %(held)d holding WEP)' % tally,
                 tally['passes'] > 3000 and tally['limp'] > 100 and not tally['apart'], '%(apart)d apart' % tally)


ROSTER = (test_her_drives_pass_in_c_as_in_python,)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    from tools.cores.build import find_cc
    if find_cc() is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, None)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
