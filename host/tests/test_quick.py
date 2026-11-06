#!/usr/bin/env python3
"""Her compiled parts called whole in C (`tools.cores.quick`'s runner, quick_run.c) beside their Python glue: each part
of her law walking given twins of its arguments made alike, both ways' results the same trees, types, views and
bits at every call."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: Her walk's passes.
PASSES = 3000


def _copied(v):
    """`v`'s arrays copied, its dicts, lists and tuples anew: a twin both of a part's ways are given alike."""
    if isinstance(v, np.ndarray):
        return v.copy()
    if isinstance(v, dict):
        return {k: _copied(x) for k, x in v.items()}
    return type(v)(_copied(x) for x in v) if isinstance(v, (list, tuple)) else v


def _apart(a, b):
    """Whether two results differ: their trees, types, dtypes, shapes, views or bits."""
    if type(a) is not type(b):
        return True
    if isinstance(a, dict):
        return list(a) != list(b) or any(_apart(a[k], b[k]) for k in a)
    if isinstance(a, (list, tuple)):
        return len(a) != len(b) or any(_apart(x, y) for x, y in zip(a, b))
    if isinstance(a, (np.ndarray, np.generic)):
        return (a.dtype != b.dtype or np.shape(a) != np.shape(b) or np.asarray(a).tobytes() != np.asarray(b).tobytes()
                or (isinstance(a, np.ndarray) and (a.base is None) != (b.base is None)))
    return not (a == b or (a != a and b != b))


def test_a_part_called_whole_in_c_as_by_its_glue(report, c):
    """Each compiled part of her law walking, called whole in C (its head, walk, C and result's views) and through its
    Python glue on twins of its arguments made alike - its turn kept as her own call finds it -: their results the
    same trees, types, views and bits at every call."""
    from machine.process import worker
    from tools.cores import bodied, quick
    if quick.module() is None:
        report.check('her parts called whole in C, where the extension is built', True, 'not built here')
        return
    tally, call = {'calls': 0, 'apart': 0, 'parts': set()}, bodied.Compiled.__call__

    def beside(self, *args, paced=True):
        b = self.c
        if b is not None and b.runner is not None:
            twins = [tuple(a if k == self.body else _copied(a) for k, a in enumerate(args)) for _ in range(2)]
            turn, got = int(b.turns[0]), b.runner(twins[0])
            if got is not quick.MISS and got is not quick.SLOW and b.fits(twins[1]) is not None:
                tally['apart'] += _apart(got, b(twins[1]))
                tally['calls'] += 1
                tally['parts'].add(self.name)
            b.turns[0] = turn
        return call(self, *args, paced=paced)
    bodied.Compiled.__call__ = beside
    try:
        s = worker.session(walks=True)
        for _ in range(PASSES):
            worker.stepped(s)
    finally:
        bodied.Compiled.__call__ = call
    report.check('her parts called whole in C as by their glue, %d calls over %d parts walking'
                 % (tally['calls'], len(tally['parts'])),
                 tally['calls'] > 100 and len(tally['parts']) >= 15 and not tally['apart'], '%d apart' % tally['apart'])


ROSTER = (test_a_part_called_whole_in_c_as_by_its_glue,)


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
