#!/usr/bin/env python3
"""The robot's compiled parts called whole in C (`tools.cores.quick`'s runner, quick_run.c) beside their Python glue - each part
of its law walking given twins of its arguments made alike -, and the robot's step asked in C (quick_step.c)
beside its Python from the same struct: both ways' results the same trees, types, views and bits at every call."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The robot's walk's passes; its lie into a bed and its rise off it, s.
PASSES, LIE_S, RISE_S = 3000, 12.0, 8.0


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
    """Each compiled part of the robot's law walking, called whole in C (its head, walk, C and result's views) and
    through its Python glue on twins of its arguments made alike - its turn kept as the robot's own call finds it -:
    their results the same trees, types, views and bits at every call."""
    from machine.process import worker
    from tools.cores import bodied, quick
    if quick.module() is None:
        report.check('its parts called whole in C, where the extension is built', True, 'not built here')
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
        out = call(self, *args, paced=False if b is None else paced)
        if self.c is None and self.job is not None:
            self.job['done'].wait()
        return out
    bodied.Compiled.__call__ = beside
    try:
        s = worker.session(walks=True)
        for _ in range(PASSES):
            worker.stepped(s)
    finally:
        bodied.Compiled.__call__ = call
    report.check('its parts called whole in C as by their glue, %d calls over %d parts walking'
                 % (tally['calls'], len(tally['parts'])),
                 tally['calls'] > 100 and len(tally['parts']) >= 15 and not tally['apart'], '%d apart' % tally['apart'])


def _numbers(k):
    """The step's struct `k` but its pointers: each field's bytes not a pointer's, in its order."""
    import ctypes

    def pointer(t):
        return t is ctypes.c_void_p or (hasattr(t, '_length_') and pointer(t._type_))
    at = ctypes.addressof(k)
    return b''.join(ctypes.string_at(at + getattr(type(k), name).offset, ctypes.sizeof(t))
                    for name, t in type(k)._fields_ if not pointer(t))


def test_the_robots_step_asked_in_c_as_by_python(report, c):
    """The robot's step's ask and senses laid into its struct, its C called and its answer made in one C call
    (quick_step.c) beside `_stepped`'s Python from the same struct - lying into a bed, rising off it and walking -:
    every answer the same trees, types, views and bits, `b.load` too, and the struct but its pointers the same
    after."""
    import ctypes
    from machine.process import worker
    from tools.cores import quick, step as cs
    from tools.sim import told
    if quick.module() is None:
        report.check('its step asked in C, where the extension is built', True, 'not built here')
        return
    tally, real = {'calls': 0, 'apart': 0, 'miss': 0, 'contacts': 0}, cs._stepped

    def beside(k, step, v, b, s, ask):
        q = k.quick
        if q is None:
            return real(k, step, v, b, s, ask)
        snap, load = ctypes.string_at(ctypes.addressof(k), ctypes.sizeof(k)), getattr(b, 'load', None)
        got = q[0](q[1], k, b, s, ask)
        if got is q[2]:
            tally['miss'] += 1
            return real(k, step, v, b, s, ask)
        after, load_c = _numbers(k), b.load
        ctypes.memmove(ctypes.addressof(k), snap, len(snap))
        b.load, k.quick = load, None
        try:
            mine = real(k, step, v, b, s, ask)
        finally:
            k.quick = q
        tally['apart'] += _apart(got, mine) or _apart(load_c, b.load) or after != _numbers(k)
        tally['calls'] += 1
        tally['contacts'] += len(s.get('contacts', ()))
        return mine
    cs._stepped = beside
    try:
        r = told.Run('bed_behind')
        r.tell(say='lie_bed')
        r.go(LIE_S)
        r.tell(say='rise_bed')
        r.go(RISE_S)
        s = worker.session(walks=True)
        for _ in range(PASSES):
            worker.stepped(s)
    finally:
        cs._stepped = real
    report.check('its step asked in C as by Python, %(calls)d calls over %(contacts)d contacts (%(miss)d missed)' % tally,
                 tally['calls'] > 3000 and tally['contacts'] > 1000 and not tally['apart'], '%(apart)d apart' % tally)


def test_a_runner_lets_its_part_go(report, c):
    """A part's runner (quick_run.c) holds its Bound and the Bound's leaf, and the Bound holds the runner: the
    collector walks what the runner holds, so a part dropped is let go - as a capsule the runner hid that cycle, and
    every Bound laid outlived its world."""
    import functools
    import gc
    import weakref
    from tools.cores import quick
    q = quick.module()
    if q is None:
        report.check('a runner lets its part go, where the extension is built', True, 'not built here')
        return

    class Part:
        def __init__(self):
            self.runner = functools.partial(q.run, q.runner(self, None, -1, (), (), (), np.array_equal, None, [], self.leaf,
                                                            (), 0, False, (None,) * 6, (), np.zeros(1, np.int64), 0, (),
                                                            (4, None), np.linalg.LinAlgError, quick.MISS, quick.SLOW))

        def leaf(self, _j, _a):
            return True
    part = Part()
    gone = weakref.ref(part)
    del part
    gc.collect()
    report.check('a part\'s runner lets the part go: the collector walks what the runner holds', gone() is None, '')


def test_a_world_laid_anew_lets_those_before_go(report, c):
    """The robot's world laid anew again and again - a restart, a fall's recovery -: the worlds laid before it let
    go by the worker itself, no more of theirs held after six than after three - the last body bound to a part till
    its next call -. A part's runner hid its cycle from the collector, a cache by the world's model and a part
    guarding the robot's edge kept each world's, and a world's cycles waited on the collector's full passes: 34 MB a
    world laid anew."""
    import weakref
    from machine.process import worker
    from machine.world import struck
    s, laid, held = worker.session(walks=True), [], {}
    for k in range(6):
        for _ in range(PASSES // 10):
            worker.stepped(s)
        struck.touching(s['g']['world'])
        laid += [weakref.ref(s['g']['world'].model), weakref.ref(s['g']['body'])]
        worker.apply(s, {'restart': True})
        if k in (2, 5):
            held[k + 1] = sum(w() is not None for w in laid)
    report.check('a world laid anew lets those before it go: %d of their models and bodies held after three, %d after '
                 'six' % (held[3], held[6]), held[6] <= held[3], '')


ROSTER = (test_a_part_called_whole_in_c_as_by_its_glue, test_the_robots_step_asked_in_c_as_by_python,
          test_a_runner_lets_its_part_go, test_a_world_laid_anew_lets_those_before_go)


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
