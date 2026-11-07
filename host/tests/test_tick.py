#!/usr/bin/env python3
"""Her drives' pass in one C call (`tools.cores.tick`, wbc_pass_tick; in her pass's, `tools.cores.beat`, her felt before
it between her law's passes) beside its python (`motion.drives_twin`, edge.felt, observer.felt, inekf.felt) - from the
same state at every STRIDE-th pass lying into a bed, resting limp there, rising and walking, every array, her master
bus and the numbers it writes the same bits -, and a move's pass (`tools.cores.played`) beside its python's."""
import ctypes
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: Her lie into the bed and her rise off it, s; her walk after, passes; a pass compared each STRIDE.
LIE_S, RISE_S, WALK_PASSES, STRIDE = 24.0, 8.0, 6000, 7
#: Each move played, s.
MOVE_S = 2.0
#: Her numbers the pass writes on `g`.
NUMBERS = ('surety', 'surety_vec', 'surety_probe', 'left', '_wake')


def _written(g):
    """The arrays her pass writes in place: her drives', her clamps told, her master bus's, her edge's, her filter's
    sums and her observer's."""
    _p, kept, _step = g['drives']
    _k, _bus, arrays = g['bus']
    o = g['observer']['c']
    return ([g[name] for name in ('eso', 'ref_q', 'ref_v', 'hold')] + [kept[name] for name in ('sb', 'feed', 'limit')]
            + [g['cmd']['limit']] + list(arrays.values()) + [g['sense'], g['inekf']['imu'], g['inekf']['span']]
            + [np.ctypeslib.as_array(o.acc), np.ctypeslib.as_array(o.tau)])


def _held(g):
    """What her pass writes, as it is: its arrays, her master bus's state, her observer's span, her numbers."""
    _k, bus, _arrays = g['bus']
    return [a.copy() for a in _written(g)] + [ctypes.string_at(ctypes.addressof(bus), ctypes.sizeof(bus)),
                                              g['observer']['c'].span, tuple(g.get(k) for k in NUMBERS)]


def _put(g, was):
    """`g` as `_held` held it."""
    _k, bus, _arrays = g['bus']
    for a, held in zip(_written(g), was):
        a[...] = held
    ctypes.memmove(ctypes.addressof(bus), was[-3], ctypes.sizeof(bus))
    g['observer']['c'].span = was[-2]
    for k, v in zip(NUMBERS, was[-1]):
        g.pop(k, None) if v is None else g.__setitem__(k, v)


def _apart(a, b):
    """Whether two things held differ: an array's bits, the bus's bytes, a number's repr."""
    if isinstance(a, np.ndarray):
        return a.dtype != b.dtype or a.tobytes() != b.tobytes()
    return a != b if isinstance(a, bytes) else repr(a) != repr(b)


def test_her_drives_pass_in_c_as_in_python(report, _c):
    """Her drives' pass in one C call - her pass's, her felt in it between her law's passes - and in its python from
    the same state - each pass of the two compared, the C's kept -: lying into a bed and resting limp there, rising
    off it, walking."""
    from machine.law import motion
    from machine.process import worker
    from machine.senses import edge, inekf, observer
    from tools.sim import told
    tally, real = {'n': 0, 'passes': 0, 'apart': 0, 'limp': 0, 'held': 0, 'felt': 0}, motion.drives

    def beside(g, dt, step=None):
        tally['n'] += 1
        if tally['n'] % STRIDE:
            return real(g, dt, step)
        was = _held(g)
        if step is g['beat'][2]:
            observer.felt(g['observer'], edge.felt(g['world'], g['sense']), dt)
            inekf.felt(g['inekf'], g['sense'], dt)
            tally['felt'] += 1
        motion.drives_twin(g, dt)
        twin = _held(g)
        _put(g, was)
        real(g, dt, step)
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
    report.check('her drives pass in C as in python, %(passes)d passes (%(felt)d felt in it, %(limp)d limp, %(held)d '
                 'holding WEP)' % tally, tally['passes'] > 3000 and tally['felt'] > 1500 and tally['limp'] > 100
                 and not tally['apart'], '%(apart)d apart' % tally)


def test_a_moves_pass_in_c_as_in_python(report, _c):
    """Each of her C moves played lying in a bed - her bed's exit, her scoot, her bed's entry, MOVE_S each -, its step
    and its joints' setpoints and clamps in one C call (`tools.cores.played`, wbc_played_step) beside its python's from
    the same state, the C's pass with its drive kept: the move's state, command, setpoints and clamps the same bits,
    its done the same."""
    from machine.process import joints, worker
    from machine.senses import edge
    from tools.cores import played
    from tools.sim import told
    tally, real = {'passes': 0, 'apart': 0, 'moves': set()}, played.laid

    def laid(g, world, p):
        made = real(g, world, p)
        if made is None:
            return None
        lib, arrays = played.typed(), (p['state'], p['cmd'], p['target'], p['limit'])

        def both(at, t, dt, buf_at):
            senses, held = [float(x) for x in made[3][:len(p['sensed'])]], [a.copy() for a in arrays]
            done = lib.wbh_played_step(at, t, dt, buf_at)
            got = [a.copy() for a in arrays]
            for a, h in zip(arrays, held):
                a[...] = h
            mine = p['move'].step(p['state'], p['cmd'], np.degrees(g['sense'][edge.Q]), t, dt, *senses)
            if not mine:
                np.radians(np.take(p['cmd'], p['joints'], out=p['target']), out=p['target'])
                np.take(g['clamp'], p['joints'], out=p['limit'])
            tally['apart'] += mine != done or any(a.tobytes() != b.tobytes() for a, b in zip(arrays, got))
            tally['passes'] += 1
            tally['moves'].add(p['move'].__name__.rsplit('.', 1)[-1])
            for a, h in zip(arrays, held):
                a[...] = h
            return made[1](at, t, dt, buf_at)
        return (made[0], both) + made[2:]
    played.laid = laid
    try:
        r = told.Run('bed_behind')
        r.tell(say='lie_bed')
        r.go(12.0)
        for move in joints.MOVES:
            worker.apply(r.s, {'joints': move})
            r.go(MOVE_S)
            if r.s['g'].get('played') is not None:
                joints.ended(r.s['g'])
    finally:
        played.laid = real
    report.check("a move's pass in C as in python, %d passes of %s" % (tally['passes'], ', '.join(sorted(tally['moves']))),
                 tally['passes'] > 1000 and len(tally['moves']) == len(joints.MOVES) and not tally['apart'],
                 '%(apart)d apart' % tally)


ROSTER = (test_her_drives_pass_in_c_as_in_python, test_a_moves_pass_in_c_as_in_python)


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
