#!/usr/bin/env python3
"""Her map's look in C (`tools.cores.registry`, wbc_registry.c) beside its numpy twin (`registry.laid(twin=True)`):
looks drawn to take every path - each thing its own slot at once, the loop where two meet one, a full registry's
least salient taken, things moving, unseen, pruned, her eyes' frustum or the whole room - and her lodge's own looks
walking out of its door, as her process passes them and as her vision apart does (its changed rows restacked
alone), every slot's every field and class bit for bit after each; a look's time beside numpy's."""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The draws' looks and their seed; the kinds they are of; her door errand's seconds.
LOOKS, SEED, DOOR_S = 600, 17, 16.0
KINDS = ('chair', 'table', 'lamp', 'bed', 'glass', 'box', 'wall', 'door')


def _same(c, twin):
    """The fields of the C's slots and the twin's apart, bit for bit, and whether their classes match: (names, ok)."""
    a, b = c['rows'], twin['rows']
    apart = [name for name in a if a[name].tobytes() != b[name].tobytes()]
    return apart, c['kinds'] == twin['kinds']


def _turn(rng):
    q, _r = np.linalg.qr(rng.standard_normal((3, 3)))
    return q * np.sign(np.linalg.det(q))


def _head(rng):
    yaw, pitch = rng.uniform(-math.pi, math.pi), rng.uniform(-0.3, 0.3)
    w, x, y, z = (math.cos(pitch / 2) * math.cos(yaw / 2), math.sin(pitch / 2) * math.cos(yaw / 2),
                  math.cos(pitch / 2) * math.sin(yaw / 2), -math.sin(pitch / 2) * math.sin(yaw / 2))
    return {'pos': np.array([rng.uniform(-2, 2), 1.5, rng.uniform(-2, 2)]), 'quat': np.array([w, x, y, z])}


def _thing(rng, k):
    """A thing drawn: its kind, centre, half sizes, rotation, its id (none for a third) and whether over it is clear."""
    return [KINDS[k % len(KINDS)], rng.uniform(-4, 4, 3) * (1.0, 0.3, 1.0) + (0.0, 0.6, 0.0), rng.uniform(0.03, 0.8, 3),
            _turn(rng), int(k + 1) if k % 3 else 0, float(rng.integers(2))]


def _drawn(rng, things, t):
    """The things as a look sees them `t` on: some moved a little, a few a lot, some unseen, some new; a twin of one
    beside it at times (the loop's path)."""
    for th in things:
        th[1] = th[1] + rng.normal(0.0, 0.004, 3) + (rng.normal(0.0, 0.3, 3) if rng.random() < 0.02 else 0.0)
        th[2] = np.abs(th[2] + rng.normal(0.0, 0.002, 3)) + 0.01
    if rng.random() < 0.3:
        things.append(_thing(rng, len(things) + int(1e3 * t)))
    look = [tuple(th) for th in things if rng.random() > 0.15]
    if look and rng.random() < 0.2:
        kind, at, half, R, _i, clear = look[int(rng.integers(len(look)))]
        look.append((kind, at + rng.normal(0.0, 0.01, 3), half, R, 0, clear))
    return look


def test_her_map_looks_in_c_as_in_numpy(report, c):
    """Drawn looks through both, passed and seen in turn: every field and class bit for bit after each - the loop's,
    a full registry's eviction and the frustum's paths all taken."""
    from machine.eyes import registry
    rng = np.random.default_rng(SEED)
    got, twin = registry.laid(), registry.laid(twin=True)
    things, t, worst, paths = [_thing(rng, k) for k in range(40)], 0.0, [], {'full': 0, 'moving': 0, 'pruned': 0}
    for k in range(LOOKS):
        dt = float(rng.choice((0.009, 0.05, 0.4)))
        t += dt
        head, everywhere = _head(rng), bool(k % 3 == 0)
        if k == LOOKS // 2:
            things = [_thing(rng, 5000 + j) for j in range(registry.SLOTS)]
        look = _drawn(rng, things, t)[:registry.SLOTS]
        for slots in (got, twin):
            registry.passed(slots, head, dt)
            registry.seen(slots, look, head, t, everywhere=everywhere)
        apart, alike = _same(got, twin)
        if apart or not alike:
            worst.append((k, apart, alike))
        r = twin['rows']
        paths['full'] += int(r['on'].sum()) == registry.SLOTS
        paths['moving'] += int((r['moving'] > 0).sum() > 0)
        paths['pruned'] += int(((r['on'] == 0) & (r['salience'] != 0)).sum() > 0)
    report.check('her map looks in C as in numpy, every field and class bit for bit after each of %d looks' % LOOKS,
                 not worst and all(paths.values()), 'apart %s; looks full %d, a thing moving %d, pruned %d'
                 % (worst[:3], paths['full'], paths['moving'], paths['pruned']))


def test_her_lodge_looks_in_c_as_in_numpy(report, c):
    """Her lodge's looks as she walks out of its door (`told`), her registry's C and a numpy twin fed each, and a third
    fed as her vision apart is (`vision.main`: the rows not the same objects as the last look's, kept and restacked
    alone): every field and class bit for bit after each look; a look's time beside numpy's."""
    from machine.eyes import registry, vision
    from machine.world import room
    from tools.sim import told
    twins, spent, worst = {}, ([], []), []
    start = vision.Here.start
    changer = vision.Apart.__new__(vision.Apart)
    changer._looks = []

    def beside(self, g, looks, head, dt, everywhere, at=None):
        twin, apart_ = twins.get(id(g['registry']), (None, None))
        if twin is None:
            twin, apart_ = twins[id(g['registry'])] = registry.laid(twin=True), (registry.laid(), {'looks': []})
        t = g['t'] if at is None else at
        t0 = time.perf_counter()
        start(self, g, looks, head, dt, everywhere, at)
        t1 = time.perf_counter()
        registry.passed(twin, head, dt)
        registry.seen(twin, looks, head, t, everywhere=everywhere)
        t2 = time.perf_counter()
        changed, (slots, kept) = changer._changed(looks), apart_
        kept['looks'] = kept['looks'] if len(kept['looks']) == len(looks) else [None] * len(looks)
        for k, look in changed.items():
            kept['looks'][k] = look
        registry.passed(slots, head, dt)
        registry.seen(slots, kept['looks'], head, t, everywhere=everywhere, kept=kept, changed=changed)
        spent[0].append(t1 - t0)
        spent[1].append(t2 - t1)
        apart, alike = _same(g['registry'], twin)
        restacked, kinds = _same(slots, twin)
        if apart or not alike or restacked or not kinds:
            worst.append((round(g['t'], 3), apart, alike, restacked, kinds))
    vision.Here.start = beside
    try:
        r = told.Run(room=True)
        r.tell(say='exit_door')
        r.go(DOOR_S, lambda q: float(q.s['g']['world'].data.qpos[2]) > room.TO)
    finally:
        vision.Here.start = start
    report.check("her lodge's looks in C as in numpy walking out of its door, passed whole and restacked as her vision "
                 "apart's, every field and class bit for bit",
                 len(spent[0]) > 400 and not worst, '%d looks, apart %s' % (len(spent[0]), worst[:3]))
    us = [1e6 * float(np.median(each)) for each in spent]
    report.check("a look of her lodge in C on this host", us[0] < us[1], '%.0f us a look, numpy %.0f' % tuple(us))


ROSTER = (test_her_map_looks_in_c_as_in_numpy, test_her_lodge_looks_in_c_as_in_numpy)


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
