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

    def beside(g, dt, step):
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
            tally['moves'].add(p['name'])
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


#: Her twin worlds' passes; their setpoints swung this far, rad, this often, Hz.
WORLD_PASSES, SWING_RAD, SWING_HZ = 2500, 0.08, 1.3


def test_her_world_steps_in_c_as_in_python(report, _c):
    """Two of her worlds laid alike in her room, their setpoints swung alike a pass at a time - one's steps between
    MuJoCo's in C (`machine.world.steps`), her ankles' sprung gearboxes stepped there, the other's in python (`in_c`
    False) -: MuJoCo's state, ctrl, her motors, gearboxes' peaks and state, meters, DC link, rates and stamps the same
    bits at every pass; MuJoCo's contacts paired in C off its data (`steps.paired`) as python's views pair them."""
    import math
    from machine.body import figure
    from machine.world import physics, steps
    from tools.cores import wbc as cores
    was, tally = (physics.ROOM, physics.CAMERA, physics.SIGHT), {'passes': 0, 'apart': 0, 'pairs': 0, 'paired': 0,
                                                                 'boxed': 0}
    physics.ROOM, physics.CAMERA, physics.SIGHT = 1.0, 0.0, 0.0
    try:
        clock, worlds = [0.0], [physics.World(), physics.World()]
        for w in worlds:
            w.reset(figure.stand(), where=(0.0, figure.standing(), 0.0))
            w.clock = lambda: clock[0]
        a, b = worlds
        b.in_c, base = False, a.target.copy()
        m = a.model
        hers, ground = (m.geom_bodyid > 0).astype(np.bool_), (m.geom_bodyid == 0).astype(np.bool_)
        masks, paired = (cores.address(hers), cores.address(ground)), cores.library().wbh_pass_paired
        for k in range(WORLD_PASSES):
            clock[0] = (k + 1) * physics.STEP_S
            for w in worlds:
                w.target[:] = base + SWING_RAD * math.sin(2.0 * math.pi * SWING_HZ * clock[0])
                w.advance()
            held = [(w.data.qpos, w.data.qvel, w.data.qacc, w.data.ctrl, w.motor, w.geared, w.rate, w.was,
                     *(w.link[n] for n in ('v', 'peak', 'bled', 'id')),
                     *(getattr(w.boxes, n) for n in ('delta', 'w', 'hat', 'p_int', 'drive', 'asked', 'rate', 'q_step',
                                                     'went')),
                     repr((w.effort, w.work, w.brake, w.heat, w.stamp, w.at))) for w in worlds]
            tally['boxed'] += a._steps is not None and a._steps[6][2] is not None
            tally['apart'] += any(_apart(x, y) if isinstance(x, str) else x.tobytes() != y.tobytes()
                                  for x, y in zip(*held))
            d = a.data
            for below in (math.inf, 0.05):
                c = steps.paired(a, *masks, below)
                geom, pos = d.contact.geom, d.contact.pos
                tally['apart'] += c != bool(paired(cores.address(geom), cores.address(pos), geom.strides[0], d.ncon,
                                                   *masks, below))
                tally['pairs'] += 1
                tally['paired'] += c
            tally['passes'] += 1
    finally:
        physics.ROOM, physics.CAMERA, physics.SIGHT = was
    report.check("her world's steps in C as in python, %(passes)d passes, her boxes in C %(boxed)d, her contacts paired "
                 "%(paired)d of %(pairs)d" % tally,
                 tally['passes'] == WORLD_PASSES == tally['boxed'] and tally['paired'] > 100 and not tally['apart'],
                 '%(apart)d apart' % tally)


SKIN_PASSES, SHOVE_N = 1500, 40.0


def test_her_skin_reads_her_contacts_in_c_as_their_views(report, _c):
    """Her world swung a pass at a time, her skin's bins filled twice each pass: off MuJoCo's data in one C call
    (wbc_world_contacted, `edge.contacted`'s) and through its contacts', forces' and impedances' numpy views
    (wbc_pass_contacts, as her edge read them before) - every bin the same bits, the contacts' layout the same, and a
    shove told where the world applies a force to a body, a third of the passes."""
    import math
    from machine.body import figure
    from machine.senses import edge
    from machine.world import physics
    from tools.cores import wbc as cores
    was, tally = (physics.ROOM, physics.CAMERA, physics.SIGHT), {'passes': 0, 'apart': 0, 'contacts': 0, 'shoved': 0}
    physics.ROOM, physics.CAMERA, physics.SIGHT = 1.0, 0.0, 0.0
    try:
        clock, w = [0.0], physics.World()
        w.reset(figure.stand(), where=(0.0, figure.standing(), 0.0))
        w.clock = lambda: clock[0]
        base, d, lib = w.target.copy(), w.data, cores.library()
        skins = [(edge.laid(w), vars(w)['contacts'])[1] for _ in range(2)]     # two skins' bins on her one world
        (c, kept, contacted, _sensed), (c2, kept2, *_rest) = skins
        for k in range(SKIN_PASSES):
            clock[0] = (k + 1) * physics.STEP_S
            w.target[:] = base + SWING_RAD * math.sin(2.0 * math.pi * SWING_HZ * clock[0])
            d.xfrc_applied[w.pelvis, 0] = SHOVE_N * (k % 3 == 0)
            w.advance()
            shoved = contacted(d._address, ctypes.byref(c))
            if d.ncon:
                edge.layout(c2, d.contact)
                lib.wbh_pass_contacts(ctypes.byref(c2), d.ncon, cores.address(d.contact.pos), cores.address(d.efc_force),
                                      cores.address(d.efc_R))
            else:
                lib.wbh_pass_contacts(ctypes.byref(c2), 0, None, None, None)
            laid = [(x.stride, x.off_geom, x.off_frame, x.off_dim, x.off_efc, x.off_friction) for x in (c, c2)]
            tally['apart'] += (any(kept[n].tobytes() != kept2[n].tobytes() for n in kept) or bool(shoved) != bool(d.xfrc_applied.any())
                               or (d.ncon > 0 and laid[0] != laid[1]))
            tally['contacts'] += d.ncon > 0
            tally['shoved'] += shoved
            tally['passes'] += 1
    finally:
        physics.ROOM, physics.CAMERA, physics.SIGHT = was
    report.check("her skin's bins off MuJoCo's data in C as through its views, %(passes)d passes, %(contacts)d with "
                 "contacts, %(shoved)d shoved" % tally,
                 tally['passes'] == SKIN_PASSES and tally['contacts'] > 1000 and tally['shoved'] == SKIN_PASSES // 3
                 and not tally['apart'], '%(apart)d apart' % tally)


AHEAD_PASSES = 2000


def test_her_step_begun_beside_her_law_as_whole(report, _c):
    """Two of her worlds laid alike, their setpoints swung alike a pass at a time: one's steps each begun beside
    (`steps.ahead`: MuJoCo's mj_step1 on a thread of her C's, mj_step2 at its step), the other's whole - MuJoCo's
    state, ctrl, poses, contacts and constraint forces the same bits at every pass; a step begun and not taken, as
    where her pass raised, is joined (`steps.behind`) and her next step whole, a seventh of them."""
    import math
    from machine.body import figure
    from machine.world import physics, steps
    from tools.cores import world as c_world
    was, tally = (physics.ROOM, physics.CAMERA, physics.SIGHT), {'passes': 0, 'apart': 0, 'beside': 0, 'dropped': 0}
    physics.ROOM, physics.CAMERA, physics.SIGHT = 1.0, 0.0, 0.0
    try:
        clock, worlds = [0.0], [physics.World(), physics.World()]
        for w in worlds:
            w.reset(figure.stand(), where=(0.0, figure.standing(), 0.0))
            w.clock = lambda: clock[0]
        a, b = worlds
        base = a.target.copy()
        for k in range(AHEAD_PASSES):
            clock[0] = (k + 1) * physics.STEP_S
            for w in worlds:
                w.target[:] = base + SWING_RAD * math.sin(2.0 * math.pi * SWING_HZ * clock[0])
            steps.ahead(a)
            tally['beside'] += bool(vars(a)['_ahead'])
            if k % 7 == 3:
                tally['dropped'] += steps.behind(a)
            a.advance()
            b.advance()
            held = [(d.qpos, d.qvel, d.qacc, d.ctrl, d.xpos, d.contact.pos, d.efc_force) for d in (a.data, b.data)]
            tally['apart'] += a.data.ncon != b.data.ncon or any(x.tobytes() != y.tobytes() for x, y in zip(*held))
            tally['passes'] += 1
    finally:
        physics.ROOM, physics.CAMERA, physics.SIGHT = was
    told = c_world.staged() is not None
    report.check("her world's steps begun beside as whole, %(passes)d passes, %(beside)d begun beside, %(dropped)d "
                 "joined untaken" % tally,
                 tally['passes'] == AHEAD_PASSES and not tally['apart']
                 and (not told or (tally['beside'] == AHEAD_PASSES and tally['dropped'] == AHEAD_PASSES // 7 + 1)),
                 '%(apart)d apart' % tally)


CONTROL_ARRAYS = ('y', 'fr', 'a', 'st', 'out', 'posture', 'jeff', 'leg_kp', 'release', 'seat_f', 'words', 'twisted',
                  'tab')
CONTROL_NUMBERS = ('heading', 'surety', 'surety_probe', 'chained', '_w_back', '_backing_yaw', '_twist_clear')
CONTROL_SKIP = frozenset({'world', 'body', 'drives', 'bus', 'cmd', 'tick', 'beat', 'gym', 'situations', 'planner',
                          'vision', 'keep', 'heat', 'schema_body', 'got', 'ask', 'solve_in', 's'})


def _sync_control(g1, g2, skip):
    """Copy `motion.control`'s live input state from `g1` into `g2` before a paired control pass."""
    import copy
    for k, v in g1.items():
        if k in skip:
            continue
        if isinstance(v, np.ndarray):
            if k in g2 and isinstance(g2[k], np.ndarray) and g2[k].shape == v.shape:
                g2[k][...] = v
            else:
                g2[k] = v.copy()
        elif isinstance(v, dict):
            if k not in g2 or not isinstance(g2[k], dict):
                g2[k] = {}
            for dk, dv in v.items():
                if isinstance(dv, np.ndarray):
                    if dk in g2[k] and isinstance(g2[k][dk], np.ndarray) and g2[k][dk].shape == dv.shape:
                        g2[k][dk][...] = dv
                    else:
                        g2[k][dk] = dv.copy()
                elif ctypes.Structure in getattr(type(dv), '__mro__', ()):
                    ctypes.memmove(ctypes.addressof(g2[k][dk]), ctypes.addressof(dv), ctypes.sizeof(dv))
                elif isinstance(dv, (int, float, bool, str, tuple, list, dict, type(None))):
                    g2[k][dk] = copy.deepcopy(dv)
        elif isinstance(v, list):
            g2[k] = copy.deepcopy(v)
        else:
            g2[k] = v
    for dk, dv in g1['drives'][1].items():
        g2['drives'][1][dk][...] = dv
    g2['cmd']['limit'][...] = g1['cmd']['limit']


def test_her_motion_control_in_c_is_her_python_twins(report, _c):
    """Each pass of `motion.control` on `motion.laid`'s C and compiled hooks beside a twin session rebound to
    `tools.cores.motion.twins` after `gc.collect()` from the same state - through an open-chain walk and a closed-chain
    bed lie (`chained` 0 and 1): every array and scalar written matches bit for bit (`exact_host`) or within 1e-11."""
    import gc
    from machine.law import motion
    from machine.process import worker
    from tools.cores import motion as c_motion
    from tools.sim import told
    skip = CONTROL_SKIP | set(c_motion.HOOKS)
    tally, chains, pair, real = {'passes': 0, 'apart': 0, 'err': 0.0}, set(), [], motion.control

    def beside(g, dt):
        if not pair or g is not pair[0]:
            return real(g, dt)
        g2 = pair[1]
        _sync_control(g, g2, skip)
        real(g, dt)
        real(g2, dt)
        chains.add(int(g.get('chained', 0)))
        tally['passes'] += 1
        arrs = [(g[k], g2[k]) for k in CONTROL_ARRAYS] + [
            (g['drives'][1]['tau'], g2['drives'][1]['tau']), (g['drives'][1]['qacc'], g2['drives'][1]['qacc']),
            (g['cmd']['feed'], g2['cmd']['feed'])]
        for a, b in arrs:
            tally['apart'] += _apart(a, b)
            d = np.abs(a - b)
            tally['err'] = max(tally['err'], float(np.nanmax(d)) if np.any(~np.isnan(d)) else 0.0)
        for k in CONTROL_NUMBERS:
            v1, v2 = float(g.get(k, 0.0)), float(g2.get(k, 0.0))
            tally['apart'] += repr(v1) != repr(v2)
            tally['err'] = max(tally['err'], abs(v1 - v2))
    motion.control = beside
    try:
        s1, s2 = worker.session(walks=True), worker.session(walks=True)
        c_motion.twins(s2['g'])
        gc.collect()
        pair[:] = [s1['g'], s2['g']]
        for _ in range(900):
            worker.stepped(s1)
        r1, r2 = told.Run('bed_behind'), told.Run('bed_behind')
        c_motion.twins(r2.s['g'])
        gc.collect()
        pair[:] = [r1.s['g'], r2.s['g']]
        r1.tell(say='lie_bed')
        r1.go(8.2)
    finally:
        motion.control = real
    exact = c_motion.exact_host()
    ok = tally['passes'] >= 3000 and {0, 1} <= chains and (not tally['apart'] if exact else tally['err'] <= 1e-11)
    report.check('her motion.control in C is her python twins, %d passes (chained %s)'
                 % (tally['passes'], sorted(chains)), ok, '%(apart)d apart, %(err).3g max' % tally)


ROSTER = (test_her_drives_pass_in_c_as_in_python, test_a_moves_pass_in_c_as_in_python,
          test_her_world_steps_in_c_as_in_python, test_her_skin_reads_her_contacts_in_c_as_their_views,
          test_her_step_begun_beside_her_law_as_whole, test_her_motion_control_in_c_is_her_python_twins)


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
