#!/usr/bin/env python3
"""A bearing sole does not drag: the robot's soles' slide under load as it walks, by the world's truth.

    python tools/sim/drag.py                                   # the first texts' walk, the robot's own, the world off
    python tools/sim/drag.py texts --against C:/sources/coaxial_ab   # a case on another tree too, beside
    python tools/sim/drag.py off --rows 20 --from 4 --to 6     # ... and a row every 20 ms of that span
    python tools/sim/drag.py off@1 off@2 off@3                 # the world drawn off by other seeds

A case walks the robot: `texts` the first texts' 101, 102 and 103 as test_ortho says them and `walk` the robot's
own walk from rest (`sprung.WALK`), both on its latent loop (`tools.sim.ortho`'s: `sprung.laid`, `sprung.passed`);
`off` its walk on the gym's world built off the figure as test_sysid_live walks it (`worker.session`, test_sysid's
`_off`). Each pass each sole's contacts are read as MuJoCo's last step solved them: the load a sole bears, N, the
friction along its plane, N, and its slide - its contacts' friction rows' speeds on their frame's tangents, each
weighed by its load. A sole bears past BEARS of the robot's weight. A case's line: its greatest tilt, how far
the robot went; each sole's seconds borne, the path slid under load, mm, its mean slide, mm/s, its mean friction
over its load, and the same slide as the law senses it (the sole's own speed along the floor off the law's
state); `--rows` a row every that many ms: what the robot does, its tilt, deg, and each sole's stance as the law
has it (S), load, friction, slide and the slide the law senses, mm/s; then the law's own of that pass: each sole's
load as it senses it and as its step plans it, N, its swing's and its heel's shares, and the acceleration it asks
of each sole in the air (`modal.SOLE_ACC`).
"""
import argparse
import json
import os
import sys

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, HOST)

#: The first texts as test_ortho says them, and how long a case walks past its last word, s.
TEXTS, PAST_S = ('#101@1', '#102@3', '#103@6'), 5.8
#: The world off the figure: test_sysid_live's seed, the twist said at 1 s and its seconds.
SEED, TWIST, OFF_S = 7, (0.69, 0.0, 0.0), 10.0
#: A sole bears past this share of the robot's weight; fallen past this tilt, deg.
BEARS, HELD_DEG = 0.2, 25.0
CASES = ('texts', 'walk', 'off')


def soles(world, feet):
    """Each sole's (load N, friction along its plane N, slide m/s (3,) world) as `world`'s last step solved its
    contacts: `feet` each side's bodies; a sole's slide its contacts' weighed by their loads."""
    import numpy as np
    d, m = world.data, world.model
    n = d.ncon
    out = [(0.0, 0.0, np.zeros(3)), (0.0, 0.0, np.zeros(3))]
    if not n:
        return out
    con = d.contact
    on = (con.efc_address[:n] >= 0) & (con.dim[:n] >= 3)
    at, frame = con.efc_address[:n][on], con.frame[:n][on].reshape(-1, 3, 3)
    along = d.efc_vel[at + 1][:, None] * frame[:, 1] + d.efc_vel[at + 2][:, None] * frame[:, 2]
    pull = d.efc_force[at + 1][:, None] * frame[:, 1] + d.efc_force[at + 2][:, None] * frame[:, 2]
    load, pairs = d.efc_force[at], m.geom_bodyid[con.geom[:n][on]]
    for k, own in enumerate(feet):
        mine = np.isin(pairs[:, 0], own) | np.isin(pairs[:, 1], own)
        borne = float(load[mine].sum())
        if borne > 0.0:
            out[k] = (borne, float(np.linalg.norm(pull[mine].sum(axis=0))), (load[mine] @ along[mine]) / borne)
    return out


def looped(case):
    """(g, a pass - what ended the walk or None -, the walk's last second) of a case on the robot's latent loop."""
    from machine.errors import MachineError
    from machine.process import sprung
    from tools.sim import ortho as run
    if case == 'texts':
        with open(os.path.join(HOST, 'lisp', 'humanoid', 'examples.lisp'), encoding='utf-8') as f:
            plan = run.planned(list(TEXTS), f.read())
    else:
        plan = run.planned(['%s@%g' % entry for entry in sprung.WALK])
    g = sprung.laid(sprung.STACK_EVERY)
    for at, label, wire, names in sorted(plan, key=lambda item: item[0]):
        sprung.said(g, label, wire, names, at)

    def passed():
        try:
            sprung.passed(g)
        except MachineError as error:
            return str(error)
        return None
    return g, passed, max(item[0] for item in plan) + PAST_S


def offed(seed=SEED):
    """(g, a pass, the walk's last second) of the robot's walk on the gym's world off the figure: test_sysid_live's
    run, its twist said at 1 s; the world drawn off by `seed`."""
    from machine.body import mjcf
    from machine.identify import sysid
    from machine.process import worker
    from machine.stack import wbc
    from machine.world import physics
    sys.path.insert(0, os.path.join(HOST, 'tests'))
    import test_sysid as case
    physics.ROOM, physics.SIGHT, mjcf.STAND = 0.0, 0.0, 0.0
    try:
        s = worker.session()
    finally:
        mjcf.STAND = 1.0
    body = wbc.Body(identified=False)
    case._off(s['g']['world'], {'body': body, 'lay': sysid.laid(body)}, seed, keep=True)
    told = []

    def passed():
        if s['g']['t'] >= 1.0 and not told:
            told.append(worker.apply(s, {'twist': TWIST}))
        worker.stepped(s)
        return 'fell at %.3f s' % s['fallen_at'] if s['fallen_at'] is not None or s['epoch'] != 0 else None
    return s['g'], passed, OFF_S


def walked(case, rows=0, span=(0.0, 1e9)):
    """A case's reading: {fell, tilt deg, went m, soles [[s borne, mm slid, mm/s, friction over load, mm/s as the
    law senses it]], rows}: the robot walked, each sole read a pass (`soles`)."""
    import numpy as np
    from machine.law import modal
    from machine.process import sprung
    g, passed, last = offed(int(case[4:] or SEED)) if case.startswith('off') else looped(case)
    world = g['world']
    m, d = world.model, world.data
    feet = [[m.body(side + part).id for part in ('_foot', '_toes')] for side in ('left', 'right')]
    pelvis = m.body('pelvis').id
    weight = float(m.body_subtreemass[pelvis]) * abs(float(m.opt.gravity[1]))
    tilt, fell, told, sums, first = 0.0, None, [], np.zeros((2, 5)), d.xpos[pelvis].copy()
    scuff = [0.0, 0.0]
    while g['t'] < last and fell is None:
        now = float(g['t'])
        fell = passed()
        tilt = max(tilt, float(sprung.tilt(g)))
        if fell is None and tilt > HELD_DEG:
            fell = 'tilted at %.3f s' % now
        s, ask, cells = g['s'], g['ask'], []
        for k, (borne, pull, slide) in enumerate(soles(world, feet)):
            sensed = float(np.hypot(*(s['feet'][k]['J6'] @ s['v'])[[3, 5]]))
            speed = float(np.hypot(slide[0], slide[2]))
            if not ask['stance'][k]:
                # a sole the law let go that still bears: its friction's impulse and its peak
                scuff[0] += pull * float(m.opt.timestep)
                scuff[1] = max(scuff[1], pull)
            if borne > BEARS * weight:
                sums[k] += (1.0, speed, pull / borne, sensed, 0.0)
            cells.append('%s %4.0f N %4.0f %6.1f (%6.1f)' % ('S' if ask['stance'][k] else '-', borne, pull,
                                                              1e3 * speed, 1e3 * sensed))
        if rows and span[0] <= now <= span[1] and int(round(now * 1000)) % rows == 0:
            a, acc = g['a'], g['a'][modal.SOLE_ACC].reshape(2, 3)
            told.append('%7.3f %-10s %4.1f | %s | %s | law N %3.0f %3.0f plans %3.0f %3.0f swing %.2f %.2f heel %.2f %.2f '
                        'acc %+5.1f %+5.1f %+5.1f, %+5.1f %+5.1f %+5.1f' % (
                            now, sprung.doing(g)[:10], sprung.tilt(g), cells[0], cells[1], *g['loads'],
                            *g.get('got', {}).get('bears', (0.0, 0.0)), *a[modal.SWINGS], *a[modal.HEELED], *acc[0], *acc[1]))
    passes = np.maximum(sums[:, 0], 1.0)
    went = float(np.hypot(*(d.xpos[pelvis] - first)[[0, 2]]))
    return {'fell': fell, 'tilt': round(tilt, 1), 'went': round(went, 3), 'rows': told,
            'scuff_ns': round(scuff[0], 2), 'scuff_n': round(scuff[1], 1),
            'soles': [[round(1e-3 * float(sums[k, 0]), 2), round(float(sums[k, 1]), 1),
                       round(1e3 * float(sums[k, 1] / passes[k]), 2), round(float(sums[k, 2] / passes[k]), 3),
                       round(1e3 * float(sums[k, 3] / passes[k]), 2)] for k in range(2)]}


def line(name, got):
    """A case's line: its greatest tilt, how far the robot went and each sole's drag."""
    each = '  '.join('%s %.2f s %.1f mm %.2f mm/s f/N %.3f (law %.2f mm/s)' % (side, *row)
                     for side, row in zip('lr', got['soles']))
    return '%-22s tilt %4.1f  %.2f m  %s%s' % (name, got['tilt'], got['went'], each,
                                               '  FELL: %s' % got['fell'] if got['fell'] else '')


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('cases', nargs='*', help='cases: %s' % ', '.join(CASES))
    p.add_argument('--against', help='another tree (its root), each case there too')
    p.add_argument('--jobs', type=int, default=6, help='cases side by side (`tools.dev.focus.relay`)')
    p.add_argument('--rows', type=int, default=0, help='a row every that many ms')
    p.add_argument('--from', dest='start', type=float, default=0.0, help='rows from this second')
    p.add_argument('--to', type=float, default=1e9, help='rows to this second')
    p.add_argument('--one', nargs=2, help=argparse.SUPPRESS)
    args = p.parse_args(argv)
    if args.one:
        root, case = args.one
        sys.path[:0] = [os.path.join(root, 'host')]
        os.environ['PYTHONPATH'] = os.path.join(root, 'host')
        print('DRAG ' + json.dumps(walked(case, args.rows, (args.start, args.to))))
        return 0
    from tools.dev import focus
    cases = args.cases or list(CASES)
    unknown = [case for case in cases if case not in CASES and not (case[:4] == 'off@' and case[4:].isdigit())]
    if unknown:
        p.error('no case %s' % ', '.join(unknown))
    trees = [os.path.dirname(HOST)] + ([os.path.abspath(args.against)] if args.against else [])
    work = [focus.Job('%s %s' % (os.path.basename(tree), case),
                      [sys.executable, '-X', 'utf8', os.path.abspath(__file__), '--rows', str(args.rows), '--from',
                       str(args.start), '--to', str(args.to), '--one', tree, case], timeout=900.0)
            for case in cases for tree in trees]
    for job, text, _code, _s in focus.relay(work, batons=args.jobs):
        got = next((ln[5:] for ln in (text or '').splitlines() if ln.startswith('DRAG ')), None)
        print(line(job.name, json.loads(got)) if got else '%-22s no run: %s' % (
            job.name, (text or 'out of time')[-300:].replace('\n', ' | ')), flush=True)
        for row in (json.loads(got)['rows'] if got else []):
            print('    ' + row, flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
