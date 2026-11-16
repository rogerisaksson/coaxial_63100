#!/usr/bin/env python3
"""What rests does not creep: the robot's and its room's things' drift at rest, by the world's truth.

    python tools/sim/creep.py                                   # stood, sat and lain on the bench and in the lodge
    python tools/sim/creep.py sat lodge_lain                    # those cases
    python tools/sim/creep.py --against C:/sources/coaxial_ab   # each case on another tree too, beside
    python tools/sim/creep.py sat --rows                        # ... and each pair's contact's rows as MuJoCo laid them
    python tools/sim/creep.py --json                            # each case whole as JSON

A case rests the robot - stood on the flat floor, sat on the bench's chair, lain on the bench's bed (`scenarios`:
their first word alone, held REST_S + READ_S past where their second is said), sat on the lodge's chair and lain on
the lodge's bed from the robot's dock (`house_watch.Watch`) - and reads its world a pass over its last READ_S. Each body's drift along the
floor, mm/s (the slope of a line through its places, least squares), and how far it wandered, mm: the robot's
pelvis and feet, and every loose thing of the room with whether it slept (`loose`). And what slides on what: each
pair of bodies' contacts' sliding along their plane as MuJoCo's last step solved it (their friction rows' speeds on
their frame's tangents), its mean over the read - the pair's creep, mm/s -, its peak and the load borne, N. And
which of the robot's joints turn: each joint's drift, mdeg/s (a line through its angles), the torque its drive
gives in the mean, N m, and whether its friction row holds (MuJoCo's frictionloss: its force under its bound). A
case's line names the robot's own, the worst loose thing, the pairs that creep most and the joints that turn most. A case a job on the relay
(`tools.dev.focus.relay`).
"""
import argparse
import json
import os
import sys

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, HOST)

#: A case rests this long before it is read, and is read this long, s.
REST_S, READ_S = 6.0, 8.0
#: The bench's cases, each the scenario it rests in (`scenarios.SCENARIOS`): None the flat floor, stood.
BENCH = {'stood': None, 'sat': 'chair', 'lain': 'bed'}
#: The lodge's cases, each the verb said at the robot's dock (`house.GOALS`).
LODGE = {'lodge_sat': 'sit_chair', 'lodge_lain': 'lie_bed'}
#: The robot's own bodies read.
OWN = ('pelvis', 'left_foot', 'right_foot')
#: A pair is read where it bears this much, N, in the mean; the pairs a case tells.
BORNE_N, TOLD = 1.0, 6


def bodies(world):
    """{name: body} of the robot's own read (OWN) and of the room's loose things in `world`."""
    from machine.world import loose
    m = world.model
    named = {name: m.body(name).id for name in OWN}
    named.update({m.body(b).name: int(b) for b, _joint in loose.things(m)})
    return named


def drift(times, places):
    """(mm/s along the floor, mm wandered) of a body's `places` (n, 3) at `times`: up is y."""
    import numpy as np
    t, p = np.asarray(times, float), np.asarray(places, float)
    speed = np.hypot(np.polyfit(t, p[:, 0], 1)[0], np.polyfit(t, p[:, 2], 1)[0])
    return round(1e3 * float(speed), 5), round(1e3 * float(np.hypot(np.ptp(p[:, 0]), np.ptp(p[:, 2]))), 4)


def slid(world, into):
    """`world`'s contacts' sliding as its last step solved them, summed into `into` {(body, body): [passes, N,
    m/s (3,), peak m/s]}: each pair of bodies' contacts' mean sliding along their plane and the load they bear."""
    import numpy as np
    d, m = world.data, world.model
    n = d.ncon
    if not n:
        return
    con = d.contact
    on = (con.efc_address[:n] >= 0) & (con.dim[:n] >= 3)
    at, frame = con.efc_address[:n][on], con.frame[:n][on].reshape(-1, 3, 3)
    along = d.efc_vel[at + 1][:, None] * frame[:, 1] + d.efc_vel[at + 2][:, None] * frame[:, 2]
    load, pairs = d.efc_force[at], m.geom_bodyid[con.geom[:n][on]]
    for pair in {(int(a), int(b)) for a, b in pairs}:
        own = (pairs[:, 0] == pair[0]) & (pairs[:, 1] == pair[1])
        row = into.setdefault(pair, [0, 0.0, np.zeros(3), 0.0])
        row[0] += 1
        row[1] += float(load[own].sum())
        row[2] += along[own].mean(axis=0)
        row[3] = max(row[3], float(np.linalg.norm(along[own], axis=1).max()))


def turned(world, kept):
    """The robot's joints' turning over a read `kept` [(t, angles (n,), drives' torques (n,))]: [[joint, mdeg/s,
    N m in the mean]] its TOLD that turn most."""
    import numpy as np
    from machine.body.figure import JOINTS
    t, q = np.array([row[0] for row in kept]), np.degrees(np.array([row[1] for row in kept]))
    rate, drive = 1e3 * np.polyfit(t, q, 1)[0], np.mean([row[2] for row in kept], axis=0)
    return [[JOINTS[k], round(float(rate[k]), 3), round(float(drive[k]), 2)] for k in np.argsort(-abs(rate))[:TOLD]]


def rows(world):
    """Each pair of bodies' first contact's rows as MuJoCo laid them at `world`'s last step: [pair, dim, friction,
    solref, solimp, each row's R, each row's (K, B, impedance)]; before them each of the robot's joints' friction
    row likewise: [[joint, its dof], 1, its bound, ...]."""
    d, m = world.data, world.model
    out, seen = [], set()
    for e in range(d.ne, d.ne + d.nf):
        dof = int(d.efc_id[e])
        out.append([[m.joint(int(m.dof_jntid[dof])).name, str(dof)], 1, [float(d.efc_frictionloss[e])],
                    [float(v) for v in m.dof_solref[dof]], [float(v) for v in m.dof_solimp[dof][:3]],
                    [float(d.efc_R[e])], [[float(v) for v in d.efc_KBIP[e][:3]]]])
    for i in range(d.ncon):
        c = d.contact[i]
        pair = tuple(m.body(int(m.geom_bodyid[g])).name for g in c.geom)
        if c.efc_address < 0 or pair in seen:
            continue
        seen.add(pair)
        at = range(int(c.efc_address), int(c.efc_address) + int(c.dim))
        out.append([list(pair), int(c.dim), [float(v) for v in c.friction[:3]], [float(v) for v in c.solref],
                    [float(v) for v in c.solimp[:3]], [float(d.efc_R[k]) for k in at],
                    [[float(v) for v in d.efc_KBIP[k][:3]] for k in at]])
    return out


def judged(world, kept, named, fell, slides, angles):
    """A case's reading of its `kept` [(t, {name: place})] over its read, and of its pairs' sliding `slides`
    (`slid`): {fell, read s, own {name: [mm/s, mm]}, loose {name: [mm/s, mm, asleep]}, slid [[pair, mm/s creep, mm/s
    peak, N]] its TOLD worst that bear, rows (`rows`)}."""
    import numpy as np
    m = world.model
    times = [row[0] for row in kept]
    read = {name: drift(times, [row[1][name] for row in kept]) for name in named}
    things = world.furnished
    asleep = {int(b): bool(state[1]) for b, state in zip(things[5], things[7])} if things else {}
    crept = sorted(((round(1e3 * float(np.linalg.norm(v)) / n, 5), [m.body(a).name, m.body(b).name],
                     round(1e3 * peak, 4), round(load / n, 1)) for (a, b), (n, load, v, peak) in slides.items()
                    if load / n >= BORNE_N), key=lambda row: row[0], reverse=True)[:TOLD]
    told = [[pair, creep, peak, load] for creep, pair, peak, load in crept]
    return {'fell': fell, 'read_s': round(times[-1] - times[0], 2),
            'own': {name: list(read[name]) for name in OWN},
            'loose': {name: list(read[name]) + [asleep.get(b, False)] for name, b in named.items() if name not in OWN},
            'slid': told, 'turned': turned(world, angles), 'rows': rows(world)}


def read(world, t, named, kept, slides, angles):
    """A pass of `world` at `t` read: its bodies `named` where they are into `kept`, its pairs' sliding into `slides`
    (`slid`), the robot's joints' angles and their drives' torques into `angles`."""
    d = world.data
    kept.append((t, {name: d.xpos[b].copy() for name, b in named.items()}))
    slid(world, slides)
    angles.append((t, d.qpos[world.qadr].copy(), d.ctrl[:len(world.qadr)].copy()))


def unread(fell, borne):
    """A case not read: the robot `fell` before its read began, or its goal was not `borne`."""
    return {'fell': fell, 'read_s': 0.0, 'own': {}, 'loose': {}, 'slid': [], 'turned': [], 'rows': [], 'borne': borne}


def bench(case):
    """A bench case's reading: its scenario's first word alone - the robot rests where it took it -, run REST_S +
    READ_S past where its second is said (none: its end), read each pass of its last READ_S."""
    from tools.sim import scenarios
    from tools.sim.player import played
    if BENCH[case] is None:
        plan, seconds = played('-%g' % (REST_S + READ_S))
        scenarios.SCENARIOS['creep'] = ('flat', plan, seconds, 'stands', None)
    else:
        scene, plan, seconds, verdict, stunt = scenarios.SCENARIOS[BENCH[case]]
        seconds = (plan[1][0] if len(plan) > 1 else seconds) + REST_S + READ_S
        scenarios.SCENARIOS['creep'] = (scene, plan[:1], seconds, verdict, stunt)
    kept, named, seen, slides, angles = [], {}, {}, {}, []

    def laid(s):
        world = seen['world'] = s['g']['world']
        if s['g']['t'] < seconds - READ_S:
            return
        named.update(named or bodies(world))
        read(world, s['g']['t'], named, kept, slides, angles)
    out = scenarios.ran('creep', laid=laid)
    if not kept:
        return unread(out['fell_at'], None)
    return judged(seen['world'], kept, named, out['fell_at'], slides, angles)


def lodge(case):
    """A lodge case's reading: its verb said at the robot's dock for it - its docks in their order till its goal is
    borne -, rested REST_S, read READ_S."""
    from machine.law import modal
    from machine.process import sprung, worker
    from tools.sim import house
    from tools.sim.house_watch import Watch
    verb = LODGE[case]
    goal, _rise, limit, _rise_s = house.GOALS[verb]
    w = borne = None
    for x, z, yaw in house.docked(verb):
        w = Watch(x, z, yaw)
        g = w.g
        worker.apply(w.s, {'say': verb})
        borne = w.go(limit, lambda _w, g=g: sprung.doing(g) == goal and float(g['y'][modal.PROP]) > 0.45)
        if borne and not w.s['fallen_at']:
            break
    if w is None or not borne:
        return unread(w.s['fallen_at'] if w else None, False)
    w.go(REST_S)
    kept, named, slides, angles = [], bodies(w.world), {}, []

    def passed(_w):
        read(w.world, w.g['t'], named, kept, slides, angles)
        return False
    w.go(READ_S, passed)
    return dict(judged(w.world, kept, named, w.s['fallen_at'], slides, angles), borne=True)


def line(name, got):
    """A case's line: the robot's own bodies' drift, the worst loose thing's and the pairs that creep most."""
    if not got['own']:
        return '%-24s not read: fell %s, its goal borne %s' % (name, got['fell'], got.get('borne'))
    own = '  '.join('%s %.4f' % (part.replace('left_', 'l ').replace('right_', 'r '), got['own'][part][0]) for part in OWN)
    worst = max(got['loose'].items(), key=lambda kv: kv[1][0], default=None)
    loose = (' | loose %d, worst %s %.4f%s' % (
        len(got['loose']), worst[0], worst[1][0], ' asleep' if worst[1][2] else '')) if worst else ''
    slides = ' | slides ' + ', '.join('%s/%s %.4f (peak %.3f, %.0f N)' % (a, b, creep, peak, load)
                                      for (a, b), creep, peak, load in got['slid'][:3]) if got['slid'] else ''
    turns = ' | turns ' + ', '.join('%s %.2f mdeg/s (%.1f N m)' % tuple(row) for row in got['turned'][:3])
    return '%-24s mm/s  %s%s%s%s%s' % (name, own, loose, slides, turns,
                                       '  FELL %.1f' % got['fell'] if got['fell'] else '')


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('cases', nargs='*', help='cases: %s' % ', '.join([*BENCH, *LODGE]))
    p.add_argument('--against', help='another tree (its root), each case there too')
    p.add_argument('--jobs', type=int, default=5, help='cases side by side (`tools.dev.focus.relay`)')
    p.add_argument('--rows', action='store_true', help="each pair's first contact's rows as MuJoCo laid them")
    p.add_argument('--json', action='store_true', help='each case whole as JSON')
    p.add_argument('--one', nargs=2, help=argparse.SUPPRESS)
    args = p.parse_args(argv)
    if args.one:
        root, case = args.one
        sys.path[:0] = [os.path.join(root, 'host')]
        os.environ['PYTHONPATH'] = os.path.join(root, 'host')
        print('CREEP ' + json.dumps((bench if case in BENCH else lodge)(case)))
        return 0
    from tools.dev import focus
    cases = args.cases or [*BENCH, *LODGE]
    unknown = [case for case in cases if case not in BENCH and case not in LODGE]
    if unknown:
        p.error('no case %s' % ', '.join(unknown))
    trees = [os.path.dirname(HOST)] + ([os.path.abspath(args.against)] if args.against else [])
    work = [focus.Job('%s %s' % (os.path.basename(tree), case),
                      [sys.executable, '-X', 'utf8', os.path.abspath(__file__), '--one', tree, case], timeout=900.0)
            for case in cases for tree in trees]
    for job, text, _code, _s in focus.relay(work, batons=args.jobs):
        got = next((ln[6:] for ln in (text or '').splitlines() if ln.startswith('CREEP ')), None)
        if got and args.json:
            print(json.dumps(dict(json.loads(got), case=job.name)), flush=True)
            continue
        print(line(job.name, json.loads(got)) if got else '%-24s no run: %s' % (
            job.name, (text or 'out of time')[-300:].replace('\n', ' | ')), flush=True)
        for pair, dim, mu, solref, solimp, r, kbi in (json.loads(got)['rows'] if got and args.rows else []):
            print('    %-34s dim %d  mu %s  solref %s  solimp %s\n        R %s\n        K, B, imp %s' % (
                '/'.join(pair), dim, mu, solref, solimp, ['%.3g' % v for v in r],
                [['%.4g' % v for v in row] for row in kbi]), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
