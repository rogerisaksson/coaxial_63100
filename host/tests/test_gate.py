"""Her streams gated (`machine.gate`) as her passes made them: each gated section the same, sample for
sample and to the bit, as a harness recording each subsystem where it runs; her triggers held off and
re-armed; the gate armed by a client of her telemetry."""
import json
import os
import socket
import sys
import tempfile
import time

from tools.dev.focus import chosen
from views_kit import Report

#: Her marks, s after the gate is armed: a trigger, one held off in its window, one re-armed, one past the count.
MARKS = (1.0, 1.3, 2.5, 3.9)


def _harnessed(s):
    """Her subsystems recorded where they run, apart from the gate: {name: [(t, sample)]} - her edge's
    array after each read, her law's layers after each control, her drives after each drives pass,
    the world after each step, her things after each look, her wear told, the commands heard -; and
    the undo."""
    import numpy as np
    from machine import edge, motion, physics, sprung, wear
    got = {k: [] for k in ('felt', 'read', 'law', 'drives', 'world', 'things', 'wear', 'heard')}
    copied = lambda d: {k: v.copy() if isinstance(v, np.ndarray) else v for k, v in d.items()}
    was = {(edge, 'felt'): edge.felt, (edge, 'read'): edge.read, (motion, 'control'): motion.control,
           (motion, 'drives'): motion.drives, (sprung, 'looked'): sprung.looked, (wear, 'told'): wear.told,
           (physics.World, 'advance'): physics.World.advance}
    t = lambda: float(s['g']['t'])

    def felt(world, y):
        out = was[edge, 'felt'](world, y)
        got['felt'].append((t(), y.copy()))
        return out

    def read(world, y):
        out = was[edge, 'read'](world, y)
        got['read'].append((t(), y.copy()))
        return out

    def control(g, dt):
        out = was[motion, 'control'](g, dt)
        got['law'].append((t(), {'st': g['st'].copy(), 'out': g['out'].copy(), 'a': g['a'].copy(),
                                 'ask': copied(g.get('ask') or {}), 'got': copied(g.get('got') or {})}))
        return out

    def drives(g, dt):
        out = was[motion, 'drives'](g, dt)
        got['drives'].append((t(), {k: v.copy() for k, v in g['cmd'].items()}))
        return out

    def looked(g):
        out = was[sprung, 'looked'](g)
        got['things'].append((float(g['things_at']), list(g['things'])))
        return out

    def told(w):
        out = was[wear, 'told'](w)
        got['wear'].append((t(), out))
        return out

    def advance(world):
        out = was[physics.World, 'advance'](world)
        m, d = world.model, world.data
        got['world'].append((t(), {m.joint(i).name: (d.joint(i).qpos.copy(), d.joint(i).qvel.copy()) for i in range(m.njnt)}))
        return out

    for (owner, name), fn in zip(was, (felt, read, control, drives, looked, told, advance)):
        setattr(owner, name, fn)
    heard_was = s['heard']
    s['heard'] = lambda e: (got['heard'].append((float(e[2]), e[3])), heard_was(e))[1]
    return got, lambda: [setattr(owner, name, fn) for (owner, name), fn in was.items()]


def _plain(v):
    from machine import introspect
    return json.dumps(introspect._plain(v), sort_keys=True)


def _expected(stream, meta, sample):
    """The gated vector `stream`'s sample as the harness's `sample` gives it, field by field through
    her code's own slices and names: the metadata's offsets checked as the values are."""
    import numpy as np
    from machine import edge, latent, modal
    fields = meta['fields']
    v = np.full(meta['n'], np.nan)
    for name, f in fields.items():
        part = name.split('.')
        at = slice(f['at'], f['at'] + f['n'])
        v[at] = (sample[getattr(edge, name)] if stream in ('felt', 'read') else
                 (sample['out'][getattr(latent, part[1])] if part[0] == 'out' else
                  sample['st'][getattr(latent, part[0])][getattr(latent, part[1])] if len(part) == 2 else
                  sample['st'][getattr(latent, part[0])]) if stream == 'latent' else
                 sample['a'][getattr(modal, name)] if stream == 'impedance' else
                 sample[name] if stream == 'drives' else
                 sample[part[1]][0 if part[0] == 'qpos' else 1])
    return v


def _sections(path, got, events):
    """Each gate of the file at `path` against the harness's samples `got` and the events served: per
    stream [k, matched] - a vector's samples equal to the bit, a record's as JSON, times the same,
    none missing -, and whether its records run in time."""
    import numpy as np
    from machine import gate, streams
    harness = {'felt': got['felt'], 'read': got['read'], 'latent': got['law'], 'impedance': got['law'],
               'drives': got['drives'], 'world': got['world'], 'things': got['things'], 'wear': got['wear'],
               'heard': got['heard'], 'wrench': [(t, {'ask': x['ask'], 'got': x['got']}) for t, x in got['law']],
               'events': [(e['t'], e) for e in events]}
    with open(path) as f:
        lines = [json.loads(line) for line in f]
    out = []
    for g in gate.read(path):
        lo, hi = g['meta']['window']
        verdict = {}
        for name, m in g['meta']['streams'].items():
            ts, vs = g['streams'].get(name, ([], []))
            h = [x for x in harness[name] if lo <= x[0] <= hi]
            held = [x for x in harness[name] if x[0] < lo][-1:] if m['period_s'] is None else []
            h = held + h
            same_t = list(ts) == [x[0] for x in h]
            same_v = same_t and all(
                np.array_equal(v, _expected(name, m, x[1]), equal_nan=True) if m['kind'] == 'vector' else
                _plain(v) == _plain(x[1]) for v, x in zip(vs, h))
            verdict[name] = [len(ts), bool(same_t and same_v)]
        out.append((g, verdict))
    order, each = {n: i for i, n in enumerate(streams.NAMES)}, []
    for r in lines:
        each.extend([[]] if 'gate' in r else [])
        each[-1].extend([(r['t'], order[r['s']])] if 's' in r else [])
    return out, len(each) == len(out) and all(keys == sorted(keys) for keys in each)


def test_each_gated_section_is_her_passes_own(report):
    """Her lie on the floor with a gate armed by a client of her telemetry on a trigger of her own
    (an analyzer saved for it): each gated section of every stream - her edge's reads, her law's
    latent, impedance and wrench, her drives, the world, her things, her wear, the commands she
    heard, her events - the same samples at the same times as a harness recording each subsystem
    where it runs, none missing and none made up; the fields' names and offsets her code's; the
    records in time, one instant's in her pass's order; the trigger in a window held off, the gate
    re-armed, none past its count."""
    from machine import introspect, worker
    s = worker.session()
    worker.apply(s, {'scenario': 'flat'})
    for _ in range(500):
        worker.stepped(s)
    t_arm = float(s['g']['t'])
    got, undo = _harnessed(s)
    with tempfile.TemporaryDirectory() as folder:
        with open(os.path.join(folder, 'mark.py'), 'w') as f:
            f.write("MARKS = %r\n\n\ndef laid(meta):\n    return {'done': set()}\n\n\ndef step(st, snap):\n"
                    "    due = [at for at in MARKS if snap['t'] >= at and at not in st['done']]\n"
                    "    st['done'].update(due)\n    return [{'event': 'mark', 'at': at, 'capture': True} for at in due]\n"
                    % [t_arm + m for m in MARKS])
        server = introspect.attach(s, 0)
        sock = socket.create_connection(('127.0.0.1', int(server.server_address[1])), timeout=30.0)
        try:
            sock.sendall(json.dumps({'gate': {'tag': 'harnessed', 'count': 2, 'folder': folder}}).encode() + b'\n')
            t0 = time.monotonic()
            while server.commands.empty() and time.monotonic() - t0 < 10.0:
                time.sleep(0.01)
            worker.stepped(s)
            worker.apply(s, {'say': 'lie_floor'})
            for k in range(int(4.5 / worker.DT)):
                worker.stepped(s)
                if k == int(2.0 / worker.DT):
                    worker.apply(s, {})
            f, told, t0 = sock.makefile('rb'), [], time.monotonic()
            while not any(e['event'] == 'gated' and not e['armed'] for e in told) and time.monotonic() - t0 < 60.0:
                rec = json.loads(f.readline())
                told.extend([rec] if 'event' in rec else [])
        finally:
            undo()
            sock.close()
            server.shutdown()
    path = next(e['file'] for e in told if e['event'] == 'armed')
    events = [e for e in told if e.get('analyzer') != 'gate' and e['event'] != 'watching']
    sections, in_time = _sections(path, got, events)
    gated = [(round(g['meta']['t'] - t_arm, 3), [h['name'] for h in g['trailer']['held_off']]) for g, _v in sections]
    report.check('every gated section of every stream her passes\' own, to the bit; in time',
                 sections and all(ok for _g, v in sections for _k, ok in v.values()) and in_time,
                 [{n: v for n, v in verdict.items()} for _g, verdict in sections])
    faults = [e for e in told if e['event'] == 'error']
    report.check('a gate a trigger, the one in its window held off, re-armed, none past its count; no fault',
                 len(gated) == 2 and abs(gated[0][0] - MARKS[0]) < 0.01 and gated[0][1] == ['mark.mark']
                 and abs(gated[1][0] - MARKS[2]) < 0.01 and gated[1][1] == [] and not faults,
                 (gated, faults))
    os.remove(path)


def _in_a_daemon(out):
    """Her session in a daemon process, as her page's worker runs her: a gate armed, fired and its
    window written - its notices put on `out`."""
    from machine import gate, worker
    s = worker.session()
    scope, told = gate.Scope(), []
    key = scope.arm(s, {'tag': 'daemon', 'count': 1, 'pre_s': 0.1, 'post_s': 0.1}, told.append)
    for k in range(400):
        worker.stepped(s)
        scope.passed(s)
        scope.fired(key, [{'t': float(s['g']['t']), 'analyzer': 'test', 'event': 'mark', 'capture': True}] if k == 200 else [])
        scope.closed(s)
    t0 = time.monotonic()
    while not any(n['event'] == 'gated' for n in told) and time.monotonic() - t0 < 60.0:
        time.sleep(0.05)
    out.put(told)


def test_a_gate_writes_from_a_daemon(report):
    """A gate fired in a daemon process - her page's worker is one, and may start no multiprocessing
    child - written by its writer all the same."""
    import multiprocessing
    ctx = multiprocessing.get_context('spawn')
    out = ctx.Queue()
    p = ctx.Process(target=_in_a_daemon, args=(out,), daemon=True)
    p.start()
    told = out.get(timeout=300.0)
    p.join(30.0)
    gated = [n for n in told if n['event'] == 'gated']
    report.check('written from a daemon process: its gate and its records told',
                 len(gated) == 1 and gated[0]['records'] > 0 and os.path.exists(gated[0]['file']),
                 [(n['event'], n.get('records')) for n in told])
    for n in gated:
        os.remove(n['file'])


ROSTER = (test_each_gated_section_is_her_passes_own, test_a_gate_writes_from_a_daemon)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
