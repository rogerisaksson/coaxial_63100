"""Her loop's layers served live (`machine.process.introspect`): the layouts generated from her code, a pass's
snapshot named, a client attached reading them, nothing paid with none attached."""
import socket
import sys
import time

from tools.dev.focus import chosen
from views_kit import Report


def test_the_layouts_are_her_codes_own(report):
    """Each vector's fields generated from its module's constants: the ask's cover 0 to modal.ASK
    without a gap, latent out's to latent.OUT, a state row's to ortho.P; none written by hand."""
    from machine.process import introspect
    from machine.law import latent, modal
    from machine.texts import ortho
    cover = lambda f: sorted((sp.start, sp.stop) for sp in f.values())
    gapless = lambda f, n: cover(f)[0][0] == 0 and cover(f)[-1][1] == n and all(
        a[1] == b[0] for a, b in zip(cover(f), cover(f)[1:]))
    ask, out, row = (introspect.layout('machine.law.modal', 'ASK'), introspect.layout('machine.law.latent', 'OUT'),
                     introspect.layout('machine.law.latent', 'P', 'NOW'))
    report.check('the ask, latent out and a state row generated, each gapless to its size',
                 gapless(ask, modal.ASK) and gapless(out, latent.OUT) and gapless(row, ortho.P)
                 and {'STANCE', 'COM_ACC', 'LET_GO'} <= set(ask) and {'KS', 'MANIFOLDED'} <= set(out),
                 '%d ask, %d out, %d row fields' % (len(ask), len(out), len(row)))


def test_a_client_reads_her_layers_live(report):
    """Attached to her session, a client connected reads a snapshot each law pass - her latent
    rows by mode, the ask, the step's answer -; none connected, the hook makes none."""
    from machine.process import introspect, worker
    from machine.texts import ortho
    s = worker.session()
    made, was = [], introspect.snapshot
    introspect.snapshot = lambda s, peaks=None, lazy=False: (made.append(1), was(s, peaks, lazy))[1]
    try:
        server = introspect.attach(s, 0)
        for _ in range(200):
            worker.stepped(s)
        idle = len(made)
        sock = socket.create_connection(('127.0.0.1', int(server.server_address[1])), timeout=10.0)
        t0 = time.monotonic()
        while not server.clients and time.monotonic() - t0 < 5.0:
            time.sleep(0.01)
        for _ in range(200):
            worker.stepped(s)
        f = sock.makefile('rb')
        import json
        snaps = [json.loads(f.readline()) for _ in range(3)]
        sock.close()
        server.shutdown()
    finally:
        introspect.snapshot = was
    k = snaps[-1]['latent']['state']['NOW']['K']
    report.check('none attached none made; attached, snapshots named by her code: modes, ask, step',
                 idle == 0 and len(made) >= 3 and set(k) == set(ortho.MODES)
                 and 'STANCE' in snaps[-1]['impedance']['ask'] and 'tau' in snaps[-1]['wrench']
                 and snaps[-1]['t'] > snaps[0]['t'],
                 'idle %d, made %d, t %s' % (idle, len(made), [round(x['t'], 3) for x in snaps]))


def test_a_traced_pass_names_her_calls_and_her_observer(report):
    """Traced (`calls.Calls`), a snapshot carries what her law called since the last - her law pass and her C
    cores by name, none of her telemetry's -; her observer named by manifold against her skin; stopped, gone."""
    from machine.texts import calls
    from machine.process import introspect, worker
    from machine.senses import observer
    s = worker.session()
    for _ in range(200):
        worker.stepped(s)
    tap, tracer, snaps = introspect.Tap(), calls.Calls().start(s), []
    try:
        while len(snaps) < 3:
            worker.stepped(s)
            snaps += [snap for snap in (tap.passed(s),) if snap is not None]
        named = snaps[-1]['observer']
    finally:
        tracer.stop(s)
    called, soles = snaps[-1]['calls'], named['manifolds']['plantar-bilateral']
    report.check("her law pass and her cores named, her telemetry's not; her observer by manifold, her soles felt and on her skin",
                 'law.motion.control' in called and any(k.startswith('c.') for k in called)
                 and not any(k.startswith(calls.QUIET) for k in called) and 'calls' not in s
                 and set(named['manifolds']) == set(observer.MANIFOLDS) and soles['felt'] > 0.0 and soles['skin'] > 0.0,
                 '%d called, soles felt %.0f N, skin %.0f N' % (len(called), soles['felt'], soles['skin']))


def test_her_analyzers_reduce_as_the_probes_do(report):
    """Her lie on the floor reduced two ways from one session: her analyzers on the peak-held snapshots
    (`Tap`, `watch.Watch`) and every pass of her world (the probes' way): each closed phase's fastest
    joint and deg/s the same, the phases told in order, a few events where thousands of snapshots."""
    import numpy as np
    from machine.process import introspect, sprung, watch, worker
    from machine.body.figure import JOINTS
    s = worker.session()
    worker.apply(s, {'scenario': 'flat'})
    for _ in range(500):
        worker.stepped(s)
    w, tap, every, told, snaps = watch.Watch(), introspect.Tap(), {}, [], 0
    worker.apply(s, {'say': 'lie_floor'})
    world = s['g']['world']
    for _ in range(int(6.0 / worker.DT)):
        worker.stepped(s)
        v = np.degrees(np.abs(world.data.qvel[world.vadr]))
        doing = sprung.doing(s['g'])
        every[doing] = max(every.get(doing, (0.0, '')), (round(float(v.max()), 1), JOINTS[int(v.argmax())]))
        snap = tap.passed(s)
        snaps += snap is not None
        told.extend(w.step(snap) if snap is not None else ())
    motion = {e['phase']: (e['deg_s'], e['fastest']) for e in told if e['event'] == 'motion'}
    phases = [e['to'] for e in told if e['event'] == 'phase']
    report.check("each closed phase's fastest as every pass's; kneel, tip, floor_rest told in order",
                 motion and all(motion[p] == every[p] for p in motion) and phases[:3] == ['kneel', 'tip', 'floor_rest']
                 and len(told) < snaps / 100,
                 'analyzers %s, every pass %s, %d events of %d snapshots' % (
                     motion, {p: every[p] for p in motion}, len(told), snaps))


def test_the_analyzers_reload_as_she_runs(report):
    """An analyzer saved, changed and removed while the snapshots run is loaded, reloaded and dropped;
    one that raises is told and the rest fire on."""
    import os
    import tempfile
    from machine.process import watch
    with tempfile.TemporaryDirectory() as folder:
        fire = os.path.join(folder, 'fire.py')
        with open(fire, 'w') as f:
            f.write("def laid(meta):\n    return {}\n\ndef step(st, snap):\n"
                    "    return [{'event': 'hit'}] if 0.3 <= snap['t'] < 0.34 else []\n")
        w = watch.Watch(folder, {'natural_s': 0.1})
        was, watch.RESCAN_S = watch.RESCAN_S, 0.0
        try:
            told = [e for k in range(100) for e in w.step({'t': 0.01 * k, 'body': {'epoch': 0}})]
            with open(fire, 'w') as f:
                f.write("def laid(meta):\n    return {}\n\ndef step(st, snap):\n    raise ValueError('a bug')\n")
            os.utime(fire, (1e9, 1e9))
            with open(os.path.join(folder, 'calm.py'), 'w') as f:
                f.write("def laid(meta):\n    return {}\n\ndef step(st, snap):\n    return [{'event': 'tick'}]\n")
            later = [e for k in range(100, 103) for e in w.step({'t': 0.01 * k, 'body': {'epoch': 0}})]
            os.remove(fire)
            gone = w.step({'t': 1.03, 'body': {'epoch': 0}})
        finally:
            watch.RESCAN_S = was
    events = lambda es, name: [e['event'] for e in es if e['analyzer'] == name]
    report.check('loaded and firing, reloaded, failing told, the rest firing on, dropped',
                 events(told, 'fire')[:1] == ['loaded'] and events(told, 'fire').count('hit') == 4
                 and events(later, 'fire')[:2] == ['reloaded', 'error'] and 'tick' in events(later, 'calm')
                 and 'dropped' in events(gone, 'fire'),
                 (events(told, 'fire'), events(later, 'fire'), events(gone, 'fire')))


ROSTER = (test_the_layouts_are_her_codes_own, test_a_client_reads_her_layers_live,
          test_a_traced_pass_names_her_calls_and_her_observer, test_her_analyzers_reduce_as_the_probes_do,
          test_the_analyzers_reload_as_she_runs)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
