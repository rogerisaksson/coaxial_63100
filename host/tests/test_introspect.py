"""Her loop's layers served live (`machine.introspect`): the layouts generated from her code, a pass's
snapshot named, a client attached reading them, nothing paid with none attached."""
import socket
import sys
import time

from tools.dev.focus import chosen
from views_kit import Report


def test_the_layouts_are_her_codes_own(report):
    """Each vector's fields generated from its module's constants: the ask's cover 0 to modal.ASK
    without a gap, latent out's to latent.OUT, a state row's to ortho.P; none written by hand."""
    from machine import introspect, latent, modal, ortho
    cover = lambda f: sorted((sp.start, sp.stop) for sp in f.values())
    gapless = lambda f, n: cover(f)[0][0] == 0 and cover(f)[-1][1] == n and all(
        a[1] == b[0] for a, b in zip(cover(f), cover(f)[1:]))
    ask, out, row = (introspect.layout('machine.modal', 'ASK'), introspect.layout('machine.latent', 'OUT'),
                     introspect.layout('machine.latent', 'P', 'NOW'))
    report.check('the ask, latent out and a state row generated, each gapless to its size',
                 gapless(ask, modal.ASK) and gapless(out, latent.OUT) and gapless(row, ortho.P)
                 and {'STANCE', 'COM_ACC', 'LET_GO'} <= set(ask) and {'KS', 'MANIFOLDED'} <= set(out),
                 '%d ask, %d out, %d row fields' % (len(ask), len(out), len(row)))


def test_a_client_reads_her_layers_live(report):
    """Attached to her session, a client connected reads a snapshot each law pass - her latent
    rows by mode, the ask, the step's answer -; none connected, the hook makes none."""
    from machine import introspect, ortho, worker
    s = worker.session()
    made, was = [], introspect.snapshot
    introspect.snapshot = lambda g: (made.append(1), was(g))[1]
    try:
        server = introspect.attach(s, 0)
        for _ in range(200):
            worker.stepped(s)
        idle = len(made)
        sock = socket.create_connection(('127.0.0.1', int(server.server_address[1])), timeout=10.0)
        t0 = time.monotonic()
        while not server.queues and time.monotonic() - t0 < 5.0:
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


ROSTER = (test_the_layouts_are_her_codes_own, test_a_client_reads_her_layers_live)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
