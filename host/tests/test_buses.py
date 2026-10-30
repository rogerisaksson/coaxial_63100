#!/usr/bin/env python3
"""Her buses under faults (`machine.senses.edge`, wbc_sil.c, Renode): a burst within her horizon ridden, past it drives off."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The bus a test's burst hits, and the passes she stands before it.
BUS, SETTLE = 'left_leg', 200
#: Her run under the line's noise (`edge.LINE_BER`), passes, and its draw.
NOISY, SEED = 10_000, 3
#: A limb on Renode, and the echoes through it: a request among them corrupted.
RENODE_NODES, RENODE_ECHOES = 3, 24


def _laid():
    """Her session standing SETTLE passes, the bus BUS, her horizon 1/omega in passes, its frames a pass."""
    from machine.senses import edge
    from machine.law import modal
    from machine.process import worker
    s = worker.session()
    for _ in range(SETTLE):
        worker.stepped(s)
    bus = next(b for b in s['g']['world'].limb_buses if b.name == BUS)
    f0 = bus.frames
    for _ in range(SETTLE):
        worker.stepped(s)
    horizon = 1.0 / (float(s['g']['fr'][modal.OMEGA]) * worker.DT)
    return s, bus, horizon, (bus.frames - f0) / SETTLE, edge


def test_a_burst_within_her_horizon_is_ridden(report):
    """Half her horizon's frames corrupted on one bus: each refused once, the boards hold their
    last setpoints, the next pass sends anew - she stands, her drives on."""
    from machine.process import worker
    s, bus, horizon, per_pass, edge = _laid()
    frames, e0 = int(horizon * per_pass / 2), bus.comm_errors
    edge.burst(s['g']['world'], BUS, frames)
    for _ in range(int(horizon)):
        worker.stepped(s)
    report.check('%d frames corrupted on %s, half her horizon: each refused once, she stands' % (frames, BUS),
                 bus.comm_errors - e0 == frames and bus.stale == 0 and s['shutdown'] is None and s['fallen_at'] is None,
                 'refused %d, stale %d, shutdown %s, fell %s' % (bus.comm_errors - e0, bus.stale, s['shutdown'], s['fallen_at']))


def test_a_systematic_fault_shuts_her_down(report):
    """A bus refusing every frame: past her horizon her drives off, no new epoch while she lies,
    a restart lays her anew."""
    from machine.process import sprung, worker
    s, _bus, horizon, _per_pass, edge = _laid()
    world, epoch = s['g']['world'], s['epoch']
    edge.burst(world, BUS, 1 << 30)
    passes = 0
    while s['shutdown'] is None and passes <= 2 * horizon:
        worker.stepped(s)
        passes += 1
    limp = not world.limit.any()
    for _ in range(int(sprung.RECOVER_S / worker.DT) + 1):
        worker.stepped(s)
    kept = s['epoch'] == epoch and s['shutdown'] is not None
    told = worker.told(s)['shutdown']
    worker.apply(s, {'restart': True})
    report.check('shut down within her horizon of %.0f passes, drives off, held past RECOVER_S, a restart lays her anew'
                 % horizon, s['shutdown'] is None and kept and limp and horizon < passes <= horizon + 1 and BUS in (told or ''),
                 'after %d passes: %s; limp %s, kept %s' % (passes, told, limp, kept))


def test_the_line_noise_is_sporadic(report):
    """Her buses at the line's noise a run's while she stands: frames refused, counted once each,
    none systematic."""
    from machine.process import worker
    s, _bus, _horizon, _per_pass, edge = _laid()
    world = s['g']['world']
    edge.noise(world, edge.LINE_BER, SEED)
    r0 = sum(b.comm_errors for b in world.limb_buses)
    for _ in range(NOISY):
        worker.stepped(s)
    refused = sum(b.comm_errors for b in world.limb_buses) - r0
    report.check('%.0e on her buses %.0f s: %d frames refused, she stands, drives on' % (edge.LINE_BER, NOISY * worker.DT, refused),
                 refused > 0 and s['shutdown'] is None and s['fallen_at'] is None, 'shutdown %s fell %s' % (s['shutdown'], s['fallen_at']))


def test_the_plate_tells_crc_wear_and_shutdown(report):
    """The RS485 plate's foot: CRC OK or its count, her wear's drift alarms apart, SHUTDOWN."""
    from terminal.views import scanner

    def foot(now, width=40):
        cells = scanner.bus_plate(now, 0, 0, width)
        return ''.join(chr(cells[(6, c)][0]) for c in range(width))
    clean = foot({'buses': {'axis': (1, 0, 0, 0)}})
    noisy = foot({'buses': {'axis': (1, 2, 0, 0)}, 'wear': {'alarms': [('left_knee.f', 1.0, 0.5)]}})
    down = foot({'buses': {'axis': (1, 0, 0, 400)}, 'shutdown': 'left_leg: drives off'})
    report.check('CRC OK; CRC 2 beside WEAR 1; SHUTDOWN', 'CRC OK' in clean and 'WEAR' not in clean
                 and 'CRC 2' in noisy and 'WEAR 1' in noisy and 'SHUTDOWN' in down, '%s | %s | %s' % (clean, noisy, down))


def test_a_corrupt_request_on_renode_is_one_lost(report):
    """A limb of RENODE_NODES on `emulator://`, one request's bit flipped on the wire (`Serial.burst`):
    every node refuses it on its CRC and stays silent, that echo alone lost, the next answered."""
    from coaxial import EMULATED, Coaxial63100
    from coaxial.errors import RigError
    from tools.emu import protocol_emulator
    from tools.emu.emulator import ELF, find_renode
    if find_renode() is None or not os.path.exists(ELF):
        print('  no Renode or no image: skipped')
        return
    url = 'emulator://?nodes=%d' % RENODE_NODES
    rigs = [Coaxial63100(port=url, unit=u, execution_mode=EMULATED, fallback=False).open() for u in range(1, RENODE_NODES + 1)]
    try:
        before = [rig.board.link.state(port=1)['bus_comm_error'] for rig in rigs]
        rigs[0].board.transport.serial.burst = 1
        lost = []
        for k in range(RENODE_ECHOES):
            try:
                rigs[k % RENODE_NODES].board.link.echo(bytes(range(40)))
            except RigError as exc:
                lost.append((k, type(exc).__name__))
        seen = [rig.board.link.state(port=1)['bus_comm_error'] - b for rig, b in zip(rigs, before)]
    finally:
        for rig in rigs:
            rig.close()
        protocol_emulator.release(url)
    report.check('on Renode a corrupt request: that echo alone lost, silent; every node counted it',
                 lost == [(0, 'NoReplyError')] and all(n >= 1 for n in seen), 'lost %s, comm errors %s' % (lost, seen))


ROSTER = [test_a_burst_within_her_horizon_is_ridden, test_a_systematic_fault_shuts_her_down, test_the_line_noise_is_sporadic,
          test_the_plate_tells_crc_wear_and_shutdown, test_a_corrupt_request_on_renode_is_one_lost]


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
