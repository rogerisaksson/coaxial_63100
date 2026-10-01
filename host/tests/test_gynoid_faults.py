"""The gynoid on her boards as built: their envelopes derating and tripping them, a gate drive
failed into its SOA - and fantasy boards, whose envelope never binds (`physics.ENVELOPE`). Her
walk and her look are test_gynoid.py's, her falls test_gynoid_falls.py's."""
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report


#: A gate drive failed low for FAILED_S from FAILED_AT of the left leg's stride - the knee taking
#: her weight as it lands - its switches FAILED_RDS times their on-resistance. At mid-stance, 0.25,
#: the knee straight, it asked too little of them to trip in 0.1 s (2026-09-28).
FAILED_RDS, FAILED_S, FAILED_AT = 500.0, 0.15, 0.05


def test_a_drive_keeps_its_heat(report):
    """A drive's board keeps its heat (`machine.heat`): stalled at its board's 100 A it derates
    from THROTTLE_AT of its envelope and drops its gates at a ceiling; armed and idle it cools,
    its derate given back at RECOVER_PER_S; its report goes on the wire and back whole."""
    from machine import drives, heat, rtu
    h = heat.Heat([drives.heat('left_knee')])
    stall = drives.kt('left_knee') * drives.of('left_knee')[1].amps
    air, rds, derated_at, tripped_at = [1.0], [1.0], None, None
    for k in range(3000):
        for _ in range(10):
            h.load(0, stall if h.gates[0] else 0.0)
        h.step(0.01, air, rds)
        if h.derate[0] < 1.0 and derated_at is None:
            derated_at = (k, h.spent[0])
        if not h.gates[0] and tripped_at is None:
            tripped_at = (k, h.derate[0], h.trips[0])
    report.check('stalled at 100 A: derated past %.2f spent, then its gates dropped' %
                 heat.THROTTLE_AT,
                 derated_at is not None and tripped_at is not None
                 and derated_at[0] < tripped_at[0] and derated_at[1] > heat.THROTTLE_AT
                 and tripped_at[2] == 1, '%s %s' % (derated_at, tripped_at))
    h.arm(0)
    for _ in range(6000):
        h.step(0.01, air, rds)
    celsius, spent, derate, status = h.report(0)
    report.check('armed and idle 60 s, cool: its gates on, its derate back to 1',
                 h.gates[0] and derate == 1.0 and spent < heat.THROTTLE_AT
                 and status & heat.GATES_ON, '%.1f C, spent %.2f, derate %.2f' % (
                     celsius, spent, derate))
    frame = rtu.reply(3, 12345, -6789, round(celsius * 100.0), round(spent * 1e4),
                      round(derate * 1e4), status)
    (unit, fc, body), = rtu.replies(frame + rtu.echo(rtu.gate(3)))[0][:1]
    got = rtu.state(body)
    report.check('its state on the wire: the angle, the rate and the heat back whole',
                 (unit, fc) == (3, rtu.READ) and len(frame) == rtu.REPLY_B
                 and got[:2] == (12345, -6789) and abs(got[2] / 100.0 - celsius) < 0.01
                 and got[5] == status, got)
    report.check('a gate write parsed as the host sends it and the board echoes it',
                 rtu.requests(rtu.gate(2))[0] == [(2, rtu.WRITE_ONE, rtu.gate(2)[2:-2])]
                 and rtu.replies(rtu.echo(rtu.gate(2)))[0][0][:2] == (2, rtu.WRITE_ONE))
    report.check('its op read back: the gates on, the phases shorted',
                 (rtu.gate_op(rtu.gate(2)), rtu.gate_op(rtu.gate(2, rtu.GATE_SHORT)))
                 == (rtu.GATE_ON, rtu.GATE_SHORT))


def test_a_drive_in_its_soa(report):
    """A knee's board run hard into its SOA as it takes her weight (its gate drive failed a
    moment): its gates drop under load, the director hears it on the bus and arms it again, its
    derate given back as it cools, and she walks on. At `physics.SOA_RDS` its envelope derates it
    before its ceiling."""
    from machine import Machine, heat, physics
    physics.SOA_RDS, was_rds = FAILED_RDS, physics.SOA_RDS
    from machine.director import Director
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    director = Director(body, 0.85)
    director.walker.start()
    director.stage = 'walk'
    body.loop.step(0.0)
    world, bus = body.nodes['pelvis'].world, body.loop.bus
    knee = director.drives['left_knee']
    dropped, armed, least, lowest, laid, was = None, None, 1.0, 9.0, None, 0.0
    while bus['t'] < 4.0:
        if laid is None and bus['t'] >= 1.0 and was < FAILED_AT <= director.walker.phase:
            world.glitch('left_knee', 'soa', FAILED_S)
            laid = bus['t']
        was = director.walker.phase
        body.loop.write(**director.step(0.001))
        body.loop.step(0.001)
        on = int(bus[knee + 'status']) & heat.GATES_ON
        dropped = bus['t'] if not on and dropped is None else dropped
        armed = bus['t'] if on and dropped is not None and armed is None else armed
        least, lowest = min(least, bus[knee + 'derate']), min(lowest, bus['pelvis.pose.y'])
    report.check('in its SOA, laid in its stance, its gates dropped, heard, and armed again '
                 'within 0.1 s', laid is not None and dropped is not None
                 and laid <= dropped < laid + 0.5 and armed is not None and armed - dropped < 0.1,
                 'laid %s, dropped %s, armed %s' % (laid, dropped, armed))
    report.check('derated to %.2f and given back to %.2f by 4 s, walking on over 2 m' % (
                     least, bus[knee + 'derate']),
                 least < 0.5 < bus[knee + 'derate'] and lowest > 0.7
                 and bus['pelvis.pose.z'] > 2.0, 'lowest %.2f m, %.2f m on' % (
                     lowest, bus['pelvis.pose.z']))
    physics.SOA_RDS = was_rds
    body.disarm()


def test_fantasy_boards_never_bind(report):
    """A fantasy board (`heat.Heat.envelope` False) counts its envelope and never derates or
    trips; the world's ENVELOPE 0 reaches every board on its bus."""
    from machine import Machine, drives, heat, physics
    h = heat.Heat([drives.heat('left_knee')])
    h.envelope = False
    stall = drives.kt('left_knee') * drives.of('left_knee')[1].amps
    for _ in range(3000):
        for _ in range(10):
            h.load(0, stall)
        h.step(0.01, [1.0], [1.0])
    report.check('stalled at 100 A for 300 heat s: spent, never derated, its gates on',
                 h.spent[0] >= 1.0 and h.derate[0] == 1.0 and h.gates[0],
                 '%.0f C, spent %.2f' % (h.t[0][h.worst[0]], h.spent[0]))
    from machine.director import Director
    from machine.modes import DYNAMIC
    physics.ENVELOPE, was = 0.0, physics.ENVELOPE
    physics.SOA_RDS, was_rds = FAILED_RDS, physics.SOA_RDS
    try:
        body = Machine.discover('gynoid', execution_mode=DYNAMIC)
        body.arm()
        director = Director(body, 0.85)
        director.walker.start()
        director.stage = 'walk'
        body.loop.step(0.0)
        world, bus = body.nodes['pelvis'].world, body.loop.bus
        knee, off, least = director.drives['left_knee'], False, 1.0
        while bus['t'] < 1.5:
            if 0.5 <= bus['t'] < 0.501:
                world.glitch('left_knee', 'soa', FAILED_S)
            body.loop.write(**director.step(0.001))
            body.loop.step(0.001)
            off = off or not int(bus[knee + 'status']) & heat.GATES_ON
            least = min(least, bus[knee + 'derate'])
        report.check('the same glitch on a fantasy knee: its gates on, never derated',
                     not off and least == 1.0, 'derate %.2f, gates %s' % (
                         least, 'dropped' if off else 'on'))
        body.disarm()
    finally:
        physics.ENVELOPE, physics.SOA_RDS = was, was_rds


ROSTER = (test_a_drive_keeps_its_heat, test_a_drive_in_its_soa, test_fantasy_boards_never_bind)

def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped'
          % (report.passed, report.failed, getattr(report, 'skipped', 0)))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
