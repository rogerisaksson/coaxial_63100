#!/usr/bin/env python3
"""Nodes, segments, unit ids, broadcast.

Split out of test_ollama.py, which had grown to 5,496 lines and 733 checks in
one file - a third of every check this tree has, and the reason a coverage
tier could not be asked for at any useful resolution. One subject per file
now, so a tier buys them separately and a reader opens the one they meant.

Run from the host directory:  python tests/test_ollama_bus.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.ollama_support import (DeviceStateError, Scope, toolmod)   # noqa: E402

def test_bus(report):
    """Five segments, one per limb plus the axis.

    A bus is a serial segment, which is how a machine like this is wired:
    shorter runs, one limb's fault confined to one limb, four segments
    carrying traffic at once instead of twenty nodes taking turns. That
    makes an odd/even side rule redundant - the bus says the side - so the
    unit id says the position down the limb, and node 2 is the knee on both
    legs. A number worth more to a controller than a unique one.
    """
    from coaxial.simulated import (SIMULATED_BUSES, bus_nodes,
                                   SimulatedSession as Sim)
    from coaxial_mcp import tools as mcp
    from coaxial_ollama import debug

    session = Sim()

    segments = mcp.HANDLERS['devices'](session, op='buses')
    report.check('every segment is listed, with what it serves',
                 all('%s ' % label in segments and serves in segments
                     for label, (serves, _) in SIMULATED_BUSES.items()),
                 segments.splitlines()[2][:46])
    report.check('and the one the tools are on is marked',
                 [l for l in segments.splitlines() if ' * ' in l][0]
                 .startswith('AX'),
                 [l for l in segments.splitlines() if ' * ' in l][0][:30])

    listed = mcp.HANDLERS['devices'](session)
    report.check('the list carries the bus, because a node number alone '
                 'names nothing',
                 listed.splitlines()[1].split()[:2] == ['bus', 'node'],
                 listed.splitlines()[1])
    report.check('every node on every segment is in it',
                 sum(len(nodes) for _, nodes in SIMULATED_BUSES.values())
                 == len(listed.splitlines()) - 2,
                 '%d rows' % (len(listed.splitlines()) - 2))

    # The same joint at the same unit id on both sides is the property the
    # segments buy: the bus says which leg, the number says which joint.
    for unit in (1, 2, 3, 4):
        left = bus_nodes('LL')[unit][2]
        right = bus_nodes('RL')[unit][2]
        report.check('unit %d is the same joint on both legs' % unit,
                     left.replace('left', '') == right.replace('right', ''),
                     '%s / %s' % (left, right))
    report.check('and the arms are numbered the same way down the limb',
                 [bus_nodes('LA')[u][2].replace('left ', '')
                  for u in (1, 2, 3, 4)]
                 == ['shoulder', 'elbow', 'wrist', 'gripper'])

    # Selecting, by number-with-bus and by what the node calls itself.
    mcp.HANDLERS['devices'](session, op='use', bus='RL', unit=2)
    report.check('op=use moves the bus as well as the node',
                 (session.bus, session.unit) == ('RL', 2),
                 '%s %d' % (session.bus, session.unit))
    report.check('and every other tool follows both',
                 'right knee' in mcp.HANDLERS['board_info'](session,
                                                            kind='identity'))

    mcp.HANDLERS['devices'](session, op='use', name='left gripper')
    report.check('a node can be picked by name across every segment',
                 (session.bus, session.unit) == ('LA', 4),
                 '%s %d' % (session.bus, session.unit))

    # 'knee' is one node on each leg, which is exactly what a symmetric
    # machine makes ambiguous - so it names both rather than picking.
    both = mcp.HANDLERS['devices'](session, op='use', name='knee')
    report.check('a name on two segments names both, and moves nothing',
                 'LL 2' in both and 'RL 2' in both
                 and (session.bus, session.unit) == ('LA', 4), both[:52])

    absent = mcp.HANDLERS['devices'](session, op='use', bus='LL', unit=9)
    report.check('a node nobody is at is refused, with who is',
                 absent.startswith('ERR no node at LL 9')
                 and (session.bus, session.unit) == ('LA', 4), absent[:46])

    mcp.HANDLERS['devices'](session, op='use', bus='AX', unit=3)
    mcp.HANDLERS['afe_power'](session, action='on')
    report.check('a reading comes from the node that is selected',
                 'samples @' in mcp.HANDLERS['analog_read'](session,
                                                            samples=8))

    report.check('and the model is offered the tool at all',
                 all('devices' in debug.SETS[name]
                     for name in ('read', 'code', 'pins')))

    # Node 0 is the Modbus broadcast address, not a node. Every node acts on
    # it and none answers, so a read there cannot work - and a timeout would
    # read as the bus having died rather than as the protocol working.
    # Refused in one place, Board.request, which every read and every
    # read-back write comes through. It is one segment's broadcast, not the
    # machine's: five buses are five broadcast domains.
    from coaxial.errors import DeviceStateError
    mcp.HANDLERS['devices'](session, op='use', unit=0)
    report.check('unit 0 is selectable even though it never answers',
                 session.unit == 0, str(session.unit))
    for tool in ('analog_read', 'digital_read', 'board_info'):
        try:
            mcp.HANDLERS[tool](session)
            report.check('%s on the broadcast address is refused' % tool,
                         False, 'it answered')
        except DeviceStateError as exc:
            report.check('%s on the broadcast address is refused' % tool,
                         'broadcast address' in str(exc), str(exc)[:40])
    sent = mcp.HANDLERS['afe_power'](session, action='on')
    report.check('but an order still goes out, and says it was not confirmed',
                 'every node' in sent and 'no read-back' in sent, sent[:52])
    report.check('a read-back action is refused rather than half-done',
                 mcp.HANDLERS['afe_power'](session, action='read')
                 .startswith('ERR'))

    # The prompt is the one place an operator sees where the tools are
    # pointed, and with five segments the bus has to be in it.
    talk = debug.Chat.__new__(debug.Chat)
    talk.toolbox = toolmod.Toolbox(session, scope=Scope())
    talk.origin = ('Simulated', False)
    # Short on purpose: the bus already carries the side, so the joint goes
    # in without it. "RL 2 knee", not "RL node 2 right knee" - and `Ra` for
    # ankle, which was the abbreviation asked for, would have collided with
    # `RA` for the right arm on a line whose whole job is to be read at a
    # glance.
    for bus, unit, expect, ok in (('RL', 2, 'RL 2 knee', False),
                                  ('LA', 1, 'LA 1 shoulder', False),
                                  ('AX', 0, 'AX ALL NODES', 'all')):
        session.use(unit, bus=bus)
        tag, got_ok = talk.prompt_tag()
        report.check('the prompt says %r' % expect, expect in tag, tag)
        report.check('   ...and the spinner paints it %s'
                     % ('red' if ok == 'all' else 'yellow'),
                     got_ok == ok, repr(got_ok))


ROSTER = (
    (test_bus, ('bus', 'tools')),
)


if __name__ == '__main__':
    from tests.ollama_support import run_file
    sys.exit(run_file(ROSTER))
