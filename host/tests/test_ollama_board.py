#!/usr/bin/env python3
"""The board, its channels, its pins, the AFE.

Split out of test_ollama.py, which had grown to 5,496 lines and 733 checks in
one file - a third of every check this tree has, and the reason a coverage
tier could not be asked for at any useful resolution. One subject per file
now, so a tier buys them separately and a reader opens the one they meant.

Run from the host directory:  python tests/test_ollama_board.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.ollama_support import (Scope, ScriptedModel, SimulatedSession, 
    call, io, toolmod)   # noqa: E402

def test_afe_trace(report):
    """A switch that did what it was told needs no line of its own.

    Measured: "sla pa afen" traced `on=1 pe15=0` above an answer that said
    the same thing in words. `on=1` is also the only evidence the write
    landed, so the line goes only when the read-back matches what was
    asked - a request that did not take still prints, loudly.
    """
    from coaxial_ollama import debug

    for action, raw, silent, why in (
            ('on', 'on=1 pe15=0', True, 'did what it was told'),
            ('off', 'on=0 pe15=1', True, 'did what it was told'),
            ('on', 'on=0 pe15=1', False, 'the write did not take'),
            ('off', 'on=1 pe15=0', False, 'the write did not take'),
            ('toggle', 'on=1 pe15=0', False, 'nothing to match it against'),
            ('read', 'on=1 pe15=0', False, 'the state is the answer'),
            ('on', 'ERR ConnectError: cable pulled', False, 'an error'),
            ('on', 'ERR not asked for', True, 'a refusal it recovers from')):
        got = debug._afe_noise('afe_power', {'action': action}, raw)
        report.check('afe_power %s -> %s (%s)'
                     % (action, 'silent' if silent else 'traced', why),
                     got is silent, 'silent' if got else 'traced')

    report.check('and no other tool is quietened by this',
                 not debug._afe_noise('analog_read', {}, 'on=1 pe15=0'))

    # End to end: the turn shows the answer, and nothing above it.
    screen = io.StringIO()
    chat = debug.Chat(ScriptedModel([
        call('afe_power', action='on'),
        {'role': 'assistant', 'content': 'AFE är nu påslagen.'},
    ]), toolmod.Toolbox(SimulatedSession(), scope=Scope()), out=screen)
    chat.toolbox.afe_mentioned = True
    said = chat.ask('slå på afen')
    report.check('a confirmed switch leaves nothing on screen above the '
                 'answer', screen.getvalue().strip() == '',
                 repr(screen.getvalue())[:46])
    report.check('and the answer itself still arrives',
                 said == 'AFE är nu påslagen.', said)

def test_digital_read(report):
    """Listing the channels and reading them are two questions.

    Measured: "ge mig en lista over alla digitala varden" - values - was
    answered with the channel list, because that is all there was. gpio_pin
    reads one pin and gpio_port hands back a register for the model to pick
    bits out of, which is arithmetic this library exists not to hand it.
    """
    from coaxial.simulated import SimulatedSession as Sim
    from coaxial_mcp import tools as mcp

    session = Sim()

    mcp.HANDLERS['afe_power'](session, action='on')
    hot = mcp.HANDLERS['digital_read'](session)
    mcp.HANDLERS['afe_power'](session, action='off')
    cold = mcp.HANDLERS['digital_read'](session)

    report.check('it has a level column the map does not',
                 'level' in hot.splitlines()[1]
                 and 'level' not in mcp.HANDLERS['board_info'](
                     session, kind='digital'),
                 hot.splitlines()[1])
    report.check('and the levels move with the board, not with the map',
                 hot != cold, '%r vs %r' % (hot.splitlines()[2][:22],
                                            cold.splitlines()[2][:22]))
    report.check('PB2 follows the AFE switch',
                 'PB2  out   1' in hot and 'PB2  out   0' in cold)
    # nFAULT, and the level is the same measurement it was before the pin
    # had that name - see FINDINGS: 0 with the front end powered reads as a
    # fault asserted, and what drives it is not established. Asserted here
    # because it is what the board does, not because it is understood.
    report.check('and nFAULT reads back inversely, as the board wires it',
                 'PE15 in    0     nFAULT' in hot
                 and 'PE15 in    1     nFAULT' in cold,
                 hot.splitlines()[-1])
    report.check('every pin the map calls digital I/O is read, and only those',
                 len(hot.splitlines()) == 2 + len(
                     Sim().board.system.channel_map()['digital']),
                 '%d rows' % (len(hot.splitlines()) - 2))
    report.check('a bus pin is not among them - it is not a channel',
                 'PB10' not in hot and 'PA13' not in hot)

    # A tool the model cannot call is a tool that does not exist. Measured:
    # digital_read worked and the question still came back with the list,
    # because it was in no named set and the default one is `code`.
    from coaxial_ollama import debug as debugmod
    missing = [name for name in ('read', 'code', 'pins')
               if 'digital_read' not in debugmod.SETS[name]]
    report.check('and the sets a bench session runs actually offer it',
                 not missing, ', '.join(missing) or 'read, code, pins')

def test_channel_map(report):
    """The board describes itself; nothing above it keeps a copy.

    Command 0x6D reports every channel - analog and digital - with the
    direction each one runs. The host used to hold three separate answers to
    "what is PB10": the firmware's testrig table, the firmware's pin table,
    and protocol.RESERVED_PINS. There is one now, and the other two read it.
    """
    from coaxial.simulated import SimulatedSession as Sim
    from coaxial import protocol

    board = Sim().board
    chart = board.system.channel_map()

    report.check('the map has all three lists',
                 chart['analog'] and chart['digital'] and chart['reserved'],
                 '%d analog, %d digital, %d reserved'
                 % (len(chart['analog']), len(chart['digital']),
                    len(chart['reserved'])))
    # The separation is the point, not a flag on a row: what a fixture may
    # set without breaking anything is a different question from what the
    # bus and the debug port sit on, and mixing them invites a pin write
    # that gets refused.
    io_pins = {r['pin'] for r in chart['digital']}
    # A superset, not an equality. Adding a pin to the board is meant to be
    # one row in s_digital and nothing else; a check that froze the whole set
    # made it two, and the property this is named for - that nothing here is
    # a pin the bus or the probe sits on - is the check below.
    report.check('digital I/O carries the board controls it has always had',
                 io_pins >= {'PB2', 'PE15'}, ', '.join(sorted(io_pins)))
    report.check('and no bus or debug pin is among the channels',
                 not (io_pins & {r['pin'] for r in chart['reserved']}))
    report.check('every analog channel says which way it runs, and it is in',
                 all(row['direction'] == 'in' for row in chart['analog']),
                 ', '.join(sorted({r['direction']
                                   for r in chart['analog']})))
    report.check('every digital channel says which way it runs',
                 all(row['direction'] in protocol.DIRECTIONS.values()
                     for row in chart['digital'] + chart['reserved']),
                 ', '.join(sorted({r['direction']
                                   for r in chart['digital']})))
    report.check('the AFE switch is a digital output',
                 any(r['pin'] == 'PB2' and r['direction'] == 'out'
                     for r in chart['digital']))
    report.check('the link pins are reported, so a refusal can be explained',
                 all(any(r['pin'] == pin for r in chart['reserved'])
                     for pin in ('PB10', 'PB11')))

    # The static copy is the fallback, and it must not drift from the map
    # while it exists. A pin the board calls reserved and this dict does not
    # is the disagreement the whole change exists to prevent.
    refused = {r['pin'] for r in chart['reserved']}
    stale = {'P%s%d' % key for key in protocol.RESERVED_PINS} - refused
    report.check('the static fallback names no pin the board does not',
                 not stale, ', '.join(sorted(stale)) or 'none')

    # The refusal is the board's answer now, so its wording is the board's
    # signal name rather than a string compiled into the host.
    try:
        board.gpio.pin_read('B', 10)
        report.check('a reserved pin is refused by name', False, 'it was read')
    except ValueError as exc:
        report.check('a reserved pin is refused by the name the board gave it',
                     'USART3_TX' in str(exc), str(exc)[:52])


ROSTER = (
    (test_afe_trace, ('board', 'render')),
    (test_digital_read, ('board', 'tools')),
    (test_channel_map, ('board', 'bus')),
)


if __name__ == '__main__':
    from tests.ollama_support import run_file
    sys.exit(run_file(ROSTER))
