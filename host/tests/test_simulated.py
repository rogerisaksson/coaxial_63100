#!/usr/bin/env python3
"""coaxial.simulated: a board that was never plugged in.

No serial port, no ollama - just the invented session run through the real
MCP tool handlers, the same way a live board would be. If a handler works
here and breaks against real hardware, the handler changed what it expects
from `session`/`board`; if it breaks here too, the fixture drifted from that
shape. Either way this is the fast way to find out which.

Run from the host directory:  python tests/test_simulated.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial.errors import DeviceStateError            # noqa: E402
from coaxial.simulated import SimulatedSession          # noqa: E402
from coaxial_mcp import tools as toolmod                # noqa: E402


class Report:
    def __init__(self):
        self.passed = 0
        self.failed = 0

    def check(self, name, condition, detail=''):
        if condition:
            self.passed += 1
            print('  PASS  %-60s %s' % (name, detail))
        else:
            self.failed += 1
            print('  FAIL  %-60s %s' % (name, detail))


def test_session(report):
    session = SimulatedSession()
    version, clock, channels = session.info()

    report.check('the session never opens a port - board is ready at once',
                 session.board is session.board)
    report.check('version says what it is, not a plausible-looking number',
                 version['firmware'] == 'simulated' and version['build']
                 == 'simulated', version)
    report.check('the channel table has all seven real channels',
                 len(channels) == 7
                 and {c['signal'] for c in channels if c['signal']}
                 == {'Phase U', 'Phase V', 'Phase W', 'NTC', 'DC bus'})
    report.check('close() and reset() are no-ops, not errors',
                 session.close() is None and session.reset() is None)


def test_board_info(report):
    session = SimulatedSession()
    text = toolmod.board_info(session)
    report.check("board_info alone says 'simulated' - no need to already "
                 'know',
                 'simulated' in text.lower(), text[:80])


def test_analog_read(report):
    afe_off = SimulatedSession()
    off_text = toolmod.analog_read(afe_off)
    report.check('AFE off carries the same banner a real board would print',
                 'AFE OFF' in off_text)
    report.check('and the codes underneath are mid-scale/zero, per '
                 'invariant 9 - not refused, not invented past that',
                 '32768' in off_text and '0.0' in off_text, off_text[:200])

    afe_on = SimulatedSession()
    toolmod.afe_power(afe_on, action='on')
    on_text = toolmod.analog_read(afe_on)
    report.check('AFE on drops the banner and reads near the nominal point, '
                 'not frozen at mid-scale',
                 'AFE OFF' not in on_text and '32768' not in on_text,
                 on_text[:200])

    single = SimulatedSession()
    toolmod.afe_power(single, action='on')
    ntc_only = toolmod.analog_read(single, ch=['ntc'])
    report.check('a single named channel resolves and reads alone',
                 'NTC' in ntc_only and 'Phase' not in ntc_only,
                 ntc_only[:120])

    # What a question calls a channel, not what the table calls it. Measured
    # at the prompt: ch=['bus'] came back "unknown channel 'bus'; names are
    # ch3,ch6,dcbus,ntc,..." - a refusal listing the channel it meant.
    named = SimulatedSession()
    toolmod.afe_power(named, action='on')
    for asked, expect in (('bus', 'DCbus'), ('temp', 'NTC'),
                          ('vbus', 'DCbus'), ('temperature', 'NTC'),
                          ('dc_bus', 'DCbus'), ('phase_a', 'PhaseU'),
                          ('w', 'PhaseW')):
        text = toolmod.analog_read(named, ch=[asked])
        report.check('%r reads the channel it means (%s)' % (asked, expect),
                     expect in text and 'unknown' not in text, text[-60:])

    # A word that could mean several is a question nobody narrowed, not a
    # typo. Naming the candidates beats "unknown", which reads as "no such
    # thing" and sends the next call somewhere else.
    try:
        toolmod.analog_read(named, ch=['phas'])
        report.check('an ambiguous name names its candidates', False)
    except ValueError as exc:
        report.check('an ambiguous name names its candidates',
                     'PhaseU' in str(exc) or 'phaseu' in str(exc), str(exc)[:80])
    try:
        toolmod.analog_read(named, ch=['ntx'])
        report.check('a name that means nothing is still refused', False)
    except ValueError as exc:
        report.check('a name that means nothing is still refused',
                     'unknown channel' in str(exc), str(exc)[:60])

    # A name built out of words. Measured at the prompt: BUS_VOLT and A0,
    # both invented by the model, both refused where one of them meant
    # something.
    for asked, expect in (('BUS_VOLT', 'DCbus'), ('bus_voltage', 'DCbus'),
                          ('NTC_TEMP', 'NTC'), ('ADC_CH3', 'ch3'),
                          ('PhaseAVolt', 'PhaseU')):
        text = toolmod.analog_read(named, ch=[asked])
        report.check('%r reads the channel its words name (%s)'
                     % (asked, expect),
                     expect in text and 'unknown' not in text, text[-56:])

    # And the words must not resolve a name that is not one: `not_a_channel`
    # went to PhaseU through its bare `a` before the rule that a single
    # letter only counts beside the word `phase`.
    for asked in ('not_a_channel', 'the analog channel', 'A0'):
        try:
            toolmod.analog_read(named, ch=[asked])
            report.check('%r is refused, not guessed at' % asked, False)
        except ValueError as exc:
            report.check('%r is refused, not guessed at' % asked,
                         'unknown channel' in str(exc), str(exc)[:46])


def test_self_test_and_link(report):
    session = SimulatedSession()
    report.check('self_test reports all pass, nothing invented as a limit',
                 'FAIL' not in toolmod.self_test(session))
    report.check('link echo is a round trip, unchanged',
                 toolmod.link(session, op='echo', text='ping')
                 == "echo ok 'ping'")
    stats = toolmod.link(session, op='stats')
    report.check('link stats carries the real field names, not a subset',
                 'char_overrun' in stats and 'bus_message' in stats, stats)


def test_gpio_gate(report):
    session = SimulatedSession()
    report.check('a read works with the gate closed',
                 toolmod.gpio_pin(session, op='read', pin='B5') == 'B5=0')
    try:
        toolmod.gpio_pin(session, op='write', pin='B5', level=True)
        wrote_without_gate = True
    except DeviceStateError:
        wrote_without_gate = False
    report.check('a write is refused with the gate closed, same courtesy '
                 'the real board documents',
                 not wrote_without_gate)

    report.check('test_gate opens it',
                 toolmod.test_gate(session, enable=True) == 'gate=1')
    report.check('and now the write goes through and reads back',
                 toolmod.gpio_pin(session, op='write', pin='B5', level=True)
                 == 'B5=1 readback')

    try:
        toolmod.gpio_pin(session, op='write', pin='B10', level=True)
        wrote_reserved = True
    except ValueError:
        wrote_reserved = False
    report.check('a reserved pin (the link itself, B10) is refused '
                 'regardless of the gate',
                 not wrote_reserved)


def main():
    report = Report()
    for test in (test_session, test_board_info, test_analog_read,
                 test_self_test_and_link, test_gpio_gate):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
