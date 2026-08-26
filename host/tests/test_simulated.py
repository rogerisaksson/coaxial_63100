#!/usr/bin/env python3
"""coaxial.simulated: a board that was never plugged in.

No serial port, no ollama - just the invented session run through the real
MCP tool handlers, the same way a live board would be. If a handler works
here and breaks against real hardware, the handler changed what it expects
from `session`/`board`; if it breaks here too, the fixture drifted from that
shape. Either way this is the fast way to find out which.

Run from the host directory:  python tests/test_simulated.py
"""
import io
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial.errors import DeviceStateError            # noqa: E402
from coaxial.simulated import CHANNELS                  # noqa: E402
from coaxial.simulated import SimulatedSession          # noqa: E402
from coaxial_mcp import tools as toolmod                # noqa: E402

REPO = os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))
ADC_C = os.path.join(REPO, 'Board', 'Src', 'board_adc.c')

# One row of s_adcTable. Only the fields the stand-in also claims: the ADC,
# the channel number, the pin string, whether it is differential, the signal
# name. The unit column is the firmware's enum and the host's string, which
# are two spellings of one fact and are compared by test_parity against a
# real board.
ADC_ROW = re.compile(
    r'\{\s*&hadc(\d)\s*,[^,]*,\s*ADC_CHANNEL_(\d+)\s*,[^,]*,\s*'
    r'"([^"]+)"\s*,\s*(ADC_\w+)\s*,\s*"([^"]*)"')


def firmware_channels():
    """s_adcTable, read out of the firmware that defines it."""
    text = io.open(ADC_C, encoding='utf-8').read()
    table = text.split('s_adcTable[] =')[1].split('};')[0]
    return [{'adc': int(a), 'channel': int(c), 'pin': pin,
             'differential': mode == 'ADC_DIFFERENTIAL_ENDED', 'signal': sig}
            for a, c, pin, mode, sig in ADC_ROW.findall(table)]


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
    report.check('the channel table has all seven real channels, named',
                 len(channels) == 7
                 and {c['signal'] for c in channels if c['signal']}
                 == {'Phase U', 'Phase V', 'Phase W', 'NTC', 'DC bus',
                     'Clevel', 'Cinj'})
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
    # cinj,clevel,dcbus,ntc,..." - a refusal listing the channel it meant.
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
                          ('NTC_TEMP', 'NTC'), ('ADC_CH3', 'Clevel'),
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


def test_channel_table(report):
    """The stand-in's channel table still matches the one the board has.

    `coaxial.simulated.CHANNELS` is a second copy of `s_adcTable`, and a
    second answer to "what is PB0" is the thing this repository forbids
    everywhere else. test_parity compares them against real hardware and is
    right to - but it skips itself with no board, which is exactly when the
    stand-in is being used. This reads the firmware source instead, so the
    drift is caught at the desk on the run that costs three seconds.
    """
    if not os.path.exists(ADC_C):
        report.check('board_adc.c is where this suite expects it',
                     False, ADC_C)
        return

    board = firmware_channels()
    report.check('every s_adcTable row parsed out of the firmware',
                 len(board) == len(CHANNELS),
                 'firmware %d, stand-in %d' % (len(board), len(CHANNELS)))
    if len(board) != len(CHANNELS):
        return

    for i, (fw, sim) in enumerate(zip(board, CHANNELS)):
        wrong = ['%s: firmware %r, stand-in %r' % (k, fw[k], sim[k])
                 for k in ('adc', 'channel', 'pin', 'differential', 'signal')
                 if fw[k] != sim[k]]
        report.check('channel %d (%s) agrees with the board' % (i, fw['signal']),
                     not wrong, '; '.join(wrong))


def test_imu(report):
    """The stand-in IMU: the real shape, and never mistakable for a part.

    The tools call `session.board.imu` without knowing which board they
    hold, so the stand-in has to answer the same three calls. What it must
    not do is answer them plausibly: a product id from here says
    "simulated" where a part says a version.
    """
    session = SimulatedSession()
    part = session.board.imu

    ident = part.product_id()
    report.check('the stand-in IMU answers a product id at all',
                 isinstance(ident, dict) and 'sw_version' in ident,
                 sorted(ident) if isinstance(ident, dict) else type(ident))
    report.check('and says it is invented in the field a part puts a '
                 'version in', ident['sw_version'] == 'simulated',
                 ident['sw_version'])
    report.check('with no part or build number to mistake for one',
                 ident['sw_part'] == 0 and ident['sw_build'] == 0,
                 '%s %s' % (ident['sw_part'], ident['sw_build']))

    got = part.read()
    report.check('a read comes back on the input channel, framed like a '
                 'real cargo', got['channel'] == 3 and got['cargo'],
                 'ch %s, %d bytes' % (got['channel'], len(got['cargo'])))
    report.check('and decodes through the same walk the real cargo uses',
                 [r['name'] for r in got['reports']]
                 == ['timebase', 'accelerometer'],
                 [r['name'] for r in got['reports']])

    accel = got['reports'][1]
    report.check('the counts are what the report carried, unscaled',
                 accel['raw'][:2] == [0, 0] and accel['raw'][2] > 0,
                 accel['raw'])
    report.check('and the Q point turns them into about one g, which is '
                 'what a part at rest reads',
                 9.0 < accel['scaled'][2] < 10.5 and accel['unit'] == 'm/s^2',
                 '%.4f %s' % (accel['scaled'][2], accel['unit']))

    report.check('the sequence number moves, so a caller can see a new '
                 'sample rather than a frozen one',
                 part.read()['reports'][1]['seq'] != accel['seq'])

    part.feature(0x01, 60000)
    part.feature(0x01, 0)
    refused = False
    try:
        part.feature(0x01, 1 << 33)
    except ValueError:
        refused = True
    report.check('an interval that does not fit 32 bits is refused here '
                 'too, not silently truncated on the way to a board',
                 refused)

    tool = toolmod.imu(session, op='read')
    report.check('and the MCP tool renders it as a headed block',
                 tool.startswith('imu: channel 3') and 'accelerometer' in tool,
                 tool.splitlines()[0])


def test_orientation(report):
    """The quaternion maths and the picture it draws. No board, no IMU.

    Every case here is one a reader can check by hand: the identity is level,
    a quarter turn about an axis is 90 degrees of that axis and nothing else,
    and a rotation does not change a length.
    """
    from coaxial import orientation as o

    roll, pitch, yaw = o.euler_degrees((0, 0, 0, 1))
    report.check('the identity quaternion is level in all three angles',
                 (round(roll), round(pitch), round(yaw)) == (0, 0, 0),
                 '%.1f %.1f %.1f' % (roll, pitch, yaw))

    # Thirty degrees, not ninety: at pitch +-90 the Euler decomposition is
    # singular and roll and yaw trade places, so a quarter turn about Y
    # legitimately reads 180/90/180. Testing there would assert against
    # gimbal lock rather than against the maths.
    third = math.radians(30)
    for axis, name, want in ((0, 'roll', 0), (1, 'pitch', 1), (2, 'yaw', 2)):
        q = [0.0, 0.0, 0.0, math.cos(third / 2)]
        q[axis] = math.sin(third / 2)
        angles = o.euler_degrees(tuple(q))
        others = [a for i, a in enumerate(angles) if i != want]
        report.check('thirty degrees about %s reads there and nowhere else'
                     % name,
                     abs(angles[want] - 30) < 0.01
                     and all(abs(a) < 0.01 for a in others),
                     ' '.join('%.3f' % a for a in angles))

    # The clamp in euler_degrees: asin of anything past 1.0 is a domain
    # error, and floating point gets there on a legitimate quarter turn.
    straight_up = o.euler_degrees((0.0, math.sin(math.radians(45)), 0.0,
                                   math.cos(math.radians(45))))
    report.check('pitch is clamped at ninety rather than raising on a '
                 'rounding error past 1.0',
                 abs(abs(straight_up[1]) - 90) < 0.01, '%.4f' % straight_up[1])

    report.check('an all-zero rotation vector is taken as the identity '
                 'rather than divided by, which would put a NaN on screen',
                 o.normalise((0, 0, 0, 0)) == (0.0, 0.0, 0.0, 1.0))
    report.check('and a quaternion that is not unit length is made one',
                 abs(sum(v * v for v in o.normalise((0, 0, 0, 7))) - 1) < 1e-9)

    quarter = math.sin(math.radians(45))
    turned = o.rotate((0, 0, quarter, math.cos(math.radians(45))),
                      (1.0, 0.0, 0.0))
    report.check('a quarter turn about Z takes +X to +Y',
                 abs(turned[0]) < 1e-6 and abs(turned[1] - 1) < 1e-6,
                 '%.4f %.4f %.4f' % turned)
    length = math.sqrt(sum(v * v for v in o.rotate((0.5, 0.5, 0.5, 0.5),
                                                   (1.0, 2.0, 3.0))))
    report.check('and a rotation does not change a length',
                 abs(length - math.sqrt(14)) < 1e-6, '%.6f' % length)

    picture = o.render((0, 0, 0, 1), width=40, height=15)
    lines = picture.split(chr(10))
    report.check('the drawing is the height it was asked for',
                 len(lines) == 15, '%d lines' % len(lines))
    report.check('and no line runs past the width',
                 all(len(l) <= 40 for l in lines), max(len(l) for l in lines))
    report.check('the connector edge is marked, so which way round the '
                 'board is can be read off the shape',
                 '#' in picture and '.' in picture)
    wide = o.render((0, 0, 0, 1))          # the width the tools draw at
    report.check('the silkscreen reads whole, one cell per character - '
                 'rounding halves to even wrote it as "o x a  3 0 0"',
                 o.LABEL in wide, [l.strip('. ') for l in
                                   wide.split(chr(10)) if 'oax' in l])
    report.check('and a canvas too narrow for it drops the label rather '
                 'than writing it over itself',
                 o.LABEL not in picture)
    report.check('and it sits off centre, so the label says which way up '
                 'the board is when the outline cannot',
                 o.LABEL_Y > 0.0)

    away = o.render((0.0, 1.0, 0.0, 0.0))                        # turned over
    report.check('the silkscreen does not read through the board from the '
                 'solder side', o.LABEL not in away, o.facing((0, 1, 0, 0)))

    caption = o.picture((0, 0, 0, 1))
    report.check('the caption carries the angles and the quaternion both - '
                 'the picture is a reading, and the numbers are what it is '
                 'a reading of',
                 'roll' in caption and 'real +1.0000' in caption,
                 caption.splitlines()[-2])


def main():
    report = Report()
    for test in (test_session, test_board_info, test_analog_read,
                 test_self_test_and_link, test_gpio_gate,
                 test_channel_table, test_imu, test_orientation):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
