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

from coaxial import ansi, ascii3d, desk                  # noqa: E402
from coaxial import orientation, scaling               # noqa: E402
from coaxial.errors import DeviceStateError            # noqa: E402
from coaxial import simulated
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
    r'\{\s*&hadc(\d)\s*,[^,]*,\s*ADC_CHANNEL_(\w+)\s*,[^,]*,\s*'
    r'"([^"]+)"\s*,\s*(ADC_\w+)\s*,\s*"([^"]*)"')


def firmware_channels():
    """s_adcTable, read out of the firmware that defines it.

    `channel` is None for an internal channel. ADC_CHANNEL_TEMPSENSOR is not
    a number in the source and cannot be made into one here: the HAL defines
    it three ways behind preprocessor conditions - channel 17 on some parts,
    18 on others, and on a different ADC in a third family. This board
    answered 18, and test_parity is what checks the stand-in against that
    answer. Asserting a number here would be a second, guessed opinion.
    """
    text = io.open(ADC_C, encoding='utf-8').read()
    table = text.split('s_adcTable[] =')[1].split('};')[0]
    return [{'adc': int(a), 'channel': int(c) if c.isdigit() else None,
             'pin': pin,
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
    # Counted and named off the stand-in's own table, which is checked
    # against the board's by test_parity. Written out here it said seven,
    # and two supply senses were added to `s_adc` - a count in a test is
    # the same second answer a pin table in a document is.
    report.check('every channel the table carries is named, none blank',
                 len(channels) == len(simulated.CHANNELS)
                 and {c['signal'] for c in channels}
                 == {c['signal'] for c in simulated.CHANNELS},
                 sorted(c['signal'] for c in channels))
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

    # Flat, not merely centred. The stand-in gives every channel a burst
    # spread so the meter's marks have something to show; with the reference
    # unpowered there is nothing to spread, and a stand-in that jittered at
    # the rail would teach the opposite of what invariant 9 is for.
    still = afe_off.board.analog.burst(0x7F, 64)['channels']
    report.check('with the front end off a burst has no spread at all',
                 all(c['min_raw'] == c['max_raw'] == int(c['mean_raw'])
                     for c in still.values()),
                 '%d channels' % len(still))

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
        # An internal channel's number is not in the source - see
        # firmware_channels. Everything else about the row still has to agree.
        fields = ('adc', 'pin', 'differential', 'signal')
        if fw['channel'] is not None:
            fields += ('channel',)
        wrong = ['%s: firmware %r, stand-in %r' % (k, fw[k], sim[k])
                 for k in fields if fw[k] != sim[k]]
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

    # A read is the board's shared record now, not a cargo off the bus: the
    # firmware polls the part from its own main loop, and a host that drove
    # SPI2 at the same time was two masters on one bus.
    tool = toolmod.imu(session, op='read')
    report.check('the MCP tool renders the poll loop, not a raw cargo',
                 tool.startswith('imu: loop ') and 'errors' in tool,
                 tool.splitlines()[0])
    report.check('and says how to get a vector when there is none, rather '
                 'than printing zeros',
                 'quaternion' not in tool.lower()
                 or 'op=feature' not in tool
                 or 'no rotation vector yet' in tool,
                 tool.splitlines()[-1])

    toolmod.imu(session, op='feature', report_id=5, interval_us=20000)
    turned = toolmod.imu(session, op='read')
    report.check('and once one is enabled the vector is in the block, counts '
                 'and scaled both',
                 'rotation vector' in turned and 'acc=' in turned,
                 turned.splitlines()[-1])


def test_subsystems(report):
    """The board says what it is made of, and the stand-in says it the same way.

    The list comes from the firmware's command tables - one subsystem per
    table - so a host that kept its own copy would go stale the moment one
    was added. What is checked here is the shape and that the renderer reads
    it; test_parity checks it against a real board.
    """
    session = SimulatedSession()
    rows = session.board.system.channel_map()['subsystems']

    report.check('the stand-in reports subsystems at all',
                 bool(rows), len(rows))
    report.check('each one names itself, says what it is for, and how many '
                 'commands it carries',
                 all(set(r) == {'name', 'what', 'commands'} for r in rows),
                 sorted(rows[0]) if rows else None)
    report.check('and every command count is a number, not a label',
                 all(isinstance(r['commands'], int) and r['commands'] > 0
                     for r in rows),
                 [r['commands'] for r in rows])

    drawn = toolmod.board_info(session, kind='subsystems')
    report.check('board_info renders them one per line, which is how the '
                 'model is given the answer',
                 drawn.startswith('subsystems: %d' % len(rows))
                 and all(r['name'] in drawn for r in rows),
                 drawn.splitlines()[0])


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

    drawn = o.render((0, 0, 0, 1))
    lines = drawn.split(chr(10))
    report.check('the drawing is the height it was asked for',
                 len(lines) == 19, '%d lines' % len(lines))
    report.check('and no line runs past the width',
                 all(len(l) <= 44 for l in lines), max(len(l) for l in lines))

    report.check('the surface is shaded, not outlined',
                 len(set(drawn) & set(o.SHADES)) > 3,
                 ''.join(sorted(set(drawn) & set(o.SHADES))))

    # Turned edge on the board flattens: it occupies fewer rows than it does
    # face on. That is the whole reason a picture beats four decimals.
    quarter = math.sin(math.radians(45))
    edge = o.render((quarter, 0.0, 0.0, math.cos(math.radians(45))))
    drawn_rows = sum(1 for l in lines if l.strip())
    edge_rows = sum(1 for l in edge.split(chr(10)) if l.strip())
    report.check('edge on it flattens - fewer rows than face on',
                 edge_rows < drawn_rows, '%d rows against %d'
                 % (edge_rows, drawn_rows))

    # facing() answers against where the camera stands, which is 13 degrees
    # above the board's plane, so a board at rest shows the component side by
    # sin(13.15) and not by 1.0. What the caption needs is the sign, and what
    # says the camera is being consulted at all is the magnitude.
    up = o.facing((0, 0, 0, 1))
    over = o.facing((0.0, 1.0, 0.0, 0.0))
    report.check('the component side faces the reader, and the solder side '
                 'does when the board is turned over',
                 up > 0.0 and over < 0.0, '%.3f then %.3f' % (up, over))
    report.check('and by exactly how far the camera is above the board',
                 abs(up - math.sin(math.radians(o.VIEW_ELEVATION))) < 1e-9,
                 '%.3f at %.2f degrees' % (up, o.VIEW_ELEVATION))

    # render() turns points through matrix() rather than rotate(), because
    # 45,000 sandwich products a frame do not fit in a frame. The two must
    # not drift: this is what says the fast path still turns things the way
    # the reference does.
    q = o.normalise((0.3, -0.5, 0.2, 0.78))
    m = o.matrix(q)
    v = (0.4, -0.9, 0.25)
    by_matrix = (m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
                 m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
                 m[6] * v[0] + m[7] * v[1] + m[8] * v[2])
    by_quaternion = o.rotate(q, v)
    report.check('the matrix render() uses agrees with rotate()',
                 all(abs(a - b) < 1e-9
                     for a, b in zip(by_matrix, by_quaternion)),
                 '%s against %s' % (by_matrix, by_quaternion))

    # Whatever the mesh came from - the CAD export or the parametric board
    # behind it - the renderer is handed one shape: nine floats of triangle
    # and three of unit normal, scaled to the board's radius.
    verts, faces, normals = o.MODEL_MESH
    report.check('the mesh is indexed: three corners and one normal a face',
                 len(verts) % 3 == 0 and len(faces) % 3 == 0
                 and len(faces) // 3 == len(normals) // 3
                 and max(faces) < len(verts) // 3,
                 '%d triangles over %d vertices'
                 % (len(normals) // 3, len(verts) // 3))
    unit = [abs(math.sqrt(normals[i] ** 2 + normals[i + 1] ** 2
                          + normals[i + 2] ** 2) - 1.0)
            for i in range(0, min(len(normals), 1500), 3)]
    report.check('and every normal is a unit vector',
                 max(unit) < 1e-5, '%.2e worst' % max(unit))
    reach = max(max(abs(verts[i]), abs(verts[i + 1]))
                for i in range(0, len(verts), 3))
    report.check('the surface is scaled to the board radius, so a model in '
                 'millimetres draws the same size as one in inches',
                 0.9 <= reach <= 1.01, '%.4f' % reach)

    # The dial: the same rules, on a different picture. It is a diagram and
    # not a lit surface, so what it must get right is where things are.
    from coaxial import dial

    turned = dial.render(90.0, field=380).split(chr(10))
    report.check('the dial is the height it was asked for',
                 len(turned) == 19, '%d rows' % len(turned))

    # A PROTRACTOR since 2026-08-29: degree labels every 30 all the way
    # round, like the reference face. The axis row is the one carrying both
    # the 180 label and the bare 0.
    face = chr(10).join(turned)
    marks = [str(deg) for deg in range(0, 360, 30)]
    missing = [m for m in marks if m not in face]
    report.check('every 30-degree graduation is labelled',
                 not missing, 'missing %s' % ', '.join(missing))

    axis = [i for i, row in enumerate(turned)
            if '180' in row and row.rstrip().endswith('0')]
    report.check('the 180-to-0 axis is one row', len(axis) == 1, axis)

    # The pointer at 90 degrees goes up from the axis and at 270 down. If
    # those come out the same way round the picture is upside down, which is
    # the one thing a shaft-angle drawing cannot be.
    def pointer_rows(deg):
        drawn = dial.render(deg, field=380).split(chr(10))
        return [i for i, row in enumerate(drawn) if dial.POINTER in row]

    up, down = pointer_rows(90.0), pointer_rows(270.0)
    report.check('the pointer goes up at 90 degrees and down at 270',
                 min(up) < axis[0] <= max(down),
                 'up %s, axis %d, down %s' % (up[:2], axis[0], down[-2:]))

    flat = dial.render(0.0, field=380).split(chr(10))[axis[0]]
    report.check('and at zero it reaches toward the rim along the axis',
                 flat.count(dial.POINTER) >= int(dial.RADIUS) - 4, flat)

    report.check('a weak field draws no pointer at all',
                 dial.POINTER not in dial.render(90.0, field=3),
                 'field 3 gauss')
    report.check('and the caption says why',
                 'no magnet' in dial.picture({'value': 20, 'degrees': 1.8,
                                              'loop': 'running', 'updates': 1,
                                              'errors': 0, 'field': 3}))

    # The sensor box moved off the face and into the view's HUD panel on
    # 2026-08-29 - the face kept four voices (rim, sweep, sensor, pointer)
    # and read as a party. The face's contract now: the instrument and the
    # reading, nothing else.
    report.check('the face carries no sensor box - the HUD names the part',
                 not any('A1335' in row for row in turned))

    report.check('a reading the board never took prints as one, not as zero '
                 'degrees',
                 'no reading' in dial.picture({'value': None, 'loop': 'off',
                                               'error': 'lost AFE_ON'}))

    counts = dial.picture({'value': 0x5000 | 2048, 'degrees': 180.0,
                           'loop': 'running', 'updates': 9, 'errors': 0})
    report.check('the counts lead the degrees - the counts are what the part '
                 'said and the degrees are this host arithmetic',
                 '2048 of 4096 counts' in counts, counts.splitlines()[0])

    plain = o.picture((0, 0, 0, 1))
    report.check('the picture carries the angles and the quaternion both - '
                 'the drawing is a reading, and the numbers are what it is '
                 'a reading of',
                 'rpy' in plain and 'real +1.0000' in plain,
                 plain.splitlines()[0])
    report.check('the quaternion leads it, because it is what moves when '
                 'the board does',
                 plain.splitlines()[0].startswith('q   i '),
                 plain.splitlines()[0])


def test_scaling(report):
    """The three conversions this host claims to know, as arithmetic.

    Nothing here needs a board, and that is the point: a wrong constant does
    not fail loudly on hardware, it returns a plausible number. Each figure
    below is checked against the schematic value it was traced from.
    """
    bus = scaling.DCBUS_ONBOARD
    report.check('the DC link divider is the schematic pair R12/R11',
                 (bus.r_top, bus.r_bottom) == (49900.0, 2200.0),
                 '%.0f/%.0f' % (bus.r_top, bus.r_bottom))
    report.check('full scale is 78.15 V - 24 % over the 63 V rating, and '
                 'deliberate (invariant 11)',
                 abs(bus.volts(65535) - 78.15) < 0.01,
                 '%.3f V' % bus.volts(65535))

    ntc = scaling.NTC_ONBOARD
    report.check('mid-scale is exactly 25.00 C, which is the AFE-off '
                 'artefact and not a temperature',
                 abs(ntc.celsius(32768) - 25.0) < 0.005,
                 '%.4f C' % ntc.celsius(32768))
    report.check('B is 3380, the NCU18XH103D60RB value, not 3950',
                 ntc.beta == 3380.0, '%.0f' % ntc.beta)
    for rail in (0, 65536):
        try:
            ntc.celsius(rail)
            report.check('raw %d is refused, not reported' % rail, False)
        except ValueError as exc:
            report.check('raw %d is refused, not reported' % rail,
                         'not recoverable' in str(exc), str(exc))

    phase = scaling.PHASE_ONBOARD
    report.check('the gain is the schematic resistor ratio, not a literal - '
                 'THS4551 Rf 1.5k over Rg 330',
                 abs(phase.gain - 1500.0 / 330.0) < 1e-9,
                 '%.4f V/V' % phase.gain)
    report.check('the shunt is two 7 mohm WSHM2818 in parallel',
                 abs(phase.r_shunt - 0.0035) < 1e-12,
                 '%.5f ohm' % phase.r_shunt)
    report.check('3.5 mohm x that gain is 15.909 mV per amp',
                 abs(phase.volts_per_amp - 0.0159090909) < 1e-9,
                 '%.6f V/A' % phase.volts_per_amp)
    report.check('no gain above 9.43 V/V could represent 100 A at all, which '
                 'is what bounds the chain independently of reading it',
                 phase.gain < phase.vref / (phase.r_shunt * 100.0),
                 'ceiling %.2f V/V' % (phase.vref / (phase.r_shunt * 100.0)))
    report.check('the ADC runs out at 207.4 A, so 100 A sits under half the '
                 'differential span - the same headroom the DC link keeps',
                 abs(phase.full_scale_amps - 207.43) < 0.05 and
                 phase.amps(16384) > 100.0,
                 '%.2f A full scale, %.2f A at half span'
                 % (phase.full_scale_amps, phase.amps(16384)))
    counts = 100.0 * phase.volts_per_amp / phase.vref * 32768.0
    report.check('100 A round-trips through the code it produces',
                 abs(phase.amps(round(counts)) - 100.0) < 0.01,
                 'raw %d -> %.4f A' % (round(counts), phase.amps(round(counts))))
    report.check('a negative code is a negative current - the shunt is '
                 'differential and direction survives',
                 phase.amps(-1580) < 0 and
                 abs(phase.amps(-1580) + phase.amps(1580)) < 1e-9,
                 '%.4f A' % phase.amps(-1580))


def _desk_rows(**over):
    """Seven channels in the shape read_all() hands back."""
    base = [('Phase U', True, 1500), ('Phase V', True, -8293),
            ('Phase W', True, 490), ('Clevel', False, 1012),
            ('NTC', False, 40207), ('DC bus', False, 20600),
            ('Cinj', False, 14613)]
    rows = []
    for index, (signal, differential, mean) in enumerate(base):
        mean = over.get(signal, mean)
        divisor = 32768.0 if differential else 65536.0
        rows.append({'index': index, 'signal': signal,
                     'differential': differential, 'mean_raw': mean,
                     'min_raw': mean - 20, 'max_raw': mean + 20,
                     'unit': None,
                     # read_all() carries this for every channel, and it is
                     # what a channel with no defined signal prints instead
                     # of a raw code. Zero here would let that check pass on
                     # a number that is not one.
                     'volts_at_pin': mean / divisor * 3.3})
    return rows


def test_desk(report):
    """The meter bridge: every channel's own scale, in its own unit."""
    rows = _desk_rows()
    for row in rows:
        low = -207.4 if row['differential'] else 0.0
        row['span'] = (low, 207.4 if row['differential'] else 3.3)
        row['reading'] = desk.fraction(row) * row['span'][1]

    face = desk.Desk().update(rows).split('\n')

    report.check('one row per channel, and nothing else',
                 len(face) == len(rows), '%d rows' % len(face))
    report.check('every row is the same width',
                 len({len(line) for line in face}) == 1,
                 'widths %s' % sorted({len(line) for line in face}))
    report.check('no row says dB - the scales are in the unit the quantity '
                 'has, which is the whole point of laying it on its side',
                 'dB' not in ' '.join(face))
    report.check('and every row carries its own scale',
                 all('207' in line for line in face[:3])
                 and all('3.3' in line for line in face[3:]),
                 face[0].strip()[:34])

    report.check('a bipolar channel is drawn from the centre',
                 desk.CENTRE in face[0] or desk.FULL in face[0],
                 repr(face[0][18:40]))
    # Column 18 is where the bar starts: three for the name, a space, and
    # SCALE for the channel's own ends.
    report.check('a unipolar one from the left',
                 face[3][18] == desk.FULL, repr(face[3][18:34]))

    # The span is the caller's to compute: deriving it from the reading and
    # its fraction of full scale is right for a linear channel and wrong for
    # the thermistor, which is logarithmic in the code.
    bare = dict(rows[0])
    bare.pop('span')
    report.check('a channel with no scale supplied says so rather than '
                 'inventing one',
                 desk.span(bare) is None
                 and '?' in desk.Desk().update([bare]))

    report.check('the ink says where the converter is before the number is '
                 'read', (desk.Desk._ink(0.1), desk.Desk._ink(0.8),
                          desk.Desk._ink(0.99))
                 == (ansi.GREEN, ansi.AMBER, ansi.RED))

    gate_drivers = desk.Desk(decay=0.04)
    loud = _desk_rows(**{'Phase U': 30000})
    for row in loud:
        row['span'] = (-207.4, 207.4) if row['differential'] else (0.0, 3.3)
    gate_drivers.update(loud)
    held = gate_drivers._held[0][1]
    quiet = _desk_rows(**{'Phase U': 30})
    for row in quiet:
        row['span'] = (-207.4, 207.4) if row['differential'] else (0.0, 3.3)
    gate_drivers.update(quiet)
    report.check('a peak falls by the decay and no further - that is the '
                 'ballistics, not a reading',
                 abs(gate_drivers._held[0][1] - (held - 0.04)) < 1e-9,
                 '%.3f -> %.3f' % (held, gate_drivers._held[0][1]))
    gate_drivers.update(loud)
    report.check('and it jumps back the instant the level does',
                 abs(gate_drivers._held[0][1] - held) < 1e-9,
                 '%.3f' % gate_drivers._held[0][1])


def test_tumble(report):
    """The stand-in's attitude, which is the only thing that moves in a
    view running without a board."""
    from coaxial import orientation as o
    from coaxial.simulated import _tumble

    def angles(seq):
        return o.euler_degrees(tuple(c / 16384.0 for c in _tumble(seq, 16384)))

    unit = [abs(sum((c / 16384.0) ** 2 for c in _tumble(seq, 16384)) - 1.0)
            for seq in range(0, 256, 8)]
    report.check('every attitude it invents is a unit quaternion',
                 max(unit) < 1e-3, '%.2e worst' % max(unit))

    travel = [max(angles(s)[a] for s in range(256))
              - min(angles(s)[a] for s in range(256)) for a in range(3)]
    report.check('all three angles move - it used to turn about Z alone, '
                 'which is the board spinning in its own plane',
                 all(span > 90.0 for span in travel),
                 'roll %.0f, pitch %.0f, yaw %.0f' % tuple(travel))

    # The sequence byte wraps at 256. A rate that had not finished a whole
    # turn by then would snap the board back to level once a cycle.
    ends = [abs(a - b) for a, b in zip(angles(255), angles(0))]
    report.check('and it comes back where it started, so the wrap is smooth',
                 all(gap < 5.0 or gap > 355.0 for gap in ends),
                 '%.1f, %.1f, %.1f apart' % tuple(ends))


def test_peak_hold(report):
    """What the mark is for: standing where the signal went, after it left.

    The ballistics themselves are checked in test_desk. What is here is the
    lag, and the bug that a single mirrored magnitude had.
    """
    def spike_then_quiet(meter, rounds):
        row = dict(index=0, signal='Phase U', differential=True, unit='mA',
                   span=(-207.4, 207.4), reading=0.0)

        def frame(mean, low, high):
            row.update(mean_raw=mean, min_raw=low, max_raw=high)
            meter.update([row])
            return meter._held[0]

        frame(24000, -500, 26000)
        for _ in range(rounds):
            settled = frame(500, -600, 700)
        return settled

    low, high = spike_then_quiet(desk.Desk(), 4)
    report.check('a peak still stands above the reading four updates on',
                 high > 700 / 32768.0,
                 '%+.3f held against %+.3f now' % (high, 700 / 32768.0))

    # One magnitude, mirrored, put the mark where the current had never been:
    # a phase sitting at +62 A drew its caret at -62.
    report.check('and the two ends are held apart, not mirrored',
                 low > -0.05 and high > 0.5, '%+.3f..%+.3f' % (low, high))


def test_ascii3d(report):
    """The AsciiEffect port: its ramp, its mapping, and a stable canvas."""
    report.check("the ramp is AsciiEffect's own, darkest first",
                 ascii3d.CHARACTERS == ' .:-+*=%@#', ascii3d.CHARACTERS)

    # floor((1 - brightness) * (len - 1)), then inverted for dark mode. Both
    # halves matter: without the inversion the background is '#' and the lit
    # face is a space, which on a terminal is a photographic negative.
    ramp = ascii3d.CHARACTERS
    for brightness in (0.0, 0.25, 0.5, 0.75, 1.0):
        want = ramp[len(ramp) - 1 - int((1.0 - brightness) * (len(ramp) - 1))]
        report.check('brightness %.2f maps the way AsciiEffect maps it'
                     % brightness,
                     ascii3d.brightness_char(brightness) == want, want)

    report.check('lit is heavy ink and unlit is a space, with invert on',
                 ascii3d.brightness_char(1.0) == '#'
                 and ascii3d.brightness_char(0.0) == ' ')
    report.check('and the other way round with it off, which is the '
                 "reference's light mode",
                 ascii3d.brightness_char(1.0, invert=False) == ' '
                 and ascii3d.brightness_char(0.0, invert=False) == '#')

    # A closer light was tried, to force some modelling onto a flat board
    # seen face-on. It was the wrong answer: what that case wanted was
    # RESOLUTION, and the reference's own ratio is what the port keeps.
    # The reference's light is where updateLightPosition() puts it, not the
    # (100, 100, 400) the PointLight is constructed with - resetPositions()
    # overwrites that on every load, and this asserted the dead value.
    report.check('the light sits where the reference actually puts it',
                 ascii3d.LIGHT_DISTANCE == 4.12
                 and ascii3d.LIGHT_DIRECTION == (35.9, -35.9, 200.0),
                 '%.2f radii along %s' % (ascii3d.LIGHT_DISTANCE,
                                          ascii3d.LIGHT_DIRECTION))

    # A canvas that changes height as the board turns leaves the last row of
    # the previous frame on screen. It is a drawing of a fixed size.
    heights = set()
    for degrees in (0, 5, 15, 30, 60, 90):
        radians = math.radians(degrees) / 2.0
        turned = (math.sin(radians), 0.0, 0.0, math.cos(radians))
        heights.add(len(orientation.render(turned, 34, 15).split('\n')))
    report.check('the canvas is the same height at every rotation',
                 heights == {15}, 'heights %s' % sorted(heights))

    flat = orientation.render((0.0, 0.0, 0.0, 1.0), 34, 15)
    edge = orientation.render((math.sin(math.pi / 4), 0.0, 0.0,
                               math.cos(math.pi / 4)), 34, 15)
    report.check('and the drawing answers the rotation it was given',
                 flat != edge)
    # Zoom moves the camera rather than scaling the projection, so the
    # perspective stays honest: a nearer board is one seen from nearer.
    near = orientation.render((0.0, 0.0, 0.0, 1.0), 60, 20, zoom=2.0)
    fit = orientation.render((0.0, 0.0, 0.0, 1.0), 60, 20, zoom=1.0)
    far = orientation.render((0.0, 0.0, 0.0, 1.0), 60, 20, zoom=0.5)

    def drawn(picture):
        return sum(1 for c in picture if c not in ' \n')

    report.check('zoom in draws more board, zoom out less',
                 drawn(far) < drawn(fit) < drawn(near),
                 '%d < %d < %d cells' % (drawn(far), drawn(fit), drawn(near)))
    report.check('and the fit is what fills the shorter axis',
                 max(len(line) for line in fit.split('\n')) <= 60)

    # The light belongs to the world, not to the camera. Moving the viewer
    # must not change how the board is lit - measured before this was fixed,
    # dropping the camera from 90 degrees to 60 darkened the whole board by
    # two ramp steps with nothing about the board or the light having moved.
    # One triangle in the XY plane, big enough to cover cells.
    face = ([-0.6, -0.6, 0.0, 0.6, -0.6, 0.0, 0.0, 0.6, 0.0],
            [0, 1, 2], [0.0, 0.0, 1.0])
    square = (1, 0, 0, 0, 1, 0, 0, 0, 1)
    edgewise = (1, 0, 0, 0, 0, -1, 0, 1, 0)

    def inked(picture):
        return sum(1 for c in picture if c.strip())

    # Projected area, which is the whole difference between rasterising a
    # mesh and splatting points sampled off it. Four earlier versions here
    # sampled the surface by TRUE area, so a wall got its full area however
    # it was turned, and 43% of this board's area is wall - which drew as
    # salt and pepper over the whole picture.
    report.check('a face-on triangle covers cells',
                 inked(ascii3d.render(face, square, 30, 10, distance=3.0)) > 0,
                 '%d cells' % inked(ascii3d.render(face, square, 30, 10,
                                                   distance=3.0)))
    report.check('and the same triangle seen edge-on covers none',
                 inked(ascii3d.render(face, edgewise, 30, 10,
                                      distance=3.0)) == 0)

    # The light belongs to the world, not to the camera. Moving the viewer
    # must not change how a face is lit - measured before this was fixed,
    # dropping the camera from 90 degrees to 60 darkened the whole board by
    # two ramp steps with nothing about the board or the light having moved.
    lamp = (0.7, -0.7, 4.0)

    def glyphs(elevation):
        turn = orientation.viewpoint(0.0, elevation)
        moved = tuple(sum(turn[r * 3 + k] * lamp[k] for k in range(3))
                      for r in range(3))
        drawn = ascii3d.render(face, turn, 30, 10, distance=3.0, light=moved)
        return set(c for c in drawn if c.strip())

    report.check('the same face lit the same way from two camera angles',
                 glyphs(90.0) == glyphs(60.0),
                 '%s against %s' % (sorted(glyphs(90.0)), sorted(glyphs(60.0))))

    # The font's shape is the only thing the projection does not work in.
    # Get it wrong and the picture is not blurred, it is stretched: a round
    # board drawn at the wrong cell aspect reads as a board turned edge-on.
    for aspect in (1.0, 2.0):
        cols, rows, cell_rows = ascii3d.grid(60, 20, 2, aspect)
        report.check('a cell %.1f times as tall as wide is %d rows deep'
                     % (aspect, cell_rows),
                     cell_rows == int(round(aspect * 2)) and cols == 120,
                     '%dx%d framebuffer' % (cols, rows))

    # Straight down at a board that is round, the drawing has to come out
    # round too - in the font's units, not the framebuffer's.
    flat_on = orientation.viewpoint(0.0, 90.0)
    cols, rows, cell_rows = ascii3d.grid(80, 24)
    distance, off_x, off_y = ascii3d.fit(orientation.MODEL_MESH[0], flat_on,
                                         cols, rows)
    picture = ascii3d.render(orientation.MODEL_MESH, flat_on, 80, 24,
                             distance=distance, centre=(off_x, off_y))
    drawn = [line for line in picture.split(chr(10)) if line.strip()]

    # The true span, not the right edge: the renderer strips trailing spaces
    # and not leading ones, so len(line) counts the left margin too. That
    # read 33% wide of round on a board that was drawn correctly.
    left = min(len(line) - len(line.lstrip()) for line in drawn)
    wide = max(len(line) for line in drawn) - left
    tall = len(drawn) * (cell_rows / float(ascii3d.SUPERSAMPLE))
    report.check('a round board drawn face-on comes out round',
                 0.85 <= wide / tall <= 1.18,
                 '%.2f wide to tall' % (wide / tall))

    report.check('a board seen face-on uses more than one character - that '
                 'was the whole complaint',
                 len(set(flat.replace(' ', '').replace('\n', ''))) > 3,
                 'glyphs %s' % ''.join(sorted(
                     set(flat.replace(' ', '').replace('\n', '')))))


def test_clock_reference(report):
    """UTC when NTP answers, this PC when it does not, and never quietly.

    The board has no clock of its own, so a sync that raised would leave a
    capture with no time on it at all - falling back is right. Falling back
    silently is not: this PC was 947 ms off UTC and 25 ppm slow the day
    this was written, with Windows reporting a good sync, so a capture that
    believes it is on UTC when it is on this machine is wrong by a second
    and says nothing.

    NTP is stubbed rather than reached. A test that needs the network to
    pass is a test that fails on a train.
    """
    import coaxial.clock as clockmod
    from coaxial.errors import RigError

    def refuse(*_, **__):
        raise RigError('no route')

    session = SimulatedSession()
    real = clockmod.ntp_offset
    clockmod.ntp_offset = refuse
    try:
        fell_back = session.board.clock.sync(seconds=0.2, rounds=1)
    finally:
        clockmod.ntp_offset = real

    report.check('asked for UTC with no NTP, it lands on the PC clock',
                 fell_back.reference == 'pc', fell_back.reference)
    report.check('and carries why, so a capture cannot think it is UTC',
                 'no route' in fell_back.note, fell_back.note)
    report.check('no PC correction is claimed when none was measured',
                 fell_back.pc_ppm is None, fell_back.pc_ppm)
    report.check('it still produces a rate - the capture keeps its timestamps',
                 fell_back.hz > 0, fell_back.hz)

    # A host 40 ppm slow makes the board look 40 ppm fast, because the rate
    # was counted in that host's short seconds. Signed the other way first,
    # and it put the board at +35 ppm where an independent method had -13.
    clockmod.ntp_offset = lambda *a, **k: (0.0, 0.001)
    try:
        flat = session.board.clock.sync(seconds=0.2, rounds=1)
    finally:
        clockmod.ntp_offset = real

    report.check('a host that agrees with UTC changes nothing',
                 flat.reference == 'utc' and abs(flat.pc_ppm) < 1e-9,
                 flat.pc_ppm)
    report.check('a sync knows what its own noise floor is, so a rate under '
                 'it can be called bounded rather than measured',
                 flat.floor_ppm > 0, flat.floor_ppm)


def test_link_bench(report):
    """What a transaction costs against what its bitrate allows.

    The arithmetic is checked here and the measurement is not: a threshold
    on how fast this bench's USB happens to be would be an expected value
    about hardware, which this repository does not keep (invariant 10). So
    every number below is one a reader can do by hand.
    """
    from coaxial import bench

    # 8N1 is ten bits a byte, so 115200 baud is 11520 B/s and 250 bytes is
    # 250/11520 of a second. Nothing about the board is in that.
    report.check('the floor is bytes x 10 / baud, and nothing else',
                 abs(bench.frame_seconds(250, 115200) - 0.0217013888) < 1e-9,
                 '%.6f ms' % (bench.frame_seconds(250, 115200) * 1e3))
    report.check('halving the bitrate doubles the floor',
                 bench.frame_seconds(64, 57600)
                 == 2 * bench.frame_seconds(64, 115200), None)

    # 100 bytes each way at 115200 is 2000 bits, 17.361 ms. A round trip
    # measured at twice that used half the wire it was given.
    made_up = bench.Result('made up', 100, 100,
                           [0.03, 0.0347222222, 0.06], 115200)
    report.check('wire bytes are both directions',
                 made_up.wire_bytes == 200, made_up.wire_bytes)
    report.check('the median is what efficiency is taken from, not the best',
                 abs(made_up.efficiency - 0.5) < 1e-6,
                 '%.3f' % made_up.efficiency)
    report.check('overhead is the part that was not the wire',
                 abs(made_up.overhead - 0.0173611111) < 1e-9,
                 '%.3f ms' % (made_up.overhead * 1e3))
    report.check('payload leaves out the framing both frames carry',
                 abs(made_up.payload_bytes_per_second
                     - (200 - 8) / 0.0347222222) < 1e-6,
                 '%.0f B/s' % made_up.payload_bytes_per_second)

    session = SimulatedSession()
    results = bench.run(session.board, rounds=2)
    report.check('every case reports what it put on the wire',
                 all(r.wire_bytes >= 8 for r in results),
                 [r.wire_bytes for r in results])
    report.check('the ping is the smallest transaction there is',
                 results[0].name == 'ping'
                 and results[0].wire_bytes == min(r.wire_bytes for r in results),
                 results[0].wire_bytes)
    report.check('and the cases climb, so the curve reads in one glance',
                 [r.wire_bytes for r in results[:-1]]
                 == sorted(r.wire_bytes for r in results[:-1]),
                 [r.wire_bytes for r in results[:-1]])

    drawn = '\n'.join(bench.table(results))
    report.check('the table renders every case',
                 all(r.name in drawn for r in results), None)
    report.check('and a stand-in beating its own bitrate says so, rather '
                 'than printing a number that looks measured',
                 'no wire to be slow' in drawn, drawn.splitlines()[-1][:60])


def test_gate_driver_arming(report):
    """Arming a power stage is asked for by name, or it does not happen.

    The 2EDL8034's inputs are independent and it has no interlock, so TIM1's
    dead time is the only thing between the two FETs of a leg. Everything
    here is about making that impossible to skip by accident; the dead time
    itself is checked against the silicon, not against a stand-in.
    """
    from coaxial import Coaxial63100
    from coaxial.errors import RigError

    # power_afe SAID, not inherited: the interlock refusal under
    # test reports the volts it read, and with the rail down it
    # refuses for the rail instead and never reads them.
    rig = Coaxial63100(simulated_device=True, power_afe=True).open()
    try:
        report.check('nothing is armed on the way in',
                     rig.gates.armed() is False, rig.gates.armed())

        try:
            rig.write(analog={'Phase U': 0.25})
            refused = None
        except RigError as exc:
            refused = str(exc)
        report.check('a duty write is refused while the gate drivers are not armed '
                     '- writing a level must not be what arms a power stage',
                     refused is not None, refused)
        report.check('and the refusal names the call that would arm it, '
                     'rather than leaving the caller to guess',
                     refused and 'gates.arm()' in refused, refused)

        # The schematic wants the charge pump up and the level detector
        # tripped first. The stand-in reports neither, the same way the
        # unmodified bench board does not - Cinj 0.77 V and Clevel 0.06 V
        # against 3 V each, measured 2026-08-27.
        try:
            rig.gates.arm(bypass_sto=True)
            held = None
        except RigError as exc:
            held = str(exc)
        report.check('the interlock refuses before the charge pump is up',
                     held is not None, held)
        report.check('and the refusal carries the volts it read, not just '
                     'the fact that it refused',
                     held and 'Cinj' in held and 'V' in held, held)

        rig.gates.arm(bypass_sto=True, ignore_interlock=True)
        report.check('after gates.arm(), MOE is set', rig.gates.armed(), True)
        report.check('and the same write goes through',
                     rig.write(analog={'Phase U': 0.25})['Phase U'] > 0,
                     rig.board.gate_drivers.state()['duty'])

        rig.gates.disarm()
        report.check('gates.disarm() clears it again',
                     rig.gates.armed() is False, rig.gates.armed())

        # The check reads BDTR every time rather than trusting one reading:
        # a .ioc regeneration and a CubeMX mode name bound to the wrong
        # channel have both moved TIM1 in this repository without saying so.
        state = dict(rig.board.gate_drivers.state(), deadtime=0)
        rig.board.gate_drivers.state = lambda: state
        try:
            rig.gates.arm(ignore_interlock=True)
            stopped = None
        except RigError as exc:
            stopped = str(exc)
        report.check('a gate driver stage reporting no dead time will not arm',
                     stopped is not None, stopped)
        report.check('and the refusal says what to look at',
                     stopped and 'DTG' in stopped and '.ioc' in stopped,
                     stopped)
        report.check('the dead time is checked before the interlock, because '
                     'no interlock makes a gate driver stage with none safe to arm',
                     'DTG' in (stopped or ''), None)
    finally:
        rig.close()


#: High side and low side of each leg, which must never be on together.
PAIRS = (('UH', 'UL'), ('VH', 'VL'), ('WH', 'WL'))


def test_gate_snapshot(report):
    """The six gate signals, and the one state they must never show.

    Read as one IDR load on the board so the six are the same instant: six
    asks at 50 kHz can straddle an edge and show a leg with both FETs on,
    which is the state the dead time exists to prevent. A stand-in that
    could show it would teach a reader the wrong thing, so it cannot.
    """
    sys.path.insert(0, os.path.join(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))), 'tools'))
    import show_gate_drivers

    session = SimulatedSession()
    gate_drivers = session.board.gate_drivers

    state = gate_drivers.state()
    report.check('every gate is named, low side and high side per leg',
                 set(state['pins']) == {'UL', 'UH', 'VL', 'VH', 'WL', 'WH'},
                 sorted(state['pins']))
    report.check('and the counter is reported beside them, so a reader knows '
                 'where in the period the snapshot landed',
                 0 <= state['pins_at'] < state['period'], state['pins_at'])

    report.check('with the outputs disabled every gate is low - both FETs '
                 'of every leg off, which is what MOE clear means',
                 not any(state['pins'].values()), state['pins'])

    gate_drivers.bypass_break(True)
    gate_drivers.enable()
    gate_drivers.duty((0, 0, 0))
    seen = [gate_drivers.state()['pins'] for _ in range(40)]
    report.check('armed at zero duty, the low sides carry it',
                 all(p['UL'] and not p['UH'] for p in seen), seen[0])

    period = gate_drivers.state()['period']
    gate_drivers.duty((period // 2, period // 2, period // 2))
    seen = [gate_drivers.state()['pins'] for _ in range(60)]
    report.check('at half duty both halves of the period show up',
                 any(p['UH'] for p in seen) and any(p['UL'] for p in seen),
                 [sum(p['UH'] for p in seen), sum(p['UL'] for p in seen)])
    conducting = [p for p in seen
                  if any(p[h] and p[l] for h, l in PAIRS)]
    report.check('and no sample ever has both FETs of a leg on',
                 not conducting, 'checked %d samples' % len(seen))

    drawn = '\n'.join(show_gate_drivers.gate_rows(gate_drivers.state(), 100))
    report.check('the view draws one row per leg', drawn.count('phase') == 3,
                 drawn.splitlines()[0])
    both_on = dict(gate_drivers.state())
    both_on['pins'] = dict(both_on['pins'], UH=True, UL=True)
    report.check('and says so in words when a leg shows both on, rather '
                 'than leaving it to be spotted in a row of ones',
                 'BOTH ON' in '\n'.join(show_gate_drivers.gate_rows(both_on, 100)),
                 None)
    gate_drivers.disable()
    gate_drivers.bypass_break(False)


def test_dead_time(report):
    """Settable, floored at 20 ns, and skewable - all refused in words.

    The 2EDL8034 has no interlock, so the dead time is the only thing
    between the two FETs of a leg. Everything here is about the floor being
    a floor. What the skew does at the gates is not checked and cannot be
    from here: it needs two probes and a scope.
    """
    from coaxial import Coaxial63100
    from coaxial.errors import RigError

    # power_afe SAID, not inherited: the interlock refusal under
    # test reports the volts it read, and with the rail down it
    # refuses for the rail instead and never reads them.
    rig = Coaxial63100(simulated_device=True, power_afe=True).open()
    try:
        gates = rig.board.gate_drivers
        at_rest = gates.dead_time()
        report.check('it reads a dead time, a skew and the floor',
                     set(at_rest) == {'nanoseconds', 'skew', 'floor'},
                     at_rest)

        low = gates.dead_time(1)
        report.check('asking for less than the floor gets the floor, not a '
                     'refusal - a bridge still switches, just not that fast',
                     low['nanoseconds'] > 0
                     and low['nanoseconds'] <= at_rest['nanoseconds'],
                     low['nanoseconds'])

        wide = gates.dead_time(200)
        report.check('a bigger ask lands on a bigger DTG count',
                     wide['nanoseconds'] > low['nanoseconds'],
                     '%d -> %d ns' % (low['nanoseconds'], wide['nanoseconds']))

        skewed = gates.dead_time(200, skew=8)
        report.check('the skew is carried and reported back',
                     skewed['skew'] == 8, skewed['skew'])
        report.check('and the dead time it was set against is unchanged - '
                     'the pair still averages what was asked for',
                     skewed['nanoseconds'] == wide['nanoseconds'],
                     skewed['nanoseconds'])

        gates.dead_time(21)
        try:
            gates.dead_time(21, skew=40)
            refused = None
        except RigError as exc:
            refused = str(exc)
        report.check('a skew that would take either half under the floor is '
                     'refused', refused is not None, refused)
        report.check('and the refusal says what to do about it',
                     refused and 'raise the dead time' in refused, refused)
    finally:
        rig.close()


#: Every live view, run headless against the stand-in. The flag exists on
#: all of them so a frame count can end one.
VIEWS = ('show_angle', 'show_capture', 'show_desk', 'show_gate_drivers',
         'show_orientation', 'show_thermal_observer', 'show_rotor_observer')


def test_views(report):
    """Each view draws three frames against the stand-in without raising.

    The suites check the classes a view is built from, not the view. Both of
    these got through: `rig.acquire()` called `daq.read`, renamed months
    earlier and gone from both implementations, and the stand-in's CENTRE
    map stopped at channel 8 after the die thermometer took 9. Neither is
    visible to a test that never runs the program.
    """
    import subprocess

    tools = os.path.join(REPO, 'host', 'tools')
    for view in VIEWS:
        done = subprocess.run(
            [sys.executable, os.path.join(tools, view + '.py'),
             '--simulated', '--frames', '3'],
            capture_output=True, text=True, timeout=120,
            stdin=subprocess.DEVNULL)
        tail = (done.stderr or done.stdout or '').strip().splitlines()
        report.check('%s draws simulated' % view,
                     done.returncode == 0, tail[-1] if tail else '')


def main():
    report = Report()
    for test in (test_session, test_board_info, test_analog_read,
                 test_self_test_and_link, test_gpio_gate,
                 test_channel_table, test_imu, test_subsystems,
                 test_orientation, test_scaling, test_desk,
                 test_tumble, test_peak_hold, test_ascii3d,
                 test_clock_reference, test_link_bench,
                 test_gate_driver_arming, test_gate_snapshot,
                 test_closing_leaves_another_session_armed,
                 test_dead_time, test_views):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


def test_closing_leaves_another_session_armed(report):
    """A session that did not arm the stage must not disarm it on the way out.

    Measured 2026-08-29: three switching runs ended the moment a second
    session asked the board an unrelated question. `close()` disarmed
    unconditionally, so a read-only peer looked exactly like a stage
    tripping - MOE clear, no fault, duty zeroed.

    The safety net stays: undo what THIS session armed, and otherwise only
    when nobody else is left to own it.
    """
    from coaxial import Coaxial63100

    def rig_that_thinks(others):
        rig = Coaxial63100(simulated_device=True, power_afe=False).open()
        rig._others_here = lambda: others
        return rig

    rig = rig_that_thinks(True)
    rig.gates.arm(bypass_sto=True, ignore_interlock=True)
    report.check('a session that armed the stage says so',
                 rig.gates.armed_here is True)
    board = rig.board
    rig.close()
    report.check('and disarms it on the way out, peers or not',
                 board.gate_drivers.state()['pwm_enabled'] is False)

    # The one that matters: this session never armed anything.
    rig = rig_that_thinks(True)
    rig.gates.arm(bypass_sto=True, ignore_interlock=True)
    board = rig.board
    rig.gates._armed_here = False          # as if a peer had armed it
    rig.close()
    report.check('a session that did NOT arm it leaves it alone '
                 'while another session is on the board',
                 board.gate_drivers.state()['pwm_enabled'] is True)

    # ...and the net still catches the last one out.
    rig = rig_that_thinks(False)
    rig.gates.arm(bypass_sto=True, ignore_interlock=True)
    board = rig.board
    rig.gates._armed_here = False
    rig.close()
    report.check('the last session out disarms it even so',
                 board.gate_drivers.state()['pwm_enabled'] is False)


if __name__ == '__main__':
    sys.exit(main())
