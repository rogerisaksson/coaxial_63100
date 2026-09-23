"""Tool schemas and handlers."""
import os
import re
import subprocess
import sys
import time

from coaxial import DividerParams, NtcParams
from coaxial.comm import ports, protocol
from coaxial.devices import scaling
from coaxial.draw import orientation as orient
from coaxial.errors import LINK_FAULTS, DeviceStateError
from coaxial.comm.wire import pack

ROTATION_VECTOR = 0x05

from. import render
from .docs import docs as _docs

#: What asking an older board for something newer raises: a link fault, no
#: subsystem or op behind the attribute, no such field in the reply.
OLDER_FIRMWARE = LINK_FAULTS + (AttributeError, KeyError)

_PIN = {'type': 'string', 'description': 'Pin as PORT+NUMBER, e.g. B2 or E15'}
_PORT = {'type': 'string', 'description': 'Port letter A-K'}

# Which section of the map board_info answers with.
BOARD_INFO_KINDS = ('all', 'analog', 'digital', 'reserved', 'identity',
                    'subsystems', 'parts')

TOOLS = [
    {
        'name': 'board_info',
        'description': "Identity, clock, the channels the board reports with their directions, and what is fitted on it. kind narrows it to one section.",
        'description_terse': "Identity, clock, channels, fitted parts. kind: analog|digital|reserved|identity|subsystems|parts|all.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'refresh': {'type': 'boolean'},
                'kind': {'type': 'string',
                         'enum': list(BOARD_INFO_KINDS),
                         'description': 'which section; omit for all'},
            },
        },
    },
    {
        'name': 'analog_read',
        'description': "Sample the board's ADC channels: mean raw, pin volts, plus degC for the NTC and volts for the DC bus. Needs the AFE on. Not the IMU.",
        'description_terse': "The board's ADC channels: mean raw, pin volts, NTC degC, bus volts. Not the IMU.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'ch': {'type': 'array', 'items': {'type': 'string'},
                       'description': 'Channel indices or short names; omit for all'},
                'samples': {'type': 'integer'},
                'rate_hz': {'type': 'number'},
                'ntc_beta': {'type': 'number', 'description': 'Override thermistor B'},
                'ntc_r25': {'type': 'number'},
                'vref': {'type': 'number', 'description': 'Measured reference, default 3.3'},
            },
        },
    },
    {
        'name': 'docs',
        'description': "This board's own documents: background, not a live reading. find=TEXT to answer a question; doc= alone returns headings, not answers.",
        'description_terse': "This board's documents, not a reading. find=TEXT answers; doc= alone lists headings.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'doc': {'type': 'string',
                        'description': 'README|CLAUDE|ARCHITECTURE|PROTOCOL|HARDWARE|FINDINGS|MODELS'},
                'section': {'type': 'string', 'description': 'A heading from that document'},
                'find': {'type': 'string', 'description': 'Search all of them instead'},
            },
        },
    },
    {
        'name': 'self_test',
        'description': 'What the board can prove about itself: clocks, calibration, firmware checksum. Pass/fail only where no external reference is needed.',
        'description_terse': 'What the board proves about itself: clocks, calibration, checksum.',
        'inputSchema': {
            'type': 'object',
            'properties': {'failures_only': {'type': 'boolean'}},
        },
    },
    {
        'name': 'imu',
        'description': "The IMU (BNO08X): accelerometer, gyro, magnetometer readings, not the ADC channels. op='read' values, 'id' the part, 'feature' enable.",
        'description_terse': "The IMU's readings - accelerometer, gyro, magnetometer. Not the ADC channels. op='read' values, 'id' the part, 'feature' enable.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'type': 'string', 'enum': ['id', 'read', 'feature']},
                'report_id': {'type': 'integer'},
                'interval_us': {'type': 'integer'},
            },
        },
    },
    {
        'name': 'angle',
        'description': "The angle sensor (A1335) on SPI4: shaft angle in degrees, its die temperature, the field it sees. Not the IMU, not an ADC channel.",
        'description_terse': "The angle sensor (A1335): shaft angle, die temperature, field strength. Not the IMU.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'type': 'string', 'enum': ['read', 'registers']},
            },
        },
    },
    {
        'name': 'orientation',
        'description': "How the board is turned or oriented, drawn as a picture from the IMU's rotation vector. op='show' opens a terminal that redraws it live.",
        'description_terse': "How the board is turned or oriented, as a picture. op='show' opens a terminal that redraws it live.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'type': 'string', 'enum': ['once', 'show']},
            },
        },
    },
    {
        'name': 'afe_power',
        'description': 'Analog front end switch. It also powers the ADC reference, so readings are meaningless with it off.',
        'description_terse': 'Front end switch. It also powers the ADC reference.',
        'inputSchema': {
            'type': 'object',
            'properties': {'action': {'enum': ['on', 'off', 'toggle', 'read']}},
            'required': ['action'],
        },
    },
    {
        'name': 'devices',
        'description': "Nodes on every bus, and what each says it is. op=buses lists the segments; op=use name='right knee' picks one.",
        'description_terse': "Nodes on every bus. op=buses for segments; op=use + name='right knee' picks one.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'enum': ['list', 'buses', 'use']},
                'unit': {'type': 'integer'},
                'name': {'type': 'string',
                         'description': 'what a node calls itself'},
                'bus': {'type': 'string', 'description': 'segment, e.g. RL'},
            },
        },
    },
    {
        'name': 'digital_read',
        'description': 'Read the digital channels: the value, 0 or 1, of every one, now. Values, not the list - board_info lists them.',
        'description_terse': 'Values of every digital channel, 0 or 1, now. Not the list.',
        'inputSchema': {'type': 'object', 'properties': {}},
    },
    {
        'name': 'gpio_pin',
        'description': 'Read, drive or configure one pin. Writing and configuring need test_gate open.',
        'description_terse': 'Read, drive or configure one pin. Write and mode need test_gate.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'enum': ['read', 'write', 'mode']},
                'pin': _PIN,
                'level': {'type': 'boolean'},
                'mode': {'enum': ['input', 'output', 'output_od', 'analog']},
                'pull': {'enum': ['none', 'up', 'down']},
            },
            'required': ['op', 'pin'],
        },
    },
    {
        'name': 'gpio_port',
        'description': 'Read a whole 16-bit port, or drive a masked set of its pins atomically. Writing needs test_gate open.',
        'description_terse': 'Read a 16-bit port, or drive masked pins. Write needs test_gate.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'enum': ['read', 'write']},
                'port': _PORT,
                'mask': {'type': 'integer'},
                'value': {'type': 'integer'},
            },
            'required': ['op', 'port'],
        },
    },
    {
        'name': 'test_gate',
        'description': 'Open or close raw pin access. Reads work without it; writes do not.',
        'description_terse': 'Open or close raw pin access. Reads work without it.',
        'inputSchema': {
            'type': 'object',
            'properties': {'enable': {'type': 'boolean'}},
            'required': ['enable'],
        },
    },
    {
        'name': 'thermal',
        'description': "Thermal observer: op=state NTC and each node's estimated degC, op=budget SOA spend and clamp, op=ident identification and margin.",
        'description_terse': "Thermal observer: op=state NTC and every node's estimate, op=budget SOA spend and clamp, op=ident the identification and the margin.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'enum': ['state', 'budget', 'ident']},
            },
        },
    },
    {
        'name': 'link',
        'description': 'echo tests the wire, stats returns frame counters, release hands the UART back to the text console.',
        'description_terse': 'echo tests the wire, stats returns counters, release hands back the console.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'enum': ['echo', 'stats', 'release']},
                'text': {'type': 'string'},
            },
            'required': ['op'],
        },
    },
]


# ---- handlers --------------------------------------------------------------

SCHEMAS = {spec['name']: spec['inputSchema'].get('properties', {})
           for spec in TOOLS}

TRUE = ('1', 'true', 'yes', 'on')


def coerce(name, arguments):
    """The arguments as the schema declares them, whatever the model sent."""
    properties = SCHEMAS.get(name) or {}
    coerced = {}
    for key, value in (arguments or {}).items():
        kind = (properties.get(key) or {}).get('type')
        if value is None or kind is None:
            coerced[key] = value
            continue
        try:
            coerced[key] = _as(kind, value)
        except (TypeError, ValueError):
            raise ValueError('%s: %s should be %s, got %r'
                             % (name, key, kind, value))
    return coerced


def _boolean(value):
    """A flag, from a bool or from the words a model spells one with."""
    if isinstance(value, str):
        return value.strip().lower() in TRUE
    return bool(value)


def _coercer(kind):
    """How a declared type takes a value; anything else passes as it came."""
    return {'array': _names, 'boolean': _boolean,
            'integer': lambda value: int(float(value)),
            'number': float}.get(kind, lambda same: same)


def _as(kind, value):
    """One value as one declared type. Raises for anything that will not go."""
    if kind not in ('array', 'boolean') and isinstance(value, bool):
        # A bool is an int in python, and a model that sent one where a number
        # belongs has made a mistake worth reporting.
        raise ValueError('not a number')
    return _coercer(kind)(value)


def _names(wanted):
    """Whatever a model spelled a list as, as a list of names."""
    if isinstance(wanted, str):
        wanted = wanted.strip().strip('[]').split(',')
    quotes = '"' + "'"
    return [str(item).strip().strip(quotes) for item in wanted if str(item).strip()]


def _key(text):
    """A channel name with an author's punctuation removed."""
    return re.sub(r'[^a-z0-9]', '', str(text).strip().lower())


# The same three phases under the other convention: U/V/W and A/B/C both appear
# in the same datasheets, so `phase_a` is a spelling, not a mistake.
PHASE_ALIASES = {
    'phasea': 'phaseu', 'phaseb': 'phasev', 'phasec': 'phasew',
    'a': 'phaseu', 'b': 'phasev', 'c': 'phasew',
    'u': 'phaseu', 'v': 'phasev', 'w': 'phasew',
}

# A channel by what it measures rather than what it is called.
SIGNAL_ALIASES = {
    'bus': 'dcbus', 'vbus': 'dcbus', 'dc': 'dcbus', 'dclink': 'dcbus',
    'link': 'dcbus', 'busvoltage': 'dcbus', 'voltage': 'dcbus',
    'temp': 'ntc', 'temperature': 'ntc', 'thermistor': 'ntc',
    'thermal': 'ntc', 'degc': 'ntc',
}


def _alias(key, by_name):
    """The name this board knows, for a name somebody else's board uses."""
    target = PHASE_ALIASES.get(key) or SIGNAL_ALIASES.get(key)
    if target and target in by_name:
        return target
    # Not a name this board knows and not an alias either, but it may still
    # single one out - `bus` is inside `dcbus` and inside nothing else.
    found = _matches(key, by_name)
    if len(found) == 1:
        return found[0]
    return key


def _words(text):
    """A name split into the words it was built from."""
    spaced = re.sub(r'(?<=[a-z0-9])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])',
                    ' ', str(text))
    return [w for w in re.split(r'[^A-Za-z0-9]+', spaced.lower()) if w]


def _matches(key, by_name):
    """Every channel this key could mean, by prefix or containment."""
    if not key:
        return []
    return [name for name in sorted(by_name)
            if name.startswith(key) or key.startswith(name) or key in name]


def _index_of(text, by_name, notes):
    """One requested channel as an index: a number, a name, an alias, a
    spelling of one, or the words it was made of - with what it took to
    get there on `notes`.
    """
    key = _alias(_key(text), by_name)
    if text.isdigit():
        return int(text)
    if key in by_name:
        _note(notes, text, key)
        return by_name[key]
    found = _matches(key, by_name)
    if not found:
        # Not a name, a spelling of one or a substring of one - so try the
        # words it is made of.
        words = _words(text)
        phased = any(w.startswith('phase') for w in words)
        found = sorted({_alias(w, by_name) for w in words
                        if len(w) > 1 or phased} & set(by_name))
    if len(found) > 1:
        raise ValueError('channel %r could be %s - say which'
                         % (text, ' or '.join(found)))
    if not found:
        raise ValueError('unknown channel %r; names are %s'
                         % (text, ','.join(sorted(by_name))))
    _note(notes, text, found[0])
    return by_name[found[0]]


def _note(notes, text, key):
    """Say how a spelling was read, when it was read as something else."""
    if notes is not None and key != _key(text):
        notes.append('%s read as %s' % (text, key))


def _resolve(session, wanted, notes=None):
    """Turn ['4', 'DC bus'] into channel indices."""
    wanted = _names(wanted)
    _, _, channels = session.info()

    # The schema says "omit for all", so a model that wants everything writes
    # ch=['all'] instead - which is the same request in the words the schema
    # used.
    if len(wanted) == 1 and _key(wanted[0]) in ('all', 'every', 'everything'):
        return list(range(len(channels)))
    by_name = {_key(render.short(c['signal'], c['index'])): c['index']
               for c in channels}
    # `ch3` is what an unnamed channel is called, and it stayed addressable by
    # that after PB1 and PC1 were given real names - a caller counting channels
    # should not stop working because somebody named one.
    for channel in channels:
        by_name.setdefault('ch%d' % channel['index'], channel['index'])

    indices = []
    for item in wanted:
        text = str(item).strip()
        index = _index_of(text, by_name, notes)
        if not 0 <= index < len(channels):
            raise ValueError('channel %d out of range 0..%d'
                             % (index, len(channels) - 1))
        indices.append(index)
    return indices


def _split_pin(text):
    text = str(text).strip().upper()
    if len(text) < 2 or not text[0].isalpha() or not text[1:].isdigit():
        raise ValueError('pin %r should look like B2 or E15' % (text,))
    return text[0], int(text[1:])


def angle(session, op='read', **_):
    """The A1335's reading, off the board's poll loop."""
    part = session.board.angle

    if op == 'registers':
        with part.configuring():
            rows = [(name, part.read(reg)['value'])
                    for reg, name in ((0x20, 'ANG'), (0x22, 'STA'),
                                      (0x24, 'ERR'), (0x28, 'TSEN'),
                                      (0x2A, 'FIELD'))]
        return render.angle_registers(rows)

    return render.angle(part.state())


def thermal(session, op='state', **_):
    """The thermal observer behind device 8, as three questions."""
    part = session.board.thermal
    if op == 'budget':
        return render.thermal_budget(part.budget())
    if op == 'ident':
        return render.thermal_ident(part.identification())
    return render.thermal_state(part.state())


def board_info(session, refresh=False, kind='all', **_):
    if kind not in BOARD_INFO_KINDS:
        return 'ERR unknown kind %r; try %s' % (kind,
                                                ', '.join(BOARD_INFO_KINDS))
    version, clock, channels = session.info(refresh=refresh)
    # The pins come from the board too (command 0x6D).
    if kind == 'parts':
        # What is fitted, which only the firmware knows: one entry per part,
        # reported by channels kind 4, with what powers each one.
        got = session.board.system.channel_map(refresh=refresh)
        return render.parts(got.get('parts') or [])

    if kind == 'subsystems':
        # What the board is made of, which only the firmware knows: one entry
        # per command table, reported by channels kind 3.
        got = session.board.system.channel_map(refresh=refresh)
        return render.subsystems(got.get('subsystems') or [])

    section = 'reserved' if kind == 'reserved' else 'digital'
    try:
        pins = session.board.system.channel_map(refresh=refresh)[section]
    except OLDER_FIRMWARE:
        pins = None
    return render.board_info(version, clock, channels, pins, kind)


def analog_read(session, ch=None, samples=64, rate_hz=2000.0,
                ntc_beta=None, ntc_r25=None, vref=3.3, **_):
    board = session.board

    # Resolve the arguments BEFORE checking the board's state.
    _, _, channels = session.info()
    notes = []
    indices = (_resolve(session, ch, notes) if ch
               else [c['index'] for c in channels])

    # Read either way, and say which it was.
    afe_on = bool(board.afe.state().get('on'))

    mask = 0
    for index in indices:
        mask |= 1 << index

    burst = board.analog.burst(mask, samples, rate_hz)

    # Derive only the two quantities whose conversion is known.
    own = board.analog.scaling()
    ntc = NtcParams(r25=ntc_r25 or own['ntc'].r25,
                    beta=ntc_beta or own['ntc'].beta,
                    r_fixed=own['ntc'].r_fixed,
                    t25_kelvin=own['ntc'].t25_kelvin)
    divider = DividerParams(r_top=own['dcbus'].r_top,
                            r_bottom=own['dcbus'].r_bottom, vref=vref)

    rows = []
    derived = {}
    for index in sorted(burst['channels']):
        meta = channels[index]
        stats = burst['channels'][index]
        convert = (scaling.differential_volts if meta['differential']
                   else scaling.single_ended_volts)
        row = dict(meta)
        row.update(stats)
        row['volts_at_pin'] = convert(stats['mean_raw'], vref)
        rows.append(row)

        if meta['signal'] == 'NTC':
            try:
                derived[index] = '%.2fC' % ntc.celsius(stats['mean_raw'])
            except ValueError as exc:
                # A rail reading (a genuinely open or shorted thermistor, not
                # the AFE-off case - that lands at mid-scale, not a rail) makes
                # the conversion undefined.
                derived[index] = 'no conversion: %s' % exc
        elif meta['signal'] == 'DC bus':
            derived[index] = '%.3fV bus' % divider.volts(stats['mean_raw'])

    banner = ('' if afe_on else
              'AFE OFF - the ADC reference is unpowered. These are the codes '
              'the converter returned, not measurements: every channel sits '
              'near mid-scale, and the degC and volts below are arithmetic '
              'on that - not a temperature, not a bus voltage. Call '
              'afe_power on to measure.' + '\n')
    # What had to be corrected to answer at all, above the answer.
    if notes:
        banner = ('read as asked, with corrections: %s%s'
                  % ('; '.join(notes), '\n')) + banner
    return banner + render.analog({'samples': burst['samples'],
                          'rate_hz': burst['rate_hz'],
                          'channels': rows}, derived)


def self_test(session, failures_only=False, **_):
    checks = session.board.system.self_test()
    failed = [c for c in checks if c['status'] == 'fail']
    if failures_only and not failed:
        return 'all pass'
    return render.checks(failed if failures_only else checks)


def imu(session, op='read', report_id=None, interval_us=None, **_):
    """The IMU. Reads by default: it is the question that gets asked."""
    part = session.board.imu

    if op == 'id':
        with part.configuring():
            return render.imu('id', part.product_id())

    if op == 'feature':
        return _imu_feature(part, report_id, interval_us)

    return render.imu('state', part.state())


def _imu_feature(part, report_id, interval_us):
    """Enable one report at an interval, or disable it with none."""
    if report_id is None:
        raise ValueError("op='feature' needs report_id - 1 accelerometer, "
                         "2 gyroscope, 3 magnetic field, 5 rotation vector")
    with part.configuring():
        # The reset is not optional: measured, a Set Feature onto a part that
        # was already running took no effect and the loop absorbed nothing
        # afterwards.
        part.reset()
        part.feature(int(report_id), int(interval_us or 0))
    return 'imu: report 0x%02X %s' % (
        int(report_id),
        'every %d us' % int(interval_us) if interval_us else 'disabled')


#: How often the rotation vector is asked for when `orientation` has to
#: enable it, and how many looks at the record it gets to arrive in.
ORIENTATION_INTERVAL_US = 20000
ORIENTATION_LOOKS = 20


def _rotation_vector(part):
    """The loop's newest record, with a rotation vector in it when the part is
    reporting one.
    """
    got = part.state()
    if got['quaternion'] is not None:
        return got
    with part.configuring():
        part.reset()
        part.feature(ROTATION_VECTOR, ORIENTATION_INTERVAL_US)
    for _ in range(ORIENTATION_LOOKS):
        got = part.state()
        if got['quaternion'] is not None:
            return got
    return got


def orientation(session, op='once', **_):
    """One picture, or a window that keeps drawing them."""
    if op == 'show':
        return _open_orientation_window(session)

    got = _rotation_vector(session.board.imu)
    if got['quaternion'] is None:
        raise DeviceStateError(
            'the IMU sent no rotation vector. It is enabled now, so a second '
            'call may find one; if not, the part is not reporting. AFE_ON '
            'powers it - with that off it answers reads and acts on no write.')

    q = got['quaternion']
    return orient.picture((q['i'], q['j'], q['k'], q['real']))


def _open_orientation_window(session):
    """Start the live view in a console of its own, and say so."""
    tools_dir = os.path.dirname(os.path.abspath(__file__))
    script = os.path.join(os.path.dirname(tools_dir), 'tools',
                          'show_orientation.py')
    port = session.port or 'COM4'

    if os.name != 'nt':
        return ('a live view needs its own console and this is not Windows - '
                'run: python tools/show_orientation.py --port %s' % port)

    subprocess.Popen(
        [sys.executable, script, '--port', str(port)],
        creationflags=subprocess.CREATE_NEW_CONSOLE,
        cwd=os.path.dirname(tools_dir))

    return ('orientation: a window is drawing the board live from %s. '
            'Ctrl+C in it to stop; it puts the IMU back as it found it.'
            % port)


def _multicast(session):
    return session.unit == protocol.BROADCAST


def afe_power(session, action='read', **_):
    if _multicast(session):
        return _afe_order(session, action)
    afe = session.board.afe
    if action != 'read':
        _afe_switch(session, afe, action)
    return render.kv(afe.state())


def _afe_order(session, action):
    """An order to every node, not a request."""
    if action not in ('on', 'off'):
        return ('ERR %s needs a reply and a broadcast has none; '
                'select one node, or use on/off' % action)
    session.board.broadcast(
        protocol.AFE, pack(('u8', protocol.AFE_ACTIONS[action])))
    return ('afe %s sent to every node - broadcast, so no read-back '
            'and no confirmation' % action)


def _afe_switch(session, afe, action):
    """Switch the rail, and wait out the reference when this turned it on."""
    was = afe.state()['on']
    {'on': afe.enable, 'off': afe.disable, 'toggle': afe.toggle}[action]()
    if afe.is_on() and not was:
        _settle(session)


def _settle(session):
    """Wait out the reference after AFE_ON went high."""

    try:
        wait = session.board.thermal.state()['sample_settle_s']
    except OLDER_FIRMWARE:
        wait = 0.5
    time.sleep(min(2.0, max(0.0, wait)))


def gpio_pin(session, op='read', pin='B2', level=False, mode='input',
             pull='none', **_):
    port, number = _split_pin(pin)
    gpio = session.board.gpio

    if op == 'read':
        return '%s=%d' % (pin.upper(), gpio.pin_read(port, number))
    if op == 'write':
        return '%s=%d readback' % (pin.upper(),
                                   gpio.pin_write(port, number, level))
    gpio.pin_mode(port, number, mode, pull)
    return '%s mode=%s pull=%s' % (pin.upper(), mode, pull)


def _interface(session):
    """How the host reaches this bus, for the list's own header."""
    if session.simulated or session.port is None:
        return 'Simulated'
    port = session.port
    try:
        kind = ports.kind_of(port)
    except OSError:                                    # the port listing
        return str(port)
    return '%s at %s' % ('debug probe' if kind == ports.PROBE
                         else 'RS485', port)


def _buses(session):
    """[(label, serves)] for this session, however it is wired."""
    return list(session.buses())


def _sweep(session, first, last, bus=None):
    """[(bus, unit, version)] across one segment or all of them."""
    units = range(int(first), int(last) + 1)
    labels = [bus] if bus else [label for label, _ in _buses(session)]
    found = []
    for label in labels:
        for unit, version in session.scan(units, bus=label):
            found.append((label, unit, version))
    return found


def devices(session, op='list', unit=None, name=None, bus=None,
            first=1, last=16, **_):
    """The other units on this bus, and which one the tools talk to."""
    here = (session.bus, session.unit)
    if op == 'buses':
        counts = [(label, serves,
                   len(_sweep(session, first, last, label)))
                  for label, serves in _buses(session)]
        return render.buses(counts, here[0])
    if op == 'list':
        return render.devices(_sweep(session, first, last, bus), here,
                              _interface(session))
    if op == 'use':
        return _use(session, here, unit, name, bus, first, last)
    return 'ERR unknown op %r; list or use' % (op,)


def _named(found, name):
    """The nodes `name` picks out by what they call themselves, across every
    segment: "the right knee" is one node on one bus, and the operator
    should not have to know which.
    """
    key = _key(name)
    return [(b, u) for b, u, v in found
            if key in _key(v.get('where', ''))
            or key in _key(v.get('description', ''))]


def _use(session, here, unit, name, bus, first, last):
    """Point the session at one node, by unit or by name - or at every node at
    once, which Modbus spells 0.
    """
    if unit is not None and int(unit) == protocol.BROADCAST:
        # Never in the scan, and never will be: nothing answers at 0.
        session.use(protocol.BROADCAST)
        return ('multicast: every node on the bus acts, none answers. '
                'Reads are refused here; an order still goes out. '
                'devices op=use unit=N picks one node again.')
    found = _sweep(session, first, last, bus)
    if unit is None and not name:
        return ('ERR use needs unit= or name=. On the bus: %s'
                % '; '.join('%s %d %s' % (b, u, v.get('where', ''))
                            for b, u, v in found))
    hit = _named(found, name) if unit is None else []
    if unit is None and len(hit) != 1:
        return ('ERR %r matches %d nodes: %s'
                % (name, len(hit),
                   ', '.join('%s %d' % pair for pair in hit) or 'none'))
    if unit is None:
        bus, unit = hit[0]
    if (bus or here[0], int(unit)) not in [(b, u) for b, u, _ in found]:
        # Not a refusal for its own sake: pointing the session at a unit nobody
        # is at makes every later call time out, and the operator reads that as
        # the board having died.
        return ('ERR no node at %s %s; answering: %s'
                % (bus or here[0], unit,
                   ', '.join('%s %d' % (b, u) for b, u, _ in found)
                   or 'none'))
    session.use(int(unit), bus=bus)
    return render.devices(found, (session.bus, session.unit),
                          _interface(session))


def digital_read(session, **_):
    """The level of every digital I/O channel, from the board's own map."""
    pins = session.board.system.channel_map()['digital']
    gpio = session.board.gpio
    rows = []
    for entry in pins:
        # The map spells them "PB2"; _split_pin takes "B2".
        name = entry['pin']
        port, number = _split_pin(name[1:] if name[:1] == 'P' else name)
        rows.append(dict(entry, level=int(bool(gpio.pin_read(port, number)))))
    return render.digital_levels(rows)


def gpio_port(session, op='read', port='E', mask=0, value=0, **_):
    letter = str(port).strip().upper()[:1]
    gpio = session.board.gpio
    reserved = {p for (prt, p) in protocol.RESERVED_PINS if prt == letter}

    if op == 'read':
        return render.port_bits(letter, gpio.port_read(letter), reserved)
    return render.port_bits(letter, gpio.port_write(letter, mask, value),
                            reserved)


def test_gate(session, enable=False, **_):
    return 'gate=%d' % session.board.gpio.test_mode(enable)


def link(session, op='stats', text='ping', **_):
    board = session.board
    if op == 'echo':
        return 'echo ok %r' % board.link.echo(text)
    if op == 'release':
        board.close_binary()
        session.reset()
        return 'console released'
    return render.kv(board.link.stats())


HANDLERS = {
    'board_info': board_info,
    'docs': _docs,
    'self_test': self_test,
    'analog_read': analog_read,
    'afe_power': afe_power,
    'devices': devices,
    'digital_read': digital_read,
    'imu': imu,
    'angle': angle,
    'orientation': orientation,
    'gpio_pin': gpio_pin,
    'gpio_port': gpio_port,
    'test_gate': test_gate,
    'thermal': thermal,
    'link': link,
}
