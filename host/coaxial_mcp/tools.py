"""Tool schemas and handlers.

SEVEN tools, not twenty. Every tool costs its name, its description and its
schema on every single turn, so the set is deliberately coarse: one per thing a
fixture actually does, with a small enum where a family of operations would
otherwise become a family of tools.

Descriptions are one line. Property names are short. There are no titles, no
prose defaults and no examples in the schema - a model that needs the channel
map calls board_info once and refers to channels by index or short name after
that. Paying for the map on every turn is the waste this design avoids.
"""
from coaxial import DividerParams, NtcParams, protocol, scaling

from . import render

_PIN = {'type': 'string', 'description': 'Pin as PORT+NUMBER, e.g. B2 or E15'}
_PORT = {'type': 'string', 'description': 'Port letter A-K'}

TOOLS = [
    {
        'name': 'board_info',
        'description': 'Identity, firmware/protocol version, clock tree and the ADC channel map. Call once; the map is stable.',
        'inputSchema': {
            'type': 'object',
            'properties': {'refresh': {'type': 'boolean'}},
        },
    },
    {
        'name': 'analog_read',
        'description': 'Sample ADC channels; returns mean raw, pin volts, plus degC for the NTC and volts for the DC bus. Needs the AFE on.',
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
        'name': 'self_test',
        'description': 'What the board can prove about itself: clocks, calibration, firmware checksum. Pass/fail only where no external reference is needed.',
        'inputSchema': {
            'type': 'object',
            'properties': {'failures_only': {'type': 'boolean'}},
        },
    },
    {
        'name': 'afe_power',
        'description': 'Analog front end switch. It also powers the ADC reference, so readings are meaningless with it off.',
        'inputSchema': {
            'type': 'object',
            'properties': {'action': {'enum': ['on', 'off', 'toggle', 'read']}},
            'required': ['action'],
        },
    },
    {
        'name': 'gpio_pin',
        'description': 'Read, drive or configure one pin. Writing and configuring need test_gate open.',
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
        'inputSchema': {
            'type': 'object',
            'properties': {'enable': {'type': 'boolean'}},
            'required': ['enable'],
        },
    },
    {
        'name': 'link',
        'description': 'echo tests the wire, stats returns frame counters, release hands the UART back to the text console.',
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

def _resolve(session, wanted):
    """Turn ['4', 'DCbus'] into channel indices.

    Both forms are accepted because a model that has seen board_info knows the
    names, and one that has not can still count.
    """
    _, _, channels = session.info()
    by_name = {render.short(c['signal'], c['index']).lower(): c['index']
               for c in channels}

    indices = []
    for item in wanted:
        text = str(item).strip()
        if text.isdigit():
            index = int(text)
        elif text.lower() in by_name:
            index = by_name[text.lower()]
        else:
            raise ValueError('unknown channel %r; names are %s'
                             % (text, ','.join(sorted(by_name))))
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


def board_info(session, refresh=False, **_):
    version, clock, channels = session.info(refresh=refresh)
    return render.board_info(version, clock, channels)


def analog_read(session, ch=None, samples=64, rate_hz=2000.0,
                ntc_beta=None, ntc_r25=None, vref=3.3, **_):
    board = session.board

    # Resolve the arguments BEFORE checking the board's state. A bad channel
    # name is the caller's mistake and deserves a specific answer; reporting
    # "the AFE is off" for it would send them after the wrong problem.
    _, _, channels = session.info()
    indices = _resolve(session, ch) if ch else [c['index'] for c in channels]

    board.afe.require()

    mask = 0
    for index in indices:
        mask |= 1 << index

    burst = board.analog.burst(mask, samples, rate_hz)

    # Derive only the two quantities whose conversion is known. The phase
    # channels sit behind unknown AFE gain, so they get pin volts and no more.
    ntc = NtcParams(r25=ntc_r25 or scaling.NTC_ONBOARD.r25,
                    beta=ntc_beta or scaling.NTC_ONBOARD.beta,
                    r_fixed=scaling.NTC_ONBOARD.r_fixed)
    divider = DividerParams(r_top=scaling.DCBUS_ONBOARD.r_top,
                            r_bottom=scaling.DCBUS_ONBOARD.r_bottom, vref=vref)

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
            derived[index] = '%.2fC' % ntc.celsius(stats['mean_raw'])
        elif meta['signal'] == 'DC bus':
            derived[index] = '%.3fV bus' % divider.volts(stats['mean_raw'])

    return render.analog({'samples': burst['samples'],
                          'rate_hz': burst['rate_hz'],
                          'channels': rows}, derived)


def self_test(session, failures_only=False, **_):
    checks = session.board.system.self_test()
    if failures_only:
        checks = [c for c in checks if c['status'] == 'fail']
        if not checks:
            return 'all pass'
    return render.checks(checks)


def afe_power(session, action='read', **_):
    afe = session.board.afe
    if action != 'read':
        {'on': afe.enable, 'off': afe.disable, 'toggle': afe.toggle}[action]()
    return render.kv(afe.state())


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
    'self_test': self_test,
    'analog_read': analog_read,
    'afe_power': afe_power,
    'gpio_pin': gpio_pin,
    'gpio_port': gpio_port,
    'test_gate': test_gate,
    'link': link,
}
