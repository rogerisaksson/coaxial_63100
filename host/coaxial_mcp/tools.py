"""Tool schemas and handlers.

EIGHT tools, not twenty. Every tool costs its name, its description and its
schema on every single turn, so the set is deliberately coarse: one per thing a
fixture actually does, with a small enum where a family of operations would
otherwise become a family of tools.

The eighth is `docs`, and it is the exception that proves the rule: it touches
no hardware. It exists because the documents in docs/ are what stop a reading
being misinterpreted - the AFE gate, the unknown phase gain, what has already
been ruled out - and until it existed the one reader who could not open them was
the model standing at the bench. Its schema is three optional strings, and it
answers with an index rather than a document, for the same token reason as
everything else here.

Descriptions are one line. Property names are short. There are no titles, no
prose defaults and no examples in the schema - a model that needs the channel
map calls board_info once and refers to channels by index or short name after
that. Paying for the map on every turn is the waste this design avoids.
"""
import re

from coaxial import DividerParams, NtcParams, protocol, scaling

from . import render
from .docs import docs as _docs

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
        'name': 'docs',
        'description': "This board's own documents. To answer a question use find=TEXT; doc= alone returns headings, which are titles and not answers.",
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

SCHEMAS = {spec['name']: spec['inputSchema'].get('properties', {})
           for spec in TOOLS}

TRUE = ('1', 'true', 'yes', 'on')


def coerce(name, arguments):
    """The arguments as the schema declares them, whatever the model sent.

    Tool calls arrive as JSON and the schema says which fields are numbers,
    which are booleans and which are arrays. A capable model sends them that
    way. A smaller one sends `samples="100"`, `ch="ntc"`, `refresh="true"` -
    and the handler then fails somewhere deep with a TypeError about dividing
    a float by a str, which tells the model nothing it can act on.

    That is not a cosmetic failure. Measured on this board with llama3.1:8b:
    the call errored, the model did not understand why, and it answered
    "25.00 C" from memory - the exact reading this board gives with its analog
    front end off, for a board that was at 37. A tool that is hard to call is a
    tool that gets guessed around.

    This is for the ollama side, where the call comes straight from the model
    and nothing between the two checks it. The MCP server does not use it: the
    protocol library validates arguments against inputSchema before a handler
    is reached, so a mistyped field is already refused there, by name and
    readably - "'32' is not of type 'integer'". Coercing after that would be
    unreachable; coercing before it would be this server accepting what the
    protocol has already called malformed.

    What cannot be converted raises by name, with the type that was wanted.
    Unknown keys pass through untouched; the handlers take **_ and ignore them.
    """
    properties = SCHEMAS.get(name) or {}
    coerced = {}
    for key, value in (arguments or {}).items():
        kind = (properties.get(key) or {}).get('type')
        if value is None or kind is None:
            coerced[key] = value
            continue
        try:
            if kind == 'array':
                value = _names(value)
            elif kind == 'boolean':
                if isinstance(value, str):
                    value = value.strip().lower() in TRUE
                else:
                    value = bool(value)
            elif isinstance(value, bool):
                # A bool is an int in python, and a model that sent one where a
                # number belongs has made a mistake worth reporting.
                raise ValueError('not a number')
            elif kind == 'integer':
                value = int(float(value))
            elif kind == 'number':
                value = float(value)
        except (TypeError, ValueError):
            raise ValueError('%s: %s should be %s, got %r'
                             % (name, key, kind, value))
        coerced[key] = value
    return coerced


def _names(wanted):
    """Whatever a model spelled a list as, as a list of names.

    The schema says array of strings and the strong models send one. The weak
    ones send `ntc`, or the string `"['NTC']"`, and the cost of not handling
    that is not a clear error: `for item in "ntc"` iterates characters, so the
    tool reports `unknown channel 'n'` and the model - having no idea what it
    did wrong - answers from memory instead. That failure was measured here,
    and what it produced was 25.00 C: the exact number this board reports with
    its analog front end off, invented for a board that was at 37.

    So a bare string is one name, a comma separated one is several, and the
    brackets and quotes of a list that arrived as text are stripped. What
    cannot be read as a channel name still raises - loudly, by name.
    """
    if isinstance(wanted, str):
        wanted = wanted.strip().strip('[]').split(',')
    quotes = '"' + "'"
    return [str(item).strip().strip(quotes) for item in wanted if str(item).strip()]


def _key(text):
    """A channel name with the punctuation an author might put in it removed.

    board_info prints the signal as `DC bus`, the short form is `DCbus`, and a
    model writing that back has to guess which of `dc_bus`, `dc-bus`, `DC bus`
    and `dcbus` this tool wanted. Measured: gemma4:12b sent `dc_bus` and got
    `unknown channel 'dc_bus'; names are ch3,ch6,dcbus,...` - a refusal listing
    a name one underscore away from the one it used, which is a tool being
    fussy rather than a model being wrong. All four collapse to the same key.
    """
    return re.sub(r'[^a-z0-9]', '', str(text).strip().lower())


# The same three phases, under the other convention. A drive is labelled U/V/W
# or A/B/C depending on which tradition the author grew up in, and both appear
# in the same datasheets; `phase_a` for `phaseu` is not a mistake, it is the
# other spelling. Measured: a model asked for ch=['ntc','dc_bus','phase_a',
# 'phase_b','phase_c'] and lost all five readings to the one it spelled the
# other way.
#
# The single letters are in here because this repository already uses them -
# the CSVs that tools/analyze_phase_log.py reads have columns U, V and W - and
# because no channel on this board is one letter, so there is nothing for them
# to collide with.
PHASE_ALIASES = {
    'phasea': 'phaseu', 'phaseb': 'phasev', 'phasec': 'phasew',
    'a': 'phaseu', 'b': 'phasev', 'c': 'phasew',
    'u': 'phaseu', 'v': 'phasev', 'w': 'phasew',
}


def _alias(key, by_name):
    """The name this board knows, for a name somebody else's board uses.

    Only ever maps onto a channel that exists: a board without a Phase U has
    no business turning `a` into one.
    """
    target = PHASE_ALIASES.get(key)
    if target and target in by_name:
        return target
    return key


def _closest(key, by_name):
    """The name a typo was probably reaching for, or None.

    Prefix and containment only - no edit distance. The failures worth catching
    here are `dcbusvoltage` and `phas`, not `ntx`, and a suggestion that is
    wrong is worse than none: it sends the next call somewhere confident.
    """
    for name in sorted(by_name):
        if name.startswith(key) or key.startswith(name):
            return name
    return None


def _resolve(session, wanted):
    """Turn ['4', 'DC bus'] into channel indices.

    Indices and names are both accepted because a model that has seen
    board_info knows the names, and one that has not can still count.
    """
    wanted = _names(wanted)
    _, _, channels = session.info()

    # The schema says "omit for all", so a model that wants everything writes
    # ch=['all'] instead - which is the same request in the words the schema
    # used. Seen from the prompt, costing a turn.
    if len(wanted) == 1 and _key(wanted[0]) in ('all', 'every', 'everything'):
        return list(range(len(channels)))
    by_name = {_key(render.short(c['signal'], c['index'])): c['index']
               for c in channels}

    indices = []
    for item in wanted:
        text = str(item).strip()
        key = _alias(_key(text), by_name)
        if text.isdigit():
            index = int(text)
        elif key in by_name:
            index = by_name[key]
        else:
            near = _closest(key, by_name)
            raise ValueError('unknown channel %r%s; names are %s'
                             % (text,
                                ' - did you mean %r?' % near if near else '',
                                ','.join(sorted(by_name))))
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

    # Read whether or not the front end is on, and say which it was.
    #
    # Refusing was worse than it looked. Asked for the raw codes with the AFE
    # deliberately off, the tool raised, and the model - having no numbers -
    # produced "PhaseU: Mid-scale ... NTC: 25.00 C" from the warning text
    # itself. A refusal did not prevent a fabricated reading, it caused one.
    #
    # It is also the wrong shape for this repository. The board is a dumb slave
    # that reports codes and judges nothing (invariant 10); deciding a
    # measurement is not worth taking is a judgement. So the codes come back,
    # with a line that cannot be mistaken for one of them.
    afe_on = bool(board.afe.state().get('on'))

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

    banner = ('' if afe_on else
              'AFE OFF - the ADC reference is unpowered. These are the codes '
              'the converter returned, not measurements: every channel sits '
              'near mid-scale and the NTC figure below is arithmetic on that, '
              'not a temperature. Call afe_power on to measure.' + chr(10))
    return banner + render.analog({'samples': burst['samples'],
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
    'docs': _docs,
    'self_test': self_test,
    'analog_read': analog_read,
    'afe_power': afe_power,
    'gpio_pin': gpio_pin,
    'gpio_port': gpio_port,
    'test_gate': test_gate,
    'link': link,
}
