"""Tool schemas and handlers.

Eight tools, not twenty. Every one costs its name, description and schema on
every turn, so the set is coarse: one per thing a fixture does, with a small
enum where a family of operations would otherwise be a family of tools.

The eighth, `docs`, touches no hardware. The documents in docs/ are what stop a
reading being misinterpreted - the AFE gate, the unknown phase gain, what is
already ruled out - and the model at the bench was the one reader who could not
open them. Three optional strings, and it answers with an index.

Descriptions are one line, property names short, no titles, prose defaults or
examples: a model that needs the channel map calls board_info once.
"""
import os
import re
import sys

from coaxial import DividerParams, NtcParams, protocol, scaling
from coaxial.wire import pack

from. import render
from .docs import docs as _docs

_PIN = {'type': 'string', 'description': 'Pin as PORT+NUMBER, e.g. B2 or E15'}
_PORT = {'type': 'string', 'description': 'Port letter A-K'}

# Which section of the map board_info answers with. A question about the
# analog channels should not cost the identity line, the clock line and the
# digital pins as well - measured, "ge mig en lista over alla analoga
# kanaler" traced eleven lines to answer with seven.
BOARD_INFO_KINDS = ('all', 'analog', 'digital', 'reserved', 'identity')

TOOLS = [
    {
        'name': 'board_info',
        'description': "Identity, clock, and the channels the board reports with their directions. kind narrows it to one section.",
        'description_terse': "Identity, clock, channels with directions. kind: analog|digital|reserved|identity|all.",
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
        'description': 'Sample ADC channels; returns mean raw, pin volts, plus degC for the NTC and volts for the DC bus. Needs the AFE on.',
        'description_terse': 'Sample ADC channels: mean raw, pin volts, NTC degC, bus volts.',
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
    """The arguments as the schema declares them, whatever the model sent.

    A smaller model sends `samples="100"`, `ch="ntc"`, `refresh="true"`, and
    the handler then fails deep with a TypeError the model cannot act on.
    Measured with llama3.1:8b: the call errored, and it answered "25.00 C"
    from memory - this board's AFE-off reading, for a board at 37.

    For the ollama side only: the MCP server gets the same protection from the
    protocol library, which validates against inputSchema before a handler is
    reached. What cannot be converted raises by name and wanted type; unknown
    keys pass through for the handlers' **_.
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

    Weak models send `ntc` or the string `"['NTC']"`, and `for item in "ntc"`
    iterates characters - so the tool said `unknown channel 'n'` and the model
    answered 25.00 C from memory, for a board at 37.

    A bare string is one name, a comma separated one several, and the brackets
    and quotes of a list that arrived as text are stripped. Anything else
    still raises, by name.
    """
    if isinstance(wanted, str):
        wanted = wanted.strip().strip('[]').split(',')
    quotes = '"' + "'"
    return [str(item).strip().strip(quotes) for item in wanted if str(item).strip()]


def _key(text):
    """A channel name with an author's punctuation removed.

    board_info prints `DC bus`, the short form is `DCbus`, and a model has to
    guess between `dc_bus`, `dc-bus`, `DC bus` and `dcbus`. Measured:
    `dc_bus` was refused with `dcbus` listed in the refusal, one underscore
    away. All four collapse to the same key.
    """
    return re.sub(r'[^a-z0-9]', '', str(text).strip().lower())


# The same three phases under the other convention: U/V/W and A/B/C both
# appear in the same datasheets, so `phase_a` is a spelling, not a mistake.
# Measured: ch=['ntc','dc_bus','phase_a','phase_b','phase_c'] lost all five
# readings to the one spelled the other way. The single letters are here
# because the CSVs in tools/analyze_phase_log.py use them and no channel on
# this board is one letter.
PHASE_ALIASES = {
    'phasea': 'phaseu', 'phaseb': 'phasev', 'phasec': 'phasew',
    'a': 'phaseu', 'b': 'phasev', 'c': 'phasew',
    'u': 'phaseu', 'v': 'phasev', 'w': 'phasew',
}

# A channel by what it measures rather than what it is called. "Read the bus",
# "what is the temperature" - neither is a spelling any punctuation-stripping
# reaches. Measured: ch=['bus'] was refused with dcbus listed in the refusal.
SIGNAL_ALIASES = {
    'bus': 'dcbus', 'vbus': 'dcbus', 'dc': 'dcbus', 'dclink': 'dcbus',
    'link': 'dcbus', 'busvoltage': 'dcbus', 'voltage': 'dcbus',
    'temp': 'ntc', 'temperature': 'ntc', 'thermistor': 'ntc',
    'thermal': 'ntc', 'degc': 'ntc',
}


def _alias(key, by_name):
    """The name this board knows, for a name somebody else's board uses.

    Only ever maps onto a channel that exists: a board without a Phase U has
    no business turning `a` into one.
    """
    target = PHASE_ALIASES.get(key) or SIGNAL_ALIASES.get(key)
    if target and target in by_name:
        return target
    # Not a name this board knows and not an alias either, but it may still
    # single one out - `bus` is inside `dcbus` and inside nothing else. One
    # match resolves; several is genuinely ambiguous and falls through to the
    # refusal, which lists them.
    found = _matches(key, by_name)
    if len(found) == 1:
        return found[0]
    return key


def _words(text):
    """A name split into the words it was built from.

    `BUS_VOLT`, `bus_voltage`, `PhaseAVolt`: a model naming a channel from
    what it measures writes several words, and none of them is the channel
    name. Separators and camelCase both split; the pieces are lowercased and
    the empty ones dropped.
    """
    spaced = re.sub(r'(?<=[a-z0-9])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])',
                    ' ', str(text))
    return [w for w in re.split(r'[^A-Za-z0-9]+', spaced.lower()) if w]


def _matches(key, by_name):
    """Every channel this key could mean, by prefix or containment.

    Containment because of what a model writes: `bus` for `dcbus` was refused
    outright at the prompt. No edit distance - the failures worth catching are
    `dcbusvoltage`, `phas` and `bus`, not `ntx`, and a wrong guess sends the
    next call somewhere confident.
    """
    if not key:
        return []
    return [name for name in sorted(by_name)
            if name.startswith(key) or key.startswith(name) or key in name]


def _closest(key, by_name):
    """The one name a key could mean, or None when it could mean several."""
    found = _matches(key, by_name)
    return found[0] if len(found) == 1 else None


def _resolve(session, wanted, notes=None):
    """Turn ['4', 'DC bus'] into channel indices.

    Indices and names are both accepted because a model that has seen
    board_info knows the names, and one that has not can still count.

    `notes` collects what had to be corrected to get there - a misspelling,
    an alias, a name assembled out of the words it was made of. The
    correction happens either way; what the list adds is the operator being
    told it happened, rather than a question about `temperatur` quietly
    coming back as a reading of something they did not name.
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
    # `ch3` is what an unnamed channel is called, and it stayed addressable
    # by that after PB1 and PC1 were given real names - a caller counting
    # channels should not stop working because somebody named one. The real
    # name wins where the two would collide, which they cannot on this board.
    for channel in channels:
        by_name.setdefault('ch%d' % channel['index'], channel['index'])

    indices = []
    for item in wanted:
        text = str(item).strip()
        key = _alias(_key(text), by_name)
        if text.isdigit():
            index = int(text)
        elif key in by_name:
            index = by_name[key]
            if notes is not None and key != _key(text):
                notes.append('%s read as %s' % (text, key))
        else:
            # Several matches is not a typo, it is a question that has not
            # been narrowed - `phas` on a three-phase board. Naming them
            # beats "unknown", which reads as "no such thing" and sends the
            # next call somewhere else entirely.
            found = _matches(key, by_name)
            if not found:
                # Not a name, a spelling of one or a substring of one - so
                # try the words it is made of. `BUS_VOLT` is not any channel,
                # but `bus` is exactly one. Measured from the prompt, twice:
                # `A0` and `BUS_VOLT`, both invented, both refused.
                words = _words(text)
                # A single letter is a phase name (`a`, `u`) only next to the
                # word that says so. Without that rule `not_a_channel`
                # resolved to PhaseU through its `a`, which is the invented
                # reading this whole file exists to prevent.
                phased = any(w.startswith('phase') for w in words)
                found = sorted(
                    {_alias(w, by_name) for w in words
                     if len(w) > 1 or phased} & set(by_name))
            if len(found) == 1:
                index = by_name[found[0]]
                if notes is not None:
                    notes.append('%s read as %s' % (text, found[0]))
            elif len(found) > 1:
                raise ValueError('channel %r could be %s - say which'
                                 % (text, ' or '.join(found)))
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


def board_info(session, refresh=False, kind='all', **_):
    if kind not in BOARD_INFO_KINDS:
        return 'ERR unknown kind %r; try %s' % (kind,
                                                ', '.join(BOARD_INFO_KINDS))
    version, clock, channels = session.info(refresh=refresh)
    # The pins come from the board too (command 0x6D). An older firmware has
    # no such command and the analog table alone is the answer, which is why
    # this is a try and not a required call.
    section = 'reserved' if kind == 'reserved' else 'digital'
    try:
        pins = session.board.system.channel_map(refresh=refresh)[section]
    except Exception:                                         # noqa: BLE001
        pins = None
    return render.board_info(version, clock, channels, pins, kind)


def analog_read(session, ch=None, samples=64, rate_hz=2000.0,
                ntc_beta=None, ntc_r25=None, vref=3.3, **_):
    board = session.board

    # Resolve the arguments BEFORE checking the board's state. A bad channel
    # name is the caller's mistake and deserves a specific answer; reporting
    # "the AFE is off" for it would send them after the wrong problem.
    _, _, channels = session.info()
    notes = []
    indices = (_resolve(session, ch, notes) if ch
               else [c['index'] for c in channels])

    # Read whether or not the front end is on, and say which it was.
    #
    # Refusing was worse than it looked. Asked for the raw codes with the AFE
    # deliberately off, the tool raised, and the model - having no numbers -
    # produced "PhaseU: Mid-scale... NTC: 25.00 C" from the warning text
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
            try:
                derived[index] = '%.2fC' % ntc.celsius(stats['mean_raw'])
            except ValueError as exc:
                # A rail reading (a genuinely open or shorted thermistor, not
                # the AFE-off case - that lands at mid-scale, not a rail) makes
                # the conversion undefined. One bad derived value must not cost
                # every other channel's raw code in the same burst: invariant 9
                # says analog_read returns the codes either way.
                derived[index] = 'no conversion: %s' % exc
        elif meta['signal'] == 'DC bus':
            derived[index] = '%.3fV bus' % divider.volts(stats['mean_raw'])

    banner = ('' if afe_on else
              'AFE OFF - the ADC reference is unpowered. These are the codes '
              'the converter returned, not measurements: every channel sits '
              'near mid-scale, and the degC and volts below are arithmetic '
              'on that - not a temperature, not a bus voltage. Call '
              'afe_power on to measure.' + '\n')
    # What had to be corrected to answer at all, above the answer. The
    # reading was taken either way - refusing a misspelling is worse than
    # taking it and saying so - but a question about `temperatur` coming
    # back as a reading of something the operator did not name is exactly
    # the quiet substitution this file exists to prevent.
    if notes:
        banner = ('read as asked, with corrections: %s%s'
                  % ('; '.join(notes), '\n')) + banner
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


def _multicast(session):
    return getattr(session, 'unit', None) == protocol.BROADCAST


def afe_power(session, action='read', **_):
    if _multicast(session):
        # An order, not a request. `read` and `toggle` both need the reply
        # a broadcast does not have - toggle because "the other one" is
        # only defined against a state somebody read.
        if action not in ('on', 'off'):
            return ('ERR %s needs a reply and a broadcast has none; '
                    'select one node, or use on/off' % action)
        session.board.broadcast(
            protocol.AFE, pack(('u8', protocol.AFE_ACTIONS[action])))
        return ('afe %s sent to every node - broadcast, so no read-back '
                'and no confirmation' % action)
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


def _interface(session):
    """How the host reaches this bus, for the list's own header.

    Asked of the session rather than passed in: `devices` is called from
    the MCP server and from the ollama loop, and only one of those has an
    Origin to hand.
    """
    if getattr(session, 'simulated', False):
        return 'Simulated'
    port = getattr(session, 'port', None)
    if port is None:
        return 'Simulated'
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'tools'))
    try:
        import find_board
        kind = find_board.kind_of(port)
    except Exception:                                         # noqa: BLE001
        return str(port)
    return '%s at %s' % ('debug probe' if kind == find_board.PROBE
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
    """The other units on this bus, and which one the tools talk to.

    One board is a bus of one. Several - a machine built out of them - are
    the same firmware at different unit ids, told apart by the identity
    each one reports for itself. Selecting one is a session change, so
    every other tool follows it without an argument of its own.
    """
    here = (getattr(session, 'bus', getattr(session, 'port', None)),
            getattr(session, 'unit', None))
    if op == 'buses':
        counts = [(label, serves,
                   len(_sweep(session, first, last, label)))
                  for label, serves in _buses(session)]
        return render.buses(counts, here[0])
    if op == 'list':
        return render.devices(_sweep(session, first, last, bus), here,
                              _interface(session))
    if op == 'use':
        if unit is not None and int(unit) == protocol.BROADCAST:
            # Never in the scan, and never will be: nothing answers at 0.
            # Selectable all the same, because an order to every node on
            # the bus is a real thing to want and Modbus spells it 0.
            session.use(protocol.BROADCAST)
            return ('multicast: every node on the bus acts, none answers. '
                    'Reads are refused here; an order still goes out. '
                    'devices op=use unit=N picks one node again.')
        found = _sweep(session, first, last, bus)
        if unit is None and not name:
            return ('ERR use needs unit= or name=. On the bus: %s'
                    % '; '.join('%s %d %s' % (b, u, v.get('where', ''))
                                for b, u, v in found))
        if unit is None:
            # By what it calls itself, across every segment: "the right
            # knee" is one node on one bus, and the operator should not
            # have to know which. A name that is on two - "knee" - names
            # both rather than picking.
            key = _key(name)
            hit = [(b, u) for b, u, v in found
                   if key in _key(v.get('where', ''))
                   or key in _key(v.get('description', ''))]
            if len(hit) != 1:
                return ('ERR %r matches %d nodes: %s'
                        % (name, len(hit),
                           ', '.join('%s %d' % pair for pair in hit) or 'none'))
            bus, unit = hit[0]
        if (bus or here[0], int(unit)) not in [(b, u) for b, u, _ in found]:
            # Not a refusal for its own sake: pointing the session at a
            # unit nobody is at makes every later call time out, and the
            # operator reads that as the board having died.
            return ('ERR no node at %s %s; answering: %s'
                    % (bus or here[0], unit,
                       ', '.join('%s %d' % (b, u) for b, u, _ in found)
                       or 'none'))
        session.use(int(unit), bus=bus)
        return render.devices(found,
                              (getattr(session, 'bus',
                                       getattr(session, 'port', None)),
                               session.unit),
                              _interface(session))
    return 'ERR unknown op %r; list or use' % (op,)


def digital_read(session, **_):
    """The level of every digital I/O channel, from the board's own map.

    One call for the question "what are the digital values", which
    otherwise took a gpio_pin per pin, or a gpio_port and the model picking
    bits out of a register - arithmetic this library exists not to hand it.
    Reads only: no gate, nothing driven.
    """
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
    'gpio_pin': gpio_pin,
    'gpio_port': gpio_port,
    'test_gate': test_gate,
    'link': link,
}
