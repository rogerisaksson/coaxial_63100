"""Compact renderers. This module is where the token budget is actually spent.

A tool result is text a model has to read on every turn, so the format is chosen
for density rather than for looking like an API response:

  * fixed-width columns, one line per item, no JSON punctuation;
  * short keys and short channel names;
  * numbers rounded to what the hardware can actually resolve;
  * only what was asked for - the channel table is returned by board_info once,
    not repeated inside every reading.

For scale: a seven-channel reading is about 40 tokens here against roughly 400
for the same data as pretty-printed JSON with full key names. That difference is
the reason a small model can drive this board through a long test sequence.
"""
from coaxial import angle as a1335     # angle() below is the renderer; this is the sensor's arithmetic


def short(signal, index):
    """'Phase U' -> 'PhaseU'; an unnamed channel becomes its index."""
    return signal.replace(' ', '') if signal else 'ch%d' % index


def si(hz):
    """475000000 -> '475.0MHz'. Frequencies are read, not computed with."""
    if hz >= 1_000_000:
        return '%.1fMHz' % (hz / 1e6)
    if hz >= 1_000:
        return '%.1fkHz' % (hz / 1e3)
    return '%dHz' % hz


def analog_map(channels):
    """The analog channels, headed and counted. Its own block, because the
    digital ones used to sit under this header with no index and the columns
    out of line - measured, asked for "en lista over alla analoga kanaler",
    the screen carried both sets as one table."""
    lines = ['analog: %d channel%s' % (len(channels),
                                       '' if len(channels) == 1 else 's'),
             'ch adc pin          dir   mode name']
    for c in channels:
        lines.append('%-2d %-3d %-12s %-5s %-4s %s' % (
            c['index'], c['adc'], c['pin'], c.get('direction', 'in'),
            'diff' if c['differential'] else 'SE',
            short(c['signal'], c['index'])))
    return lines


def digital_map(pins, what='digital'):
    """The digital I/O, or the reserved pins, in their own block with their
    own header. No index and no mode column: they have neither.

    The I/O are counted as channels and the reserved list as pins, because
    that is what they are: one is something to read or set, the other is
    the bus and the debug port."""
    unit = 'pin' if what == 'reserved' else 'channel'
    lines = ['%s: %d %s%s' % (what, len(pins), unit,
                              '' if len(pins) == 1 else 's'),
             'ch   dir   name']
    for d in pins:
        lines.append('%-4s %-5s %s' % (d['pin'], d['direction'], d['signal']))
    return lines


def devices(found, here, interface='unknown'):
    # `found` is [(bus, unit, version)] and `here` is the (bus, unit) pair
    # the tools are on: with five segments, a node number alone does not
    # identify anything - node 2 is a knee on two of them.
    """Every node on the interface, and which one is selected.

    Name and type are the node's own (command 0x41), not invented here:
    "node 3" is a number, and a name and a kind are a device. The header
    names the interface rather than the port, because how the host reaches
    the bus and which node answers on it are two different facts and the
    list is about the second.

    `where` is the last column because sixteen rows of the same name and
    the same type is not a list of nodes, it is a list of numbers. A board
    does not know where it is bolted; a bus that has been commissioned
    does, and it says so in the description each node reports.
    """
    lines = ['Nodes on the communication interface (%s):' % interface,
             'bus  node name           type           where']
    for bus, unit, version in found:
        # '>' in the node column rather than a column of its own: which node
        # the tools are on is one character, and the prompt carries it too.
        lines.append(('%-4s %-4s %-14s %-14s %s'
                      % (bus,
                         ('>%d' % unit) if (bus, unit) == here else unit,
                         version.get('device', '?'),
                         version.get('type', ''),
                         version.get('where', ''))).rstrip())
    return '\n'.join(lines)


def buses(found, here):
    """The segments, and how many nodes answer on each.

    A bus is a serial segment - one limb of a machine, or its axis. Which
    limb a segment serves is the operator's knowledge: a board cannot know
    where it was bolted, and nothing here pretends it can.
    """
    lines = ['Buses on this machine:', 'bus  sel serves           nodes']
    for label, serves, count in found:
        lines.append('%-4s %-3s %-16s %d'
                     % (label, '*' if label == here else '', serves, count))
    return '\n'.join(lines)


def digital_levels(rows):
    """What each digital channel reads right now.

    A `level` column beside the map's own, because "list the channels" and
    "give me their values" are two questions and answering the first with
    the second is what board_info alone could do.
    """
    lines = ['digital: %d channel%s'
             % (len(rows), '' if len(rows) == 1 else 's'),
             'ch   dir   level name']
    for row in rows:
        lines.append('%-4s %-5s %-5d %s'
                     % (row['pin'], row['direction'], row['level'],
                        row['signal']))
    return '\n'.join(lines)


def board_info(version, clock, channels, digital=None, kind='all'):
    """Identity, clock and the map - or one section of it.

    `kind` narrows it: a question about the analog channels should not cost
    the identity line, the clock line and the digital pins as well.
    """
    lines = []
    if kind in ('all', 'identity'):
        lines += [
            '%s %s fw%s proto%d.%d build "%s"' % (
                version.get('device', '?'), version.get('mcu', '?'),
                version['firmware'], version['proto_major'],
                version['proto_minor'], version.get('build', '?')),
            'sysclk %s hclk %s src %s cmds %s' % (
                si(clock['sysclk_hz']), si(clock['hclk_hz']), clock['source'],
                version.get('commands', '?')),
        ]
        # What it IS, from the device, when the device says. A name picks a
        # codec; a description says what is on the other end of the bus,
        # which is the difference between five units and five devices.
        if version.get('description'):
            lines.append(version['description'])
    if kind in ('all', 'analog'):
        lines += analog_map(channels)
    if kind in ('all', 'digital') and digital is not None:
        lines += digital_map(digital)
    if kind == 'reserved':
        lines += digital_map(digital or (), 'reserved')
    return '\n'.join(lines)


# The row, and the header built from the same widths. Written out by hand
# they drifted: `code` and `voltage` are right-aligned, so a seven-digit
# number starts one column later than an eight-digit one and a header
# placed over the first is wrong for the second.
ANALOG_ROW = '%-2d %-7s %-4s %8.1f %+8.4fV %s'
ANALOG_HEAD = ('%-2s %-7s %-4s %8s %9s %s'
               % ('ch', 'name', 'mode', 'code', 'voltage', 'measure')).rstrip()


def analog(result, derived):
    """A reading, headed the same way the map is.

    The count and the column names are there because the two blocks sit
    on one screen when a question asks for both, and the reading used to
    arrive as a bare `64 smp @2000Hz` over unlabelled columns while the
    map above it was headed and counted.

    The last column is empty for a channel with no defined unit, which is
    most of them - `measure` is the board's own scaling where there is one,
    not something computed here. `code` is the raw converter output and
    `voltage` is at the ADC pin, which for a phase channel is not the sensed
    quantity - invariant 7.
    """
    lines = ['analog: %d channel%s'
             % (len(result['channels']),
                '' if len(result['channels']) == 1 else 's'),
             '%d samples @%.0fHz'
             % (result['samples'], result['rate_hz'] or 0),
             ANALOG_HEAD]
    for row in result['channels']:
        name = short(row['signal'], row['index'])
        extra = derived.get(row['index'], '')
        # The parentheses matter: without them.rstrip() binds to the
        # argument tuple rather than to the formatted string.
        lines.append((ANALOG_ROW % (
            row['index'], name, 'diff' if row['differential'] else 'SE',
            row['mean_raw'], row['volts_at_pin'], extra)).rstrip())
    return '\n'.join(lines)


def kv(mapping, keys=None):
    """Single line of k=v pairs. Used for small results like afe or stats."""
    items = mapping.items() if keys is None else ((k, mapping[k]) for k in keys)
    return ' '.join('%s=%s' % (k, _terse(v)) for k, v in items)


def _terse(value):
    if isinstance(value, bool):
        return '1' if value else '0'
    if isinstance(value, float):
        return '%.4g' % value
    return str(value)


def port_bits(port, value, reserved):
    """Hex plus the set bits, so a model does not have to decode 16 bits itself."""
    high = [pin for pin in range(16) if value >> pin & 1]
    note = ' reserved:' + ','.join('%s%d' % (port, p) for p in sorted(reserved)) \
        if reserved else ''
    return 'GPIO%s=0x%04X high=%s%s' % (
        port, value, ','.join(str(p) for p in high) or '-', note)


NO_CONTACT = (' -> check the board is powered, and that a JTAG programmer or '
             'a dedicated serial adapter is connected between it and this PC')

# ConnectError and NoReplyError both mean no data came back, from different
# places - ConnectError is the port itself not existing, NoReplyError is the
# port opening fine and the board staying silent - but a first identify wraps
# either one as ConnectError (`coaxial.board._build`), so both need the same
# troubleshooting line: there is no reliable way to tell a caller which of the
# two actually happened.
HINTS = {
    'DeviceStateError': ' -> afe_power(action=on)',
    'ConnectError': NO_CONTACT,
    'NoReplyError': NO_CONTACT,
    'UnsupportedProtocolError': ' -> host library is older than the firmware',
}


def hint(exc):
    """The suffix error() appends for this exception, or ''.

    Exposed separately so a caller building its own message around the same
    exception - dbg.py's own startup banner, not just a tool result - gets the
    same troubleshooting line rather than a second, differently-worded one.
    """
    return HINTS.get(type(exc).__name__, '')


def error(exc):
    """One line. Names the exception type and, where known, the way out."""
    return 'ERR %s: %s%s' % (type(exc).__name__, exc, hint(exc))


def subsystems(rows):
    """What the board is made of, one per line, as the board reported it."""
    if not rows:
        return 'subsystems: this firmware does not report them'
    return '\n'.join(
        ['subsystems: %d' % len(rows)]
        + ['  %-9s %2d cmd  %s' % (r['name'], r['commands'], r['what'])
           for r in rows])


def parts(rows):
    """What is fitted, one per line, as the board reported it.

    `power` earns its column: AFE_ON powers the IMU as well as the analog
    front end, and a part reading `unpowered` is the answer to most of the
    questions that get asked about it.
    """
    if not rows:
        return 'parts: this firmware does not report them'
    return '\n'.join(
        ['parts: %d' % len(rows)]
        + ['  %-15s %-10s %-16s %-9s %s'
           % (r['name'], r['state'], r['where'],
              r['power'] or '-', r['what'])
           for r in rows])


def angle(state):
    """The angle sensor's reading, headed like every other one."""
    if state['value'] is None:
        return ('angle: no reading - loop %s, %s'
                % (state['loop'], state['error']))

    lines = ['angle: %s   loop %s, %d readings, %d errors'
             % (state['register_name'], state['loop'], state['updates'],
                state['errors'])]

    if 'degrees' in state:
        lines.append('  %7.2f deg   %5d counts of 4096   flags %X'
                     % (state['degrees'], a1335.counts(state['value']),
                        state['flags']))
    elif 'kelvin' in state:
        lines.append('  %7.1f K     %5d counts, eighths of a kelvin'
                     % (state['kelvin'], a1335.counts(state['value'])))
    else:
        lines.append('  0x%04X raw' % state['value'])

    return '\n'.join(lines)


#: The observer's nodes in the order a reader wants them: the legs, the
#: housekeeping, the laminate, the motor.
THERMAL_GROUPS = (
    ('legs', ('driver_u', 'driver_v', 'driver_w', 'phase_u', 'phase_v',
              'phase_w')),
    ('parts', ('mcu', 'regulators', 'afe', 'hotswap')),
    ('laminate', ('board', 'patch_u', 'patch_v', 'patch_w', 'patch_left',
                  'patch_bottom', 'patch_right')),
    ('motor', ('winding', 'stator', 'rotor')),
)


def thermal_state(state):
    """The observer's picture: the one measurement, then every estimate
    by group, then the room it runs against - device 8 op 0."""
    from coaxial.thermal import pretty

    ntc = state.get('ntc')
    head = ('thermal: NTC %.1f C measured' % ntc if ntc is not None
            else 'thermal: NTC unread - AFE off, the observer runs open')
    lines = [head + '   open %d s, %s, a sample every %.0f s'
             % (state.get('seconds') or 0,
                'settled' if state.get('settled') else 'settling',
                state.get('sample_every_s') or 0.0)]
    nodes = state.get('nodes') or {}
    for group, names in THERMAL_GROUPS:
        have = [(name, nodes[name]) for name in names if name in nodes]
        if have:
            lines.append('  %-8s %s' % (group, '  '.join(
                '%s %.1f' % (pretty(name), celsius) for name, celsius in have)))
    if state.get('ambient') is not None:
        lines.append('  room     %.1f C, identified - the board has no '
                     'ambient sensor' % state['ambient'])
    lines.append('  every figure an ESTIMATE but the NTC, degrees C')
    return '\n'.join(lines)


def thermal_budget(budget):
    """The envelope's spend - device 8 op 4: the worst node against the
    ceiling in force, what the clamp is doing, the joules left."""
    from coaxial.thermal import pretty

    worst_node = budget.get('worst_node') or '?'
    verdict = ('TRIPPED' if budget.get('tripped')
               else 'throttling' if budget.get('throttling') else 'ok')
    trips = budget.get('trips') or 0
    lines = ['budget: worst %s at %.0f %% of its span in force, %s%s'
             % (pretty(worst_node), 100.0 * (budget.get('worst') or 0.0),
                verdict, ', %d trips' % trips if trips else '')]
    if budget.get('derate') is not None:
        lines.append('  clamp %.0f %% of the record\'s current limit'
                     % (100.0 * budget['derate']))
    soak = budget.get('soak_j') or {}
    left = budget.get('seconds_to_limit')
    if worst_node in soak:
        lines.append('  %.1f J left in %s before its ceiling%s'
                     % (soak[worst_node], pretty(worst_node),
                        ', %.0f s at this power' % left
                        if left is not None else ''))
    if 'winding_c' in budget:
        lines.append('  winding %.1f C, %.0f %% of its span, own factor %.2f'
                     % (budget['winding_c'], 100.0 * budget['winding_used'],
                        budget['winding_derate']))
    lines.append('  the ceilings are the record\'s, trimmed by the margin - '
                 'thermal op=ident says how much')
    return '\n'.join(lines)


def thermal_ident(ident):
    """The online identification - device 8 op 10: the state as a word,
    the margin the envelope acts on, the scales, the room."""
    floor = ident.get('margin_floor')
    lines = ['ident: %s   margin %.2f of every span%s'
             % (ident.get('state', '?'), ident.get('margin', 1.0),
                ', floor %.2f' % floor if floor is not None else '')]
    scales, sigma = ident.get('scales') or {}, ident.get('sigma') or {}
    online = [name for name in (ident.get('online') or []) if name in scales]
    if online:
        lines.append('  ' + '  '.join('%s %.2f +-%.2f' % (name, scales[name],
                                                          sigma.get(name, 0.0))
                                      for name in online)
                     + '   scales on the record\'s network, one the default')
    if ident.get('ambient') is not None:
        lines.append('  room %.1f +-%.1f C identified'
                     % (ident['ambient'], ident.get('ambient_sigma') or 0.0))
    lines.append('  innovation %.2f K, %d updates'
                 % (ident.get('innovation_k') or 0.0, ident.get('updates') or 0))
    truth = ident.get('truth')
    if truth:
        lines.append('  stand-in: %s, air %.2f, capacity %.2f, room %.0f C'
                     % (truth.get('situation'), truth.get('air', 1.0),
                        truth.get('capacity', 1.0), truth.get('ambient', 25.0)))
    lines.append('  the state is a word; the margin is what the envelope acts on')
    return '\n'.join(lines)


def angle_registers(rows):
    """The registers a bring-up asks for, raw beside what they decode to."""
    out = ['angle registers: %d' % len(rows)]
    for name, value in rows:
        if name == 'ANG':
            said = '%7.2f deg' % a1335.degrees(value)
        elif name == 'TSEN':
            said = '%7.1f K' % a1335.kelvin(value)
        elif name == 'FIELD':
            said = '%5d gauss' % a1335.gauss(value)
        else:
            said = ''
        out.append('  %-6s 0x%04X  flags %X  %s' % (name, value, value >> 12,
                                                    said))
    return '\n'.join(out)


def imu(what, payload):
    """The IMU, as a headed block like every other reading.

    Raw counts and the scaled value both, on one line, because the counts
    are what the part said and the scaling is this host's arithmetic over a
    Q point - the same distinction the analog table keeps.
    """
    if what == 'id':
        return 'imu: %s' % ('  '.join('%s=%s' % (k, payload[k]) for k in
                                      ('sw_version', 'sw_part', 'sw_build',
                                       'reset_cause_name')))

    if what == 'state':
        head = ('imu: loop %s, %d vectors, %d cargoes, %d errors'
                % (payload['loop'], payload['updates'], payload['cargoes'],
                   payload['errors']))
        q = payload.get('quaternion')
        if q is None:
            return (head + '\n' + '  no rotation vector yet - %s'
                    % ('enable one with op=feature report_id=5'
                       if payload['error'] == 'none' else payload['error']))
        counts = payload['counts']
        return (head + '\n' + '  %-12s %7d %7d %7d %7d  %+.4f %+.4f %+.4f '
                '%+.4f  acc=%s'
                % (payload['name'], counts['i'], counts['j'], counts['k'],
                   counts['real'], q['i'], q['j'], q['k'], q['real'],
                   payload['accuracy']))

    head = 'imu: channel %d (%s), %d cargo bytes' % (
        payload['channel'], payload['channel_name'], len(payload['cargo']))
    rows = payload['reports']
    if not rows:
        return head + '\n' + ('  nothing decoded - an idle '
                                  'part sends a zero-length cargo')

    lines = [head]
    for row in rows:
        if 'base_delta_100us' in row:
            lines.append('  %-12s base delta %d x100us'
                         % (row['name'], row['base_delta_100us']))
            continue
        counts = ' '.join('%7d' % v for v in row['raw'])
        if 'scaled' in row:
            lines.append('  %-12s %s  %s %s  acc=%s'
                         % (row['name'], counts,
                            ' '.join('%+9.4f' % v for v in row['scaled']),
                            row['unit'], row['accuracy']))
        else:
            lines.append('  %-12s %s  acc=%s'
                         % (row['name'], counts, row['accuracy']))
    return '\n'.join(lines)


def checks(results):
    """Self-test results, one per line. A leading marker so the pass/fail split
    is visible without the model parsing a word: 'ok', 'FAIL', or blank for the
    informational values whose judgement belongs to the test executive."""
    marks = {'pass': 'ok  ', 'fail': 'FAIL', 'info': '    '}
    lines = []
    for check in results:
        # An INFO value of zero is a reading, not an absence, so it is printed.
        # For pass/fail rows the value field is unused, so a zero is noise.
        show = check['status'] == 'info' or check['value']
        lines.append(('%s %-20s %s' % (marks.get(check['status'], '?   '),
                                       check['name'],
                                       check['value'] if show else '')).rstrip())
    return '\n'.join(lines)
