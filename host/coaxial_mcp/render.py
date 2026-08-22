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


def board_info(version, clock, channels):
    lines = [
        '%s %s fw%s proto%d.%d build "%s"' % (
            version.get('device', '?'), version.get('mcu', '?'),
            version['firmware'], version['proto_major'], version['proto_minor'],
            version.get('build', '?')),
        'sysclk %s hclk %s src %s cmds %s' % (
            si(clock['sysclk_hz']), si(clock['hclk_hz']), clock['source'],
            version.get('commands', '?')),
        'ch adc pin          mode name',
    ]
    for c in channels:
        lines.append('%-2d %-3d %-12s %-4s %s' % (
            c['index'], c['adc'], c['pin'],
            'diff' if c['differential'] else 'SE',
            short(c['signal'], c['index'])))
    return '\n'.join(lines)


def analog(result, derived):
    """One line per channel: index, name, mode, mean raw, pin volts, unit value."""
    lines = ['%d smp @%.0fHz' % (result['samples'], result['rate_hz'] or 0)]
    for row in result['channels']:
        name = short(row['signal'], row['index'])
        extra = derived.get(row['index'], '')
        # The parentheses matter: without them .rstrip() binds to the argument
        # tuple rather than to the formatted string.
        lines.append(('%-2d %-7s %-4s %8.1f %+8.4fV %s' % (
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


def error(exc):
    """One line. Names the exception type and, where known, the way out."""
    hint = {
        'DeviceStateError': ' -> afe_power(action=on)',
        'ConnectError': ' -> check the cable and the port name',
        'UnsupportedProtocolError': ' -> host library is older than the firmware',
    }.get(type(exc).__name__, '')
    return 'ERR %s: %s%s' % (type(exc).__name__, exc, hint)


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
    return chr(10).join(lines)
