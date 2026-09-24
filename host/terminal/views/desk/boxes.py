"""The meter bridge's boxes: the chain, the digital pins, the buffer, the scale, the legend."""
import re
from contextlib import suppress

from coaxial.devices import scaling
from coaxial.draw import desk
from terminal.ui.screen import gauge
from terminal.ui.stage import hud
from terminal.views.desk.load import _line_share, wire_rate


#: The codes a scale is measured between. Not 0 and 65535: the thermistor
#: conversion diverges at both rails - infinite resistance one end, zero the
#: other - so its scale is quoted between codes the converter can actually
#: resolve rather than at two asymptotes.
SPAN_CODES = (1024, 64512)


def chain_box(chain, sweeps, channels=0):
    """The low-pass the board is running, and the loop underneath it."""

    title = ('LOW PASS (%d CHANNELS)' % channels if channels
             else 'LOW PASS')
    if chain is None:
        return hud(title, [
            '  none - the window averages instead',
            '  the loop could not be measured to design one',
            '  loop    %8s sweeps/s' % ('%.0f' % sweeps if sweeps else '-'),
        ])

    return hud(title, [
        '  cutoff  %8.2f Hz   order %d' % (chain['cutoff'],
                                            chain['order']),
        '  rejects %8.1f dB   %d biquads' % (chain['worst_alias_db'],
                                              len(chain['sections'])),
        '  sum %5d x keep 1/%-4d -> %6.1f rec/s' % (chain['boxcar'],
                                                     chain['decimate'],
                                                     chain['out_rate']),
        '  loop    %8s sweeps/s%s'
        % ('%.0f' % sweeps if sweeps else '-',
           '' if not chain.get('sweeps') else
           '  (designed %.0f)' % chain['sweeps']),
    ])


#: The indicator, and the two colours it comes in. One glyph, so the
#: only difference is the colour - U+25A3, a thin-framed box with a fill.
#: Written as ANSI because the art reaches the frame as a string that
#: `Marquee` parses; a Rich style object would arrive as literal text.
BOX = '\u25a3'

LIT = '\x1b[38;5;40m'          # green: the pin was high for most of the window

DARK = '\x1b[38;5;242m'        # grey: it was not

OFF = '\x1b[0m'

LABEL = '\x1b[38;5;66m'        # the theme's `label`, the street

#: The half of a pin's name that is a peripheral function rather than a
#: signal. The board names a pin both ways - `TIM1_CH1N/PWMUL` is the
#: timer's channel and the schematic's gate - and truncating the string
#: keeps the wrong one: six gates all read `TIM1_CH1N/` and nothing said
#: which leg. Dropped, `PWMUL` and `nFAULT` are what is left.
PERIPHERAL = re.compile(r'^(TIM|SPI|USART|UART|JT|NJ)')


def short(name):
    """The signal half of a pin's name, at most nine cells."""
    parts = [p for p in name.split('/') if not PERIPHERAL.match(p)]
    return (parts[0] if parts else name)[:9]


def digital_box(pins, width=34):
    """Every sampled pin as a lit or dark box, in its own frame."""

    if not pins:
        return hud('DIGITAL', ['  this task carries no pins'])

    # The colour is the value.
    cells = ['%s%s%s %s%-10s%s'
             % (LIT if duty >= 0.5 else DARK, BOX, OFF, LABEL,
                short(name), OFF)
             for name, duty in pins.items()]

    # Wrapped to the region rather than the terminal: the frame crops, and a
    # row that ran past it would slide instead of showing.
    lines, line = [], ''
    for cell in cells:
        plain = re.sub(r'\x1b\[[0-9;]*m', '', line + cell)
        if line and len(plain) > width:
            lines.append(line.rstrip())
            line = ''
        line += cell + '  '
    if line:
        lines.append(line.rstrip())
    return hud('DIGITAL', lines)


def buffer_box(state, host, link=None):
    """Every buffer between the converter and the frame, and the rate."""

    def row(name, held, peak, dropped, capacity=None):
        # The scale is the buffer's own peak, never its capacity: the ring
        # holds thousands and a reader keeping up leaves one in it, so a bar
        # against capacity is pinned at zero however hard the link works.
        top = float(max(peak, 1))
        room = '' if capacity is None else ' of %d' % capacity
        bar = gauge(min(1.0, held / top), 12)
        return ['  %-7s ' % name + bar + ' %4d' % held,
                '          peak %4d%s   dropped %d' % (peak, room, dropped)]

    lines = []
    if link is not None:
        lines += row('LINK', link['host'], link['peak'], link['dropped'])
    lines += row('HOST', host['held'], host['peak'], host['dropped'])

    capacity = (state or {}).get('capacity') or 0
    if not capacity:
        lines.append('  TARGET  this board reports no level')
    else:
        lines += row('TARGET', state.get('available') or 0,
                     state.get('worst') or 0, state.get('dropped') or 0,
                     capacity)
    if link is not None and link.get('rate'):
        lines.append('  link    %6.1f reads/s%s'
                     % (link['rate'], _line_share(link)))

    # The rate in the title, as LOW PASS carries its channel count, in a unit
    # that has digits to show.
    title = 'BUFFER'
    if link is not None and link.get('bits'):
        title = 'BUFFER (%s)' % wire_rate(link['bits'])
    return hud(title, lines)


def scale(rows, params=None):
    """Add the scaled value and the channel's own scale, in its own unit."""
    low_code, high_code = SPAN_CODES

    for row in rows:
        unit, raw = row.get('unit'), row['mean_raw']

        convert = scaling.converter(unit, row['differential'],
                                    signal=row.get('signal'),
                                    params=params)

        # The inset codes are the thermistor's alone.
        if row['differential']:
            first, last = -32768, 32767
        elif unit == 'centi-degC':
            first, last = low_code, high_code
        else:
            first, last = 0, 65535

        with suppress(ValueError):
            row['reading'] = convert(raw)
            ends = (convert(first), convert(last))
            row['span'] = (min(ends), max(ends))

    return rows


def legend(rows, held):
    """The channel legend: full name, live value, held min and max."""

    lines = []
    for row in rows:
        name = row['signal']
        now = row.get('reading')
        unit = scaling.symbol(row.get('unit'), name)
        if now is None:
            continue
        convert = scaling.converter(row.get('unit'), row['differential'],
                                    signal=name, params=row.get('params'))
        try:
            lo = convert(row['min_raw'])
            hi = convert(row['max_raw'])
        except (KeyError, ValueError):
            lo = hi = now
        keep = held.setdefault(name, [now, now])
        keep[0] = min(lo, keep[0] + desk.RELEASE * (now - keep[0]))
        keep[1] = max(hi, keep[1] + desk.RELEASE * (now - keep[1]))
        lines.append((name, '%+9.3f %-2s  %+8.2f/%+8.2f'
                      % (now, unit, keep[0], keep[1])))
    return hud('LEGEND  now / held lo / hi', lines)
