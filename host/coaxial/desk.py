"""Every analog channel at once, drawn as a meter bridge.

A table of seven numbers says what each channel reads. It does not say which
one moved, how far it swung while you were reading the row above it, or that
two of them are pinned. A meter bridge says all three at a glance, which is
why consoles have had one since before anyone wrote a test executive.

**Every channel carries its own scale, in its own unit.** A vertical bridge
cannot: one gutter cannot label amperes, volts and degrees at once, so the
first version of this shared a dBFS axis and left the reader to work out what
-14 dBFS meant on the DC link. Laid on its side there is room for the ends of
each scale beside the bar they belong to, which is what a meter is for.

The span is the CONVERTER'S, not an opinion about the board: a differential
channel runs +/-full scale and a single-ended one zero to full scale, both
converted through the same arithmetic `coaxial.scaling` uses. Invariant 10
holds - there is no expected value here, no limit, and no mark that means bad.

Three marks per bar, and they are three different measurements:

  * the block is the burst's mean;
  * the tick is the burst's own extreme - what the channel reached during the
    sample window, straight off the wire, not inferred here;
  * the caret is peak hold, decaying, and it is this module's memory of the
    windows before.

Pure but not stateless: a decay is memory by definition. It runs per update
rather than per second, so a test drives it without a clock.
"""
from . import ansi

#: Columns of bar. Wide, because the bar is the only part of a row that a
#: reader scans rather than reads.
BAR = 38

#: Columns for a channel's own scale. Wide enough for the widest of them -
#: the thermistor's, which is asymmetric and needs both ends written out.
SCALE = 13

#: How far a held peak creeps back per update, as a fraction of full scale.
#: At the ~8 updates a second these views run, 1.5 % falls the whole bar in
#: about eight seconds. 4 % was tried and is too quick to read as a hold: the
#: mark chases the bar closely enough to look like part of it, and the point
#: of a peak hold is that it lags.
DECAY = 0.015

#: Where the ink changes, as a fraction of full scale. A converter near its
#: rail has stopped moving, which is worth seeing before the number is read;
#: it says nothing about whether the current was too high.
HOT_ABOVE = 0.70
RAIL_ABOVE = 0.98

FULL = '█'
TRACK = '─'
CENTRE = '┼'
PEAK = '╵'
TICK = '│'

#: What a row is labelled with, from the board's own signal name.
SHORT = {'Phase U': 'U', 'Phase V': 'V', 'Phase W': 'W', 'DC bus': 'DC',
         'Clevel': 'CL', 'NTC': 'T', 'Cinj': 'CI'}

#: How each unit prints, and how many decimals a reader of it wants.
UNITS = {'mA': ('A', 1, 1000.0), 'mV': ('V', 1, 1000.0),
         'centi-degC': ('C', 1, 100.0)}


def fraction(row):
    """Where the mean sits, -1.0 to +1.0 of the converter's own range."""
    return _at(row, row['mean_raw'])


def _at(row, code):
    divisor = 32768.0 if row['differential'] else 65536.0
    return max(-1.0, min(1.0, code / divisor))


def unit_of(row):
    """(suffix, decimals, divisor) for whatever the board called this."""
    return UNITS.get(row.get('unit'), ('V', 3, 1.0))


def span(row):
    """(low, high) of this channel's scale, in its own unit, or None.

    Supplied by the caller, because converting a code is the analog layer's
    job and not a drawing's. Deriving it here from the reading and its
    fraction of full scale was tried: right for the phases and the DC link,
    which are linear in the code, and wrong for the thermistor, which is not.
    """
    got = row.get('span')
    return tuple(got) if got else None


class Desk:

    """A meter bridge over the analog channels, with decaying peak hold.

    Built from the board's own channel rows, so a board that grows a channel
    grows a bar and nothing here needs telling.
    """

    def __init__(self, decay=DECAY):
        self.decay = decay
        self._held = {}

    def _hold(self, key, low, high):
        """Advance one channel's held extremes and return them.

        The two ends are held separately and signed. Holding one magnitude
        and mirroring it was what put the peak mark on the wrong side of the
        bar: a phase sitting at +62 A showed its caret at -62, where the
        current had never been.

        Each end is pulled out at once by a reading beyond it and creeps back
        by `decay` an update, so the mark trails the bar instead of tracking
        it. That lag is the whole point of a peak hold - it is there to show
        where the signal went while you were reading the row above.
        """
        was = self._held.get(key)

        if was is None:
            now = (low, high)
        else:
            now = (min(low, was[0] + self.decay),
                   max(high, was[1] - self.decay))

        self._held[key] = now
        return now

    @staticmethod
    def _ink(magnitude):
        """Green, amber, red - read before any scale is."""
        if magnitude >= RAIL_ABOVE:
            return ansi.RED
        return ansi.AMBER if magnitude >= HOT_ABOVE else ansi.GREEN

    def _bar(self, row, colour):
        """One channel's bar, as a string of BAR columns."""
        bipolar = row['differential']
        here = fraction(row)
        least = _at(row, row.get('min_raw', row['mean_raw']))
        most = _at(row, row.get('max_raw', row['mean_raw']))
        held_low, held_high = self._hold(row['index'], least, most)

        origin = (BAR - 1) / 2.0 if bipolar else 0.0
        wide = (BAR - 1) / 2.0 if bipolar else (BAR - 1)

        def column(value):
            return int(round(origin + value * wide))

        # Both ends on a channel that swings either way; only the top on one
        # that cannot go below zero, where a mark at the floor says nothing.
        peaks = {column(held_high)}
        ticks = {column(most)}
        if bipolar:
            peaks.add(column(held_low))
            ticks.add(column(least))

        low, high = sorted((column(0.0) if bipolar else 0, column(here)))
        cells = []

        for index in range(BAR):
            if low <= index <= high and (bipolar or index <= column(here)):
                cells.append((FULL, self._ink(abs(here)) if colour else None))
            elif index in peaks:
                cells.append((PEAK, ansi.WHITE if colour else None))
            elif index in ticks:
                cells.append((TICK, ansi.WHITE if colour else None))
            elif bipolar and index == column(0.0):
                cells.append((CENTRE, ansi.DIM if colour else None))
            else:
                cells.append((TRACK, ansi.DIM if colour else None))

        if colour:
            return ansi.run(cells)
        return ''.join(text for text, _ in cells)

    @staticmethod
    def _scale(row):
        """This channel's ends and its reading, in its own unit."""
        suffix, decimals, _divisor = unit_of(row)
        ends = span(row)

        # Both ends, always. A plus-or-minus was tried and is wrong twice
        # over: the thermistor's scale is cold at one rail and hot at the
        # other, and even a phase channel is not symmetric - the converter
        # runs -32768 to +32767, which is one code short at the top.
        label = '?' if ends is None else '%.3g..%.3g' % ends
        label = label.rjust(SCALE)

        now = row.get('reading')
        if now is None:
            now = row.get('volts_at_pin', 0.0)

        return label, '%+.*f %s' % (decimals, now, suffix)

    def update(self, rows, colour=False):
        """The desk, as text, from one read_all()'s channel rows.

        `colour` off by default so a suite can compare the picture to an
        expected string. The tool that knows it is writing to a terminal
        turns it on.
        """
        lines = []
        for row in sorted(rows, key=lambda r: r['index']):
            name = SHORT.get(row['signal'], (row['signal'] or '?')[:2])
            ends, now = self._scale(row)
            bar = self._bar(row, colour)

            head = '%3s %s ' % (name, ends)
            tail = ' %9s' % now
            if colour:
                head = ansi.paint(head, ansi.WHITE)
                tail = ansi.paint(tail, ansi.WHITE)
            lines.append(head + bar + tail)

        return '\n'.join(lines)
