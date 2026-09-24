"""Every analog channel at once, drawn as a meter bridge."""
from coaxial.draw import ansi, machine, gauges
from coaxial.devices.scaling import ADC_CODES, ADC_HALF_CODES

#: Columns of bar. Wide, because the bar is the only part of a row that a
#: reader scans rather than reads.
BAR = 38

#: Columns for a channel's own scale. Wide enough for the widest of them -
#: the thermistor's, which is asymmetric and needs both ends written out.
SCALE = 13

#: THE RELEASE: how far a held peak falls back toward the bar's level an
#: update, as a fraction of the distance between them. Fifteen percent
#: at the ~8 updates a second these views run is a time constant of
#: eight tenths of a second - pushed out by the burst's extreme at once,
#: back on the level within a few seconds - and it can never fall below
#: the window's own extreme, which is the tick beside it. The bench,
#: 2026-09-06: "the decay meter lags behind the value in the bar; it
#: should be a typical peak hold that decays toward the current value -
#: the value pushes the hold, which then falls back". Before, it fell a
#: FIXED 1.5 % of full scale an update whatever the distance: eight
#: seconds for the whole bar, and on a phase swinging over a couple of
#: seconds the caret was never where the bar had been but where it was
#: seconds ago. A quarter ran first - 3.2 cells behind at the sweep's
#: worst - and the bench asked for "a bit slower": 0.15 is 5.1 cells.
RELEASE = 0.15

#: The least a held peak moves an update, as a fraction of full scale, so
#: the release lands on the level rather than approaching it for ever.
#: 1.5 % was the whole decay once; 4 % was tried then and was too quick
#: to read as a hold at all.
DECAY = 0.015

#: Where the ink changes, as a fraction of full scale. A converter near its
#: rail has stopped moving, which is worth seeing before the number is read;
#: it says nothing about whether the current was too high.
HOT_ABOVE = 0.70
RAIL_ABOVE = 0.98

#: THE BAR IS THE MOTOR PAGE'S GAUGE, in dots: `coaxial.draw.gauges` draws it,
#: the level three dots tall at dot resolution, the empty scale in the
#: track's grey, the centre of a bipolar channel marked, the burst's
#: extremes as full-height ticks and the held peaks as a single top dot.
#: It was `█` on `─` with `╵` and `│` for the marks - a third drawing
#: convention on a terminal whose other pages draw in braille, and the
#: bench asked for one instrument on every page.

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
    divisor = ADC_HALF_CODES if row['differential'] else ADC_CODES
    return max(-1.0, min(1.0, code / divisor))


def unit_of(row):
    """(suffix, decimals, divisor) for whatever the board called this."""
    return UNITS.get(row.get('unit'), ('V', 3, 1.0))


def span(row):
    """(low, high) of this channel's scale, in its own unit, or None."""
    got = row.get('span')
    return tuple(got) if got else None


class Desk:

    """A meter bridge over the analog channels, with decaying peak hold."""

    def __init__(self, decay=DECAY, bar=BAR):
        self.decay = decay
        self.bar = bar
        self._held = {}

    def _hold(self, key, low, high, here):
        """Advance one channel's held extremes and return them."""
        was = self._held.get(key)

        if was is None:
            now = (low, high)
        else:
            fall = max(self.decay, RELEASE * (was[1] - here))
            rise = max(self.decay, RELEASE * (here - was[0]))
            now = (min(low, was[0] + rise), max(high, was[1] - fall))

        self._held[key] = now
        return now

    @staticmethod
    def _ink(magnitude):
        """The level's class - the motor page's green, amber, red - read
        before any scale is: a converter near its rail has stopped
        moving, which is worth seeing before the number is read."""
        if magnitude >= RAIL_ABOVE:
            return machine.SOA_TRIP
        return machine.SOA_WARN if magnitude >= HOT_ABOVE else machine.SOA_OK

    def _bar(self, row, colour):
        """One channel's bar, as a string of BAR columns."""

        bipolar = row['differential']
        here = fraction(row)
        least = _at(row, row.get('min_raw', row['mean_raw']))
        most = _at(row, row.get('max_raw', row['mean_raw']))
        held_low, held_high = self._hold(row['index'], least, most, here)

        def share(value):
            # A bipolar channel's -1..+1 laid on the gauge's 0..1.
            return (value + 1.0) / 2.0 if bipolar else value

        # Both ends on a channel that swings either way; only the top on one
        # that cannot go below zero, where a mark at the floor says nothing.
        marks = [(share(most), gauges.MARK),
                 (share(held_high), gauges.MARK, gauges.PEAK)]
        if bipolar:
            marks += [(share(least), gauges.MARK),
                      (share(held_low), gauges.MARK, gauges.PEAK)]
        return gauges.gauge(share(here), self.bar, cls=self._ink(abs(here)),
                            centre=0.5 if bipolar else None, marks=marks,
                            colour=colour)

    @staticmethod
    def _scale(row):
        """This channel's ends and its reading, in its own unit."""
        suffix, decimals, _divisor = unit_of(row)
        ends = span(row)

        # Both ends, always.
        label = '?' if ends is None else '%.3g..%.3g' % ends
        label = label.rjust(SCALE)

        now = row.get('reading')
        if now is None:
            now = row.get('volts_at_pin', 0.0)

        return label, '%+.*f %s' % (decimals, now, suffix)

    def update(self, rows, colour=False):
        """The desk, as text, from one analog read()'s channel rows."""
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
