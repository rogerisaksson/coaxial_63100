"""One acquisition record, as an object with fields.

    values = daq.read(-1)
    start = values[0].start_time
    dt    = values[0].dt
    for sample in values[0].samples:
        print(sample.name, sample.value, sample.unit)

A `dict` underneath, so everything written against the mapping keeps
working: `record['NTC']` is still the SUM the board sent and
`record['samples']` is still the count that made it. Both spellings for
one wire is deliberate, because the wire's vocabulary is not the one a
script wants to read.

THE ONE COLLISION, said out loud: `record['samples']` is a COUNT and
`record.samples` is the ARRAY of per-channel values. The count is on the
object as `.count`. They are not the same thing and never were - the
board sends a sum and the number of readings in it (invariant 7), and a
caller wanting the mean divides one by the other, which is what `.value`
below has already done.
"""


class Sample:

    """One channel's value in one record.

    `raw` is what the board sent - a SUM of `count` readings - and `value`
    is that sum over that count, which is the mean of the window. Nothing
    here scales into volts or amperes: that is `board.analog`'s, and a
    record that guessed at units would be a second answer to a question
    the calibration record already answers.
    """

    __slots__ = ('name', 'unit', 'raw', 'count', 'value')

    def __init__(self, name, unit, raw, count):
        self.name = name
        self.unit = unit
        self.raw = raw
        self.count = count
        self.value = raw / float(count) if count else float(raw)

    def __repr__(self):
        return '<%s %.6g%s>' % (self.name, self.value,
                                ' ' + self.unit if self.unit else '')


class Record(dict):

    """One record: when it started, how long it covered, what it holds."""

    __slots__ = ('start_time', 'dt', 'samples', 'digital')

    def __init__(self, mapping, fields, start_time=None, dt=None):
        dict.__init__(self, mapping)
        self.start_time = start_time
        self.dt = dt
        self.digital = mapping.get('digital')
        count = mapping.get('samples', 1)
        self.samples = tuple(
            Sample(f['signal'], f.get('unit'), mapping[f['signal']], count)
            for f in fields if f['signal'] in mapping)

    @property
    def count(self):
        """Readings the board summed into every value in this record."""
        return self.get('samples', 1)

    def __getattr__(self, name):
        # Channels by attribute where the name allows it, so `r.NTC` reads
        # as well as `r['NTC']`. Only after the slots above, and only for
        # keys that are actually there - anything else is an
        # AttributeError like any other typo.
        try:
            return self[name]
        except KeyError:
            raise AttributeError(name) from None

    def __repr__(self):
        return '<Record %s +%s, %d channels>' % (
            '%.6f' % self.start_time if self.start_time else 'no clock',
            '%.6gs' % self.dt if self.dt else '?', len(self.samples))


def build(records, fields, times=None):
    """Wrap decoded records as `Record`s, giving each its own `dt`.

    `dt` IS MEASURED, NOT CONFIGURED. It comes from the gap to the next
    record's timestamp, because what a task was asked for and what the
    loop managed are different numbers - the whole reason the board sends
    a count with every sum. The last record in a block has no next one and
    inherits the gap before it; a block of one has none to inherit and
    says so with None.
    """
    if not records:
        return []
    stamps = list(times) if times is not None else [None] * len(records)
    gaps = [None] * len(records)
    for i in range(len(records) - 1):
        if stamps[i] is not None and stamps[i + 1] is not None:
            gaps[i] = stamps[i + 1] - stamps[i]
    if len(records) > 1:
        gaps[-1] = gaps[-2]
    return [Record(r, fields, stamps[i], gaps[i])
            for i, r in enumerate(records)]
