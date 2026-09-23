"""One acquisition record, as an object with fields."""


class Sample:

    """One channel's value in one record."""

    __slots__ = ('name', 'unit', 'raw', 'count', 'value')

    def __init__(self, name, unit, raw, count):
        self.name = name
        self.unit = unit
        self.raw = raw
        self.count = count
        self.value = raw / float(count) if count else float(raw)

    def __repr__(self):
        # RAW, and it says so: the window's mean in converter codes, with the
        # unit the calibration record scales it into.
        return '<%s %.6g raw%s>' % (self.name, self.value,
                                    ' -> ' + self.unit if self.unit else '')


class Record(dict):

    """One record: when it started, how long it covered, what it holds."""

    __slots__ = ('start_time', 'dt', 'samples', 'digital', 'sensors')

    def __init__(self, mapping, fields, start_time=None, dt=None):
        dict.__init__(self, mapping)
        self.start_time = start_time
        self.dt = dt
        self.digital = mapping.get('digital')
        #: SNAPSHOTS, four raw words per sensor field - never sums, a
        #: summed quaternion means nothing (MINOR 7).
        self.sensors = mapping.get('sensors')
        count = mapping.get('samples', 1)
        self.samples = tuple(
            Sample(f['signal'], f.get('unit'), mapping[f['signal']], count)
            for f in fields if f['signal'] in mapping)

    @property
    def count(self):
        """Readings the board summed into every value in this record."""
        return self.get('samples', 1)

    def sample(self, name):
        """One channel's `Sample`, by name."""
        for s in self.samples:
            if s.name == name:
                return s
        raise KeyError('%r is not in this record. It has: %s'
                       % (name, ', '.join(self.channel_name)))

    def value(self, name):
        """One channel's mean - its sum over the count that made it."""
        return self.sample(name).value

    @property
    def channel_name(self):
        """The channels in this record, in `samples` order."""
        return tuple(s.name for s in self.samples)

    def __getattr__(self, name):
        # Channels by attribute where the name allows it, so `r.NTC` reads as
        # well as `r['NTC']`.
        try:
            return self[name]
        except KeyError:
            raise AttributeError(name) from None

    def __repr__(self):
        return '<Record %s +%s, %d channels>' % (
            '%.6f' % self.start_time if self.start_time else 'no clock',
            '%.6gs' % self.dt if self.dt else '?', len(self.samples))


def build(records, fields, times=None, before=None):
    """Wrap decoded records as `Record`s, giving each its own `dt`."""
    if not records:
        return []
    stamps = list(times) if times is not None else [None] * len(records)
    gaps = [None] * len(records)
    for i in range(len(records) - 1):
        here, following = stamps[i], stamps[i + 1]
        if here is not None and following is not None:
            gaps[i] = following - here
    if len(records) > 1:
        gaps[-1] = gaps[-2]
    elif before is not None and stamps[0] is not None:
        gaps[0] = stamps[0] - before
    return [Record(r, fields, stamps[i], gaps[i])
            for i, r in enumerate(records)]
