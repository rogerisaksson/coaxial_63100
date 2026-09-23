"""Binary payload codecs, mirroring comms/inc/wire.h on the firmware side."""
import struct

from coaxial.errors import PayloadError

_FORMATS = {'u8': '>B', 'i8': '>b', 'u16': '>H', 'i16': '>h',
            'u32': '>I', 'i32': '>i'}
_SIZES = {width: struct.calcsize(fmt) for width, fmt in _FORMATS.items()}

#: The wire's fixed-point scales. An SI value is multiplied by one of these
#: to become the integer a field holds, and divided by it on the way back.
CENTI = 100
MILLI = 1_000
MICRO = 1_000_000
NANO = 1_000_000_000
#: Q16.16: a count with sixteen fractional bits, in a u32.
Q16 = 1 << 16
#: A fraction in one byte, 255 being all of it.
BYTE_FRACTION = 255


def centi(value):
    """`value` as the wire's centi-unit integer: 25.37 C is 2537."""
    return int(round(value * CENTI))


def milli(value):
    """`value` as the wire's milli-unit integer: 1.5 A is 1500."""
    return int(round(value * MILLI))


def micro(value):
    """`value` as the wire's micro-unit integer: 0.8 is 800000."""
    return int(round(value * MICRO))


def q16(value):
    """`value` as Q16.16: sixteen fractional bits in a u32."""
    return int(round(value * Q16))


def bits(word, names):
    """`word`'s low bits as {name: bool}, `names` in bit order from bit 0."""
    return {name: bool(word >> i & 1) for i, name in enumerate(names)}


def label(names, index, kind):
    """`names[index]`, or `kind` and the number for one past the table."""
    return names[index] if index < len(names) else '%s%d' % (kind, index)


def pack(*fields):
    """Encode ('u16', 1234), ('u8', 7), ... into a request payload."""
    out = []
    for kind, value in fields:
        try:
            out.append(struct.pack(_FORMATS[kind], value))
        except KeyError as exc:
            raise ValueError('unknown field width %s; have %s'
                             % (exc, ', '.join(sorted(_FORMATS)))) from exc
        except struct.error as exc:
            raise ValueError('%r does not fit a %s' % (value, kind)) from exc
    return b''.join(out)


def pages(fetch, absent=(), first=0):
    """Every page of a paged reply, until the board says that was the last."""
    while True:
        try:
            page = Page(fetch(first))
        except absent:
            return
        yield page
        if page.last:
            return
        first = page.first + page.count


class Reader:
    """Forward-only reader over a response payload."""

    def __init__(self, payload):
        self.payload = payload
        self.position = 0

    def take(self, count):
        end = self.position + count
        if end > len(self.payload):
            raise PayloadError('payload underrun: wanted %d more byte(s) at %d '
                               'of %d' % (count, self.position, len(self.payload)))
        chunk = self.payload[self.position:end]
        self.position = end
        return chunk

    def field(self, width):
        """One field of the named width: u8, i8, u16, i16, u32 or i32."""
        return struct.unpack(_FORMATS[width], self.take(_SIZES[width]))[0]

    def maybe(self, width):
        """The next field, or None when the reply stopped before it."""
        return self.field(width) if self.remaining >= _SIZES[width] else None

    def u8(self):
        return self.take(1)[0]

    def i8(self):
        return self.field('i8')

    def u16(self):
        return self.field('u16')

    def u32(self):
        return self.field('u32')

    def i16(self):
        return self.field('i16')

    def i32(self):
        return self.field('i32')

    def centi(self, width='i32'):
        """A field in hundredths: centi-degrees, centi-volts."""
        return self.field(width) / CENTI

    def milli(self, width='i32'):
        """A field in thousandths: milliamps, milliseconds, milli-K/W."""
        return self.field(width) / MILLI

    def micro(self, width='i32'):
        """A field in millionths: microradians, parts per million."""
        return self.field(width) / MICRO

    def nano(self, width='u32'):
        """A field in billionths: nanoseconds."""
        return self.field(width) / NANO

    def q16(self):
        """A u32 in Q16.16."""
        return self.u32() / Q16

    def fraction(self):
        """A u8 as a fraction of 255."""
        return self.u8() / BYTE_FRACTION

    def flags(self, names):
        """A u8 of flags as {name: bool}, `names` from bit 0 up."""
        return bits(self.u8(), names)

    def string(self):
        """One length byte, then that many ASCII characters. Never terminated."""
        return self.take(self.u8()).decode('ascii', 'replace')

    @property
    def remaining(self):
        return len(self.payload) - self.position


class Page(Reader):
    """One page of a paged reply: `total` rows in all, `first` where this page
    starts, `count` how many it holds - and then the rows.
    """

    def __init__(self, payload):
        super().__init__(payload)
        self.total, self.first, self.count = self.u8(), self.u8(), self.u8()

    def rows(self):
        """One step per row on this page."""
        return range(self.count)

    def indices(self):
        """The rows' indices in the whole table."""
        return range(self.first, self.first + self.count)

    @property
    def last(self):
        """Whether the board has nothing after this page."""
        return self.count == 0 or self.first + self.count >= self.total
