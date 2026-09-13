"""Binary payload codecs, mirroring comms/inc/wire.h on the firmware side.

Big-endian throughout, matching every field in a Modbus PDU except the CRC.
No floating point ever goes on the wire: physical quantities travel as scaled
integers in units the command documents, and the scaling happens here or in
scaling.py where it can be parameterised.

The scales are named once. A field documented as centi-degrees is read with
`r.centi()` and written with `centi(value)`, not divided by a literal at
every call site that has to get it right - invariant 7 applied to the wire:
the conversion is named where it is defined, and defined once.

The firmware's writer is deliberately total - it sets a sticky flag rather than
failing at the point of use - because that keeps the C handlers flat. The host
has exceptions, so the reader raises instead. Same contract, idiomatic on each
side.
"""
import struct

from .errors import PayloadError

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
    """`names[index]`, or `kind` and the number for one past the table.

    A board can answer more rows than this host has names for - a node
    added to the thermal graph, a scale the identification grew - and
    'node11' beside the named ones is honest where an IndexError is not.
    """
    return names[index] if index < len(names) else '%s%d' % (kind, index)


def pack(*fields):
    """Encode ('u16', 1234), ('u8', 7), ... into a request payload.

    Naming the width at each field rather than passing a format string keeps a
    call site readable next to the command's documented layout. A value that
    does not fit its width is a ValueError here, before a request is formed.
    """
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
    """Every page of a paged reply, until the board says that was the last.

    `fetch(first)` asks for the rows from `first` on and returns the
    payload; each page's header says how many rows there are in all, where
    this one starts and how many it holds - `Page` reads it. `absent` names
    the exceptions that mean an older firmware has no such op, which ends
    the walk with what was read rather than raising: an empty list says so
    without making the whole map fail.
    """
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
    """Forward-only reader over a response payload.

    Raises PayloadError on underrun rather than returning filler, so a truncated
    reply surfaces as an error at the field that was missing instead of as a
    plausible zero somewhere downstream. The scaled readers - `centi`,
    `milli`, `micro`, `nano`, `q16`, `fraction` - read a field in the unit
    the command documents it in.
    """

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
        """The next field, or None when the reply stopped before it.

        Payloads are append-only (invariant 3): a board older than the MINOR
        that appended a field answers a reply that ends before it, and
        absent is the honest answer - not a raise, and not a zero.
        """
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
    """One page of a paged reply: `total` rows in all, `first` where this
    page starts, `count` how many it holds - and then the rows."""

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
