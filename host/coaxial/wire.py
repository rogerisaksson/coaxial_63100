"""Binary payload codecs, mirroring Comms/Inc/wire.h on the firmware side.

Big-endian throughout, matching every field in a Modbus PDU except the CRC.
No floating point ever goes on the wire: physical quantities travel as scaled
integers in units the command documents, and the scaling happens here or in
scaling.py where it can be parameterised.

The firmware's writer is deliberately total - it sets a sticky flag rather than
failing at the point of use - because that keeps the C handlers flat. The host
has exceptions, so the reader raises instead. Same contract, idiomatic on each
side.
"""
import struct

from .errors import PayloadError

_FORMATS = {'u8': '>B', 'u16': '>H', 'u32': '>I', 'i16': '>h', 'i32': '>i'}


def pack(*fields):
    """Encode ('u16', 1234), ('u8', 7), ... into a request payload.

    Naming the width at each field rather than passing a format string keeps a
    call site readable next to the command's documented layout.
    """
    try:
        return b''.join(struct.pack(_FORMATS[kind], value) for kind, value in fields)
    except KeyError as exc:
        raise ValueError('unknown field width %s; have %s'
                         % (exc, ', '.join(sorted(_FORMATS)))) from exc


class Reader:
    """Forward-only reader over a response payload.

    Raises PayloadError on underrun rather than returning filler, so a truncated
    reply surfaces as an error at the field that was missing instead of as a
    plausible zero somewhere downstream.
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

    def u8(self):
        return self.take(1)[0]

    def u16(self):
        return struct.unpack('>H', self.take(2))[0]

    def u32(self):
        return struct.unpack('>I', self.take(4))[0]

    def i16(self):
        return struct.unpack('>h', self.take(2))[0]

    def i32(self):
        return struct.unpack('>i', self.take(4))[0]

    def string(self):
        """One length byte, then that many ASCII characters. Never terminated."""
        return self.take(self.u8()).decode('ascii', 'replace')

    @property
    def remaining(self):
        return len(self.payload) - self.position
