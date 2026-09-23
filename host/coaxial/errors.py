"""Exception hierarchy for the coaxial_63100 host library."""


class RigError(Exception):
    """Base class, so a caller can catch this library in one clause."""


class ConnectError(RigError):
    """The serial port would not open, or a slave did not answer on it."""


class NoReplyError(RigError):
    """The slave said nothing within the timeout."""


class CrcError(RigError):
    """A reply arrived but its checksum did not hold."""


class FrameError(RigError):
    """A reply arrived that was not the answer to the question asked."""


class PayloadError(FrameError):
    """A reply was the right shape but ran out of bytes while being decoded."""


class UnsupportedProtocolError(RigError):
    """The slave speaks a protocol major this host has no codec for."""


class DeviceStateError(RigError):
    """The board is in a state where the request cannot mean anything."""


EXCEPTION_NAMES = {
    1: 'ILLEGAL FUNCTION',
    2: 'ILLEGAL DATA ADDRESS',
    3: 'ILLEGAL DATA VALUE',
    4: 'SERVER DEVICE FAILURE',
}


class ModbusException(RigError):
    """The slave understood the request and refused it, with a reason code."""

    def __init__(self, unit, function, code):
        self.unit = unit
        self.function = function
        self.code = code
        super().__init__('unit %d, fc 0x%02X: exception 0x%02X (%s)'
                         % (unit, function, code,
                            EXCEPTION_NAMES.get(code, 'reserved')))


#: What a round trip over the link can raise: the library's own errors and
#: the port or socket underneath them - the guard a caller uses where a
#: quiet board must not stop it. Never a bare Exception: a bug in this
#: library is not a quiet board, and must not read as one.
LINK_FAULTS = (RigError, OSError)
