"""Exception hierarchy for the coaxial_63100 host library.

Nothing in this library returns a status code, and nothing returns None to mean
failure. Every call either produces its result or raises. That is what keeps
call sites flat: a test script is a list of measurements, not a list of checks.

The classes are specific because a production rig needs to tell the difference
between "the cable is out" and "the board refused". Catch RigError to catch
everything.
"""


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
    """The slave speaks a protocol major this host has no codec for.

    Raised rather than guessed at. A host that decodes a layout it does not
    understand produces numbers that look plausible and are wrong, which in a
    production rig is worse than not talking to the board at all.
    """


class DeviceStateError(RigError):
    """The board is in a state where the request cannot mean anything.

    Raised by, for instance, an analog read attempted with the front end
    unpowered - where every channel would read mid-scale and the thermistor
    would report exactly 25.00 C.
    """


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
