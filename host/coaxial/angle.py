"""The A1335 on SPI4: what it reads, and what the poll loop saw."""
from . import protocol
from .protocol import AngleOp
from .sensor import PolledSensor
from .subsystem import Device
from .wire import Reader, pack

#: The registers the reference library names. Six address bits, so 0x00-0x3F.
CTRL = 0x1E
ANG = 0x20
STA = 0x22
ERR = 0x24
XERR = 0x26
TSEN = 0x28
FIELD = 0x2A

REGISTERS = {
    CTRL: 'CTRL', ANG: 'ANG', STA: 'STA', ERR: 'ERR',
    XERR: 'XERR', TSEN: 'TSEN', FIELD: 'FIELD',
}

ADDRESS_BITS = 6
"""A register address is six bits wide."""

LOOP_STATES = {0: 'off', 1: 'running', 2: 'held'}
"""What the board's angle poll loop is doing. 'off' means AFE_ON is low."""

LOOP_ERRORS = {
    0: 'none', 1: 'lost AFE_ON', 2: 'SPI4 would not configure',
    3: 'the transfer failed', 4: 'all ones - absent or unpowered',
}

DATA_BITS = 12
"""The twelve data bits of any of these registers; the four above are flags."""

COUNTS = 1 << DATA_BITS
"""Angle codes in a full turn: twelve bits, and the datasheet's stated
resolution. 360/4096 is 0.0879 degrees a count."""

TSEN_COUNTS_PER_KELVIN = 8
"""TSEN counts eighths of a kelvin."""


def degrees(value):
    """`value`'s low twelve bits as degrees, 0 to 360."""
    return counts(value) * 360.0 / COUNTS


def kelvin(value):
    """TSEN's low twelve bits as kelvin: the count is eighths of one."""
    return counts(value) / TSEN_COUNTS_PER_KELVIN


def counts(value):
    """The twelve data bits of any of these registers."""
    return value & (COUNTS - 1)


def gauss(value):
    """FIELD's low twelve bits. The count IS the gauss."""
    return counts(value)


def _name(register):
    return REGISTERS.get(register, '0x%02X' % register)


def _address(register):
    """`register` if it fits the six address bits, or a raise."""
    if not 0 <= register < 1 << ADDRESS_BITS:
        raise ValueError('register %r is past the six address bits'
                         % (register,))
    return register


def _angle(value):
    return {'degrees': degrees(value), 'flags': value >> DATA_BITS}


def _temperature(value):
    return {'kelvin': kelvin(value)}


def _nothing(value):
    return {}


#: What a register's value means beyond its counts, by register.
DECODED = {ANG: _angle, TSEN: _temperature}


class Angle(Device, PolledSensor, device=protocol.DEVICE_ANGLE):
    """The A1335 behind SPI4."""

    LOOP_STATES = LOOP_STATES

    def state(self):
        """The poll loop's shared record, and the reading in it."""
        r = Reader(self._op(AngleOp.LATEST))
        got = {
            'loop': LOOP_STATES.get(r.u8(), 'unknown'),
            'error': LOOP_ERRORS.get(r.u8(), 'unknown'),
            'updates': r.u32(),
            'errors': r.u32(),
        }
        have = bool(r.u8())
        register = r.u8()
        value = r.u16()
        crc = r.u8()

        got.update({
            'register': register,
            'register_name': _name(register),
            'value': value if have else None,
            'crc': crc,
        })
        if have:
            got.update(DECODED.get(register, _nothing)(value))
        return got

    def read(self, register):
        """One register: its sixteen data bits and its four CRC bits."""
        r = Reader(self._op(AngleOp.READ, pack(('u8', _address(register)))))
        got = r.u8()
        return {'register': got,
                'register_name': _name(got),
                'value': r.u16(), 'crc': r.u8()}

    def write(self, register, value):
        """Eight data bits into one register."""
        self._op(AngleOp.WRITE,
                 pack(('u8', _address(register)), ('u8', value)))

    def poll_register(self, register=None):
        """Which register the loop reads, asked or set."""
        payload = b'' if register is None else pack(('u8', register))
        got = Reader(self._op(AngleOp.POLLREG, payload)).u8()
        return {'register': got, 'register_name': _name(got)}

    def clock(self):
        """SPI4's kernel clock and the bitrate derived from it, in hertz."""
        r = Reader(self._op(AngleOp.CLOCK))
        return {'kernel_hz': r.u32(), 'bitrate_hz': r.u32()}

    def hold(self):
        """Stop the poll loop so the part can be read or written directly."""
        return self._loop_state(self._op(AngleOp.HOLD))

    def resume(self):
        """Start the poll loop again."""
        return self._loop_state(self._op(AngleOp.RESUME))
