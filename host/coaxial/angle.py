"""The A1335 on SPI4: what it reads, and what the poll loop saw.

The board clocks 20-bit packets and does not interpret them. This is where a
register's sixteen bits become an angle, the same division the ADC channels
and the IMU keep: the firmware reports raw counts, and the scaling belongs
here.

Register addresses come from the reference library at
github.com/ScranchNew/Allegro-A1335-Sensor-library, not from
datasheets/angle_sensor - that datasheet defers the register map to the
Programming Manual, which is not in this tree.
"""
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
    """`value`'s low twelve bits as degrees, 0 to 360.

    The high four bits are flags, not part of the angle - masking them off
    is what makes an error flag read as an error rather than as half a turn.
    """
    return counts(value) * 360.0 / COUNTS


def kelvin(value):
    """TSEN's low twelve bits as kelvin: the count is eighths of one."""
    return counts(value) / TSEN_COUNTS_PER_KELVIN


def counts(value):
    """The twelve data bits of any of these registers.

    Named because four call sites were spelling `value & 0x0FFF` out, and
    two of them went on to re-derive degrees and kelvin from it with their
    own literals - invariant 7, two copies of one conversion.
    """
    return value & (COUNTS - 1)


def gauss(value):
    """FIELD's low twelve bits. The count IS the gauss.

    The datasheet in this tree has no register map - it refers to the A1335
    programming guide - so this comes from the reference implementation, like
    the addresses above. Measured on this board with nothing mounted: 2.
    The recommended operating range is 300 to 1000 G (datasheet, Field
    Strength), so a reading in the low tens means no magnet.
    """
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
    """The A1335 behind SPI4. Every call raises rather than returning a
    status: a reading that did not happen is not a reading of zero."""

    LOOP_STATES = LOOP_STATES

    def state(self):
        """The poll loop's shared record, and the reading in it.

        The board polls from its own main loop and writes here; a host only
        ever reads. `updates` is monotonic, so the same reading read twice is
        telling rather than a guess from the value.
        """
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
        """One register: its sixteen data bits and its four CRC bits.

        The CRC comes back unchecked - the datasheet in this tree gives the
        field's width and not its polynomial, so checking it against a
        guessed one would reject good readings.
        """
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
        """Which register the loop reads, asked or set.

        Settable because the register map came from a reference
        implementation rather than from the datasheet in this tree.
        """
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
