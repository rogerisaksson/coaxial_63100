"""Raw pin access, for a production test fixture.

Driving pins directly is exactly what a fixture needs and exactly what can
damage a board, so the firmware keeps it behind a gate. Reads are always
allowed; anything that reconfigures or drives a pin needs test_mode(True).

Two classes of pin are refused in every mode, and this module explains why
before the firmware has to: USART3 on PB10/PB11 carries the very command being
issued, and PA13-PA15/PB3/PB4 are the debug port. Losing either costs more than
the measurement is worth.
"""
from . import protocol
from .subsystem import Subsystem
from .wire import Reader, pack


def _port_byte(port):
    letter = str(port).upper()[:1]
    if not 'A' <= letter <= 'K':
        raise ValueError('port %r is not one of A..K' % (port,))
    return ord(letter)


def reserved_reason(port, pin):
    """Why this pin is refused, or None if it is available."""
    return protocol.RESERVED_PINS.get((str(port).upper()[:1], int(pin)))


class Gpio(Subsystem):

    def test_mode(self, enable):
        """Open or close the gate. Returns the state the firmware reports.

        The key is required so the mode cannot be entered by a stray frame or a
        mistyped command; a wrong key leaves the gate exactly as it was.
        """
        reader = Reader(self.request(
            protocol.TEST_GATE,
            pack(('u32', protocol.TEST_GATE_KEY), ('u8', 1 if enable else 0))))
        return bool(reader.u8())

    def _guard(self, port, pin):
        reason = reserved_reason(port, pin)
        if reason is not None:
            raise ValueError('P%s%d is %s and is refused in every mode; driving '
                             'it would cost the link or the debug port'
                             % (str(port).upper()[:1], pin, reason))

    def pin_mode(self, port, pin, mode, pull='none'):
        """Configure one pin. Needs the gate open."""
        self._guard(port, pin)
        if mode not in protocol.PIN_MODES:
            raise ValueError('mode %r is not one of %s'
                             % (mode, ', '.join(sorted(protocol.PIN_MODES))))
        if pull not in protocol.PIN_PULLS:
            raise ValueError('pull %r is not one of %s'
                             % (pull, ', '.join(sorted(protocol.PIN_PULLS))))
        self.request(protocol.PIN_MODE,
                     pack(('u8', _port_byte(port)), ('u8', pin),
                          ('u8', protocol.PIN_MODES[mode]),
                          ('u8', protocol.PIN_PULLS[pull])))

    def pin_read(self, port, pin):
        """Read one pin. Allowed with the gate shut."""
        self._guard(port, pin)
        reader = Reader(self.request(protocol.PIN_READ,
                                     pack(('u8', _port_byte(port)), ('u8', pin))))
        return bool(reader.u8())

    def pin_write(self, port, pin, level):
        """Drive one pin and return the level READ BACK from it.

        Read back rather than echoed: on an open-drain output, or a pin the
        fixture is holding, those differ, and that difference is the whole
        reason a rig drives a pin in the first place.
        """
        self._guard(port, pin)
        reader = Reader(self.request(
            protocol.PIN_WRITE,
            pack(('u8', _port_byte(port)), ('u8', pin), ('u8', 1 if level else 0))))
        return bool(reader.u8())

    def port_read(self, port):
        """The whole input register of one port, as 16 bits."""
        reader = Reader(self.request(protocol.PORT_READ,
                                     pack(('u8', _port_byte(port)))))
        return reader.u16()

    def port_write(self, port, mask, value):
        """Drive a masked set of pins atomically, through BSRR.

        Reserved bits are masked out by the firmware rather than the write being
        rejected, so a fixture driving a bank of outputs need not know which
        bits this board keeps for itself. Returns the input register afterwards.
        """
        reader = Reader(self.request(
            protocol.PORT_WRITE,
            pack(('u8', _port_byte(port)), ('u16', mask), ('u16', value))))
        return reader.u16()
