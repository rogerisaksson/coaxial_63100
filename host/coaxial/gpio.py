"""Raw pin access, for a production test fixture."""
from . import protocol
from .errors import RigError
from .subsystem import Subsystem, remembered
from .wire import Reader, pack


def _letter(port):
    return str(port).upper()[:1]


def _port_byte(port):
    letter = _letter(port)
    if not 'A' <= letter <= 'K':
        raise ValueError('port %r is not one of A..K' % (port,))
    return ord(letter)


def _pin_name(port, pin):
    """'PB10', the way the board names a pin."""
    return 'P%s%d' % (_letter(port), int(pin))


def reserved_reason(port, pin):
    """Why this pin is refused, or None if it is available."""
    return protocol.RESERVED_PINS.get((_letter(port), int(pin)))


class Gpio(Subsystem):

    """The digital pins a fixture may read or drive."""

    def test_mode(self, enable):
        """Open or close the gate. Returns the state the firmware reports."""
        reader = Reader(self.request(
            protocol.TEST_GATE,
            pack(('u32', protocol.TEST_GATE_KEY), ('u8', int(bool(enable))))))
        return bool(reader.u8())

    @remembered
    def _reserved(self):
        """The board's own reserved-pin table, or None on a board older than
        protocol 1.3 - asked once, so an old board is not asked per pin.
        """
        try:
            return self._board.system.channel_map()['reserved']
        except RigError:
            return None

    def _refusal(self, port, pin):
        """Why this pin is refused, asked of the board that owns the answer."""
        reserved = self._reserved()
        if reserved is None:
            return reserved_reason(port, pin)
        want = _pin_name(port, pin)
        return next((row['signal'] for row in reserved
                     if row['pin'].upper() == want), None)

    def _guard(self, port, pin):
        reason = self._refusal(port, pin)
        if reason is not None:
            raise ValueError('%s is %s and is refused in every mode; driving '
                             'it would cost the link or the debug port'
                             % (_pin_name(port, pin), reason))

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
        """Drive one pin and return the level READ BACK from it."""
        self._guard(port, pin)
        reader = Reader(self.request(
            protocol.PIN_WRITE,
            pack(('u8', _port_byte(port)), ('u8', pin),
                 ('u8', int(bool(level))))))
        return bool(reader.u8())

    def port_read(self, port):
        """The whole input register of one port, as 16 bits."""
        reader = Reader(self.request(protocol.PORT_READ,
                                     pack(('u8', _port_byte(port)))))
        return reader.u16()

    def port_write(self, port, mask, value):
        """Drive a masked set of pins atomically, through BSRR."""
        reader = Reader(self.request(
            protocol.PORT_WRITE,
            pack(('u8', _port_byte(port)), ('u16', mask), ('u16', value))))
        return reader.u16()
