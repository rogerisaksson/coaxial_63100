"""The analog front end switch.

PB2 powers the amplifier chains AND the voltage reference. That second part is
why this is its own subsystem rather than one more GPIO: with it off, every
channel reads exact mid-scale, and the thermistor divider then sits at R25,
which by definition is 25.00 C. The board reports a plausible temperature that
is not a measurement.

So every reader in analog.py that reports a number checks this first, and
scan() refuses on the afe_on flag its own reply already carries. The exception
is burst(), which returns raw codes and nothing else: it is the primitive its
callers gate, so a caller can still sample the front end off on purpose.
"""
from . import protocol
from .errors import DeviceStateError
from .subsystem import Subsystem
from .wire import Reader, pack


class Afe(Subsystem):
    """The analog front end switch. It powers the ADC reference, not
    just the signal path - with it off every channel reads exact
    mid-scale, which is a plausible number and not a measurement."""
    def _act(self, action):
        reader = Reader(self.request(protocol.AFE,
                                     pack(('u8', protocol.AFE_ACTIONS[action]))))
        return {'on': bool(reader.u8()), 'pe15': bool(reader.u8())}

    def state(self):
        """Whether the front end is powered, and the PE15 input beside it.

        PE15 follows this inversely on the assembled board - measured, not
        assumed - which makes it an independent witness that a write landed.
        """
        return self._act('read')

    def is_on(self):
        return self.state()['on']

    def enable(self):
        return self._act('on')['on']

    def disable(self):
        return self._act('off')['on']

    def toggle(self):
        return self._act('toggle')['on']

    def require(self):
        """Raise unless the front end is powered.

        Called before every reading this library reports - read_all,
        ntc_temperature, dcbus_voltage, noise - and by any caller of the raw
        burst primitive. Refusing is better than returning mid-scale codes,
        because mid-scale looks like data.
        """
        if not self.is_on():
            raise DeviceStateError(
                'the analog front end is off, so every channel would read '
                'mid-scale and the NTC would report exactly 25.00 C. '
                'Call board.afe.enable() first.')
