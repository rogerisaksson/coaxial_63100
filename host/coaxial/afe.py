"""The analog front end switch."""
import functools

from . import protocol
from .errors import DeviceStateError
from .power import named
from .subsystem import Subsystem
from .wire import Reader, pack


def powered(reading):
    """A method that reports a measurement: refused while AFE_ON is off."""
    @functools.wraps(reading)
    def when_powered(self, *args, **kwargs):
        self.board.afe.require()
        return reading(self, *args, **kwargs)
    return when_powered


class Afe(Subsystem):
    """The analog front end switch."""

    def _act(self, action):
        reader = Reader(self.request(protocol.AFE,
                                     pack(('u8', protocol.AFE_ACTIONS[action]))))
        # `users` is why the rail is where it is.
        return {'on': bool(reader.u8()), 'pe15': bool(reader.u8()),
                'users': named(reader.u8())}

    def state(self):
        """Whether the front end is powered, and the PE15 input beside it."""
        return self._act('read')

    def is_on(self):
        return self.state()['on']

    def enable(self):
        return self._act('on')['on']

    def disable(self):
        return self._act('off')['on']

    def toggle(self):
        return self._act('toggle')['on']

    def set(self, on):
        """On or off as `on` says; whether it is on after."""
        return self.enable() if on else self.disable()

    def require(self):
        """Raise unless the front end is powered."""
        if not self.is_on():
            raise DeviceStateError(
                'the analog front end is off, so every channel would read '
                'mid-scale and the NTC would report exactly 25.00 C. '
                'Call board.afe.enable() first.')
