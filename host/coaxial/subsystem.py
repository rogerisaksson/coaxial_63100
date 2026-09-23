"""Base classes for the board's functional subsystems, and what a subsystem
declares about itself.
"""
import functools
from typing import Any, Callable

from .transport import ACK

from . import protocol
from .errors import RigError
from .wire import Reader


class Subsystem:
    """One group of board commands, sharing a transport."""

    def __init__(self, board):
        self._board = board

    @property
    def board(self):
        return self._board

    def _op(self, op, payload=b'', **kwargs):
        """One of this subsystem's ops on the wire - each subsystem's own
        function code and framing.
        """
        raise NotImplementedError

    def __repr__(self):
        """Names itself and its unit."""
        return '<%s of unit %s>' % (type(self).__name__,
                                    getattr(self._board, 'unit', '?'))

    def request(self, function, payload=b'', **kwargs):
        return self._board.request(function, payload, **kwargs)

    def _ack(self, op, payload=b''):
        """One `u8 took` op, stopped on the ack byte."""
        return self.took(self._op(op, payload, reply_shape=ACK))

    @staticmethod
    def took(reply):
        """Raise with the board's own reason when it refused."""

        r = Reader(reply)
        if r.u8():
            return True
        raise RigError(r.string() if r.remaining else
                       'the board refused, and said nothing about why')


class Device(Subsystem):
    """A peripheral behind 0x6E, selected by the device byte it declares."""

    #: The byte 0x6E selects this device by - declared in the class
    #: statement, `class X(Device, device=protocol.DEVICE_X)`.
    DEVICE: int

    def __init_subclass__(cls, device=None, **kwargs):
        super().__init_subclass__(**kwargs)
        if device is not None:
            cls.DEVICE = device
        if not hasattr(cls, 'DEVICE'):
            raise TypeError('%s is a Device and declares no device byte: '
                            'class %s(Device, device=protocol.DEVICE_...)'
                            % (cls.__name__, cls.__name__))

    def _frame(self, op, payload):
        return bytes((self.DEVICE, op)) + bytes(payload)

    def _op(self, op, payload=b'', **kwargs):
        return self.request(protocol.DEVICE, self._frame(op, payload),
                            **kwargs)

    def _broadcast(self, op, payload=b'', **kwargs):
        """The same op to every unit on the wire, answered by none."""
        self._board.broadcast(protocol.DEVICE, self._frame(op, payload),
                              **kwargs)


def _slot(name):
    return '_remembered_' + name


def remembered(fetch: Callable[[Any], Any]):
    """A fetch cached on its instance until `refresh=True`, or forgotten."""
    slot = _slot(fetch.__name__)

    def remembering(self: Any, refresh: bool = False) -> Any:
        if refresh or slot not in vars(self):
            vars(self)[slot] = fetch(self)
        return vars(self)[slot]
    # Not functools.wraps, and annotated: the wrapper adds `refresh`, and a
    # checker reading the wrapped signature through wraps - or treating an
    # untyped decorator as identity - would refuse the keyword.
    remembering.__name__ = fetch.__name__
    remembering.__doc__ = fetch.__doc__
    return remembering


def forget(instance, *names):
    """Drop what the `remembered` methods called `names` hold on `instance`."""
    for name in names:
        vars(instance).pop(_slot(name), None)


def forgetting(*names):
    """A writer after which the `remembered` methods named are stale."""
    def decorate(write):
        @functools.wraps(write)
        def writing(self, *args, **kwargs):
            forget(self, *names)
            return write(self, *args, **kwargs)
        return writing
    return decorate
