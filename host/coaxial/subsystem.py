"""Base classes for the board's functional subsystems, and what a subsystem
declares about itself.

A subsystem owns one area of the board and nothing else: the analog front end,
the discrete I/O, the link. It borrows the board's transaction method rather
than holding a transport, so there is exactly one place a request is issued and
exactly one place a reply is validated.

The declaration is the class statement. A peripheral behind 0x6E says which
device byte selects it:

    class Thermal(Device, device=protocol.DEVICE_THERMAL):

and `Device._op` frames every op from that byte, once - ten subclasses each
carried the same three lines, and the byte in them was the only difference.
A subsystem on bare function codes - the AFE, the pins - is a `Subsystem`.
Which attribute of the board a subsystem answers at is the board's
declaration and not the subsystem's: `Board.parts()`.

Two decorators for what a method wants of the subsystem's own state:
`remembered` for a fetch of something the board describes about itself and
does not change at run time, cached until told to `refresh`; `forgetting`
for a writer after which such a cache is wrong. What a method wants of the
BOARD's state - the front end powered - is `afe.powered`, kept with the
switch it reads.
"""
import functools
from typing import Any, Callable

from . import protocol


class Subsystem:
    """One group of board commands, sharing a transport. Subclasses add
    the calls; this holds the unit id and the request plumbing."""

    def __init__(self, board):
        self._board = board

    @property
    def board(self):
        return self._board

    def _op(self, op, payload=b'', **kwargs):
        """One of this subsystem's ops on the wire - each subsystem's own
        function code and framing."""
        raise NotImplementedError

    def __repr__(self):
        """Names itself and its unit. NO I/O - a repr that talks to the
        board raises from inside a debugger, which is where it is read."""
        return '<%s of unit %s>' % (type(self).__name__,
                                    getattr(self._board, 'unit', '?'))

    def request(self, function, payload=b'', **kwargs):
        return self._board.request(function, payload, **kwargs)

    def _ack(self, op, payload=b''):
        """One `u8 took` op, stopped on the ack byte.

        The reply's length is knowable from its first payload byte - `1`
        alone, or `0` and the length-prefixed refusal - so the transport
        stops on the last byte instead of waiting out QUIET_TIME. That
        wait was 8 ms of the ~15 ms every write-class transaction cost.
        """
        from .transport import ACK
        return self.took(self._op(op, payload, reply_shape=ACK))

    @staticmethod
    def took(reply):
        """Raise with the board's own reason when it refused.

        Every refusal comes back as `u8 took` and, when it did not, the
        board's words for what is wrong and what to do. The board is the
        only thing that knows which check failed - a host listing possible
        causes is a second answer that goes stale the moment a check moves.
        """
        from .errors import RigError
        from .wire import Reader

        r = Reader(reply)
        if r.u8():
            return True
        raise RigError(r.string() if r.remaining else
                       'the board refused, and said nothing about why')


class Device(Subsystem):
    """A peripheral behind 0x6E, selected by the device byte it declares.

    One function code carries every peripheral because the user-defined
    ranges are spent (protocol.py); the op is the byte after the device.
    """

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
    """A fetch cached on its instance until `refresh=True`, or forgotten.

    For what the board describes about itself and does not change at run
    time - the channel table, the map, the calibration record. Each was a
    hand-rolled `if self._x is None or refresh:`, and one writer had
    forgotten to drop one (`Calibration.defaults()`, found when this
    replaced them). `forget(instance, 'read')` drops the cache by the
    method's name; `forgetting` is how a writer says it does.
    """
    slot = _slot(fetch.__name__)

    def remembering(self: Any, refresh: bool = False) -> Any:
        if refresh or slot not in vars(self):
            vars(self)[slot] = fetch(self)
        return vars(self)[slot]
    # Not functools.wraps, and annotated: the wrapper adds `refresh`, and
    # a checker reading the wrapped signature through wraps - or treating
    # an untyped decorator as identity - would refuse the keyword.
    remembering.__name__ = fetch.__name__
    remembering.__doc__ = fetch.__doc__
    return remembering


def forget(instance, *names):
    """Drop what the `remembered` methods called `names` hold on `instance`."""
    for name in names:
        vars(instance).pop(_slot(name), None)


def forgetting(*names):
    """A writer after which the `remembered` methods named are stale.

    Before the write rather than after: a write the board refuses leaves
    the cache dropped and the next read fetches, which costs a round trip
    and can never hand back a record the board no longer holds.
    """
    def decorate(write):
        @functools.wraps(write)
        def writing(self, *args, **kwargs):
            forget(self, *names)
            return write(self, *args, **kwargs)
        return writing
    return decorate
