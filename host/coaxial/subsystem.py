"""Base class for the board's functional subsystems.

A subsystem owns one area of the board and nothing else: the analog front end,
the discrete I/O, the link. It borrows the board's transaction method rather
than holding a transport, so there is exactly one place a request is issued and
exactly one place a reply is validated.
"""


class Subsystem:
    """One group of board commands, sharing a transport. Subclasses add
    the calls; this holds the unit id and the request plumbing."""
    def __init__(self, board):
        self._board = board

    @property
    def board(self):
        return self._board

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
