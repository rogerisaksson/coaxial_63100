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

    def request(self, function, payload=b'', **kwargs):
        return self._board.request(function, payload, **kwargs)
