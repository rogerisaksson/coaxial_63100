"""One lazily-opened board connection, shared by every tool call.

Connecting is not a tool. A model driving a production rig should not have to
spend a turn on it, and a forgotten connect is a whole wasted round trip - so
the first tool call that needs the board opens it, and it stays open.

The board boots into its ASCII console, so opening means handing the UART over
to the binary protocol first. On the way out the console is handed back, which
matters: a board left in binary mode looks dead to anyone with a terminal.
"""
import sys

sys.path.insert(0, __file__.rsplit('coaxial_mcp', 1)[0])

from coaxial import connect, disconnect          # noqa: E402
from coaxial.errors import RigError              # noqa: E402


class Session:
    def __init__(self, port='COM4', baud=115200, unit=1):
        self.port = port
        self.baud = baud
        self.unit = unit
        self._board = None
        self._info = None

    @property
    def board(self):
        """The connected board, opening the link on first use."""
        if self._board is None:
            self._board = connect([(self.unit, self.baud, self.port)])[0]
        return self._board

    def info(self, refresh=False):
        """Version, clock and channel table, cached: none of it changes at run
        time, and re-reading it on every call is exactly the waste this server
        exists to avoid."""
        if self._info is None or refresh:
            board = self.board
            self._info = (board.version_info or board.probe(),
                          board.system.clock(),
                          board.analog.channels(refresh=refresh))
        return self._info

    def close(self):
        if self._board is None:
            return
        try:
            disconnect([self._board])
        except RigError:
            pass
        finally:
            self._board = None
            self._info = None

    def reset(self):
        """Drop the connection so the next call reopens it. For recovering from
        a cable pull without restarting the server."""
        self.close()
