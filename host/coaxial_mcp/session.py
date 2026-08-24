"""One lazily-opened board connection, shared by every tool call.

Connecting is not a tool. A model driving a production rig should not have to
spend a turn on it, and a forgotten connect is a whole wasted round trip - so
the first tool call that needs the board opens it, and it stays open.

The board boots into its ASCII console, so opening means handing the UART over
to the binary protocol first. On the way out the console is handed back, which
matters: a board left in binary mode looks dead to anyone with a terminal.
"""
import os
import sys

# host/ on the path: this file's own directory's parent, so it does
# not matter what the working directory is or what any directory
# along the way is called.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import connect, disconnect          # noqa: E402
from coaxial.errors import RigError              # noqa: E402


def open_session(port='COM4', baud=115200, unit=1, simulated=None):
    """`(session, real)` - the board on `port`, or a stand-in for it.

    `simulated=None` probes and decides: `find_board.probe` makes the same
    Modbus round trip a session's first tool call would, so "a board
    answers here" cannot mean one thing to this factory and another to the
    caller a moment later. `True` skips the probe and takes the stand-in,
    `False` takes the real Session and lets it fail on first use.

    `real` is not decoration. A suite that ran against `SimulatedSession`
    proved the host and nothing about the firmware, and a tally that does
    not say which it was is the plausible-sentence-for-a-fact failure this
    codebase documents everywhere else. Every caller prints it.
    """
    if simulated is None:
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'tools'))
        import find_board
        simulated = not find_board.probe(port, baud, unit)
    if simulated:
        from coaxial.simulated import SimulatedSession
        return SimulatedSession(), False
    return Session(port, baud, unit), True


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
