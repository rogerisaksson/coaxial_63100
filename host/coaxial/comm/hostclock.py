"""The board's time on this host: the PC's clocks, or an emulated board's virtual seconds.

One emulated at a 28th of real time is stamped, timed out and waited on at its own pace
(tools.emu), so a record 20 ms apart on the board is 20 ms apart here."""
import time


class HostClock:

    """`now()` a `time.time()` value, `perf()` a `perf_counter()` one, `sleep(s)` s of the
    board's: the PC's clocks, or `virtual` - the board's seconds - offset to read like them."""

    #: How often the board's time is read, wall s; between, it runs on at the scale.
    READ_S = 0.2

    def __init__(self, virtual=None, scale=None):
        self._virtual = virtual
        self._scale = scale or (lambda: 1.0)
        if virtual is not None:
            at = virtual()
            self._wall_at, self._perf_at = time.time() - at, time.perf_counter() - at
            self._read, self._read_at = at, time.monotonic()
            self._last = at

    @property
    def virtual(self):
        """Whether this is an emulated board's time, not the PC's."""
        return self._virtual is not None

    def _board(self):
        """The board's seconds: read every READ_S, carried between at the scale, never back."""
        virtual = self._virtual
        if virtual is None:
            return 0.0
        gone = time.monotonic() - self._read_at
        if gone > self.READ_S:
            self._read, self._read_at = virtual(), time.monotonic()
            gone = 0.0
        self._last = max(self._last, self._read + gone / max(1.0, self._scale()))
        return self._last

    def now(self):
        return time.time() if self._virtual is None else self._wall_at + self._board()

    def perf(self):
        return time.perf_counter() if self._virtual is None else self._perf_at + self._board()

    def sleep(self, seconds):
        time.sleep(seconds * self._scale())


#: The PC's own.
WALL = HostClock()


def clock_of(owner):
    """The clock of whatever `owner` - a rig, a board, a device - talks through; the PC's where
    it names none (the stand-in, a broker)."""
    board = getattr(owner, 'board', owner)
    return getattr(getattr(board, 'transport', None), 'clock', None) or WALL
