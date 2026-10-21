"""The board's time on this host: the PC's clocks, or an emulated board's virtual seconds.

One emulated at a 28th of real time is stamped, timed out and waited on at its own pace
(tools.emu), so a record 20 ms apart on the board is 20 ms apart here."""
import time


class HostClock:

    """`now()` a `time.time()` value, `perf()` a `perf_counter()` one, `sleep(s)` s of the
    board's: the PC's clocks, or `virtual` - the board's seconds - offset to read like them."""

    #: How often the board's time is read, wall s; between, it runs on at the scale.
    READ_S = 0.05

    def __init__(self, virtual=None, scale=None):
        self._virtual = virtual
        self._scale = scale or (lambda: 1.0)
        self._pace = 1.0
        if virtual is not None:
            at = virtual()
            self._wall_at, self._perf_at = time.time() - at, time.perf_counter() - at
            self._read, self._read_at = at, time.monotonic()
            self._last = at

    @property
    def virtual(self):
        """Whether this is an emulated board's time, not the PC's."""
        return self._virtual is not None

    def _sync(self):
        now, mono = self._virtual(), time.monotonic()
        if mono - self._read_at > 0.005 and now > self._read:
            self._pace = min(100.0, (mono - self._read_at) / (now - self._read))
        self._read, self._read_at = now, mono
        self._last = max(self._last, now)
        return self._last

    def _board(self):
        """The board's seconds: read every READ_S, carried between at the scale, never back."""
        if self._virtual is None:
            return 0.0
        gone = time.monotonic() - self._read_at
        if gone > self.READ_S:
            return self._sync()
        self._last = max(self._last, self._read + gone / max(1.0, self._scale(), self._pace))
        return self._last

    def now(self):
        return time.time() if self._virtual is None else self._wall_at + self._board()

    def perf(self):
        return time.perf_counter() if self._virtual is None else self._perf_at + self._board()

    def sleep(self, seconds):
        """`seconds` of the board's: the PC's at the scale; an emulated board's read as it
        goes, its pace between two reads being a guess - CI's runner fell behind the scale
        and a chain's settle was cut short."""
        if self._virtual is None:
            time.sleep(seconds * self._scale())
            return
        until = self._sync() + seconds
        while True:
            left = until - self._sync()
            if left <= 0.0:
                return
            time.sleep(min(left * self._scale(), self.READ_S))


class Told:

    """A stand-in's seconds: its law run by what it is told (`paced`) and by no wall, a read of
    the clock `STEP` s on. On the wall's, six processes on two logical cores, a 23 Hz ring was
    read 131 ms apart and one board's two reads came 100-134 % apart; a velocity lane's 2 s
    ended where the wall left it on its 0.7 Hz ring, 1421 rpm on Python 3.12, told 1446."""

    STEP = 0.0002
    virtual = True

    def __init__(self, paced):
        self._paced, self._at = paced, 0.0
        paced(0.0)

    def sleep(self, seconds):
        self._paced(seconds)
        self._at += seconds

    def perf(self):
        self.sleep(self.STEP)
        return self._at

    now = perf


#: The PC's own.
WALL = HostClock()


def clock_of(owner):
    """The clock of whatever `owner` - a rig, a board, a device - talks through: the board's own
    (`host_clock`, a stand-in told its seconds), its transport's, else the PC's (the stand-in,
    a broker)."""
    board = getattr(owner, 'board', owner)
    return (getattr(board, 'host_clock', None)
            or getattr(getattr(board, 'transport', None), 'host_clock', None) or WALL)
