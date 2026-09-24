"""The stand-in's cycle counter, tied to the host's clock as the board ties it to UTC."""
import time

from coaxial.acquire.clock import Timebase
from coaxial.simulated.values import SYSCLK_HZ


class SimulatedClock(Timebase):
    """The cycle counter tied to this machine's clock, 12 ppm off: against UTC it
    shows this machine's error plus its own."""

    NOMINAL_HZ = SYSCLK_HZ
    SKEW = 1 - 12e-6

    def __init__(self):
        self._seq = 0
        self._latched = 0
        self._t0 = None

    def _cycles(self):
        if self._t0 is None:
            self._t0 = time.time()
        return int((time.time() - self._t0) * self.NOMINAL_HZ
                   * self.SKEW) % (1 << 32)

    def trigger(self, settle=0.05):
        self._latched = self._cycles()
        self._seq += 1

    def read(self, count=None, timeout=None):
        return {'seq': self._seq, 'latched': self._latched,
                'now': self._cycles(), 'sysclk_hz': self.NOMINAL_HZ}

    def _bracket(self):
        """One latch, bracketed - on `perf_counter`, as the real one is."""
        before = time.perf_counter()
        self.trigger()
        after = time.perf_counter()
        return (before + after) / 2.0, after - before
