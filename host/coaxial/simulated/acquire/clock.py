"""The stand-in's cycle counter, tied to the host's clock as the board ties it to UTC."""
import time
from typing import cast

from coaxial.acquire.clock import Clock, NTP_SERVER
from coaxial.simulated.values import SYSCLK_HZ


class SimulatedClock:
    """The cycle counter tied to nothing, but tied consistently."""

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

    def latch(self):
        self._latched = self._cycles()
        self._seq += 1

    def read_latch(self):
        return {'seq': self._seq, 'latched': self._latched,
                'now': self._cycles(), 'sysclk_hz': self.NOMINAL_HZ}

    def probe(self, rounds=16):
        return Clock.probe(cast(Clock, self), rounds=rounds)
    def sync(self, seconds=2.0, rounds=8, reference='utc', ntp_server=None):
        # Its cycles come off this machine's clock, so against UTC it is this
        # machine's error plus its own 12 ppm - which is the honest answer, not
        # a bug.
        return Clock.sync(cast(Clock, self), seconds=seconds, rounds=rounds,
                          reference=reference,
                          ntp_server=ntp_server or NTP_SERVER)

    def _bracket(self):
        """One latch, bracketed - on `perf_counter`, as the real one is."""
        before = time.perf_counter()
        self.latch()
        after = time.perf_counter()
        return (before + after) / 2.0, after - before
