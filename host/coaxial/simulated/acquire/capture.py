"""The stand-in's capture ring: the AFE, the pins and both SPI parts in one buffer."""
import random

from coaxial.acquire.capture import Ring
from coaxial.simulated.values import MASK32


class SimulatedCapture(Ring):
    """The measurement ring, without measurements."""

    DEPTH = 1024

    def __init__(self):
        self._mask = 0
        self._pending = []
        self._seq = [0, 0, 0]
        self._at = 0
        self._dropped = 0

    def _fill(self):
        for src in (0, 1, 2):
            if not self._mask >> src & 1:
                continue
            for _ in range(4):
                self._at += random.randint(9000, 10000)
                v = {0: (1400 + random.randint(-60, 60),
                         -9020 + random.randint(-60, 60),
                         -650 + random.randint(-60, 60), 1385),
                     1: (24442, 8, 32, 0),
                     2: (random.randint(-16384, 16384),) * 4}[src]
                self._pending.append({'at': self._at & MASK32,
                                      'source': ('phases', 'angle', 'imu')[src],
                                      'seq': self._seq[src] & 0xFF,
                                      'v': tuple(v)})
                self._seq[src] += 1

    def state(self):
        names = ('phases', 'angle', 'imu')
        return {'sources': [names[i] for i in range(3) if self._mask >> i & 1],
                'mask': self._mask, 'count': len(self._pending),
                'depth': self.DEPTH, 'dropped': self._dropped,
                # Never thinned here; the field is the board's (test_parity).
                'thinned': 0}

    def _arm(self, mask):
        self._mask = mask
        self._pending = []
        self._seq = [0, 0, 0]
        self._dropped = 0
        return True

    def take(self, want=15):
        want = max(1, min(int(want), 15))
        if self._mask and len(self._pending) < want:
            self._fill()
        batch, self._pending = self._pending[:want], self._pending[want:]
        return batch

