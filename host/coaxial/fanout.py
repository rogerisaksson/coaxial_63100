"""One ring, many readers, each with its own place in it.

The broker owns the link, so it owns the records that come off it. Every
client - another process, another thread, a view and a chat session at
once - reads the SAME ring from its OWN cursor, and none of them takes
records away from the others.

WHAT IS LOST IS COUNTED. A reader that falls behind far enough for the
writer to lap it does lose records, because a ring is finite and the
alternative is to stall the board for the slowest reader in the building.
It is told exactly how many, in the same answer that carries the records
it did get. A silent gap is the one outcome this must not have: 208
dropped records were charged to the board once because a reader replaced
its last result and said nothing.

Sequence numbers are monotonic and never reused, which is what makes
"lapped" answerable at all - an index into the ring cannot tell a reader
that has not moved from one that has been round exactly once.
"""
import threading


class Fanout:

    """A circular buffer of fixed-size records, read by cursor."""

    def __init__(self, stride, capacity):
        self.stride = int(stride)
        self.capacity = max(1, int(capacity))
        self._buf = bytearray(self.stride * self.capacity)
        self._lock = threading.Lock()
        #: Sequence of the NEXT record to be written. Also the count of
        #: everything ever written, which is what a cursor compares to.
        self.head = 0
        #: Records the writer has overwritten before anybody read them.
        #: Not per reader - this is the ring's own high-water damage.
        self.overwritten = 0

    # -- the writer -------------------------------------------------------

    def put(self, blob):
        """Append whole records from `blob`. Returns how many went in."""
        if not blob:
            return 0
        n = len(blob) // self.stride
        with self._lock:
            for i in range(n):
                at = (self.head % self.capacity) * self.stride
                self._buf[at:at + self.stride] = \
                    blob[i * self.stride:(i + 1) * self.stride]
                self.head += 1
            if self.head > self.capacity:
                self.overwritten = max(self.overwritten,
                                       self.head - self.capacity)
        return n

    # -- the readers ------------------------------------------------------

    @property
    def oldest(self):
        """Sequence of the oldest record still in the ring."""
        return max(0, self.head - self.capacity)

    def take(self, cursor, most=0):
        """Records from `cursor` on. (blob, first, lost, next).

        `lost` is what the writer overwrote between `cursor` and what is
        still here - the reader was too slow and the ring is finite. It is
        returned rather than hidden, and `first` says where the blob
        actually starts, so a caller can put a hole in its own record
        rather than a silent join.
        """
        with self._lock:
            head, oldest = self.head, max(0, self.head - self.capacity)
            start = max(int(cursor), oldest)
            lost = max(0, oldest - int(cursor))
            available = head - start
            if most > 0:
                available = min(available, int(most))
            if available <= 0:
                return b'', start, lost, start
            out = bytearray(available * self.stride)
            for i in range(available):
                at = ((start + i) % self.capacity) * self.stride
                out[i * self.stride:(i + 1) * self.stride] = \
                    self._buf[at:at + self.stride]
            return bytes(out), start, lost, start + available

    def state(self):
        """What the ring holds, for anyone asking about it."""
        with self._lock:
            return {'stride': self.stride, 'capacity': self.capacity,
                    'head': self.head, 'oldest': max(0, self.head - self.capacity),
                    'held': min(self.head, self.capacity),
                    'overwritten': self.overwritten}
