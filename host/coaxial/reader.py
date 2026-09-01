"""A host-side reader that keeps the link busy while the caller is not.

THE LINK IS ONE THREAD'S. Between `start()` and `stop()` this thread is
the only one that touches the transport - the consumer reads a deque that
this fills, and its `print()`, its terminal, its plotting never sit
between two round trips.

The measurement that made it exist: a `for block in daq.blocks(20)` loop
that prints costs one round trip (~25 ms of fixed latency on the debug
probe's VCP) PLUS whatever the terminal takes, serially, so the link idles
for the terminal's share. The board meanwhile fills its ring at the rate
it was configured for, and the overflow is charged to a slowness that was
never the board's. Threaded, the round trips happen while the consumer
works: pyserial's read and write release the GIL, so this is real overlap
and not a scheduler trick.
"""
import collections
import threading
import time

from .errors import CrcError, NoReplyError, RigError

#: Blocks the host keeps between the reader and the consumer. Past this
#: the OLDEST goes, and the drop is counted rather than hidden: a reader
#: that silently replaced its last result is how 208 dropped records were
#: once charged to the board.
#:
#: A block is a read's worth - four records at ten channels and the pins -
#: so this is thousands of records and a few megabytes of dicts. Cheap on
#: the machine at this end of the link, and the whole point of buffering
#: here: a consumer that stops to plot must not cost the board a record.
HOST_BLOCKS = 4096

#: Consecutive unanswered reads that still count as a busy link rather
#: than a gone one. Measured at about one transaction in fifty while the
#: board is busy, so a run of these is the signal, not one.
MISSES_ALLOWED = 20


class BufferedReader:

    """Drains the board into a host-side deque on its own thread.

    `acquire` is the only link call it makes - the read answers its own
    backlog, so pacing costs no extra round trip. Nothing else may
    touch the transport while it runs.
    """

    def __init__(self, acquire, backlog=None, idle=0.005, batch=1,
                 patience=4):
        self._acquire, self._backlog_of = acquire, backlog
        self._idle = idle
        #: Records a reply can carry. A transaction costs the same whether
        #: it brings one record or a full reply, so reading the instant a
        #: single record appears spends the whole cost on 55 bytes.
        #: MEASURED: eager reads ran 95 a second at 1.0 records each and
        #: took 56% of the line; waiting for a full reply is worth more
        #: than the milliseconds it costs.
        self._batch = max(1, int(batch))
        #: How many idle slices to spend waiting for one. The bound is
        #: what keeps this a throughput trade and not a latency leak:
        #: patience * idle is the worst a record waits that could have
        #: gone earlier.
        self._patience = max(0, int(patience))
        self._blocks = collections.deque()
        self._thread = None
        self._stop = threading.Event()
        self.error = None
        self.dropped = 0
        self.peak = 0
        #: What the board still held after the last read, off that same
        #: reply. This is the pacing signal: a backlog means read again
        #: NOW, and nothing means the ring is dry and a spin would just
        #: ask an empty ring the same question at the link's expense.
        self.backlog = None
        #: What the last reply said was left, read by the loop before it
        #: decides to wait. Kept apart from `backlog` only because that
        #: one is the public reading.
        self._backlog = None
        self.reads = 0
        #: Records taken, not reads: a read carries as many as fit, so
        #: this is what a byte rate has to be worked out from.
        self.records = 0

    # -- lifecycle -------------------------------------------------------

    def start(self):
        """Take the link. Idempotent."""
        if self._thread is not None:
            return self
        self._stop.clear()
        self.error = None
        self._thread = threading.Thread(target=self._run, name='daq-reader',
                                        daemon=True)
        self._thread.start()
        return self

    def stop(self, timeout=5.0):
        """Give the link back, and do not return until it is given back.

        The join is the point: the caller's next act is usually another
        round trip, and two threads on one serial transport is the one
        thing this arrangement must not do.
        """
        if self._thread is None:
            return self
        self._stop.set()
        self._thread.join(timeout)
        self._thread = None
        return self

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    # -- what the consumer sees -----------------------------------------

    def __len__(self):
        return len(self._blocks)

    def take(self):
        """The oldest block, or None. Never waits."""
        try:
            return self._blocks.popleft()
        except IndexError:
            return None

    def drain(self):
        """Every block queued now, oldest first, as one list."""
        out = []
        while True:
            block = self.take()
            if block is None:
                return out
            out.append(block)

    def raise_if_failed(self):
        """The thread's exception, on the caller's thread."""
        if self.error is not None:
            raise self.error

    # -- the thread ------------------------------------------------------

    def _keep(self, block):
        if len(self._blocks) >= HOST_BLOCKS:
            self._blocks.popleft()
            self.dropped += 1
        self._blocks.append(block)
        self.peak = max(self.peak, len(self._blocks))

    def _run(self):
        misses, waited = 0, 0
        while not self._stop.is_set():
            # A PARTIAL RING IS WORTH WAITING FOR, up to a bound. The
            # backlog came free with the last reply, so this costs no
            # round trip to decide.
            if (self._backlog is not None and 0 < self._backlog < self._batch
                    and waited < self._patience):
                waited += 1
                time.sleep(self._idle)
                continue
            waited = 0
            try:
                block = self._acquire()
            except (NoReplyError, CrcError) as exc:
                misses += 1
                if misses > MISSES_ALLOWED:
                    self.error = RigError(
                        '%d replies in a row went missing, so the link is '
                        'gone rather than busy: %s' % (misses, exc))
                    return
                time.sleep(0.01)
                continue
            except Exception as exc:              # noqa: BLE001 - re-raised
                self.error = exc
                return
            misses = 0
            self.reads += 1
            if self._backlog_of is not None:
                self.backlog = self._backlog = self._backlog_of()
            if block:
                self.records += len(block)
                self._keep(block)
            # PACED BY THE BOARD, NOT BY A CLOCK. While records are still
            # queued on the target the next read goes out with no wait, so
            # the link runs at its practical maximum exactly when there is
            # something to carry. Empty, it idles instead of spinning.
            if not self.backlog and not block:
                time.sleep(self._idle)
