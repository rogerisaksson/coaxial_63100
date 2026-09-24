"""The rig's task running: the host reader, blocks off the board or the ring, their times."""
import time

from coaxial.acquire.clock import WRAP
from coaxial.acquire.daq import REPLY_ROOM
from coaxial.acquire.reader import BufferedReader
from coaxial.acquire.record import build
from coaxial.errors import CrcError, NoReplyError, RigError


class TaskStream:

    """The task started, read and stopped; every record timed on the board's clock."""

    #: How often an idle read asks whether a finite run has ended.
    #: A round trip, so not on every turn of a 2 ms poll.
    DONE_EVERY = 0.25

    #: Consecutive looks that have to agree the run has ended.
    DONE_LOOKS = 2

    #: The idle read's pause between looks at an empty buffer.
    TAKE_PAUSE = 0.002

    def enable(self):
        """Power the analog front end for this session."""
        if not self._afe_held:
            self.board.afe.on()
            self._afe_held = True
        return self

    def disable(self):
        """Release this session's hold on the front end."""
        if self._afe_held:
            self.board.afe.off()
            self._afe_held = False
        return self

    def capture(self, *channels, **kw):
        """One burst at the loop's full rate, then read it out. Single shot."""
        digital = kw.pop('digital', True)
        records = kw.pop('records', None)
        timeout = kw.pop('timeout', 30.0)
        sample_time = kw.pop('sample_time', 0)
        if kw:
            raise TypeError('capture() got %s' % ', '.join(sorted(kw)))

        # NO CHAIN.
        self.board.daq.shape()
        self.configure(*channels, accumulate=1, interval_us=0,
                       digital=digital, sample_time=sample_time)
        if records is None:
            # The ring's own size at THIS stride, asked rather than worked out
            # here: the board knows what its buffer holds and the arithmetic
            # changes with every field added to a record.
            records = (self.state() or {}).get('capacity') or 1
            self.configure(*channels, accumulate=1, interval_us=0,
                           digital=digital, sample_time=sample_time,
                           records=records)
        else:
            self.configure(*channels, accumulate=1, interval_us=0,
                           digital=digital, sample_time=sample_time,
                           records=int(records))

        self.start()
        got = []
        try:
            # TO EXHAUSTION, not `read(-1)`.
            deadline = time.time() + timeout
            for block in self.read_buffer(-1):
                got.extend(block)
                if time.time() > deadline:
                    break
        finally:
            self.stop()
        return got

    def configure_buffer(self, records):
        """Size the circular buffer the records land in, in RECORDS."""
        self._buffer_records = max(1, int(records))
        return self._buffer_records

    def start(self):
        """Begin sampling into the board's buffer, and into the host's."""
        self.board.daq.start()
        self._last_raw = None
        self._last_stamp = None
        # A reply carries as many records as fit in the board's own reply room,
        # and that is what the reader waits for rather than reading the instant
        # one record lands.
        stride = (self.layout or {}).get('stride') or 0
        take = self._from_broker(stride)
        self._reader = BufferedReader(
            acquire=take or (lambda: self._timed(
                self.board.daq.acquire(layout=self.layout))),
            backlog=lambda: self.board.daq.backlog,
            batch=(REPLY_ROOM // stride) if stride else 1).start()
        return self

    def _from_broker(self, stride):
        """A reader that takes from the broker's ring, or None."""
        wire = self.board.transport
        if not stride or wire is None or wire.stream is None:
            return None
        # This session's unit, so a segment with several nodes fills the ring
        # from the one this rig configured.
        wire.stream(stride, self._buffer_records, self.unit)
        self._cursor = wire.stream_state().get('head', 0)

        def take():
            blob, first, lost, nxt = wire.take(self._cursor)
            self._lost += lost
            self._cursor = nxt
            return self._timed(self.board.daq.decode(blob, self.layout))

        return take

    def stop(self):
        """Stop sampling. What is already buffered stays readable."""
        if self._reader is not None:
            self._reader.stop()
            self._reader = None
        self.board.daq.stop()
        return self

    @property
    def buffered(self):
        """Blocks waiting on the host, and records still on the board."""
        r = self._reader
        if r is None:
            return {'host': 0, 'peak': 0, 'dropped': 0, 'backlog': None,
                    'reads': 0, 'records': 0, 'rate': 0.0,
                    'lost': self._lost, 'cursor': self._cursor}
        # `taken`, not `records`: the reader resets that one to measure its own
        # rate, and a byte rate differentiated off a counter that resets reads
        # as negative throughput.
        return {'host': len(r), 'peak': r.peak, 'dropped': r.dropped,
                'backlog': r.backlog, 'reads': r.reads,
                'records': r.taken, 'rate': r.rate,
                'lost': self._lost, 'cursor': self._cursor}

    def state(self):
        """How the task is doing: rate, what is buffered, what was lost."""
        return self.board.daq.state()

    def acquire(self):
        """One block of records, oldest first, with times on them."""
        # ONE DRAINER.
        if self._reader is not None:
            return self._reader.take() or []

        # `samples` rides in the record: no state() round trip per block.
        return self._timed(self.board.daq.acquire(layout=self.layout))

    #: Consecutive unanswered reads that still count as a busy link.
    MISSES_ALLOWED = 5

    #: The pause after a missed reply, and the one after an empty block.
    RETRY_PAUSE = 0.01

    EMPTY_PAUSE = 0.005

    def read(self, count=-1, timeout=None):
        """Records, waiting for them: `count` (negative: all there is), or what
        came within `timeout` seconds."""
        out = []
        if count == 0:
            return out
        for block in self.read_buffer(-1, timeout):
            out.extend(block)
            if count < 0:
                # EVERYTHING THERE IS, not the first block of it.
                out.extend(self._queued())
                break
            if len(out) >= count:
                break
        # WHAT THERE IS, when a finite run ends first.
        return out[:count] if count > 0 else out

    def _queued(self):
        """Every record the reader has queued, without waiting - and none when
        nothing is buffering.
        """
        reader = self._reader
        blocks = reader.drain() if reader is not None else ()
        return [record for block in blocks for record in block]

    def _ended(self, reader):
        """Whether a finite run has ended AND drained, asked of the board."""
        now = time.time()
        if now - self._asked_done <= self.DONE_EVERY:
            return False
        self._asked_done = now
        state = self.state() or {}
        ended = (state.get('done')
                 and not (state.get('available') or 0)
                 and not len(reader))
        self._done_seen = self._done_seen + 1 if ended else 0
        return self._done_seen >= self.DONE_LOOKS

    def read_buffer(self, count, timeout=None):
        """`count` blocks off the HOST buffer, one at a time, until `timeout`."""
        if self._reader is None:
            raise RigError('nothing is buffering yet - start() puts the '
                           'reader on the link, and read_buffer() takes '
                           'what it has collected')
        seen = 0
        self._done_seen = 0
        until = None if timeout is None else time.time() + timeout
        while (count < 0 or seen < count) and (until is None or time.time() < until):
            self._reader.raise_if_failed()
            block = self._reader.take()
            if block:
                seen += 1
                yield block
                continue
            if not self._reader.running:
                return                    # the link is gone, or stopped
            if self._ended(self._reader):
                return
            time.sleep(self.TAKE_PAUSE)

    def blocks(self, count):
        """`count` non-empty blocks, one at a time, for a `for` loop."""
        if self._reader is not None:
            for block in self.read_buffer(count):
                yield block
            return

        seen, missed = 0, 0
        while count < 0 or seen < count:
            try:
                block = self.acquire()
            except (NoReplyError, CrcError) as exc:
                # A missed reply is a fact of this link, measured at about one
                # transaction in fifty while the board is busy, and a loop of
                # twenty reads meets one more often than not.
                missed += 1
                if missed > self.MISSES_ALLOWED:
                    raise RigError(
                        '%d replies in a row went missing, so the link is '
                        'gone rather than busy: %s'
                        % (missed, exc)) from exc
                time.sleep(self.RETRY_PAUSE)
                continue
            missed = 0
            if not block and count < 0 and self.state()['done']:
                return              # the run ended and the buffer is dry
            if not block:
                time.sleep(self.EMPTY_PAUSE)
                continue
            seen += 1
            yield block

    def latest(self, block=True):
        """The running average since the last time you asked."""
        return self.board.daq.latest(layout=self.layout, block=block)

    def _timed(self, records):
        """Wall-clock time on each record, and each as a `Record`."""
        if not records:
            return records

        stamps = None
        sync = self.sync
        if sync is not None:
            stamps = [sync.to_host(c) for c in self._unwrapped(records, sync)]
            for record, when in zip(records, stamps):
                record['time'] = when

        fields = (self.layout or {}).get('fields') or []
        before, self._last_stamp = self._last_stamp, (stamps[-1] if stamps
                                                      else None)
        return build(records, fields, stamps, before)

    def _unwrapped(self, records, sync):
        """The records' stamps as monotonic cycle counts, across blocks."""
        out = []
        for raw in (r['at'] for r in records):
            self._epoch = self._epoch_of(raw, sync)
            self._last_raw = raw
            out.append(raw + self._epoch)
        return out

    def _epoch_of(self, raw, sync):
        """The wrap this stamp is in: the host clock picks the first, and every
        stamp after follows the last, a wrap added where the count falls.
        """
        if self._last_raw is None:
            expected = (sync.at_cycles
                        + (time.time() - sync.at_host) * sync.hz)
            return int(round((expected - raw) / WRAP)) * WRAP
        if raw < self._last_raw:
            return self._epoch + WRAP
        return self._epoch
