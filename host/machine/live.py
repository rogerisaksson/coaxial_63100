"""A machine run live: chunks streamed in, played from a buffer, a failsafe on silence.

The machine never waits on whoever writes - a model on this computer or behind a network.

    live = Live(machine, timeout=1.0, horizon=2.0)   # failsafe: the machine's own
    live.start()                       # armed; the loop runs, holding, until stop()
    live.send('0.5 left_knee=30')      # a whole chunk: None, or why not in a line
    live.feed(tokens)                  # text as it is written: each line plays once finished
    live.wait()                        # blocks until low, tripped or failsafe: one line + now
    live.stop()                        # the failsafe, then disarmed

A chunk is checked whole on arrival (names, ranges, routines) and refused whole. Fed text
goes a line at a time; a line naming a label, a jump or a group waits for its block's blank
line. The buffer holds at most `horizon` seconds, the chunk playing included: past it the
writer is told to wait. Empty, the loop holds the last setpoints; `timeout` seconds of that
plays the failsafe once. A trip, a fault or a lost node plays the failsafe without limits,
then stops.
"""
import collections
import re
import threading
import time

from machine.errors import MachineError
from machine.sequencer import Sequencer, Tripped

#: What ties a line to others: it waits for its block's blank line.
_TIED = re.compile(r'\b(label|goto|then|else|group)=')


class Live:

    """One thread plays chunks on the machine's loop; `send`, `feed`, `wait` from another."""

    def __init__(self, machine, failsafe=None, timeout=1.0, horizon=2.0, clock=time.monotonic):
        self.machine, self.timeout, self.horizon = machine, float(timeout), float(horizon)
        self._clock = clock
        self._failsafe = failsafe or getattr(machine, 'failsafe', None)
        self._chunks = collections.deque()
        self._lock = threading.RLock()
        self._cond = threading.Condition(self._lock)
        self._thread, self._stopping, self._passed = None, False, threading.Event()
        self._until, self._partial, self._block = 0.0, '', []
        self.status, self.reason, self.errors, self.waits, self.played = 'stopped', None, [], [], 0
        self.events = []
        if self._failsafe:
            self._parse(self._failsafe)      # a failsafe that would be refused is refused now

    def _parse(self, text):
        m = self.machine
        seq = Sequencer.parse(text, ranges=m.ranges, limits=m.limits, routines=m.routines)
        seq._checked(m.loop)
        return seq

    def buffered(self):
        """Seconds to play: what is queued, and what is left of the chunk playing."""
        with self._lock:
            return (sum(seq.seconds() for _, seq in self._chunks)
                    + max(0.0, self._until - self._clock()))

    def send(self, text):
        """Queue a chunk: None, or why not - in a line, for the writer to act on."""
        if self.status in ('tripped', 'fault', 'stopped'):
            return 'refused: the machine is %s%s' % (self.status,
                                                     ' - ' + self.reason if self.reason else '')
        try:
            seq = self._parse(text)
        except (MachineError, ValueError) as exc:
            self.errors.append(str(exc).splitlines()[-1])
            return 'refused: ' + self.errors[-1]
        with self._cond:
            if self.buffered() + seq.seconds() > self.horizon:
                return 'refused: %.2f s buffered, the horizon %.2f s - send when it drains' % (
                    self.buffered(), self.horizon)
            self._chunks.append((self._clock(), seq))
            self._cond.notify_all()
        return None

    def feed(self, text):
        """Text as the model writes it: each finished line checked and queued at once, so
        motion starts on the first line, not the last. What was refused, a line each."""
        self._partial += text
        *lines, self._partial = self._partial.split('\n')
        return [said for line in lines for said in self._line(line)]

    def flush(self):
        """The end of an answer: its last line, and a block still open, sent."""
        tail, self._partial = self._partial, ''
        return self._line(tail) + (self._close() if self._block else [])

    def _line(self, line):
        if not line.strip():
            return self._close() if self._block else []
        body = line.split('#')[0].strip()
        if not body:
            return []
        if self._block or _TIED.search(body):
            self._block.append(body)
            return []
        said = self.send(body)
        return [said] if said else []

    def _close(self):
        text, self._block = '\n'.join(self._block), []
        said = self.send(text)
        return [said] if said else []

    def wait(self, low=0.5, timeout=5.0):
        """Until `low` seconds or less are left to play, an event (a failsafe, a trip, a
        fault, a stop) or `timeout`: then one line - what happened, and what changed."""
        deadline, seen = self._clock() + timeout, len(self.events)
        with self._cond:
            while True:
                if len(self.events) > seen or self.status in ('tripped', 'fault', 'stopped'):
                    what = self.events[-1] if self.events else self.status
                    break
                if self.buffered() <= low:
                    what = 'low %.2f s' % self.buffered()
                    break
                if self._clock() >= deadline:
                    what = 'playing %.2f s' % self.buffered()
                    break
                self._cond.wait(0.05)
        return '%s | %s' % (what, self.machine.status(changed=True))

    def _event(self, status, reason=None):
        with self._cond:
            self.status, self.reason = status, reason
            self.events.append(status + (': ' + reason if reason else ''))
            self._cond.notify_all()

    def start(self):
        """Arm, then play from the buffer on a thread of its own."""
        self.machine.arm()
        self.status, self.reason, self._stopping = 'running', None, False
        self._passed.clear()
        self._thread = threading.Thread(target=self._play, daemon=True)
        self._thread.start()
        self._passed.wait(1.0)              # one pass in: the bus holds a reading
        return self

    def _play(self):
        loop = self.machine.loop
        idle, failed = self._clock(), False
        try:
            while not self._stopping:
                with self._cond:
                    chunk = self._chunks.popleft() if self._chunks else None
                    if chunk is not None:
                        self._until = self._clock() + chunk[1].seconds()
                if chunk is not None:
                    sent, seq = chunk
                    self.waits.append(self._clock() - sent)
                    self.status = 'running'
                    seq.play(loop)
                    self.played += 1
                    idle = self._clock()
                    with self._cond:
                        self._until = 0.0
                        self._cond.notify_all()
                    continue
                loop.run(loop.pause, watch=self._limits)
                if not failed and self._failsafe and self._clock() - idle > self.timeout:
                    self._event('failsafe', 'nothing sent for %.2f s' % self.timeout)
                    self._parse(self._failsafe).play(loop)
                    failed = True
            if not failed:
                self._safe(loop)
        except Tripped as trip:
            self._event('tripped', str(trip))
            self._safe(loop)
        except Exception as fault:          # a lost node, a link, a fault: the machine stops
            self._event('fault', '%s: %s' % (type(fault).__name__, fault))
            self._safe(loop)
        finally:
            self.machine.disarm()
            if self.status in ('running', 'failsafe'):
                self._event('stopped')

    def _limits(self, loop):
        """The machine's limits on a holding pass."""
        self._passed.set()
        for channel, levels in self.machine.limits.items():
            value = loop.bus.get(channel)
            for level, bound in levels.items():
                if value is not None and (value >= bound if level.startswith('H') else
                                          value <= bound) and len(level) == 2:
                    raise Tripped('%s %.4g past %s %.4g' % (channel, value, level, bound))
        return False

    def _safe(self, loop):
        if self._failsafe:
            try:
                self._parse(self._failsafe).play(loop, trips=False)
            except Exception as exc:        # the failsafe itself failed: disarm is next
                self.errors.append('failsafe: %s' % exc)

    def stop(self):
        """Stop playing: whatever is queued is dropped; the failsafe; disarmed."""
        self._stopping = True
        with self._cond:
            self._chunks.clear()
            self._block, self._partial = [], ''
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        return self.state()

    def state(self):
        waits = self.waits[-50:]
        return {'status': self.status, 'reason': self.reason, 'buffered': self.buffered(),
                'queued': len(self._chunks), 'played': self.played,
                'wait_max': max(waits) if waits else None,
                'wait_mean': sum(waits) / len(waits) if waits else None,
                'errors': self.errors[-5:]}
