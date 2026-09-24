"""A machine run live: chunks streamed in, played from a buffer, a failsafe on silence.

The machine never waits on whoever writes - a model on this computer or behind a network.

    live = Live(machine, failsafe='0 run=stand', timeout=1.0, horizon=2.0)
    live.start()                       # armed; the loop runs, holding, until stop()
    live.send('0.5 left_knee=30')      # None, or why not in a line; checked, then queued
    live.state()                       # status, buffered s, waits (latency), the last error
    live.stop()                        # the failsafe, then disarmed

A chunk is checked whole on arrival (names, ranges, routines) and refused whole. The
buffer holds at most `horizon` seconds: past it `send` refuses, and the writer waits.
Empty, the loop holds the last setpoints; `timeout` seconds of that plays the failsafe
once. A trip, a fault or a lost node plays the failsafe without limits, then stops.
"""
import collections
import threading
import time

from machine.errors import MachineError
from machine.sequencer import Sequencer, Tripped


class Live:

    """One thread plays chunks on the machine's loop; `send` from any other."""

    def __init__(self, machine, failsafe=None, timeout=1.0, horizon=2.0, clock=time.monotonic):
        self.machine, self.timeout, self.horizon = machine, float(timeout), float(horizon)
        self._clock = clock
        self._failsafe = failsafe
        self._chunks = collections.deque()
        self._lock = threading.Lock()
        self._thread, self._stopping = None, False
        self.status, self.reason, self.errors, self.waits, self.played = 'stopped', None, [], [], 0
        if failsafe:
            self._parse(failsafe)            # a failsafe that would be refused is refused now

    def _parse(self, text):
        m = self.machine
        seq = Sequencer.parse(text, ranges=m.ranges, limits=m.limits, routines=m.routines)
        seq._checked(m.loop)
        return seq

    def buffered(self):
        """Seconds queued, each row once."""
        with self._lock:
            return sum(seq.seconds() for _, seq in self._chunks)

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
        if self.buffered() + seq.seconds() > self.horizon:
            return 'refused: %.2f s buffered, the horizon %.2f s - send when it drains' % (
                self.buffered(), self.horizon)
        with self._lock:
            self._chunks.append((self._clock(), seq))
        return None

    def start(self):
        """Arm, then play from the buffer on a thread of its own."""
        self.machine.arm()
        self.status, self.reason, self._stopping = 'running', None, False
        self._thread = threading.Thread(target=self._play, daemon=True)
        self._thread.start()
        return self

    def _play(self):
        loop = self.machine.loop
        idle, failed = self._clock(), False
        try:
            while not self._stopping:
                with self._lock:
                    chunk = self._chunks.popleft() if self._chunks else None
                if chunk is not None:
                    sent, seq = chunk
                    self.waits.append(self._clock() - sent)
                    self.status = 'running'
                    seq.play(loop)
                    self.played += 1
                    idle = self._clock()
                    continue
                loop.run(loop.pause, watch=self._limits)
                if not failed and self._failsafe and self._clock() - idle > self.timeout:
                    self.status, self.reason = 'failsafe', 'nothing sent for %.2f s' % self.timeout
                    self._parse(self._failsafe).play(loop)
                    failed = True
            if not failed:
                self._safe(loop)
        except Tripped as trip:
            self.status, self.reason = 'tripped', str(trip)
            self._safe(loop)
        except Exception as fault:          # a lost node, a link, a fault: the machine stops
            self.status, self.reason = 'fault', '%s: %s' % (type(fault).__name__, fault)
            self._safe(loop)
        finally:
            self.machine.disarm()
            if self.status in ('running', 'failsafe'):
                self.status = 'stopped'

    def _limits(self, loop):
        """The machine's limits on a holding pass."""
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
        with self._lock:
            self._chunks.clear()
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
