"""The board's loop without a board: machine.parts stepped in a thread, rows played by the wall."""
import threading
import time
from typing import Any

from coaxial.devices.ctrl import COMMANDS, KINDS, MEASURED, SLOTS, _index, _rows
from coaxial.errors import RigError
from machine.roles import Controller, Part

#: The stand-in's fastest tick: its drive is paced by the wall, not a PWM.
HZ_MAX = 200.0

#: Rows the ring holds (`CTRL_ROWS` - 1, ctrl.h).
RING = 63


class SimulatedCtrl(Controller):

    """ctrl_runner_t's rules over machine.parts: a pass in slot order, the last row held."""

    def __init__(self):
        #: What it measures and drives; the board wires them.
        self.drive: Any = None
        self.angle: Any = None
        self._slots: dict[str, Any] = dict.fromkeys(SLOTS)
        self._wire: tuple = ()
        self._queue = []
        self._lock = threading.Lock()
        self._thread = None
        self._running = False
        self._playing = False
        self._left = 0.0
        self._setpoint = 0.0
        self._channels = dict.fromkeys(('ref', 'value', 'estimate', 'out'), 0.0)
        self._counts = dict.fromkeys(('played', 'idle', 'blind'), 0)

    def state(self):
        with self._lock:
            measured, command, hz = self._wire or (0, 0, 0)
            return dict({'running': self._running, 'playing': self._playing,
                         'measured': MEASURED[measured], 'command': COMMANDS[command],
                         'hz': int(round(hz)), 'rows': len(self._queue),
                         'free': RING - len(self._queue),
                         'queued_s': max(self._left, 0.0) * self._playing
                                     + sum(ms for ms, _ in self._queue) / 1e3,
                         'setpoint': self._setpoint}, **self._counts, **self._channels)

    def read(self, count=None, timeout=None):
        return self.state()

    def _idle(self):
        if self._running:
            raise RigError('the loop is running - off() first (simulated)')

    def slot(self, slot, part=None):
        self._idle()
        index = _index(SLOTS, slot, 'slot')
        if part is not None:
            _index(KINDS, type(part).__name__, 'kind')
            part = Part.KINDS[type(part).__name__](**{n: float(getattr(part, n))
                                                      for n in part.PARAMS})
        self._slots[SLOTS[index]] = part
        return True

    def load(self, feedback):
        for name in SLOTS:
            self.slot(name, getattr(feedback, name))
        return True

    def wire(self, measured, command, hz):
        self._idle()
        if not 0 < float(hz):
            raise RigError('hz is 1 up to the PWM rate (simulated)')
        self._wire = (_index(MEASURED, measured, 'measured'), _index(COMMANDS, command, 'command'),
                      min(float(hz), HZ_MAX))
        return True

    def rows(self, rows):
        wire = _rows(rows)
        with self._lock:
            if len(self._queue) + len(wire) > RING:
                raise RigError('more rows than the ring has room for - state() says how many '
                               '(simulated)')
            self._queue += wire
        return len(wire)

    def drop(self):
        with self._lock:
            self._queue, self._left, self._playing = [], 0.0, False
        return True

    def write(self, setpoint):
        return self.rows([(0.0, setpoint)])

    def on(self):
        if not self._wire:
            raise RigError('nothing wired - wire() first (simulated)')
        if self._slots['regulator'] is None:
            raise RigError('no regulator in slot 3 - slot() first (simulated)')
        for part in self._slots.values():
            if part is not None and hasattr(part, 'reset'):
                part.reset()
        self._channels = dict.fromkeys(self._channels, 0.0)
        self._running = True
        self._thread = threading.Thread(target=self._loop, name='simulated ctrl', daemon=True)
        self._thread.start()
        return True

    def off(self):
        self._running = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        return True

    def is_on(self):
        return self._running

    def _loop(self):
        was = time.monotonic()
        while self._running:
            time.sleep(1.0 / self._wire[2])
            now = time.monotonic()
            self._tick(now - was)
            was = now

    def _measure(self):
        measured = MEASURED[self._wire[0]]
        if measured == 'angle':
            return self.angle.state().get('degrees')
        return self.drive.state()[measured]

    def _tick(self, dt):
        with self._lock:
            if self._left <= 0.0:
                if self._playing:
                    self._counts['played'] += 1
                    self._playing = False
                if self._queue:
                    ms, milli = self._queue.pop(0)
                    self._setpoint, self._left, self._playing = milli / 1e3, self._left + ms / 1e3, True
                else:
                    self._left = 0.0
            if self._playing:
                self._left -= dt
            else:
                self._counts['idle'] += 1
            setpoint = self._setpoint
        x = self._measure()
        if x is None:
            self._counts['blind'] += 1
            return
        u = self._pass(dt, setpoint, float(x))
        self.drive._set_setpoints(**{COMMANDS[self._wire[1]]: u})

    def _pass(self, dt, setpoint, measured):
        """ctrl_feedback_step: prefilter, measure, estimator, regulator."""
        s, c = self._slots, self._channels
        c['ref'] = s['prefilter'].step(dt, x=setpoint)['y'] if s['prefilter'] else setpoint
        c['value'] = s['measure'].step(dt, x=measured)['y'] if s['measure'] else measured
        c['estimate'] = (s['estimator'].step(dt, measured=c['value'], command=c['out'])['estimate']
                         if s['estimator'] else c['value'])
        c['out'] = s['regulator'].step(dt, setpoint=c['ref'], measured=c['estimate'])['command']
        return c['out']
