"""Recording without a board: deep capture, the acquisition engine and the
cycle-counting clock.
"""
import math
import random
import time
from typing import Any

from coaxial.acquire.acquisition import Acquisition
from coaxial.acquire.daq import REPLY_ROOM, Daq
from coaxial.devices import angle, imu
from coaxial.errors import RigError
from coaxial.simulated.system import UNITS
from coaxial.simulated.values import (ACCUMULATE_MAX, AMPS_PER_CODE, CHANNELS, DCBUS_V, MASK32,
                                      PHASE_LEG, RING_BYTES, TICKS_PER_US, PHASE_STEP,
                                      phase_codes, quiet_code)
from tools.cores.stand import DaqCore

#: What the injected group converts, by channel: the triple, the NTC, the DC link
#: (Board_AdcInjected's CH_*). Under the drive's sync a software-clocked field outside it is
#: never read, and no record closes (Board_DaqPoll through read_index).
INJECTED = frozenset((0, 1, 2, 4, 5))

#: The share of the line rate the stand-in quotes as its ceiling.
LINE_SHARE_PERCENT = 75


#: The capture ring's sources the stand-in fills, by bit.
SOURCES = {'phases': 0, 'angle': 1, 'imu': 2}


def _source_mask(sources):
    """A source mask from names, or the mask itself when given one."""
    if isinstance(sources, int):
        return sources
    unknown = [s for s in sources if s not in SOURCES]
    if unknown:
        raise ValueError('no such source: %s - have %s'
                         % (', '.join(unknown), ', '.join(SOURCES)))
    return sum(1 << SOURCES[s] for s in set(sources))


class SimulatedDaq(Acquisition):
    """One acquisition task, backed by DaqCore (`daq.c` and `filter.c`)."""

    backlog = None
    baud = 115200
    FRAME_BYTES = 2 + 1 + 4 + 2 + 8
    TURNAROUND = 0.0
    TABLE = {c['index']: (c['signal'], UNITS.get(c['signal']), c['differential'])
             for c in CHANNELS}
    PHASES = tuple(c['index'] for c in CHANNELS if c['differential'])

    def __init__(self):
        self._core = DaqCore(RING_BYTES)
        self._cfg = None
        self.clock: Any = None
        self.drive: Any = None
        self.thermal: Any = None
        self.sto: Any = None
        self.calibration: Any = None
        self.angle: Any = None
        self.imu: Any = None
        self._noise_pool: Any = None
        self._noise_at = 0
        self._shape = (0, 1)
        self._wall: Any = None
        self._order = []
        self._running = False
        self._done = False
        self._produced = 0
        self._at = 0

    _owed = 0.0
    SLEEP_FLOOR = 0.002
    _POOL = 1021

    def _noise(self):
        """One unit-variance noise term, O(1) and no RNG call."""
        if self._noise_pool is None:
            self._noise_pool = [random.gauss(0.0, 1.0) for _ in range(self._POOL)]
        self._noise_at = (self._noise_at + 1) % self._POOL
        return self._noise_pool[self._noise_at]

    STEADY = {'AFE_ON': 1.0, 'nFAULT/TIM1_BKIN': 1.0, 'KEEPALIVE': 0.5}
    _theta = 0.0
    _theta_at = None

    def _spin(self, seconds):
        """(theta, amps, modulation index, voltage angle) for this record."""
        drive = self.drive
        if drive is None:
            return 0.0, 0.0, 0.0, 0.0
        omega = drive._omega()
        self._theta = (self._theta + omega * seconds) % (2.0 * math.pi)
        iid, iq, vd, vq = drive._dq()
        amps = math.hypot(iid, iq)
        volts = math.hypot(vd, vq)
        vdc = DCBUS_V
        index = min(1.0, (2.0 * volts / vdc) if vdc else 0.0)
        return self._theta, amps, index, math.atan2(vq, vd)

    PHASE_STEP = PHASE_STEP
    PHASE_LEG = PHASE_LEG
    AMPS_PER_CODE = AMPS_PER_CODE
    _last_spin = (0.0, 0.0, 0.0, 0.0)

    GATES = {'TIM1_CH1/PWMUH': (0, True), 'TIM1_CH1N/PWMUL': (0, False),
             'TIM1_CH2/PWMVH': (1, True), 'TIM1_CH2N/PWMVL': (1, False),
             'TIM1_CH3/PWMWH': (2, True), 'TIM1_CH3N/PWMWL': (2, False)}

    QUATERNION_BIT = 0
    VECTOR_BITS = {1: ('acceleration', imu.ACCELEROMETER),
                   2: ('rotation rate', imu.GYROSCOPE),
                   3: ('magnetic field', imu.MAGNETIC_FIELD)}
    SHAFT_BIT = 4
    NO_WORDS = (0, 0, 0, 0)

    def _sensor_words(self, bit):
        if bit == self.SHAFT_BIT:
            return self._shaft_words()
        return self._imu_words(bit)

    def _shaft_words(self):
        part = self.angle
        if part is None:
            return self.NO_WORDS
        got = part.peek(angle.ANG)
        value = got['value'] - 0x10000 if got['value'] >= 0x8000 else got['value']
        return (value, got['crc'], angle.ANG, 1)

    def _imu_words(self, bit):
        part = self.imu
        state = part.state() if part is not None else {}
        if bit == self.QUATERNION_BIT:
            q = state.get('quaternion') or {}
            scale = 1 << imu.SCALE[imu.ROTATION_VECTOR][0]
            return tuple(int(q.get(k, 0.0) * scale) for k in imu.QUATERNION_AXES)
        name, report = self.VECTOR_BITS[bit]
        v = state.get(name) or {}
        scale = float(1 << imu.SCALE[report][0])
        return (int(v.get('x', 0.0) * scale), int(v.get('y', 0.0) * scale),
                int(v.get('z', 0.0) * scale), 3)

    def _pin_duty(self, signal):
        gate = self.GATES.get(signal)
        if gate is not None:
            return self._gate_duty(*gate)
        level = self.STEADY.get(signal, 0.0)
        if level in (0.0, 1.0):
            return level
        return min(1.0, max(0.0, level + self._noise() * 0.02))

    def _gate_duty(self, leg, high):
        theta, _amps, index, delta = self._last_spin
        if index <= 0.0:
            return 0.0
        duty = 0.5 + (index / 2.0) * math.cos(theta + delta - leg * self.PHASE_STEP)
        duty = min(1.0, max(0.0, duty))
        return duty if high else 1.0 - duty

    def _wire_time(self, records):
        if not self.baud:
            return 0.0
        octets = self.FRAME_BYTES + records * self._stride()
        return (octets * 10.0 / float(self.baud)) + self.TURNAROUND

    def _charge_line(self, records):
        self._owed += self._wire_time(records)
        if self._owed < self.SLEEP_FLOOR:
            return
        began = time.time()
        time.sleep(self._owed)
        self._owed = max(self._owed - (time.time() - began), -self.SLEEP_FLOOR)

    def _period_us(self):
        base = 20.0 if (self._cfg or {}).get('clock') == 'tim1' else 47.0
        cfg = self._cfg or {}
        if not cfg.get('accumulate'):
            return float(cfg.get('interval_us') or base)
        return base * max(1, self._core.triggers_per_record())

    def state(self):
        cfg = dict(self._cfg or {'channels': 0, 'clock': 'software', 'sample_time': 0,
                                 'decimate': 0, 'accumulate': 0, 'records': 0})
        cs = self._core.state()
        if self._cfg is not None:
            cfg['accumulate'] = cs['accumulate']
        held = cs['available'] + self._buffered()
        stride = cs['stride'] or self._stride()
        capacity = cs['capacity'] or (RING_BYTES // max(1, stride))
        return {'running': self._running, 'done': self._done,
                'lost_power': False,
                'stride': stride, 'fields': len(self._order),
                'available': held, 'produced': cs['produced'],
                'dropped': cs['dropped'], 'capacity': capacity,
                'worst': max(cs['worst'], min(capacity, held)),
                'rung': cs['rung'], 'rungs': cs['rungs'],
                'rung_changes': cs['rung_changes'],
                'max_rate_hz': int(((self.baud // 10) * LINE_SHARE_PERCENT // 100)
                                   // max(1, stride)),
                'sensors_available': (1 << len(self.SENSORS)) - 1,
                'sensors_supported': True,
                **cfg}

    #: A record's pins in the board's order: the sampled set (`s_digital`'s
    #: `sampled` column), held by the parity suite.
    PINS = ({'signal': 'AFE_ON', 'direction': 'out'},
            {'signal': 'nFAULT/TIM1_BKIN', 'direction': 'in'},
            {'signal': 'KEEPALIVE', 'direction': 'out'},
            {'signal': 'TIM1_CH1N/PWMUL', 'direction': 'out'},
            {'signal': 'TIM1_CH1/PWMUH', 'direction': 'out'},
            {'signal': 'TIM1_CH2N/PWMVL', 'direction': 'out'},
            {'signal': 'TIM1_CH2/PWMVH', 'direction': 'out'},
            {'signal': 'TIM1_CH3N/PWMWL', 'direction': 'out'},
            {'signal': 'TIM1_CH3/PWMWH', 'direction': 'out'})

    def layout(self):
        fields = []
        for i in self._order:
            signal, unit, diff = self.TABLE[i]
            fields.append({'channel': i, 'unit': unit, 'differential': diff,
                           'signal': signal})
        pins = list(self.PINS) if (self._cfg or {}).get('digital') else []
        mask = (self._cfg or {}).get('sensors') or 0
        rows = [{'bit': b, 'words': 4, 'signal': name}
                for b, name in enumerate(self.SENSORS) if mask & (1 << b)]
        return {'stride': self._stride(), 'fields': fields, 'pins': pins,
                'sensors': rows}

    def _stride(self):
        """The record's width, by the board's own arithmetic: the timestamp,
        one sum per field, the digital word when the task has one, and
        the sample count that closes every record.
        """
        digital = len(self.PINS) if (self._cfg or {}).get('digital') else 0
        mask = (self._cfg or {}).get('sensors') or 0
        return (4 + 4 * len(self._order) + digital
                + 8 * bin(mask).count('1') + 2)

    def _resolve(self, channels):
        if isinstance(channels, int):
            return [i for i in sorted(self.TABLE) if channels >> i & 1]
        by_name = {v[0]: k for k, v in self.TABLE.items()}
        out = []
        for c in channels:
            if isinstance(c, int):
                out.append(c)
            elif c in by_name:
                out.append(by_name[c])
            else:
                raise KeyError('no channel carries signal %r; the stand-in '
                               'reports %r' % (c, sorted(by_name)))
        return sorted(out)

    #: The sensor rows, index = wire bit (MINOR 7), the board's spelling.
    SENSORS = ('orientation', 'acceleration', 'rotation rate',
               'magnetic field', 'shaft angle')

    def configure(self, channels, clock='software', sample_time=0,
                  decimate=1, accumulate=None, records=0, digital=False,
                  sample_rate=None, interval_us=None, adapt=False,
                  sensors=0):
        if accumulate is None:
            accumulate = 0 if sample_rate is not None else 1
        if clock not in ('software', 'tim1', 0, 1):
            raise ValueError('clock is %s, not one of software, tim1' % clock)
        if decimate < 1 or accumulate < 0:
            raise ValueError('decimate counts samples so it is at least 1, '
                             'and accumulate at least 0 - zero closes the '
                             'record on the clock instead')
        if self._running:
            raise RigError('the board refused that task - it is running '
                           '(simulated)')
        order = self._resolve(channels)
        if isinstance(clock, int):
            clock = {0: 'software', 1: 'tim1'}.get(clock, str(clock))
        if clock == 'tim1' and any(i not in self.PHASES for i in order):
            raise RigError('the board refused that task - a TIM1 clock '
                           'carries only the phases (simulated)')
        if int(sensors) >> len(self.SENSORS):
            raise RigError('the board refused that task - the sensor mask '
                           'has five bits (simulated)')
        if sensors and clock == 'tim1':
            raise RigError('the board refused that task - sensor fields '
                           'ride the software clock only: a TIM1 record '
                           'closes in the interrupt, which would read the '
                           'poll records torn (simulated)')
        self._order = order
        if interval_us is None:
            interval_us = (0 if sample_rate is None
                           else int(1e6 / float(sample_rate)))
        pins = len(self.PINS) if digital else 0
        self._core.begin(order, accumulate, decimate, int(interval_us * TICKS_PER_US),
                         pins, int(sensors), int(records), bool(adapt))
        self._cfg = {'channels': sum(1 << i for i in order), 'clock': clock,
                     'sample_time': sample_time, 'decimate': decimate,
                     'accumulate': accumulate, 'records': records,
                     'digital': bool(digital), 'interval_us': interval_us,
                     'sensors': int(sensors)}
        self._produced = 0
        self._done = False
        return self.layout()

    def ladder(self, chains):
        """Load the ladder of chains into the C acquisition core (`daq_set_rung`)."""
        if not chains:
            why = self._core.set_rung(0, 0, (), 1)
            if why:
                raise RigError('%s (simulated)' % why)
            return True
        for n, chain in enumerate(chains):
            why = self._core.set_rung(n, chain['boxcar'], chain['sections'], chain['decimate'])
            if why:
                raise RigError('%s (simulated)' % why)
        if self._cfg is not None:
            self._cfg['accumulate'] = self._core.state()['accumulate']
        return True

    def shape(self, sections=(), decimate=1):
        """Load the anti-alias chain into the C acquisition core (`daq_set_filter`)."""
        why = self._core.set_filter(sections, decimate)
        if why:
            raise RigError('%s (simulated)' % why)
        self._shape = (len(sections), int(decimate))
        return True

    def tone(self, hz=0, rate_hz=0, amplitude=10000, offset=32768, kind=0):
        """Configure the C acquisition core's tone generator (`daq_set_tone`)."""
        why = self._core.set_tone(int(hz), int(rate_hz), int(amplitude), int(offset), int(kind),
                                  int(TICKS_PER_US * 1e6), self._at)
        if why:
            raise RigError('%s (simulated)' % why)
        self._tone = (int(hz), int(rate_hz))
        return True

    def start(self):
        if self._cfg is None:
            raise RigError('the board refused to start - configure it first '
                           '(simulated)')
        if self.clock is not None:
            self._at = self.clock.read()['now']
        self._core.start(self._at)
        self._running = True
        self._done = False
        self._produced = 0
        self._wall = time.time()
        self._owed_records = 0.0
        return True

    def _buffered(self):
        """What a stopped run still owes: the real board's buffer stays
        readable after stop, so a bounded run's remainder is served - the
        timed-burst notebook drained 0 records here while the board gave
        512.
        """
        if self._cfg is None or not self._cfg['records']:
            return 0
        return max(0, self._cfg['records'] - self._produced)

    def stop(self):
        self._core.stop()
        self._running = False
        return True

    #: Conversions a second the board's own loop manages, all channels
    #: together - PROTOCOL.md, device 6, measured. What a window holds is
    #: that divided by the fields in a sweep.
    LOOP_HZ = 13200

    def _configured(self):
        """The task as configured; a task nobody configured refuses."""
        if self._cfg is None:
            raise RigError('configure the task first (simulated)')
        return self._cfg

    def _samples_per_record(self, fields):
        """How many sweeps a record holds: `accumulate`, or what the loop would
        fit in the window when the clock closes it.
        """
        cfg = self._configured()
        acc = self._core.state()['accumulate'] or cfg['accumulate']
        if acc:
            return acc
        window_s = (cfg['interval_us'] or 0) / 1e6
        sweeps = self.LOOP_HZ / max(1, fields) * window_s
        return max(1, min(ACCUMULATE_MAX, int(sweeps)))

    def _pace(self, n, step_us):
        """How many of the `n` records a read may answer, and how far apart
        their stamps fall, against the wall since the last read.
        """
        cfg = self._cfg or {}
        now = time.time()
        since = self._wall
        if not n:
            return n, step_us
        if not cfg.get('interval_us'):
            self._wall = now
            spread = step_us if since is None else (now - since) * 1e6 / n
            return n, max(step_us, spread)
        if not self._running:
            return n, step_us
        owed = self._owed_records
        owed += 0.0 if since is None else (now - since) * 1e6 / step_us
        self._wall = now
        n = min(n, int(owed))
        self._owed_records = owed - n
        return n, step_us

    def acquire(self, want=0, layout=None):
        if not self._running and not self._buffered():
            return []
        lay = layout or self.layout()
        fields = lay['fields']
        stride = 4 + 4 * len(fields)
        room = max(1, REPLY_ROOM // stride)
        n = min(int(want) or room, room)
        cfg = self._configured()
        drive = self.drive
        if (cfg.get('clock') == 'software' and drive is not None and drive._sync_armed()
                and any(f['channel'] not in INJECTED for f in fields)):
            return []
        left = cfg['records'] - self._produced if cfg['records'] else n
        n = max(0, min(n, left))
        n, step_us = self._pace(n, self._period_us())
        has_pins = bool(lay.get('pins'))
        mask = cfg.get('sensors') or 0
        tone_on = self._core.tone_on()
        for _ in range(n):
            if tone_on:
                triggers = max(1, self._core.triggers_per_record())
                self._at = (self._at + triggers * max(1, self._core.tone_cycles())) & MASK32
                self._core.tone_poll(self._at, triggers + 1)
                continue
            cs = self._core.state()
            took = self._samples_per_record(len(fields))
            windows = (cs['chain_decimate']
                       if (cs['accumulate'] and (cs['sections'] or cs['chain_decimate'] > 1))
                       else 1)
            win_us = step_us / windows
            sample_s = (win_us * 1e-6) / max(1, took)
            dtheta = (drive._omega() if drive is not None else 0.0) * sample_s
            half = 0.5 * dtheta
            sh = math.sin(half)
            boxcar = (math.sin(took * half) / (took * sh)
                      if (took > 1 and abs(sh) > 1e-7) else 1.0)
            spread = math.sqrt(took * 1210.0)
            for _ in range(windows):
                self._at = (self._at + max(1, int(win_us * TICKS_PER_US))) & MASK32
                self._last_spin = self._spin(win_us * 1e-6)
                theta, amps, _index, _delta = self._last_spin
                theta_mid = theta - 0.5 * (took - 1) * dtheta
                sums = [int(self._trimmed(f['channel'], quiet_code(f['channel'], self.thermal,
                                                                   self.sto)
                                          + phase_codes(f['signal'], amps * boxcar,
                                                        theta_mid)) * took)
                        + int(spread * self._noise())
                        for f in fields]
                duties = ([int(round(self._pin_duty(p['signal']) * 255.0)) for p in self.PINS]
                          if has_pins else None)
                sensors = ({b: self._sensor_words(b) for b in range(len(self.SENSORS))
                            if mask & (1 << b)} if mask else None)
                self._core.push_window(sums, took, self._at, duties, sensors)
        blob = self._core.take(n, lay['stride'])
        out = self.decode(blob, lay) if blob else []
        cs = self._core.state()
        self._produced = cs['produced']
        if cs['done']:
            self._running = False
            self._done = True
        self.backlog = self._buffered()
        self._charge_line(len(out))
        return out

    def _trimmed(self, index, code):
        """A polled record's code through the record, as Board_AdcRead gives it - off the
        latest injected triple while the sync holds the converters, trimmed all the same."""
        record = self.calibration
        return code if record is None else record.apply(index, code)

    _record = staticmethod(Daq._record)

    def decode(self, blob, layout=None):
        """Records out of raw record bytes, decoded by the board's own wire format (`Daq.decode`)."""
        lay = layout or self.layout()
        if not blob:
            return []
        return Daq.decode(self, blob, lay)

    def latest(self, layout=None, block=True, timeout=2.0, poll=0.002):
        if not self._running and not block:
            return None
        if not self._running:
            raise RigError('no sample in %.1f s - is the task running? '
                           '(simulated)' % timeout)
        layout = layout or self.layout()
        base = random.randint(8, 40)
        self._at = (self._at + base * 9500) & MASK32
        drive = self.drive
        amps, theta = drive._carrying() if drive is not None else (0.0, 0.0)
        digital = random.getrandbits(len(self.PINS))
        for idx, f in enumerate(layout['fields']):
            n = base - random.randint(0, 1)
            centre = self._trimmed(f['channel'], quiet_code(f['channel'], self.thermal, self.sto)
                                   + phase_codes(f['signal'], amps, theta))
            self._core.live_insert(idx, centre - 60, self._at, digital)
            self._core.live_insert(idx, centre + 60, self._at, digital)
            for _ in range(max(0, n - 2)):
                self._core.live_insert(idx, centre + random.randint(-60, 60), self._at, digital)
        _fresh, first, last, dig, slots = self._core.take_live()
        out = {'first': first, 'last': last, 'sum': {}, 'count': {},
               'lowest': {}, 'highest': {}}
        for idx, f in enumerate(layout['fields']):
            s_sum, s_cnt, s_lo, s_hi = slots[idx]
            sig = f['signal']
            out['sum'][sig] = s_sum
            out['count'][sig] = s_cnt
            out['lowest'][sig] = s_lo
            out['highest'][sig] = s_hi
        out['mean'] = {k: (v / out['count'][k] if out['count'][k] else None)
                       for k, v in out['sum'].items()}
        if layout.get('pins'):
            out['digital'] = {p['signal']: bool((dig >> i) & 1)
                              for i, p in enumerate(self.PINS)}
        return out
