"""Recording without a board: deep capture, the acquisition engine
and the cycle-counting clock."""
import math
import random
import time

from ..acquisition import Acquisition
from ..errors import RigError
from .values import CHANNELS, DCBUS_V, NOMINAL, _sweep
from .system import UNITS


class SimulatedCapture:
    """The measurement ring, without measurements.

    Invents records at the rates the real sources run at - the injected
    triple at 50 kHz, the angle loop as fast as its SPI allows, the IMU at
    whatever it was asked for - so a caller draining it sees the same shape
    and the same ordering it would off a board.
    """

    DEPTH = 1024

    def __init__(self):
        self._mask = 0
        self._pending = []
        self._seq = [0, 0, 0]
        self._at = 0
        self._dropped = 0

    def _fill(self):
        import random
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
                self._pending.append({'at': self._at & 0xFFFFFFFF,
                                      'source': ('phases', 'angle', 'imu')[src],
                                      'seq': self._seq[src] & 0xFF,
                                      'v': tuple(v)})
                self._seq[src] += 1

    def state(self):
        names = ('phases', 'angle', 'imu')
        return {'sources': [names[i] for i in range(3) if self._mask >> i & 1],
                'mask': self._mask, 'count': len(self._pending),
                'depth': self.DEPTH, 'dropped': self._dropped,
                # Nothing here is throttled - there is no link to be short
                # of - but the field has to exist or a view written against
                # the board would fail on the stand-in, which is the one
                # thing test_parity is for.
                'thinned': 0}

    def arm(self, sources):
        if isinstance(sources, int):
            self._mask = sources
        else:
            names = {'phases': 0, 'angle': 1, 'imu': 2}
            unknown = [s for s in sources if s not in names]
            if unknown:
                raise ValueError('no such source: %s - have %s'
                                 % (', '.join(unknown), ', '.join(names)))
            self._mask = 0
            for s in sources:
                self._mask |= 1 << names[s]
        self._pending = []
        self._seq = [0, 0, 0]
        self._dropped = 0
        return True

    def stop(self):
        return self.arm(0)

    def take(self, want=15):
        want = max(1, min(int(want), 15))
        if self._mask and len(self._pending) < want:
            self._fill()
        batch, self._pending = self._pending[:want], self._pending[want:]
        return batch

    def drain(self, limit=None):
        out = []
        while limit is None or len(out) < limit:
            batch = self.take()
            if not batch:
                break
            out.extend(batch)
            if limit is None and len(out) >= self.DEPTH:
                break
        return out[:limit] if limit is not None else out


class SimulatedDaq(Acquisition):
    """One acquisition task, without a converter.

    Answers `Acquisition` like the real one, and refuses the same things for
    the same reasons: a TIM1 clock carries only the phases, and a task cannot
    be reconfigured while it runs. Inheriting the surface is what makes a name
    dropped from one of them fail here at construction rather than at the
    first call that reached for it.
    """

    #: Records still owed after the last `acquire()` - the stand-in's
    #: answer to the board's backlog field. A free-running task owes
    #: nothing, because it invents each record as it is asked for.
    backlog = None

    #: The line this stand-in pretends to be on. Set from the session's
    #: baud, and it is what makes a simulated run mean anything about
    #: throughput: without it every reply is instant and the host looks
    #: infinitely fast. 10 Mbit/s is an RS485 segment; 115200 is the
    #: debug probe's VCP.
    baud = 115200

    #: Bytes a reply carries beyond its records - unit, function code,
    #: the count byte, the backlog and the CRC - plus the request. The
    #: board's own arithmetic, so the emulated line charges for the same
    #: frame the real one sends.
    FRAME_BYTES = 2 + 1 + 4 + 2 + 8

    #: Fixed cost of a transaction, whatever it carries. On the real link
    #: this is t3.5 and the board's turnaround: measured 1.75 ms and
    #: 2.70 ms on the port itself. It does not shrink with the bitrate,
    #: which is the whole point of emulating it - at 10 Mbit/s it is what
    #: is left.
    TURNAROUND = 0.0

    #: Channel index -> (signal, unit, differential), the stand-in's table.
    #: Built from CHANNELS rather than written out. It was written out, and
    #: two supply senses added to the board's table left the stand-in's DAQ
    #: refusing a channel its own analog side reported - the second answer
    #: this module exists to not be.
    TABLE = {c['index']: (c['signal'], UNITS.get(c['signal']),
                          c['differential'])
             for c in CHANNELS}
    PHASES = tuple(c['index'] for c in CHANNELS if c['differential'])
    # One per channel the board reports. A channel added to `s_analog` and
    # not here is a KeyError on the first frame, which is what happened when
    # the die thermometer took index 9.
    #: ONE TABLE FOR ONE BOARD. This was a second set of quiet points,
    #: different from the one `SimulatedAnalog` reads through - Phase U
    #: sat at 1400 in a record and 900 in a read of the same channel.
    #: A tare could then never zero a record: `cal.zero()` measures
    #: through the analog path and the records came from somewhere
    #: else, so the offset was stored, applied, and wrong.
    CENTRE = NOMINAL

    def __init__(self):
        self._cfg = None
        self._order = []
        self._running = False
        self._done = False
        self._produced = 0
        self._at = 0

    #: Emulated line time owed but not yet slept. `time.sleep` cannot
    #: honour a sub-millisecond wait on Windows - profiled at an
    #: emulated 10 Mbit/s, where a reply is 292 us, it slept 2.878 s
    #: of a 4 s run and the emulator became the bottleneck it was
    #: written to measure. Owed time is banked and paid in one sleep
    #: when it is worth sleeping, so the AVERAGE rate is right at any
    #: bitrate and no single wait is below the clock's resolution.
    _owed = 0.0

    #: Bank this much before sleeping it off.
    SLEEP_FLOOR = 0.002

    #: Noise drawn once and cycled, rather than drawn per field per
    #: record. PROFILED at an emulated 10 Mbit/s: 401 200 gauss calls
    #: and 361 080 randint calls in four seconds, which was the top
    #: of the profile - the library's own decode did not appear in
    #: it at all. A stand-in that costs more than the code it stands
    #: in for cannot benchmark it. Same distribution to a reader,
    #: and a pool this size does not repeat visibly.
    _POOL = 1021

    def _noise(self):
        """One unit-variance noise term, O(1) and no RNG call."""
        pool = getattr(self, '_noise_pool', None)
        if pool is None:
            pool = [random.gauss(0.0, 1.0) for _ in range(self._POOL)]
            self._noise_pool = pool
            self._noise_at = 0
        self._noise_at = (self._noise_at + 1) % self._POOL
        return pool[self._noise_at]

    #: What each pin is doing, as a duty over the record's window.
    #: A STEADY OUTPUT IS STEADY: AFE_ON reads 1.0 when the rail is
    #: up, not a fresh random number every record - noise there made
    #: the stand-in's own example print `AFE 0.43` for a pin that is
    #: simply on. KEEPALIVE is the one that genuinely toggles, at
    #: ~100 kHz, so it lands near half; the gates are down until
    #: something arms them.
    STEADY = {'AFE_ON': 1.0, 'nFAULT/TIM1_BKIN': 1.0,
              'KEEPALIVE': 0.5}

    #: Where the electrical angle is, and when it was there. Advanced by
    #: the drive's own omega, so a record's currents and the gate duties in
    #: the same record are one rotation seen twice.
    _theta = 0.0
    _theta_at = None

    def _spin(self, seconds):
        """(theta, amps, modulation index, voltage angle) for this record.

        THE MOTOR IS THE ONE IN `SimulatedDrive`: R 0.05 ohm, Ld 20 uH,
        Lq 30 uH, lambda 0.005 Wb, 7 pole pairs, 50 kHz. Nothing here
        invents a constant - the currents are the dq references the drive
        was given, put back into the stator frame, and the duties are the
        voltage vector that drive computed for them over the DC link this
        stand-in reports.

        Zero amps and a half duty when nothing is commanded, which is what
        a bench with the stage down looks like.
        """
        drive = getattr(self, 'drive', None)
        if drive is None:
            return 0.0, 0.0, 0.0, 0.0

        omega = drive._omega()                 # electrical rad/s
        self._theta = (self._theta + omega * seconds) % (2.0 * math.pi)

        # The drive's own solution: what current it settled at and
        # what voltage it needed, not the references it was handed.
        iid, iq, vd, vq = drive._dq()
        amps = math.hypot(iid, iq)
        volts = math.hypot(vd, vq)
        # The link this stand-in reports, through its own scaling: the
        # modulation index is what fraction of half the link the vector
        # asks for, and it cannot exceed one.
        vdc = DCBUS_V
        index = min(1.0, (2.0 * volts / vdc) if vdc else 0.0)
        return self._theta, amps, index, math.atan2(vq, vd)


    #: Radians a phase lags the one before it.
    PHASE_STEP = 2.0 * math.pi / 3.0

    #: Which leg each phase channel is, by the board's own name.
    PHASE_LEG = {'Phase U': 0, 'Phase V': 1, 'Phase W': 2}

    #: Amps per code on a phase shunt - `SimulatedDrive.APC`, the same
    #: number `scaling.PHASE_ONBOARD` gives: 3.3 V over 32768 codes
    #: through 3.5 mohm times 4.5455.
    AMPS_PER_CODE = 3.3 / 32768.0 / (0.0035 * 1500.0 / 330.0)

    #: The last (theta, amps, index, delta), so the pins in a record
    #: use the angle its currents were taken at.
    _last_spin = (0.0, 0.0, 0.0, 0.0)

    #: Which leg each gate belongs to, and whether it is the high side.
    #: The board's own spelling, so this table and `PINS` agree by being
    #: the same strings rather than by anyone keeping them in step.
    GATES = {'TIM1_CH1/PWMUH': (0, True), 'TIM1_CH1N/PWMUL': (0, False),
             'TIM1_CH2/PWMVH': (1, True), 'TIM1_CH2N/PWMVL': (1, False),
             'TIM1_CH3/PWMWH': (2, True), 'TIM1_CH3N/PWMWL': (2, False)}

    def _sensor_words(self, bit):
        """Four raw words, the board's own encodings - the shaft off the
        SAME rotor the drive torques, the IMU off the poll record. Wired
        by `SimulatedBoard` like `drive` is; unwired, zeros with have 0,
        which is what an absent part answers."""
        if bit == 4:
            part = getattr(self, 'angle', None)
            if part is None:
                return (0, 0, 0, 0)
            got = part.read(0x20)
            return (got['value'] - 0x10000 if got['value'] >= 0x8000
                    else got['value'], got['crc'], 0x20, 1)
        part = getattr(self, 'imu', None)
        state = part.state() if part is not None else {}
        if bit == 0:
            q = state.get('quaternion') or {}
            return tuple(int(q.get(k, 0.0) * 16384) for k in
                         ('i', 'j', 'k', 'real'))
        name = ('acceleration', 'rotation rate', 'magnetic field')[bit - 1]
        v = state.get(name) or {}
        scale = (256.0, 512.0, 16.0)[bit - 1]      # Q8, Q9, Q4
        return (int(v.get('x', 0.0) * scale), int(v.get('y', 0.0) * scale),
                int(v.get('z', 0.0) * scale), 3)

    def _pin_duty(self, signal):
        """One pin's duty for one record."""
        got = self.GATES.get(signal)
        if got is not None:
            leg, high = got
            theta, _amps, index, delta = self._last_spin
            if index <= 0.0:
                return 0.0            # the stage is down; both gates idle
            # Sine modulation about half: the voltage vector's angle, one
            # third of a turn per leg. The low side is the complement,
            # which is what a half bridge is.
            duty = 0.5 + (index / 2.0) * math.cos(
                theta + delta - leg * self.PHASE_STEP)
            duty = min(1.0, max(0.0, duty))
            return duty if high else 1.0 - duty

        level = self.STEADY.get(signal, 0.0)
        if level in (0.0, 1.0):
            return level
        # Only what actually toggles gets jitter, and only a little.
        return min(1.0, max(0.0, level + self._noise() * 0.02))

    def _wire_time(self, records):
        """What a reply of `records` costs on the emulated line."""
        if not self.baud:
            return 0.0
        octets = self.FRAME_BYTES + records * self._stride()
        return (octets * 10.0 / float(self.baud)) + self.TURNAROUND

    def _charge_line(self, records):
        """Bank this reply's line time, and pay when it is worth it."""
        self._owed += self._wire_time(records)
        if self._owed >= self.SLEEP_FLOOR:
            began = time.time()
            time.sleep(self._owed)
            # Whatever the sleep overshot comes off the next bill,
            # so a coarse clock does not compound into a slow line.
            self._owed -= (time.time() - began)
            if self._owed < 0.0:
                self._owed = max(self._owed, -self.SLEEP_FLOOR)

    def _period_us(self):
        base = 20.0 if (self._cfg or {}).get('clock') == 'tim1' else 47.0
        cfg = self._cfg or {}
        # A CLOCK-CLOSED RECORD HAS NO ACCUMULATE, and multiplying by
        # it gave a period of zero: every record carried the same
        # timestamp, so `dt` came out 0.0 and a host could not tell
        # how long a window covered. The clock's own interval is what
        # closes those, exactly as it does on the board.
        if not cfg.get('accumulate'):
            return float(cfg.get('interval_us') or base)
        return base * cfg['decimate'] * cfg['accumulate']

    def state(self):
        cfg = self._cfg or {'channels': 0, 'clock': 'software', 'sample_time': 0,
                            'decimate': 0, 'accumulate': 0, 'records': 0}
        held = self._buffered()
        # DAQ_BYTES, and the board's number: 16384 was the ring before it
        # moved into the AXI SRAM, and a stand-in quoting the old one
        # reports a capacity no host would ever see.
        capacity = (448 * 1024) // max(1, self._stride())
        return {'running': self._running, 'done': self._done,
                'lost_power': False,
                'stride': self._stride(), 'fields': len(self._order),
                'available': held, 'produced': self._produced,
                'dropped': 0, 'capacity': capacity,
                'worst': min(capacity, held),
                'rung': 0, 'rungs': getattr(self, '_ladder', 0),
                'rung_changes': 0,
                # `cmd_link_records_per_second`, the board's own formula,
                # against this stand-in's line. Missing here until now,
                # so a view asking what the link carries got nothing and
                # fell back to the frame rate.
                'max_rate_hz': int(((self.baud // 10) * 75 // 100)
                                   // max(1, self._stride())),
                'sensors_available': (1 << len(self.SENSORS)) - 1,
                'sensors_supported': True,
                **cfg}

    #: What a record carries, in the board's order - the sampled set, not
    #: the writable one. The six gates are read and never driven, and the
    #: buses and the debug port stay out; `s_digital`'s `sampled` column is
    #: the original and this follows it or the parity suite says so.
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
        """The record's width, by the board's own arithmetic: the
        timestamp, one sum per field, the digital word when the task has
        one, and the sample count that closes every record.

        One formula, not two: the stand-in quoted a stride two bytes
        short of the board's the day the count was appended, and a
        stride is exactly what a host decodes by."""
        # ONE BYTE A PIN, not one word: the pins go through the same
        # window as everything else and come out as a duty. The stand-in
        # quoted the board's old shape once already and a stride is what
        # a host decodes by.
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
        from ..errors import RigError
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
        clock = {0: 'software', 1: 'tim1'}.get(clock, clock)
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
        self._cfg = {'channels': sum(1 << i for i in order), 'clock': clock,
                     'sample_time': sample_time, 'decimate': decimate,
                     'accumulate': accumulate, 'records': records,
                     'digital': bool(digital), 'interval_us': interval_us,
                     'sensors': int(sensors)}
        self._produced = 0
        self._done = False
        return self.layout()

    def ladder(self, chains):
        """Remembered, not climbed: what a ladder answers is what a real
        ring does under a real link, and the stand-in has neither."""
        self._ladder = len(chains)
        return True

    def shape(self, sections=(), decimate=1):
        """Accepted and remembered; the stand-in invents values rather
        than filtering them, and says so by changing nothing."""
        self._shape = (len(sections), int(decimate))
        return True

    def tone(self, hz=0, rate_hz=0, amplitude=10000, offset=32768, kind=0):
        """Remembered, not generated: proving a transfer needs the real
        ring and the real link, which is what the tone is for."""
        self._tone = (int(hz), int(rate_hz))
        return True

    def start(self):
        from ..errors import RigError
        if self._cfg is None:
            raise RigError('the board refused to start - configure it first '
                           '(simulated)')
        self._running = True
        self._done = False
        # Anchored where the clock is now, then advanced by the period: a
        # record's stamp has to be on the same timebase the sync was made
        # against, and monotone within a burst.
        if getattr(self, 'clock', None) is not None:
            self._at = self.clock.read_latch()['now']
        self._produced = 0
        return True

    def _buffered(self):
        """What a stopped run still owes: the real board's buffer stays
        readable after stop, so a bounded run's remainder is served -
        measured jank: the timed-burst notebook drained 0 records here
        while the board gave 512."""
        if self._cfg is None or not self._cfg['records']:
            return 0
        return max(0, self._cfg['records'] - self._produced)

    def stop(self):
        self._running = False
        return True

    #: Conversions a second the board's own loop manages, all channels
    #: together - PROTOCOL.md, device 6, measured. What a window holds is
    #: that divided by the fields in a sweep.
    LOOP_HZ = 13200

    def _samples_per_record(self, fields):
        """How many sweeps a record holds: `accumulate`, or what the loop
        would fit in the window when the clock closes it."""
        if self._cfg['accumulate']:
            return self._cfg['accumulate']
        window_s = (self._cfg['interval_us'] or 0) / 1e6
        sweeps = self.LOOP_HZ / max(1, fields) * window_s
        return max(1, min(32767, int(sweeps)))

    def acquire(self, want=0, layout=None):
        import random
        if not self._running and not self._buffered():
            return []
        fields = (layout or self.layout())['fields']
        stride = 4 + 4 * len(fields)
        room = max(1, 240 // stride)
        n = min(int(want) or room, room)
        left = self._cfg['records'] - self._produced if self._cfg['records'] else n
        n = max(0, min(n, left))
        # THE STAMPS TRACK THE WALL. A free-running software clock (no
        # accumulate, no interval) stamped every record `base` apart while
        # the line paced how many were actually made: 192 records "in"
        # 9 ms of stamp against 0.6 s of wall, and an omega read off a
        # recorded frame came out in megaradians. The records a batch
        # invents span the wall time since the last batch, floored at the
        # sweep cost; a clock-closed config keeps its own interval, as the
        # board does.
        step_us = self._period_us()
        cfg = self._cfg or {}
        if n and not cfg.get('interval_us'):
            now = time.time()
            since = getattr(self, '_wall', None)
            if since is not None:
                step_us = max(step_us, (now - since) * 1e6 / n)
            self._wall = now
        out = []
        for _ in range(n):
            self._at = (self._at + int(step_us * 475)) & 0xFFFFFFFF
            took = self._samples_per_record(len(fields))
            rec = {'at': self._at, 'samples': took}
            # THE SUM, NOT THE SAMPLES. Drawing `took` uniform noise
            # terms per field and adding them was the single most
            # expensive thing in the stand-in: profiled at an
            # emulated 10 Mbit/s it made 384 275 randint calls in
            # four seconds and was most of what the run measured,
            # so a benchmark against it was benchmarking the
            # simulator. A sum of `took` draws from [-60, 60] has
            # mean 0 and variance took * 1210, and one gauss call
            # gives the same distribution to a reader that only
            # ever sees the sum.
            spread = math.sqrt(took * 1210.0)
            # ONE ROTATION, SEEN TWICE. The currents below and the gate
            # duties further down come from the same electrical angle, so
            # a trace and the modulation that produced it line up because
            # they ARE the same thing rather than because two generators
            # were started together. The rotor advances by the SAME step
            # the stamp does - theta against `at` is what an
            # identification differentiates.
            self._last_spin = self._spin(step_us * 1e-6)
            theta, amps, _index, _delta = self._last_spin
            for f in fields:
                index = f['channel']
                centre = self.CENTRE[index]
                leg = self.PHASE_LEG.get(f['signal'])
                if leg is not None and amps:
                    # Balanced three-phase, in codes: the dq solution the
                    # drive settled at, put back into the stator frame
                    # through the stand-in's own amps-per-code.
                    offset = took * amps * math.cos(
                        theta - leg * self.PHASE_STEP) / self.AMPS_PER_CODE
                else:
                    # ONE SOURCE FOR A QUIET CHANNEL. `_sweep` is what a
                    # read of this channel returns, and a record that made
                    # up its own quiet point could never be zeroed by a
                    # tare measured through the other path - the offset was
                    # stored, applied, and wrong.
                    offset = took * _sweep(index)
                rec[f['signal']] = (centre * took + int(offset)
                                    + int(spread * self._noise()))
            if (layout or self.layout()).get('pins'):
                # A duty like the board's, not a level: 0.0 to 1.0 of the
                # window the record covers.
                rec['digital'] = {p['signal']: self._pin_duty(p['signal'])
                                  for p in self.PINS}
            mask = (self._cfg or {}).get('sensors') or 0
            if mask:
                rec['sensors'] = {name: self._sensor_words(b)
                                  for b, name in enumerate(self.SENSORS)
                                  if mask & (1 << b)}
            out.append(rec)
        self._produced += n
        if self._cfg['records'] and self._produced >= self._cfg['records']:
            self._running = False
            self._done = True
        self.backlog = self._buffered()
        # THE LINE, CHARGED IN TIME. A stand-in that answers instantly
        # measures the host and calls it the link.
        self._charge_line(len(out))
        return out

    def decode(self, blob, layout=None):
        """Records out of raw record bytes, as the board's decoder does.

        THE STAND-IN HAS NO WIRE, so there is no blob to decode: records
        are invented whole. It answers the name because the front door
        calls it when a broker's ring hands over bytes, and a name that
        exists on one side of the parity and not the other fails at the
        first call that reaches for it - which is what this suite is for.
        """
        layout = layout or self.layout()
        stride = layout['stride'] or 1
        return self.acquire(want=len(blob) // stride, layout=layout)

    def drain(self, limit=None, layout=None):
        out = []
        while limit is None or len(out) < limit:
            batch = self.read(layout=layout)
            if not batch:
                break
            out.extend(batch)
        return out[:limit] if limit is not None else out

    def latest(self, layout=None, block=True, timeout=2.0, poll=0.002):
        import random
        from ..errors import RigError
        if not self._running:
            if not block:
                return None
            raise RigError('no sample in %.1f s - is the task running? '
                           '(simulated)' % timeout)
        layout = layout or self.layout()
        base = random.randint(8, 40)
        self._at = (self._at + base * 9500) & 0xFFFFFFFF
        out = {'first': self._at, 'last': self._at, 'sum': {}, 'count': {},
               'lowest': {}, 'highest': {}}
        for f in layout['fields']:
            # A channel or two behind the rest, the way the real poll leaves
            # them: it reads one per turn and a take lands mid-sweep.
            n = base - random.randint(0, 1)
            out['sum'][f['signal']] = sum(
                self.CENTRE[f['channel']] + random.randint(-60, 60)
                for _ in range(n))
            out['count'][f['signal']] = n
            centre = self.CENTRE[f['channel']]
            out['lowest'][f['signal']] = centre - 60
            out['highest'][f['signal']] = centre + 60
        out['mean'] = {k: (v / out['count'][k] if out['count'][k] else None)
                       for k, v in out['sum'].items()}
        if layout.get('pins'):
            out['digital'] = {p['signal']: bool(random.getrandbits(1))
                              for p in self.PINS}
        return out

    def once(self, channels, records, clock='software', sample_time=0,
                decimate=1, accumulate=1, timeout=10.0, digital=False,
                sample_rate=None):
        layout = self.configure(channels, clock=clock, sample_time=sample_time,
                                decimate=decimate, accumulate=accumulate,
                                records=records, digital=digital,
                                sample_rate=sample_rate)
        self.start()
        out = []
        while len(out) < records:
            batch = self.read(layout=layout)
            if not batch:
                break
            out.extend(batch)
        self.stop()
        return out[:records], layout


class SimulatedClock:
    """The cycle counter tied to nothing, but tied consistently.

    Runs 12 ppm slow of its nominal, which is about what a real crystal
    does, so a caller that checks the measured rate against the asked-for
    one sees a number of the right size rather than an exact match that
    could only come from a stand-in.
    """

    NOMINAL_HZ = 475000000
    SKEW = 1 - 12e-6

    def __init__(self):
        self._seq = 0
        self._latched = 0
        self._t0 = None

    def _cycles(self):
        import time
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
        from ..clock import Clock
        return Clock.probe(self, rounds=rounds)

    def sync(self, seconds=2.0, rounds=8, reference='utc', ntp_server=None):
        from ..clock import Clock, NTP_SERVER
        # Its cycles come off this machine's clock, so against UTC it is
        # this machine's error plus its own 12 ppm - which is the honest
        # answer, not a bug.
        return Clock.sync(self, seconds=seconds, rounds=rounds,
                          reference=reference,
                          ntp_server=ntp_server or NTP_SERVER)

    def _bracket(self):
        """One latch, bracketed - on `perf_counter`, as the real one is.

        THE SAME CLOCK BASE, because `Clock.sync` converts what this
        returns from perf to wall with `time.time() - perf_counter()`.
        Returning a wall time here made it add the offset to a value
        that already had it: `at_host` came out at exactly twice the
        epoch, and a DataFrame indexed by it landed in the year 2083.
        """
        import time
        before = time.perf_counter()
        self.latch()
        after = time.perf_counter()
        return (before + after) / 2.0, after - before
