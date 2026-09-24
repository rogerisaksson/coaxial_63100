"""The rig's task: channels by name, the configuration, the outputs, frames and columns."""
import re
import time

from coaxial.acquire.record import Record
from coaxial.devices import angle as angle_scaling
from coaxial.errors import RigError


#: What the board measures besides ADC channels and pins, in wire order: a
#: row's index is its bit in daq op 1's sensor mask (MINOR 7); older firmware
#: lists them as unselectable, with the reason.
SENSOR_FIELDS = (
    {'name': 'orientation', 'kind': 'sensor', 'direction': 'in',
     'unit': 'quaternion'},
    {'name': 'acceleration', 'kind': 'sensor', 'direction': 'in',
     'unit': 'm/s^2'},
    {'name': 'rotation rate', 'kind': 'sensor', 'direction': 'in',
     'unit': 'rad/s'},
    {'name': 'magnetic field', 'kind': 'sensor', 'direction': 'in',
     'unit': 'uT'},
    {'name': 'shaft angle', 'kind': 'sensor', 'direction': 'in',
     'unit': 'deg'},
)


#: Each sensor field's four words, in wire order - the encodings device 5
#: established, raw with the scale left to the caller.
SENSOR_WORDS = {
    'orientation': ('i', 'j', 'k', 'real'),
    'acceleration': ('x', 'y', 'z', 'status'),
    'rotation rate': ('x', 'y', 'z', 'status'),
    'magnetic field': ('x', 'y', 'z', 'status'),
    'shaft angle': ('value', 'crc', 'reg', 'have'),
}


class Task:

    """What the task samples and writes, and what it yields as frames."""

    def channels(self):
        """What the board says it has. Not a list written down here."""
        return self.board.analog.names()

    def _lifted(self, records):
        """The records as `Record`s."""
        fields = (self.layout or {}).get('fields') or []
        return [r if isinstance(r, Record)
                else Record(r, fields, r.get('start_time'), r.get('dt'))
                for r in records]

    def channel_names(self, record=None):
        """What the records carry, in the order they carry it."""
        if record is None:
            return [f['signal'] for f in (self.layout or {}).get('fields') or []]
        return list(self._lifted([record])[0].channel_name)

    def series(self, records, name):
        """One channel out of a run, as a plain list of means."""
        if not records:
            return []
        records = self._lifted(records)
        want = self._match(name)
        if want in ('time', 'starttime'):
            return [r.start_time for r in records]
        if want == 'dt':
            return [r.dt for r in records]
        spelling = next((s.name for s in records[0].samples
                         if self._match(s.name) == want), None)
        if spelling is not None:
            return [r.value(spelling) for r in records]
        # A pin, then.
        pin = self._pin_called(records[0], want)
        if pin is not None:
            return [(r.digital or {}).get(pin) for r in records]
        raise RigError(
            'no channel called %r in these records. They have: %s'
            % (name, ', '.join(records[0].channel_name)))

    def _pin_called(self, record, want):
        """The pin in a record's digital word that `want` names, loosely - with
        or without its port - or None.
        """
        return next((pin for pin in (record.digital or {})
                     if want in (self._match(pin),
                                 self._match(pin.split('/')[-1]))), None)

    def frame(self, records, index='time', scaled=False):
        """A run as a pandas DataFrame: one column per channel."""
        try:
            import pandas
        except ImportError:
            raise RigError(
                'frame() needs pandas, which this library does not require '
                '- `pip install pandas`, or use columns() and build what '
                'you like from plain lists') from None

        cols = self.columns(records)
        if scaled:
            cols.update(self._in_units(cols))
        frame = pandas.DataFrame(cols)
        stamped = cols['time'] and cols['time'][0] is not None
        if index == 'since' and stamped:
            # SECONDS BEFORE NOW: newest at 0, older negative.
            frame['since'] = [t - cols['time'][-1] for t in cols['time']]
            return frame.set_index('since')
        if index == 'elapsed' and stamped:
            frame['elapsed'] = [t - cols['time'][0] for t in cols['time']]
            return frame.set_index('elapsed')
        if index == 'time' and cols['time'] and cols['time'][0] is not None:
            # A real timestamp rather than a float, so resample() and the rest
            # of the time machinery work without a conversion the caller has to
            # remember.
            frame['time'] = pandas.to_datetime(frame['time'], unit='s')
            return frame.set_index('time')
        return frame

    def _in_units(self, cols):
        """Real-unit columns beside the codes, named `Phase U (A)`."""
        scale = self.board.analog.scaling()
        pick = {'centi-degC': ('ntc', 'celsius', 'C'),
                'mA': ('phase', 'amps', 'A'),
                'mV': ('dcbus', 'volts', 'V')}
        # THE CHANNEL'S OWN ZERO AND GAIN, from the calibration record - what
        # `tare()` wrote.
        trim = {c['index']: c for c in
                self.board.calibration.read()['channels']}
        out = {}
        # The sensor snapshots' real units, the same one-place scalings the
        # subsystems use: the shaft through coaxial.devices.angle, the quaternion out
        # of Q14.
        if 'shaft angle value' in cols:
            out['shaft angle (deg)'] = [
                None if v is None else angle_scaling.degrees(v & 0xFFFF)
                for v in cols['shaft angle value']]
        if 'orientation real' in cols:
            for word in ('i', 'j', 'k', 'real'):
                out['orientation %s (unit)' % word] = [
                    None if v is None else v / 16384.0
                    for v in cols['orientation %s' % word]]
        for field in (self.layout or {}).get('fields') or []:
            got = pick.get(field.get('unit'))
            if got is None or field['signal'] not in cols:
                continue
            part, method, short = got
            convert = getattr(scale[part], method)
            fix = trim.get(field['channel'], {})
            offset = fix.get('offset_raw') or 0
            gain = 1.0 + (fix.get('gain_ppm') or 0) / 1e6
            out['%s (%s)' % (field['signal'], short)] = [
                convert((v - offset) * gain) for v in cols[field['signal']]]
        return out

    def frames(self, window=2.0, buffer=None, seconds=None, scaled=False,
               **kw):
        """The last `window` seconds, again each time records arrive."""

        deep = max(float(buffer or window), float(window))
        self._history = []
        began = time.time()
        kw.setdefault('index', 'since')
        while seconds is None or time.time() - began < seconds:
            got = self.read(-1)
            if not got and self.state().get('done'):
                return
            if got:
                yield self.frame(self._window(got, deep, window),
                                 scaled=scaled, **kw)

    def _window(self, got, deep, window):
        """`got` into the history, the history trimmed to `deep` seconds behind
        its newest stamp, and the last `window` seconds of it.
        """
        self._history.extend(got)
        edge = self._history[-1].start_time
        if edge is None:
            return self._history
        self._history = [r for r in self._history
                         if r.start_time is None or r.start_time > edge - deep]
        return [r for r in self._history
                if r.start_time is None or r.start_time > edge - window]

    def history(self, scaled=False, **kw):
        """Everything `frames()` still holds - the buffer behind the window."""
        kw.setdefault('index', 'since')
        return self.frame(self._history, scaled=scaled, **kw)

    def columns(self, records):
        """Records as columns: {name: values}, plus `time` and `dt`."""
        records = self._lifted(records)
        names = self.channel_names(records[0] if records else None)
        # THE PINS ARE COLUMNS TOO.
        pins = list((records[0].digital or {}) if records else {})
        # And the sensor snapshots (MINOR 7), one column per word: 'shaft angle
        # value' beside the currents it was latched with.
        first = (records[0].sensors or {}) if records else {}
        subs = {field: ['%s %s' % (field, w) for w in
                        SENSOR_WORDS.get(field,
                                         ('w0', 'w1', 'w2', 'w3'))]
                for field in first}
        flat = [col for cols in subs.values() for col in cols]
        out = {name: [] for name in names + pins + flat}
        out['time'] = []
        out['dt'] = []
        for record in records:
            for sample in record.samples:
                if sample.name in out:
                    out[sample.name].append(sample.value)
            duties = record.digital or {}
            for pin in pins:
                out[pin].append(duties.get(pin))
            snaps = record.sensors or {}
            for field, cols in subs.items():
                words = snaps.get(field) or (None,) * len(cols)
                for col, word in zip(cols, words):
                    out[col].append(word)
            out['time'].append(record.start_time)
            out['dt'].append(record.dt)
        return out

    def catalogue(self):
        """Everything this board can put in a record, named."""
        chart = self.board.system.channel_map()
        rows = [{'name': c['signal'], 'kind': 'analog',
                 'direction': c.get('direction', 'in'),
                 'unit': c.get('unit'), 'selectable': True}
                for c in chart['analog']]
        # A GROUP, NOT A CHOICE.
        pins = (self.layout or {}).get('pins') or chart['digital']
        rows += [{'name': p['signal'], 'kind': 'digital',
                  'direction': p.get('direction', 'out'),
                  'unit': 'duty', 'selectable': True}
                 for p in pins]
        rows += [dict(row, selectable=self._sensors_carried())
                 for row in SENSOR_FIELDS]
        return rows

    def _sensors_carried(self):
        """Whether this board puts sensor fields in a DAQ record."""
        return bool((self.state() or {}).get('sensors_supported'))

    @staticmethod
    def _match(name):
        """A name as it compares: case and punctuation do not count."""
        return re.sub(r'[^a-z0-9]', '', str(name).lower())

    def pick(self, *names):
        """Resolve names to the board's own spelling, in the board's order."""
        rows = self.catalogue()
        by_name = {self._match(r['name']): r for r in rows}
        found = {str(name): by_name.get(self._match(name)) for name in names}
        missing = [name for name, row in found.items() if row is None]
        if missing:
            raise RigError(
                'no channel called %s. This board has: %s'
                % (', '.join(repr(m) for m in missing),
                   ', '.join(r['name'] for r in rows if r['selectable'])))
        picked = [row for row in found.values() if row is not None]
        unselectable = next((row for row in picked
                             if not row['selectable']), None)
        if unselectable is not None:
            raise RigError(
                '%r is a %s this board does not put in a record yet - '
                'read it through its own subsystem instead'
                % (unselectable['name'], unselectable['kind']))
        order = [r['name'] for r in rows]
        return sorted({row['name'] for row in picked}, key=order.index)

    def _split(self, names):
        """The picked names as what a record is made of: the analog channels
        for the mask, whether a pin is among them, and the sensor mask.
        """
        kinds = {r['name']: r['kind'] for r in self.catalogue()}
        picked = self.pick(*names)
        analog = [c for c in picked if kinds.get(c) == 'analog']
        if not analog:
            raise RigError(
                'a record needs at least one analog channel - '
                'the pins and the sensors ride along, they do not '
                'make a record on their own')
        bits = {row['name']: b for b, row in enumerate(SENSOR_FIELDS)}
        sensors = sum(1 << bits[c] for c in picked
                      if kinds.get(c) == 'sensor')
        pinned = any(kinds.get(c) == 'digital' for c in picked)
        return analog, pinned, sensors

    def configure(self, *channels, **kw):
        """Set up the acquisition. Replaces whatever was there."""
        # NAMES AS ARGUMENTS, OR A LIST.
        sample_rate = kw.pop('sample_rate', None)
        accumulate = kw.pop('accumulate', None)
        decimate = kw.pop('decimate', 1)
        digital = kw.pop('digital', True)
        clock = kw.pop('clock', 'software')
        sample_time = kw.pop('sample_time', 0)
        records = kw.pop('records', None)
        interval_us = kw.pop('interval_us', None)
        adapt = kw.pop('adapt', False)
        chain = kw.pop('chain', None)             # bessel.design(): boxcar, sections, decimate
        if chain is not None:
            accumulate = chain['boxcar']
        if kw:
            raise TypeError('configure() got %s'
                            % ', '.join(sorted(kw)))
        if len(channels) == 1 and not isinstance(channels[0], str):
            channels = channels[0]        # a list, sliced or whole
        channels = list(channels) if channels else None
        sensors = 0
        if channels is not None:
            channels, pinned, sensors = self._split(channels)
            digital = digital or pinned

        # Stopped first: the board refuses to reconfigure a running task (a
        # stride changing under a half-drained buffer mixes record shapes).
        self.board.daq.stop()

        # AND THE CHAIN CLEARED, for the same reason and the same failure.
        if accumulate is None or chain is not None:
            self.board.daq.shape()

        burst = {}
        if records is not None:
            burst['records'] = records          # a run that ENDS: the burst
        if interval_us is not None:
            burst['interval_us'] = interval_us  # vocabulary, passed through
        if sensors:
            burst['sensors'] = sensors
        self.layout = self.board.daq.configure(
            channels if channels is not None else self.channels(),
            clock=clock, sample_time=sample_time, decimate=decimate,
            accumulate=accumulate, digital=digital, sample_rate=sample_rate,
            adapt=adapt,
            **burst)
        if chain is not None:
            self.shape(chain['sections'], chain['decimate'])
        return self.layout

    def shape(self, sections=(), decimate=1):
        """Load the anti-alias chain `coaxial.acquire.bessel` designed."""
        self.board.daq.shape(sections, decimate)
        return self

    def ladder(self, chains):
        """Load the ladder of chains the board climbs when its ring fills."""
        self.board.daq.ladder(chains)
        return self

    def tone(self, hz=0, rate_hz=0, amplitude=10000, offset=32768, kind=0):
        """A known sequence in the converter's place - for proving the path
        carried every sample, not for measuring anything.
        """
        self.board.daq.tone(hz, rate_hz, amplitude, offset, kind)
        return self

    #: Written by the main loop every few microseconds, so a write to it is
    #: gone before the reply is. Refused rather than accepted and lost.
    LOOP_OWNED = ('KEEPALIVE',)

    def outputs(self):
        """What can be written, asked of the board rather than listed here."""
        pins = [d for d in self.board.system.channel_map()['digital']
                if d['direction'] == 'out'
                and d['signal'] not in self.LOOP_OWNED]
        return {'digital': [d['signal'] for d in pins],
                'analog': ['Phase U', 'Phase V', 'Phase W']}

    def write(self, digital=None, analog=None):
        """Put levels out: named pins, and duties on the three legs."""
        done = {}
        for name, level in (digital or {}).items():
            done[name] = self._write_pin(name, bool(level))
        if analog:
            done.update(self._write_duty(analog))
        return done

    def _write_pin(self, name, level):
        """One named pin, by whichever route the board gives that pin."""
        if name in self.LOOP_OWNED:
            raise RigError('%s is driven by the main loop every few '
                           'microseconds - a write to it would be overwritten '
                           'before the reply came back' % name)

        if name == 'AFE_ON':
            self.board.afe.write(level)
            return level

        pins = {d['signal']: d for d in
                self.board.system.channel_map()['digital']}
        if name not in pins or pins[name]['direction'] != 'out':
            raise RigError('%s is not an output this board reports; it has %s'
                           % (name, ', '.join(self.outputs()['digital'])))

        port, number = pins[name]['pin'][1], int(pins[name]['pin'][2:])
        self.board.gpio.test_mode(True)
        try:
            self.board.gpio.pin_write(port, number, level)
        finally:
            self.board.gpio.test_mode(False)
        return level

    def _write_duty(self, analog):
        """Duties on the three legs, as one all-or-none update."""
        legs = ('Phase U', 'Phase V', 'Phase W')
        unknown = [n for n in analog if n not in legs]
        if unknown:
            raise RigError('%s cannot be written; this board has no DAC and '
                           'its only analog outputs are %s'
                           % (', '.join(unknown), ', '.join(legs)))

        # ONE state read serves the arm check, the period and the held duties.
        state = self.board.gate_drivers.state()
        if not state['pwm_enabled']:
            raise RigError(
                'the gate drivers are not armed, and writing a duty is not what '
                'arms it - call gates.arm() first, which says what that '
                'means. %s'
                % ('The break is latched, so gates.arm(bypass_sto=True) '
                   'is what gets past it'
                   if state['fault']
                   else 'Nothing is holding it off'))

        period = state['period'] - 1
        held = state['duty']
        ticks = tuple(
            int(max(0.0, min(1.0, analog[name])) * period)
            if name in analog else held[i]
            for i, name in enumerate(legs))
        self.board.gate_drivers.duty(ticks)
        return dict(zip(legs, (t / period for t in ticks)))
