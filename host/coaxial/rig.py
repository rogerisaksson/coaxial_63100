"""One board behind one class: connect, configure, trigger, read."""
import re
import sys
import time
import zlib

from . import angle as angle_scaling
from .acquisition import Acquisition
from .board import Board
from .clock import NTP_SERVER, WRAP
from .errors import LINK_FAULTS, CrcError, NoReplyError, RigError
from .gates import GateStage
from .reader import BufferedReader
from .record import Record, build
from .motion import Motion
from . import boot as bootmod
from . import broker
from . import session as sessionmod
from contextlib import suppress

#: Bytes the board leaves for records in one reply - `DAQ_REPLY_ROOM` in
#: `cmd_daq.c`. Named here because it decides how many records a single
#: transaction is worth waiting for.
REPLY_ROOM = 240


def _subsystem_names():
    """The board's subsystem names, off its declaration, plus `gates` and
    `daq`.
    """
    return frozenset(Board.parts()) | {'gates', 'daq'}


class Later:

    """A subsystem named before its session is open."""

    def __init__(self, device, name):
        self._device, self._name = device, name

    def open(self):
        """Open the device this handle belongs to. Returns the live handle."""
        self._device.open()
        return getattr(self._device, self._name)

    def _live(self):
        if self._device.board is None:
            raise RigError(
                '%s is a handle on a session that is not open yet - '
                'open() on it, or on the device, is what makes it live'
                % self._name)
        return getattr(self._device, self._name)

    def __getattr__(self, attr):
        return getattr(self._live(), attr)

    def __repr__(self):
        if self._device.board is None:
            return ('<%s of a session not yet open - open() opens it>'
                    % self._name)
        return repr(self._live())


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

#: What the acquisition front door answers. A whitelist, not everything:
#: `daq.write` reaching the pin writer would put the device vocabulary
#: behind the wrong name.
DAQ_DOOR = ('configure', 'shape', 'ladder', 'tone', 'start', 'stop',
            'state', 'acquire', 'latest', 'blocks', 'read_buffer',
            'buffered', 'channels', 'outputs', 'catalogue', 'pick', 'read',
            'configure_buffer', 'capture', 'enable', 'disable',
            'channel_names', 'columns', 'series', 'frame', 'frames',
            'history')


class DaqView:

    """The acquisition front door, as its own handle."""

    def __init__(self, device):
        self._device = device

    def open(self):
        """Open the device this handle belongs to."""
        self._device.open()
        return self

    def close(self):
        """End the acquisition: the task stopped, what it buffered still
        readable.
        """
        self._device.stop()
        return self

    @property
    def layout(self):
        return self._device.layout

    def __getattr__(self, name):
        if name in DAQ_DOOR:
            return getattr(self._device, name)
        raise AttributeError(
            '%r is not part of the acquisition front door. It has: open, '
            'close, layout, %s' % (name, ', '.join(DAQ_DOOR)))

    def __enter__(self):
        """Start the task, and stop it on the way out however that goes."""
        self._device.start()
        return self

    def __exit__(self, *_):
        self._device.stop()
        return False

    def __repr__(self):
        return ('<the acquisition front door - configure(), then `with` it '
                'or start(); read()/series()/columns(); stop(), close()>')


class Coaxial63100(Acquisition):

    """One board, one acquisition task, one clock."""

    def __init__(self, port='COM4', baud=115200, unit=1, link='auto',
                 simulated_device=False, power_afe=False, own_image=True):
        """Say where the board is. Nothing is opened until `open()`, which
        makes a real board run this host's own build (`own_image`)."""
        self.port = port
        self.baud = baud
        self.unit = unit
        self.link = link
        self.simulated_device = simulated_device
        self.power_afe = power_afe
        self.own_image = own_image
        #: (path of this host's build, whether open() loaded it), or None.
        self.image_loaded = None

        self.session = None
        self._board = None
        # No `self.gates = None` here: before open() the name goes through
        # __getattr__ like every subsystem, so `stage = device.gates` binds a
        # Later whose open() opens the device.
        self.daq = DaqView(self)
        # Motion, the same way: bound before open() so `device.motion` reads
        # like `device.daq`, opened lazily by its factories.
        self.motion = Motion(self)
        self._origin = None
        self.simulated = simulated_device
        self.layout = None
        self.sync = None
        # The stamps' wrap count, carried from block to block - `_epoch` - and
        # the last stamp itself, for a block of one record's dt.
        self._last_raw = None
        self._epoch = 0
        self._last_stamp = None
        self._afe_was_on = None
        #: Whether this session holds a reference on the AFE rail, so
        #: close() releases exactly what it took and no more.
        self._afe_held = False
        # The host-side reader, alive only between start() and stop().
        self._reader = None
        self._buffer_records = self.BUFFER_RECORDS
        self._history = []
        self._asked_done = 0.0
        self._done_seen = 0
        self._cursor = 0
        self._lost = 0

    # -- opening and closing --------------------------------------------

    def open(self):
        """Open the link, bring the supply up, and hand back self."""
        if self.session is not None:
            return self

        simulated = True if self.simulated_device else (
            None if self.link == 'auto' else False)

        self.session, self._origin = sessionmod.open_session(
            self.port, baud=self.baud, unit=self.unit, simulated=simulated)
        self._board = self.session.board
        self.gates = GateStage(self.board)
        # THE WAY BACK.
        self.board.rig = self
        self.simulated = not self.origin.real
        if self.own_image and not self.simulated:
            self._own_image()

        if self.power_afe:
            self._take_afe()
        return self

    def _own_image(self):
        """THE HOST'S OWN BUILD ON THE BOARD (docs/BOOT.md): a board running
        another image takes this host's through its bootloader, into RAM and
        its store - once per build - so nothing is asked of a firmware this
        host was not built with. A board others share is not reset under
        them: that is refused in words."""
        found = bootmod.host_image()
        if found is None:
            return                        # no build at hand: nothing to compare
        path, image = found
        want = (len(image), zlib.crc32(image))
        running = bootmod.stale(self.board, image)
        if running is None:
            self.image_loaded = (path, False)
            return
        if self.origin.label.endswith('- shared'):
            raise RigError('unit %d runs another image (%s) than this host\'s build %s '
                           '(%d B, crc %08x), and other sessions share the board - close '
                           'them, and the next open loads it'
                           % (self.unit, running, path, want[0], want[1]))
        print('coaxial: unit %d runs image %s; loading this host\'s build %s (%d B, crc '
              '%08x) through its bootloader' % ((self.unit, running, path) + want),
              file=sys.stderr)
        bootmod.load(self.board, image)
        self.board.probe()
        self.image_loaded = (path, True)

    #: What the parts need after their rail comes up before anything talks
    #: to them. Enabling and configuring in the same breath answered SERVER
    #: DEVICE FAILURE.
    AFE_SETTLE = 0.3

    def _take_afe(self):
        """This session's own reference on the rail, and the parts' settle when
        it was this that switched the rail on.
        """
        already = self.board.afe.is_on()
        self.board.afe.enable()
        self._afe_held = True
        if not already:
            time.sleep(self.AFE_SETTLE)

    def __getattr__(self, name):
        """`device.imu` is `device.board.imu`, and it can be NAMED early."""
        if name.startswith('_') or name in ('board', 'session'):
            raise AttributeError(name)

        board = self.__dict__.get('_board')
        if board is None:
            return self._later(name)
        try:
            return getattr(board, name)
        except AttributeError:
            raise AttributeError(
                '%r is not a subsystem of this board. It has: %s'
                % (name, ', '.join(sorted(
                    n for n in vars(board) if not n.startswith('_')))))

    def _later(self, name):
        """A handle on a subsystem named before open(), checked against the
        board's declaration so a typo fails at the binding.
        """
        if name in _subsystem_names():
            return Later(self, name)
        raise AttributeError(
            '%r is not a subsystem of this board. It has: %s'
            % (name, ', '.join(sorted(_subsystem_names()))))

    def _others_here(self):
        """Whether another session is on this board. False if unknowable."""
        if self.simulated:
            return False
        try:
            count = broker.clients()          # None: nobody is serving
        except LINK_FAULTS + (ValueError,):   # the socket, the address file
            return False
        return count is not None and count > 1

    def close(self):
        """Everything this session started, undone."""
        if self._board is not None:
            # One try per step.
            for step in (self.board.daq.stop, self._release_stage,
                         self._release_afe):
                with suppress(RigError):
                    step()
        if self.session is not None:
            self.session.close()
        self.session = self._board = None
        vars(self).pop('gates', None)      # back to a Later, reopenable

    def _release_stage(self):
        """Disarm on the way out - the safety net for a run that was killed."""
        if (self.gates is None or self.gates.armed_here
                or not self._others_here()):
            self.board.gate_drivers.disable()

    def _release_afe(self):
        """Release OUR reference; the refcount keeps the rail up for whoever
        else holds it.
        """
        if not self._afe_held:
            return
        self.board.afe.disable()
        self._afe_held = False

    def __enter__(self):
        return self.open()

    def __exit__(self, *_):
        self.close()

    @property
    def board(self):
        """The board behind the session: what every subsystem holds and
        every page reads.
        """
        if self._board is None:
            raise RigError('the rig is not open - open() first, or use it '
                           'as a context manager')
        return self._board

    @property
    def origin(self):
        """Where the board was reached: `open_session`'s Origin, with its
        `interface`, `label` and `real`.
        """
        if self._origin is None:
            raise RigError('the rig is not open - open() first, or use it '
                           'as a context manager')
        return self._origin

    def __repr__(self):
        where = self._origin.label if self._origin else 'not open'
        return '<Coaxial63100 %s%s>' % (
            where, ' SIMULATED' if self.simulated else '')

    # -- the clock -------------------------------------------------------

    def set_time_from_pc(self, seconds=3.0, reference='utc',
                         ntp_server=NTP_SERVER):
        """Tie the board's cycle counter to a real clock."""
        self.sync = self.board.clock.sync(
            seconds=seconds, reference=reference,
            ntp_server=ntp_server)
        return self.sync

    # -- the acquisition task --------------------------------------------

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
        # subsystems use: the shaft through coaxial.angle, the quaternion out
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
        if accumulate is None:
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
        return self.layout

    def shape(self, sections=(), decimate=1):
        """Load the anti-alias chain `coaxial.bessel` designed."""
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

    #: Records the host keeps when nobody has said. Ten thousand at a
    #: fifty-byte stride is half a megabyte, which is nothing at this end
    #: of the link and several seconds of headroom at the other.
    BUFFER_RECORDS = 10000

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
            self.board.afe.enable()
            self._afe_held = True
        return self

    def disable(self):
        """Release this session's hold on the front end."""
        if self._afe_held:
            self.board.afe.disable()
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

    # -- reading ---------------------------------------------------------

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
            self.board.afe.set(level)
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

    def read(self, count=-1):
        """Records, waiting for them. NEGATIVE means everything there is."""
        out = []
        if count == 0:
            return out
        for block in self.read_buffer(-1):
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

    def read_buffer(self, count):
        """`count` blocks off the HOST buffer, one at a time."""
        if self._reader is None:
            raise RigError('nothing is buffering yet - start() puts the '
                           'reader on the link, and read_buffer() takes '
                           'what it has collected')
        seen = 0
        self._done_seen = 0
        while count < 0 or seen < count:
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
