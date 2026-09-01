"""One board behind one class: connect, configure, trigger, read.

    from coaxial import Coaxial63100

    with Coaxial63100(port='COM4') as daq:
        daq.set_time_from_pc()
        daq.configure(['Phase U', 'NTC'], accumulate=8)
        daq.start()
        for block in daq.blocks(20):
            print(block[-1]['time'], block[-1]['NTC'] / block[-1]['samples'])

The front door. `Board` and its subsystems stay reachable under `daq.board`,
but a caller wanting measurements should not have to know the supply lives in
`afe`, the converters in `daq`, the counter in `clock` and the gates in `gate
drivers` - nor the order to touch them in.

It owns the preflight every view was writing out again: AFE_ON powers the ADC
reference and both SPI parts, so it goes on for a reading to mean anything
(invariant 9) and back the way it was found afterwards - a board left powered
because a script ended is a change nobody asked for.

Nothing here judges a reading. Raw codes and the board's own units;
`board.analog` has the conversions.
"""
import re
import time

from .acquisition import Acquisition
from .clock import NTP_SERVER, unwrap
from .errors import CrcError, NoReplyError, RigError
from .gates import GateStage
from .reader import BufferedReader
from .record import build

#: Bytes the board leaves for records in one reply - `DAQ_REPLY_ROOM` in
#: `cmd_daq.c`. Named here because it decides how many records a single
#: transaction is worth waiting for.
REPLY_ROOM = 240


_KNOWN_SUBSYSTEMS = None


def _subsystem_names():
    """The board's subsystem names, off the stand-in, plus `gates` and `daq`.

    Read once from SimulatedSession rather than written down: the simulated
    board carries the same names as the real one by construction - the
    parity suite is what holds the two together - so a handle named before
    open() is checked against the same set that will answer after.
    """
    global _KNOWN_SUBSYSTEMS
    if _KNOWN_SUBSYSTEMS is None:
        from .simulated import SimulatedSession

        board = SimulatedSession().board
        _KNOWN_SUBSYSTEMS = frozenset(
            name for name in vars(board)
            if not name.startswith('_')) | {'gates', 'daq'}
    return _KNOWN_SUBSYSTEMS


class Later:

    """A subsystem named before its session is open.

    `imu = device.imu` reads best at the top of a script, next to the
    device it belongs to - but before open() there is no board to reach.
    This stands in: open() opens the device, and every attribute after
    that resolves against the live subsystem.
    """

    def __init__(self, device, name):
        self._device, self._name = device, name

    def open(self):
        """Open the device this handle belongs to. Returns the live handle."""
        self._device.open()
        return getattr(self._device, self._name)

    def _live(self):
        if self._device.board is None:
            from .errors import RigError
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


#: Everything the board can measure that is not an ADC channel or a pin.
#: Listed here rather than read off the board because it is the ONE thing
#: in this file that the wire does not carry yet - `catalogue()` marks
#: them unselectable until the firmware puts them in a record, so a caller
#: sees the name and the reason rather than a silent nothing.
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

#: What the acquisition front door answers. A whitelist, not everything:
#: `daq.write` reaching the pin writer would put the device vocabulary
#: behind the wrong name.
DAQ_DOOR = ('configure', 'shape', 'ladder', 'tone', 'start', 'stop',
            'state', 'acquire', 'latest', 'blocks', 'read_buffer',
            'buffered', 'channels', 'outputs', 'catalogue', 'pick', 'read',
            'configure_buffer', 'tare', 'compensate',
            'channel_names', 'columns', 'series', 'frame', 'frames')


class DaqView:

    """The acquisition front door, as its own handle.

    The device owns the lifecycle - it stops before reconfiguring, keeps
    the layout, puts times and the sample count on records - and this is
    that same vocabulary behind the name `daq`, so a script reads
    subsystem-first like the sensor examples do. The raw ops stay at
    `device.board.daq`.
    """

    def __init__(self, device):
        self._device = device

    def open(self):
        """Open the device this handle belongs to."""
        self._device.open()
        return self

    def close(self):
        """End the acquisition: the task stopped, what it buffered still
        readable. The port is the device's to close."""
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
        """Start the task, and stop it on the way out however that goes.

            with device.daq as daq:
                rec = daq.read(-1)

        The bracket a task wants, and not for tidiness: a script that dies
        between start() and stop() leaves the board sampling, and the next
        run is refused with "a task is running - stop it first" until
        somebody clears it by hand. That happened repeatedly while this
        library was being written, which is the argument for it.
        """
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
                 simulated_device=False, power_afe=False):
        """Say where the board is. Nothing is opened until `open()`.

        port              the serial port: 'COM4' on Windows, something
                          like '/dev/ttyACM0' on Linux.
        baud              bits per second. 115200 is the debug probe's
                          virtual COM port; an RS485 segment can be faster.
        unit              the Modbus address. One board on a bench is 1.
        link              'auto' finds a board on any port when `port` does
                          not answer. 'port' takes you at your word.
        simulated_device  no cable at all. Every value is invented, and
                          `self.simulated` says so - a number from nowhere
                          and one from hardware must never look alike.
        power_afe         switch AFE_ON for the session if it is off, and

                          FALSE BY DEFAULT. It was true, so every rig that
                          opened switched the rail and every one that closed
                          switched it back - connect, disconnect, connect,
                          and AFE_ON drives an LED. A script that needs the
                          analog front end says so; one that only reads
                          counters no longer touches it at all.
                          switch it back on the way out. Off leaves the
                          supply exactly as found, and every reading then
                          means whatever the supply was doing.
        """
        self.port = port
        self.baud = baud
        self.unit = unit
        self.link = link
        self.simulated_device = simulated_device
        self.power_afe = power_afe

        self.session = None
        self.board = None
        # No `self.gates = None` here: before open() the name goes through
        # __getattr__ like every subsystem, so `stage = device.gates` binds
        # a Later whose open() opens the device. open() below sets the real
        # GateStage over it.
        self.daq = DaqView(self)
        self.origin = None
        self.simulated = simulated_device
        self.layout = None
        self.sync = None
        self._afe_was_on = None
        # The host-side reader, alive only between start() and stop().
        self._reader = None
        self._buffer_records = self.BUFFER_RECORDS
        self._cursor = 0
        self._lost = 0

    # -- opening and closing --------------------------------------------

    def open(self):
        """Open the link, bring the supply up, and hand back self."""
        from coaxial_mcp.session import open_session

        simulated = True if self.simulated_device else (
            None if self.link == 'auto' else False)

        self.session, self.origin = open_session(
            self.port, baud=self.baud, unit=self.unit, simulated=simulated)
        self.board = self.session.board
        self.gates = GateStage(self.board)
        self.simulated = not self.origin.real

        if self.power_afe:
            already = self.board.afe.is_on()
            # ALWAYS take our own reference, even when the rail is already
            # up: the rail is refcounted, and a session that merely
            # observed it on held NOTHING - the other holder let go and the
            # rail dropped mid-view. Measured 2026-08-29: the meter bridge
            # opened onto a lit rail and its configure was refused with
            # 'AFE_ON is off' one round trip later.
            self.board.afe.enable()
            if not already:
                # The parts need their supply up before anything talks to
                # them. Enabling and configuring in the same breath answered
                # SERVER DEVICE FAILURE.
                time.sleep(0.3)
        return self

    def __getattr__(self, name):
        """`device.imu` is `device.board.imu`, and it can be NAMED early.

        Forwarded rather than ten properties: the subsystem names come from
        Board, so a new one is reachable here with nothing added. A list in
        this file would be the second answer that goes stale - which is what
        happened to test_parity's hand-written table of view calls.

        Before open() the name comes back as a `Later`: a bound handle whose
        open() opens the device, so an example reads handle-first -
        `imu = device.imu` then `imu.open()`. The name is still checked
        against the board's own set, off the stand-in, so a typo fails at
        the binding and not at the first call.

        `gates` after open is the real attribute holding GateStage, the
        arming policy - reaching past it to `gate_drivers` is how a duty
        write becomes what arms a stage.
        """
        if name.startswith('_') or name in ('board', 'session'):
            raise AttributeError(name)

        board = self.__dict__.get('board')
        if board is None:
            if name in _subsystem_names():
                return Later(self, name)
            raise AttributeError(
                '%r is not a subsystem of this board. It has: %s'
                % (name, ', '.join(sorted(_subsystem_names()))))
        try:
            return getattr(board, name)
        except AttributeError:
            raise AttributeError(
                '%r is not a subsystem of this board. It has: %s'
                % (name, ', '.join(sorted(
                    n for n in vars(board) if not n.startswith('_')))))

    def _others_here(self):
        """Whether another session is on this board. False if unknowable.

        False and not True on failure: without a broker there is nobody else
        by construction, and an unknown answer must not be what stops a
        stage being disarmed.
        """
        from . import broker

        try:
            count = broker.clients()          # None: nobody is serving
        except Exception:                                     # noqa: BLE001
            return False
        return count is not None and count > 1

    def close(self):
        """Everything this session started, undone.

        The task first: one left running keeps the converters busy after the
        script that asked for them has gone. Then the supply, but only if
        this session was what switched it on.
        """
        if self.board is not None:
            # One try per step. They were in one block, and a RigError from
            # daq.stop() or gate_drivers.disable() then skipped the supply
            # restore - leaving AFE_ON high, which on this board takes the
            # gate drivers' supply away. A switching run started after that
            # toggles TIM1 into unpowered drivers and heats nothing, with
            # every counter reading normal. Measured 2026-08-28.
            for step in (self.board.daq.stop,):
                try:
                    step()
                except RigError:
                    pass

            # THE STAGE IS THE BOARD'S, NOT THIS SESSION'S. Disarming on the
            # way out is the safety net for a run that was killed, and it
            # stays that - but with a broker every session shares one board,
            # and this used to run unconditionally. Measured 2026-08-29:
            # three switching runs ended the moment a second session asked
            # the board an unrelated question, MOE clear and no fault, which
            # reads as a stage tripping rather than a peer tidying up.
            #
            # So it undoes what this session started, and otherwise only
            # when nobody else is left to own it.
            try:
                if (self.gates is None or self.gates.armed_here
                        or not self._others_here()):
                    self.board.gate_drivers.disable()
            except RigError:
                pass
            try:
                if self.power_afe:
                    # Release OUR reference; the refcount keeps the rail up
                    # for whoever else holds it.
                    self.board.afe.disable()
            except RigError:
                pass                    # closing is not the place to raise
        if self.session is not None:
            self.session.close()
        self.session = self.board = None
        self.__dict__.pop('gates', None)   # back to a Later, reopenable

    def __enter__(self):
        return self.open()

    def __exit__(self, *_):
        self.close()

    def __repr__(self):
        where = self.origin.label if self.origin else 'not open'
        return '<Coaxial63100 %s%s>' % (
            where, ' SIMULATED' if self.simulated else '')

    # -- the clock -------------------------------------------------------

    def set_time_from_pc(self, seconds=3.0, reference='utc',
                         ntp_server=NTP_SERVER):
        """Tie the board's cycle counter to a real clock.

        The board has no clock of its own - no RTC, no LSE - so every
        timestamp it gives you is a raw cycle count. This measures where
        that counter was and how fast it really runs, and after it every
        record from `read()` carries a wall-clock `time`.

        `reference='utc'` still goes through this PC, but measures the PC's
        own offset and rate against NTP over the same window and takes both
        out. Worth doing: on 2026-08-27, six minutes after W32Time had
        synced, this machine sat 947 ms behind UTC and was losing a further
        25 ppm. `'pc'` ties it to this machine as it stands. With no
        network, `'utc'` becomes `'pc'` and the Sync says so.

        Longer `seconds` buys a better rate: 3 s bounds it at parts per
        thousand, 300 s resolves a few per million.
        """
        self.sync = self.board.clock.sync(
            seconds=seconds, reference=reference,
            ntp_server=ntp_server)
        return self.sync

    # -- the acquisition task --------------------------------------------

    def channels(self):
        """What the board says it has. Not a list written down here."""
        return self.board.analog.names()

    def channel_names(self, record=None):
        """What the records carry, in the order they carry it.

        With no argument it answers off the CONFIGURED TASK, so a caller
        can write its header before the first record has arrived. With a
        record it answers that record's, which is the same list whenever
        the task has not been reconfigured under it - and is not, when it
        has, which is the reason to be able to ask a record directly.

            names = daq.channel_names()          # before reading
            names = daq.channel_names(values[0]) # off what arrived
        """
        if record is not None:
            got = getattr(record, 'channel_name', None)
            if got is not None:
                return list(got)
            # A plain mapping - from `board.daq` rather than the front
            # door, or one a caller built. The layout's order is what
            # makes it a sequence rather than whatever the dict holds.
            return [f['signal'] for f in (self.layout or {}).get('fields') or []
                    if f['signal'] in record]
        return [f['signal']
                for f in (self.layout or {}).get('fields') or []]

    def series(self, records, name):
        """One channel out of a run, as a plain list of means.

        THE COMMON CASE, and the terse one:

            rec = daq.read(-1)
            t   = daq.series(rec, 'time')
            ntc = daq.series(rec, 'ntc')
            for i in range(len(ntc)):
                print(t[i], ntc[i])

        `columns()` is the whole table and wants a dict to hold it; this is
        one column and wants nothing. `time` and `dt` are spellings too, so
        a plot's two axes come out the same way. The name matches the way
        `pick()` matches - case and punctuation do not count - because a
        long channel name is what makes this shape worth having.
        """
        if not records:
            return []
        want = self._match(name)
        if want in ('time', 'starttime'):
            return [r.start_time for r in records]
        if want == 'dt':
            return [r.dt for r in records]
        spelling = None
        for s in getattr(records[0], 'samples', ()):
            if self._match(s.name) == want:
                spelling = s.name
                break
        if spelling is None:
            # A pin, then. Same records, same window, a duty instead of a
            # mean - and named as loosely as everything else here.
            for pin in (getattr(records[0], 'digital', None) or {}):
                if self._match(pin) == want or self._match(
                        pin.split('/')[-1]) == want:
                    return [(getattr(r, 'digital', None) or {}).get(pin)
                            for r in records]
        if spelling is None:
            raise RigError(
                'no channel called %r in these records. They have: %s'
                % (name, ', '.join(records[0].channel_name)))
        return [r.value(spelling) for r in records]

    def frame(self, records, index='time', scaled=False):
        """A run as a pandas DataFrame: one column per channel.

            df = daq.frame(daq.read(-1))
            df['NTC'].rolling(50).mean().plot()
            df.describe()

        `index` is 'time' for the wall clock, 'elapsed' for seconds from
        the first record, or None to keep a plain range. The columns are
        the board's own channel names, so a notebook reads the same names
        `catalogue()` listed and `configure()` took.

        pandas is NOT a dependency of this library and is imported here
        rather than at the top: a bench that only reads a thermistor
        should not have to install it, and the refusal below says what to
        do rather than raising ImportError at a call three frames up.
        """
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
        if index == 'elapsed' and cols['time'] and cols['time'][0] is not None:
            frame['elapsed'] = [t - cols['time'][0] for t in cols['time']]
            return frame.set_index('elapsed')
        if index == 'time' and cols['time'] and cols['time'][0] is not None:
            # A real timestamp rather than a float, so resample() and the
            # rest of the time machinery work without a conversion the
            # caller has to remember.
            frame['time'] = pandas.to_datetime(frame['time'], unit='s')
            return frame.set_index('time')
        return frame

    def _in_units(self, cols):
        """Real-unit columns beside the codes, named `Phase U (A)`.

        The board's OWN converters (invariant 7): every scaling lives in
        the calibration record, so this asks `board.analog` rather than
        holding a constant. Codes stay in the frame under their own names
        - what arrived and what it means are two columns, not one that
        quietly became the other.
        """
        scale = self.board.analog.scaling()
        pick = {'centi-degC': ('ntc', 'celsius', 'C'),
                'mA': ('phase', 'amps', 'A'),
                'mV': ('dcbus', 'volts', 'V')}
        # THE CHANNEL'S OWN ZERO AND GAIN, from the calibration
        # record - what `tare()` wrote. Without this a tare stored an
        # offset the board kept and no column ever used, so the
        # currents came back exactly as they went in and nothing said
        # the call had done nothing.
        trim = {c['index']: c for c in
                self.board.calibration.read()['channels']}
        out = {}
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

    def frames(self, window=2.0, seconds=None, scaled=False, **kw):
        """A rolling window of the last `window` seconds, as it arrives.

            with daq:
                for df in daq.frames(window=2.0, seconds=10, scaled=True):
                    redraw(df)

        The bookkeeping a live plot was doing by hand: a None sentinel, a
        concat per turn, a trim by index, and the whole window rescaled
        every frame. It belongs here - the window is records, not frames,
        so nothing is concatenated and nothing grows, and a plot that
        forgets to trim cannot become the bottleneck that fills the ring.

        Ends after `seconds`, or when the task does, or when the caller
        breaks out.
        """
        import time as _t

        held = []
        began = _t.time()
        while seconds is None or _t.time() - began < seconds:
            got = self.read(-1)
            if not got:
                if self.state().get('done'):
                    return
                continue
            held.extend(got)
            edge = held[-1].start_time
            if edge is not None and window:
                held = [r for r in held
                        if r.start_time is None or r.start_time > edge - window]
            yield self.frame(held, scaled=scaled, **kw)

    def columns(self, records):
        """Records as columns: {name: values}, plus `time` and `dt`.

        THE OTHER WAY ROUND. A record is a struct and a run is an array of
        them, which is the shape the link delivers; anything that plots or
        fits wants one array per channel. This is that flip, and it is here
        rather than in a caller because the order comes from the layout and
        the mean comes from the count - two things a caller would have to
        get right the same way every time.

            cols = daq.columns(daq.read(-1))
            plot(cols['time'], cols['Phase U'])
        """
        names = self.channel_names(records[0] if records else None)
        # THE PINS ARE COLUMNS TOO. They ride the same records as the
        # analog fields, so every point on both is the SAME window - which
        # is the whole reason to plot a gate against a phase current. They
        # were dropped here, and a live plot of the switches came back
        # empty with nothing saying why.
        pins = list((getattr(records[0], 'digital', None) or {})
                    if records else {})
        out = {name: [] for name in names + pins}
        out['time'] = []
        out['dt'] = []
        for record in records:
            for sample in getattr(record, 'samples', ()):
                if sample.name in out:
                    out[sample.name].append(sample.value)
            duties = getattr(record, 'digital', None) or {}
            for pin in pins:
                out[pin].append(duties.get(pin))
            out['time'].append(getattr(record, 'start_time', None))
            out['dt'].append(getattr(record, 'dt', None))
        return out

    def catalogue(self):
        """Everything this board can put in a record, named.

        THE BOARD'S OWN LIST, not one written here: the analog channels and
        the sampled pins come off `0x6D`, so a board that grows a channel
        grows an entry and nothing above it is told twice.

        Each row is `{'name', 'kind', 'direction', 'unit', 'selectable'}`.
        `kind` is 'analog', 'digital' or 'sensor'; `selectable` says whether
        `configure()` can ask for it today - a sensor the firmware does not
        yet put in a record is listed and refused, which is a better answer
        than a name that silently does nothing.
        """
        chart = self.board.system.channel_map()
        rows = [{'name': c['signal'], 'kind': 'analog',
                 'direction': c.get('direction', 'in'),
                 'unit': c.get('unit'), 'selectable': True}
                for c in chart['analog']]
        # A GROUP, NOT A CHOICE. The board puts every sampled pin in a
        # record or none of them, so picking one picks them all -
        # selectable, and `configure` turns the group on.
        rows += [{'name': p['signal'], 'kind': 'digital',
                  'direction': p.get('direction', 'out'),
                  'unit': 'duty', 'selectable': True}
                 for p in chart['digital']]
        rows += [dict(row, selectable=self._sensors_carried())
                 for row in SENSOR_FIELDS]
        return rows

    def _sensors_carried(self):
        """Whether this board puts sensor fields in a DAQ record.

        The IMU and the shaft angle are readable through their own
        subsystems on every board here; carrying them INSIDE a record is a
        wire format the firmware has to have. Asked of the board rather
        than assumed, so this file does not have to know which builds do.
        """
        return bool((self.state() or {}).get('sensors_supported'))

    @staticmethod
    def _match(name):
        """A name as it compares: case and punctuation do not count.

        `phaseU`, `Phase U` and `phase_u` are the same channel, because a
        caller typing a name into a script should not have to reproduce
        the board's spacing.
        """
        return re.sub(r'[^a-z0-9]', '', str(name).lower())

    def pick(self, *names):
        """Resolve names to the board's own spelling, in the board's order.

        Raises with what it does have when a name is not one of them: a
        list of channels is exactly the kind of thing a caller gets one
        character wrong, and the board's list is the answer.
        """
        rows = self.catalogue()
        by_name = {self._match(r['name']): r for r in rows}
        wanted, missing = [], []
        for name in names:
            row = by_name.get(self._match(name))
            if row is None:
                missing.append(str(name))
            elif not row['selectable']:
                raise RigError(
                    '%r is a %s this board does not put in a record yet - '
                    'read it through its own subsystem instead'
                    % (row['name'], row['kind']))
            else:
                wanted.append(row)
        if missing:
            raise RigError(
                'no channel called %s. This board has: %s'
                % (', '.join(repr(m) for m in missing),
                   ', '.join(r['name'] for r in rows if r['selectable'])))
        order = [r['name'] for r in rows]
        return sorted({r['name'] for r in wanted}, key=order.index)

    def configure(self, *channels, **kw):
        """Set up the acquisition. Replaces whatever was there.

        channels    names, e.g. ['Phase U', 'NTC']. None takes all of them.
        sample_rate records a second the HOST gets. The converter is not
                    slowed to it - it runs flat out and the board sums
                    into a record the clock closes, so a rate the link
                    can drain costs no samples. None lets the board
                    choose what the link carries, which is the safe
                    default.
        accumulate  close each record on this many samples instead, and
                    let `sample_rate` gate the triggers. Unset it follows
                    `sample_rate`.
        decimate    keep one sample in N and discard the rest. Prefer
                    `accumulate`; it keeps what this loses.
        digital     put the board's digital pins in every record.
        clock       'software' for the main loop, or 'tim1' for one record
                    per PWM period. 'tim1' carries only the three phases.
        sample_time 0..7, the converter's own sampling window, shortest
                    first.
        """
        # NAMES AS ARGUMENTS, OR A LIST. Both read well and neither is
        # ambiguous: `configure('phaseU', 'NTC')` for a script written
        # by hand, `configure(daq.channels()[:5])` for one that took
        # the board's own list and sliced it.
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
        if channels is not None:
            kinds = {r['name']: r['kind'] for r in self.catalogue()}
            channels = self.pick(*channels)
            # A pin among the names turns the group on, and the pins
            # ride the record as one - they are not analog fields
            # and do not go in the channel mask.
            if any(kinds.get(c) == 'digital' for c in channels):
                digital = True
            channels = [c for c in channels
                        if kinds.get(c) == 'analog']
            if not channels:
                raise RigError(
                    'a record needs at least one analog channel - '
                    'the pins ride along, they do not make a record '
                    'on their own')

        # Stopped first, because the board refuses to reconfigure under a
        # running task - a stride changing beneath a half-drained buffer
        # hands out records of two shapes - and a caller reaching for
        # configure wants the new shape either way. A script that died
        # holding one otherwise leaves the next one unable to start.
        self.board.daq.stop()

        # AND THE CHAIN CLEARED, for the same reason and the same failure.
        # A clock-closed record and a filter are alternatives the board
        # refuses to hold at once - a fixed decimation needs a fixed rate,
        # and a window's length is whatever the loop managed. A view that
        # loaded a chain and exited leaves one behind, and the next plain
        # `configure(sample_rate=...)` is refused by a filter its caller
        # never asked for. MEASURED 2026-09-01: the README example, run
        # verbatim after METER BRIDGE, raised on exactly that. A caller
        # loading a chain passes `accumulate`, so this leaves those alone.
        if accumulate is None:
            self.board.daq.shape()

        burst = {}
        if records is not None:
            burst['records'] = records          # a run that ENDS: the burst
        if interval_us is not None:
            burst['interval_us'] = interval_us  # vocabulary, passed through
        self.layout = self.board.daq.configure(
            channels if channels is not None else self.channels(),
            clock=clock, sample_time=sample_time, decimate=decimate,
            accumulate=accumulate, digital=digital, sample_rate=sample_rate,
            adapt=adapt,
            **burst)
        return self.layout

    def shape(self, sections=(), decimate=1):
        """Load the anti-alias chain `coaxial.bessel` designed.

        The chain's boxcar is the task's `accumulate`, so configure with
        `accumulate=chain['boxcar']` and pass the sections and
        `chain['decimate']` here. No arguments clears it.
        """
        self.board.daq.shape(sections, decimate)
        return self

    def ladder(self, chains):
        """Load the ladder of chains the board climbs when its ring fills.

        Every rung is a whole design, so climbing one is still an
        anti-alias filter and not just fewer samples. Ask for
        `configure(adapt=True)` and the board does the choosing;
        `state()['rung']` says which it chose.
        """
        self.board.daq.ladder(chains)
        return self

    def tone(self, hz=0, rate_hz=0, amplitude=10000, offset=32768, kind=0):
        """A known sequence in the converter's place - for proving the
        path carried every sample, not for measuring anything.

        `kind` 0 is a sine at `hz`; 1 is a ramp, `offset + (n * hz) mod
        amplitude`, which a host computes in closed form so every record
        can be checked exactly. `hz=0` puts the converter back.
        """
        self.board.daq.tone(hz, rate_hz, amplitude, offset, kind)
        return self

    #: Records the host keeps when nobody has said. Ten thousand at a
    #: fifty-byte stride is half a megabyte, which is nothing at this end
    #: of the link and several seconds of headroom at the other.
    BUFFER_RECORDS = 10000

    def compensate(self, name, gain=None, offset=None, save=True):
        """Write one channel's gain and offset into the calibration record.

            daq.compensate('phaseU', gain=1.002, offset=-7155)

        CLASSIC OFFSET AND GAIN, in the order the board applies them:
        `(code - offset) * gain`. `gain` is a plain multiplier here and
        parts per million on the wire, because 1.002 is what an operator
        means and 2000 is what the record stores.

        Either may be left out to keep what the channel already has - a
        span that must not disturb a zero, or the other way round.

        THE BOARD KEEPS IT. It goes in the calibration record behind
        `0x6E` device 3, where invariant 7 says every conversion lives, so
        the next session and every other host read the same channel the
        same way. `save=False` holds it in RAM for this session only;
        `save=True` commits the whole record to flash.

        Returns `{'offset_raw', 'gain_ppm'}` as stored.
        """
        spelling = self.pick(name)[0]
        index = {c['signal']: c['index']
                 for c in self.board.system.channel_map()['analog']}[spelling]
        was = {c['index']: c for c in
               self.board.calibration.read()['channels']}.get(index, {})

        offset_raw = (was.get('offset_raw') or 0 if offset is None
                      else int(round(offset)))
        gain_ppm = (was.get('gain_ppm') or 0 if gain is None
                    else int(round((float(gain) - 1.0) * 1e6)))

        self.board.calibration.set_channel(index, offset_raw, gain_ppm)
        if save:
            self.board.calibration.save()
        return {'offset_raw': offset_raw, 'gain_ppm': gain_ppm}

    def tare(self, *names, **kw):
        """Zero the current channels: what they read now becomes zero.

            daq.tare('phaseU', auto=True, save=False)
            daq.tare()                    # every current channel, saved

        A MEASUREMENT AND THEN A `compensate()`. With `auto` it reads the
        channel here and writes what it read as the offset; with
        `auto=False` it asks the board to do both in one op, which is what
        `cal.zero()` is - the same answer, one round trip, and no window in
        which the host holds a number the board has not agreed to.

        NOTHING HERE KNOWS WHAT IS ON THE INPUT. Taring a live channel
        stores a live reading as zero, which is the operator's mistake to
        make; the codes returned are what make it visible.

        Refused with the AFE off: it powers the converter's reference, so
        every channel reads exact mid-scale and a tare against that writes
        a plausible number that means nothing (invariant 9).

        Returns `{name: code}`.
        """
        auto = kw.pop('auto', True)
        save = kw.pop('save', True)
        if kw:
            raise TypeError('tare() got %s' % ', '.join(sorted(kw)))
        if not self.board.afe.is_on():
            raise RigError(
                'AFE_ON is off, and it powers the converter reference - '
                'every channel reads exact mid-scale, so a tare would store '
                'that as zero. Switch it on first')

        wanted = self.pick(*names) if names else [
            r['name'] for r in self.catalogue()
            if r['kind'] == 'analog' and r.get('unit') == 'mA']

        index = {c['signal']: c['index']
                 for c in self.board.system.channel_map()['analog']}
        got = {}
        for spelling in wanted:
            if auto:
                code = self._read_now(spelling)
                self.compensate(spelling, offset=code, save=False)
            else:
                code = self.board.calibration.zero(index[spelling])
            got[spelling] = code
        if save:
            self.board.calibration.save()
        return got

    def _read_now(self, name):
        """One channel's code, meaned over a burst, for a tare to keep.

        A BURST AND NOT A SAMPLE. A zero taken from one conversion carries
        that conversion's noise into every reading afterwards, which is the
        opposite of what a tare is for.
        """
        for row in self.board.analog.read_all()['channels']:
            if row['signal'] == name:
                return int(round(row['mean_raw']))
        raise RigError('%r is not a channel this board reads' % name)

    def configure_buffer(self, records):
        """Size the circular buffer the records land in, in RECORDS.

            daq.configure_buffer(10000)

        WHERE it lands depends on who owns the link. With a broker in the
        path this sizes the BROKER'S ring, and every client on it - another
        process, another thread, a view and a chat session at once - reads
        that one ring from its own cursor without taking records from the
        others. Alone on the port it sizes this process's own queue.

        A reader that falls far enough behind for the writer to lap it does
        lose records, because a ring is finite and the alternative is
        stalling the board for the slowest reader in the building. It is
        told how many, in `buffered()['lost']`. A gap nobody counted is the
        one outcome a shared ring must not have.
        """
        self._buffer_records = max(1, int(records))
        return self._buffer_records

    def start(self):
        """Begin sampling into the board's buffer, and into the host's.

        TWO BUFFERS, THE WAY A DAQ CARD HAS TWO: the board's ring fills at
        the sample rate, and a reader thread here empties it into a host
        queue as fast as the link goes. `read_buffer()` takes from that
        queue, so the caller's own work - a print, a plot, a terminal -
        never sits between two round trips.
        """
        self.board.daq.start()
        # A reply carries as many records as fit in the board's own reply
        # room, and that is what the reader waits for rather than reading
        # the instant one record lands.
        stride = (self.layout or {}).get('stride') or 0
        take = self._from_broker(stride)
        self._reader = BufferedReader(
            acquire=take or (lambda: self._timed(
                self.board.daq.acquire(layout=self.layout))),
            backlog=lambda: self.board.daq.backlog,
            batch=(REPLY_ROOM // stride) if stride else 1).start()
        return self

    def _from_broker(self, stride):
        """A reader that takes from the broker's ring, or None.

        ONE DRAINER OF THE BOARD, and when a broker owns the port it is the
        broker's thread - so this process reads the ring rather than the
        wire, and every other client on that broker gets the same records
        from its own place in it.
        """
        wire = getattr(self.board, 'transport', None)
        if not stride or not hasattr(wire, 'stream'):
            return None
        wire.stream(stride, self._buffer_records)
        self._cursor = wire.stream_state().get('head', 0)

        def take():
            blob, first, lost, nxt = wire.take(self._cursor)
            self._lost += lost
            self._cursor = nxt
            return self._timed(self.board.daq.decode(blob, self.layout))

        return take

    def stop(self):
        """Stop sampling. What is already buffered stays readable.

        The reader is stopped and JOINED before the board is told anything:
        two threads on one serial transport is the one thing this
        arrangement must not do.
        """
        if self._reader is not None:
            self._reader.stop()
            self._reader = None
        self.board.daq.stop()
        return self

    @property
    def buffered(self):
        """Blocks waiting on the host, and records still on the board.

        `backlog` is the board's own answer to the last read, carried by
        that same transaction - not a second round trip asking about a
        later moment.
        """
        r = self._reader
        if r is None:
            return {'host': 0, 'peak': 0, 'dropped': 0, 'backlog': None,
                    'reads': 0, 'records': 0, 'rate': 0.0,
                    'lost': self._lost, 'cursor': self._cursor}
        # `taken`, not `records`: the reader resets that one to measure
        # its own rate, and a byte rate differentiated off a counter
        # that resets reads as negative throughput.
        return {'host': len(r), 'peak': r.peak, 'dropped': r.dropped,
                'backlog': r.backlog, 'reads': r.reads,
                'records': r.taken, 'rate': r.rate,
                'lost': self._lost, 'cursor': self._cursor}

    def state(self):
        """How the task is doing: rate, what is buffered, what was lost."""
        return self.board.daq.state()

    # -- reading ---------------------------------------------------------

    def acquire(self):
        """One block of records, oldest first, with times on them.

        Empty when nothing has been buffered yet - call it again.

        **A channel's value is the SUM of `samples` readings, not one
        reading.** `record['samples']` says how many, so the mean is
        `record['NTC'] / record['samples']`. The board sends the sum
        because it keeps the bits an average throws away.
        """
        # ONE DRAINER. While the reader thread is on the link it is the
        # only thing that may take records: two drainers split the ring
        # between them and each sees gaps the other took. So this serves
        # from the host queue instead - same records, same order, and no
        # round trip at all.
        if self._reader is not None:
            return self._reader.take() or []

        # `samples` rides in the record now, so this no longer spends a
        # round trip on state() per block to ask what it was configured
        # with - which was the wrong number the moment the clock, rather
        # than a count, closed the record.
        return self._timed(self.board.daq.acquire(layout=self.layout))

    #: Consecutive unanswered reads that still count as a busy link.
    MISSES_ALLOWED = 5

    #: Written by the main loop every few microseconds, so a write to it is
    #: gone before the reply is. Refused rather than accepted and lost.
    LOOP_OWNED = ('KEEPALIVE',)

    def outputs(self):
        """What can be written, asked of the board rather than listed here.

        Digital: the pins the board's own map calls outputs. Analog: the three
        inverter legs. There is no DAC here, so an analog write is a PWM duty
        from 0.0 to 1.0, the nearest thing to putting a level out.
        """
        pins = [d for d in self.board.system.channel_map()['digital']
                if d['direction'] == 'out'
                and d['signal'] not in self.LOOP_OWNED]
        return {'digital': [d['signal'] for d in pins],
                'analog': ['Phase U', 'Phase V', 'Phase W']}

    def write(self, digital=None, analog=None):
        """Put levels out: named pins, and duties on the three legs.

        digital  {'AFE_ON': True, 'UART5_TERM': False}. Names come from the
                 board's own map. AFE_ON goes through the supply's own
                 call; the rest go through the pin writer, which needs test
                 mode - this turns it on and off around the write.

        analog   {'Phase U': 0.25, ...}, 0.0 to 1.0. There is no DAC here,
                 so this is a PWM duty. **Refused unless the gate drivers are
                 armed**: arming a power stage should be asked for by name,
                 not fall out of writing a level. `gates.arm()` is that name.

        Returns what it did, so a caller can check rather than assume.
        """
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
            self.board.afe.enable() if level else self.board.afe.disable()
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

        # ONE state read serves the arm check, the period and the held
        # duties. As three reads at 31 ms each it cost 110 ms before the
        # duty went out - measured 2026-08-30 on a pulse meant to last
        # two writes.
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
        """Records, waiting for them. NEGATIVE means everything there is.

        The blocking read the loop-free shape wants:

            daq.configure('Phase U', 'NTC')
            daq.start()
            values = daq.read(-1)
            daq.stop()

        With `count` negative it drains what the board has and returns it,
        waiting for at least one record so a caller never gets an empty
        list from a healthy link. With a count it waits for that many.

        Returns a flat list of records, not blocks: a block is how the
        link carried them and nothing a caller should have to know.
        """
        out = []
        if count == 0:
            return out
        for block in self.read_buffer(-1):
            out.extend(block)
            if count < 0 or len(out) >= count:
                break
        if count > 0:
            while len(out) < count:
                for block in self.read_buffer(-1):
                    out.extend(block)
                    break
        return out[:count] if count > 0 else out

    def read_buffer(self, count):
        """`count` blocks off the HOST buffer, one at a time.

        The reader thread is already filling it; this only takes, so the
        loop body costs the link nothing. NEGATIVE runs until the task ends
        or the caller breaks out.

        Needs `start()` - that is what puts a reader on the link. To read
        the board directly instead, one round trip per block on the calling
        thread, use `blocks()`.
        """
        if self._reader is None:
            raise RigError('nothing is buffering yet - start() puts the '
                           'reader on the link, and read_buffer() takes '
                           'what it has collected')
        seen = 0
        while count < 0 or seen < count:
            self._reader.raise_if_failed()
            block = self._reader.take()
            if block:
                seen += 1
                yield block
                continue
            if not self._reader.running:
                return                    # the link is gone, or stopped
            time.sleep(0.002)

    def blocks(self, count):
        """`count` non-empty blocks, one at a time, for a `for` loop.

        NEGATIVE runs the whole capture: blocks keep coming until the task
        stops on its own - a `records=` run reaching its end - or the
        caller breaks out. A free-running task never stops, which is what
        `for block in daq.blocks(-1)` is for.

        Waits for the board rather than spinning: an empty block means the
        buffer has not filled yet, not that anything is wrong.

        With a reader thread running - which `start()` puts there - this is
        `read_buffer()`: the queue it fills is the same records in the same
        order, and a second drainer would only take some of them.
        """
        if self._reader is not None:
            for block in self.read_buffer(count):
                yield block
            return

        seen, missed = 0, 0
        while count < 0 or seen < count:
            try:
                block = self.acquire()
            except (NoReplyError, CrcError) as exc:
                # A missed reply is a fact of this link, measured at about
                # one transaction in fifty while the board is busy, and a
                # loop of twenty reads meets one more often than not. It is
                # not a dead link until it keeps happening, so this counts
                # rather than raises - and raises when the count says the
                # link really has gone, because a generator that spun
                # forever on a dead port would be worse than either.
                missed += 1
                if missed > self.MISSES_ALLOWED:
                    raise RigError(
                        '%d replies in a row went missing, so the link is '
                        'gone rather than busy: %s'
                        % (missed, exc)) from exc
                time.sleep(0.01)
                continue
            missed = 0
            if not block:
                if count < 0 and self.state()['done']:
                    return          # the run ended and the buffer is dry
                time.sleep(0.005)
                continue
            seen += 1
            yield block

    def latest(self, block=True):
        """The running average since the last time you asked.

        Different from `read()`, and worth knowing why: `read()` drains a
        buffer that drops when it fills, this takes an accumulator that
        cannot. A slow link widens its window instead of losing samples, so
        over a bad connection this is the one to use.

        Each channel carries its own count, lowest and highest, because
        they are not sampled at the same instant and a mean cannot tell you
        a spike happened.
        """
        return self.board.daq.latest(layout=self.layout, block=block)

    def _timed(self, records):
        """Wall-clock time on each record, and each as a `Record`.

        The counter is 32 bits and wraps every nine seconds at 475 MHz, so
        the raw stamps are unwrapped first. Per block is enough as long as
        blocks are read more often than the counter wraps.

        A `Record` is a dict underneath, so `r['NTC']` and `r['samples']`
        mean exactly what they meant before; `r.start_time`, `r.dt` and
        `r.samples` are the shape a script reads. `coaxial.record` says
        why one word carries two meanings.
        """
        if not records:
            return records

        stamps = None
        if self.sync is not None:
            stamps = [self.sync.to_host(c)
                      for c in unwrap([r['at'] for r in records])]
            for record, when in zip(records, stamps):
                record['time'] = when

        fields = (self.layout or {}).get('fields') or []
        return build(records, fields, stamps)
