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
import time

from .acquisition import Acquisition
from .clock import NTP_SERVER, unwrap
from .errors import CrcError, NoReplyError, RigError
from .gates import GateStage
from .reader import BufferedReader

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


#: What the acquisition front door answers. A whitelist, not everything:
#: `daq.write` reaching the pin writer would put the device vocabulary
#: behind the wrong name.
DAQ_DOOR = ('configure', 'shape', 'ladder', 'tone', 'start', 'stop',
            'state', 'acquire', 'latest', 'blocks', 'read_buffer',
            'buffered', 'channels', 'outputs')


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

    def __repr__(self):
        return ('<the acquisition front door - open(), configure(), '
                'start(), acquire()/latest()/blocks(), stop(), close()>')


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

    def configure(self, channels=None, sample_rate=None, accumulate=None,
                  decimate=1, digital=True, clock='software',
                  sample_time=0, records=None, interval_us=None,
                  adapt=False):
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
        self._reader = BufferedReader(
            acquire=lambda: self._timed(
                self.board.daq.acquire(layout=self.layout)),
            backlog=lambda: self.board.daq.backlog,
            batch=(REPLY_ROOM // stride) if stride else 1).start()
        return self

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
                    'reads': 0, 'records': 0}
        return {'host': len(r), 'peak': r.peak, 'dropped': r.dropped,
                'backlog': r.backlog, 'reads': r.reads,
                'records': r.records}

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
        """Put a wall-clock time on each record, if the clock was set.

        The counter is 32 bits and wraps every nine seconds at 475 MHz, so
        the raw stamps are unwrapped first. Per block is enough as long as
        blocks are read more often than the counter wraps.
        """
        if not records or self.sync is None:
            return records

        for record, cycles in zip(records, unwrap([r['at'] for r in records])):
            record['time'] = self.sync.to_host(cycles)
        return records
