"""One board behind one class: connect, configure, trigger, read.

    from coaxial import Coaxial63100

    with Coaxial63100(port='COM4') as daq:
        daq.set_time_from_pc()
        daq.configure_daq(['Phase U', 'NTC'], accumulate=8)
        daq.start()
        for block in daq.blocks(20):
            print(block[-1]['time'], block[-1]['NTC'] / block[-1]['samples'])

This is the front door. `Board` and its subsystems are still there under
`daq.board` and nothing is hidden, but a caller that wants measurements
should not have to know that the supply lives in `afe`, the converters in
`daq`, the counter in `clock` and the gates in `gate drivers` - or the order they
have to be touched in.

It also owns the preflight every view was writing out again: AFE_ON powers
the ADC's reference and both SPI parts, so it has to be on for a reading to
mean anything (invariant 9), and it has to be put back the way it was found
- leaving a board powered because a script ended is a change nobody asked
for, and switching one off that was on before is worse.

Nothing here judges a reading. Raw converter codes and the board's own
units; `board.analog` has the conversions when you want them.
"""
import time

from .clock import NTP_SERVER, unwrap
from .errors import CrcError, NoReplyError, RigError


class Coaxial63100:

    """One board, one acquisition task, one clock."""

    def __init__(self, port='COM4', baud=115200, unit=1, link='auto',
                 simulated_device=False, power_afe=True):
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
        self.origin = None
        self.simulated = simulated_device
        self.layout = None
        self.sync = None
        self._afe_was_on = None

    # -- opening and closing --------------------------------------------

    def open(self):
        """Open the link, bring the supply up, and hand back self."""
        from coaxial_mcp.session import open_session

        simulated = True if self.simulated_device else (
            None if self.link == 'auto' else False)

        self.session, self.origin = open_session(
            self.port, baud=self.baud, unit=self.unit, simulated=simulated)
        self.board = self.session.board
        self.simulated = not self.origin.real

        if self.power_afe:
            self._afe_was_on = self.board.afe.is_on()
            if not self._afe_was_on:
                self.board.afe.enable()
                # The parts need their supply up before anything talks to
                # them. Enabling and configuring in the same breath answered
                # SERVER DEVICE FAILURE.
                time.sleep(0.3)
        return self

    def close(self):
        """Everything this session started, undone.

        The task first: one left running keeps the converters busy after the
        script that asked for them has gone. Then the supply, but only if
        this session was what switched it on.
        """
        if self.board is not None:
            try:
                self.board.daq.stop()
                self.board.gate_drivers.disable()
                if self.power_afe and self._afe_was_on is False:
                    self.board.afe.disable()
            except RigError:
                pass                    # closing is not the place to raise
        if self.session is not None:
            self.session.close()
        self.session = self.board = None

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

    def configure_daq(self, channels=None, rate_hz=None, accumulate=1,
                      decimate=1, digital=True, clock='software',
                      sample_time=0):
        """Set up the acquisition. Replaces whatever was there.

        channels    names, e.g. ['Phase U', 'NTC']. None takes all of them.
        rate_hz     how often to sample. None lets the board choose what
                    the link can carry, which is the safe default.
        accumulate  sum this many samples into each record - averaging that
                    keeps what an average throws away, since the record
                    carries the sum and the count.
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
        self.layout = self.board.daq.configure(
            channels if channels is not None else self.channels(),
            clock=clock, sample_time=sample_time, decimate=decimate,
            accumulate=accumulate, digital=digital, rate_hz=rate_hz)
        return self.layout

    def start(self):
        """Begin sampling into the board's buffer."""
        self.board.daq.start()
        return self

    def stop(self):
        """Stop sampling. What is already buffered stays readable."""
        self.board.daq.stop()
        return self

    def status(self):
        """How the task is doing: rate, what is buffered, what was lost."""
        return self.board.daq.state()


    #: What the schematic wants true before the gate drive is armed, as volts
    #: at the pin. The charge pump has to have pumped and the level detector
    #: has to have tripped; arming under either of them is arming into a
    #: supply that is still coming up.
    #:
    #: Volts and not codes: a threshold in codes stops meaning anything the
    #: moment a divider changes, and the divider is the board's, not this
    #: file's (invariant 7).
    INTERLOCK = (('Cinj', 3.0), ('Clevel', 3.0))

    def interlock(self):
        """What the arming conditions read now, and which of them hold.

        Measured every time. Returns a list of (name, volts, ok, want) - it
        does not raise, so a view can show the conditions coming up rather
        than only learning about them when an arm is refused.
        """
        if not self.board.afe.is_on():
            # AFE_ON powers the reference, so with it off every one of these
            # reads exact mid-scale and would pass or fail by accident.
            return [('AFE_ON', None, False, None)]

        rows = []
        readings = {r['signal']: r for r in
                    self.board.analog.read_all(nr_of_samples=32)['channels']}
        for name, want in self.INTERLOCK:
            got = readings.get(name)
            volts = got['volts_at_pin'] if got else None
            rows.append((name, volts, volts is not None and volts >= want,
                         want))
        return [('AFE_ON', None, True, None)] + rows

    def arm_gate_drivers(self, bypass_sto=False, ignore_interlock=False):
        """Set MOE. Nothing switches before this and everything can after.

        **This arms a power stage**, at zero duty - all three low sides on,
        a braked stage rather than a floating one.

        TIM1's dead time is the only protection: the 2EDL8034's inputs are
        independent and it has no interlock. Measured in the silicon, not
        the `.ioc` - BDTR DTG 19, CR1 CKD 00, 237.5 MHz, so **80.0 ns**
        against about 65 ns needed. `gate_drivers_check()` re-reads it and
        refuses at zero.

        `ignore_interlock` skips `INTERLOCK`, which this bench board needs:
        Cinj reads 0.77 V and Clevel 0.06 V against 3 V each. `bypass_sto`
        disconnects the break input, without which a latched break outranks
        this. Both are decisions, which is why neither is silent.
        """
        self.gate_drivers_check()

        if not ignore_interlock:
            failed = [row for row in self.interlock() if not row[2]]
            if failed:
                raise RigError(
                    'the arming interlock is not satisfied: %s. The '
                    'schematic wants the charge pump up and the level '
                    'detector tripped before the gate drive is armed. Pass '
                    'ignore_interlock=True to arm anyway, which is what an '
                    'unmodified bench board needs'
                    % ', '.join(
                        '%s %s' % (name, 'is off' if volts is None
                                   else '%.2f V, wants %.1f' % (volts, want))
                        for name, volts, _, want in failed))

        if bypass_sto:
            self.board.gate_drivers.bypass_break(True)
        self.board.gate_drivers.enable()
        return self.board.gate_drivers.state()

    def disarm_gate_drivers(self, keep_bypass=False):
        """Clear MOE, and put the break input back unless told otherwise."""
        self.board.gate_drivers.disable()
        if not keep_bypass:
            self.board.gate_drivers.bypass_break(False)
        return self.board.gate_drivers.state()

    def gate_drivers_armed(self):
        """Whether MOE is set, read off the board rather than remembered."""
        return bool(self.board.gate_drivers.state()['pwm_enabled'])

    def gate_drivers_check(self):
        """Refuse to arm a gate driver stage with no dead time.

        The one thing between the two FETs of a leg. Read every time rather
        than trusted once: a `.ioc` regeneration, a CubeMX mode name bound
        to the wrong channel - which has happened twice here - or a stray
        BDTR write all land in the same place, and none of them announce
        themselves.
        """
        state = self.board.gate_drivers.state()
        if not state['deadtime']:
            raise RigError(
                'TIM1 BDTR DTG reads 0, so there is no dead time and the '
                '2EDL8034 has no interlock of its own - both FETs of a leg '
                'would conduct together. Check TIM1.DeadTime in the .ioc '
                'and that the generated MX_TIM1_Init still applies it')
        if state.get('gate_shorts'):
            raise RigError(
                'the gate pins of leg %s are on one node, so that leg cannot '
                'be driven complementary: both FETs get the same command and '
                'the leg never switches. Measured by the board, which drives '
                'one pin and watches the other sink through its own pull-down.'
                % ', '.join(state['gate_shorts']))
        return state

    def configure_pwm(self, duty=0.0, bypass_sto=False):
        """Run the gate drivers at one duty on all three phases.

        `duty` is 0.0 to 1.0 and goes to every leg equally, which puts no
        voltage between them: real switching, no phase current.

        `bypass_sto` disconnects the Safe Torque Off break input.
        **Treat this as arming a power stage.** The argument for it being
        safe was that the STO chain gates the drivers' own supply, which no
        MCU pin reaches - and on the bench board that argument did not
        hold: 25 % duty tripped the hot-swap's over-current and took the
        board down. FINDINGS has what is ruled out. A reset restores it.
        """
        self.arm_gate_drivers(bypass_sto=bypass_sto)
        period = self.board.gate_drivers.state()['period'] - 1
        ticks = int(max(0.0, min(1.0, duty)) * period)
        self.board.gate_drivers.duty((ticks, ticks, ticks))
        return self.board.gate_drivers.state()

    def stop_pwm(self):
        """Gates down, and the break input back where it was."""
        self.disarm_gate_drivers()

    # -- reading ---------------------------------------------------------

    def read(self):
        """One block of records, oldest first, with times on them.

        Empty when nothing has been buffered yet - call it again.

        **A channel's value is the SUM of `samples` readings, not one
        reading.** `record['samples']` says how many, so the mean is
        `record['NTC'] / record['samples']`. The board sends the sum
        because it keeps the bits an average throws away.
        """
        records = self.board.daq.read(layout=self.layout)
        samples = max(1, self.board.daq.state()['accumulate'])
        for record in records:
            record['samples'] = samples
        return self._timed(records)

    #: Consecutive unanswered reads that still count as a busy link.
    MISSES_ALLOWED = 5

    #: Written by the main loop every few microseconds, so a write to it is
    #: gone before the reply is. Refused rather than accepted and lost.
    LOOP_OWNED = ('KEEPALIVE',)

    def outputs(self):
        """What can be written, asked of the board rather than listed here.

        Digital: the pins the board's own map calls outputs. Analog: the
        three gate drivers legs. There is no DAC on this board, so an analog
        write is a PWM duty from 0.0 to 1.0 - the nearest thing it has to
        putting a level out.
        """
        pins = [d for d in self.board.system.channel_map()['digital']
                if d['direction'] == 'out'
                and d['signal'] not in self.LOOP_OWNED]
        return {'digital': [d['signal'] for d in pins],
                'analog': ['Phase U', 'Phase V', 'Phase W']}

    def daq_read(self):
        """One block of records. The same call as `read()`, named to pair
        with `daq_write`."""
        return self.read()

    def daq_write(self, digital=None, analog=None):
        """Put levels out: named pins, and duties on the gate drivers legs.

        digital  {'AFE_ON': True, 'UART5_TERM': False}. Names come from the
                 board's own map. AFE_ON goes through the supply's own
                 call; the rest go through the pin writer, which needs test
                 mode - this turns it on and off around the write.

        analog   {'Phase U': 0.25, ...}, 0.0 to 1.0. There is no DAC here,
                 so this is a PWM duty. It is **refused unless the gate drivers is
                 armed**: arming a power stage should be something a caller
                 asked for by name, not the side effect of writing a level.
                 `arm_gate_drivers()` is that name.

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

        if not self.gate_drivers_armed():
            raise RigError(
                'the gate drivers are not armed, and writing a duty is not what '
                'arms it - call arm_gate_drivers() first, which says what that '
                'means. %s'
                % ('The break is latched, so arm_gate_drivers(bypass_sto=True) '
                   'is what gets past it'
                   if self.board.gate_drivers.state()['fault']
                   else 'Nothing is holding it off'))

        period = self.board.gate_drivers.state()['period'] - 1
        held = self.board.gate_drivers.state()['duty']
        ticks = tuple(
            int(max(0.0, min(1.0, analog[name])) * period)
            if name in analog else held[i]
            for i, name in enumerate(legs))
        self.board.gate_drivers.duty(ticks)
        return dict(zip(legs, (t / period for t in ticks)))

    def blocks(self, count):
        """`count` non-empty blocks, one at a time, for a `for` loop.

        Waits for the board rather than spinning: an empty block means the
        buffer has not filled yet, not that anything is wrong.
        """
        seen, missed = 0, 0
        while seen < count:
            try:
                block = self.read()
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
