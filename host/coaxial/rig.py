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
`daq`, the counter in `clock` and the gates in `bridge` - or the order they
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
from .errors import RigError


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
                self.board.bridge.disable()
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

    # -- the bridge ------------------------------------------------------

    def configure_pwm(self, duty=0.0, bypass_sto=False):
        """Run the bridge at one duty on all three phases.

        `duty` is 0.0 to 1.0 and goes to every leg equally, which puts no
        voltage between them: real switching, no phase current.

        `bypass_sto` disconnects the Safe Torque Off break input.
        **Treat this as arming a power stage.** The argument for it being
        safe was that the STO chain gates the drivers' own supply, which no
        MCU pin reaches - and on the bench board that argument did not
        hold: 25 % duty tripped the hot-swap's over-current and took the
        board down. FINDINGS has what is ruled out. A reset restores it.
        """
        if bypass_sto:
            self.board.bridge.bypass_break(True)

        self.board.bridge.enable()
        period = self.board.bridge.state()['period'] - 1
        ticks = int(max(0.0, min(1.0, duty)) * period)
        self.board.bridge.duty((ticks, ticks, ticks))
        return self.board.bridge.state()

    def stop_pwm(self):
        """Gates down, and the break input back where it was."""
        self.board.bridge.disable()
        self.board.bridge.bypass_break(False)

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

    #: Written by the main loop every few microseconds, so a write to it is
    #: gone before the reply is. Refused rather than accepted and lost.
    LOOP_OWNED = ('KEEPALIVE',)

    def outputs(self):
        """What can be written, asked of the board rather than listed here.

        Digital: the pins the board's own map calls outputs. Analog: the
        three bridge legs. There is no DAC on this board, so an analog
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

    def daq_write(self, digital=None, analog=None, bypass_sto=False):
        """Put levels out: named pins, and duties on the bridge legs.

        digital  {'AFE_ON': True, 'UART5_TERM': False}. Names come from the
                 board's own map. AFE_ON goes through the supply's own
                 call; the rest go through the pin writer, which needs test
                 mode - this turns it on and off around the write.

        analog   {'Phase U': 0.25, ...}, 0.0 to 1.0. There is no DAC here,
                 so this is a PWM duty. **Writing one arms the bridge**,
                 and on the bench board that has twice tripped the
                 hot-swap's over-current - see `configure_pwm`.

        Returns what it did, so a caller can check rather than assume.
        """
        done = {}
        for name, level in (digital or {}).items():
            done[name] = self._write_pin(name, bool(level))
        if analog:
            done.update(self._write_duty(analog, bypass_sto))
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

    def _write_duty(self, analog, bypass_sto):
        """Duties on the three legs, as one all-or-none update."""
        legs = ('Phase U', 'Phase V', 'Phase W')
        unknown = [n for n in analog if n not in legs]
        if unknown:
            raise RigError('%s cannot be written; this board has no DAC and '
                           'its only analog outputs are %s'
                           % (', '.join(unknown), ', '.join(legs)))

        if bypass_sto:
            self.board.bridge.bypass_break(True)
        if not self.board.bridge.state()['pwm_enabled']:
            self.board.bridge.enable()

        period = self.board.bridge.state()['period'] - 1
        held = self.board.bridge.state()['duty']
        ticks = tuple(
            int(max(0.0, min(1.0, analog[name])) * period)
            if name in analog else held[i]
            for i, name in enumerate(legs))
        self.board.bridge.duty(ticks)
        return dict(zip(legs, (t / period for t in ticks)))

    def blocks(self, count):
        """`count` non-empty blocks, one at a time, for a `for` loop.

        Waits for the board rather than spinning: an empty block means the
        buffer has not filled yet, not that anything is wrong.
        """
        seen = 0
        while seen < count:
            block = self.read()
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
