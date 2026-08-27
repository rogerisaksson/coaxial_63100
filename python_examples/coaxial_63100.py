"""A whole board behind one class, for people who want a DAQ session.

    from coaxial_63100 import Coaxial63100

    daq = Coaxial63100(port='COM4')
    daq.open()
    daq.set_time_from_pc()
    daq.configure_daq(['Phase U', 'NTC'], rate_hz=1000, accumulate=8)
    daq.start()
    for block in daq.blocks(20):
        print(block[0]['time'], block[0]['NTC'])
    daq.stop()
    daq.close()

The library underneath is `host/coaxial`, and everything here is a thin
wrapper over it: `daq.board` is the real object if you want the rest. This
exists so a first script does not have to know which subsystem owns what.

The class is `Coaxial63100` and the file is `coaxial_63100.py`. Python
names classes in CamelCase and modules in lower case, and following that
here means a reader picks up the convention rather than an exception to it.
"""
import os
import sys
import time

# The library lives under host/, one directory up from this one. A real
# install would put it on the path for you; an example in a repository has
# to say where it is.
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'host'))

from coaxial.clock import unwrap                            # noqa: E402
from coaxial_mcp.session import open_session                # noqa: E402


class Coaxial63100:

    """One board, one acquisition task, one clock.

    Nothing here judges a reading. The board reports raw converter codes and
    its own units; turning those into amperes or degrees is the caller's,
    and `board.analog` has the conversions when you want them.
    """

    def __init__(self, port='COM4', baud=115200, unit=1,
                 link='auto', simulated_device=False):
        """Say where the board is. Nothing is opened until `open()`.

        port              the serial port. On Windows 'COM4', on Linux
                          something like '/dev/ttyACM0'.
        baud              bits per second. 115200 is the debug probe's
                          virtual COM port; an RS485 segment can be faster.
        unit              the Modbus address. One board on a bench is 1.
        link              'auto' finds a board on any port if `port` does
                          not answer. 'port' takes you at your word and
                          fails on first use if nothing is there.
        simulated_device  no cable at all. Every value is invented and
                          `self.simulated` stays True so a script can say
                          so - a number that came from nowhere and one that
                          came from hardware must never look alike.
        """
        self.port = port
        self.baud = baud
        self.unit = unit
        self.link = link
        self.simulated_device = simulated_device

        self.session = None
        self.board = None
        self.origin = None
        self.simulated = simulated_device
        self.layout = None
        self.sync = None

    # -- opening and closing --------------------------------------------

    def open(self):
        """Open the link and hand back self, so it chains."""
        simulated = True if self.simulated_device else (
            None if self.link == 'auto' else False)

        self.session, self.origin = open_session(
            self.port, baud=self.baud, unit=self.unit, simulated=simulated)
        self.board = self.session.board
        self.simulated = not self.origin.real
        return self

    def close(self):
        """Stop whatever is running and let the port go.

        Stopping first matters: a task left running keeps the converters
        busy after the script that asked for them has gone.
        """
        if self.board is not None:
            self.board.daq.stop()
            self.board.bridge.disable()
        if self.session is not None:
            self.session.close()
        self.session = self.board = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *_):
        self.close()

    def __repr__(self):
        where = self.origin.label if self.origin else 'not open'
        return '<Coaxial63100 %s%s>' % (where,
                                        ' SIMULATED' if self.simulated else '')

    # -- the clock -------------------------------------------------------

    def set_time_from_pc(self, seconds=3.0):
        """Tie the board's cycle counter to this machine's clock.

        The board has no clock of its own - no RTC, no crystal for one - so
        every timestamp it gives you is a raw cycle count. This measures
        where that counter was and how fast it really runs, and after it
        `daq.read()` puts a wall-clock time on every record.

        `seconds` is how far apart the two measurements are taken. Longer is
        a better rate estimate and a longer wait; three is plenty.
        """
        self.sync = self.board.clock.sync(seconds=seconds)
        return self.sync

    # -- the acquisition task --------------------------------------------

    def channels(self):
        """What the board says it has. Not a list written down here."""
        return [c['signal'] for c in self.board.analog.channels()]

    def configure_daq(self, channels=None, rate_hz=None, accumulate=1,
                      decimate=1, digital=True, clock='software',
                      sample_time=0):
        """Set up the acquisition. Replaces whatever was there.

        channels    names, e.g. ['Phase U', 'NTC']. None takes all of them.
        rate_hz     how often to sample. None lets the board choose what
                    the link can carry, which is the safe default.
        accumulate  sum this many samples into each record. This is how you
                    average without losing anything: the record carries the
                    sum and the count, and `read()` gives you the mean.
        decimate    keep one sample in N and throw the rest away. Prefer
                    `accumulate` - it keeps what this discards.
        digital     include the board's digital pins in every record.
        clock       'software' for the main loop, or 'tim1' for one record
                    per PWM period. 'tim1' carries only the three phases.
        sample_time 0..7, the converter's own sampling window, shortest
                    first. Leave it at 0 unless a channel looks unsettled.
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
        """Set the bridge running at one duty on all three phases.

        `duty` is 0.0 to 1.0 and goes to all three legs equally, which puts
        no voltage between them - real switching, no phase current. Anything
        else needs three values and a reason.

        `bypass_sto` disconnects the Safe Torque Off break input so the
        bridge can be enabled on a bench. **Read this before passing True.**

        The argument for it being safe was that the STO chain gates the gate
        drivers' own supply, which no MCU pin reaches, so the outputs toggle
        into an unpowered stage. On the bench board that argument did not
        hold: 25 % duty on all three phases tripped the hot-swap's
        over-current protection and took the board down. Equal duty puts no
        voltage between the legs, so it was not phase current - something
        else drew it, and it is not understood.

        Until it is, treat this as arming a power stage, not as a
        configuration flag. A reset puts the break back.
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

        Empty when nothing has been buffered yet - call it again. Each
        record is a plain dict: one entry per channel, `time` if the clock
        has been set, and `digital` if the task asked for the pins.

        **A channel's value is the SUM of `accumulate` samples, not one
        reading.** With `accumulate=8` it is about eight times what a meter
        would show. `record['samples']` says how many went in, so the mean
        is `record['Phase U'] / record['samples']`. The sum is what the
        board sends because it keeps the bits an average throws away.
        """
        records = self.board.daq.read(layout=self.layout)
        samples = self._count()
        for record in records:
            record['samples'] = samples
        return self._timed(records)

    def blocks(self, count):
        """`count` non-empty blocks, one at a time, for a `for` loop.

        This is the measurement loop. It waits for the board rather than
        spinning: an empty block means the buffer has not filled yet, not
        that anything is wrong.
        """
        seen = 0
        while seen < count:
            block = self.read()
            if not block:
                time.sleep(0.005)
                continue
            seen += 1
            yield block

    def latest(self):
        """The running average since the last time you asked.

        Different from `read()` and worth knowing why. `read()` drains a
        buffer that drops when it fills; this takes an accumulator that
        cannot. A slow link makes its averaging window wider instead of
        losing samples, so over a bad connection this is the one to use.

        Each channel carries its own count, because they are not sampled at
        the same instant.
        """
        return self.board.daq.latest(layout=self.layout)

    def _timed(self, records):
        """Put a wall-clock time on each record, if the clock was set.

        The board's counter is 32 bits and wraps every nine seconds at
        475 MHz, so the raw stamps have to be unwrapped before they can be
        turned into times. `unwrap` does that; doing it per block is enough
        as long as blocks are read more often than the counter wraps.
        """
        if not records or self.sync is None:
            return records

        for record, cycles in zip(records, unwrap([r['at'] for r in records])):
            record['time'] = self.sync.to_host(cycles)
        return records

    def _count(self):
        """How many samples went into each record, for turning sums into
        means. Read from the board rather than remembered here."""
        return max(1, self.board.daq.state()['accumulate'])
