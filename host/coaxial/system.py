"""Identity, versions and the clock tree."""
from . import protocol
from .errors import PayloadError
from .subsystem import Subsystem
from .wire import Reader


class System(Subsystem):
    """What the board is, and what it is running at."""

    def version(self):
        """Read the frozen version record. Safe against any firmware vintage.

        Only the first five bytes are guaranteed; everything after them is
        decoded opportunistically, because the payload is append-only and a
        future firmware may carry fields this host has never heard of. Stopping
        when the bytes run out is the whole point of that rule.

        A tail that does not decode stops the loop for the same reason. A new
        protocol major may reorder or resize anything after the prefix, and the
        prefix is what says so - so demanding the optional tail would turn a
        readable 'this host has no codec for that major' into a decode failure
        on a board that answered perfectly.
        """
        reader = Reader(self.request(protocol.VERSION))

        info = {
            'proto_major': reader.u8(),
            'proto_minor': reader.u8(),
            'firmware': '%d.%d.%d' % (reader.u8(), reader.u8(), reader.u8()),
        }

        for key, decode in (('device', reader.string), ('mcu', reader.string),
                            ('build', reader.string), ('commands', reader.u16)):
            if reader.remaining == 0:
                break
            try:
                info[key] = decode()
            except PayloadError:
                break

        return info

    def clock(self):
        """The live clock tree, as the board itself measures it."""
        reader = Reader(self.request(protocol.CLOCK))
        return {
            'sysclk_hz': reader.u32(),
            'hclk_hz': reader.u32(),
            'cycle_counter': reader.u32(),
            'ticks_per_us': reader.u32(),
            'source': protocol.CLOCK_SOURCES.get(reader.u8(), 'unknown'),
        }

    def self_test(self):
        """What the board can prove about itself, with nothing attached.

        Returns a list of {name, status, value}. status is 'pass' or 'fail' only
        for checks the board can settle from its own registers - a locked PLL, a
        calibration that ran, a firmware checksum. Everything that would need a
        calibrated instrument to judge comes back as 'info' with its value.

        Deliberately no limits here and none in the firmware. This board is a
        dumb slave: it measures and reports. Pass/fail against real thresholds
        belongs to the test executive on the line, beside the DMM and the load,
        where a limit is visible, changeable, and recordable against a
        calibration certificate.
        """
        reader = Reader(self.request(protocol.SELF_TEST))
        return [{'name': reader.string(),
                 'status': protocol.CHECK_STATUS.get(reader.u8(), 'unknown'),
                 'value': reader.i32()}
                for _ in range(reader.u8())]

    def self_test_failures(self):
        """Just the checks the board itself calls failures. Empty is good."""
        return [c for c in self.self_test() if c['status'] == 'fail']

    def release_console(self):
        """Hand the UART back to the text console.

        The reply goes out before the switch happens, and the console starts
        printing immediately after, so the frame is read to an exact length.
        A quiet-time read would swallow the banner into the frame.
        """
        self.request(protocol.CONSOLE, exact_payload=0)
