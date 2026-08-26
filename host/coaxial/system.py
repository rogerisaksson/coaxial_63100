"""Identity, versions and the clock tree."""
from . import protocol
from .errors import PayloadError, RigError
from .subsystem import Subsystem
from .wire import Reader, pack


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
                            ('build', reader.string), ('commands', reader.u16),
                            ('description', reader.string),
                            ('type', reader.string)):
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

    def channel_map(self, refresh=False):
        """Every channel this board has, analog and digital, and which way
        each one runs.

        `{'analog': [...], 'digital': [...], 'reserved': [...]}`.

        `analog` and `digital` are the channels: what can be read, and what a
        fixture may read or set without breaking anything. `reserved` is the
        bus and the debug port - USART3, JTAG - which are not channels and are
        never driven; they are here only so "why was PB10 refused" has an
        answer. Keeping them in a third list rather than behind a flag is what
        stops the two being confused.

        An analog row carries index, adc, channel, pin, direction,
        differential, signal and unit; the other two carry pin, direction and
        signal. Direction is 'in', 'out' or 'inout', from the MCU's side, and
        every ADC channel is 'in'.

        The map, not a reading: `analog.channels()` fetches the same analog
        metadata with a live conversion attached, and `read_all()` is what
        measures. Cached, because none of it changes at run time.

        This is the board describing itself. Nothing above the firmware
        should carry a copy - a pin table in a host, a document or a prompt
        is a second answer to "what is PB10", and the board is the one that
        is right. Needs protocol 1.3; an older board raises, and the caller
        falls back to protocol.RESERVED_PINS.
        """
        if getattr(self, '_map', None) is None or refresh:
            # Two round trips, one per section: both together are 273 bytes
            # against the 253-byte PDU, so the wire carries them separately
            # and this joins them. The split is the frame's, not the map's.
            reader = Reader(self.request(protocol.CHANNELS, pack(('u8', 0))))
            analog = []
            for _ in range(reader.u8()):
                analog.append({
                    'index': reader.u8(),
                    'adc': reader.u8(),
                    'channel': reader.u8(),
                    'pin': reader.string(),
                    'direction': protocol.DIRECTIONS.get(reader.u8()),
                    'differential': bool(reader.u8()),
                    'signal': reader.string(),
                    'unit': protocol.CHANNEL_UNITS.get(reader.u8()),
                })
            pins = {}
            for kind, name in ((1, 'digital'), (2, 'reserved')):
                # Paged: the reserved section is 19 pins now that SPI2, SPI4
                # and the IMU's control lines are listed, which is 418 bytes
                # against a 253-byte PDU.
                rows = []
                first = 0
                while True:
                    reader = Reader(self.request(protocol.CHANNELS,
                                                 pack(('u8', kind),
                                                      ('u8', first))))
                    total, _, count = reader.u8(), reader.u8(), reader.u8()
                    for _ in range(count):
                        rows.append({
                            'pin': reader.string(),
                            'direction': protocol.DIRECTIONS.get(reader.u8()),
                            'signal': reader.string(),
                        })
                    first += count
                    if count == 0 or first >= total:
                        break
                pins[name] = rows
            self._map = {'analog': analog, 'digital': pins['digital'],
                         'reserved': pins['reserved'],
                         'subsystems': self._subsystems(),
                         'parts': self._parts()}
        return self._map

    def _subsystems(self):
        """What the firmware says it is made of: one entry per command table.

        Read from the board rather than listed here, for the same reason the
        channel map is: a host that answers "what can this do" from a table
        of its own is a second answer to a question only the firmware knows.
        An older firmware has no kind 3, and an empty list says so without
        making the whole map fail.
        """
        try:
            reader = Reader(self.request(protocol.CHANNELS, pack(('u8', 3))))
        except RigError:
            return []
        return [{'name': reader.string(), 'what': reader.string(),
                 'commands': reader.u8()} for _ in range(reader.u8())]

    def _parts(self):
        """What is fitted on the board, one entry per part.

        Read from the firmware for the same reason the channel map is: a
        parts list in a host, a document or a prompt is a second answer to
        "what is on this board". `power` names what must be on for the part
        to work at all - AFE_ON powers the IMU as well as the analog front
        end, and a day went into SPI before that was checked.

        Paged, because six parts with their strings come to 380 bytes against
        the 253-byte PDU. An older firmware has no kind 4, and an empty list
        says so without making the whole map fail.
        """
        out = []
        first = 0

        while True:
            try:
                reader = Reader(self.request(protocol.CHANNELS,
                                             pack(('u8', 4), ('u8', first))))
            except RigError:
                return out
            total, _, count = reader.u8(), reader.u8(), reader.u8()
            for _ in range(count):
                out.append({
                    'name': reader.string(),
                    'what': reader.string(),
                    'where': reader.string(),
                    'power': reader.string(),
                    'state': protocol.PART_STATES.get(reader.u8(), 'unknown'),
                })
            first += count
            if count == 0 or first >= total:
                return out

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
