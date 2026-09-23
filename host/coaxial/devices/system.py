"""Identity, versions and the clock tree."""
from coaxial.comm import protocol
from coaxial.errors import PayloadError, RigError
from coaxial.comm.protocol import MapKind
from coaxial.devices.subsystem import Subsystem, remembered
from coaxial.comm.wire import Reader, pack, pages


class System(Subsystem):
    """What the board is, and what it is running at."""

    def version(self):
        """Read the frozen version record."""
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
        out = {
            'sysclk_hz': reader.u32(),
            'hclk_hz': reader.u32(),
            'cycle_counter': reader.u32(),
            'ticks_per_us': reader.u32(),
            'source': protocol.CLOCK_SOURCES.get(reader.u8(), 'unknown'),
        }
        # Appended, so a board older than this answers a shorter reply.
        out['adc_hz'] = reader.maybe('u32')
        return out

    @remembered
    def channel_map(self):
        """Every channel this board has, analog and digital, and which way each
        one runs.
        """
        return {'analog': self._analog(),
                'digital': self._pins(MapKind.DIGITAL),
                'reserved': self._pins(MapKind.RESERVED),
                'subsystems': self._subsystems(),
                'parts': self._parts()}

    def _section(self, kind, absent=()):
        """The pages of one kind of the map."""
        return pages(lambda first: self.request(
            protocol.CHANNELS, pack(('u8', kind), ('u8', first))), absent)

    def _analog(self):
        """The ADC channels: one request, the section fits a frame."""
        reader = Reader(self.request(protocol.CHANNELS,
                                     pack(('u8', MapKind.ANALOG))))
        return [self._analog_row(reader) for _ in range(reader.u8())]

    @staticmethod
    def _analog_row(reader):
        return {
            'index': reader.u8(),
            'adc': reader.u8(),
            'channel': reader.u8(),
            'pin': reader.string(),
            'direction': protocol.DIRECTIONS.get(reader.u8()),
            'differential': bool(reader.u8()),
            'signal': reader.string(),
            'unit': protocol.CHANNEL_UNITS.get(reader.u8()),
        }

    def _pins(self, kind):
        """One pin section, paged: the reserved section is 19 pins now that
        SPI2, SPI4 and the IMU's control lines are listed, which is 418
        bytes against a 253-byte PDU.
        """
        return [self._pin_row(page)
                for page in self._section(kind) for _ in page.rows()]

    @staticmethod
    def _pin_row(reader):
        return {
            'pin': reader.string(),
            'direction': protocol.DIRECTIONS.get(reader.u8()),
            'signal': reader.string(),
        }

    def _subsystems(self):
        """What the firmware says it is made of: one entry per command table.
        """
        try:
            reader = Reader(self.request(protocol.CHANNELS,
                                         pack(('u8', MapKind.SUBSYSTEMS))))
        except RigError:
            return []
        return [{'name': reader.string(), 'what': reader.string(),
                 'commands': reader.u8()} for _ in range(reader.u8())]

    def _parts(self):
        """What is fitted on the board, one entry per part."""
        return [self._part_row(page)
                for page in self._section(MapKind.PARTS, absent=RigError)
                for _ in page.rows()]

    @staticmethod
    def _part_row(reader):
        return {
            'name': reader.string(),
            'what': reader.string(),
            'where': reader.string(),
            'power': reader.string(),
            'state': protocol.PART_STATES.get(reader.u8(), 'unknown'),
        }

    def self_test(self):
        """What the board can prove about itself, with nothing attached."""
        reader = Reader(self.request(protocol.SELF_TEST))
        return [{'name': reader.string(),
                 'status': protocol.CHECK_STATUS.get(reader.u8(), 'unknown'),
                 'value': reader.i32()}
                for _ in range(reader.u8())]

    def self_test_failures(self):
        """Just the checks the board itself calls failures. Empty is good."""
        return [c for c in self.self_test() if c['status'] == 'fail']

    def release_console(self):
        """Hand the UART back to the text console."""
        self.request(protocol.CONSOLE, exact_payload=0)
