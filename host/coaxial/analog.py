"""The analog front end: channels, bursts and the conversions that are known.

The firmware reports raw ADC codes. This module turns them into numbers with
units, but only where the conversion is actually known:

  * the DC bus, through a divider whose resistors are on the schematic;
  * the thermistor, through constants from its datasheet;
  * the three phase inputs, through a shunt and an amplifier chain traced off
    the schematic on 2026-08-26. They are current, not voltage - the sense
    element sits in the phase conductor.

Every one of those three is a number this module could get wrong, so each is a
named constant in scaling.py rather than a literal at a call site, and each
says in its docstring where it came from.
"""
from . import protocol, scaling
from .errors import DeviceStateError
from .subsystem import Subsystem
from .wire import Reader, pack


class Analog(Subsystem):

    """The ADC channels: what exists, and what they read now. Raw codes
    and pin volts; the host owns every conversion beyond that."""
    def __init__(self, board):
        super().__init__(board)
        self._channels = None
        self._scaling = None

    def scaling(self, refresh=False):
        '''The board's own conversion parameters, fetched once and cached.

        INVARIANT 7: these live in the calibration record. They used to be
        literals here too, so calibrating a board left every cooked value on
        this side using the old reference, the old shunt and the old
        thermistor - and nothing said the two had parted company.

        `refresh` after writing the record, which is the one time the cache
        can be wrong.
        '''
        if self._scaling is None or refresh:
            self._scaling = scaling.from_calibration(
                self._board.calibration.read())
        return self._scaling

    # -- the channel table -------------------------------------------------

    def channels(self, refresh=False):
        """Channel metadata, fetched once and cached.

        The table is the board's own description of its ADC wiring: which ADC,
        which channel, which pins, differential or not, and what the signal is
        called. Indices into it are what mask and adc_chan arguments mean. None
        of it changes at run time, which is what makes caching it honest.

        The reply also carries a live conversion per channel - raw, microvolts
        and the board's own scaled value. Those are measurements, not metadata:
        they were taken at the instant of the fetch, under whatever front-end
        state held then, so they are read off the wire to keep the decode
        aligned and then dropped. A cached mid-scale code served later as a
        reading is exactly the invented number this library refuses to produce;
        use read_all(), ntc_temperature() or scan() for live values.
        """
        if self._channels is None or refresh:
            # Asked for in pages. A row costs 18 bytes plus its two names and
            # one reply holds 252: seven channels fitted, nine did not
            # - the board sends what fits, says how many there are, and this
            # asks again from where it stopped.
            table = []
            while True:
                reader = Reader(self.request(protocol.ADC_TABLE,
                                             bytes([len(table)])))
                sent = reader.u8()
                for _ in range(sent):
                    row = {
                        'index': len(table),
                        'adc': reader.u8(),
                        'channel': reader.u8(),
                        'pin': reader.string(),
                        'differential': bool(reader.u8()),
                        'signal': reader.string(),
                    }
                    reader.i32()                      # raw, at fetch time
                    reader.i32()                      # microvolts, at fetch
                    row['unit'] = protocol.CHANNEL_UNITS.get(reader.u8())
                    reader.i32()                      # scaled, at fetch time
                    table.append(row)
                total = reader.u8() if reader.remaining else len(table)
                if len(table) >= total or not sent:
                    break
            self._channels = table
        return self._channels

    def index_of(self, signal):
        """Index of the channel carrying a named signal, e.g. 'NTC'.

        Off the channel MAP, not the table: `channels()` is 0x42, which
        takes a reading of every channel on the way past, so it refuses
        outright while the injected group owns the converters - and then a
        caller cannot even look up a name. The map is 0x6D kind 0, which
        answers what exists without measuring anything, so it works armed
        or not.
        """
        rows = self.board.system.channel_map()['analog']
        for channel in rows:
            if channel['signal'] == signal:
                return channel['index']
        named = [c['signal'] for c in rows if c['signal']]
        raise KeyError('no channel carries signal %r; the board reports %r'
                       % (signal, named))

    def names(self):
        """Every channel's signal name, in the board's own order.

        The map rather than the table, for the reason `index_of` gives.
        """
        return [c['signal']
                for c in self.board.system.channel_map()['analog']]

    def mask_all(self):
        return (1 << len(self.channels())) - 1

    # -- sampling ----------------------------------------------------------

    def burst(self, mask, nr_of_samples, sample_rate=None):
        """Sample the masked channels and return raw statistics per channel.

        sample_rate is in hertz; None or 0 means as fast as the conversions
        allow. The reply carries the elapsed time the SLAVE measured, so the
        caller can see the rate it actually got rather than trust the one it
        asked for.
        """
        interval_us = 0 if not sample_rate else int(round(1e6 / sample_rate))
        duration_us = nr_of_samples * interval_us

        if duration_us > protocol.BURST_MAX_MICROSECONDS:
            raise ValueError(
                "%d samples at %g Hz would take %.1f s; the firmware refuses "
                "bursts over %.1f s so the link is never left silent longer "
                "than the master will wait"
                % (nr_of_samples, sample_rate, duration_us / 1e6,
                   protocol.BURST_MAX_MICROSECONDS / 1e6))

        # A burst legitimately blocks the slave for as long as it samples. The
        # default timeout is sized for register reads, so widen it here. An
        # unpaced burst asked for no duration, so there is nothing to derive one
        # from: wait the only bound either side guarantees, the firmware's
        # ceiling. Deriving it from zero would time out a legal request and
        # leave its reply to arrive during the next transaction.
        budget_us = duration_us if interval_us else protocol.BURST_MAX_MICROSECONDS
        timeout = budget_us / 1e6 + 1.0

        reader = Reader(self.request(
            protocol.ANALOG_BURST,
            pack(('u16', mask), ('u16', nr_of_samples), ('u32', interval_us)),
            timeout=timeout))

        samples = reader.u16()
        elapsed_us = reader.u32()
        count = reader.u8()

        per_channel = {}
        for _ in range(count):
            index = reader.u8()
            per_channel[index] = {
                'mean_raw': reader.i32() / 1000.0,
                'min_raw': reader.i32(),
                'max_raw': reader.i32(),
                'stddev_raw': reader.u32() / 1000.0,
            }

        return {
            'samples': samples,
            'elapsed_us': elapsed_us,
            'rate_hz': (samples * 1e6 / elapsed_us) if elapsed_us else None,
            'channels': per_channel,
        }

    def _one(self, index, nr_of_samples, sample_rate):
        """Burst a single channel and return just its statistics."""
        self._board.afe.require()
        result = self.burst(1 << index, nr_of_samples, sample_rate)
        stats = dict(result['channels'][index])
        stats['samples'] = result['samples']
        stats['rate_hz'] = result['rate_hz']
        return stats

    # -- readings ----------------------------------------------------------

    def read_all(self, nr_of_samples=64, sample_rate=1000.0, vref=3.3):
        """Every configured channel at once, with its table metadata merged in.

        Reports volts at the ADC pin and nothing beyond it. Use
        ntc_temperature(), dcbus_voltage() or phase_current() for the three
        channels whose conversion is known.
        """
        self._board.afe.require()

        table = self.channels()
        result = self.burst(self.mask_all(), nr_of_samples, sample_rate)

        rows = []
        for index, stats in sorted(result['channels'].items()):
            channel = table[index]
            convert = (scaling.differential_volts if channel['differential']
                       else scaling.single_ended_volts)
            row = dict(channel)
            row.update(stats)
            row['volts_at_pin'] = convert(stats['mean_raw'], vref)
            row['noise_volts_rms'] = convert(stats['stddev_raw'], vref)
            rows.append(row)

        return {'samples': result['samples'], 'rate_hz': result['rate_hz'],
                'channels': rows}

    def ntc_temperature(self, adc_chan=None, ntc_params=None,
                        nr_of_samples=64, sample_rate=2000.0):
        """Temperature in degrees Celsius.

        adc_chan defaults to whichever channel the board calls 'NTC'. Averaging
        is worth it: a single sample carries a couple of milli-kelvin of ADC
        noise, and the burst costs the same round trip as one read.
        """
        ntc_params = ntc_params or self.scaling()['ntc']
        index = self.index_of('NTC') if adc_chan is None else adc_chan
        stats = self._one(index, nr_of_samples, sample_rate)

        return {
            'celsius': ntc_params.celsius(stats['mean_raw']),
            'ohms': ntc_params.resistance(stats['mean_raw']),
            'spread_millikelvin': 1000.0 * abs(
                ntc_params.celsius(stats['max_raw']) -
                ntc_params.celsius(stats['min_raw'])),
            'params': ntc_params.name,
            'mean_raw': stats['mean_raw'],
            'samples': stats['samples'],
        }

    def dcbus_voltage(self, adc_chan=None, divider=None,
                      nr_of_samples=64, sample_rate=2000.0):
        """DC bus volts.

        Absolute, not ratiometric: the answer scales with divider.vref, so pass
        a DividerParams carrying a measured reference if you need better than a
        percent.
        """
        divider = divider or self.scaling()['dcbus']
        index = self.index_of('DC bus') if adc_chan is None else adc_chan
        stats = self._one(index, nr_of_samples, sample_rate)

        return {
            'volts': divider.volts(stats['mean_raw']),
            'volts_at_pin': divider.volts_at_pin(stats['mean_raw']),
            'ripple_volts': (divider.volts(stats['max_raw']) -
                             divider.volts(stats['min_raw'])),
            'noise_volts_rms': (divider.volts_at_pin(stats['stddev_raw']) *
                                divider.scale),
            'scale': divider.scale,
            'params': divider.name,
            'mean_raw': stats['mean_raw'],
            'samples': stats['samples'],
        }

    def phase_current(self, signal='Phase U', shunt=None,
                      nr_of_samples=64, sample_rate=2000.0):
        """Phase current in amperes.

        Absolute, like the DC bus, and with two more ways to be wrong: the
        shunt value and the amplifier gain. Both are ShuntParams fields, so a
        board that populates a different shunt needs new numbers there and no
        new firmware.

        `signal` is what the board calls the channel - 'Phase U', 'Phase V',
        'Phase W' - not an index, because the index is the board's to choose.
        """
        shunt = shunt or self.scaling()['phase']
        stats = self._one(self.index_of(signal), nr_of_samples, sample_rate)

        return {
            'amps': shunt.amps(stats['mean_raw']),
            'volts_at_pin': shunt.volts_at_pin(stats['mean_raw']),
            'ripple_amps': (shunt.amps(stats['max_raw']) -
                            shunt.amps(stats['min_raw'])),
            'noise_amps_rms': shunt.amps(stats['stddev_raw']),
            'full_scale_amps': shunt.full_scale_amps,
            'params': shunt.name,
            'mean_raw': stats['mean_raw'],
            'samples': stats['samples'],
        }

    # -- the firmware's own reports, kept for cross-checking ---------------

    def scan(self):
        """The board's own one-shot scan, with the board's own scaling applied.

        Redundant with read_all() by design: two independent paths to the same
        numbers is how a scaling mistake gets caught.

        Refuses when the reply's own afe_on flag is false, rather than handing
        back the mid-scale artefacts as a report.
        """
        reader = Reader(self.request(protocol.ADC_SCAN))
        result = {
            'phase_u_raw': reader.i32(),
            'phase_v_raw': reader.i32(),
            'phase_w_raw': reader.i32(),
            'dcbus_raw': reader.i32(),
            'dcbus_mv': reader.i32(),
            'ntc_raw': reader.i32(),
            'ntc_centidegc': reader.i32(),
            'afe_on': bool(reader.u8()),
            'pe15': bool(reader.u8()),
        }

        # The reply is its own witness, so the gate costs no extra round trip.
        # It has to be a gate rather than a note beside the numbers: the
        # firmware only suppresses the temperature at a rail, and mid-scale is
        # not a rail - it is exactly 25.00 C and a plausible bus voltage.
        if not result['afe_on']:
            raise DeviceStateError(
                'the scan reports the analog front end off, so every channel '
                'read mid-scale: ntc_centidegc would be exactly 2500 and '
                'dcbus_mv a plausible number that is not a measurement. '
                'Call board.afe.enable() first.')

        return result

    def noise(self, adc, nr_of_samples=200):
        """The firmware's own noise measurement on one ADC's phase channel.

        Gated on the front end like any other read: with it off the input sits
        at exact mid-scale and the spread collapses to nearly nothing, which as
        a noise floor reads as a very good board rather than an unpowered one.
        """
        self._board.afe.require()
        reader = Reader(self.request(protocol.ADC_NOISE,
                                     pack(('u8', adc), ('u16', nr_of_samples))))
        return {
            'samples': reader.u16(),
            'mean_uv': reader.i32(),
            'min_raw': reader.i32(),
            'max_raw': reader.i32(),
            'span_raw': reader.u32(),
            'stddev_uv': reader.u32(),
        }
