"""The analog front end: channels, bursts and the conversions that are known.
"""
from . import protocol, scaling
from .afe import powered
from .errors import DeviceStateError
from .subsystem import Subsystem, remembered
from .wire import Reader, pack


class Analog(Subsystem):

    """The ADC channels: what exists, and what they read now. Raw codes and pin
    volts; the host owns every conversion beyond that.
    """

    @remembered
    def scaling(self):
        '''The board's own conversion parameters, fetched once and cached.'''
        return scaling.from_calibration(self._board.calibration.read())

    # -- the channel table -------------------------------------------------

    @remembered
    def channels(self):
        """Channel metadata, fetched once and cached."""
        # Asked for in pages.
        table = []
        while True:
            reader = Reader(self.request(protocol.ADC_TABLE,
                                         pack(('u8', len(table)))))
            sent = reader.u8()
            for _ in range(sent):
                table.append(self._row(reader, len(table)))
            total = reader.u8() if reader.remaining else len(table)
            if len(table) >= total or not sent:
                return table

    @staticmethod
    def _row(reader, index):
        """One channel's metadata, the live conversion beside it dropped."""
        row = {
            'index': index,
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
        return row

    def index_of(self, signal):
        """Index of the channel carrying a named signal, e.g. 'NTC'."""
        rows = self.board.system.channel_map()['analog']
        index = next((c['index'] for c in rows if c['signal'] == signal), None)
        if index is None:
            named = [c['signal'] for c in rows if c['signal']]
            raise KeyError('no channel carries signal %r; the board reports %r'
                           % (signal, named))
        return index

    def names(self):
        """Every channel's signal name, in the board's own order."""
        return [c['signal']
                for c in self.board.system.channel_map()['analog']]

    def mask_all(self):
        return (1 << len(self.channels())) - 1

    # -- sampling ----------------------------------------------------------

    def burst(self, mask, nr_of_samples, sample_rate=None):
        """Sample the masked channels and return raw statistics per channel."""
        interval_us = 0 if not sample_rate else int(round(1e6 / sample_rate))
        duration_us = nr_of_samples * interval_us

        if duration_us > protocol.BURST_MAX_MICROSECONDS:
            raise ValueError(
                "%d samples at %g Hz would take %.1f s; the firmware refuses "
                "bursts over %.1f s so the link is never left silent longer "
                "than the master will wait"
                % (nr_of_samples, sample_rate, duration_us / 1e6,
                   protocol.BURST_MAX_MICROSECONDS / 1e6))

        # A burst legitimately blocks the slave for as long as it samples.
        budget_us = duration_us if interval_us else protocol.BURST_MAX_MICROSECONDS
        timeout = budget_us / 1e6 + 1.0

        reader = Reader(self.request(
            protocol.ANALOG_BURST,
            pack(('u16', mask), ('u16', nr_of_samples), ('u32', interval_us)),
            timeout=timeout))

        samples = reader.u16()
        elapsed_us = reader.u32()
        per_channel = {reader.u8(): self._statistics(reader)
                       for _ in range(reader.u8())}

        return {
            'samples': samples,
            'elapsed_us': elapsed_us,
            'rate_hz': (samples * 1e6 / elapsed_us) if elapsed_us else None,
            'channels': per_channel,
        }

    @staticmethod
    def _statistics(reader):
        """One channel's four numbers off a burst reply."""
        return {
            'mean_raw': reader.milli(),
            'min_raw': reader.i32(),
            'max_raw': reader.i32(),
            'stddev_raw': reader.milli('u32'),
        }

    @powered
    def _one(self, index, nr_of_samples, sample_rate):
        """Burst a single channel and return just its statistics."""
        result = self.burst(1 << index, nr_of_samples, sample_rate)
        stats = dict(result['channels'][index])
        stats['samples'] = result['samples']
        stats['rate_hz'] = result['rate_hz']
        return stats

    # -- readings ----------------------------------------------------------

    @powered
    def read_all(self, nr_of_samples=64, sample_rate=1000.0, vref=3.3):
        """Every configured channel at once, with its table metadata merged in.
        """
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
        """Temperature in degrees Celsius."""
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
        """DC bus volts."""
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
        """Phase current in amperes."""
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
        if not result['afe_on']:
            raise DeviceStateError(
                'the scan reports the analog front end off, so every channel '
                'read mid-scale: ntc_centidegc would be exactly 2500 and '
                'dcbus_mv a plausible number that is not a measurement. '
                'Call board.afe.enable() first.')

        return result

    @powered
    def noise(self, adc, nr_of_samples=200):
        """The firmware's own noise measurement on one ADC's phase channel."""
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
