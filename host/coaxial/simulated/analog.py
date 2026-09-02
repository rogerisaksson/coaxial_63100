"""The analog front end, its seven channels and the calibration
record - invariant 9 acted out without an ADC."""
import math
import random
import time

from .. import protocol
from ..calibration import CalibrationOps
from ..errors import DeviceStateError
from .values import CHANNELS, DRIFT, NOMINAL, _spread, _sweep
from .system import UNITS


class SimulatedAfe:
    """The stand-in AFE, with PE15 following AFE_ON inversely - the same
    relation the real board was measured to have."""
    def __init__(self):
        self.on = False

    def state(self):
        # `users` mirrors the board's reference count. The stand-in has
        # only ever one holder, so the mask follows `on` exactly - which
        # is the one thing the real board does not promise.
        return {'on': self.on, 'pe15': not self.on,
                'users': ['host'] if self.on else []}

    def is_on(self):
        """The real Afe has this and the stand-in did not, which is a gap
        nothing caught until a view asked. See test_parity."""
        return self.on

    def require(self):
        if not self.on:
            raise DeviceStateError('AFE_ON is off (simulated)')
        return True

    def enable(self):
        self.on = True
        return True

    def disable(self):
        self.on = False
        return False

    def toggle(self):
        self.on = not self.on
        return self.on


class SimulatedAnalog:
    """Invented readings, in the shape the real ones come in. Every number
    here is made up; only the columns and the channel names are real."""
    def __init__(self, afe):
        self._afe = afe

    def scaling(self, refresh=False):
        """The same shape the board's own record produces.

        Built from an EMPTY record on purpose, so every value falls back to
        the compiled-in constants. That is the honest stand-in answer: there
        is no calibrated board here to have a record of its own, and a made-up
        one would be a number pretending to be a measurement.
        """
        del refresh
        from .. import scaling as _scaling
        return _scaling.from_calibration({})

    def channels(self, refresh=False):
        return CHANNELS

    def names(self):
        """Signal names in the board's order. Off the map on a real board,
        because the table takes a reading on the way past and refuses while
        the injected group owns the converters."""
        return [c['signal'] for c in CHANNELS]

    def index_of(self, signal):
        for channel in CHANNELS:
            if channel['signal'] == signal:
                return channel['index']
        named = [c['signal'] for c in CHANNELS if c['signal']]
        raise KeyError('no channel carries signal %r; the board reports %r'
                       % (signal, named))

    def burst(self, mask, samples, rate=None):
        chosen = {}
        for meta in CHANNELS:
            index = meta['index']
            if not (mask >> index & 1):
                continue
            if self._afe.on:
                mean = NOMINAL[index] + _sweep(index) + random.uniform(
                    -DRIFT[index], DRIFT[index])
            else:
                # Invariant 9, reproduced exactly: with the reference
                # unpowered, a differential input sits at 0 and a
                # single-ended one at mid-scale - measured on real hardware,
                # not a rounder number picked to look plausible.
                mean = 0.0 if meta['differential'] else 32768.0
            chosen[index] = _spread(meta, mean, self._afe.on)
        return {'samples': samples, 'rate_hz': rate or 2000.0,
                'channels': chosen}

    def ntc_temperature(self, adc_chan=None, ntc_params=None,
                        nr_of_samples=64, sample_rate=2000.0):
        """The NTC in the shape `Analog.ntc_temperature` returns it.

        Invented, like everything here, and it says so through `params`. Mid
        scale on a real board with AFE_ON low is exactly 25.00 C, so the
        stand-in stays away from that number: a value nobody can tell from
        the unpowered case is worse than an obviously fake one.
        """
        celsius = 31.4 + 0.6 * math.sin(time.time() / 30.0)
        return {
            'celsius': celsius,
            'ohms': 10000.0,
            'spread_millikelvin': 28.0,
            'params': 'simulated',
            'mean_raw': 40500,
            'samples': nr_of_samples,
        }

    def dcbus_voltage(self, adc_chan=None, divider=None,
                      nr_of_samples=64, sample_rate=2000.0):
        """The DC link in the shape `Analog.dcbus_voltage` returns it."""
        return {
            'volts': 24.5,
            'volts_at_pin': 24.5 / 23.68,
            'ripple_volts': 0.025,
            'noise_volts_rms': 0.004,
            'scale': 23.68,
            'params': 'simulated',
            'mean_raw': 20375,
            'samples': nr_of_samples,
        }

    def scan(self):
        """The one-shot scan, refusing on the same condition as the real one.

        The refusal is the point of having it here: invariant 9 says a scan
        with AFE_ON low reads mid-scale, and a stand-in that answered anyway
        would let a view ship with that path never taken.
        """
        if not self._afe.is_on():
            from ..errors import DeviceStateError
            raise DeviceStateError(
                'the scan reports the analog front end off, so every channel '
                'read mid-scale: ntc_centidegc would be exactly 2500 and '
                'dcbus_mv a plausible number that is not a measurement. '
                'Call board.afe.enable() first.')

        by_signal = {row['signal']: row['index'] for row in CHANNELS}
        taken = self.burst((1 << len(CHANNELS)) - 1, 1)['channels']
        raw = {name: int(taken[index]['mean_raw'])
               for name, index in by_signal.items() if index in taken}
        params = self.scaling()
        return {
            'phase_u_raw': raw.get('Phase U', 0),
            'phase_v_raw': raw.get('Phase V', 0),
            'phase_w_raw': raw.get('Phase W', 0),
            'dcbus_raw': raw.get('DC bus', 0),
            'dcbus_mv': int(params['dcbus'].volts(raw.get('DC bus', 0))
                            * 1000.0),
            'ntc_raw': raw.get('NTC', 0),
            'ntc_centidegc': int(params['ntc'].celsius(
                max(1, raw.get('NTC', 1))) * 100.0),
            'afe_on': True,
            'pe15': not self._afe.is_on(),
        }

    def read_all(self, nr_of_samples=64, sample_rate=1000.0, vref=3.3):
        """Every channel with its table row merged in, like the real one.

        Here because the stand-in is duck-typed against `Analog` and the
        meter bridge calls this: a view that works on a board and crashes on
        the stand-in is a view nobody can develop without a cable.
        """
        table = {row['index']: row for row in CHANNELS}
        result = self.burst((1 << len(CHANNELS)) - 1, nr_of_samples,
                            sample_rate)

        rows = []
        for index, stats in sorted(result['channels'].items()):
            row = dict(table[index])
            row.update(stats)
            row['unit'] = UNITS.get(row['signal'])
            row['stddev_raw'] = 1.0
            divisor = 32768.0 if row['differential'] else 65536.0
            row['volts_at_pin'] = stats['mean_raw'] / divisor * vref
            rows.append(row)

        return {'samples': result['samples'], 'rate_hz': result['rate_hz'],
                'channels': rows}


class SimulatedCalibration(CalibrationOps):
    """The record an uncalibrated board holds: `stored` false, and the
    firmware's compiled-in defaults behind it.

    Invented numbers would be the one thing this stand-in must not do - a
    calibration is a measurement against an instrument, and there is no
    instrument here. Empty params is what an uncalibrated board answers,
    and `from_calibration({})` already reads that as the fallback.
    """
    #: Channels the record trims, as many as the board's ADC table has.
    CHANNELS = 10

    #: The board this belongs to, so `zero()` can read a channel the
    #: way the real one does. Set by SimulatedBoard.
    board = None

    def __init__(self):
        self._params = {}
        self._channels = [{'index': i, 'offset_raw': 0, 'gain_ppm': 0}
                          for i in range(self.CHANNELS)]

    def read(self):
        return {'stored': False, 'version': 0, 'params': dict(self._params),
                'channels': [dict(c) for c in self._channels],
                'soa_limit_c': [], 'soa_throttle_at': 0.0}

    def set_param(self, name, value):
        """Held, not invented: what a caller wrote is what it reads back."""
        if name not in protocol.CAL_PARAMS:
            raise DeviceStateError('%r is not a calibration parameter (simulated)'
                                   % (name,))
        self._params[name] = int(value) & 0xFFFFFFFF

    def set_channel(self, index, offset_raw, gain_ppm):
        if not 0 <= index < self.CHANNELS:
            raise DeviceStateError('no channel %d (simulated)' % index)
        self._channels[index] = {'index': index, 'offset_raw': int(offset_raw),
                                 'gain_ppm': int(gain_ppm)}

    def zero(self, index):
        """Measure the channel now and keep the reading as its offset.

        IT HAS TO MEASURE. Storing a flat zero made a tare on the
        stand-in a no-op that still reported success - the currents
        came back at the same offset they went in with, and nothing
        said the call had done nothing. The board reads the channel;
        so does this.
        """
        code = 0
        if self.board is not None:
            for row in self.board.analog.read_all()['channels']:
                if row['index'] == index:
                    code = int(row['mean_raw'])
                    break
        self.set_channel(index, code, self._channels[index]['gain_ppm'])
        return code

    def span(self, index, reference):
        raise DeviceStateError('the stand-in has no instrument to span '
                               'against (simulated)')

    def save(self):
        return True

    def load(self):
        return False

    def defaults(self):
        self.__init__()
        return True
