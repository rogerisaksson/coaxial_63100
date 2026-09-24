"""The analog front end, its seven channels and the calibration record: invariant 9 without an ADC.
"""
import math
import random
import time
from typing import Any

from coaxial.comm import protocol
from coaxial.devices import scaling
from coaxial.devices.calibration import CalibrationOps
from coaxial.devices.roles import Output
from coaxial.devices.scaling import ADC_CODES, ADC_HALF_CODES
from coaxial.errors import DeviceStateError
from coaxial.simulated.system import UNITS
from coaxial.simulated.values import (AMPS_PER_CODE, CHANNELS, DRIFT, NOMINAL, _spread, _sweep,
                                      phase_codes)


class SimulatedAfe(Output):
    """The stand-in AFE; PE15 follows AFE_ON inversely, as measured on the board."""

    def __init__(self):
        self._on = False

    def state(self):
        return {'on': self._on, 'pe15': not self._on, 'users': ['host'] if self._on else []}

    def require(self):
        if not self._on:
            raise DeviceStateError('AFE_ON is off (simulated)')
        return True

    def on(self):
        self._on = True
        return True

    def off(self):
        self._on = False
        return False

    def write(self, on):
        return self.on() if on else self.off()

    def toggle(self):
        self._on = not self._on
        return self._on


class SimulatedAnalog:
    """Invented readings, in the shape the real ones come in."""
    def __init__(self, afe):
        self._afe = afe
        #: The drive whose current the phases carry - the board wires it.
        self.drive: Any = None

    def scaling(self, refresh=False):
        """The same shape the board's own record produces."""
        del refresh
        return scaling.from_calibration({})

    def channels(self, refresh=False):
        return CHANNELS

    def names(self):
        """Signal names in the board's order."""
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
        # THE MACHINE'S CURRENT ON THE PHASES, the same one a record carries:
        # what the drive holds, at the angle it holds it.
        drive = self.drive
        amps, theta = drive._carrying() if drive is not None else (0.0, 0.0)
        omega = drive._omega() if drive is not None else 0.0
        window = samples / float(rate or 2000.0)
        swing = amps * min(2.0, abs(omega) * window) / AMPS_PER_CODE / 2.0
        for meta in CHANNELS:
            index = meta['index']
            if not (mask >> index & 1):
                continue
            if self._afe._on:
                mean = (NOMINAL[index] + _sweep(index)
                        + phase_codes(meta['signal'], amps, theta)
                        + random.uniform(-DRIFT[index], DRIFT[index]))
            else:
                # Invariant 9, reproduced exactly: with the reference
                # unpowered, a differential input sits at 0 and a single-ended
                # one at mid-scale - measured on real hardware, not a rounder
                # number picked to look plausible.
                mean = 0.0 if meta['differential'] else ADC_HALF_CODES
            chosen[index] = _spread(
                meta, mean, self._afe._on,
                swing if meta['signal'] in ('Phase U', 'Phase V', 'Phase W')
                else 0.0)
        return {'samples': samples, 'rate_hz': rate or 2000.0,
                'channels': chosen}

    def ntc_temperature(self, adc_chan=None, ntc_params=None,
                        nr_of_samples=64, sample_rate=2000.0):
        """The NTC in the shape `Analog.ntc_temperature` returns it."""
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
        """
        if not self._afe.is_on():
            raise DeviceStateError(
                'the scan reports the analog front end off, so every channel '
                'read mid-scale: ntc_centidegc would be exactly 2500 and '
                'dcbus_mv a plausible number that is not a measurement. '
                'Call board.afe.on() first.')

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
        """Every channel with its table row merged in, like the real one."""
        table = {row['index']: row for row in CHANNELS}
        result = self.burst((1 << len(CHANNELS)) - 1, nr_of_samples,
                            sample_rate)

        rows = []
        for index, stats in sorted(result['channels'].items()):
            row = dict(table[index])
            row.update(stats)
            row['unit'] = UNITS.get(row['signal'])
            row['stddev_raw'] = 1.0
            divisor = ADC_HALF_CODES if row['differential'] else ADC_CODES
            row['volts_at_pin'] = stats['mean_raw'] / divisor * vref
            rows.append(row)

        return {'samples': result['samples'], 'rate_hz': result['rate_hz'],
                'channels': rows}


class SimulatedCalibration(CalibrationOps):
    """The record an uncalibrated board holds: `stored` false, and the
    firmware's compiled-in defaults behind it.
    """
    #: Channels the record trims, as many as the board's ADC table has.
    CHANNELS = 10

    #: The board this belongs to, so `zero()` can read a channel the
    #: way the real one does. Set by SimulatedBoard.
    board: Any = None
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
        """Measure the channel now and keep the reading as its offset."""
        rows = (self.board.analog.read_all()['channels']
                if self.board is not None else ())
        code = next((int(row['mean_raw']) for row in rows
                     if row['index'] == index), 0)
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
