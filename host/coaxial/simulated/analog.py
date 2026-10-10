"""The analog front end, its seven channels and the calibration record: invariant 9 without an ADC.
"""
import math
import random
import time
from typing import Any, Callable

from coaxial.comm import protocol
from coaxial.devices import scaling
from coaxial.devices.calibration import CalibrationOps
from coaxial.devices.scaling import ADC_CODES, ADC_HALF_CODES
from coaxial.devices.thermal import THROTTLE_AT
from coaxial.errors import DeviceStateError
from coaxial.simulated.system import UNITS
from coaxial.simulated.values import (AMPS_PER_CODE, CHANNELS, DRIFT, NOMINAL, _spread,
                                      phase_codes, quiet_code)
from machine.roles import Input, Output


class SimulatedAfe(Output):
    """The stand-in AFE; PE15 the STO chain's FAULTOUT, which the board wires."""

    def __init__(self):
        self._on = False
        self._pe15: Callable[[], bool] = lambda: False

    def state(self):
        return {'on': self._on, 'pe15': self._pe15(), 'users': ['host'] if self._on else []}

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


class SimulatedAnalog(Input):
    """Invented readings, in the shape the real ones come in."""

    #: Its noise's seed: on the module's random a flight on four stand-ins was another flight
    #: run again, its boards' SOA 0.83 then 0.91 (2026-10-10).
    NOISE_SEED = 2

    def __init__(self, afe):
        self._afe = afe
        self._random = random.Random(self.NOISE_SEED)
        #: The drive whose current the phases carry - the board wires it.
        self.drive: Any = None
        #: The thermal stand-in whose MCU die the die channel reads - the board wires it.
        self.thermal: Any = None
        #: The STO chain whose Cinj, Clevel and +15V7 three channels read, and the calibration
        #: record the scaling is - the board wires both.
        self.sto: Any = None
        self.calibration: Any = None

    def scaling(self, refresh=False):
        """What the board's own record produces: the stand-in's record, once the board wired it."""
        del refresh
        return scaling.from_calibration(self.calibration.read() if self.calibration else {})

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

    def burst(self, mask, samples, rate=None, trimmed=True):
        """The firmware's burst: each channel through the record (Board_CalApply), `trimmed`
        False the uncorrected read a zero takes."""
        chosen = {}
        # The motor's current on the phases, the same one a record carries:
        # what the drive holds, at the angle it holds it.
        drive = self.drive
        amps, theta = drive._carrying() if drive is not None else (0.0, 0.0)
        omega = drive._omega() if drive is not None else 0.0
        window = samples / float(rate or 2000.0)
        swing = amps * min(2.0, abs(omega) * window) / AMPS_PER_CODE / 2.0
        thermal = self.thermal
        for meta in CHANNELS:
            index = meta['index']
            if not (mask >> index & 1):
                continue
            if self._afe._on:
                mean = (quiet_code(index, thermal, self.sto)
                        + phase_codes(meta['signal'], amps, theta)
                        + self._random.uniform(-DRIFT[index], DRIFT[index]))
                record = self.calibration
                if trimmed and record is not None:
                    mean = record.apply(index, mean)
            else:
                # Invariant 9: with the reference unpowered, a differential
                # input sits at 0 and a single-ended one at mid-scale, as
                # measured on the board.
                mean = 0.0 if meta['differential'] else ADC_HALF_CODES
            chosen[index] = _spread(
                meta, mean, self._afe._on, self._random,
                swing if meta['signal'] in ('Phase U', 'Phase V', 'Phase W')
                else 0.0)
        return {'samples': samples, 'rate_hz': rate or 2000.0,
                'channels': chosen}

    def ntc_temperature(self, adc_chan=None, ntc_params=None,
                        samples=64, sample_rate=2000.0):
        """The NTC in the shape `Analog.ntc_temperature` returns it."""
        celsius = 31.4 + 0.6 * math.sin(time.time() / 30.0)
        return {
            'celsius': celsius,
            'ohms': 10000.0,
            'spread_millikelvin': 28.0,
            'params': 'simulated',
            'mean_raw': 40500,
            'samples': samples,
        }

    def mask_all(self):
        return (1 << len(CHANNELS)) - 1

    def phase_current(self, signal='Phase U', shunt=None, samples=64, sample_rate=2000.0):
        """A phase at rest in the shape `Analog.phase_current` returns it."""
        shunt = shunt or scaling.PHASE_ONBOARD
        return {'amps': 0.0, 'volts_at_pin': shunt.volts_at_pin(ADC_HALF_CODES),
                'ripple_amps': 0.05, 'noise_amps_rms': 0.01,
                'full_scale_amps': shunt.full_scale_amps, 'params': 'simulated',
                'mean_raw': ADC_HALF_CODES, 'samples': samples}

    def noise(self, adc, samples=200):
        """A quiet converter in the shape of the firmware's noise report."""
        return {'samples': samples, 'mean_uv': 0, 'min_raw': -6, 'max_raw': 6,
                'span_raw': 12, 'stddev_uv': 150}

    def dcbus_voltage(self, adc_chan=None, divider=None,
                      samples=64, sample_rate=2000.0):
        """The DC link in the shape `Analog.dcbus_voltage` returns it."""
        return {
            'volts': 24.5,
            'volts_at_pin': 24.5 / 23.68,
            'ripple_volts': 0.025,
            'noise_volts_rms': 0.004,
            'scale': 23.68,
            'params': 'simulated',
            'mean_raw': 20375,
            'samples': samples,
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
            'pe15': self._afe.state()['pe15'],
        }

    def state(self):
        return {'channels': CHANNELS}

    def read(self, samples=64, sample_rate=1000.0, vref=3.3):
        """Every channel with its table row merged in, like the real one."""
        table = {row['index']: row for row in CHANNELS}
        result = self.burst((1 << len(CHANNELS)) - 1, samples, sample_rate)

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
        # A zeroed board's record: each phase's zero its rest offset, what a tare stores.
        self._channels = [{'index': i, 'gain_ppm': 0,
                           'offset_raw': int(NOMINAL[i]) if CHANNELS[i]['differential'] else 0}
                          for i in range(self.CHANNELS)]

    def read(self):
        return {'stored': False, 'version': 0, 'params': dict(self._params),
                'channels': [dict(c) for c in self._channels],
                'soa_limit_c': [], 'soa_throttle_at': THROTTLE_AT}

    def _set_param(self, name, value):
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
        """Board_CalZero's: the channel's uncorrected reading now kept as its offset."""
        taken = (self.board.analog.burst(1 << index, 64, trimmed=False)['channels']
                 if self.board is not None else {})
        code = int(taken[index]['mean_raw']) if index in taken else 0
        self.set_channel(index, code, self._channels[index]['gain_ppm'])
        return code

    def apply(self, index, code):
        """Board_CalApply's: `code` less the channel's zero, times its gain."""
        channel = self._channels[index]
        corrected = code - channel['offset_raw']
        return corrected + corrected * channel['gain_ppm'] / 1e6

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
