"""Conversion from raw ADC codes to physical quantities."""
import math

#: The converter's 16-bit result: the full scale of codes, and the half of
#: it that is a differential reading's span.
ADC_CODES = 65536.0
ADC_HALF_CODES = 32768.0
KELVIN_AT_ZERO_C = 273.15


class NtcParams:
    """A thermistor and the divider it sits in."""

    def __init__(self, r25=10000.0, beta=3380.0, r_fixed=10000.0,
                 t25_kelvin=KELVIN_AT_ZERO_C + 25.0, high_side=True, name=None):
        self.r25 = r25
        self.beta = beta
        self.r_fixed = r_fixed
        self.t25_kelvin = t25_kelvin
        self.high_side = high_side
        self.name = name or 'R25=%.0f B=%.0f' % (r25, beta)

    def __repr__(self):
        return '<NtcParams %s>' % self.name

    def resistance(self, raw):
        """Thermistor resistance in ohms, from a single-ended raw code."""
        fraction = raw / ADC_CODES
        if not 0.0 < fraction < 1.0:
            raise ValueError('raw %r sits at a divider rail; the resistance is '
                             'not recoverable there' % (raw,))
        if self.high_side:
            return self.r_fixed * (1.0 / fraction - 1.0)
        return self.r_fixed * fraction / (1.0 - fraction)

    def celsius(self, raw):
        """Temperature by the B-parameter form of the Steinhart-Hart equation."""
        ohms = self.resistance(raw)
        inverse = 1.0 / self.t25_kelvin + math.log(ohms / self.r25) / self.beta
        return 1.0 / inverse - KELVIN_AT_ZERO_C


class DividerParams:
    """A resistive divider ahead of a single-ended ADC input."""

    def __init__(self, r_top=49900.0, r_bottom=2200.0, vref=3.3, offset_v=0.0,
                 name=None):
        self.r_top = r_top
        self.r_bottom = r_bottom
        self.vref = vref
        self.offset_v = offset_v
        self.name = name or '%.1fk/%.1fk' % (r_top / 1000.0, r_bottom / 1000.0)

    def __repr__(self):
        return '<DividerParams %s vref=%.3f>' % (self.name, self.vref)

    @property
    def scale(self):
        """Ratio from pin volts to source volts."""
        return (self.r_top + self.r_bottom) / self.r_bottom

    def volts_at_pin(self, raw):
        return raw / ADC_CODES * self.vref

    def volts(self, raw):
        return self.volts_at_pin(raw) * self.scale + self.offset_v


class ShuntParams:
    """A current shunt and the differential amplifier chain above it."""

    def __init__(self, r_shunt=0.0035, gain=1500.0 / 330.0,
                 vref=3.3, name=None):
        self.r_shunt = r_shunt
        self.gain = gain
        self.vref = vref
        self.name = name or '%.2f mohm x %.1f' % (r_shunt * 1000.0, gain)

    def __repr__(self):
        return '<ShuntParams %s vref=%.3f>' % (self.name, self.vref)

    @property
    def volts_per_amp(self):
        return self.r_shunt * self.gain

    @property
    def full_scale_amps(self):
        """Where the ADC runs out, not where the board does."""
        return self.vref / self.volts_per_amp

    def volts_at_pin(self, raw):
        return differential_volts(raw, self.vref)

    def amps(self, raw):
        return self.volts_at_pin(raw) / self.volts_per_amp


def differential_volts(raw, vref=3.3):
    """A differential code is offset binary already centred by the firmware."""
    return raw / ADC_HALF_CODES * vref


def single_ended_volts(raw, vref=3.3):
    return raw / ADC_CODES * vref


# This board as built - the FALLBACK, for a caller with no board to ask.
NTC_ONBOARD = NtcParams(r25=10000.0, beta=3380.0, r_fixed=10000.0,
                        name='Murata NCU18XH103, onboard')
DCBUS_ONBOARD = DividerParams(r_top=49900.0, r_bottom=2200.0, vref=3.3,
                              name='onboard 49.9k/2.2k')
PHASE_ONBOARD = ShuntParams(r_shunt=0.0035, vref=3.3,
                            name='RU1||RU2 3.5 mohm, THS4551 1.5k/330')


#: What a channel's reported unit converts to, and the symbol to print it
#: with. The acquisition task buffers converter codes and does not scale
#: them - the unit in a layout says what the channel means, not what the
#: number is in - so anything showing a DAQ record has to do this.
UNIT_SYMBOL = {'mA': 'A', 'mV': 'V', 'centi-degC': 'C', None: 'V'}

#: The one `centi-degC` channel this host can convert. The other is the MCU's
#: own die sensor, whose curve is the factory TS_CAL pair in system memory -
#: not on this board and not in its record, so no arithmetic here can reach
#: it. The firmware has it and reports the die cooked, over 0x6E device 8.
THERMISTOR_SIGNAL = 'NTC'


def symbol(unit, signal=None):
    """What to print beside a converted value."""
    if unit == 'centi-degC' and signal not in (None, THERMISTOR_SIGNAL):
        return 'V'
    return UNIT_SYMBOL.get(unit, '')


#: The two supply senses, off R113 and R119 on the MCU sheet. Named apart
#: from the DC link because they are millivolts through a different divider
#: - a unit says what a number is, not what scaled it.
RAIL5_ONBOARD = DividerParams(r_top=10000.0, r_bottom=10000.0, vref=3.3,
                              name='onboard R113 10k/10k')
VGATE_ONBOARD = DividerParams(r_top=57000.0, r_bottom=10000.0, vref=3.3,
                              name='onboard R119 47k + R113 10k over 10k')

#: What a set of parameters calls itself, so a reading says where its numbers
#: came from rather than repeating them. The distinction is not cosmetic: a
#: value cooked from the fallback is the schematic's arithmetic, and one
#: cooked from the record is what the board was told it is.
FROM_RECORD = "the board's own record"
FROM_FALLBACK = "compiled-in fallback, the board sent no record"


def from_calibration(cal):
    """The board's own scaling, out of `calibration.read()`."""
    p = cal.get('params', {})
    vref = p.get('vref_uv', 3300000) / 1e6
    where = FROM_RECORD if p else FROM_FALLBACK

    return {
        'ntc': NtcParams(
            r25=float(p.get('ntc_r25_ohm', NTC_ONBOARD.r25)),
            beta=p.get('ntc_beta_mk', NTC_ONBOARD.beta * 1000) / 1000.0,
            r_fixed=float(p.get('ntc_rfixed_ohm', NTC_ONBOARD.r_fixed)),
            t25_kelvin=p.get('ntc_t25_ck', 29815) / 100.0,
            name=where),
        'dcbus': DividerParams(
            r_top=float(p.get('bus_r_top_ohm', DCBUS_ONBOARD.r_top)),
            r_bottom=float(p.get('bus_r_bottom_ohm', DCBUS_ONBOARD.r_bottom)),
            vref=vref, name=where),
        'phase': ShuntParams(
            r_shunt=p.get('shunt_uohm', 3500) / 1e6,
            gain=p.get('amp_gain_ppm', 4545455) / 1e6,
            vref=vref, name=where),
        'rail5': DividerParams(
            r_top=float(p.get('r5_r_top_ohm', RAIL5_ONBOARD.r_top)),
            r_bottom=float(p.get('r5_r_bottom_ohm', RAIL5_ONBOARD.r_bottom)),
            vref=vref, name=where),
        'vgate': DividerParams(
            r_top=float(p.get('vg_r_top_ohm', VGATE_ONBOARD.r_top)),
            r_bottom=float(p.get('vg_r_bottom_ohm', VGATE_ONBOARD.r_bottom)),
            vref=vref, name=where),
    }


def converter(unit, differential=False, vref=3.3, signal=None, params=None):
    """The board's conversion for a channel, chosen by the unit it reports."""
    p = params or {}
    if unit == 'mA':
        return p.get('phase', PHASE_ONBOARD).amps
    if unit == 'mV':
        by_signal = {'+5V': p.get('rail5', RAIL5_ONBOARD),
                     'Vgate': p.get('vgate', VGATE_ONBOARD)}
        return by_signal.get(signal or '', p.get('dcbus', DCBUS_ONBOARD)).volts
    if unit == 'centi-degC' and signal in (None, THERMISTOR_SIGNAL):
        return p.get('ntc', NTC_ONBOARD).celsius

    if params:
        vref = p['dcbus'].vref
    full = ADC_HALF_CODES if differential else ADC_CODES
    return lambda code: code / full * vref
