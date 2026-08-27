"""Conversion from raw ADC codes to physical quantities.

The firmware deliberately reports raw codes and nothing else. Everything in this
module is the FIXTURE's knowledge of what those codes mean, so a board with a
different divider or a different thermistor needs new numbers here rather than
new firmware.
"""
import math


class NtcParams:
    """A thermistor and the divider it sits in.

    high_side means the NTC is between the reference rail and the ADC node, with
    r_fixed from the node to ground, which is how this board is wired.

    The conversion is RATIOMETRIC: the reference voltage cancels out of
    r_fixed * (65536/raw - 1), so an inaccurate rail does not bias the
    temperature. That is worth knowing, because the same is emphatically not
    true of DividerParams below.
    """

    def __init__(self, r25=10000.0, beta=3380.0, r_fixed=10000.0,
                 t25_kelvin=298.15, high_side=True, name=None):
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
        fraction = raw / 65536.0
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
        return 1.0 / inverse - 273.15


class DividerParams:
    """A resistive divider ahead of a single-ended ADC input.

    Unlike the thermistor this is an ABSOLUTE measurement: the answer scales
    directly with vref, so an error in the reference is an error in the result.
    Pass a measured rail if you need better than a percent.
    """

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
        return raw / 65536.0 * self.vref

    def volts(self, raw):
        return self.volts_at_pin(raw) * self.scale + self.offset_v


class ShuntParams:
    """A current shunt and the differential amplifier chain above it.

    ABSOLUTE, like DividerParams and for the same reason: the answer scales
    with vref, and with two more numbers that belong to the board rather than
    to the ADC. Get either wrong and the current is wrong by that factor.

    RU1 || RU2 sit in the phase conductor, two Vishay WSHM2818 of 7 mohm
    each, tapped by RU3/RU4 into a THS4551 with Rg 330 and Rf 1.5k. Both
    outputs swing in anti-phase about +1V65_bias, so the full +/-vref
    differential span is reachable and 100 A lands at 48 % of it - the same
    deliberate headroom the DC link divider keeps, for the same reason.

    The gain is bounded as well as traced: 100 A across 3.5 mohm is 350 mV,
    and vref/that is 9.43 V/V, so anything above it could not represent the
    board's own rating. That is what rules out reading the ADA4891 quad on
    the same sheet as further gain in this path - see docs/HARDWARE.md.
    """

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
    return raw / 32768.0 * vref


def single_ended_volts(raw, vref=3.3):
    return raw / 65536.0 * vref


# This board as built. Named constants so a call site reads as a deliberate
# choice rather than an accepted default.
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


#: The two supply senses, off R113 and R119 on the MCU sheet. Named apart
#: from the DC link because they are millivolts through a different divider
#: - a unit says what a number is, not what scaled it.
RAIL5_ONBOARD = DividerParams(r_top=10000.0, r_bottom=10000.0, vref=3.3,
                              name='onboard R113 10k/10k')
VGATE_ONBOARD = DividerParams(r_top=57000.0, r_bottom=10000.0, vref=3.3,
                              name='onboard R119 47k + R113 10k over 10k')

#: Which divider a millivolt channel is on, by the name the board gives it.
#: Three channels report mV and no two share a divider.
BY_SIGNAL = {'+5V': RAIL5_ONBOARD, 'Vgate': VGATE_ONBOARD}


def converter(unit, differential=False, vref=3.3, signal=None):
    """The board's conversion for a channel, chosen by the unit it reports.

    Here rather than in a view because two of them need it now, and the
    second copy is always the one that goes stale (invariant 7). A channel
    with no unit of its own is read as volts at the pin, which is the only
    thing a bare code can honestly be called.

    `signal` picks the divider where a unit cannot: the DC link, the +5 rail
    and the gate supply all report millivolts through three different ones.
    """
    if unit == 'mA':
        return PHASE_ONBOARD.amps
    if unit == 'mV':
        return BY_SIGNAL.get(signal, DCBUS_ONBOARD).volts
    if unit == 'centi-degC':
        return NTC_ONBOARD.celsius

    full = 32768.0 if differential else 65536.0
    return lambda code: code / full * vref
