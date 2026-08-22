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
