"""Invented readings: the channel table, nominals, drift and tumble every device draws from.
"""
import math

from coaxial.devices.scaling import ADC_CODES, KELVIN_AT_ZERO_C, NTC_ONBOARD

#: The stand-in's clock, as the board reports it: 475 MHz, the cycle
#: counter at that rate, HCLK at half.
SYSCLK_HZ = 475000000
TICKS_PER_US = SYSCLK_HZ // 1000000
#: The board's record ring (board_limits.h) and the most sweeps a record
#: may accumulate before its sum overflows (LIVE_MAX_ADDITIONS).
RING_BYTES = 448 * 1024
ACCUMULATE_MAX = 32767
#: A 32-bit counter's wrap.
MASK32 = 0xFFFFFFFF


CHANNELS = [
    {'index': 0, 'adc': 3, 'channel': 1, 'pin': 'PC3_C/PC2_C',
     'differential': True, 'signal': 'Phase U'},
    {'index': 1, 'adc': 1, 'channel': 3, 'pin': 'PA6/PA7',
     'differential': True, 'signal': 'Phase V'},
    {'index': 2, 'adc': 2, 'channel': 4, 'pin': 'PC4/PC5',
     'differential': True, 'signal': 'Phase W'},
    {'index': 3, 'adc': 2, 'channel': 5, 'pin': 'PB1',
     'differential': False, 'signal': 'Clevel'},
    {'index': 4, 'adc': 1, 'channel': 9, 'pin': 'PB0',
     'differential': False, 'signal': 'NTC'},
    {'index': 5, 'adc': 3, 'channel': 10, 'pin': 'PC0',
     'differential': False, 'signal': 'DC bus'},
    {'index': 6, 'adc': 3, 'channel': 11, 'pin': 'PC1',
     'differential': False, 'signal': 'Cinj'},
    # The two supply senses.
    {'index': 7, 'adc': 1, 'channel': 18, 'pin': 'PA4',
     'differential': False, 'signal': '+5V'},
    {'index': 8, 'adc': 1, 'channel': 19, 'pin': 'PA5',
     'differential': False, 'signal': 'Vgate'},
    # The die's own thermometer: no pin, and ADC3 only.
    {'index': 9, 'adc': 3, 'channel': 18, 'pin': 'internal',
     'differential': False, 'signal': 'MCU die'},
]

# What a live board reads with the front end on and nothing moving, AFE gain and all: the
# phases' offsets off a bench boot, the link and the rails where they sit; the NTC and the die
# follow the thermal stand-in, Cinj, Clevel and Vgate the STO chain, where the board wires them
# (`quiet_code`).
NOMINAL = {0: 1400.0, 1: -8030.0, 2: 360.0, 3: 1010.0, 4: 41000.0,
          5: 20775.0, 6: 16500.0, 7: 50700.0, 8: 1030.0,
          9: 33000.0}

#: What the stand-in's DC link IS, in volts: its rest code through the
#: divider (78.15 V full scale over 16 bits). One number, derived - the
#: drive reported 24.0, the DAQ's modulation index divided by 31.0 and
#: the DC bus channel read 24.8, and an identification off a recorded
#: frame folded the disagreement into every constant it recovered.
DCBUS_V = NOMINAL[5] * 78.15 / ADC_CODES
#: Vgate's pin over the gate drivers' supply: 10k under 57k (HARDWARE.md).
VGATE_PIN_RATIO = 10.0 / 67.0
#: The MCU's temperature sensor at 30 C and a kelvin, V (the part's typicals, native.c's and
#: Coaxial63100_AFE.cs's): the die channel reads its die, not mid-scale's 545 C.
DIE_V_AT_30, DIE_V_PER_K = 0.62, 0.002


def die_code(celsius):
    """The die channel's code at `celsius`."""
    return (DIE_V_AT_30 + (celsius - 30.0) * DIE_V_PER_K) / 3.3 * ADC_CODES


def ntc_code(celsius):
    """The NTC channel's code at `celsius`: the element on the board's divider
    (`scaling.NTC_ONBOARD`), high side as it sits."""
    ntc = NTC_ONBOARD
    ohms = ntc.r25 * math.exp(ntc.beta * (1.0 / (celsius + KELVIN_AT_ZERO_C)
                                          - 1.0 / ntc.t25_kelvin))
    share = (ntc.r_fixed / (ohms + ntc.r_fixed) if ntc.high_side
             else ohms / (ohms + ntc.r_fixed))
    return share * ADC_CODES


def pin_code(volts):
    """A single-ended pin's code at `volts`, the converter's range clipping it."""
    return max(0.0, min(ADC_CODES, volts / 3.3 * ADC_CODES))


def quiet_code(index, thermal=None, sto=None):
    """A channel's code with no current on it: the NTC's element and the MCU's die off the
    thermal stand-in, Cinj, Clevel and Vgate off the STO chain, where the board wired them; the
    rest where it sits."""
    signal = CHANNELS[index]['signal']
    if thermal is not None and signal == 'NTC':
        return ntc_code(thermal._thermistor())
    if thermal is not None and signal == 'MCU die':
        return die_code(thermal._die('mcu'))
    if sto is not None and signal in ('Cinj', 'Clevel', 'Vgate'):
        sto.advance()
        return pin_code({'Cinj': sto.cinj, 'Clevel': sto.clevel,
                         'Vgate': sto.vgate * VGATE_PIN_RATIO}[signal])
    return NOMINAL[index]


#: How far a channel moves between reads, codes: the phases' measured 0.35-0.41 A floor, the
#: quiet ones the converter's few codes - an NTC does not jump 2.5 K a read.
DRIFT = {0: 40.0, 1: 60.0, 2: 40.0, 3: 5.0, 4: 5.0, 5: 20.0, 6: 20.0,
         7: 30.0, 8: 20.0, 9: 20.0}

#: How far a channel moves WITHIN one burst - a different quantity from how
#: far it wanders between them. A flat +/-5 codes for everything is 0.015 %
#: of a differential range, so the burst extremes drew under the bar and two
#: of the views' three marks were invisible. Sized by where each channel
#: sits, not to look busy.
RIPPLE = {0: 60.0, 1: 60.0, 2: 60.0, 3: 40.0, 4: 150.0, 5: 700.0,
          6: 300.0, 7: 60.0, 8: 40.0, 9: 80.0}

#: Now and then a burst catches something bigger. Without it every burst is
#: the same width and the held peak sits a constant distance from the bar,
#: which reads as decoration rather than as memory.
GUST_CHANCE = 0.14
GUST = 2.8


#: Radians a phase lags the one before it.
PHASE_STEP = 2.0 * math.pi / 3.0

#: Which leg each phase channel is, by the board's own name.
PHASE_LEG = {'Phase U': 0, 'Phase V': 1, 'Phase W': 2}

#: Amps per code on a phase shunt - `SimulatedDrive.APC`, the same
#: number `scaling.PHASE_ONBOARD` gives: 3.3 V over 32768 codes
#: through 3.5 mohm times 4.5455.
AMPS_PER_CODE = 3.3 / 32768.0 / (0.0035 * 1500.0 / 330.0)


def phase_codes(signal, amps, theta):
    """The motor's current on one phase, in codes a sample: `amps` of
    stator current at electrical angle `theta`, put into the leg's own
    phase.
    """
    leg = PHASE_LEG.get(signal)
    if leg is None or not amps:
        return 0.0
    return amps * math.cos(theta - leg * PHASE_STEP) / AMPS_PER_CODE


def _spread(meta, mean, powered, rng, extra=0.0):
    """One burst's mean and its two extremes, as the board reports them, its noise `rng`'s."""
    index = meta['index']
    if not powered:
        return {'mean_raw': mean, 'min_raw': int(mean), 'max_raw': int(mean)}

    reach = RIPPLE[index] * rng.uniform(0.55, 1.0)
    if rng.random() < GUST_CHANCE:
        reach *= GUST
    # What the motor moved within the burst, on top of the noise.
    reach += extra

    floor, ceiling = ((-32768, 32767) if meta['differential']
                      else (0, 65535))
    low = max(floor, mean - reach * rng.uniform(0.7, 1.0))
    high = min(ceiling, mean + reach * rng.uniform(0.7, 1.0))
    return {'mean_raw': mean, 'min_raw': int(low), 'max_raw': int(high)}


#: Turns per TUMBLE_S about the board's own X and Y. Whole numbers on
#: purpose: the attitude comes back where it started once a period.
#:
#: About X and Y rather than Z, which is what this used to do. A rotation
#: about Z is the board spinning in its own plane - roll and pitch stay at
#: zero, the silhouette never changes, and a view built to show attitude
#: shows one number moving. Two unequal rates about the other two axes make
#: it tumble, so all three angles move and the drawing has depth to show.
ROLL_TURNS = 1.0
PITCH_TURNS = 2.0

#: The tumble's period, s, the emulator's and native's too: at 2.56 s the attitude page
#: turned 140 and 280 deg/s, and stepped a read the stand-in turned once in 1.28 s at the
#: page's 200 reads a second (2026-09-26).
TUMBLE_S = 25.6


def _tumble(seconds, unit):
    """(i, j, k, real) counts for the stand-in's attitude, `seconds` into its tumble."""
    roll = seconds * ROLL_TURNS * 2.0 * math.pi / TUMBLE_S
    pitch = seconds * PITCH_TURNS * 2.0 * math.pi / TUMBLE_S
    sin_r, cos_r = math.sin(roll / 2.0), math.cos(roll / 2.0)
    sin_p, cos_p = math.sin(pitch / 2.0), math.cos(pitch / 2.0)

    # Nod about Y, then roll about X - the product of the two, in the (i, j, k,
    # real) order a rotation vector is reported in.
    return (int(sin_r * cos_p * unit), int(cos_r * sin_p * unit),
            int(-sin_r * sin_p * unit), int(cos_r * cos_p * unit))
