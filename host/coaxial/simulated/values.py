"""Invented readings: the channel table, nominals, drift, sweep and
tumble textures every simulated device draws from."""
import math
import random
import time


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
    # The two supply senses. No unit, because their dividers are not in the
    # calibration record yet, so a host reads volts at the pin - x2.00 for
    # the +5 rail and x6.70 for the gate supply, per R113 and R119.
    {'index': 7, 'adc': 1, 'channel': 18, 'pin': 'PA4',
     'differential': False, 'signal': '+5V'},
    {'index': 8, 'adc': 1, 'channel': 19, 'pin': 'PA5',
     'differential': False, 'signal': 'Vgate'},
    # The die's own thermometer: no pin, and ADC3 only. Channel 18 is what
    # the board answered with - LL_ADC_CHANNEL_TEMPSENSOR has three variants
    # behind preprocessor conditions, so it could not be read off the source.
    {'index': 9, 'adc': 3, 'channel': 18, 'pin': 'internal',
     'differential': False, 'signal': 'MCU die'},
]

# Roughly what a live board reads with the front end on, AFE gain and all -
# not a calibrated value, just something to drift around so a repeated read
# does not look frozen. Phase channels stay near their own nominal point;
# NTC and DC bus get a slightly wider walk since those are what a question
# is usually about.
# 7 and 8 are the supply senses, near what the board reads: +5 through a
# 10 k/10 k divider is 2.55 V of 3.3, and the gate supply sits near zero
# because the STO chain has not released it.
NOMINAL = {0: 1400.0, 1: -8030.0, 2: 360.0, 3: 1010.0, 4: 41000.0,
          5: 20775.0, 6: 16500.0, 7: 50700.0, 8: 1030.0,
          9: 33000.0}

#: What the stand-in's DC link IS, in volts: its rest code through the
#: divider (78.15 V full scale over 16 bits). One number, derived - the
#: drive reported 24.0, the DAQ's modulation index divided by 31.0 and
#: the DC bus channel read 24.8, and an identification off a recorded
#: frame folded the disagreement into every constant it recovered.
DCBUS_V = NOMINAL[5] * 78.15 / 65536.0
DRIFT = {0: 40.0, 1: 60.0, 2: 40.0, 3: 5.0, 4: 800.0, 5: 500.0, 6: 400.0,
         7: 30.0, 8: 20.0, 9: 60.0}

#: One electrical revolution every seven seconds or so. Slow enough to watch
#: a meter follow it, fast enough that a still frame is rarely the same twice.
SWEEP_HZ = 0.14

#: How far each channel swings, in codes. The three phases get most of the
#: converter, because a stand-in whose meters never leave the bottom segment
#: teaches nobody what the view looks like with a machine running.
SWING = {0: 9000.0, 1: 9000.0, 2: 9000.0, 3: 300.0, 4: 6000.0, 5: 4000.0,
         6: 3000.0, 7: 200.0, 8: 100.0, 9: 300.0}

#: How far a channel moves WITHIN one burst - a different quantity from how
#: far it wanders between them. A flat +/-5 codes for everything is 0.015 %
#: of a differential range, so the burst extremes drew under the bar and two
#: of the views' three marks were invisible. Sized by where each channel
#: sits, not to look busy.
RIPPLE = {0: 2600.0, 1: 2600.0, 2: 2600.0, 3: 40.0, 4: 150.0, 5: 700.0,
          6: 300.0, 7: 60.0, 8: 40.0, 9: 80.0}

#: Now and then a burst catches something bigger. Without it every burst is
#: the same width and the held peak sits a constant distance from the bar,
#: which reads as decoration rather than as memory.
GUST_CHANCE = 0.14
GUST = 2.8


def _sweep(index):
    """Where a simulated channel sits right now.

    Invented, and deliberately not still: the three phases run 120 degrees
    apart like a machine turning, and the rest wander. Every number this
    module produces is made up - see the module docstring - and a moving one
    is no more a measurement than a still one. It is here so the views can be
    demonstrated and developed without a cable.
    """
    turn = time.time() * SWEEP_HZ * 2.0 * math.pi

    if index in (0, 1, 2):
        return SWING[index] * math.sin(turn - index * 2.0 * math.pi / 3.0)
    return SWING[index] * math.sin(turn * 0.31 + index)


def _spread(meta, mean, powered):
    """One burst's mean and its two extremes, as the board reports them.

    With the front end off there is nothing to ripple: invariant 9 says the
    input sits exactly at its rail, and a stand-in that jitters there would
    teach the opposite of what the invariant is for.
    """
    index = meta['index']
    if not powered:
        return {'mean_raw': mean, 'min_raw': int(mean), 'max_raw': int(mean)}

    reach = RIPPLE[index] * random.uniform(0.55, 1.0)
    if random.random() < GUST_CHANCE:
        reach *= GUST

    floor, ceiling = ((-32768, 32767) if meta['differential']
                      else (0, 65535))
    low = max(floor, mean - reach * random.uniform(0.7, 1.0))
    high = min(ceiling, mean + reach * random.uniform(0.7, 1.0))
    return {'mean_raw': mean, 'min_raw': int(low), 'max_raw': int(high)}


#: Turns per 256 reads about the board's own X and Y. Whole numbers on
#: purpose: the sequence byte wraps at 256, and a rate that did not finish a
#: turn there would snap the board back to level once a cycle.
#:
#: About X and Y rather than Z, which is what this used to do. A rotation
#: about Z is the board spinning in its own plane - roll and pitch stay at
#: zero, the silhouette never changes, and a view built to show attitude
#: shows one number moving. Two unequal rates about the other two axes make
#: it tumble, so all three angles move and the drawing has depth to show.
ROLL_TURNS = 1.0
PITCH_TURNS = 2.0


def _tumble(seq, unit):
    """(i, j, k, real) counts for the stand-in's attitude, at this sequence.

    Invented, like every value in this file - see the module docstring. A
    moving one is no more a measurement than a still one; it is here so the
    views can be developed without a cable.
    """
    roll = seq * ROLL_TURNS * 2.0 * math.pi / 256.0
    pitch = seq * PITCH_TURNS * 2.0 * math.pi / 256.0
    sin_r, cos_r = math.sin(roll / 2.0), math.cos(roll / 2.0)
    sin_p, cos_p = math.sin(pitch / 2.0), math.cos(pitch / 2.0)

    # Nod about Y, then roll about X - the product of the two, in the
    # (i, j, k, real) order a rotation vector is reported in.
    return (int(sin_r * cos_p * unit), int(cos_r * sin_p * unit),
            int(-sin_r * sin_p * unit), int(cos_r * cos_p * unit))
