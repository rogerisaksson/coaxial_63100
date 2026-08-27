"""A board that was never plugged in, for exercising the rest of this
codebase - the REPL, the spinner, a tool call - without touching a serial
port at all.

This is not a protocol simulator. Nothing here builds a Modbus frame or
answers one; it is a duck-typed stand-in for `coaxial_mcp.session.Session`
and `coaxial.board.Board`, shaped exactly like the real ones so every tool
in `coaxial_mcp/tools.py` works against it unmodified. Every value it
returns is invented, and every touchpoint says so - `firmware` and `build`
in the version record read literally "simulated", not a plausible-looking
number, so `board_info` alone is enough to tell the two apart. This is the
same principle as invariant 9's AFE-off banner: a label a reader cannot
mistake for the real thing, not a refusal that would just push someone into
guessing instead.

Values otherwise follow the same shapes real hardware was measured to
produce this session: the seven-channel table, mid-scale codes for every
channel with the front end off, small drift on top of a nominal reading
with it on. None of it is a claim about calibration - see `coaxial.scaling`
for the one thing that already is.
"""
import contextlib
import math
import random
import time

from . import angle
from . import protocol
from .errors import DeviceStateError
from .gpio import reserved_reason

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
]

# Roughly what a live board reads with the front end on, AFE gain and all -
# not a calibrated value, just something to drift around so a repeated read
# does not look frozen. Phase channels stay near their own nominal point;
# NTC and DC bus get a slightly wider walk since those are what a question
# is usually about.
NOMINAL = {0: 900.0, 1: -8650.0, 2: -80.0, 3: 1010.0, 4: 41000.0,
          5: 21000.0, 6: 16500.0}
DRIFT = {0: 40.0, 1: 60.0, 2: 40.0, 3: 5.0, 4: 800.0, 5: 500.0, 6: 400.0}

#: One electrical revolution every seven seconds or so. Slow enough to watch
#: a meter follow it, fast enough that a still frame is rarely the same twice.
SWEEP_HZ = 0.14

#: How far each channel swings, in codes. The three phases get most of the
#: converter, because a stand-in whose meters never leave the bottom segment
#: teaches nobody what the view looks like with a machine running.
SWING = {0: 9000.0, 1: 9000.0, 2: 9000.0, 3: 300.0, 4: 6000.0, 5: 4000.0,
         6: 3000.0}

#: How far a channel moves WITHIN one burst, in codes, which is a different
#: quantity from how far it wanders between them. This was a flat +/-5 codes
#: for everything: 0.015% of a differential channel's range, so the burst's
#: own extremes drew underneath the bar and the peak hold never had anything
#: to hold. The views have marks for all three - mean, burst extreme, held
#: peak - and two of them were invisible.
#:
#: Sized by where each channel sits rather than picked to look busy: the
#: phases are inside a switching bridge and the thermistor is a slow thing on
#: the end of a divider.
RIPPLE = {0: 2600.0, 1: 2600.0, 2: 2600.0, 3: 40.0, 4: 150.0, 5: 700.0,
          6: 300.0}

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


class SimulatedLink:
    """A stand-in link. It builds no frames: the point is that a missing
    cable is not a failing suite, not that the protocol is exercised."""
    def echo(self, data):
        return data

    def stats(self):
        return self.port_stats(0)

    def loopback(self, port):
        """What a healthy board answers: all four patterns back on the two
        RS485 ports, none on the console port, and the port carrying the
        conversation refused."""
        if port not in protocol.PORTS:
            raise ValueError('port %r is not one of the three' % (port,))
        if port == 0:
            raise DeviceStateError(
                'port 0 carries this conversation; a port cannot check its '
                'own loopback while it is answering on it')

        return {
            'port': port, 'name': protocol.PORTS[port], 'rs485': True,
            'matched': 0x0F, 'returned': 4,
            'patterns': [{'sent': p, 'back': True}
                         for p in protocol.ECHO_PATTERNS],
            'ok': True,
        }

    def port_stats(self, port=0):
        if port not in protocol.PORTS:
            raise ValueError('port %r is not one of the three' % (port,))
        rs485 = port != 0
        return {'port': port, 'name': protocol.PORTS[port], 'rs485': rs485,
                'open': True, 'baud': 115200, 'unit_id': 1,
                't15_ticks': 1750, 't35_ticks': 4083,
                'bus_message': 42, 'bus_comm_error': 0, 'server_message': 42,
                'server_exception': 0, 'server_no_response': 0,
                'char_overrun': 0, 'ring_dropped': 0, 'for_others': 0}


# The same shape the firmware reports over command 0x6D, so a host driven
# against the stand-in exercises the same decode. Values invented like
# everything else here - see the module docstring.
UNITS = {'NTC': 'centi-degC', 'DC bus': 'mV',
         'Phase U': 'mA', 'Phase V': 'mA', 'Phase W': 'mA'}

# What the firmware answers for channels kind 3: one entry per command table.
# Shaped like the board's, invented like everything else here - the counts
# are what this stand-in offers, not what a part reports.
SUBSYSTEMS = [
    {'name': 'board', 'commands': 11,
     'what': 'ADC channels, digital I/O, clocks, self test'},
    {'name': 'testrig', 'commands': 7,
     'what': 'gated raw pin access for a fixture'},
    {'name': 'imu', 'commands': 1, 'what': 'BNO08X on SPI2 over SHTP'},
]

DIGITAL = [
    {'pin': 'PB2',  'direction': 'out', 'signal': 'AFE_ON'},
    {'pin': 'PE14', 'direction': 'out', 'signal': 'UART5_TERM'},
    {'pin': 'PE15', 'direction': 'in',  'signal': 'nFAULT'},
]

# Not channels: the bus the command arrived on and the debug port. Reported
# so "why was PB10 refused" has an answer, never to be driven.
RESERVED = [
    {'pin': 'PB10', 'direction': 'out',   'signal': 'USART3_TX'},
    {'pin': 'PB11', 'direction': 'in',    'signal': 'USART3_RX'},
    {'pin': 'PA13', 'direction': 'inout', 'signal': 'JTMS/SWDIO'},
    {'pin': 'PA14', 'direction': 'in',    'signal': 'JTCK/SWCLK'},
    {'pin': 'PA15', 'direction': 'in',    'signal': 'JTDI'},
    {'pin': 'PB3',  'direction': 'out',   'signal': 'JTDO/TRACESWO'},
    {'pin': 'PB4',  'direction': 'in',    'signal': 'NJTRST'},
    {'pin': 'PB12', 'direction': 'out',   'signal': 'SPI2_NSS/H_CSN'},
    {'pin': 'PB13', 'direction': 'out',   'signal': 'SPI2_SCK'},
    {'pin': 'PB14', 'direction': 'in',    'signal': 'SPI2_MISO'},
    {'pin': 'PB15', 'direction': 'out',   'signal': 'SPI2_MOSI'},
    {'pin': 'PD8',  'direction': 'in',    'signal': 'IMU H_INTN'},
    {'pin': 'PD9',  'direction': 'out',   'signal': 'IMU PS0/WAKE'},
    {'pin': 'PD10', 'direction': 'out',   'signal': 'IMU NRSTN'},
    {'pin': 'PD11', 'direction': 'out',   'signal': 'IMU BOOTN'},
    {'pin': 'PE2',  'direction': 'out',   'signal': 'SPI4_SCK'},
    {'pin': 'PE4',  'direction': 'out',   'signal': 'SPI4_NSS/A1335_CS'},
    {'pin': 'PE5',  'direction': 'in',    'signal': 'SPI4_MISO'},
    {'pin': 'PE6',  'direction': 'out',   'signal': 'SPI4_MOSI'},
]

# What is fitted, mirroring s_parts in Board/Src/board_io.c. The stand-in's
# states are what a powered board reports, because a stand-in with no supply
# to switch has nothing else to say.
PARTS = [
    {'name': 'STM32H753VIT6', 'what': 'the MCU, 475 MHz',
     'where': 'U3', 'power': '', 'state': 'not probed'},
    {'name': 'BNO085', 'what': '9-axis IMU, SHTP',
     'where': 'SPI2, U13', 'power': 'AFE_ON', 'state': 'ready'},
    {'name': 'A1335', 'what': 'magnetic angle sensor',
     'where': 'SPI4, U14', 'power': 'AFE_ON', 'state': 'ready'},
    {'name': 'AFE', 'what': 'phase chains + ADC ref',
     'where': 'PB2 switches it', 'power': '', 'state': 'ready'},
    {'name': 'UART5 termination', 'what': '120 ohm across the pair',
     'where': 'PE14 switches it', 'power': '', 'state': 'not probed'},
    {'name': 'NTC', 'what': 'thermistor',
     'where': 'ADC3', 'power': 'AFE_ON', 'state': 'ready'},
    {'name': 'DC link divider', 'what': '49.9k/2.2k, 78.15 V FS',
     'where': 'ADC', 'power': 'AFE_ON', 'state': 'ready'},
    {'name': 'USART3', 'what': 'console or Modbus RTU',
     'where': 'PB10/PB11', 'power': '', 'state': 'not probed'},
]


class SimulatedSystem:
    """The stand-in's version record and clocks. `firmware` and `build`
    read literally `simulated`, so board_info alone tells them apart."""
    def channel_map(self, refresh=False):
        analog = []
        for row in CHANNELS:
            analog.append({
                'index': row['index'], 'adc': row['adc'],
                'channel': row['channel'], 'pin': row['pin'],
                'direction': 'in',
                'differential': row['differential'],
                'signal': row['signal'] or '',
                # The board reports these; a stand-in that did not was a
                # difference in the map itself rather than in the numbers,
                # which is the one thing the two must never disagree on.
                'unit': UNITS.get(row['signal']),
            })
        return {'subsystems': SUBSYSTEMS,
                'parts': [dict(p) for p in PARTS],
                'analog': analog,
                'digital': [dict(d) for d in DIGITAL],
                'reserved': [dict(d) for d in RESERVED]}

    def self_test(self):
        return [{'name': 'PLL lock', 'status': 'pass', 'value': 1},
                {'name': 'ADC calibrated', 'status': 'pass', 'value': 1},
                {'name': 'flash checksum', 'status': 'info', 'value': 0}]

    def clock(self):
        return {'sysclk_hz': 475000000, 'hclk_hz': 237500000,
                'cycle_counter': 0, 'ticks_per_us': 475, 'source': 'PLL1'}


class SimulatedAfe:
    """The stand-in AFE, with PE15 following AFE_ON inversely - the same
    relation the real board was measured to have."""
    def __init__(self):
        self.on = False

    def state(self):
        return {'on': self.on, 'pe15': not self.on}

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

    def channels(self, refresh=False):
        return CHANNELS

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


class SimulatedGpio:
    """In-memory pins, gated the same way the firmware documents the real
    ones - reads always allowed, writes only with the gate open - but this
    is a courtesy for a script that forgets the gate, not a protocol
    simulation of the rejection a real board would send back."""

    # PB2 is the AFE switch, not just a pin. A GPIO write that clears it
    # turns the front end off on real hardware, and a simulator that kept
    # the two in separate dictionaries answered `afe_power read` with `on=1`
    # one call after GPIOB went low - measured, and the one place invariant
    # 9 could be broken by a stand-in without anyone noticing.
    AFE_PORT, AFE_PIN = 'B', 2
    # PE15 follows AFE_ON inversely - HARDWARE.md, Discrete I/O. SimulatedAfe
    # already reports it in state(); this is what makes reading the pin agree
    # with reading the switch.
    PE15_PORT, PE15_PIN = 'E', 15

    def __init__(self, afe=None):
        self.gate_open = False
        self._pins = {}
        self._ports = {}
        self.afe = afe

    def test_mode(self, enable):
        self.gate_open = bool(enable)
        return self.gate_open

    def _guard(self, port, pin):
        reason = reserved_reason(port, pin)
        if reason is not None:
            raise ValueError('P%s%d is %s and is refused in every mode; '
                             'driving it would cost the link or the debug '
                             'port' % (str(port).upper()[:1], pin, reason))

    def _require_gate(self):
        if not self.gate_open:
            raise DeviceStateError('the gate is closed; call test_gate '
                                   'first')

    def pin_mode(self, port, pin, mode, pull='none'):
        self._guard(port, pin)
        self._require_gate()

    def _drive_afe(self, level):
        """PB2 written by hand: move the front end with it, or the pin and
        the switch it is disagree for the rest of the session."""
        if self.afe is None:
            return
        self.afe.enable() if level else self.afe.disable()

    def _afe_on(self):
        return bool(self.afe.state()['on'])

    def pin_read(self, port, pin):
        self._guard(port, pin)
        letter = str(port).upper()[:1]
        if self.afe is not None:
            if (letter, pin) == (self.AFE_PORT, self.AFE_PIN):
                return self._afe_on()
            if (letter, pin) == (self.PE15_PORT, self.PE15_PIN):
                return not self._afe_on()
        return self._pins.get((letter, pin), False)

    def pin_write(self, port, pin, level):
        self._guard(port, pin)
        self._require_gate()
        letter = str(port).upper()[:1]
        self._pins[(letter, pin)] = bool(level)
        if (letter, pin) == (self.AFE_PORT, self.AFE_PIN):
            self._drive_afe(bool(level))
        return bool(level)

    def port_read(self, port):
        letter = str(port).upper()[:1]
        value = self._ports.get(letter, 0)
        if self.afe is not None and letter == self.AFE_PORT:
            bit = 1 << self.AFE_PIN
            value = value | bit if self._afe_on() else value & ~bit
        return value

    def port_write(self, port, mask, value):
        self._require_gate()
        letter = str(port).upper()[:1]
        current = self._ports.get(letter, 0)
        self._ports[letter] = (current & ~mask) | (value & mask)
        if letter == self.AFE_PORT and mask & (1 << self.AFE_PIN):
            self._drive_afe(bool(value & (1 << self.AFE_PIN)))
        return self._ports[letter]


BROADCAST_REFUSAL = ('unit 0 is the broadcast address: every node acts on a '
                     'broadcast and none answers it, so there is nothing to '
                     'read back. Send an order with broadcast(), or select '
                     'one node.')


class _BroadcastRefuses:
    """Every read on unit 0, refused the way the real board refuses it.

    The real Board has one guard, in request(), which every subsystem call
    goes through. The stand-in has no such choke point - its subsystems
    answer directly - so this stands in for all of them at once. Without
    it a broadcast read succeeded here and raised on the board, which is
    the difference test_parity.py exists to catch.
    """

    def __getattr__(self, _name):
        def refuse(*_a, **_k):
            raise DeviceStateError(BROADCAST_REFUSAL)
        return refuse


# Five buses, one per limb plus the axis. A bus is a serial segment, which
# is how a machine like this is actually wired: shorter runs, one limb's
# fault confined to one limb, and four segments that can carry traffic at
# once instead of twenty nodes taking turns on one.
#
# That makes the odd/even trick redundant - the bus says the side - so the
# unit id says the position down the limb instead. Node 2 is the knee on LL
# and on RL, which is worth more to a controller than a unique number.
#
# Two-letter labels, not emoji. spinner.py records the width problem with
# forced-colour glyphs twice over, and this is a column-aligned table, which
# is where it shows worst. AX for the axis, which is what the docs and the
# tests already call it.
SIMULATED_BUSES = {
    # label: (what it serves, {unit: (name, type, where)})
    'LL': ('left leg', {
        1: ('coaxial_63100', 'bldc_inverter', 'left hip'),
        2: ('coaxial_63100', 'bldc_inverter', 'left knee'),
        3: ('coaxial_63020', 'bldc_inverter', 'left ankle'),
        4: ('coaxial_63020', 'bldc_inverter', 'left foot'),
    }),
    'RL': ('right leg', {
        1: ('coaxial_63100', 'bldc_inverter', 'right hip'),
        2: ('coaxial_63100', 'bldc_inverter', 'right knee'),
        3: ('coaxial_63020', 'bldc_inverter', 'right ankle'),
        4: ('coaxial_63020', 'bldc_inverter', 'right foot'),
    }),
    'LA': ('left arm', {
        1: ('coaxial_63100', 'bldc_inverter', 'left shoulder'),
        2: ('coaxial_63020', 'bldc_inverter', 'left elbow'),
        3: ('coaxial_63020', 'bldc_inverter', 'left wrist'),
        4: ('coaxial_63020', 'bldc_inverter', 'left gripper'),
    }),
    'RA': ('right arm', {
        1: ('coaxial_63100', 'bldc_inverter', 'right shoulder'),
        2: ('coaxial_63020', 'bldc_inverter', 'right elbow'),
        3: ('coaxial_63020', 'bldc_inverter', 'right wrist'),
        4: ('coaxial_63020', 'bldc_inverter', 'right gripper'),
    }),
    'AX': ('axis', {
        1: ('coaxial_63100', 'bldc_inverter', 'pelvis'),
        2: ('coaxial_63100', 'bldc_inverter', 'waist'),
        3: ('coaxial_63020', 'bldc_inverter', 'neck'),
        4: ('coaxial_63020', 'bldc_inverter', 'head'),
    }),
}

DEFAULT_BUS = 'AX'


def bus_nodes(label):
    """{unit: (name, type, where)} for one bus, empty for an unknown one."""
    return SIMULATED_BUSES.get(label, ('', {}))[1]


class SimulatedImu:
    """A BNO08X that was never soldered on.

    Shaped like `coaxial.imu.Imu` so every caller works against it
    unmodified, and labelled the way the version record is: the software
    part number reads 0 and the version literally says "simulated", so a
    product id from here cannot be read as one from a part.

    The reports it invents are an accelerometer at rest - roughly one g on
    Z and nothing on X and Y - because the alternative is a number that
    looks like a measurement of something.
    """

    #: Q8 counts for 9.81 m/s^2, which is what SCALE[0x01] divides by.
    ONE_G = 2511

    #: Q14 counts for 1.0, the scale a rotation vector is reported in.
    UNIT = 16384

    def __init__(self):
        self._seq = 0
        self._enabled = {}

    def product_id(self):
        return {
            'reset_cause': 1,
            'reset_cause_name': 'power on reset',
            'sw_version': 'simulated',
            'sw_part': 0,
            'sw_build': 0,
            'sw_patch': 0,
        }

    def read(self):
        """A timebase, then whatever has been enabled - framed the way a real
        cargo on channel 3 is, Figure 5-2.

        Only what feature() turned on, because that is the one thing about a
        sensor hub a caller can get wrong: reading a report nobody asked for.
        With nothing enabled this reports the accelerometer, which is what a
        bring-up looks at first.
        """
        self._seq = (self._seq + 1) & 0xFF
        cargo = bytes([0xFB, 0, 0, 0, 0])

        wanted = set(self._enabled) or {0x01}

        if 0x01 in wanted:
            cargo += bytes([0x01, self._seq, 0x03, 0])
            for value in (0, 0, self.ONE_G):
                cargo += int(value).to_bytes(2, 'little', signed=True)

        if 0x05 in wanted:
            cargo += bytes([0x05, self._seq, 0x03, 0])
            for value in _tumble(self._seq, self.UNIT) + (0,):
                cargo += int(value).to_bytes(2, 'little', signed=True)

        from .imu import CHANNELS, decode
        return {'channel': 3, 'channel_name': CHANNELS[3],
                'cargo': cargo, 'reports': decode(cargo)}

    def feature(self, report_id, interval_us):
        if not 0 <= report_id <= 0xFF:
            raise ValueError('report id %r is not a byte' % (report_id,))
        if not 0 <= interval_us <= 0xFFFFFFFF:
            raise ValueError('interval %r does not fit 32 bits'
                             % (interval_us,))
        if interval_us:
            self._enabled[report_id] = interval_us
        else:
            self._enabled.pop(report_id, None)


def _imu_extras(cls):
    """Everything the poll loop added to `coaxial.imu.Imu`, on the stand-in.

    Bolted on here rather than woven through the class above because they
    are one thing: the board polls the part into shared memory now, and a
    stand-in has no part to poll. They exist so a view running -Simulated
    does not crash on a call the real one answers - which is exactly what
    happened, and what test_parity now checks for.
    """
    def state(self):
        self._updates = getattr(self, '_updates', 0) + 17
        got = {'loop': 'held' if getattr(self, '_held', False) else 'running',
               'error': 'none', 'updates': self._updates,
               'cargoes': self._updates, 'errors': 0}
        for report in self.read()['reports']:
            if 'quaternion' not in report:
                continue
            # The same shape the real state() builds: the counts the part
            # sent and the quaternion this host divided out of them.
            got.update({
                'report_id': report['report_id'],
                'name': report['name'],
                'accuracy': report.get('accuracy', 'unknown'),
                'counts': dict(zip(('i', 'j', 'k', 'real'), report['raw'])),
                'quaternion': report['quaternion'],
            })
            return got
        got['quaternion'] = None
        return got

    def latest(self):
        return self.state()['quaternion']

    def hold(self):
        self._held = True
        return 'held'

    def resume(self):
        self._held = False
        return 'running'

    @contextlib.contextmanager
    def configuring(self):
        self.hold()
        try:
            yield self
        finally:
            self.resume()

    def reset(self):
        return 3        # the advertisement and the two announcements

    def write(self, channel, payload):
        if not 0 <= channel <= 5:
            raise ValueError('channel %r is not one of the six' % (channel,))

    def probe(self, length=4, select=True):
        return {'kernel_hz': 190000000, 'bitrate_hz': 1484375,
                'raw': bytes(length)}

    def pins(self):
        names = {12: 'NSS/H_CSN', 13: 'SCK', 14: 'MISO', 15: 'MOSI'}
        return [{'pin': 'PB%d' % p, 'signal': names[p], 'bits': 0x0F,
                 'held': False} for p in sorted(names)]

    def wake_test(self, ms=200):
        return 0

    for fn in (state, latest, hold, resume, configuring, reset, write,
               probe, pins, wake_test):
        setattr(cls, fn.__name__, fn)
    return cls


_imu_extras(SimulatedImu)


class SimulatedAngle:
    """The A1335 without an A1335.

    Turns steadily, because a stand-in that reports one angle for ever is
    indistinguishable from a link that has stopped. The field it reports is
    what a magnet in place would give; the real board reads 2 gauss with
    none, which is a measurement and not this object's business to imitate.
    """
    def __init__(self):
        self._at = time.monotonic()
        self._updates = 0
        self._reg = 0x20
        self._held = False

    def _turn(self):
        """One turn every twelve seconds, in counts."""
        return int(((time.monotonic() - self._at) / 12.0) * 4096.0) % 4096

    def _value(self, register):
        if register == 0x20:
            return 0x5000 | self._turn()
        if register == 0x28:
            return 0xF000 | (296 * 8)          # 296 K, eighths of a kelvin
        if register == 0x2A:
            return 0xE000 | 380                # gauss, a magnet in place
        return 0x8000

    def state(self):
        self._updates += 37
        value = self._value(self._reg)
        got = {
            'loop': 'held' if self._held else 'running',
            'error': 'none', 'updates': self._updates, 'errors': 0,
            'register': self._reg,
            'register_name': angle.REGISTERS.get(self._reg,
                                                 '0x%02X' % self._reg),
            'value': value, 'crc': 0,
        }
        if self._reg == 0x20:
            got['degrees'] = angle.degrees(value)
            got['flags'] = value >> 12
        elif self._reg == 0x28:
            got['kelvin'] = angle.kelvin(value)
        return got

    def read(self, register):
        if not 0 <= register <= 0x3F:
            raise ValueError('register %r is past the six address bits'
                             % (register,))
        return {'register': register,
                'register_name': angle.REGISTERS.get(register,
                                                     '0x%02X' % register),
                'value': self._value(register), 'crc': 0}

    def write(self, register, value):
        if not 0 <= register <= 0x3F:
            raise ValueError('register %r is past the six address bits'
                             % (register,))

    def poll_register(self, register=None):
        if register is not None:
            self._reg = register
        return {'register': self._reg,
                'register_name': angle.REGISTERS.get(self._reg,
                                                     '0x%02X' % self._reg)}

    def clock(self):
        return {'kernel_hz': 118750000, 'bitrate_hz': 1855468}

    def hold(self):
        self._held = True
        return 'held'

    def resume(self):
        self._held = False
        return 'running'

    @contextlib.contextmanager
    def configuring(self):
        self.hold()
        try:
            yield self
        finally:
            self.resume()


class SimulatedBoard:
    """A whole board without a board. Duck-typed against the real one, so
    the tools above cannot tell which they are holding - except that
    every touchpoint labels itself."""
    def __init__(self, unit=1, bus=DEFAULT_BUS):
        self.unit = int(unit)
        self.bus = bus
        name, kind, where = bus_nodes(bus).get(
            self.unit,
            ('coaxial_63100', 'bldc_inverter',
             'unassigned unit %d on %s' % (self.unit, bus)))
        self.version_info = {
            'proto_major': 2, 'proto_minor': 1, 'firmware': 'simulated',
            'device': name, 'mcu': 'STM32H753 (simulated)',
            'build': 'simulated', 'commands': 21, 'type': kind,
            # Says what it is AND that it is invented, in the same line, so
            # a list of five devices cannot be read as five real ones.
            'description': 'SIMULATED three-phase BLDC inverter at the %s'
                           % where,
            'where': where,
        }
        if self.unit == 0:
            refuse = _BroadcastRefuses()   # see BROADCAST_REFUSAL
            self.system = self.link = self.afe = refuse
            self.analog = self.gpio = self.imu = refuse
            self.angle = refuse
        else:
            self.system = SimulatedSystem()
            self.link = SimulatedLink()
            self.afe = SimulatedAfe()
            self.analog = SimulatedAnalog(self.afe)
            self.gpio = SimulatedGpio(self.afe)
            self.imu = SimulatedImu()
            self.angle = SimulatedAngle()

    def __repr__(self):
        return '<SimulatedBoard - no port, no cable, invented values>'

    def close_binary(self):
        pass

    def broadcast(self, function, payload=b''):
        """Acted on by every simulated node, answered by none."""
        return None

    def request(self, *_a, **_k):
        from .errors import DeviceStateError
        if getattr(self, 'unit', 1) == 0:
            raise DeviceStateError(
                'unit 0 is the broadcast address: every node acts on a '
                'broadcast and none answers it, so there is nothing to '
                'read back.')
        raise DeviceStateError('the simulated board answers through its '
                               'subsystems, not raw requests')


# Several of this board on one bus, which is what a machine built out of
# them looks like: same firmware, same commands, different unit id and a
# different thing bolted to the shaft. Non-contiguous on purpose - a scan
# that assumes 1..n is one that stops at the first gap.
#
# The joints are invented, like every other value in this file, and each
# one says so in its own description. What is not invented is the shape:
# one unit id per device, and identity is how a host tells them apart.
class SimulatedSession:
    """Drop-in for `coaxial_mcp.session.Session` that never opens a port.

    Same public shape - `.board`, `.info()`, `.close()`, `.reset()` - so
    `Toolbox` and every handler in `coaxial_mcp/tools.py` work against it
    without knowing the difference. `--simulated` on dbg.py is the only
    thing that decides which one gets built.
    """

    # Read by anything that must not mistake this for a board - see
    # `coaxial_mcp.tools._interface`, which used to decide from the port
    # and started calling a bus label an RS485 segment.
    simulated = True

    def __init__(self, port=None, baud=115200, unit=1, bus=DEFAULT_BUS,
                 **_kwargs):
        # Takes what a real Session takes, so a caller that always builds
        # "the session" the same way is one fewer branch to keep in step.
        # A real bus is a serial segment and its label is its port, so a
        # `port` that names a bus is taken as one.
        self.baud = baud
        self.bus = port if port in SIMULATED_BUSES else bus
        self.port = self.bus
        self.unit = int(unit)
        self._board = SimulatedBoard(self.unit, self.bus)
        self._info = None

    def buses(self):
        """[(label, what it serves)] - every segment on this machine."""
        return [(label, serves)
                for label, (serves, _) in sorted(SIMULATED_BUSES.items())]

    def scan(self, units=range(1, 17), bus=None):
        """[(unit, version)] for the nodes in `units` on one bus."""
        label = bus or self.bus
        nodes = bus_nodes(label)
        return [(unit, SimulatedBoard(unit, label).version_info)
                for unit in units if unit in nodes]

    def use(self, unit, bus=None):
        """Point this session at another node, and another bus with it."""
        if bus is not None:
            self.bus = self.port = bus
        self.unit = int(unit)
        self._board = SimulatedBoard(self.unit, self.bus)
        self._info = None
        return self.unit

    def broadcast(self, function, payload=b''):
        """Acted on by every simulated node, answered by none."""
        return None

    @property
    def board(self):
        return self._board

    def info(self, refresh=False):
        if self._info is None or refresh:
            board = self._board
            self._info = (board.version_info, board.system.clock(),
                          board.analog.channels())
        return self._info

    def close(self):
        pass

    def reset(self):
        pass
