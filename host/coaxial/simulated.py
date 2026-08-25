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
import random

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


class SimulatedLink:
    def echo(self, data):
        return data

    def stats(self):
        return {'unit_id': 1, 't15_ticks': 1750, 't35_ticks': 4083,
                'bus_message': 42, 'bus_comm_error': 0, 'server_message': 42,
                'server_exception': 0, 'server_no_response': 0,
                'char_overrun': 0}


# The same shape the firmware reports over command 0x6D, so a host driven
# against the stand-in exercises the same decode. Values invented like
# everything else here - see the module docstring.
UNITS = {'NTC': 'centi-degC', 'DC bus': 'mV'}

DIGITAL = [
    {'pin': 'PB2',  'direction': 'out', 'signal': 'AFE_ON'},
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
]


class SimulatedSystem:
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
        return {'analog': analog,
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
    def __init__(self):
        self.on = False

    def state(self):
        return {'on': self.on, 'pe15': not self.on}

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
                mean = NOMINAL[index] + random.uniform(-DRIFT[index],
                                                        DRIFT[index])
            else:
                # Invariant 9, reproduced exactly: with the reference
                # unpowered, a differential input sits at 0 and a
                # single-ended one at mid-scale - measured on real hardware,
                # not a rounder number picked to look plausible.
                mean = 0.0 if meta['differential'] else 32768.0
            chosen[index] = {'mean_raw': mean, 'min_raw': int(mean - 5),
                             'max_raw': int(mean + 5)}
        return {'samples': samples, 'rate_hz': rate or 2000.0,
                'channels': chosen}


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


class SimulatedBoard:
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
            self.analog = self.gpio = refuse
        else:
            self.system = SimulatedSystem()
            self.link = SimulatedLink()
            self.afe = SimulatedAfe()
            self.analog = SimulatedAnalog(self.afe)
            self.gpio = SimulatedGpio(self.afe)

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
