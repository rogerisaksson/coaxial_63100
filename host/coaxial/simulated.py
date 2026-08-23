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
     'differential': False, 'signal': None},
    {'index': 4, 'adc': 1, 'channel': 9, 'pin': 'PB0',
     'differential': False, 'signal': 'NTC'},
    {'index': 5, 'adc': 3, 'channel': 10, 'pin': 'PC0',
     'differential': False, 'signal': 'DC bus'},
    {'index': 6, 'adc': 3, 'channel': 11, 'pin': 'PC1',
     'differential': False, 'signal': None},
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


class SimulatedSystem:
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

    def __init__(self):
        self.gate_open = False
        self._pins = {}
        self._ports = {}

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

    def pin_read(self, port, pin):
        self._guard(port, pin)
        return self._pins.get((str(port).upper()[:1], pin), False)

    def pin_write(self, port, pin, level):
        self._guard(port, pin)
        self._require_gate()
        self._pins[(str(port).upper()[:1], pin)] = bool(level)
        return bool(level)

    def port_read(self, port):
        return self._ports.get(str(port).upper()[:1], 0)

    def port_write(self, port, mask, value):
        self._require_gate()
        letter = str(port).upper()[:1]
        current = self._ports.get(letter, 0)
        self._ports[letter] = (current & ~mask) | (value & mask)
        return self._ports[letter]


class SimulatedBoard:
    def __init__(self):
        self.version_info = {
            'proto_major': 2, 'proto_minor': 1, 'firmware': 'simulated',
            'device': 'coaxial_63100', 'mcu': 'STM32H753 (simulated)',
            'build': 'simulated', 'commands': 21,
        }
        self.system = SimulatedSystem()
        self.link = SimulatedLink()
        self.afe = SimulatedAfe()
        self.analog = SimulatedAnalog(self.afe)
        self.gpio = SimulatedGpio()

    def __repr__(self):
        return '<SimulatedBoard - no port, no cable, invented values>'

    def close_binary(self):
        pass


class SimulatedSession:
    """Drop-in for `coaxial_mcp.session.Session` that never opens a port.

    Same public shape - `.board`, `.info()`, `.close()`, `.reset()` - so
    `Toolbox` and every handler in `coaxial_mcp/tools.py` work against it
    without knowing the difference. `--simulated` on dbg.py is the only
    thing that decides which one gets built.
    """

    def __init__(self, *_args, **_kwargs):
        # Accepts and ignores whatever a real Session would take (port,
        # baud, unit) - a caller that always builds "the session" the same
        # way, real or simulated, is one fewer branch to keep in step.
        self._board = SimulatedBoard()
        self._info = None

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
