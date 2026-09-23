"""Who the board says it is: the identity tables behind 0x6D, and the digital pins.

The tables: units, subsystems, pins, parts.
"""
from ..errors import DeviceStateError
from ..gpio import reserved_reason
from .values import CHANNELS, SYSCLK_HZ, TICKS_PER_US


# The same shape the firmware reports over command 0x6D, so a host driven
# against the stand-in exercises the same decode.
UNITS = {'NTC': 'centi-degC', 'DC bus': 'mV',
         'Phase U': 'mA', 'Phase V': 'mA', 'Phase W': 'mA',
         # Millivolts like the DC link, through their own dividers - which is
         # why `scaling.converter` is given the signal and not just the unit.
         '+5V': 'mV', 'Vgate': 'mV', 'MCU die': 'centi-degC'}

# What the firmware answers for channels kind 3: one entry per command table.
SUBSYSTEMS = [
    {'name': 'board', 'commands': 11,
     'what': 'ADC channels, digital I/O, clocks, self test'},
    {'name': 'testrig', 'commands': 7,
     'what': 'gated raw pin access for a fixture'},
    {'name': 'imu', 'commands': 1, 'what': 'BNO08X on SPI2 over SHTP'},
]

# No PE15: it carries TIM1_BKIN, and the pin path reconfigures what it touches,
# which would take the break off the timer.
DIGITAL = [
    {'pin': 'PB2',  'direction': 'out', 'signal': 'AFE_ON'},
    {'pin': 'PE14', 'direction': 'out', 'signal': 'UART5_TERM'},
    {'pin': 'PA10', 'direction': 'out', 'signal': 'KEEPALIVE'},
]

# Not channels: the bus the command arrived on and the debug port.
RESERVED = [
    # TIM1_BKIN, first because that is where it sits in the board's own table.
    {'pin': 'PE15', 'direction': 'in',    'signal': 'nFAULT/TIM1_BKIN'},
    # The six gate signals.
    {'pin': 'PE8',  'direction': 'out',   'signal': 'TIM1_CH1N/PWMUL'},
    {'pin': 'PE9',  'direction': 'out',   'signal': 'TIM1_CH1/PWMUH'},
    {'pin': 'PE10', 'direction': 'out',   'signal': 'TIM1_CH2N/PWMVL'},
    {'pin': 'PE11', 'direction': 'out',   'signal': 'TIM1_CH2/PWMVH'},
    {'pin': 'PE12', 'direction': 'out',   'signal': 'TIM1_CH3N/PWMWL'},
    {'pin': 'PE13', 'direction': 'out',   'signal': 'TIM1_CH3/PWMWH'},
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

# What is fitted, mirroring s_parts in board/src/board_io.c.
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
    {'name': '2EDL8034 x3', 'what': 'half bridge gate drivers',
     'where': 'PE8..PE13, TIM1', 'power': 'STO chain',
     'state': 'not probed'},
    {'name': 'IAUCN10S7N021', 'what': 'bridge FETs, 63 V 100 A',
     'where': 'HalfBridge x3', 'power': 'STO chain', 'state': 'not probed'},
    {'name': 'NTC', 'what': 'thermistor',
     'where': 'ADC3', 'power': 'AFE_ON', 'state': 'ready'},
    {'name': 'DC link divider', 'what': '49.9k/2.2k, 78.15 V FS',
     'where': 'ADC', 'power': 'AFE_ON', 'state': 'ready'},
    {'name': 'USART3', 'what': 'console or Modbus RTU',
     'where': 'PB10/PB11', 'power': '', 'state': 'not probed'},
]


class SimulatedSystem:
    """The stand-in's version record and clocks."""
    def __init__(self, version_info=None):
        self._version = dict(version_info or {})

    def version(self):
        """What SimulatedBoard was built with."""
        return dict(self._version)

    def self_test_failures(self):
        """Just the checks called failures. Empty here - see self_test."""
        return [c for c in self.self_test() if c['status'] == 'fail']

    def release_console(self):
        """Nothing to hand back: there is no UART under this."""

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
        return {'sysclk_hz': SYSCLK_HZ, 'hclk_hz': SYSCLK_HZ // 2,
                'cycle_counter': 0, 'ticks_per_us': TICKS_PER_US, 'source': 'PLL1',
                # PLL2 at 75 MHz through the ADCs' DIV2 prescaler.
                'adc_hz': 37500000}


class SimulatedGpio:
    """In-memory pins, gated the same way the firmware documents the real
    ones - reads always allowed, writes only with the gate open - but
    this is a courtesy for a script that forgets the gate, not a protocol
    simulation of the rejection a real board would send back.
    """

    # PB2 is the AFE switch, not just a pin.
    AFE_PORT, AFE_PIN = 'B', 2
    # PE15 follows AFE_ON inversely - HARDWARE.md, Discrete I/O.
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
        """PB2 written by hand: move the front end with it, or the pin and the
        switch it is disagree for the rest of the session.
        """
        if self.afe is None:
            return
        self.afe.enable() if level else self.afe.disable()

    def _afe_on(self):
        return self.afe is not None and bool(self.afe.state()['on'])
    def _witnesses(self):
        """The two pins the front end's switch decides: PB2 is the switch, PE15
        follows it inversely on the assembled board.
        """
        return {(self.AFE_PORT, self.AFE_PIN): self._afe_on,
                (self.PE15_PORT, self.PE15_PIN): lambda: not self._afe_on()}

    def pin_read(self, port, pin):
        self._guard(port, pin)
        letter = str(port).upper()[:1]
        witness = self._witnesses().get((letter, pin))
        if witness is not None and self.afe is not None:
            return witness()
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
