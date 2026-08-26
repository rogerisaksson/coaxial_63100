"""Command codes and the versioning contract.

The codes live in the two ranges the Modbus specification reserves for
user-defined functions, 65..72 and 100..110. Nothing here is invented namespace.

VERSIONING
----------
Command VERSION is the frozen one. Its payload begins with the protocol major
and minor, so a host of any vintage can read two bytes, decide whether it
understands the device, and stop. Fields may only ever be APPENDED after that:
an old host decodes the prefix it knows and ignores the rest. Reordering or
resizing an existing field creates a new MAJOR whether or not that was intended.

A host selects its codec on the protocol MAJOR alone. The firmware version is
for the test record - binding a host to firmware numbers means every rebuild of
the firmware breaks the host.
"""

# Application commands, Modbus user range 65..72.
VERSION = 0x41
ADC_TABLE = 0x42
ADC_SCAN = 0x43
ADC_NOISE = 0x44
CLOCK = 0x45
AFE = 0x46
LINK_STATS = 0x47
CONSOLE = 0x48

# Test fixture commands, Modbus user range 100..110.
TEST_GATE = 0x64
ECHO = 0x65
PIN_MODE = 0x66
PIN_READ = 0x67
PIN_WRITE = 0x68
PORT_READ = 0x69
PORT_WRITE = 0x6A
ANALOG_BURST = 0x6B
SELF_TEST = 0x6C
CHANNELS = 0x6D
IMU = 0x6E
"""Every IMU operation, chosen by the first payload byte. One code because it
is the last one: MODBUS reserves 65..72 and 100..110 for user-defined
functions and this board had spent all but 110. A second code is answered
ILLEGAL FUNCTION by the protocol layer, before the command table sees it."""

IMU_OP_ID = 0
IMU_OP_READ = 1
IMU_OP_FEATURE = 2
IMU_OP_PROBE = 3
IMU_OP_RESET = 4

NAMES = {
    VERSION: 'version', ADC_TABLE: 'adc_table', ADC_SCAN: 'adc_scan',
    ADC_NOISE: 'adc_noise', CLOCK: 'clock', AFE: 'afe',
    LINK_STATS: 'link_stats', CONSOLE: 'console',
    TEST_GATE: 'test_gate', ECHO: 'echo', PIN_MODE: 'pin_mode',
    PIN_READ: 'pin_read', PIN_WRITE: 'pin_write', PORT_READ: 'port_read',
    PORT_WRITE: 'port_write', ANALOG_BURST: 'analog_burst',
    SELF_TEST: 'self_test', CHANNELS: 'channels',
    IMU: 'imu',
}

BROADCAST = 0
"""Unit address every slave acts on and none answers."""

TEST_GATE_KEY = 0x54455354
"""ASCII "TEST". Required to open raw pin access, so the mode cannot be entered
by a stray frame or a mistyped command."""

BURST_MAX_MICROSECONDS = 5_000_000
"""The firmware refuses a longer burst rather than leaving the link silent past
the master's patience. Mirrored here so the host can say so without a round
trip."""

MAX_PAYLOAD = 250
"""Room in one RTU frame once the unit id, function code and CRC are counted."""

# Physical wiring facts the firmware reports, decoded to something readable.
CLOCK_SOURCES = {0: 'HSI', 1: 'CSI', 2: 'HSE', 3: 'PLL1', 4: 'other'}
CHANNEL_UNITS = {0: None, 1: 'mV', 2: 'centi-degC'}
PIN_MODES = {'input': 0, 'output': 1, 'output_pp': 1, 'output_od': 2, 'analog': 3}
PIN_PULLS = {'none': 0, 'up': 1, 'down': 2}
AFE_ACTIONS = {'read': 0, 'off': 1, 'on': 2, 'toggle': 3}

DIRECTIONS = {0: 'in', 1: 'out', 2: 'inout'}
"""Which way a channel's signal runs, from the MCU's side. Command 0x6D."""

RESERVED_PINS = {
    ('B', 10): 'USART3_TX',
    ('B', 11): 'USART3_RX',
    ('A', 13): 'JTMS/SWDIO',
    ('A', 14): 'JTCK/SWCLK',
    ('A', 15): 'JTDI',
    ('B', 3): 'JTDO/TRACESWO',
    ('B', 4): 'NJTRST',
}
"""Pins the firmware refuses in every mode - the **fallback** only.

The board carries this map itself now (command 0x6D, `system.channel_map()`),
and `Gpio._guard` asks it. This copy is what a board older than protocol 1.3
gets answered from, and what explains WHY a request will fail instead of
relaying an exception code: driving the first two severs the link the command
arrived on, and the rest cost the ability to reflash.

A second copy of a hardware fact is one edit from disagreeing with the first.
This one is kept deliberately and is not to be extended - a new pin belongs in
the firmware's own table, `Board/Src/board_io.c`, where the board can report
it."""

CHECK_STATUS = {0: 'pass', 1: 'fail', 2: 'info'}
"""Self-test verdicts. The board returns pass or fail only where it can prove
the answer from its own registers or its own flash; anything a calibrated
instrument would have to judge comes back as info with a value, and the decision
belongs to the test executive."""
