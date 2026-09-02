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
DEVICE = 0x6E
IMU = DEVICE
"""0x6E carries every peripheral device, chosen by a device byte. IMU is
the old name for it, kept because nothing else on this board reads better
for the code itself."""
"""Every IMU operation, chosen by the first payload byte. One code because it
is the last one: MODBUS reserves 65..72 and 100..110 for user-defined
functions and this board had spent all but 110. A second code is answered
ILLEGAL FUNCTION by the protocol layer, before the command table sees it."""

DEVICE_IMU = 0
DEVICE_ANGLE = 1
"""Which peripheral 0x6E's payload is addressed to. One function code for
all of them because the user-defined ranges are spent - see cmd_device.c."""

ANGLE_OP_READ = 0
ANGLE_OP_WRITE = 1
ANGLE_OP_LATEST = 2
ANGLE_OP_HOLD = 3
ANGLE_OP_RESUME = 4
ANGLE_OP_POLLREG = 5
ANGLE_OP_CLOCK = 6

DEVICE_LINK = 2
LINK_OP_ECHO = 0
LINK_OP_STATS = 1

DEVICE_CAL = 3
DEVICE_GATE_DRIVERS = 4
DEVICE_LOG = 5
DEVICE_DAQ = 6
DEVICE_TIME = 7
DEVICE_THERMAL = 8
DEVICE_POWER = 9
DEVICE_DRIVE = 10
CAL_OP_GET = 0
CAL_OP_SET_PARAM = 1
CAL_OP_SET_CHANNEL = 2
CAL_OP_ZERO = 3
CAL_OP_SPAN = 4
CAL_OP_SAVE = 5
CAL_OP_LOAD = 6
CAL_OP_DEFAULTS = 7
CAL_OP_PARAMS = 8

CAL_PARAMS = ('vref_uv', 'shunt_uohm', 'amp_gain_ppm',
              'bus_r_top_ohm', 'bus_r_bottom_ohm',
              'ntc_r25_ohm', 'ntc_beta_mk', 'ntc_rfixed_ohm', 'ntc_t25_ck',
              # ids 9..12, the two supply-sense dividers. Missing here
              # until 2026-08-28: the board sends 13 parameters and
              # this list named 9, so the reader stopped four u32
              # early and read the channel count out of the middle of
              # parameter 10. It came out 0, so every caller had an
              # empty channel list and nothing said so.
              'r5_r_top_ohm', 'r5_r_bottom_ohm',
              'vg_r_top_ohm', 'vg_r_bottom_ohm',
              # id 13, the half-bridge dead time. In the record because it is
              # the one number between the two FETs of a leg, and a compiled
              # constant means the board carries whatever the last flash held.
              'deadtime_ns',
              # id 14, the lead-lag trim in DTG counts. The gate drive is
              # asymmetric by design, so the two transitions of a leg need
              # not want the same dead time.
              'deadtime_skew',
              # ids 15..44, CAL_VERSION 8: what the drive is told. The
              # names carry the unit; coaxial.drive.PARAMS carries the
              # scale, so a commissioning writes SI.
              'motor_r_uohm', 'motor_ld_nh', 'motor_lq_nh',
              'motor_lambda_uvs', 'motor_pole_pairs',
              'drv_kp_mv_per_a', 'drv_ki_v_per_as',
              'drv_l1_milli', 'drv_l2_milli',
              'drv_inj_mv', 'drv_inj_periods', 'drv_inj_phase_mrad',
              'drv_eps_gain_ua_per_rad', 'drv_i_max_ma', 'drv_i_trip_ma',
              'drv_v_frac_ppm', 'drv_sign',
              'drv_w_lo_mrad_s', 'drv_w_hi_mrad_s', 'drv_dt_step_ma',
              'drv_dt_mv0', 'drv_dt_mv1', 'drv_dt_mv2', 'drv_dt_mv3',
              'drv_dt_mv4', 'drv_dt_mv5', 'drv_dt_mv6', 'drv_dt_mv7',
              'drv_sigma_i_ua', 'drv_trigger_ticks')
"""The record's scalars, in the order 0x6E device 3 op 0 sends them, and the
order their ids run in. Integers in the unit that makes them integers, because
the wire bans floating point - the names carry the unit for the same reason
the firmware's do."""

PORTS = {0: 'USART3', 1: 'USART2', 2: 'UART5'}
"""The board's three Modbus ports. 0 is the debug probe's VCP and shares its
wire with the ASCII console; 1 and 2 are RS485 and carry Modbus only."""

ECHO_PATTERNS = (0x00, 0xFF, 0x5A, 0xA5)
"""What the board's loopback check sends, one bit of the reply each. All four
must return on an RS485 port: RE is tied to GND, so it hears itself."""

PART_STATES = {
    0: 'not probed', 1: 'ready', 2: 'unpowered', 3: 'silent',
}
"""What the board can say about a fitted part without judging it. 'not
probed' is what nothing on the board can prove either way - invariant 10."""

IMU_OP_ID = 0
IMU_OP_READ = 1
IMU_OP_FEATURE = 2
IMU_OP_PROBE = 3
IMU_OP_RESET = 4
IMU_OP_WRITE = 5
IMU_OP_PINS = 6
IMU_OP_WAKE = 7
IMU_OP_LATEST = 8
IMU_OP_HOLD = 9
IMU_OP_RESUME = 10

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
CHANNEL_UNITS = {0: None, 1: 'mV', 2: 'centi-degC', 3: 'mA'}
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
the firmware's own table, `board/src/board_io.c`, where the board can report
it."""

CHECK_STATUS = {0: 'pass', 1: 'fail', 2: 'info'}
"""Self-test verdicts. The board returns pass or fail only where it can prove
the answer from its own registers or its own flash; anything a calibrated
instrument would have to judge comes back as info with a value, and the decision
belongs to the test executive."""
