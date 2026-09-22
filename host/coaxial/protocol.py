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

THE DEVICES
-----------
0x6E carries every peripheral, chosen by a device byte, and each device's ops
are an `IntEnum` here - `ThermalOp.STATE`, `DriveOp.MODE` - so the op a
subsystem sends and the op the length oracle proves are one name. The op
tables mirror `comms/inc/cmd.h`; the subsystem behind each is named beside it.
"""
from enum import IntEnum

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
DEVICE_LINK = 2
DEVICE_CAL = 3
DEVICE_GATE_DRIVERS = 4
DEVICE_LOG = 5
DEVICE_DAQ = 6
DEVICE_TIME = 7
DEVICE_THERMAL = 8
DEVICE_POWER = 9
DEVICE_DRIVE = 10
DEVICE_BOOT = 11


class ImuOp(IntEnum):
    """Device 0, the BNO08X on SPI2 - `coaxial.imu`."""
    ID = 0
    READ = 1
    FEATURE = 2
    PROBE = 3
    RESET = 4
    WRITE = 5
    PINS = 6
    WAKE = 7
    LATEST = 8
    HOLD = 9
    RESUME = 10


class AngleOp(IntEnum):
    """Device 1, the A1335 on SPI4 - `coaxial.angle`."""
    READ = 0
    WRITE = 1
    LATEST = 2
    HOLD = 3
    RESUME = 4
    POLLREG = 5
    CLOCK = 6


class LinkOp(IntEnum):
    """Device 2, the link's own loopback and port counters - `coaxial.link`."""
    ECHO = 0
    STATS = 1


class CalOp(IntEnum):
    """Device 3, the calibration record - `coaxial.calibration`."""
    GET = 0
    SET_PARAM = 1
    SET_CHANNEL = 2
    ZERO = 3
    SPAN = 4
    SAVE = 5
    LOAD = 6
    DEFAULTS = 7
    PARAMS = 8


class GateOp(IntEnum):
    """Device 4, TIM1 and the STO chain - `coaxial.gate_drivers`."""
    STATE = 0
    PWM = 1
    DUTY = 2
    SYNC = 3
    TRIGGER = 4
    CLEAR = 5
    BYPASS = 6
    GAP_RESET = 7
    DUTY_FINE = 8
    DEADTIME = 9
    ALTERNATE = 10


class LogOp(IntEnum):
    """Device 5, the measurement ring - `coaxial.capture`."""
    STATE = 0
    ARM = 1
    TAKE = 2


class DaqOp(IntEnum):
    """Device 6, the acquisition task - `coaxial.daq`."""
    STATE = 0
    CONFIGURE = 1
    START = 2
    STOP = 3
    READ = 4
    LAYOUT = 5
    LIVE = 6
    FILTER = 7
    TONE = 8
    RUNG = 9


class TimeOp(IntEnum):
    """Device 7, the cycle counter - `coaxial.clock`."""
    LATCH = 0
    READ = 1


class ThermalOp(IntEnum):
    """Device 8, the thermal observer - `coaxial.thermal_device`."""
    STATE = 0
    SET_NODE = 1
    SET_BOARD = 2
    SET_SAMPLE = 3
    BUDGET = 4
    SET_LIMIT = 5
    SET_WINDING = 6
    NODES = 7
    EDGES = 8
    SET_EDGE = 9
    IDENT = 10
    IDENT_RESET = 11
    SET_MARGIN = 12


class PowerOp(IntEnum):
    """Device 9, the rails' reference counts - `coaxial.power`."""
    STATE = 0
    RELEASE_ALL = 1


class BootOp(IntEnum):
    """Device 11, the bootloader - `coaxial.boot`; a running application
    serves STATE and STAY and refuses the rest in words."""
    HOLD = 0
    WHO = 1
    ASSIGN = 2
    ERASE = 3
    CHUNK = 4
    MISSING = 5
    VERIFY = 6
    RECORD = 7
    SEAL = 8
    GO = 9
    STATE = 10
    DUMP = 11
    STAY = 12


class DriveOp(IntEnum):
    """Device 10, the control law - `coaxial.drive`."""
    STATE = 0
    MODE = 1
    SETPOINT = 2
    SETPOINTS = 3
    THETA = 4
    WINDOW = 5
    MOMENTS_ARM = 6
    MOMENTS = 7
    RELOAD = 8
    CYCLES_RESET = 9
    SOURCE = 10
    MODEL_PARAM = 11
    MODEL = 12
    MODEL_RESET = 13
    OBSERVERS = 14


class MapKind(IntEnum):
    """What command 0x6D is asked for: the sections of the channel map,
    and the two lists that ride beside it."""
    ANALOG = 0
    DIGITAL = 1
    RESERVED = 2
    SUBSYSTEMS = 3
    PARTS = 4


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
              'drv_sigma_i_ua', 'drv_trigger_ticks',
              # id 45, CAL_VERSION 9: the RS485 pair's baud, applied to
              # USART2 and UART5 at init. USART3 never follows it - the
              # debug probe stays the recovery path at 115200.
              'link_baud',
              # ids 46..48, CAL_VERSION 12: the winding's envelope - K/W
              # to the air and J/K in milli, and a ceiling in
              # centi-degrees that zero disables. The one node that is
              # not on the board, so the stage throttles on the motor's
              # SOA as well as the switches'.
              'winding_k_per_w_milli', 'winding_j_per_k_milli',
              'winding_limit_centi')
"""The record's scalars, in the order 0x6E device 3 op 0 sends them, and the
order their ids run in. Integers in the unit that makes them integers, because
the wire bans floating point - the names carry the unit for the same reason
the firmware's do."""

#: The MAJOR.MINOR from which the board dispatches a proven request on its
#: own CRC (MINOR 9), so a host may drop its pre-TX gap after one.
PROVEN_DISPATCH_SINCE = (2, 9)

#: The audited fixed shapes behind 0x6E, mirroring `device_length` in
#: `cmd_length.c`: (device, op) -> the whole request's length, `fc, device,
#: op` being 3. A row exists only where the handler takes nothing optional
#: past it; an op that grows an optional tail moves to GROWN_REQUESTS in the
#: same commit, and the suite's prefix sweep fails the row that fires early.
DEVICE_REQUESTS = {
    (DEVICE_CAL, CalOp.GET): 3,
    (DEVICE_CAL, CalOp.SET_PARAM): 8,                # u8 id, u32
    (DEVICE_GATE_DRIVERS, GateOp.STATE): 3,
    (DEVICE_DAQ, DaqOp.STATE): 3,
    (DEVICE_DAQ, DaqOp.START): 3,
    (DEVICE_DAQ, DaqOp.STOP): 3,
    (DEVICE_DAQ, DaqOp.LAYOUT): 3,
    (DEVICE_DAQ, DaqOp.LIVE): 3,
    (DEVICE_TIME, TimeOp.LATCH): 3,
    (DEVICE_TIME, TimeOp.READ): 3,
    (DEVICE_THERMAL, ThermalOp.STATE): 3,
    (DEVICE_THERMAL, ThermalOp.BUDGET): 3,
    (DEVICE_POWER, PowerOp.STATE): 3,
    (DEVICE_DRIVE, DriveOp.STATE): 3,
    (DEVICE_DRIVE, DriveOp.MODE): 4,                 # u8
    (DEVICE_DRIVE, DriveOp.SETPOINT): 8,             # u8 id, i32
    (DEVICE_DRIVE, DriveOp.SETPOINTS): 3,
    (DEVICE_DRIVE, DriveOp.THETA): 7,                # i32
    (DEVICE_DRIVE, DriveOp.CYCLES_RESET): 3,
    (DEVICE_DRIVE, DriveOp.MODEL): 3,
    (DEVICE_DRIVE, DriveOp.MODEL_RESET): 3,
    (DEVICE_BOOT, BootOp.STATE): 3,
    (DEVICE_BOOT, BootOp.STAY): 3,
}

#: Ops with two shapes, proven only once enough bytes rule the shorter one
#: out: (device, op) -> (bytes in hand that settle it, the long form).
#: Gate op 2 is u16 x3 or that plus a u32 period count since MINOR 8 -
#: nine bytes might be a whole short form, a tenth settles the long one.
#: DAQ op 4's `want` is optional the same way; the host always sends it.
GROWN_REQUESTS = {
    (DEVICE_GATE_DRIVERS, GateOp.DUTY): (10, 13),
    (DEVICE_DAQ, DaqOp.READ): (4, 4),
}

#: The custom commands outside 0x6E whose request length is fixed, as their
#: dispatch rows state it - whole PDU, function code counted. The ADC
#: table, the channel map and echo are variable and stay unproven.
FIXED_REQUESTS = {
    VERSION: 1, ADC_SCAN: 1, ADC_NOISE: 4, CLOCK: 1, AFE: 2, LINK_STATS: 1,
    CONSOLE: 1, TEST_GATE: 6, PIN_MODE: 5, PIN_READ: 3, PIN_WRITE: 4,
    PORT_READ: 2, PORT_WRITE: 6, ANALOG_BURST: 9, SELF_TEST: 1,
}

#: The specification's own shapes, which cannot drift with this repository:
#: the six fixed five-byte functions, and the two whose byte count sits at
#: index 5 with that many bytes behind it.
STANDARD_FIXED = frozenset((0x01, 0x02, 0x03, 0x04, 0x05, 0x06))
STANDARD_FIXED_LENGTH = 5
STANDARD_COUNTED = frozenset((0x0F, 0x10))
STANDARD_COUNT_AT = 5


def request_length(pdu, have=None):
    """Full PDU length of the request these bytes begin, or 0 when the
    bytes so far cannot prove it - the Python mirror of the firmware's
    `cmd_length.c`, which is the authority. The suite binds the two: the
    prefix sweep in test_modbus_core drives every hinted shape through
    BOTH and fails on any disagreement, so this cannot drift quietly.

    What it is for: since MINOR 9 the board dispatches a proven request
    on its own CRC instead of after t3.5 of silence, and a host that
    just sent a proven frame owes no inter-frame gap before its next -
    the previous frame cannot still be open on the board's side.
    """
    have = len(pdu) if have is None else have
    if have == 0:
        return 0
    if pdu[0] == DEVICE:
        return _device_length(pdu, have)
    if pdu[0] in FIXED_REQUESTS:
        return FIXED_REQUESTS[pdu[0]]
    return _standard_length(pdu, have)


def _device_length(pdu, have):
    """0x6E: from the audited tables, once device and op are in hand."""
    if have < 3:
        return 0
    key = (pdu[1], pdu[2])
    settled_at, length = GROWN_REQUESTS.get(
        key, (3, DEVICE_REQUESTS.get(key, 0)))
    return length if have >= settled_at else 0


def _standard_length(pdu, have):
    """The specification's fixed and counted shapes; 0 for anything else."""
    if pdu[0] in STANDARD_FIXED:
        return STANDARD_FIXED_LENGTH
    if pdu[0] in STANDARD_COUNTED and have > STANDARD_COUNT_AT:
        return STANDARD_COUNT_AT + 1 + pdu[STANDARD_COUNT_AT]
    return 0


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
