# Protocol

What goes over the wire between a host and the board. The stack is
`comms/` (`cmd` over `proto` over `dev`) on top of `modbus/`; the host
mirror is `host/coaxial/protocol.py`, `wire.py` and `transport.py`.
CMD_PROTO is **2.9** (`comms/inc/cmd.h`); the firmware is 1.6.0
(`comms/inc/version.h`). The two are independent: a host selects its
codec on the protocol MAJOR alone, never the firmware version
(invariant 4).

## Framing

Modbus RTU per the MODBUS over Serial Line Specification V1.02. A frame
is `unit u8, function u8, data, CRC-16/MODBUS` with the CRC low byte
first. Nothing delimits a frame except silence: t3.5 of idle before and
after, never more than t1.5 between two characters inside it. Above
19 200 baud the fixed values apply, t1.5 = 750 us and t3.5 = 1.750 ms;
at or below, both derive from the character time. Both branches are in
`modbus_rtu.c`. A gap longer than t1.5 inside a frame makes it not a
frame: it is drained and discarded, never truncated and parsed.

Timing is raw `DWT->CYCCNT` ticks, never microseconds (invariant 2):
dividing moves the wrap off a power of two and the unsigned elapsed
arithmetic breaks silently across it. The counter wraps every 9.04 s
at 475 MHz.

Since MINOR 9 the receiver takes a length oracle (`cmd_length.c`,
`mb_rtu_length_fn`). A frame whose shape is proven by its own bytes is
delivered the moment its CRC checks instead of after t3.5 of silence -
1.75 ms of every such transaction, and the host may drop its own pre-TX
gap for those. The rule the table lives under: an answer other than 0
must equal the full length of every real request it can match, so only
shapes with a fixed tail are answered, and only once enough bytes have
arrived to rule the shorter form out. A failure of proof waits out t3.5
as before; a CRC miss on the oracle path is neither counted nor
consumed, the silence gate judges the same bytes. Proven today: the
standard reads and single writes (5 bytes), the multiple writes
(`6 + byte count`), and behind 0x6E: cal ops 0 and 1; gate ops 0 and 2
(op 2 only once a tenth byte rules the short form out); daq ops 0, 2,
3, 4 (once `want` has arrived), 5, 6; time 0 and 1; thermal 0 and 4;
power 0; drive 0, 1, 2, 3, 4, 9, 12, 13. The host mirror is
`coaxial.protocol.request_length`; `test_modbus_core.py` sweeps every
prefix and fails a row that fires early.

`MB_MAX_PDU` is 253 bytes. The CRC is bit-serial, no table: a 256-byte
frame is 2048 iterations of a four-instruction loop, a few microseconds
at 475 MHz against the 1.75 ms t3.5 budget.

## Ports

Three serial ports, one link at a time (`comms/src/dev_uart.c`,
`link.c`):

| dev | UART | Where | Baud |
|---|---|---|---|
| 0 | USART3, PB10/PB11 | the debug probe's virtual COM port | 115 200, fixed |
| 1 | USART2 | RS485 through a THVD1450 | `link_baud` from the calibration record |
| 2 | UART5 | RS485 through a THVD1450, termination on PE14 | `link_baud` from the calibration record |

`link_baud` is 9 600 to 921 600, default 115 200. The RS485
transceivers have RE tied to GND, so each port hears its own
transmissions. Every port receives on interrupt with a per-byte
timestamp; the UART FIFO is disabled; the receive ring is `DEV_RING` =
256 bytes.

The debug port carries either the text console or Modbus RTU, never
both. At boot it is the console: `m` switches to binary, `r` prints the
link status, `?` the key list. The way back is command 0x48 CONSOLE or
holding register 0x0001 = 1. No printf may run while the binary link is
open (invariant 5): a blocking transmit inside a frame corrupts framing
and latches a UART overrun, which on this silicon kills reception
permanently.

Command traffic is budgeted at `CMD_LINK_SHARE_PCT` = 75 % of the
link's time; the board's own polling has the rest. A host silent for
10 s loses its rail claims and its armed stage (the firmware deadman
and `Board_PwmSessionDrop`); the broker answers for an attached client
every 3 s.

## Wire types

`comms/inc/wire.h` and `host/coaxial/wire.py`. Every multi-byte integer
is big-endian. No floats: a physical quantity goes as an integer with a
declared scale - mV, mA, uV, milli-codes, urad, mrad/s, ppm, Q16.16,
Q28. `str` is `u8 length` followed by that many ASCII bytes. Readers
and writers are total: a short read is `CMD_ERR_LENGTH`, a full writer
is `CMD_ERR_DEVICE`, and neither produces a truncated frame.

Every op that takes parameters answers **`u8 took`** first: `1` when
the request was acted on, otherwise `0` followed by a `str` carrying
the board's own reason - what is wrong and what to do. The board is the
only thing that knows which check failed, so it is the only thing that
says (`cmd_took` in `cmd.c`). The host validates only what stops a
request being formed.

A malformed request is a Modbus exception, `function | 0x80` and one
code byte:

| Firmware status | Exception |
|---|---|
| `CMD_ERR_UNKNOWN`, no such command | 01 ILLEGAL FUNCTION |
| `CMD_ERR_LENGTH`, wrong payload length | 03 ILLEGAL DATA VALUE |
| `CMD_ERR_VALUE`, a field out of range | 03 ILLEGAL DATA VALUE |
| `CMD_ERR_DEVICE`, the board could not comply | 04 SERVER DEVICE FAILURE |

The host reads a `took` reply to its last byte (`reply_shape` in
`transport.py`) and an exception frame as exactly five bytes; every
other reply is read until the line falls quiet. Stopping a general
reply on a valid CRC was measured and rejected: a prefix passes about
once in 4096.

Unit id 0 is broadcast: every node acts, none answers, and reads are
refused. Op 0 of device 7 (TIME) is meant for it.

## Standard Modbus map

`modbus/src/modbus_map.c`, function codes 01 to 06 and 16.

Input registers (04):

| Address | Contents |
|---|---|
| 0x0000 .. | raw ADC code per table row, in table order |
| 0x0010 | DC bus, mV |
| 0x0011 | NTC, centi-degrees C |
| 0x0020 / 0x0021 | SYSCLK Hz, high / low |
| 0x0022 / 0x0023 | HCLK Hz, high / low |
| 0x0030 .. 0x003B | the six RTU counters, u32 each |

Holding registers (03 / 06 / 16): 0x0000 the unit id, 0x0001 a command
word - 1 hands the port back to the console, 2 clears the counters.
Coil 0 (01 / 05) is AFE_ON. Discrete input 0 (02) is PE15.

## User function codes

0x41 .. 0x48 in `cmd_board.c`, 0x64 .. 0x6A in `cmd_test.c`, 0x6B ..
0x6D in `cmd_board.c`, 0x6E in `cmd_device.c`. A command with a fixed
request length is refused with 03 on any other length;
`CMD_LEN_VARIABLE` commands parse their own.

### 0x41 VERSION

Request: empty. Reply, append-only (invariant 3):

    u8 proto_major, u8 proto_minor,
    u8 fw_major, u8 fw_minor, u8 fw_patch,
    str device, str mcu, str build,
    u16 command_count,
    str description,          the device's one line, under 170 chars
    str type                  "bldc_inverter"

Appending a field is a MINOR. Moving, resizing or repurposing one is a
MAJOR whether it was meant or not.

### 0x42 ADC_TABLE

Request: empty, or `u8 first`. Reply: `u8 n`, then `n` rows of
`u8 adc_index, u8 channel, str pin, u8 differential, str signal,
i32 raw, i32 uV, u8 unit, i32 scaled`, then `u8 total` appended. A row
is 18 bytes plus its two names against the 252 the PDU leaves: seven
channels came to 197 bytes and nine to 254, which is why it pages.
`scaled` is in the unit named, from the calibration record
(invariant 7).

### 0x43 ADC_SCAN

Request: empty. Reply: `i32 u, i32 v, i32 w` raw, `i32 dc_raw,
i32 dc_mv, i32 ntc_raw, i32 ntc_centi_c, u8 afe_on, u8 pe15`, then
`u8 afe_users` appended. The NTC field is 0 when the AFE is off: with
the reference unpowered mid-scale puts the divider at R25 by
definition, and the reply would say exactly 25.00 C every time.

### 0x44 ADC_NOISE

Request: `u8 adc` (1 .. 3), `u16 samples` (1 .. 1000). Reply:
`u16 samples, i32 mean_uv, i32 min_raw, i32 max_raw, u32 span,
u32 sd_uv`.

### 0x45 CLOCK

Request: empty. Reply: `u32 sysclk_hz, u32 hclk_hz, u32 cycles,
u32 ticks_per_us, u8 sysclk_source`, then `u32 adc_hz` appended - what
the converters run at after the prescaler, so a sampling time in ADC
cycles converts to seconds.

### 0x46 AFE

Request: `u8 op` - 0 read, 1 off, 2 on, 3 toggle. Reply:
`u8 afe_on, u8 pe15`, then `u8 users` appended. On and off go through
the reference count in `board_power.c`, never the pin: `on` after an
explicit off means somebody else still holds the rail, and `users`
says who.

### 0x47 LINK_STATS

Request: empty. Reply for the current port: `u8 unit_id,
u32 t15_ticks, u32 t35_ticks, u32 bus_message, u32 bus_comm_error,
u32 server_message, u32 server_exception, u32 server_no_response,
u32 char_overrun`.

### 0x48 CONSOLE

Request: empty. Reply: empty; the board then hands the UART back to the
console and starts printing ASCII. The host reads this reply to its
known length rather than to a quiet gap, or it swallows the console's
first text.

### 0x64 TEST_GATE

Request: `u32 key, u8 open`; the key is 0x54455354, "TEST". Reply:
`u8 open` as it now stands. A wrong key leaves the gate as it was. The
pin and port commands below need the gate open.

### 0x65 ECHO

Request: up to 250 bytes. Reply: the same bytes.

### 0x66 .. 0x6A pins and ports

Addressing is `u8 port` as the ASCII letter and `u8 pin`.

| Code | Request | Reply |
|---|---|---|
| 0x66 PIN_MODE | `port, pin, u8 mode, u8 pull` | empty |
| 0x67 PIN_READ | `port, pin` | `u8 level` |
| 0x68 PIN_WRITE | `port, pin, u8 level` | `u8 level` read back |
| 0x69 PORT_READ | `port` | `u16 idr` |
| 0x6A PORT_WRITE | `port, u16 mask, u16 value` | `u16 idr` read back |

PIN_WRITE reads the pin back rather than echoing the request: on an
open-drain output or a pin held by the fixture the two differ, and that
difference is what a rig looks for. The reserved pins - the link's own
UART, JTAG/SWD, the gate lines - are refused with 03; `0x6D` kind 2
lists them with the reason.

### 0x6B ANALOG_BURST

Request: `u16 mask, u16 samples, u32 interval_us`; the mask non-zero,
samples 1 to 10 000, and `samples x interval` at most 5 s. Reply:
`u16 samples, u32 elapsed_us, u8 count`, then per channel
`u8 index, i32 mean_milliraw, i32 min_raw, i32 max_raw,
u32 sd_milliraw`. A conversion that times out mid-burst is 04, not 03:
the arguments were fine.

### 0x6C SELF_TEST

Request: empty. Reply: `u8 count`, then per check `str name,
u8 status, i32 value`. Each pass/fail is judged from the board's own
registers or flash and nothing else (invariant 10): `hse_rdy,
pll1_lock, clk_crystal, clk_agrees, cyccnt_runs, vref_ext, adc_pcsel`;
the rest are information - `cal_d1 .. cal_d3, image_len, image_crc,
sysclk_hz, hclk_hz, afe_on`.

### 0x6D CHANNELS

Request: `u8 kind [, u8 first]`. The board's own map of itself.

| kind | Reply |
|---|---|
| 0 analog | `u8 n`, rows `u8 index, u8 adc_index, u8 channel, str pin, u8 dir, u8 differential, str signal, u8 unit` |
| 1 digital, drivable | `u8 matching, u8 first, u8 sent`, rows `str pin, u8 dir, str signal` |
| 2 reserved | the same shape as kind 1, for the pins a fixture may not drive |
| 3 subsystems | `u8 groups`, rows `str name, str what, u8 commands` - one per command table |
| 4 parts | `u8 total, u8 first, u8 sent`, rows `str name, str what, str where, str power, u8 state` |

Kinds 1, 2 and 4 page from `first`: 19 reserved rows came to 418 bytes
against MB_MAX_PDU's 253. Part `state` is 0 unknown, 1 ready,
2 unpowered, 3 silent, measured by a probe case per part. Adding
hardware is one row in `s_parts` in `board/src/board_io.c`, its pins in
`s_digital`, and a probe case.

### 0x6E DEVICE

Request: `u8 device, u8 op, parameters`. Devices 0 .. 10; an unknown
device is 03. Every op's exact layout is in the `cmd_*.c` file named
below; this is what they carry.

## Devices

### 0 IMU, `cmd_imu.c`

BNO085 on SPI2. Ops:

| op | Request | Reply |
|---|---|---|
| 0 id | - | `u8 reset_cause, u8 sw_major, u8 sw_minor, u32 sw_part, u32 sw_build, u16 sw_patch` |
| 1 read | - | `u8 channel, u8 len`, the SHTP cargo as it arrived; len 0 is nothing waiting |
| 2 feature | `u8 report_id, u32 interval_us` | `u8 took`; 0 disables the report |
| 3 probe | - | `u32 kernel_hz, u32 bitrate, u8 len` and what the bus answered |
| 4 reset | - | `u8 drained` |
| 5 write | `u8 channel, bytes` | raw SHTP |
| 6 pins | - | per control pin `u8 pin, u8 check` |
| 7 wake | `[u16 ms]` | `u16` the wake test's answer |
| 8 latest | - | below |
| 9 hold | - | `u8 loop` |
| 10 resume | - | `u8 loop` |

Op 8 touches no SPI: `u8 loop, u8 error, u32 updates, u32 cargoes,
u32 errors, u8 have`, and when `have`: `u8 report_id, u8 status,
u16 i, u16 j, u16 k, u16 real` (the quaternion in the part's Q14
counts); appended: `u8 asked_id, u32 asked_us, u8 asked_pending`,
`u8 last_fault, u8 last_fault_id`, and since MINOR 6 the three vectors,
each `u8 have, u8 status, u16 x, u16 y, u16 z` - accelerometer Q8,
gyroscope Q9, magnetometer Q4. `updates` is monotonic, so the same
reading read twice is telling. The bus ops need the poll loop held (op
9); op 10 goes back to RUN when the part is still up and through INIT
when the hold spanned a reset. The part is powered by AFE_ON, and
`Board_ImuInit` refuses while PB2 is low.

### 1 ANGLE, `cmd_angle.c`

A1335 on SPI4. Ops: 0 read (`u8 reg` → `u8 reg, u16 value, u8 crc`),
1 write (`u8 reg, u8 value`), 2 latest (`u8 loop, u8 error,
u32 updates, u32 errors, u8 have, u8 reg, u16 value, u8 crc`), 3 hold
and 4 resume (→ `u8 loop`), 5 pollreg (`[u8 reg]` → `u8 reg` the loop
reads), 6 clock (→ `u32 kernel_hz, u32 bitrate`). Six address bits, so
a register above 0x3F is 03. Registers: ANG 0x20, STA 0x22, ERR 0x24,
XERR 0x26, TSEN 0x28, FIELD 0x2A. A packet is 20 bits; the CRC is
reported, not checked - the datasheet in this tree gives the field's
width and not its polynomial. Read and write are refused while the poll
loop runs. Also behind AFE_ON.

### 2 LINK, `cmd_link.c`

Op 0 echo: `u8 port` → `u8 port, u8 rs485, u8 matched, u8 seen,
str name`; `matched` is one bit per pattern of 00 FF 5A A5, 0x0F is
all four. Refused for the port carrying the request, whose own
patterns would land in front of the reply. Op 1 stats: `u8 port` →
`u8 port, u8 unit_id, u8 rs485, u8 open, u32 baud, u32 t15_ticks,
u32 t35_ticks`, the six counters, `u32 dropped, str name`.
`bus_message` counts every frame on the segment and `server_message`
only the ones addressed to this unit.

### 3 CAL, `cmd_cal.c`

The calibration record, CAL_VERSION 9, in flash bank 2 sector 7 at
0x081E0000, magic 'CX63', CRC-16/MODBUS over everything ahead of the
CRC field. Ops:

| op | Request | Reply |
|---|---|---|
| 0 get | - | `u8 stored, u16 version, u8 15`, params 0 .. 14 as u32, `u8 10`, per channel `i32 offset, i32 gain_ppm`, `u8 10`, per node `i32 soa_limit_centi`, `u32 soa_throttle_ppm` |
| 1 set_param | `u8 id, u32 value` | empty; 03 on a bad id |
| 2 set_channel | `u8 index, i32 offset, i32 gain_ppm` | empty |
| 3 zero | `u8 index` | `i32 measured` - what became the offset |
| 4 span | `u8 index, i32 reference` | `i32 measured`; phase (mA) and DC bus (mV) only |
| 5 save | - | `u8 1` |
| 6 load | - | `u8 1` |
| 7 defaults | - | `u8 1`; RAM only until saved |
| 8 params | `[u8 first]` | `u8 46, u8 first, u8 count`, up to 60 u32 |

`stored` separates a calibrated board from one running the schematic's
numbers; the two are otherwise identical on the wire. A stored record
of another version is refused and the defaults used. The 46 parameter
ids and their defaults are in HARDWARE.md, "Calibration record".

### 4 GATE_DRIVERS, `cmd_gate_drivers.c`

| op | Request | Reply |
|---|---|---|
| 0 state | - | below |
| 1 pwm | `u8 on` | `u8 took`; the only thing that sets MOE, always at zero duty |
| 2 duty | `u16 x3 ticks [, u32 periods]` | `u8 took`; the count since MINOR 8 |
| 3 sync | `u8 on` | `u8 took`; TIM1 triggers the injected ADC group |
| 4 trigger | `u16 ccr` | `u16` as it reads back |
| 5 clear | - | `u8`; clears the break flag, does not re-arm |
| 6 bypass | `u8 on` | `u8`; drops BDTR.BKE |
| 7 gap reset | - | `u8`; forgets the worst keepalive gap |
| 8 dutyq | `u32 x3 Q16.16 ticks` | `u8 took`; sigma-delta dither |
| 9 deadtime | `u32 ns, i8 skew` | `u8 took`, then `u32 ns, u8 skew, u8 floor` as applied |
| 10 alternate | `u16 x3 A, u16 x3 B` | `u8 took`; A one period, B the next |

Op 0: `u8 flags` (0x01 ready, 0x02 enabled, 0x04 fault, 0x08 sync
ready, 0x10 sync armed, 0x20 afe_on, 0x40 pilot ok, 0x80 level ok),
`u16 period, u8 dtg, u16 duty x3, u16 trigger, i16 phase x3, u16 at,
u32 updates, u32 overruns, u32 keepalive, u32 worst_gap, i32 pilot_raw,
i32 pilot_uv, i32 level_raw, i32 level_uv`; appended in this order:
`u8 bypassed`, `u32 requested x3` (Q16.16), `u8 pins, u16 pins_at` (the
six gate lines in one instant), `u32 deadtime_ns, u8 skew, u8 floor`,
`u8 gate_shorts` (bit 0 U, 1 V, 2 W; 0 while armed), `u32 dcbus_raw,
u32 ntc_raw` (MINOR 2), `u32 periods_left` (MINOR 8).

A duty write before op 1 is refused. Op 2 with a count: the update ISR
zeroes the compares when the count reaches zero - 500 periods at
50 kHz is 10.000 ms exactly. Op 10 swaps the two triples every
overflow.

### 5 LOG, `cmd_log.c`

A ring of 1024 records in DTCM. Op 0 state: `u8 sources, u16 count,
u16 depth, u32 dropped`, then `u32 thinned` appended - dropped is a
sample the ring had no room for, thinned one it declined because the
link could not carry it. Op 1 arm: `u8 source_mask` → `u8 1`, empties
the ring; each armed source gets an equal share of what the link can
drain. Op 2 take: `[u8 want]` → `u8 got` then `got` records of 14
bytes: `u32 at, u8 source, u8 seq, i16 x4`. Fifteen fit a reply.
Sources: 0 phases, 1 angle, 2 imu, 3 drive.

### 6 DAQ, `cmd_daq.c`

| op | Request | Reply |
|---|---|---|
| 0 state | - | below |
| 1 configure | below | `u8 took`; refused while running |
| 2 start | - | `u8 took` |
| 3 stop | - | `u8 took` |
| 4 read | `[u8 want]` | `u8 got`, `got` records, `u32 backlog` (MINOR 5) |
| 5 layout | - | below |
| 6 live | - | `u8 fresh`, and when fresh `u32 first, u32 last`, per field `i32 sum, u32 additions, i32 lowest, i32 highest`, `u32 digital` if pins are sampled |
| 7 filter | `u8 count, u16 decimate, i32 x5 x count` Q28 | `u8 took` |
| 8 tone | `u32 hz, u32 rate, i32 amplitude, i32 offset [, u8 kind]` | `u8 took`; kind 0 sine, 1 ramp |
| 9 rung | `u8 rung, u16 boxcar, u8 count, u16 decimate, i32 x5 x count` | `u8 took` |

Op 0: `u8 flags` (0x01 running, 0x02 done, 0x04 lost power),
`u16 stride, u8 fields, u32 available, u32 produced, u32 dropped,
u16 channels, u8 clock, u8 sample_time, u16 decimate, u16 accumulate,
u32 records, u8 digital, u32 interval_us, u32 records_per_second`;
appended: `u32 capacity, u32 worst` (MINOR 4), `u8 rung, u8 rungs,
u32 rung_changes, u32 triggers`, `u16 sensors, u16 selectable`
(MINOR 7).

Configure: `u16 channels` (mask over the ADC table), `u8 clock`
(0 software, 1 TIM1), `u8 sample_time, u16 decimate, u16 accumulate,
u32 records`, then optional `u8 digital` (pin mask), `u32 interval_us`,
`u8 adapt`, `u16 sensors` (MINOR 7). Accumulate 0 closes a record on
the clock instead of on a count (MINOR 3). The TIM1 clock carries only
the phases, the DC bus and the NTC; the sensor fields ride the software
clock only - a TIM1 record closes in ADC3's ISR and would read the poll
records torn.

Op 5: `u8 fields, u16 stride`, per field `u8 index, u8 unit,
u8 differential, str signal` in the channel table's order, `u8 digital`
and when set `u8 pins` then per pin `u8 dir, str signal`; appended
(MINOR 7) `u8 count` then per sensor `u8 bit, u8 4, str name` -
orientation, acceleration, rotation rate, magnetic field, shaft angle.

A record: `u32 stamp`, 4 bytes per analog field (the SUM over the
accumulated samples), 1 byte of duty per sampled digital pin, four
i16 SNAPSHOT words per sensor, and `u16 count` last. The stride is
what op 5 says; a decoder that recomputes it mis-frames.
`DAQ_REPLY_ROOM` is 240 bytes, so `got` is however many whole records
fit; the worst case `1 + 240 + 4` is 245 against the 252 the PDU
leaves. The ring behind it is 448 KB in AXI SRAM.

### 7 TIME, `cmd_time.c`

Op 0 latch: every node captures its CYCCNT at the frame; meant for
broadcast, so no turnaround sits inside the measurement. Op 1 read:
`u32 seq, u32 latched, u32 now, u32 sysclk_hz`. The host side is
`coaxial.clock` and `set_time_from_pc()`.

### 8 THERMAL, `cmd_thermal.c`

Op 0 state: `u8 ntc_measured, i32 ntc_centi, u8 10`, per node
`i32 centi`, `i32 ambient_centi, i32 expected_ntc_centi, u32 seconds,
u8 settled`; appended `u32 every_ms, u32 settle_ms`, `u8 afe_measured,
i32 afe_centi, u8 mcu_measured, i32 mcu_centi, u32 seen_ms_ago`,
`u32 steps`. Op 1 set node: `u8 node, i32 to_board_milli,
i32 capacity_milli` → `u8 took`. Op 2 set board: `i32 to_ambient_milli,
i32 capacity_milli` → `u8 took`. Op 3 set sample: `u32 every_ms,
u32 settle_ms` → `u8 took`. Op 4 budget: `u8 10`, per node `u8 used`
(0 at ambient, 255 at the limit), `u8 worst, u8 worst_node,
i32 millis_to_limit, u8 throttling, u8 tripped, u32 trips`. Op 5 set
limit: `u8 node, i32 limit_milli_c, i32 throttle_ppm` → `u8 took`.
Nodes 0 .. 9: driver U/V/W, phase U/V/W, mcu, regulators, afe, board.
MAJOR 2 (2026-08-29) gave each leg its own node and repurposed the
indices.

### 9 POWER, `cmd_power.c`

Op 0 state: `u8 rails`, per rail `u8 on, u8 users, u8 count,
u8 blocked, u8 leased`; the user bits are host, thermal, imu, angle,
daq. The host's claim is unleased; the others hold 3 s leases. Op 1
release: every claim dropped → `u8 took`. Releasing switches the AFE
rail off, which gives the drivers their supply rather than taking it
away - the direction that is safe while armed.

### 10 DRIVE, `cmd_drive.c`

Angles in urad, speeds in mrad/s, currents mA, volts mV; the window's
means and deviations in micro-units.

| op | Request | Reply |
|---|---|---|
| 0 state | - | below |
| 1 mode | `u8` 0 off, 1 volt, 2 hold, 3 sensorless, 4 polarity | `u8 took` |
| 2 setpoint | `u8 id, i32 value` | `u8 took` |
| 3 setpoints | - | `u8 10, i32 x10` |
| 4 theta | `i32 urad` | `u8 took`; both frames |
| 5 window | - | `u32 n`, 7 fields of `u32 n, i32 mean, u32 sd`, `u8 7`, 7 lags of `i32 rho_ppm`, `i32 i_peak_ma`; reset on read |
| 6 moments arm | `u32 periods` | `u8 took`; needs the sync armed |
| 7 moments | - | `u8 done, u32 n, u32 want, u16 trigger`, 4 channels of `i32 mean_milli, u32 sd_milli, i32 lo, i32 hi` |
| 8 reload | - | `u8 took`; parameters out of the record |
| 9 cycles reset | - | `u8` |
| 10 source | `u8` 0 converters, 1 the model | `u8 took` |
| 11 model param | `u8 id, i32 value` | `u8 took`; 16 ids |
| 12 model | - | `u8 source, i32 theta, i32 omega, i32 id, i32 iq, i32 vdc, i32 theta_hat, i32 omega_hat` |
| 13 model reset | - | `u8` |

Op 0: `u8 mode, u8 fault, u8 flags` (0x01 MOE, 0x02 afe_on,
0x04 injection valid, 0x08 drive owns the compares, 0x10 sync armed),
`i32 theta_hat, i32 omega_hat, i32 theta_cmd, i32 omega_cmd, i32 id,
i32 iq, i32 vd, i32 vq, i32 vdc, i32 eps, i32 eps_amps, i32 ih,
i32 e_bemf, u32 periods, u32 cycles_last, u32 cycles_max, i32 pol_pos,
i32 pol_neg, u16 trigger, u32 ts_ns, u16 exit_ticks, u32 cyc_sample,
u32 cyc_step, u32 cyc_advance`. The window's seven fields are id, iq,
vd, vq, eps, ih, vdc; the moments' four channels are U, V, W and the
DC bus in milli-codes.

Setpoint ids: 0 id_ref mA, 1 iq_ref mA, 2 theta mrad, 3 omega_target
mrad/s, 4 accel mrad/s², 5 vd mV, 6 vq mV, 7 pol_volts, 8 pol_periods,
9 pol_gap. The host names and scales are `coaxial.drive.SETPOINTS` and
`PARAMS`.

## Versioning

MAJOR breaks a codec; MINOR appends. The MINOR history, from `cmd.h`:

| MINOR | Change |
|---|---|
| 1 | gate op 10 alternate |
| 2 | device 10 DRIVE; the DC link appended to gate op 0 |
| 3 | a DAQ record ends with `u16 count`; accumulate 0 closes on the clock - resizes the record, op 5 says the stride |
| 4 | DAQ op 0 appends the buffer level, capacity and high-water mark |
| 5 | DAQ op 4 appends the backlog |
| 6 | IMU op 8 appends the three vectors, each with its own `have` |
| 7 | DAQ op 1 appends a sensor mask; records append four i16 per sensor after the pins; op 5 appends the rows |
| 8 | gate op 2 takes an optional period count; op 0 appends `periods_left` |
| 9 | fixed-shape requests dispatch on their own CRC, not t3.5 |

MAJOR 2, 2026-08-29: the thermal nodes went per leg and the node
indices were repurposed - a host could follow the length and not the
meaning.

A host reads 0x41 first, picks its codec on `proto_major`, and treats
any field past what it knows as opaque. The stand-in
(`coaxial.simulated`) reports proto 2.8 and firmware "simulated";
`test_parity.py` holds its replies to the live board's, and
`test_conformance.py` holds the live board to this document.
