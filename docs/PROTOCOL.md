# Protocol

Host mirror: `host/coaxial/protocol.py`, `wire.py`, `transport.py`. Version
in `comms/inc/cmd.h` (`CMD_PROTO_MAJOR`/`MINOR`, 2.18); firmware version in
`version.h`. A host picks its codec on MAJOR only (invariant 4).

## Framing

- Modbus RTU (V1.02): `unit, function, data, CRC-16/MODBUS` (CRC low first).
  Above 19 200 baud t1.5 = 750 us, t3.5 = 1.75 ms; a gap > t1.5 inside a
  frame discards it. Timing in raw CYCCNT ticks (invariant 2).
- Length oracle (`cmd_length.c`, MINOR 9): a request whose shape its own
  bytes prove is dispatched on its CRC, not after t3.5. Host mirror
  `protocol.request_length`; `test_modbus_core.py` sweeps every prefix.
- `MB_MAX_PDU` 253 bytes. Unit 0 = broadcast: all act, none answer.

## Ports

| dev | UART | Where | Baud |
| --- | --- | --- | --- |
| 0 | USART3, PB10/PB11 | the debug probe's virtual COM port | 115 200, fixed |
| 1 | USART2 | RS485 through a THVD1450 | `link_baud` from the calibration record |
| 2 | UART5 | RS485 through a THVD1450, termination on PE14 | `link_baud` from the calibration record |

- `link_baud` 9 600-921 600, default 115 200. RS485 RE tied low: each port
  hears itself.
- Debug port: console at boot (`m` binary, `r` link status, `?` keys);
  back via 0x48 or holding reg 0x0001 = 1. No printf in binary mode.
- Commands may take 75 % of link time (`CMD_LINK_SHARE_PCT`). Host silent
  10 s: claims and armed stage dropped.

## Wire types

Big-endian integers, no floats (declared scales: mV, mA, uV, urad, mrad/s,
ppm, Q16.16, Q28). `str` = `u8 len` + ASCII. Short read -> `CMD_ERR_LENGTH`,
full writer -> `CMD_ERR_DEVICE`; never a truncated frame.

Every op taking parameters answers **`u8 took`**: `1`, or `0` + `str` with
the board's reason (`wr_took`). The host validates only what stops a
request being formed.

| Firmware status | Exception |
| --- | --- |
| `CMD_ERR_UNKNOWN`, no such command | 01 ILLEGAL FUNCTION |
| `CMD_ERR_LENGTH`, wrong payload length | 03 ILLEGAL DATA VALUE |
| `CMD_ERR_VALUE`, a field out of range | 03 ILLEGAL DATA VALUE |
| `CMD_ERR_DEVICE`, the board could not comply | 04 SERVER DEVICE FAILURE |

The host reads a `took` reply to its last byte, an exception as 5 bytes,
anything else until the line is quiet.

## Standard Modbus map

`comms/src/modbus_map.c`, function codes 01 to 06 and 16.

Input registers (04):

| Address | Contents |
| --- | --- |
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

Append = MINOR; move, resize or repurpose = MAJOR.

### 0x42 ADC_TABLE

Request: empty, or `u8 first`. Reply: `u8 n`, then `n` rows of
`u8 adc_index, u8 channel, str pin, u8 differential, str signal,
i32 raw, i32 uV, u8 unit, i32 scaled`, then `u8 total` appended. Pages
(nine rows exceed a PDU). `scaled` uses the record (invariant 7).

### 0x43 ADC_SCAN

Request: empty. Reply: `i32 u, i32 v, i32 w` raw, `i32 dc_raw,
i32 dc_mv, i32 ntc_raw, i32 ntc_centi_c, u8 afe_on, u8 pe15`, then
`u8 afe_users` appended. NTC field 0 when the AFE is off.

### 0x44 ADC_NOISE

Request: `u8 adc` (1 .. 3), `u16 samples` (1 .. 1000). Reply:
`u16 samples, i32 mean_uv, i32 min_raw, i32 max_raw, u32 span,
u32 sd_uv`.

### 0x45 CLOCK

Request: empty. Reply: `u32 sysclk_hz, u32 hclk_hz, u32 cycles,
u32 ticks_per_us, u8 sysclk_source`, then `u32 adc_hz` appended.

### 0x46 AFE

Request: `u8 op` - 0 read, 1 off, 2 on, 3 toggle. Reply:
`u8 afe_on, u8 pe15`, then `u8 users` appended. On/off go through the
reference count (`board_power.c`); `users` says who holds the rail.

### 0x47 LINK_STATS

Request: empty. Reply for the current port: `u8 unit_id,
u32 t15_ticks, u32 t35_ticks, u32 bus_message, u32 bus_comm_error,
u32 server_message, u32 server_exception, u32 server_no_response,
u32 char_overrun`.

### 0x48 CONSOLE

Request/reply empty; the UART then returns to the console. Read the reply
by length, not by silence.

### 0x64 TEST_GATE

Request: `u32 key, u8 open`; the key is 0x54455354, "TEST". Reply:
`u8 open`. The pin/port commands need it open.

### 0x65 ECHO

Request: up to 250 bytes. Reply: the same bytes.

### 0x66 .. 0x6A pins and ports

Addressing is `u8 port` as the ASCII letter and `u8 pin`.

| Code | Request | Reply |
| --- | --- | --- |
| 0x66 PIN_MODE | `port, pin, u8 mode, u8 pull` | empty |
| 0x67 PIN_READ | `port, pin` | `u8 level` |
| 0x68 PIN_WRITE | `port, pin, u8 level` | `u8 level` read back |
| 0x69 PORT_READ | `port` | `u16 idr` |
| 0x6A PORT_WRITE | `port, u16 mask, u16 value` | `u16 idr` read back |

PIN_WRITE reads the pin back. Reserved pins (link UART, JTAG/SWD, gate
lines) are refused with 03; `0x6D` kind 2 lists them.

### 0x6B ANALOG_BURST

Request: `u16 mask, u16 samples, u32 interval_us`; the mask non-zero,
samples 1 to 10 000, and `samples x interval` at most 5 s. Reply:
`u16 samples, u32 elapsed_us, u8 count`, then per channel
`u8 index, i32 mean_milliraw, i32 min_raw, i32 max_raw,
u32 sd_milliraw`. A timed-out conversion is 04.

### 0x6C SELF_TEST

Request: empty. Reply: `u8 count`, then per check `str name,
u8 status, i32 value`. Pass/fail only from own registers/flash
(invariant 10): `hse_rdy, pll1_lock, clk_crystal, clk_agrees, cyccnt_runs,
vref_ext, adc_pcsel`; the rest are information.

### 0x6D CHANNELS

Request: `u8 kind [, u8 first]`. The board's own map of itself.

| kind | Reply |
| --- | --- |
| 0 analog | `u8 n`, rows `u8 index, u8 adc_index, u8 channel, str pin, u8 dir, u8 differential, str signal, u8 unit` |
| 1 digital, drivable | `u8 matching, u8 first, u8 sent`, rows `str pin, u8 dir, str signal` |
| 2 reserved | the same shape as kind 1, for the pins a fixture may not drive |
| 3 subsystems | `u8 groups`, rows `str name, str what, u8 commands` - one per command table |
| 4 parts | `u8 total, u8 first, u8 sent`, rows `str name, str what, str where, str power, u8 state` |

Kinds 1, 2, 4 page from `first`. Part `state`: 0 unknown, 1 ready,
2 unpowered, 3 silent (probed).

### 0x6E DEVICE

Request: `u8 device, u8 op, parameters`. Devices 0 .. 11; unknown is 03.

## Devices

### 0 IMU, `cmd_imu.c`

BNO085 on SPI2. Ops:

| op | Request | Reply |
| --- | --- | --- |
| 0 id | - | `u8 reset_cause, u8 sw_major, u8 sw_minor, u32 sw_part, u32 sw_build, u16 sw_patch` |
| 1 read | - | `u8 channel, u8 len`, the SHTP cargo as it arrived; len 0 is nothing waiting |
| 2 feature | `u8 report_id, u32 interval_us` | empty; 0 disables the report, 04 when the part will not take it |
| 3 probe | `[u8 len, u8 select]` | `u32 kernel_hz, u32 bitrate, u8 len` and what the bus answered; len absent or 0 is the header, select absent is 1 |
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
gyroscope Q9, magnetometer Q4. `updates` is monotonic. Bus ops need
the loop held (op 9). Powered by AFE_ON.

### 1 ANGLE, `cmd_angle.c`

A1335 on SPI4. Ops:

| op | Request | Reply |
| --- | --- | --- |
| 0 read | `u8 reg` | `u8 reg, u16 value, u8 crc` |
| 1 write | `u8 reg, u8 value` | empty; 03 above 0x3F, 04 when the part will not take it |
| 2 latest | - | `u8 loop, u8 error, u32 updates, u32 errors, u8 have, u8 reg, u16 value, u8 crc` |
| 3 hold | - | `u8 loop` |
| 4 resume | - | `u8 loop` |
| 5 pollreg | `[u8 reg]` | `u8 reg` the loop reads |
| 6 clock | - | `u32 kernel_hz, u32 bitrate` |

Registers: ANG 0x20, STA 0x22, ERR 0x24, XERR 0x26, TSEN 0x28, FIELD 0x2A.
CRC reported, not checked. Read/write refused while polling.

### 2 LINK, `cmd_link.c`

The serial links, by index. Ops:

| op | Request | Reply |
| --- | --- | --- |
| 0 echo | `u8 port` | `u8 port, u8 rs485, u8 matched, u8 seen, str name` |
| 1 stats | `u8 port` | `u8 port, u8 unit_id, u8 rs485, u8 open, u32 baud, u32 t15_ticks, u32 t35_ticks, u32 bus_message, u32 bus_comm_error, u32 server_message, u32 server_exception, u32 server_no_response, u32 char_overrun, u32 dropped, str name` |

Op 0 `matched`: one bit per pattern 00 FF 5A A5; refused on the port
carrying the request.

### 3 CAL, `cmd_cal.c`

The record at 0x081E0000, magic 'CX63', CAL_VERSION 15, CRC-16/MODBUS.

| op | Request | Reply |
| --- | --- | --- |
| 0 get | - | `u8 stored, u16 version, u8 15`, params 0 .. 14 as u32, `u8 10`, per channel `i32 offset, i32 gain_ppm`, `u8 10`, per node `i32 soa_limit_centi`, `u32 soa_throttle_ppm` |
| 1 set_param | `u8 id, u32 value` | empty; 03 on a bad id |
| 2 set_channel | `u8 index, i32 offset, i32 gain_ppm` | empty |
| 3 zero | `u8 index` | `i32 measured` - what became the offset |
| 4 span | `u8 index, i32 reference` | `i32 measured`; phase (mA) and DC bus (mV) only |
| 5 save | - | `u8 1` |
| 6 load | - | `u8 1` |
| 7 defaults | - | `u8 1`; RAM only until saved |
| 8 params | `[u8 first]` | `u8 total, u8 first, u8 count`, up to 60 u32 |

`stored` = calibrated vs schematic defaults. Another version is refused,
except the two before (prefix layouts). Ids and defaults: `board_cal.c`.

### 4 GATE_DRIVERS, `cmd_gate_drivers.c`

| op | Request | Reply |
| --- | --- | --- |
| 0 state | - | below |
| 1 pwm | `u8 on` | `u8 took`; the only thing that sets MOE, always at zero duty |
| 2 duty | `u16 x3 ticks [, u32 periods]` | `u8 took`; the count since MINOR 8 |
| 3 sync | `u8 on` | `u8 took`; TIM1 triggers the injected ADC group |
| 4 trigger | `u16 ccr` | `u16` as it reads back |
| 5 clear | - | `u8`; clears the break flag, does not re-arm |
| 6 bypass | `u8 on` | `u8`; drops BDTR.BKE |
| 7 gap reset | - | `u8`; forgets the worst keepalive gap |
| 8 dutyq | `u32 x3 Q16.16 ticks` | `u8 took`; sigma-delta dither |
| 9 deadtime | `u32 ns, i8 skew` | `u8 took`, then `u32 ns, i8 skew, u8 floor` as applied |
| 10 alternate | `u16 x3 A, u16 x3 B` | `u8 took`; A one period, B the next |

Op 0: `u8 flags` (0x01 ready, 0x02 enabled, 0x04 fault, 0x08 sync
ready, 0x10 sync armed, 0x20 afe_on, 0x40 pilot ok, 0x80 level ok),
`u16 period, u8 dtg, u16 duty x3, u16 trigger, i16 phase x3, u16 at,
u32 updates, u32 overruns, u32 keepalive, u32 worst_gap, i32 pilot_raw,
i32 pilot_uv, i32 level_raw, i32 level_uv`; appended in this order:
`u8 bypassed`, `u32 requested x3` (Q16.16), `u8 pins, u16 pins_at` (the
six gate lines in one instant), `u32 deadtime_ns, i8 skew, u8 floor`,
`u8 gate_shorts` (bit 0 U, 1 V, 2 W; 0 while armed), `u32 dcbus_raw,
u32 ntc_raw` (MINOR 2), `u32 periods_left` (MINOR 8).

Duty before op 1 is refused. Op 2's count: 500 periods = 10.000 ms.

### 5 LOG, `cmd_log.c`

A ring of 1024 records in DTCM. Ops:

| op | Request | Reply |
| --- | --- | --- |
| 0 state | - | `u8 sources, u16 count, u16 depth, u32 dropped, u32 thinned` - the last appended |
| 1 arm | `u8 source_mask` | `u8 1`; empties the ring |
| 2 take | `[u8 want]` | `u8 got`, per record `u32 at, u8 source, u8 seq, i16 x4` - 14 bytes, fifteen fit a reply |

Sources: 0 phases, 1 angle, 2 imu, 3 drive; equal link share each.

### 6 DAQ, `cmd_daq.c`

| op | Request | Reply |
| --- | --- | --- |
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
`u8 adapt`, `u16 sensors` (MINOR 7). Accumulate 0 closes on the clock
(MINOR 3). Sensor fields: software clock only.

Op 5: `u8 fields, u16 stride`, per field `u8 index, u8 unit,
u8 differential, str signal` in the channel table's order, `u8 digital`
and when set `u8 pins` then per pin `u8 dir, str signal`; appended
(MINOR 7) `u8 count` then per sensor `u8 bit, u8 4, str name` -
orientation, acceleration, rotation rate, magnetic field, shaft angle.

A record: `u32 stamp`, 4 B SUM per analog field, 1 B duty per digital pin,
4x i16 snapshot per sensor, `u16 count`. Use op 5's stride; never recompute.

### 7 TIME, `cmd_time.c`

The cycle counter, latched. Ops:

| op | Request | Reply |
| --- | --- | --- |
| 0 latch | - | `u8 1`; every node captures its CYCCNT at the frame |
| 1 read | - | `u32 seq, u32 latched, u32 now, u32 sysclk_hz` |

Op 0 is for broadcast. Host side: `coaxial.clock`.

### 8 THERMAL, `cmd_thermal.c`

Ops:

| op | Request | Reply |
| --- | --- | --- |
| 0 state | - | below |
| 1 set node | `u8 node, i32 to_board_milli, i32 capacity_milli` | `u8 took` |
| 2 set board | `i32 to_ambient_milli, i32 capacity_milli` | `u8 took` |
| 3 set sample | `u32 every_ms, u32 settle_ms` | `u8 took` |
| 4 budget | - | below |
| 5 set limit | `u8 node, i32 limit_milli_c, i32 throttle_ppm` | `u8 took` |
| 6 set winding | `i32 limit_milli_c, i32 k_per_w_milli, i32 j_per_k_milli` | `u8 took`; a zero ceiling disables the winding, the constants must be positive (MINOR 12) |
| 7 nodes | `u8 first` | `u8 count, u8 first, u8 n`, per node `i32 capacity_milli, i32 to_ambient_milli, i32 area_ppm, i32 rth_milli, i32 forced_milli` - ten a page (MINOR 13) |
| 8 edges | - | `u8 count`, per edge `u8 a, u8 b, i32 r_milli`, zero for an open one (MINOR 13) |
| 9 set edge | `u8 edge, i32 r_milli` | `u8 took`; negative opens it (MINOR 13) |
| 10 ident | - | below |
| 11 ident reset | - | `u8 took`; scales to one, UNCERTAIN, the margin at the floor - nothing is written, so nothing is refused for (MINOR 14) |
| 12 set margin | `i32 floor_ppm` | `u8 took`; the floor into the record's RAM copy, cal op 2 persists it - refused outside 1 .. 1 000 000 in the board's words, zero would put every ceiling at 25 C the moment it booted (MINOR 16) |

Op 0: `u8 ntc_measured, i32 ntc_centi, u8 count`, per node `i32 centi`,
`i32 ambient_centi, i32 expected_ntc_centi, u32 seconds, u8 settled`;
appended `u32 every_ms, u32 settle_ms`, `u8 afe_measured, i32 afe_centi,
u8 mcu_measured, i32 mcu_centi, u32 seen_ms_ago`, `u32 steps`; MINOR 13
appends `i32 x3 junction_over_centi` - each leg's FET junction over its
node - and `i32 speed_rpm`.

Op 4: `u8 count`, per node `u8 used` (0 at ambient, 255 at the limit),
`u8 worst, u8 worst_node, i32 millis_to_limit, u8 throttling,
u8 tripped, u32 trips`; MINOR 11 appends `i32 derate_micro`, per node
`i32 soak_mj`, per phase `i32 duty_micro`; MINOR 12 appends the winding,
`i32 winding_centi, u8 winding_used, i32 winding_derate_micro`.

Op 10: THE ONLINE IDENTIFICATION beside the observer
(`thermal/inc/thermal_ident.h`, MINOR 14): `u8 state` (0 UNCERTAIN,
1 CONVERGING, 2 STABLE), `u8 online_mask` (bit k: the samples move
scale k), `u8 count` (4), per scale `i32 scale_milli, i32 sigma_milli`
in the order air, capacity, spread, ntc, `i32 innovation_milli_k,
i32 margin_micro, u32 updates, u32 saves, u32 since_save_s`; MINOR 15
appends `i32 ambient_centi, i32 ambient_sigma_centi`, MINOR 16
`i32 margin_floor_micro`, MINOR 17 `i32 trip_cap_micro`.

Scales are multipliers on the record's network (1000 = default), in the
order air, capacity, spread, ntc; only air, capacity and the room are
online. The margin multiplies every ceiling's span over 25 C: from the
floor (record `soa_margin_floor_ppm`, default 800 000) to 1 000 000 as the
doubt falls; after a trip, the trip cap (700 000, +1 %/min) while lower.
`saves` is 0 and `since_save_s` all ones: nothing identified is kept.

Nodes (MINOR 13): 0-9 driver U/V/W, phase U/V/W, mcu, regulators, afe,
board (centre patch); 10 hotswap, 11-16 laminate patches, 17 winding,
18 stator, 19 rotor.

### 9 POWER, `cmd_power.c`

The rails and who holds them. Ops:

| op | Request | Reply |
| --- | --- | --- |
| 0 state | - | `u8 rails`, per rail `u8 on, u8 users, u8 count, u8 blocked, u8 leased` |
| 1 release all | - | `u8 took`; every claim dropped |

Users: host, thermal, imu, angle, daq (host unleased, others 3 s leases).

### 10 DRIVE, `cmd_drive.c`

Angles in urad, speeds in mrad/s, currents mA, volts mV; the window's
means and deviations in micro-units.

| op | Request | Reply |
| --- | --- | --- |
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
| 14 observers | - | `u8 valid, i32 theta, i32 omega, i32 blend, i32 dual_theta, i32 dual_omega, i32 flux_theta, i32 flux_omega, i32 lambda_hat, i32 theta_hat, i32 omega_hat, i32 blend_lo, i32 blend_hi, i32 wc` |

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
mrad/s, 4 accel, 5 vd mV, 6 vq mV, 7-9 polarity (`coaxial.drive.SETPOINTS`).

### 11 BOOT, `boot_core.c` and `cmd_boot.c`

Served by the bootloader; the application serves ops 10 and 12 and refuses
the rest (MINOR 18). Blank node = unit 247; `hold`, `erase`, `chunk`, `go`
are broadcasts. Design: [BOOT.md](BOOT.md).

| op | Request | Reply |
| --- | --- | --- |
| 0 hold | `u32 session` | none; every node in its window stays |
| 1 who | `u8 bits, bytes` | `u8 x12 uid, u8 type, u8 state, u8 unit`, from every node whose uid begins with those bits of the prefix; silence from the rest |
| 2 assign | `u8 x12 uid, u8 unit, u8 position, u8 flags` | `u8 took`, from that node only; bit 0 of flags closes the termination |
| 3 erase | `u8 type, u32 size, u32 crc, u16 chunks` | none; a node of that type clears RAM and takes the image's shape, another ignores it - a node whose RAM already holds a valid image of that size and crc keeps it, and one whose store holds it sealed copies it into RAM: verified at once, nothing streamed |
| 4 chunk | `u16 index, bytes` | none; 224 bytes into RAM at `index * 224`, the image's first word held back |
| 5 missing | - | `u16 first, u16 count, bytes` - the bitmap of chunks held |
| 6 verify | - | `u8 ok, u32 crc` over the image as it will stand |
| 7 record | `u16 offset, bytes` | `u8 took`; the record's bytes into RAM |
| 8 seal | `[u8 flags]` | `u8 took`; the record programmed where its words differ from the sector's, the first word written, the image valid; flags bit 0 persists it: the store erased and written, its seal last, unless it holds this image already |
| 9 go | `u32 session` | none; a sealed node of the session jumps |
| 10 state | - | `u8 state, u8 type, u8 unit, u8 position, u32 chunks_held, u32 chunks_of, u8 app_valid, u8 x12 uid, u32 image_bytes, u32 image_crc, u8 flags` - the image RAM holds verified, 0 for none, and assign's flags (MINOR 19) |
| 11 dump | `u16 offset` | `u16 offset, bytes` - the record sector, 224 bytes a page |
| 12 stay | - | `u8 took`; the application writes STAY and resets; the bootloader refuses |

States: 0 blank, 1 held, 2 assigned, 3 erased, 4 verified, 5 sealed.

### Drive op 14

A second angle estimate beside the loop (`drive_observer.c`), steering
nothing. `blend` 0 = dual flux model, 1e6 = leaking flux model, ramped
between `blend_lo` and `blend_hi` (from `wc`). `valid` 0 below `wc`.

### Thermal envelope (op 4, MINOR 11-12)

`derate` (micro) multiplies the drive's current clamp: 1 at the throttle
point, 0 at the ceiling, taken on the worse of now and a lookahead; falls
at once, recovers slowly. `soak_mj` = capacity x (limit - t) per node.
`duty` is the effective duty per phase. The winding is its own envelope;
the stage gets the smaller factor, and either at its ceiling trips.
`tripped` is judged on the record's ceiling, untrimmed.

## Versioning

MINOR appends; MAJOR breaks a codec.

| MINOR | Change |
| --- | --- |
| 1 | gate op 10 alternate |
| 2 | device 10 DRIVE; the DC link appended to gate op 0 |
| 3 | a DAQ record ends with `u16 count`; accumulate 0 closes on the clock - resizes the record, op 5 says the stride |
| 4 | DAQ op 0 appends the buffer level, capacity and high-water mark |
| 5 | DAQ op 4 appends the backlog |
| 6 | IMU op 8 appends the three vectors, each with its own `have` |
| 7 | DAQ op 1 appends a sensor mask; records append four i16 per sensor after the pins; op 5 appends the rows |
| 8 | gate op 2 takes an optional period count; op 0 appends `periods_left` |
| 9 | fixed-shape requests dispatch on their own CRC, not t3.5 |
| 10 | drive op 14, the back-EMF observer chain |
| 11 | thermal budget appends the derate, the soak joules and the effective duty |
| 12 | thermal budget appends the winding - estimate, spend, own factor; thermal op 6 sets its envelope |
| 13 | twenty thermal nodes, the count says so; op 0 appends the FET junction rises and the speed; ops 7, 8, 9 read the node table, the edge table, set an edge |
| 14 | thermal op 10 reads the online identification - state, which scales move, each scale and sigma, innovation, the envelope's margin, saves; op 11 resets it |
| 15 | thermal op 10 appends the room as identified, `i32 ambient_centi, i32 ambient_sigma_centi` - the board has no ambient sensor |
| 16 | thermal op 10 appends `i32 margin_floor_micro` and writes `saves` 0, `since_save_s` never - the margin is continuous on the doubt, the state a word, nothing kept; op 12 sets the floor; op 11 no longer refuses while armed |
| 17 | thermal op 10 appends `i32 trip_cap_micro`, the trip cap as it stands, so a host can say whether the trip or the model holds the margin |
| 18 | device 11 BOOT as the application serves it: op 10 `state`, op 12 `stay`; the rest refused in words. The image sits at 0x08020000 with its header, and a bootloader's assignment reaches it through the handover slot (BOOT.md) |
| 19 | device 11 `state` appends `u32 image_bytes, u32 image_crc, u8 flags` - the image the bootloader verified and ran, and assign's flags; `seal` takes `[u8 flags]`. The application runs from D2 SRAM at 0x30000000; flash at 0x08020000 keeps a sealed copy (BOOT.md) |

MAJOR 2 (2026-08-29): thermal nodes went per leg, indices repurposed.
A host ignores fields past what it knows. `test_conformance.py` holds a
live board to this document.
