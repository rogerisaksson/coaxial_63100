# Protocol

Modbus RTU, 115200 8N1, on three ports. USART3 shares the wire with the ASCII
console at boot; binary mode needs `0x48` or Holding Register `0x0001`. The
RS485 ports carry Modbus only, from boot. Big-endian integers, no floating
point, strings as one length byte then that many ASCII characters.
`Comms/Inc/cmd.h` carries the byte layouts; this file what a host has to
*decide* from.

## RTU framing and compliance

| | |
|---|---|
| Delimiters | silence only - t1.5 = 750 µs, t3.5 = 1750 µs at 475 MHz |
| CRC error | silence, never an exception, so multidrop cannot collide |
| Illegal quantity | `0x03`, checked in 32-bit math so the count cannot wrap |
| Reading not taken | `0x04`, never a zero - on a differential channel code 0 *is* 0 V |
| Broadcast | address 0 executes silently |

Arguments are validated before any conversion runs: `0x03` means the
request, `0x04` the device.

## Standard Modbus map

| Table | FC | Holds |
|---|---|---|
| Input registers | `0x04` | raw ADC codes, DC bus mV, NTC 0.01 °C, clock frequencies, error counters |
| Holding registers | `0x03`/`0x06`/`0x10` | unit id (`0x0000`, applied next frame), mode switch (`0x0001`) |
| Coils | `0x01`/`0x05`/`0x0F` | `AFE_ON` (`0x0000`) |
| Discrete inputs | `0x02` | `PE15` / `nFAULT` (`0x0000`) |

## Custom binary commands

FC 65-72 and 100-110, the user-definable ranges - spent, which is why `0x6E`
absorbs every new peripheral: a second function code answered ILLEGAL
FUNCTION from the protocol layer before dispatch saw it.

### 0x41 Version

Append-only, the frozen record. A host binds decoding to `CMD_PROTO_MAJOR`
alone; appending a field is a MINOR. MAJOR 2, 2026-08-29: the thermal nodes
went per leg, repurposing device 8's node order and the record's ceiling
indices - cmd.h has the reasoning beside the number.

### 0x42 ADC table

Optional `u8` start index, `u8 total` appended. A row is 18 bytes plus its
pin and signal names against a 252-byte reply - seven channels came to 197
and nine to 254 - so the board sends what fits and says how many there are.

### 0x6B Analog burst

Welford statistics in milli-codes over up to `BOARD_BURST_MAX_SAMPLES`,
capped again at `BOARD_BURST_MAX_US`. A failed conversion aborts the burst
rather than folding a zero into the mean.

### 0x6C Self test

PASS/FAIL only for register-provable states - PLL lock, PCSEL. Anything
external or uncalibrated is INFO, for the host to judge.

### 0x6D Channels

The pin map and the parts list, from the board. One section per request -
two together came to 273 bytes against `MB_MAX_PDU`'s 253.

| Kind | Section | Request |
|---|---|---|
| 0 | analog channels | `0x6D 00` |
| 1 | digital I/O | `0x6D 01` |
| 2 | reserved pins - the bus and the debug port | `0x6D 02` |
| 3 | subsystems, one per command table | `0x6D 03` |
| 4 | fitted parts | `0x6D 04 <first>` |

Kinds 1, 2 and 4 are paged: six parts with strings are 380 bytes, 19
reserved pins 418. Kind 4 answers `u8 total, u8 first, u8 count`, then per
part `str name, str what, str where, str power, u8 state`. `power` names
what must be on for the part to work; `state` is `0` not probed, `1`
ready, `2` unpowered, `3` silent - measured, never asserted (invariant
10). Adding a part is one row in `Board/Src/board_io.c`.

### Refusals come from the board, with a fix

Anything taking parameters answers `u8 took`, and on a refusal a string
saying what is wrong **and what to do**. Only the board knows which check
failed; a host listing causes goes stale when a check moves. The host
validates only what stops a request being formed.

```
NTC on tim1       -> the TIM1 clock converts the three phases and nothing else -
                     any other channel has to come through the meter on the
                     software clock
start twice       -> already running - stop it first, or leave it be
```

## `0x6E` Device

`0x6E <device> <op> [payload]`. Adding a device is a row in `cmd_device.c`
and an op dispatcher beside it. `coaxial.Coaxial63100` is the host side.

| Device | Part | Bus | Ops |
|---|---|---|---|
| 0 | BNO08X IMU | SPI2, mode 3, 2.97 MHz | 0 product id, 1 raw cargo, 2 Set Feature, 3 raw bytes off the bus, 4 reset, 5 raw write on any SHTP channel, 6 per-pin drive/pull check, 7 time H_INTN's answer to a wake, 8 shared record, 9 hold, 10 resume |
| 1 | A1335 angle sensor | SPI4, mode 3, 1.86 MHz | 0 read register, 1 write register, 2 shared record, 3 hold, 4 resume, 5 which register the loop reads, 6 clock |
| 2 | the three serial ports | USART3, USART2, UART5 | 0 loopback check, 1 per-port counters |
| 3 | the calibration record | flash, bank 2 sector 7, CAL_VERSION 8 | 0 get, 1 set param, 2 set channel, 3 zero, 4 span, 5 save, 6 load, 7 defaults, 8 params (paged) |
| 4 | the gate drivers | TIM1, injected ADC, STO chain | 0 state, 1 pwm on/off, 2 duty x3, 3 sync arm/disarm, 4 sample point, 5 clear break, 6 bypass break, 7 reset worst gap, 8 duty Q16.16, 9 dead time + skew, 10 alternate: `u16 x3` A, `u16 x3` B - A one PWM period, B the next, swapped by the update interrupt until the next duty write; the thermal observer is charged each leg's mean over the pair (MINOR 1) |
| 5 | the measurement ring | phases, angle, IMU | 0 state, 1 arm a source mask, 2 take a burst |
| 6 | one acquisition task | ADC, optionally clocked by TIM1 | 0 state, 1 configure, 2 start, 3 stop, 4 read, 5 layout, 6 live, 7 filter, 8 tone, 9 rung |
| 7 | the cycle counter | latched, for a host to tie a clock to | 0 latch, 1 read |
| 8 | the thermal observer | NTC, both dies, the model | 0 state, 1 set node, 2 set board, 3 set sampling, 4 budget, 5 set limit |
| 9 | the rails and who holds them | AFE_ON | 0 state, 1 release all |
| 10 | the drive | TIM1, the injected triple, the record, a PMSM model | 0 state, 1 mode, 2 setpoint, 3 setpoints, 4 theta, 5 window, 6 moments arm, 7 moments, 8 reload, 9 cycles reset, 10 source, 11 model param, 12 model, 13 model reset |

### Devices 0 and 1 - the SPI sensors

The board polls both from its main loop into shared memory; a host reads
that. A cargo per request cost 45 ms each and caught one frame in eight.
Ops that drive a bus are refused unless that device's loop is held - hold,
configure, resume.

| Record | Layout |
|---|---|
| IMU | `u8 loop, u8 error, u32 updates, u32 cargoes, u32 errors, u8 have`, then `u8 report_id, u8 status` and four Q14 counts |
| Angle | `u8 loop, u8 error, u32 updates, u32 errors, u8 have, u8 register, u16 value, u8 crc` |

`updates` is monotonic in both, so a new reading is told from the same one
read twice. The A1335's packet is 20 bits (Figure 31): MOSI SYNC=0, R/W,
six address bits, eight data bits, four CRC bits; MISO sixteen data bits and
four CRC bits; four 5-bit words under one chip select, because
`HAL_SPI_Init` refuses a data size above 16 bits on SPI4. **The answer lags
one frame**, so a register read is two packets. The CRC is reported and not
checked - the datasheet gives the width, not the polynomial.

### Device 2 - the serial ports

Op 0 transmits 00, FF, 5A, A5 on the named port and answers which came
back - all four on an RS485 port, none on USART3. **The port carrying the
request refuses**: its own patterns land in front of the reply. Op 1
answers `bus_message` and `server_message` separately; the difference is
traffic addressed to another node.

### Device 3 - the calibration record

Where a code becomes a quantity. Nine scalars - reference, shunt, amplifier
gain, both divider resistors, four thermistor constants - and an offset/gain
pair per ADC channel. Op 0 answers `u8 stored, u16 version, u8 params, u32 x
params, u8 channels`, then `i32 offset, i32 gain` per channel; 97 bytes.
`stored` is the difference between a calibrated board and one running the
schematic's arithmetic. Integers in the unit that makes them integers -
microhms, ppm, microvolts, centikelvin; a gain correction is ppm of 1 V/V,
applied as `1 + ppm/1e6` after the offset.

Only op 5 writes flash: erases a 128 KB sector, reprograms 32 bytes at a
time, answers from the read-back - a master needs a timeout of seconds. The
sector is the last of bank 2, the image in bank 1, so the erase does not
stall the core.

Op 4 (span) is refused where the quantity is not linear in the code - the
thermistor is logarithmic - and on a channel with no unit; both answer
SERVER DEVICE FAILURE, as does a channel reading zero. A refused edit leaves
the record byte-for-byte unchanged, which `test_conformance.py` checks after
every refusal it provokes: the first version validated after assigning and
rolled back by reloading flash, which does nothing on a board whose record
has never been saved.

**Op 8 pages every parameter**: `u8 first` -> `u8 total, u8 first, u8
count, u32 x count`. Op 0 carries the first fifteen and keeps its MINOR 1
shape: with forty-five its reply was 310 bytes against the PDU and every
read answered 0x04. CAL_VERSION 8 added ids 15..44, the drive's - device 10
names them, `coaxial.drive.PARAMS` their units.

### Device 4 - the gate drivers

TIM1, the synced phase triple and Safe Torque Off answer together because
tuning the sample point needs all three from the same moment.

**Op 0**, 48 bytes: `u8 flags, u16 period, u8 deadtime, u16 duty[3], u16
trigger, i16 phase[3], u16 at, u32 updates, u32 overruns, u32 keepalive,
u32 worst_gap, i32 pilot_raw, pilot_uv, level_raw, level_uv, u8 flags2`.
Flag bits LSB first: pwm ready, pwm enabled, break latched, sync ready,
sync armed, AFE on, Cinj read, Clevel read; `flags2` bit 0 the break bypass
- appended, because moving an offset breaks every decoder for one bit.
Then the requested duties, the pins, the dead time and the gate shorts, and
from MINOR 2 `u32 dcbus_raw, u32 ntc_raw`: the DC link as ADC3 rank 2 and
the NTC as ADC1 rank 2 of the injected sequence, read beside the triple.
Both need scan mode, which `Board_SyncArm` switches on once. While the sync
is armed the meter is locked out of every channel but these two;
`Board_Ntc` and `Board_DcBus` answer from the latch, so the thermal
observer keeps its thermometer under the drive.

| Field | Meaning |
|---|---|
| `deadtime` | raw DTG, not nanoseconds |
| `trigger` | CCR4 in timer ticks. **Both ends of the range disable it**: 0 because OC4REF in PWM1 never goes active, ARR because the compare never falls below the counter - measured, `updates` stops and the latched triple freezes, which reads as a perfectly quiet channel. Past ARR is refused with CCR4 unchanged; op 4 replies with the register as read back |
| `at` | `TIM1->CNT` as the interrupt read it - 385 ticks (1.6 µs) after the sample with the instruction cache on since 2026-08-31, 965 ticks (4.06 µs) before it. The sample point is `trigger`, not `at` |
| `worst_gap` | the longest interval between keepalive edges in **raw CYCCNT ticks** (invariant 2). Op 7 forgets it |

**Op 8 is op 2 with the fraction kept.** One tick of ARR 2375 is 0.0421 %
of duty, so 34.54 % is 820.32 ticks and neither 820 nor 821 is it. Op 8
takes `u32 x3` in ticks Q16.16; a first-order sigma-delta in TIM1's update
interrupt spends the whole ticks and carries the fraction, so the **mean**
is what was asked. Op 0 appends the requested value beside the register.
Measured, sampling the register asynchronously 120 times: 34.540 % asked
came back 34.5379 %, 10.000 % gave 10.0011 %, 75.250 % gave 75.2495 %.
First order buys three adds in a 50 kHz interrupt and costs **idle tones**
below the switching frequency. The interrupt is enabled by the first
fractional duty and disabled by the next whole one; measured, worst
keepalive gap 190.4 µs with it on against 186.5 off.

**Op 9** takes `u32 nanoseconds, i8 skew`, answers `u8 took`, then `u32 ns,
i8 skew, u8 floor`; op 0 appends the same three. One op because they
constrain each other: the board floors dead time at **20 ns** - the
2EDL8034 has no interlock - and refuses a skew taking either half under it.
The skew exists because TIM1 puts the same DTG on both transitions and a
real stage is not symmetric; it cannot come from moving a compare (that
shifts the whole transition), so it is written into DTG between the two,
which is why `RCR` is **0** and the update lands at every overflow *and*
underflow. **Not measured** - needs two probes and a scope.

**Op 6 disconnects the break input** - clears `BDTR.BKE`, not just `BIF`,
because with nFAULT low the break is a *level* and hardware holds MOE clear
whatever software does. Bench only; a reset restores it. What makes it safe
is the STO chain gating the drivers' own DC/DC, which no MCU pin reaches.

Op 2 takes all three compares or none. Op 1 always enables at zero duty. Op
5 clears the break latch and does **not** re-arm; with nFAULT still low it
re-latches before op 1 can succeed - the STO interlock, not a bug.

### Device 5 - the measurement ring

One sample per round trip caps a host at a couple of hundred samples a
second - a 53-byte reply at 115200 is 4.6 ms - so the board rings 1024
records and hands out fifteen at a time.

| Op | Request | Reply |
|---|---|---|
| 0 state | - | `u8 sources, u16 count, u16 depth, u32 dropped, u32 thinned` |
| 1 arm | source bitmask: bit 0 phases, 1 angle, 2 IMU | **empties the ring** - a burst whose first records predate the run is worse than an empty one |
| 2 take | optional `u8 want` | `u8 got`, then 14-byte records `u32 at, u8 source, u8 seq, i16 v[4]` |

`thinned` (appended, MINOR 21) is not `dropped`: dropped had no room,
thinned was declined because that source had used its share of what the
link can drain - `cmd_link_records_per_second(14) / armed`, a minimum gap
in raw CYCCNT. Without it the angle loop's 24 kHz filled the ring in 43 ms
and the IMU's 50 Hz never got in: 1 record a second against angle's 198.

`at` is raw CYCCNT (invariant 2); `seq` counts per source, so a gap shows
without trusting timestamps. `v` is raw and source-defined: phases U, V, W
and `TIM1->CNT` at the latch; angle value, CRC, register; IMU the
quaternion. Measured, phases captured from the injected interrupt land at
19.81/20.00/20.14 µs min/mean/max against 50 kHz. Full drops the newest and
counts it: at 50 kHz the ring is 20 ms of history - a snapshot buffer, and
`dropped` says by how much.

### Device 6 - the acquisition task

Configure / start / read - DAQmx's shape cut to one task: one MCU, three
converters, one timer.

**Op 1** takes `u16 channels, u8 clock, u8 sample_time, u16 decimate, u16
accumulate, u32 records, u8 digital, u32 interval_us`. `channels` is a
bitmask over `0x6D` kind 0; `clock` 0 is the main loop, 1 the injected
group, one record per PWM period - a TIM1 clock carries **what the injected
sequence converts**: the three phases, and the DC link and the NTC that
ride rank 2. Any other channel is refused; `sample_time` 0..7 over the H7's eight
windows, shortest first; `decimate` keeps one trigger in N; `records` 0
runs until stopped; `digital` appends one `u32` of drivable-pin levels per
record, sampled at the record's timestamp.

**Two ways to close a record, and `accumulate` picks which** (MINOR 3).

| `accumulate` | closes on | `interval_us` gates | samples a record holds |
|---|---|---|---|
| >= 1 | a count of N | the triggers | N |
| 0 | the clock | the record | whatever the window held |

The clock is what a host wants when the converter is faster than the
link: nothing gates the triggers, every sweep the loop manages goes into
the sum, and the record closes on `interval_us` - so asking for 100
records a second costs no samples, it averages them. Either way the sum
is a SUM: it keeps the bits an average throws away, and the divisor is in
the record.

**The sum saturates rather than wrapping.** A window has no bound on how
many samples it holds and the accumulator is `i32` against a single-ended
code of 65535, so it stops at `LIVE_MAX_ADDITIONS` (INT32_MAX / 65535 =
32767) - the same bound and the same reasoning as the live accumulator's.
The mean over what did go in stays true, and the count says how many that
was; a wrapped sum would divide a negative wreck by the count and call it
a mean. A window that held nothing is not pushed at all.

**The channel mask is `u16`, from MINOR 23** - it was `u8` and the ninth
channel did not fit. A resized field, not an appended one: a host older
than 23 mis-decodes rather than misses. Not a MAJOR - invariant 3 is
0x41's, and 0x41 is untouched - but a break, written down here.

**AFE_ON off stops the task and empties the buffers.** An accumulator
holding half a window of real samples and half of mid-scale divides out to
something plausible with no field to say so; op 0's flag bit 2 says it
happened, and a stopped task stays stopped.

**The board picks its own rate when asked for none.** Free-running with
`interval_us` 0 took the link down, so `configure` substitutes what the link
can carry from the stride and the answering port's baud; op 0 reports it as
`max_rate_hz`. Measured at 115200 over the debug probe's VCP:

| channels | stride | max rec/s | interval |
|---|---|---|---|
| 1 | 8 | 475 | 2105 µs |
| 3 | 16 | 237 | 4219 µs |
| 7 | 32 | 118 | 8474 µs |
| 7 + digital | 36 | 105 | 9523 µs |

A third of the line rate is measured, not derived - 3.8 kB/s of payload
against 11.52 kB/s raw. Running free at the board's own rate delivered 88
rec/s with **zero drops**. A finite run is left alone. The ceiling is on
records, and a record is `decimate` × `accumulate` triggers, so the
substituted interval gates the triggers at that multiple - gating at the
record rate would have sampled sixteen times slower at `accumulate` 16
instead of averaging sixteen samples. Measured, zero drops throughout:

| task | accumulate | rec/s | samples/s |
|---|---|---|---|
| 1 channel | 1 | 376 | 376 |
| 1 channel | 16 | 294 | 4701 |
| 1 channel | 64 | 182 | 11614 |
| 7 channels | 1 | 96 | 96 |
| 7 channels | 16 | 89 | 1422 |
| 7 channels | 64 | 29.5 | 1886 |

Sixteen-fold averaging costs 7 % of the output rate on seven channels.
Where samples/s stops climbing is the board's limit: about **11.6 kHz on
one channel and 13.2 k conversions/s in total**. One `read` already fills a
PDU, so blocking bigger buys nothing - the payload ceiling is ~3.8 kB/s
whatever the record size; past that only fewer records help, which
`accumulate` and `decimate` do on the target. Measured: seven channels and
the digital word drop 3851 records at `accumulate` 1, none at 16.

**Op 0 appends the buffer level** (MINOR 4): `u32 capacity, u32 worst` -
what the ring holds at THIS stride, and the fullest it has been.
`available` alone is a count nobody can read as full or empty without
the first, and a level sampled at a host's leisure misses the peak that
dropped a record - which is the one worth knowing, so the high-water
mark is taken where a record is pushed.

**Op 4** replies `u8 got`, then records of `u32 at`, one `i32` per
enabled channel, the digital `u32` when the task has one, and `u16
samples` last - whole records only. The count travels with the sums
because the clock decides it: a host that took it from the config would
divide by a number the board never used. **This RESIZED the record**
(MINOR 3), like the `u16` channel mask did - op 5 says the stride and a
decoder that recomputes it mis-frames every record after the first. **Op 5** makes op 4 decodable: `u8
fields, u16 stride`, per field `u8 channel, u8 unit, u8 differential, str
signal`, then `u8 digital` and, when set, `u8 pins` and per pin `u8
direction, str signal`. Drivable pins only: all twenty-three came to 312
bytes against 253. A host builds its decoder from that. **The TIM1 clock is the one FOC wants and it does not need the gates.**
MOE is a separate thing: with the sync armed and the stage down the
injected sequence still triggers, so the samples come at the PWM period
whatever the bridge is doing - measured 49 300 samples/s over five
channels, 0 dropped, against the software clock's 1129 sweeps a second
with the same channel count and its scheduling jitter.

Measured: the TIM1
clock lands at 19.93/20.00/20.09 µs min/mean/max against 50 kHz;
`decimate=2` with `accumulate=50` gives exactly 2000 µs per record; the
software clock manages about 10.6 kHz on two channels.

**Op 7 loads the anti-alias chain** (MINOR 4): `u8 count, u16 decimate`,
then five `i32` a section in b0 b1 b2 a1 a2 order, **Q28** - the wire
carries no floating point, and a biquad's a1 reaches -2, so a scale of
2^28 leaves +/-8 of range. `count` of 0 clears it.

THE TASK'S `accumulate` IS THE CHAIN'S FIRST STAGE. One boxcar, not two
that would fight: configure with `chain['boxcar']` and send the sections
and `chain['decimate']` here. What the biquads see is the mean that
accumulate produced, at the precision it bought - pushing the sum
instead multiplied every reading by the count, and a 32 768-code tone
arrived as 8.2 million (FINDINGS). A filter and a clock-closed record
are alternatives and the board refuses the pair: a fixed-rate filter
needs a fixed decimation, and a window's length is whatever the loop
managed. `host/coaxial/bessel.py` designs the coefficients and reports
what the chain fails to stop, which is the number to read before
believing one.

**Op 9 loads one rung of the ladder**: `u8 rung, u16 boxcar, u8 count,
u16 decimate`, then the sections as op 7 takes them. With
`configure(adapt)` the board CLIMBS IT WHEN ITS RING FILLS and comes
back down when the link has caught up, so what a slow link costs is
bandwidth rather than records.

Every rung is a WHOLE design - boxcar, coefficients, decimation -
because decimating harder without redesigning is exactly how a fold
gets in. The board cannot design anything; it chooses between designs
sent to it. Rung 0 forgets every rung above it, so a rebuilt ladder
leaves no stale rung to climb into, and a rung is refused while a task
runs.

It climbs at six eighths of the ring and falls below one eighth after
64 records there - hysteresis, because a level that crosses one
threshold both ways chatters between rungs and every change costs the
filter its settling. A record says which rung made it without a field
for it: `samples` IS that rung's boxcar. Op 0 appends `u8 rung, u8
rungs, u32 rung_changes` beside it.

**Op 8 puts a known tone in the converter's place**: `u32 hz, u32
rate_hz, i32 amplitude, i32 offset`; `hz` of 0 gives the converter back.
For proving the path rather than measuring anything - a host that knows
the frequency, the rate and the decimation knows what every output
sample should be, so a record that fell out of the ring shows up as a
phase that jumped. The generator counts the cycles that elapsed and
makes exactly the samples they bought, bounded at
`BOARD_DAQ_TONE_BURST` a turn: the SEQUENCE is exact, the timing is
bursty, and what the bound drops is dropped rather than owed - a debt
carried forward bursts again and never catches up.
`tools/daq_integrity.py` is the test; FINDINGS has what it measured.

**Op 6 is the other way to read, and cannot overflow.** Every trigger adds
into a static accumulator; op 6 takes it away and resets it - a late reader
gets a wider averaging window, not a backlog. It replies `u8 fresh` and
stops there when nothing arrived; otherwise `u32 first, u32 last`, then per
field an `i32` sum, **a `u32` count of the additions**, the `i32` lowest and
highest seen, then the digital word. One count per channel: the software
poll reads one channel per loop turn, so a take lands mid-sweep - measured
on seven channels over half a second, 1044/1043/1043/1043/1044/1044/1044.
The sample loop is not throttled - 14 610 conversions per second on seven
channels; `interval_us` gates **record** production. `fresh` of 0 is the
answer, not a wait: a slave sitting on a reply would hold the segment past
t3.5. Measured, seven channels free-running: takes 50 ms apart returned 10,
16, 20, 25, 31 and 36 samples with means tracking the meter (NTC
40859-40884 against 40878.7), and a 1.5 s wait **166 samples over
1 578 492 µs with nothing dropped**.

### Device 7 - the cycle counter

Every timestamp is raw CYCCNT (invariant 2). **Op 0 latches the counter and
is meant to be BROADCAST** - no reply, so the board acts at an instant the
host brackets with no turnaround in the middle; op 1 fetches `u32 seq,
latched, now, sysclk_hz` at leisure.

| method | uncertainty |
|---|---|
| broadcast bracket | **5 243 µs** |
| round trip, best of 20 | 35 883 µs, ~17 941 µs one way |

A 16-byte reply is 1.7 ms of line time; the rest is the VCP driver's
latency timer, which a broadcast never waits for. `clock.probe()` is kept
so another driver can be compared rather than assumed.

The rate is measured, **against UTC rather than the host**: `sync()` takes
an SNTP offset at each end of its window and removes the host's offset and
rate. This bench PC was 947 ms out and 25.3 ppm slow (fitted over 121 s) six
minutes after Windows called its sync good. Corrected, the board runs
**-11.62 ppm** over 900 s against a 1.11 ppm floor; the floor is 16.5 ppm at
60 s, on `Sync.floor_ppm`. No network falls back to the host clock and
records that in `Sync.note`. **CYCCNT wraps every 9.04 s at 475 MHz**, so
`sync()` samples through its window instead of refusing a long one. The
board keeps no wall clock: no RTC, no LSE, and a plausible wrong time is
worse than ticks.

### Device 8 - the thermal observer

**Measured and estimated never share a field.** Op 0 sends each thermometer
with its own flag and the node temperatures apart: with AFE_ON low there is
no NTC, no die and no reference, and the reply has to say so rather than
send a stale number as live. `seen_ms_ago` is the age of the sample -
judging it is the host's (invariant 10). `steps` closes op 0: how many
times the model integrated; `seconds` is wall clock beside it, so a
benchmark could only see the observer stop, never slow.

**Ten nodes, PER LEG**: `driver U/V/W`, `phase U/V/W`, `mcu`, `regulators`,
`afe`, `board` - the order `thermal_node_t` declares and every op answers
in. Six until 2026-08-29, when the camera showed switching one leg heating
one leg and the estimate showing all three (FINDINGS). Both node-carrying
ops send `u8 nodes` first; the record resized with them - `CAL_VERSION` 7.

Op 4, the budget: `u8 nodes`, one byte a node, then `worst, worst_node, i32
millis_to_limit, throttling, tripped, u32 trips`. A byte because "how
close" cannot be answered by a temperature without the ceiling beside it;
milliseconds because 35 W into the phase node crosses the throttle point
with under a second left. The ceilings live in the record (device 3); the
board holds a limit it was given and *acts* - drops MOE at the ceiling -
the narrow exception invariant 10 carries.

### Device 9 - the rails

AFE_ON is reference counted, so `on` after an explicit off means somebody
else holds it - the users bitmask tells that from a write that never
landed. Every hold but the host's is a lease that expires on its own: the
observer took the rail, `link_busy()` starved the poll holding the release,
and it stayed high until reset. `on` is the PIN read back, not what the
count implies; the case worth reporting is where they disagree.

### Device 10 - the drive

The control law - `Drive/` behind `board_drive.c` - one step per PWM period
from ADC3's injected interrupt. Angles in microradians, speeds in
milliradians a second, currents mA, volts mV, the window's means and
deviations in micro-units. MINOR 2.

| Op | Request | Reply |
|---|---|---|
| 0 state | - | `u8 mode, u8 fault, u8 flags, i32 theta_hat, i32 omega_hat, i32 theta_cmd, i32 omega_cmd, i32 id, iq, vd, vq, vdc, i32 eps, eps_amps, ih, e_bemf, u32 periods, u32 isr_cycles_last, u32 isr_cycles_max, i32 pol_pos, pol_neg, u16 trigger, u32 ts_ns`, then appended `u16 exit_ticks_max` (the whole interrupt, TIM1 ticks past the trigger), `u32 cyc_sample, cyc_step, cyc_advance` (the virtual step block by block; zero on the converters) |
| 1 mode | `u8` 0 off, 1 volt, 2 hold, 3 sensorless, 4 polarity | `u8 took` |
| 2 setpoint | `u8 id, i32` | `u8 took` |
| 3 setpoints | - | `u8 count, i32 x count` |
| 4 theta | `i32 urad` | `u8 took` - both frames |
| 5 window | - | `u32 n`, then per field `u32 n, i32 mean, u32 sd` for id, iq, vd, vq, eps, ih, vdc; `u8 lags, i32 rho_ppm x lags, i32 i_peak_ma`. Resets |
| 6 moments arm | `u32 periods` | `u8 took`; needs the sync armed |
| 7 moments | - | `u8 done, u32 n, u32 want, u16 trigger`, per channel U, V, W, DC bus: `i32 mean_milli, u32 sd_milli, i32 lo, i32 hi` |
| 8 reload | - | `u8 took`: parameters out of the record |
| 9 cycles reset | - | `u8`: forget the worst step cost |
| 10 source | `u8` 0 converters, 1 model | `u8 took`; refused while a mode runs |
| 11 model param | `u8 id, i32` | `u8 took` |
| 12 model | - | `u8 source, i32 theta_urad, omega_mrad_s, id_ma, iq_ma, vdc_mv`, then the estimate in the same reply: `i32 theta_hat_urad, omega_hat_mrad_s` |
| 13 model reset | - | `u8 took`: the rotor back to theta0, at rest |

`flags`: bit 0 MOE, 1 AFE_ON, 2 injecting (two whole cycles seen), 3 the
drive holds the compares, 4 sync armed. `fault`: 0 none, 1 overcurrent - a
phase passed `drv_i_trip_ma` and the stage was dropped, 2 the stage went
away under a running mode, 3 the supply. Setpoints: 0 id_ref mA, 1 iq_ref
mA, 2 theta mrad, 3 omega_target mrad/s, 4 accel mrad/s2, 5 vd mV, 6 vq
mV, 7 pol_volts mV, 8 pol_periods, 9 pol_gap.

**A mode that switches needs MOE set and AFE_ON on**, and says so. MOE
stays the host's (`gates.arm()`); entering a mode arms the sync. While a
mode runs the drive holds the compares - device 4 ops 2, 8 and 10 are
refused until mode 0 - and commits each triple at the UNDERFLOW so the
pulse is symmetric: written at the overflow it would land mid-pulse, and an
fs/2 injection would average to nothing at the sample point.

**The parameters are the record's** (device 3, ids 15..44): motor R, Ld,
Lq, lambda, pole pairs; loop kp, ki; observer l1, l2; injection mV,
periods, phase, the demodulated gain; the current clamp and the trip; the
voltage fraction; the shunt sign; the blend speeds; the dead-time table;
the measured noise; the sample point. Op 1 reloads them on every mode
change.

**The model is the second source** (op 10): a PMSM the board integrates
itself - dq in the rotor frame, Ld bent by the d current, friction, a load,
the inverter's dead-time volts, the two-period pipeline - fed to the law in
place of the converters, so the observer is watched against a rotor whose
angle is known with the AFE off and no motor; the duties reach real gates
only if MOE happens to be set. Model parameters by id (op 11): 0 r uohm, 1
ld nH, 2 lq nH, 3 lambda uV.s, 4 pole pairs, 5 sat ppm, 6 i_sat mA, 7 J
nkg.m2, 8 B nN.m.s, 9 load uN.m, 10 v_dt mV, 11 i_knee mA, 12 vdc mV, 13
noise uA, 14 theta0 urad, 15 sub-steps. Op 12 carries the estimate beside
the truth because two requests are 15 ms apart - six radians of rotor at
440 rad/s.

**What the board decides**: nothing but the trip - the same exception the
thermal ceiling holds (invariant 10). Measured 2026-08-31, the instruction
cache on, drivers unpowered: the idle step costs 1 780 cycles; sensorless
on the model, injecting and spinning, the whole interrupt ends 12.3 µs after
the trigger of the 20 µs period; the DC link on ADC3 rank 2 read 31.06 V
against the meter's 31.05. FINDINGS, *The caches were off*.

## Hardware safeguards and conformance

* **Reserved pins are hard-masked** in atomic `port_write`: USART3
  `PB10/PB11` and JTAG `PA13-15/PB3/PB4`, so a host cannot sever its own
  link or the debug port.
* **Conformance is independent.** `test_conformance.py` implements a
  separate Modbus stack from scratch, so shared code cannot hide a fault,
  and validates logical writes through physical side effects - writing the
  `AFE_ON` coil and watching the hardware invert `PE15`.
