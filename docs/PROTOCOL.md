# Protocol

Modbus RTU, 115200 8N1, on three ports. USART3 shares the wire with the ASCII
console at boot; binary mode needs `0x48` or Holding Register `0x0001`. The two
RS485 ports carry Modbus only and are open from boot.

Big-endian integers, no floating point, strings as one length byte then that
many unterminated ASCII characters. `Comms/Inc/cmd.h` carries the byte layouts;
this file carries what a host has to *decide* from.

## RTU framing and compliance

| | |
|---|---|
| Delimiters | silence only - t1.5 = 750 µs, t3.5 = 1750 µs at 475 MHz |
| CRC error | silence, never an exception, so multidrop cannot collide |
| Illegal quantity | `0x03`, checked in 32-bit math so the count cannot wrap |
| Reading not taken | `0x04`, never a zero - on a differential channel code 0 *is* 0 V |
| Broadcast | address 0 executes silently |

Arguments are validated before any conversion runs, so `0x03` means the request
and `0x04` means the device.

## Standard Modbus map

| Table | FC | Holds |
|---|---|---|
| Input registers | `0x04` | raw ADC codes, DC bus mV, NTC 0.01 °C, clock frequencies, error counters |
| Holding registers | `0x03`/`0x06`/`0x10` | unit id (`0x0000`, applied next frame), mode switch (`0x0001`) |
| Coils | `0x01`/`0x05`/`0x0F` | `AFE_ON` (`0x0000`) |
| Discrete inputs | `0x02` | `PE15` / `nFAULT` (`0x0000`) |

## Custom binary commands

FC 65-72 and 100-110, the two user-definable ranges. `0x6E` had to absorb every
new peripheral because the ranges were spent: a second function code answered
ILLEGAL FUNCTION from the protocol layer before dispatch ever saw it.

### 0x41 Version

Append-only struct, the frozen record. Hosts bind decoding to `CMD_PROTO_MAJOR`
alone; appending a field is a MINOR and keeps old hosts working.

MAJOR 2, 2026-08-29: the thermal nodes went per leg, repurposing device 8's
node order and the calibration record's ceiling indices - cmd.h has the
reasoning beside the number.

### 0x42 ADC table

Optional `u8` start index, and `u8 total` appended. A row is 18 bytes plus its
pin and signal names against a 252-byte reply - seven channels came to 197 and
nine to 254 - so the board sends what fits and says how many there are. Absent,
the index reads 0.

### 0x6B Analog burst

Welford statistics in milli-codes over up to `BOARD_BURST_MAX_SAMPLES`, capped
again at `BOARD_BURST_MAX_US` so a burst cannot outlive the master's patience. A
failed conversion aborts the burst rather than folding a zero into the mean.
Scaling is the host's.

### 0x6C Self test

PASS/FAIL only for register-provable states - PLL lock, PCSEL. Anything external
or uncalibrated is INFO, for the host to judge.

### 0x6D Channels

The pin map and the parts list, from the board. One section per request, because
two together came to 273 bytes against `MB_MAX_PDU`'s 253.

| Kind | Section | Request |
|---|---|---|
| 0 | analog channels | `0x6D 00` |
| 1 | digital I/O | `0x6D 01` |
| 2 | reserved pins - the bus and the debug port | `0x6D 02` |
| 3 | subsystems, one per command table | `0x6D 03` |
| 4 | fitted parts | `0x6D 04 <first>` |

Kinds 1, 2 and 4 are paged: six parts with their strings are 380 bytes, and 19
reserved pins are 418. Kind 4 answers `u8 total, u8 first, u8 count`, then per
part `str name, str what, str where, str power, u8 state`. `power` names what
must be on for the part to work at all; `state` is `0` not probed, `1` ready,
`2` unpowered, `3` silent - measured, never asserted (invariant 10).

Adding a part is one row in `Board/Src/board_io.c`; nothing above it needs
telling.

### Refusals come from the board, with a fix

Anything taking parameters answers `u8 took`, and on a refusal a string saying
what is wrong **and what to do**. Only the board knows which check failed; a
host listing causes is a second answer that goes stale when a check moves. The
host validates only what stops a request being formed - a clock name that will
not pack into a byte - and repeats the rest.

```
accumulate=0      -> accumulate counts samples per record, so the smallest is 1
NTC on tim1       -> the TIM1 clock converts the three phases and nothing else -
                     any other channel has to come through the meter on the
                     software clock
start twice       -> already running - stop it first, or leave it be
```

## `0x6E` Device

`0x6E <device> <op> [payload]`. Adding a device is a row in `cmd_device.c` and
an op dispatcher beside it.

| Device | Part | Bus | Ops |
|---|---|---|---|
| 0 | BNO08X IMU | SPI2, mode 3, 2.97 MHz | 0 product id, 1 raw cargo, 2 Set Feature, 3 raw bytes off the bus, 4 reset, 5 raw write on any SHTP channel, 6 per-pin drive/pull check, 7 time H_INTN's answer to a wake, 8 shared record, 9 hold, 10 resume |
| 1 | A1335 angle sensor | SPI4, mode 3, 1.86 MHz | 0 read register, 1 write register, 2 shared record, 3 hold, 4 resume, 5 which register the loop reads, 6 clock |
| 2 | the three serial ports | USART3, USART2, UART5 | 0 loopback check, 1 per-port counters |
| 3 | the calibration record | flash, bank 2 sector 7, CAL_VERSION 5 | 0 get, 1 set param, 2 set channel, 3 zero, 4 span, 5 save, 6 load, 7 defaults |
| 4 | the gate drivers | TIM1, injected ADC, STO chain | 0 state, 1 pwm on/off, 2 duty x3, 3 sync arm/disarm, 4 sample point, 5 clear break, 6 bypass break, 7 reset worst gap, 8 duty Q16.16, 9 dead time + skew |
| 5 | the measurement ring | phases, angle, IMU | 0 state, 1 arm a source mask, 2 take a burst |
| 6 | one acquisition task | ADC, optionally clocked by TIM1 | 0 state, 1 configure, 2 start, 3 stop, 4 read, 5 layout, 6 live |
| 7 | the cycle counter | latched, for a host to tie a clock to | 0 latch, 1 read |
| 8 | the thermal observer | NTC, both dies, the model | 0 state, 1 set node, 2 set board, 3 set sampling, 4 budget, 5 set limit |
| 9 | the rails and who holds them | AFE_ON | 0 state, 1 release all |

`coaxial.Coaxial63100` is the host side and the preferred way in - `acquire()`
and `write()` over the raw ops.

### Device 8 - the thermal observer

**Measured and estimated never share a field.** Op 0 sends each thermometer
with its own flag and the node temperatures apart from them: with AFE_ON low
there is no NTC, no die and no reference, so the model runs on power and time
and the reply has to be able to say that rather than send a stale number as a
live one. `seen_ms_ago` is the age of the whole sample, and judging it is the
host's (invariant 10) - a reading two intervals old is not a measurement.
`steps` closes op 0: how many times the model has integrated. `seconds` is
wall clock beside it, so its rate is 1.0 whatever the observer does, and a
benchmark watching that could only see the observer stop, never slow down.

**The nodes are PER LEG, and there are ten of them.** `driver U/V/W`,
`phase U/V/W`, `mcu`, `regulators`, `afe`, `board`, in that order - the order
`thermal_node_t` declares and the order every op answers in. It was six until
2026-08-29, when the camera showed switching one leg heating one leg and the
estimate showing all three (FINDINGS). Both ops that carry nodes send `u8
nodes` first, so a host that reads the count follows; one that assumed six
does not. The calibration record resized with them - `CAL_VERSION` 7.

Op 4 is the budget: `u8 nodes`, one byte a node, then `worst`, `worst_node`,
`i32 millis_to_limit`, `throttling`, `tripped`, `u32 trips`. A byte because
"how close" is the question and a temperature cannot answer it without the
ceiling beside it; milliseconds because 35 W into the phase node crosses the
throttle point with under a second left.

The ceilings live in the calibration record (device 3), not here. The board
holds a limit it was given and never invents one - what it does with it is
act, dropping MOE at the ceiling, which is the narrow exception invariant 10
carries.

### Device 9 - the rails

AFE_ON is reference counted, so `on` after an explicit off means somebody else
still holds it. That is a different thing from a write that never landed, and
the users bitmask is what tells them apart. Every hold but the host's is a
lease and expires on its own: the observer took the rail, `link_busy()` starved
the poll holding the release, and it stayed high until reset.

`on` is the PIN, read back, not what the count implies. They should agree, and
the case worth reporting is the one where they do not.

### Devices 0 and 1 - the SPI sensors

**The board polls both from its main loop into shared memory; a host reads
that.** Reading a cargo per request cost 45 ms each and caught one frame in
eight. Ops that drive a bus are refused unless that device's loop is held - both
running is two masters on one bus. Hold, configure, resume.

The IMU's record:
`u8 loop, u8 error, u32 updates, u32 cargoes, u32 errors, u8 have`, then
`u8 report_id, u8 status` and four Q14 counts. The angle sensor's:
`u8 loop, u8 error, u32 updates, u32 errors, u8 have, u8 register, u16 value, u8 crc`.
`updates` is monotonic in both, so a host tells a new reading from the same one
read twice without guessing from the values.

The A1335's packet is 20 bits (Figure 31): MOSI is SYNC=0, R/W, six address
bits, eight data bits, four CRC bits; MISO is sixteen data bits and four CRC
bits. It goes out as four 5-bit words under one chip select - `HAL_SPI_Init`
refuses a data size above 16 bits on SPI4, which `IS_SPI_HIGHEND_INSTANCE` does
not name. **The answer lags one frame**, so a register read is two packets. The
CRC is reported and not checked: the datasheet gives the field's width and not
its polynomial.

### Device 2 - the serial ports

Op 0 transmits 00, FF, 5A, A5 on the named port and answers which came back -
all four on an RS485 port, none on USART3. **The port carrying the request
refuses**: its own patterns land in front of the reply and the master sees a
checksum failure. Op 1 answers `bus_message` and `server_message` separately,
and their difference is traffic addressed to another node on the segment.

### Device 3 - the calibration record

Where a code becomes a quantity. Nine scalars - reference, shunt, amplifier
gain, both divider resistors, four thermistor constants - and one offset/gain
pair per ADC channel. Op 0 answers
`u8 stored, u16 version, u8 params, u32 x params, u8 channels`, then
`i32 offset, i32 gain` per channel; 97 bytes today. `stored` is the difference
between a calibrated board and one running the schematic's arithmetic, and
nothing else in the reply shows it.

Integers in the unit that makes them integers - microhms, ppm, microvolts,
centikelvin - because the wire bans floating point. A gain correction is ppm of
1 V/V, applied as `1 + ppm/1e6` after the offset is subtracted.

Only op 5 writes flash. It erases a 128 KB sector, reprograms it 32 bytes at a
time, and answers from the read-back rather than the programmer's return value -
so a master needs a timeout of seconds. The sector is the last of bank 2 and the
image is in bank 1, so the erase does not stall the core fetching instructions.

Op 4 (span) is refused where the quantity is not linear in the code: the
thermistor is logarithmic, and a channel with no unit has nothing to be told.
Both answer SERVER DEVICE FAILURE, as does a channel reading zero - no finite
gain turns nothing into something. A refused edit leaves the record
byte-for-byte unchanged, which `test_conformance.py` checks after every refusal
it provokes: the first version validated after assigning and rolled back by
reloading flash, which does nothing on a board whose record has never been
saved.

### Device 4 - the gate drivers

TIM1, the synced phase triple and Safe Torque Off answer together because tuning
the sample point needs all three from the same moment.

**Op 0** replies 48 bytes: `u8 flags`, `u16 period`, `u8 deadtime`,
`u16 duty[3]`, `u16 trigger`, `i16 phase[3]`, `u16 at`, `u32 updates`,
`u32 overruns`, `u32 keepalive`, `u32 worst_gap`, then
`i32 pilot_raw, pilot_uv, level_raw, level_uv`, then `u8 flags2`. Flag bits LSB
first: pwm ready, pwm enabled, break latched, sync ready, sync armed, AFE on,
Cinj read, Clevel read. `flags2` bit 0 is the break bypass - appended rather
than squeezed into the first byte, which is full, because moving an offset
breaks every decoder for one bit.

`deadtime` is raw DTG, not nanoseconds. `trigger` is CCR4 in timer ticks. **Both
ends of the range disable it**: 0 because OC4REF in PWM1 never goes active, ARR
because the compare never falls below the counter either - measured, `updates`
stops and the latched triple freezes at its last value, which reads as a
perfectly quiet channel. A value past ARR is refused with CCR4 unchanged, and op
4 replies with the register as it reads back so the caller sees the refusal.
`at` is `TIM1->CNT` as the interrupt read it, about 965 ticks (4.06 µs) after
the sample - the sample point is `trigger`, not `at`.

`worst_gap` is the longest interval between keepalive edges in **raw CYCCNT
ticks** (invariant 2). **Op 7** forgets it, so a run can be measured on its own.

**Op 8 is op 2 with the fraction kept.** One tick of ARR 2375 is 0.0421 % of
duty, so an asked-for 34.54 % is 820.32 ticks and neither 820 nor 821 is it. Op
8 takes `u32 x3` in ticks Q16.16 and a first-order sigma-delta in TIM1's update
interrupt spends the whole ticks and carries the fraction, so the **mean** is
what was asked for. Op 0 appends the requested value beside the register, so a
caller sees the two differ on purpose rather than think it was rounded.

Measured, sampling the register asynchronously 120 times: 34.540 % asked came
back 34.5379 %, 10.000 % gave 10.0011 %, 75.250 % gave 75.2495 %. First order
buys three adds in a 50 kHz interrupt and costs **idle tones** - the pattern is
periodic and its lines sit below the switching frequency. The interrupt is
enabled by the first fractional duty and disabled by the next whole one; an
interrupt that does nothing should not run at 50 kHz, though measured it costs
little - worst keepalive gap 190.4 µs with it on against 186.5 off.

**Op 9** takes `u32 nanoseconds, i8 skew` and answers `u8 took`, then
`u32 ns, i8 skew, u8 floor`. Both in one op because they constrain each other: a
skew is only legal against a dead time big enough to carry it. The board floors
dead time at **20 ns** - the 2EDL8034 has no interlock, so this is the only
thing between the two FETs of a leg - and refuses a skew taking either half
under it. Op 0 appends the same three.

The skew exists because TIM1's dead-time generator puts the same DTG on both
transitions and a real stage is not symmetric. It cannot come from moving a
compare register - that shifts the whole transition and leaves the gap alone. It
comes from writing DTG itself between the two, which is why `RCR` is **0** and
the update lands at every overflow *and* every underflow. **Not measured**: what
it does at the gates needs two probes and a scope.

**Op 6 disconnects the break input** - it clears `BDTR.BKE`, not just the `BIF`
latch, because with nFAULT low the break is a *level* and the hardware holds MOE
clear whatever software does. Bench work only, and a reset restores it. What
makes it safe is not firmware: the STO chain gates the gate drivers' own DC/DC,
which no MCU pin reaches.

Op 2 takes all three compares or none: a half update runs one cycle with two
phases from this call and one from the last. Op 1 always enables at zero duty.
Op 5 clears the break latch and does **not** re-arm; with nFAULT still low it
re-latches before op 1 can succeed, which is the STO interlock, not a bug.

### Device 5 - the measurement ring

One sample per round trip caps a host at a couple of hundred samples a second
whatever the board managed - a 53-byte reply at 115200 is 4.6 ms - so the board
rings 1024 records and hands out fifteen at a time.

**Op 0** replies `u8 sources, u16 count, u16 depth, u32 dropped, u32 thinned`.
`thinned` is appended (MINOR 21) and is not `dropped`: dropped is a sample the
ring had no room for, thinned is one the board declined because that source had
already used its share of what the link can drain. Each armed source gets
`cmd_link_records_per_second(14) / armed`, held as a minimum gap in raw CYCCNT.
Without it the angle loop's 24 kHz filled a 1024-deep drop-newest ring in 43 ms
and the IMU's 50 Hz never got in - measured, 1 record a second against angle's
198.

**Op 1** takes a source bitmask (bit 0 phases, 1 angle, 2 IMU) and **empties the
ring**, because a burst whose first records predate the run is worse than an
empty one and no field would say so. **Op 2** takes an optional `u8 want` and
replies `u8 got` then that many 14-byte records:
`u32 at, u8 source, u8 seq, i16 v[4]`.

`at` is raw CYCCNT (invariant 2). `seq` counts per source, so a gap is visible
without trusting the timestamps. `v` is source-defined and raw: phases are U, V,
W and `TIM1->CNT` at the latch; angle is value, CRC, register; IMU is the
quaternion. Measured, phases captured from the injected interrupt land at
19.81/20.00/20.14 µs min/mean/max against 50 kHz.

Full drops the newest and counts it rather than overwriting the oldest. At 50
kHz the ring is 20 ms of history and a host draining fifteen per round trip
cannot keep up - it is a snapshot buffer, and `dropped` says by how much.

### Device 6 - the acquisition task

Configure / start / read, DAQmx's shape cut to one task - one MCU, three
converters, one timer.

**Op 1** takes
`u16 channels, u8 clock, u8 sample_time, u16 decimate, u16 accumulate, u32 records, u8 digital, u32 interval_us`.
`channels` is a bitmask over `0x6D` kind 0. `clock` is 0 for the main loop or 1
for the injected group, one record per PWM period; a TIM1 clock **carries only
the phases** and any other channel is refused rather than answered with zeros.
`sample_time` is 0..7 over the H7's eight sampling windows, shortest first.
`decimate` keeps one trigger in N, `accumulate` **sums** N samples into a record
- summing keeps the bits an average would throw away and the host has the count
- and `records` of 0 runs until stopped. `digital` appends one `u32` of pin
levels per record, the drivable pins only, sampled at the record's timestamp
rather than summed. `interval_us` is the software clock's minimum gap between
samples.

**The channel mask is `u16`, from MINOR 23.** It was `u8` and the ninth ADC
channel did not fit. This **resizes a field** rather than appending one, so it
is the one place a host older than 23 mis-decodes rather than simply missing
something. Not a MAJOR - invariant 3's append-only rule is 0x41's and 0x41 is
untouched - but a break, written down here rather than hidden in a MINOR.

**AFE_ON off stops the task and empties the buffers.** That pin powers the ADC
reference, so every channel would read exact mid-scale (invariant 9) - and an
accumulator holding half a window of real samples and half of mid-scale divides
out to something entirely plausible with no field to say so. Op 0's flag bit 2
says it happened. A stopped task stays stopped: turning the supply back on does
not restart it, because nothing else would have noticed the gap.

**The board picks its own rate when asked for none.** Free-running with
`interval_us` 0 is the one combination that took the link down, so `configure`
replaces it with what the link can carry, from the task's stride and the baud of
whichever port is answering. Op 0 reports it as `max_rate_hz`. Measured at
115200 over the debug probe's VCP:

| channels | stride | max rec/s | interval |
|---|---|---|---|
| 1 | 8 | 475 | 2105 µs |
| 3 | 16 | 237 | 4219 µs |
| 7 | 32 | 118 | 8474 µs |
| 7 + digital | 36 | 105 | 9523 µs |

A third of the line rate is measured, not derived - 3.8 kB/s of payload against
11.52 kB/s raw, the rest being the request, the turnaround and the host's
latency, none of which the board can compute. Running free at the board's own
rate delivered 88 rec/s with **zero drops**, so the guess sits just under the
ceiling. A finite run is left alone: it stops on its own, and a short burst at
full speed is the point of one.

**The ceiling is on records, and a record is `decimate` × `accumulate`
triggers**, so the substituted interval gates the triggers at that multiple.
Gating at the record rate instead would have sampled sixteen times slower at
`accumulate` 16 rather than averaging sixteen samples - the same output, every
sample but one thrown away. Reduce on the target; do not slow it down.

Measured, zero drops throughout:

| task | accumulate | rec/s | samples/s |
|---|---|---|---|
| 1 channel | 1 | 376 | 376 |
| 1 channel | 16 | 294 | 4701 |
| 1 channel | 64 | 182 | 11614 |
| 7 channels | 1 | 96 | 96 |
| 7 channels | 16 | 89 | 1422 |
| 7 channels | 64 | 29.5 | 1886 |

Sixteen-fold averaging costs 7 % of the output rate on seven channels. Where
samples/s stops climbing is the board's limit, not the link's: about **11.6 kHz
on one channel and 13.2 k conversions/s in total**, which is the converter and
the main loop.

**Reads are already whole frames.** One `read` fills a Modbus PDU, so blocking
bigger buys nothing - the payload ceiling is about 3.8 kB/s whatever the record
size, and records per second scale inversely with stride. Past that the only
thing that helps is producing fewer records, which is what `accumulate` and
`decimate` do on the target. Measured: seven channels and the digital word drop
3851 records at `accumulate` 1, and none at 16.

**Op 4** replies `u8 got` then that many records of `u32 at` plus one `i32` per
enabled channel. Whole records only: half of one is not a short read, it is a
corrupt one.

**Op 5 is what makes op 4 decodable.** `u8 fields, u16 stride`, then per field
`u8 channel, u8 unit, u8 differential, str signal`, then `u8 digital` and, when
set, `u8 pins` and per pin `u8 direction, str signal`. Only the drivable pins:
naming all twenty-three came to 312 bytes against `MB_MAX_PDU`'s 253 and the
reply failed outright. A host builds its decoder from that, so a channel added
to `Board/Src/board_adc.c` shows up in a capture with nothing else told.

Measured: the TIM1 clock lands at 19.93/20.00/20.09 µs min/mean/max against 50
kHz, and `decimate=2` with `accumulate=50` gives exactly 2000 µs per record. The
software clock manages about 10.6 kHz on two channels.

**Op 6 is the other way to read, and it cannot overflow.** Every trigger adds
into a static accumulator sized to the maximum channel count; op 6 takes it away
and resets it. A late reader gets a **wider averaging window**, not a backlog -
the ring drops when full, this has nothing to drop.

It replies `u8 fresh` and stops there when nothing has arrived since the last
take. Otherwise `u32 first, u32 last`, then per field an `i32` sum, **a `u32`
count of the additions that went into it**, and the `i32` lowest and highest it
saw, then the digital word if the task has one. The two ends are measured rather
than inferred: a mean and a count cannot tell you a spike happened, and it is
the same two comparisons a meter face would make anyway.

**One count per channel, not one for the lot.** The software poll reads one
channel per turn of the main loop, so a take lands mid-sweep and the channels
have had different numbers of samples: measured on seven channels over half a
second, 1044/1043/1043/1043/1044/1044/1044. A single count would divide six of
them by the wrong number.

**The sample loop is not throttled.** It runs at whatever the converter and the
main loop manage - 14 610 conversions per second on seven channels - because
that is what makes the window worth having. `interval_us` gates **record**
production instead: the ring is a capture and its rate is the link's business,
the accumulator's is not.

**Blocking is the caller's side.** `fresh` of 0 is the answer, not a wait. A
slave sitting on a reply until a sample arrived would hold the segment silent
past t3.5 and break framing for everyone else on it.

Measured, seven channels free-running: takes 50 ms apart returned 10, 16, 20,
25, 31 and 36 samples with means tracking the meter (NTC 40859-40884 against
40878.7), and a 1.5 s wait returned **166 samples over 1 578 492 µs with nothing
dropped**.

### Device 7 - the cycle counter

Every timestamp this board makes is raw CYCCNT (invariant 2), which leaves a
host holding ticks. **Op 0 latches the counter and is meant to be BROADCAST**: a
broadcast has no reply, so the board acts at an instant the host can bracket
with no turnaround in the middle. Op 1 then fetches
`u32 seq, latched, now, sysclk_hz`, and being late costs nothing - the value
stopped moving when taken.

Measured on the debug probe's VCP:

| method | uncertainty |
|---|---|
| broadcast bracket | **5 243 µs** |
| round trip, best of 20 | 35 883 µs, so ~17 941 µs one way |

A 16-byte reply is 1.7 ms of line time; the rest is the VCP driver's latency
timer, which a broadcast never waits for. A segment with a different driver may
answer differently - `clock.probe()` is kept so the two can be compared rather
than assumed.

The rate is measured, not taken from `sysclk_hz`, and **against UTC rather than
against the host**: `sync()` takes an SNTP offset at each end of its own window
and removes the host's offset from the epoch and the host's rate from the
frequency. This bench PC was 947 ms out and 25.3 ppm slow (fitted over 121 s)
six minutes after Windows called its sync good, so tying the board to it
measured the wrong oscillator. Corrected, the board runs **-11.62 ppm** over 900
s against a 1.11 ppm floor. No network and it falls back to the host clock and
records that in `Sync.note`.

**CYCCNT wraps every 9.04 s at 475 MHz**, so any series longer than that has to
be unwrapped - and so does a sync window, which is why `sync()` samples through
its window instead of refusing one longer than a wrap. The floor on the rate is
the reference's noise over that window: 16.5 ppm at 60 s, 1.1 ppm at 900 s, on
`Sync.floor_ppm`.

The board keeps no wall clock and is not given one. No RTC and no LSE, so a time
it held would drift against nothing, and a board reporting a plausible wrong
time is worse than one reporting ticks.

## Hardware safeguards and conformance

* **Reserved pins are hard-masked** in atomic `port_write`: USART3 `PB10/PB11`
  and JTAG `PA13-15/PB3/PB4`, so a host cannot sever its own link or the debug
  port.
* **Conformance is independent.** `test_conformance.py` implements a separate
  Modbus stack from scratch, so shared code cannot hide a fault, and validates
  logical writes through physical side effects - writing the `AFE_ON` coil and
  watching the hardware invert `PE15`.
