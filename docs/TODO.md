# TODO

State as of 2026-08-28.

| | Value |
|---|---|
| `run_tests.ps1 -All` | 1713 checks, 18 suites |
| Debug build | 0 warnings |
| FLASH / DTCMRAM | 134 748 B / 48 200 B |
| Protocol | MAJOR 1, MINOR 26 |
| Firmware | 1.4.1 |

## What runs

**TIM1 is configured and the gate drivers are held off.** Centre-aligned, ARR
2375 off 237.5 MHz = 50.000 kHz exact, RCR 1, DTG 19 = 80.0 ns, break on PE15
active low, AOE off. `Board_PwmInit()` starts the counter with MOE clear and
CCxE set, so OSSI drives all six outputs to their idle level. Read back off the
silicon over SWD, not inferred.

**The synced current path works.** TIM1 TRGO2 (OC4REF) → three injected groups →
ADC3's interrupt latches the triple. Measured 49976 triples/s against 50 kHz,
zero overruns, injected values agreeing with the meter inside its own noise. The
sample point is CCR4 and moves anywhere in the period; the handler reads `CNT` a
constant 4.06 µs later. FINDINGS has the sweep.

**KEEPALIVE runs from the main loop.** PA10, toggled above every branch. 214 kHz
idle, 124 kHz while the host polls Modbus.

**USB is configured and nothing sits on it.** OTG_FS device, interrupt priority
5, 48.000 MHz from PLL3Q off the crystal. `MX_USB_OTG_FS_PCD_Init` runs at boot;
with no device class, a host sees a device that fails enumeration.

**`0x6E` device 7 ties the counter to a host clock.** Broadcast latch, measured
at 5 243 µs against a round trip's 17 941 µs one way. The rate is measured
against UTC rather than against the host - this PC is neither (FINDINGS) - so
`sync(reference='utc')` takes an SNTP offset at each end and removes both. The
board runs **-11.62 ppm**, floor 1.11 ppm over 900 s. `clock.unwrap()` handles
the 9.04 s wrap, in a capture and in a sync window alike.

**The link runs at 89.8 % of its bitrate for a full block and 4.5 % for a
ping.** `tools/link_bench.py` measures it. A transaction costs a flat ~5 ms, so
the curve is that amortised. The transport had been spending 31 ms of every 46
on itself - a 20 ms quiet tail, a 5 ms interframe sleep and three 3.25 ms
`serial.timeout` reconfigurations - and is 15.5 ms now.

**A DAQ record is a converter code; its layout's unit is the cooked one.**
`scaling.converter` maps one to the other on the host and both views use it.
Doing it on target would cost a float conversion per sample at the task's own
rate, and the calibration record already publishes every parameter.

**The board says why it refused, and what to do.** `u8 took` plus a sentence, on
every op that takes parameters. `host/coaxial/subsystem.py`'s `took()` is the
whole host side.

**Two ways to read, answering different questions.** Op 4 drains a ring - a
capture, with history, that drops when the host falls behind. Op 6 takes a live
accumulator that cannot drop: a slow link widens the averaging window instead.
Measured, 1.5 s between takes gave 166 samples in one reply and no drops.

**A buffered capture view.** `demos/capture.ps1` drains both buffers at once -
every AFE channel, every drivable pin, both SPI parts - and raises accumulation
on its own when the board reports drops.

**`0x6E` device 6 is an acquisition task** - configure, start, read, in DAQmx's
shape. Channels as a bitmask over the board's own table, a software or TIM1
clock, sampling window 0..7, decimation and accumulation. Op 5 describes the
record so no host holds a copy of its shape. Measured: TIM1 clock
19.93/20.00/20.09 µs against 50 kHz, `decimate=2` × `accumulate=50` exactly 2000
µs, software clock 10.6 kHz on two channels, values matching the meter (NTC
40505.8 vs 40498.3).

**`0x6E` device 5 is the event ring** - 1024 records the angle and IMU loops
push into, drained fifteen per reply.

**`0x6E` device 4 puts the gate drivers on the wire.** State, PWM on/off, duty,
sync arm/disarm, sample point, clear break - `Comms/Src/cmd_gate_drivers.c`,
`host/coaxial/gate_drivers.py`, mirrored in `simulated.py`.

## What blocks the gate drivers

**The STO chain has not released, so PWM cannot be enabled.** Proven: clearing
the break latch and enabling in the same round trip leaves the latch set again,
because PE15 is still low. Two independent conditions are needed and neither is
met - a pilot tone from a master on RS485, and the KEEPALIVE pump. Only the
second runs.

Nothing has run near 63 V or 100 A. No number this board reports has been
measured against an instrument - invariant 7.

## Next, in order

1. **Move the sensor polls off the blocking path.** The worst gap is 163 µs,
   inside the latch's ~200-400 µs, but the edge rate is 36 kHz against the 100
   kHz asked for. Measured by holding each in turn: the **A1335 costs 42 µs per
   loop iteration, the IMU 0.5 µs**. Converting the angle packet to
   interrupt-driven SPI is the cheap half - a fixed 4-byte frame with chip
   select held across it. The IMU's SHTP path is header-then-body with a
   variable length and has already cost six bugs; it deserves its own change.
2. **Cinj and Clevel cannot be sampled asynchronously** - apparent duty tracks
   the sample rate. Take them through the injected group, or with a longer
   sampling time. FINDINGS has the table.
3. **Replace the conformance check `PE15 follows AFE_ON`.** It reads a pin the
   MCU does not drive and changes meaning the moment the STO chain releases.
   Replace it before the supply is switched on, not after.
4. **Dead time is simulated, not measured.** 80 ns comes from a 59.4 ns
   worst-corner gate overlap plus the 2EDL8034's 6 ns TDMOFF. Nothing has been
   on a scope.
5. **A USB device class**, if USB is to do anything.

## Standing

- `electronic_simulations/motor_inverters/half_bridge.asc` has three `mc()`
  calls with no `run`/`val` wrapper, unlike `amplifiers.asc` and `hot_swap.asc`.
  Running that file randomises trace inductance and the shunt.
- CubeMX rewrites `Core/Src/main.c` to CRLF on every generation. Git normalises
  it back; byte-level edits must match `\r\n`.
