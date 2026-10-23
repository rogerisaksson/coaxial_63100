# Findings

Measured on the bench and settled, one entry each. The record before
430b91f: `git show 430b91f:docs/FINDINGS.md`. Subjects off the board: the
table under [Subjects](#subjects).

## AFE and ADC

- AFE_ON (PB2) powers the ADC reference, the BNO085 and the A1335. Off: every
  channel reads exact mid-scale, the NTC exactly 25.00 C.
- PE15 follows AFE_ON inverted; reads as a fault with the AFE on. Cause open.
- Converters calibrated at boot with AFE_ON low, without their reference:
  CALFACT 0, no linearity words. Every single-ended code ~1000 high (a dead
  Vgate 1049; NTC +1.9 K at 35 C; MCU die +20 K; the DC link's -32 418 ppm
  this offset), a wide code 70 below every 512. Fix: the rail up 100 ms
  first (the link within its noise 25 ms after the rail): CALFACT 989-1068,
  one count boot to boot, no wide code. Phase zeros moved 448-539 codes, 3 A:
  every board re-zeroed.
- Self test's image CRC over 0x08000000-`_etext`: bus fault at 0x08200000
  since the image runs from D2 SRAM; taken from the vector table now. Its
  PCSEL count included the injected group's rank 2; the group arms on a
  cleared PCSEL and counts its three.
- HAL only ORs PCSEL: ADC3 PCSEL read 0xC03, four channels live. Every read
  path clears it (invariant 6).
- Phase noise floor, AFE on: 0.35-0.41 A rms per phase.
- A suite borrowing the AFE rail flips another suite's AFE row, ~1 run in 3.
- MCU die through `__LL_ADC_CALC_TEMPERATURE`: whole degrees, under an
  identification with a 0.1 K floor. Factory points in float: 0.01 K.

## Link

- IMU cargo (276 B at 1.48 MHz = 1.5 ms) blocking the poll lost 0.45 % of
  USART3 frames (7 of 1393). Fix: no polling mid-frame.
- Serving a 229-byte DAQ reply (19.9 ms) cut acquisition 477 -> 133 rec/s.
- CubeMX left USART2/UART5 at 9 216 000 baud; baud in the record since
  CAL_VERSION 9.
- Write-class transaction 46.6 ms: t3.5 1.75 ms, `serial.timeout` assignment
  3.25 ms each (x3), QUIET_TIME 20 ms. Now ~13 ms; sized ACK replies stop on
  the last byte.
- A compare write lands in ~15 ms (~800 PWM periods); a link-timed 100 ms
  hold is 93-108 ms. Counted hold (MINOR 8): 500 periods = 10.000 ms.
- Stopping a reply on a valid CRC: a 20-byte prefix passes 1 in 4096.
  Rejected.
- Windows refuses a second open of a held port; every probe then reads
  silent. Ask the session that holds it.
- `close()` disarmed a running stage when a second session asked a question:
  the broker exists for this (open 0.05 s vs 5.85 s).
- RS485 was pumped only with the console in binary mode: main() polled the
  link only then (the emulated limb).
- A byte a microsecond on emulated RS485 (a 10 Mbit adapter onto the app's
  115 200, once passed by Renode) lost one byte a pass. The link
  drains up to LINK_TAKE_MAX a pass, closes a frame at its silence, reads the
  clock once a pass: 200 echoes of 240 B, none lost.

## Gate stage

- Release on a zero triple with MOE up turned the three low sides on: a
  short across the windings. On native 865 rpm stopped in 1 s at 31 A rms;
  from 1 979 rpm the short's first peak tripped the drive; the core's model
  606 -> 60 rpm in 0.5 s at 43 A. Fix: release with CCER's six enables
  clear, OSSR holding each output inactive (bridge open); closed on the first
  triple: 607 -> 589 rpm on the drag's 589, 0 A. The model summed omega in
  float, 2.6 ulp a sub-step at the flywheel's drag, 4.2 % more off a coast;
  compensated.
- 30 ns dead time truncated to 29.5 ns (7 DTG) tripped the supply's OCP.
  Rounding is up: 8 counts = 33.7 ns.
- Two gate stages 15 C hotter than the third: gate pins at CubeMX LOW speed;
  VERY_HIGH since. Found by a 600-sample pin count and a register dump.
- Gate short probe: the neighbour follows within 76 ns; pull-down ~40 k.
- Alternate (op 10) proven: 12 mid-run reads, both triples,
  scope. A zero triple every other period is not half frequency: the pulse
  sits on the underflow, two quarter pulses a period, the same edges.
- Dry, three legs at 50 %, 63 ns dead time, rested, 60 s: NTC rise, K, at 50
  / 25 kHz below; armed without an edge 0.6 K. A 25-50 kHz staircase at 33.8
  V, 60 s a step: 55.7 C, no knee. At 33.7 ns 11.5 K at 24.2 V for 63 ns's
  8.2: 0.66 W through both FETs; 42 ns as 63 ns at 24 and 34 V
  (`tools/bench/dry_heat.py`).

| Link V | 50 kHz | 25 kHz |
| --- | --- | --- |
| 24.2 | 8.2 | 5.1 |
| 33.8 | 9.9 | 5.4-5.9 |
| 43.8 | 13.0 | 6.3 |
| 53.8 | 14.6 | 8.2 |
| 60.8 | 17.7 | 9.2 |

- In the rig's free air at 23.9 V (`dry_heat.py --cool`): 7.35 K at 50 kHz
  and 3.84 at 25, 60 s each; 301 s at 50 kHz 36.65 to 50.97 C, the
  model's still air 12.7 K from idle for its 14.3. Its cooldown 7.04 K over
  55 s and 9.20 over 250 (rms 0.06 K), half its way back in 71 s where the
  model's still air has 140: 5.1 K/W on the board's 49 J/K against 8.33.

- The unmodified board arms with the break in circuit: AFE_ON low, PE15 high
  at once, the latch's low cleared (op 5), MOE. 300 s and twenty 60 s runs,
  no trip; `switch.py` does so.
- STO interlock: Cinj 0.77 V, Clevel 0.06 V against 3 V. The
  keepalive latch holds a few hundred microseconds.
- STO chain modelled from `sto.asc` (`world_sto.c`):
  + Nominal pilot: Clevel settles at 2.86 V, under the interlock's 3.0 V; D10
    clamps the pump at RESET's 0.8 Cinj. It needs 2.0 V now, Q10A releasing
    at 1.55 V. Cinj's 3.70 V exceeds VDDA on PC1 and reads full scale.
  + Clean bus: releases from ~0.9 V at 5 kHz (`sto.asc`'s note: 0.7); TP67
    tops at 0.52 V against U16B's 0.543: no high cutoff (the note: off from
    2.2 V). With its 1.8 V CMNOISE TP67 reaches 0.547 V at 2.2 V for ~1 us,
    under the TLV3492's slew.
- Keepalive starved whenever main() blocked > ~120 us: an ADC burst 1.4-6.9
  ms, the gate probe's settles 348 us, a thermal slice at -O0 236 us. Pumped
  now at each conversion, settle and thermal step; the thermal core at -O2
  as the drive. The identification's shadow step still holds it 130 us every
  few hundred ms: a FAULTOUT glitch the break latches when armed with neither
  bypass.
- Renode's pacing (idle core at 100 MIPS, main() skipped under the drive)
  stretched the keepalive's gaps in virtual time. The emulated chain runs on
  the part's time: 475 MIPS, sleeps whole.

## CPU and memory

- SPI4's kernel is APB2, 118.75 MHz (docs said 100): A1335 at /64 = 1.86 MHz.
- CubeMX enabled neither cache. I-cache off: a drive step 7 400 cycles; at
  -O0 the ISR 10 040 cycles = 21 us > the 20 us period. I-cache on, -O2: 6
  756; with the polynomial sin/cos: 2 922 on target.
- D-cache off: the record is read back through a pointer; .data/.bss in DTCM.
  Sample path from ITCM (~30 K Debug, 27 K Release).
- An edited linker script did not relink until `LINK_DEPENDS` was set.
- ADC3's interrupt (`tools/target/hot.py`, Release): 93
  functions, 4 504 instructions, 436 conditional branches, 69 divides.
  + Outside ITCM, each behind a veneer: the handler, `Board_AdcPhaseSlot`,
    `Board_AdcDifferential`, `Board_AfeOn`, `Board_CalApply` and its 64-bit
    divide (`__udivmoddi4`, 40 branches), `Board_AngleState`, `lrintf`,
    `memset`/`memcpy`, the virtual plant's `tanhf`, libm's errno path. The
    UARTs interrupt a byte.
  + Cut: those into ITCM; calibration a laid Q28 scale (SMULL and a shift);
    rounding by `vrintr`; `drive_tanh` a clamped rational. Result: 74
    functions, 358 branches, 53 divides; on Renode the virtual plant's ISR 1
    728 -> 1 682 instructions, its advance 320 -> 274.
  + newlib-nano's `memset`/`memcpy` a byte a turn; the board's own a word a
    turn in ITCM (`board_mem.c`).
  + Second pass: `TIM1_UP_IRQn` enabled once in `Board_PwmInit` with `UIE`
    clear on `TIM1->DIER` so `update_irq` toggles `DIER` alone; `ADC3_IRQHandler`
    returns on `Board_SyncIrq` without `HAL_ADC_IRQHandler` and its 8 callbacks;
    `Board_DaqOnInjected` reads `&s.daq` directly; `drive_sincos` and
    `drive_wrap` fold on `vrintm`/`vminnm`/`vmaxnm`/`vabs` (0 branches, 0
    divides); `step_flux` multiplies by `1 + j wc / speed` without `sqrtf`,
    `drive_atan2` or `drive_sincos`; `Board_DriveOnSample` caches `s.arr` and
    rounds on `vrintr`. Result: `ADC3_IRQHandler` 76 -> 60 functions, 4 204 ->
    3 655 instructions, 396 -> 305 branches, 54 -> 47 divides, outside ITCM 0;
    `TIM1_UP_IRQHandler` 8 -> 4 functions, 201 -> 171 instructions, 23 -> 21
    branches, 0 divides, outside ITCM 0.
  + Ranked remaining in `ADC3_IRQHandler` (unavoidable): `drive_step` (884 ins,
    61 br, 9 div: mode dispatch, trip checks, `drive_atan2` octant folds, SVM
    overmodulation scale), `push_record`/`daq_feed` (249/95 ins, 33/16 br, 8/0
    div: ring/ladder watermarks and pin duty division at window close),
    `Board_DriveOnSample` (163 ins, 13 br, 0 div: source select, trip vs commit,
    log guard), `drive_observer_step` (209 ins, 10 br, 6 div: flux norms and
    `drive_atan2`), `current_loop` (107 ins, 10 br, 1 div: `vsqrt` circle room
    and anti-windup).

## IMU (BNO085)

Six firmware defects, no hardware fault:

| Symptom | Cause |
| --- | --- |
| CS never moved | configured before `HAL_SPI_DeInit`, whose MSP reset the pin |
| every read `FF FF FF FF` | CS released between header and cargo |
| reads refused after reset | advertisement 276 B, buffer 64 |
| 60 ms interval never reported | interval sent little-endian, wire big-endian |
| write works twice, fails 3rd | gated on an INTN an awake part never asserts |

- Reset then Set Feature: 0 rotation vectors; feature alone 49/s.
  `Board_ImuWrite` drains queued announcements first.
- Wake answers in < 1 ms, 2 in 10 not at all; re-asserting WAKE recovers.
- After an AFE cycle the loop ran with no reports for 15 s; the feature set
  0.5 s later always worked (not a settle time).
- A zero byte after the last report is padding (46 "frame errors" in 30 s).

## A1335

- R/W bit: read is 0 (datasheet silent). Two frames per read; one returns
  the previous register.
- A read on demand returned the register asked by the read before, CRC good,
  at any gap from a few instructions to 200 us. Effect: the observer's AFE
  die read ANG's twelve bits as -241.77 C, its AFE node -196 C, its legs
  80-250 C; the envelope throttled at rest. Cause: a reply is two packets
  behind its request (the manual: one). Fix: a read asks four times; the poll
  drops two replies behind another register's; a die's reply carries TSEN's
  identifier. Result: 10 of 10 replies their own; die 34.59 C beside an NTC
  of 34.41. The emulated parts answer two packets on; the old driver reads
  -273.15 C there.
- TSEN is the die, reset whenever AFE_ON breaks; 0.125 K steps. FIELD ~2 G
  with no magnet.
- The poll read the part back to back whenever AFE_ON was up: 8 346 reads a
  second on native, 3 575 on Renode, where the board then ran at 2.2-3.5 wall
  s a virtual s (1.34 with the loop held) and the attitude page's tumble at
  0.4 of real time. With the drive and the DAQ off a read every 1 ms
  (`ANGLE_IDLE_HZ`; the part refreshes every 32 us), back to back under
  either: Renode 1.48, the page's reads 16 -> 12 ms, its vectors 52 -> 78 a
  second; native 993 reads a second, hold and resume as before.

## DAQ and clocks

- Reader thread: 84.4-134.6 rec/s with 4 ms of work a block.
- A 16 K ring overflowed in 6 s of a stalled terminal; the ring is in the AXI
  SRAM (HARDWARE.md).
- A record's stamp is 32 bits of cycles, round every 9.04 s at 475 MHz: a
  recording across the wrap failed the time-order check on CI;
  order is the modular difference.
- `interval_us` 0 with `records` 0 took the link down: refused.
- A block read after a CYCCNT wrap came back 9.02 s in the past: stamps are
  unwrapped across blocks. CYCCNT wraps every 9.04 s at 475 MHz.
- A Windows clock "in sync" can sit most of a second out; not a reference.

## Calibration

- DC link spanned against a DMM: 31.04 read, 30.05 true, -32 418
  ppm ch 5.
- Phase gain from the schematic, not spanned.
- LTspice's AFE (amplifiers.asc 6bc736b, nominal, 33 samples to 106 A): 10.24
  mV/A, zero 8.7 mV at the ADC's differential input. The record's shunt x
  THS4551 gain is 15.9 mV/A, the schematic's own note 9.2 mV/A and 110 mV:
  unspanned, a phase reads 0.64 of its current.
- Unspanned, an emulated drive put 1.55 times its command through the plant:
  the world's legs 170-300 C under an observer's 100; unzeroed, the 8.7 mV
  zero is 0.85 A. An emulated MCU is spanned on the repl's 10.238 mV/A and
  zeroed at open: the world carries the regulated current within 0.2 %.
- An id added without moving `BOARD_CAL_PARAM_COUNT` is held but never
  reported.
- Replies past 253 B page (ADC table, pins, parts).
- `BOARD_CAL_PARAM_COUNT` stayed 46 after the winding's ids 46-48
  (CAL_VERSION 12): settable, never reported by cal op 8. Now 49.

## Bootloader

- On the board, the ST-Link's port: Debug 201 544 B, 900 chunks
  20.6 s, verify 0.28 s, seal with persist 2.6 s (2 sectors, 6 300 words),
  `go` to the application's answer 0.13 s; `open()` on another image 19.7 s.
  From D2 SRAM: conformance 110/110, 17 719 angle updates/s, 106 % of
  flash's.
- Flashed over SWD, a warm board ran RAM's old image: the slot named it.
  `build_and_flash.py` clears the slot.
- The host streamed chunks 2 ms after each write; the probe's port shifts a
  232 B frame out in 20.1 ms: frames ran together, 0 of 634 held. A
  broadcast's settle counts from the frame's last bit (`Transport.on_wire`);
  Renode's adapter spaced them itself.
- After `go` the application wakes as a console on the ST-Link's port:
  `from_bootloader` polled binary for 5 s. It hands over first.
- Unassigned, the bootloader handed over unit 247, the blank nodes': the
  application answered there and `open()` took it for a bootloader; a warm
  reset renamed an assigned node. `boot_hand_over`: this run's assign, a warm
  slot's own, else unit 0 and the application's own.
- VOS1 was set with PWR_CR3 unwritten (RM0433 6.8.4: once after power-on,
  before VOS or the clock). Written now, the LDO; unproven - the bench read
  0x42, locked since power-on.
- On a bus the host looked for a blank node, and polled the application
  after `go`, at one rate: the bootloader listens at 10 Mbit, the
  application at its record's `link_baud`. Renode's rate-blind lines and a
  limb adapter apart from the host's port hid it. The port moves to 10 Mbit
  for the bootloader and back for the application; the emulated adapter
  follows the port.

## Subjects

| Subject | File |
| --- | --- |
| A woman's walk as the metric: the band, walks off it, the take | [findings/normal.md](findings/normal.md) |
| Feet: passive toes, the sole's give, the push-off | [findings/feet.md](findings/feet.md) |
| Her eyes: the stereo, the map she keeps, ways round its things | [findings/sight.md](findings/sight.md) |
| Kinematics: the figure's recipes, dual quaternions, the seams | [findings/kinematics.md](findings/kinematics.md) |
| The whole-body stack: its solver, standing, shoves, the loop in C | [findings/wbc.md](findings/wbc.md) |
| Leap on the stack: which soles stand from one pass, a hop stood | [findings/leap.md](findings/leap.md) |
| Orthokinematics, the one way she moves: text, a latent loop, the stride, the walk | [findings/orthokinematics.md](findings/orthokinematics.md) |
| The whole body's step without an active set: 21 unknowns, four solves | [findings/wrench.md](findings/wrench.md) |
| What her law senses: edge, estimate, insoles | [findings/sensing.md](findings/sensing.md) |
| Her body identified | [findings/identification.md](findings/identification.md) |
| Her scenarios as a player commands them | [findings/scenarios.md](findings/scenarios.md) |
| Her errands: her way to a chair, a bed, the door, her dock, its stunt | [findings/errands.md](findings/errands.md) |
| The stairs told her: footholds, her height over the incline, the climb | [findings/stairs.md](findings/stairs.md) |
| Her place: the glazed mountain lodge, its scene, its drawing | [findings/lodge.md](findings/lodge.md) |
| The mocap takes as her scenarios, each judged against its take | [findings/stunts.md](findings/stunts.md) |
| The takes as a sequencer's score: beats, voices, a gait's pattern | [findings/sequencer.md](findings/sequencer.md) |
| The autotuner on her texts, offline and at runtime: reach, step, her loop linked | [findings/autotuner.md](findings/autotuner.md) |
| Support, whatever bears her: a seat, a bed, a knee on the floor | [findings/support.md](findings/support.md) |
| The board's thermal: the network, the camera, the observer, the envelope | [findings/thermal.md](findings/thermal.md) |
| Her floor's grip: felt as a sole slides, told, her law bounded by it | [findings/grip.md](findings/grip.md) |
| Her situation read from her sensors | [findings/situation.md](findings/situation.md) |
| Her jerk through transitions against her steady walk, and its sources | [findings/jerk.md](findings/jerk.md) |
| Build, buses, falls, drawing, clothes | [findings/body.md](findings/body.md) |
| Her pace against real time: MuJoCo's step, her law in C, the page's tiers | [findings/pace.md](findings/pace.md) |
| Drives: motors, gearboxes, inverters, sizing | [findings/drives.md](findings/drives.md) |
| Each drive's demand on her loop, the chair's stunts against it | [findings/demand.md](findings/demand.md) |
| One stack for every drive: demand, candidates, stacks as built | [findings/stacks.md](findings/stacks.md) |
| The drive and its observers: stand-in, emulator, native://, rotor pages | [findings/drive.md](findings/drive.md) |
| QUAD: frame, law, routine, course, tuner | [findings/quad.md](findings/quad.md) |
| The air every craft flies: its kinds, gusts, eddies | [findings/air.md](findings/air.md) |
| The helicopter, the ornithopter: governed, geared | [findings/aircraft.md](findings/aircraft.md) |
| The aerobat: its wing, its propeller, its routine, its field | [findings/aerobat.md](findings/aerobat.md) |
| The envelope in flight: a climb, a hover, what a board may carry | [findings/envelope.md](findings/envelope.md) |
| Flown from compiled streams: the core, the rotors' reach, upsets, the power loop | [findings/streams.md](findings/streams.md) |
| Renode, native:// and CI: the emulated image, peripherals, speed | [findings/emulation.md](findings/emulation.md) |
| Host pages, tooling, suites, the local model's runner | [findings/host.md](findings/host.md) |
| Her law compiled from her numpy: numpy's orders, her parts, their time and memory | [findings/compiler.md](findings/compiler.md) |

## Ruled Out

Settled; not to be investigated again.

### PCSEL accumulation explains the Phase V offset

No. PCSEL accumulation is real (0xC03) and every path clears it; it is not
the Phase V offset.

### The NTC channel is anomalous

No, it is quiet. The 15 nF node capacitor supplies the sample-and-hold
charge; 1.5-cycle sampling is ruled out on the quiet channels. Not ruled out
for Cinj/Clevel.

### The rest

- Hot gate stages: pin speed, not hardware.
- JTAG `Unable to get core ID`: probe firmware; cabling proven.
- BNO085 needing longer after power-up: no; the feature set by hand works.
- BNO085 hardware hypotheses: all four failed; six firmware causes.
- MISO held on the IMU bus: the check's own CS floating low.
- Board halted with two sessions: the port was opened twice.
- H_INTN never asserting: it does; 15 ms round trips missed it.
