# Findings

Measured on the bench and settled, one entry each, dated. The record to
2026-09-23: `git show 430b91f:docs/FINDINGS.md`. Subjects off the board: the
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
  every board re-zeroed (2026-10-05).
- Self test's image CRC over 0x08000000-`_etext`: bus fault at 0x08200000
  since the image runs from D2 SRAM; taken from the vector table now. Its
  PCSEL count included the injected group's rank 2; the group arms on a
  cleared PCSEL and counts its three (2026-10-05).
- HAL only ORs PCSEL: ADC3 PCSEL read 0xC03, four channels live. Every read
  path clears it (invariant 6).
- Phase noise floor, AFE on: 0.35-0.41 A rms per phase.
- A suite borrowing the AFE rail flips another suite's AFE row, ~1 run in 3.
- MCU die through `__LL_ADC_CALC_TEMPERATURE`: whole degrees, under an
  identification with a 0.1 K floor. Factory points in float: 0.01 K
  (2026-09-26).

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
- `close()` disarmed a running stage when a second session asked a question
  (2026-08-29): the broker exists for this (open 0.05 s vs 5.85 s).
- RS485 was pumped only with the console in binary mode: main() polled the
  link only then (the emulated limb, 2026-09-25).
- A byte a microsecond on emulated RS485 (a 10 Mbit adapter onto the app's
  115 200, passed by Renode until 2026-09-28) lost one byte a pass. The link
  drains up to LINK_TAKE_MAX a pass, closes a frame at its silence, reads the
  clock once a pass: 200 echoes of 240 B, none lost (2026-09-25).

## Gate stage

- Release on a zero triple with MOE up turned the three low sides on: a
  short across the windings. On native 865 rpm stopped in 1 s at 31 A rms;
  from 1 979 rpm the short's first peak tripped the drive; the core's model
  606 -> 60 rpm in 0.5 s at 43 A. Fix: release with CCER's six enables
  clear, OSSR holding each output inactive (bridge open); closed on the first
  triple: 607 -> 589 rpm on the drag's 589, 0 A. The model summed omega in
  float, 2.6 ulp a sub-step at the flywheel's drag, 4.2 % more off a coast;
  compensated (2026-09-28).
- 30 ns dead time truncated to 29.5 ns (7 DTG) tripped the supply's OCP
  (2026-08-29). Rounding is up: 8 counts = 33.7 ns.
- Two gate stages 15 C hotter than the third: gate pins at CubeMX LOW speed;
  VERY_HIGH since. Found by a 600-sample pin count and a register dump.
- Gate short probe: the neighbour follows within 76 ns; pull-down ~40 k.
- Alternate (op 10) proven 2026-08-30: 12 mid-run reads, both triples,
  scope. A zero triple every other period is not half frequency: the pulse
  sits on the underflow, two quarter pulses a period, the same edges
  (2026-10-05).
- Dry, three legs at 50 %, 63 ns dead time, rested, 60 s: NTC rise, K, at 50
  / 25 kHz below; armed without an edge 0.6 K. A 25-50 kHz staircase at 33.8
  V, 60 s a step: 55.7 C, no knee. At 33.7 ns 11.5 K at 24.2 V for 63 ns's
  8.2: 0.66 W through both FETs; 42 ns as 63 ns at 24 and 34 V
  (`tools/bench/dry_heat.py`, 2026-10-05).

| Link V | 50 kHz | 25 kHz |
| --- | --- | --- |
| 24.2 | 8.2 | 5.1 |
| 33.8 | 9.9 | 5.4-5.9 |
| 43.8 | 13.0 | 6.3 |
| 53.8 | 14.6 | 8.2 |
| 60.8 | 17.7 | 9.2 |

- The unmodified board arms with the break in circuit: AFE_ON low, PE15 high
  at once, the latch's low cleared (op 5), MOE. 300 s and twenty 60 s runs,
  no trip; `switch.py` does so (2026-10-05).
- STO interlock: Cinj 0.77 V, Clevel 0.06 V against 3 V (2026-08-27). The
  keepalive latch holds a few hundred microseconds.
- STO chain modelled from `sto.asc` (`world_sto.c`), 2026-09-27:
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
  bypass (2026-09-27).
- Renode's pacing (idle core at 100 MIPS, main() skipped under the drive)
  stretched the keepalive's gaps in virtual time. The emulated chain runs on
  the part's time: 475 MIPS, sleeps whole (2026-09-27).

## CPU and memory

- SPI4's kernel is APB2, 118.75 MHz (docs said 100): A1335 at /64 = 1.86 MHz.
- CubeMX enabled neither cache. I-cache off: a drive step 7 400 cycles; at
  -O0 the ISR 10 040 cycles = 21 us > the 20 us period. I-cache on, -O2: 6
  756; with the polynomial sin/cos: 2 922 on target.
- D-cache off: the record is read back through a pointer; .data/.bss in DTCM.
  Sample path from ITCM (~30 K Debug, 27 K Release).
- An edited linker script did not relink until `LINK_DEPENDS` was set.
- ADC3's interrupt (`tools/target/hot.py`, Release, 2026-10-10): 93
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
    turn in ITCM (`board_mem.c`). Left outside ITCM: a mode change's two NVIC
    calls.

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
  -273.15 C there (2026-10-05).
- TSEN is the die, reset whenever AFE_ON breaks; 0.125 K steps. FIELD ~2 G
  with no magnet.

## Thermal

- Dry at 50 kHz, converters calibrated, room ~22 C (2026-10-05):
  + 300 s at 33.8 V: NTC 33.1 -> 54.5 C, MCU die +14.3 K, A1335 +12.3.
    150 s at 43.8 V: NTC +21.4, the dies +12.9 and +11.0. The network had
    the NTC at +4.2 and +3.6.
  + Fitted to both cooldowns (NTC rms 0.3-0.5 K, dies 0.4-1.4): a leg's dry
    watts from the camera's dump (1.20 W at 24.6 V for three, all in the
    legs) plus 0.40 W of gate drive, 3.4 and 4.4 W; the NTC 0.56 of the V
    patch over the centre, no lag of its own; laminate 21 J/K (the 49 of a
    25 min step is its parts'); in-plane graph unchanged.
  + Checks: the camera's switching state +19.0 K at the NTC for its 18.9;
    the 60 s point 9.55 for 9.75. The law runs 11 % over at 53.8 V, 7 % at
    60.8.
- The MCU die reads its package: 46.0 C awake where the camera has 47; 4.0 K
  lower asleep (0.49 W). With the uncalibrated converter's 40.5 K/W the
  observer held the laminate at 2 C, the legs at 91-104, the room at 7-33 C
  on a board at 33; now the NTC 33.2 for 33.5, the room 21.6 (2026-10-05).
- Refit on the board, record laminate 21 J/K, dry and blind (2026-10-05):
  120 s at 53.8 V observer NTC 57.85 C for the thermistor's 57.90 (rise 25.4
  K for 24.3); 300 s at 23.9 V 51.9 for 50.1 (19.1 for 17.4). Thirty seconds
  on it runs 4 K over, for the anchor to take. Its room: 35-43 C in a room of
  22 on that record, 21.6 on the default 49 J/K. Idle two hours, AFE_ON off
  but for reads (2026-10-06): UNCERTAIN, the air's scale at its floor 0.25,
  the capacity's 2.09 (44 J/K; the transient's 49), room 33.4 C, the board's
  own 33.
- The stand-in's load is its balanced `load_cycle`: 569ae47's turning vector
  walked the heat U, V, W, the hottest leg 9 times in 16 s (2026-09-28).
- Tour on an emulated board's world (`coaxial.model.rooms`): thermal page on
  native STABLE at 244 s, the room on at 254, UNCERTAIN 259, STABLE again 351
  (2026-09-28).
- Rotor demo on native: 0.85 of the span on the stand-in's model alone; on
  the world (`emu_world_drag`) 0.93-0.94, the clamp at 0.61-0.75, no trip
  (2026-09-28).
- Network against the camera states, stand-in truth: worst miss 9.0 K, 20.1
  K before the emulator (regulators +28 for +8, AFE on read as passive).
  Open: regulators 3-9 K hot, bridge 4-6 K cool (2026-09-28).
- Camera 2026-08-28, 20 C room, four states x 25 min (tau 6.8 min): board and
  rises (MCU / regulators / bridge / AFE) passive 30.0 / +15.0 / +8.0 / +1.0
  / +1.0. NTC - TSEN: -0.74 C idle, +10.94 C switching.
- Measured: board 8.33 K/W to ambient (one passive point, 1.2 W), 49 J/K from
  a transient. Assumed: per-leg `to_board` 45.6 K/W (one camera zone x3),
  part capacities, NTC fraction 0.30 (pick-place geometry, R a floor).
- IAUCN10S7N021 datasheet: Rth JC 0.69 K/W (6.2 K at 100 A), Rth JA 25.9
  K/W, Rds(on) 1.8 typ / 2.1 max mOhm (the model books typ, -17 % worst
  case).
- SOA to that sheet (Rev 1.2, 2026-10-09):
  + A leg is judged on its FETs' junction: its node plus each FET's half of
    its watts through R_th,JC, Z_th at 100 ms (the envelope's slice) 0.68,
    against 175 C. Its node against 125 C left 44 K unused.
  + Rds(on) second order on Fig. 8: 5.55e-3/K and 1.78e-5/K^2, within 1.2 %
    from -55 to 175 C; the 0.78 %/K was 5.7 % off.
  + The 2EDL8034 sits 8-13 mm from its FETs on the leg's patch; the patch's
    105 C laminate holds it under its own 125.
  + Current is no ceiling: 175 A DC, 779 A for 100 us, against the board's
    100.
  + QUAD's course on four boards: 0.65-0.80 of their envelopes and the pull
    cut to 21-58 % became 0.48-0.58 and none cut; laps 14.8 and 14.3 s for
    15.3 and 15.2; in a stuffy room 17.9 and 17.0, where the laminate binds
    next. A board's stored record keeps its own ceilings.
- Applications (2026-10-09): what a board is mounted in, thermal op 15 into
  the record (CAL_VERSION 16), laid over the still-air network by
  `thermal_app.c`, ballpark until a test cycle runs in each. At 3 W a leg,
  1.5 W its shunts, 20 W the winding, the legs settle at:

| Application | Legs, C |
| --- | --- |
| still air | 184 |
| a hover's wash, 1 470 rpm | 115 |
| sealed finned housing | 74 |
| a fan's sink | 53 |
| cold plate | 37 |
| PAO | 76 |
| transformer oil | 80 |

- A record load or defaults lays the observer's network anew; it ran the old
  one until a boot.
- A world mounts its board (2026-10-09): its JSON names its link and its
  application, laid on the plant's heat and told to the board as the rig
  opens it (board/emu/worlds/dyno.json, 63 V on a fan's sink). The rotor
  observer's demo there:
  + still air, 63 V: switches at their ceiling, the clamp throttled to
    nothing in five segments of seven;
  + on the sink, 50 A: 0.24-0.60 of their span, never throttled;
  + 90 A: the observer's crossover rose to 129 rad/s, over DYNO's spool on
    the 10 A under it;
  + 70 A: 0.91 in SPIN, 0.81 in DYNO (the board 0.94), throttled to 0.57;
    the stand-in 0.93 and 0.95, to 0.42.
- The envelope must step and evaluate per 100 ms slice: a 1 s step let the
  driver node reach 178 C before the clamp saw it. Catch-up capped at 2 s.
- A rail another had just raised was read without a borrow's settle, and a
  reading without the reference is mid-scale: a 25.00 C thermistor and a 545
  C die, which the anchor took. Read after the settle now, never without the
  reference (2026-09-26).
- Squaring one synced sample per step aliased; `Board_SyncMeanSquare`
  accumulates per leg in counts. Its slope was taken over one code, where the
  record's integer gain trim truncates to nothing: the observer's I^2 never
  saw a channel's gain. Over 32 768 codes now (2026-09-26).
- The stand-in's losses were `phase_power`'s (no duty, link, dead time,
  tempco); now `thermal_power_estimate`'s, mirrored in
  `coaxial.model.thermal` and held to the C. Its held vector turned the
  current's size, not the phases: one leg heated (2026-09-26).
- With the tempco a leg has no equilibrium past 28 K/W x P x 0.78 %/K = 1: 60
  A rms held ran one away before the winding warmed. The stand-in's setters
  wrote its live network and the identification's next update undid them;
  they write the base, the scales re-applied (2026-09-26).
- Steady state at 5.30 mOhm: continuous 19.1 A against a 105 C laminate, 22.0
  A against a 125 C junction. Throttle at 90 % of span (2026-09-05).
- Identification (host, against a ground truth): air path box 2.0 -> 1.93,
  fan 0.5 -> 0.50 in three cycles, innovation 0.05 K. Spread and NTC share
  are unobservable from a cooldown and held.

## DAQ and clocks

- Reader thread: 84.4-134.6 rec/s with 4 ms of work a block.
- A 16 K ring overflowed in 6 s of a stalled terminal; the ring is 448 KB AXI
  SRAM.
- `interval_us` 0 with `records` 0 took the link down: refused.
- A block read after a CYCCNT wrap came back 9.02 s in the past: stamps are
  unwrapped across blocks. CYCCNT wraps every 9.04 s at 475 MHz.
- A Windows clock "in sync" can sit most of a second out; not a reference.

## Calibration

- DC link spanned against a DMM 2026-08-30: 31.04 read, 30.05 true, -32 418
  ppm ch 5.
- Phase gain from the schematic (2026-08-26), not spanned.
- LTspice's AFE (amplifiers.asc 6bc736b, nominal, 33 samples to 106 A): 10.24
  mV/A, zero 8.7 mV at the ADC's differential input. The record's shunt x
  THS4551 gain is 15.9 mV/A, the schematic's own note 9.2 mV/A and 110 mV:
  unspanned, a phase reads 0.64 of its current (2026-09-25).
- Unspanned, an emulated drive put 1.55 times its command through the plant:
  the world's legs 170-300 C under an observer's 100; unzeroed, the 8.7 mV
  zero is 0.85 A. An emulated MCU is spanned on the repl's 10.238 mV/A and
  zeroed at open: the world carries the regulated current within 0.2 %
  (2026-09-26).
- An id added without moving `BOARD_CAL_PARAM_COUNT` is held but never
  reported.
- Replies past 253 B page (ADC table, pins, parts).
- `BOARD_CAL_PARAM_COUNT` stayed 46 after the winding's ids 46-48
  (CAL_VERSION 12): settable, never reported by cal op 8. Now 49
  (2026-09-23).

## Bootloader

- On the board, the ST-Link's port (2026-10-05): Debug 201 544 B, 900 chunks
  20.6 s, verify 0.28 s, seal with persist 2.6 s (2 sectors, 6 300 words),
  `go` to the application's answer 0.13 s; `open()` on another image 19.7 s.
  From D2 SRAM: conformance 110/110, 17 719 angle updates/s, 106 % of
  flash's.
- Flashed over SWD, a warm board ran RAM's old image: the slot named it.
  `build_and_flash.py` clears the slot (2026-10-05).
- The host streamed chunks 2 ms after each write; the probe's port shifts a
  232 B frame out in 20.1 ms: frames ran together, 0 of 634 held. A
  broadcast's settle counts from the frame's last bit (`Transport.on_wire`);
  Renode's adapter spaced them itself (2026-10-05).
- After `go` the application wakes as a console on the ST-Link's port:
  `from_bootloader` polled binary for 5 s. It hands over first (2026-10-05).
- Unassigned, the bootloader handed over unit 247, the blank nodes': the
  application answered there and `open()` took it for a bootloader; a warm
  reset renamed an assigned node. `boot_hand_over`: this run's assign, a warm
  slot's own, else unit 0 and the application's own (2026-10-05).
- VOS1 was set with PWR_CR3 unwritten (RM0433 6.8.4: once after power-on,
  before VOS or the clock). Written now, the LDO; unproven - the bench read
  0x42, locked since power-on (2026-10-05).
- On a bus the host looked for a blank node, and polled the application
  after `go`, at one rate: the bootloader listens at 10 Mbit, the
  application at its record's `link_baud`. Renode's rate-blind lines and a
  limb adapter apart from the host's port hid it. The port moves to 10 Mbit
  for the bootloader and back for the application; the emulated adapter
  follows the port (2026-09-28).

## Local model

- Wrote "Mid-scale ... 25.00 C" from the warning text when refusing was
  tried.
- Invented a coaxial cable twice; guessed `ch=['phA']`, BUS_VOLT, A0; sent
  the left knee (asked in Swedish) as `right knee`.
- gemma4:12b answered a measurement with HARDWARE.md's table; denied being
  able to flash with the tool listed.
- qwen2.5:14b switched the AFE on four times in one turn; answered in
  Chinese, Japanese and Thai.
- A search hit without its chapter was reported as the explanation when it
  sat under Ruled Out: `find` reports the chapter.

## Subjects

| Subject | File |
| --- | --- |
| Walk: plan, shape, look, start from the squat | [findings/walk.md](findings/walk.md) |
| Run: a bounce a foot, its demand on the drives, landings | [findings/run.md](findings/run.md) |
| One going law: stand, walk and run as setpoints; its rows | [findings/going.md](findings/going.md) |
| A woman's walk as the metric: the band, walks off it, the take | [findings/normal.md](findings/normal.md) |
| Feet: passive toes, the sole's give, the push-off | [findings/feet.md](findings/feet.md) |
| Kinematics: the figure's recipes, dual quaternions, the jeans' seams | [findings/kinematics.md](findings/kinematics.md) |
| Balance: the capture law, the side step, floor events, shoves, the scoreboard | [findings/balance.md](findings/balance.md) |
| Standing: the rigs, the one law in the capture point's plane, the push polar | [findings/standing.md](findings/standing.md) |
| Balance as one convex QP: the stack, its solver, the MPC, the step | [findings/wbc.md](findings/wbc.md) |
| Build, buses, falls, get-up, drawing, clothes | [findings/body.md](findings/body.md) |
| Drives: motors, gearboxes, inverters, sizing | [findings/drives.md](findings/drives.md) |
| One stack for every drive: demand, candidates, stacks as built | [findings/stacks.md](findings/stacks.md) |
| The drive and its observers: stand-in, emulator, native://, rotor pages | [findings/drive.md](findings/drive.md) |
| QUAD: frame, air, law, routine, course, tuner | [findings/quad.md](findings/quad.md) |
| Renode, native:// and CI: the emulated image, peripherals, speed | [findings/emulation.md](findings/emulation.md) |
| Host pages, tooling, suites, the local model's runner | [findings/host.md](findings/host.md) |

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
