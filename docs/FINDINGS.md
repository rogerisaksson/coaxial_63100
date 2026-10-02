# Findings

What was measured on this bench and what it settled. One line each. The
long-form record to 2026-09-23 is `git show 430b91f:docs/FINDINGS.md`.

## AFE and ADC

- AFE_ON (PB2) powers the ADC reference: off, every channel reads exact
  mid-scale and the NTC exactly 25.00 C. It also powers the BNO085 and A1335.
- PE15 follows AFE_ON inversely; reads as a fault with the AFE on. Cause open.
- ADC offset calibration runs with AFE_ON low: offsets vary ~100 mV boot to
  boot.
- HAL only ORs PCSEL: ADC3 PCSEL read 0xC03 (four channels live). Every read
  path clears it (invariant 6).
- Phase noise floor, AFE on: 0.35-0.41 A rms per phase.
- A suite borrowing the AFE rail flips another suite's AFE row (~1 run in 3).
- The MCU die through `__LL_ADC_CALC_TEMPERATURE` read whole degrees under an
  identification with a 0.1 K floor; the factory points in float, 0.01 K
  (2026-09-26).

## Link

- IMU cargo (276 B at 1.48 MHz = 1.5 ms) blocking the poll lost 0.45 % of
  USART3 frames (7 of 1393). Fixed by not polling mid-frame.
- Serving a 229-byte DAQ reply (19.9 ms) cut acquisition 477 -> 133 rec/s.
- CubeMX left USART2/UART5 at 9 216 000 baud; baud is in the record since
  CAL_VERSION 9.
- Write-class transaction was 46.6 ms: t3.5 1.75 ms, `serial.timeout`
  assignment 3.25 ms each (x3), 20 ms QUIET_TIME. Now ~13 ms; sized ACK
  replies stop on the last byte.
- A compare write lands in ~15 ms (~800 PWM periods); a link-timed 100 ms
  hold is 93-108 ms. Counted hold (MINOR 8): 500 periods = 10.000 ms.
- Stopping a reply on a valid CRC: a 20-byte prefix passes 1 in 4096. Rejected.
- Windows refuses a second open of a held port; every probe then reads
  silent. Ask the session that holds it.
- `close()` disarmed a running stage when a second session asked a question
  (2026-08-29): the broker exists for this (open 0.05 s vs 5.85 s).
- RS485 was never pumped unless the console was in binary mode: main() polled
  the link only then (found on the emulated limb, 2026-09-25).
- A byte a microsecond on RS485, emulated (a 10 Mbit adapter onto the app's
  115 200, which Renode passed until 2026-09-28): one byte a pass lost. The
  link drains up to LINK_TAKE_MAX a pass, a frame closed at its silence, one
  clock read a pass; 200 echoes of 240 B, none lost (2026-09-25).

## Gate stage

- The drive let go on a zero triple with MOE up: the three low sides on, a
  short across the windings. On native 865 rpm stopped in a second at 31 A rms,
  and from 1 979 rpm the short's first peak tripped the drive; the core's
  model 606 -> 60 rpm in 0.5 s at 43 A. It lets go with CCER's six enables
  clear now, OSSR holding each output inactive - the bridge open - and closes
  on its first triple in force: 607 -> 589 rpm on the drag's 589, 0 A. The
  model summed omega in float, 2.6 ulp a sub-step at the flywheel's drag, and
  took 4.2 % more off a coast; compensated now (2026-09-28).
- 30 ns dead time truncated to 29.5 ns (7 DTG) tripped the supply's OCP
  (2026-08-29). Rounding is up: 8 counts = 33.7 ns.
- Two gate stages 15 C hotter than the third: gate pins at CubeMX LOW speed.
  VERY_HIGH since. Found by a 600-sample pin count and a register dump.
- Gate short probe: neighbour follows within 76 ns; pull-down ~40 k.
- Alternate (op 10) proven 2026-08-30: 12 mid-run reads, both triples, scope.
- STO interlock: Cinj 0.77 V, Clevel 0.06 V against 3 V (2026-08-27). The
  keepalive latch holds a few hundred microseconds.
- The STO chain modelled from `sto.asc` (`world_sto.c`): on the nominal
  pilot Clevel settles at 2.86 V, under the interlock's 3.0 V - D10 clamps
  the pump at RESET's 0.8 Cinj; it wants 2.0 V now, Q10A releasing at
  1.55 V. Cinj's 3.70 V is over VDDA on PC1 and reads full scale. On a clean
  bus it releases from ~0.9 V at 5 kHz (`sto.asc`'s note: 0.7) and TP67 tops
  at 0.52 V against U16B's 0.543, so no high cutoff (the note: off from 2.2
  V); with its 1.8 V CMNOISE TP67 reaches 0.547 V at 2.2 V for ~1 us, under
  the TLV3492's slew (2026-09-27).
- The keepalive starved whenever main() blocked past ~120 us: an ADC burst
  1.4-6.9 ms, the gate probe's settles 348 us, a thermal slice at -O0 236 us.
  Pumped at each conversion, settle and thermal step now, the thermal core
  at -O2 as the drive's; the identification's shadow step still holds it
  130 us every few hundred ms, a FAULTOUT glitch the break latches when armed
  with neither bypass (2026-09-27).
- Renode's pacing (the idle core at 100 MIPS, main() skipped under the drive)
  stretched the keepalive's gaps in virtual time: the emulated chain runs on
  the part's time, its instructions at 475 MIPS and its sleeps whole
  (2026-09-27).

## CPU and memory

- SPI4's kernel is APB2, 118.75 MHz (docs said 100): A1335 at /64 = 1.86 MHz.
- CubeMX enabled neither cache. I-cache off: a drive step 7 400 cycles; at
  -O0 the ISR was 10 040 cycles = 21 us > 20 us period. I-cache on + -O2:
  6 756; with the polynomial sin/cos: 2 922 on target.
- D-cache stays off: the record is read back through a pointer; .data/.bss
  are in DTCM. Sample path runs from ITCM (~30 K Debug, 27 K Release).
- An edited linker script did not relink until `LINK_DEPENDS` was set.

## IMU (BNO085)

- Six firmware defects, no hardware fault:

| Symptom | Cause |
| --- | --- |
| CS never moved | configured before `HAL_SPI_DeInit`, whose MSP reset the pin |
| every read `FF FF FF FF` | CS released between header and cargo |
| reads refused after reset | advertisement 276 B, buffer 64 |
| 60 ms interval never reported | interval sent little-endian, wire big-endian |
| write works twice, fails 3rd | gated on an INTN an awake part never asserts |

- Reset then Set Feature gave 0 rotation vectors; feature alone 49/s.
  `Board_ImuWrite` drains queued announcements first.
- Wake answers in < 1 ms, but 2 in 10 not at all; re-asserting WAKE recovers.
- After an AFE cycle the loop ran but no reports for 15 s; setting the feature
  0.5 s later always worked (not a settle time).
- A zero byte after the last report is padding (46 "frame errors" in 30 s).

## A1335

- R/W bit: read is 0 (datasheet silent). Two frames per read; one returns the
  previous register.
- TSEN is the die, reset whenever AFE_ON breaks; 0.125 K steps. FIELD ~2 G
  with no magnet.

## Thermal

- The stand-in's load is its balanced `load_cycle`: 569ae47's turning vector
  walked the heat U, V, W, the hottest leg 9 times in 16 s (2026-09-28).
- The tour on an emulated board's world (coaxial.model.rooms): the thermal
  page on native STABLE at 244 s, the room on at 254, UNCERTAIN 259, STABLE
  again 351 (2026-09-28).
- The rotor demo on native: 0.85 of the span, its loads on the stand-in's model
  alone; on the world (emu_world_drag) 0.93-0.94, the clamp at 0.61-0.75, no trip
  (2026-09-28).
- The network against the camera states, stand-in truth: worst miss 9.0 K now,
  20.1 K before the emulator (regulators +28 for +8, AFE on read as passive).
  Left: regulators 3-9 K hot, bridge 4-6 K cool (2026-09-28).
- Camera 2026-08-28, 20 C room, four states x 25 min (tau 6.8 min): board and
  rises (MCU/regulators/bridge/AFE) passive 30.0 / +15.0 / +8.0 / +1.0 /
  +1.0. NTC - TSEN: -0.74 C idle, +10.94 C switching.
- Measured: board 8.33 K/W off the board (one passive point, 1.2 W), 49 J/K
  from a transient. Assumed: per-leg `to_board` 45.6 K/W (one camera zone
  x3), part capacities, NTC fraction 0.30 (pick-place geometry, R a floor).
- Datasheet IAUCN10S7N021: Rth JC 0.69 K/W (6.2 K at 100 A), Rth JA 25.9 K/W,
  Rds(on) 1.8 typ / 2.1 max mOhm (model books typ, -17 % worst case).
- The envelope must step and evaluate per 100 ms slice: a 1 s step let the
  driver node reach 178 C before the clamp saw it. Catch-up capped 2 s.
- A rail another had just raised was read without the settle a borrow gets,
  and a reading without the reference is mid-scale: a 25.00 C thermistor and
  a 545 C die, which the anchor took. Read after the settle now, never
  without the reference (2026-09-26).
- Squaring one synced sample per step aliased; `Board_SyncMeanSquare`
  accumulates per leg in counts. Its slope was taken over one code, where the
  record's integer gain trim truncates to nothing: the observer's I^2 never
  saw a channel's gain. Over 32 768 codes (2026-09-26).
- The stand-in's losses were `phase_power`'s (no duty, link, dead time,
  tempco); now `thermal_power_estimate`'s, mirrored in `coaxial.model.thermal`
  and held to the C. Its held vector turned the current's size, not the
  phases: one leg heated (2026-09-26).
- With the tempco a leg has no equilibrium past 28 K/W x P x 0.78 %/K = 1:
  60 A rms held ran one away before the winding warmed. The stand-in's
  setters wrote its live network and the identification's next update undid
  them; they write the base, the scales re-applied (2026-09-26).
- Steady state at 5.30 mOhm: continuous 19.1 A vs a 105 C laminate, 22.0 A vs
  a 125 C junction. Throttle at 90 % of span (2026-09-05).
- Model above ~40 C board is extrapolation. Settles it: a camera run under
  load, and an NTC slope after a power step.
- Identification (host, vs a ground truth): air path box 2.0 -> 1.93, fan
  0.5 -> 0.50 in three cycles, innovation 0.05 K. Spread and NTC share are not
  observable from a cooldown and are held.

## DAQ and clocks

- Reader thread: 84.4-134.6 rec/s with 4 ms of work a block.
- 16 K ring overflowed in 6 s of a stalled terminal; ring is 448 KB AXI SRAM.
- `interval_us` 0 with `records` 0 took the link down: refused.
- A block read after a CYCCNT wrap came back 9.02 s in the past: stamps are
  unwrapped across blocks. CYCCNT wraps every 9.04 s at 475 MHz.
- A Windows clock "in sync" can sit most of a second out; not a reference.

## Calibration

- DC link spanned vs DMM 2026-08-30: 31.04 read, 30.05 true, -32 418 ppm ch 5.
- Phase gain from the schematic (2026-08-26), not spanned.
- LTspice's AFE (amplifiers.asc 6bc736b, nominal, 33 samples to 106 A):
  10.24 mV/A, zero 8.7 mV at the ADC's differential input. The record's shunt x
  THS4551 gain is 15.9 mV/A, the schematic's own note 9.2 mV/A and 110 mV:
  unspanned, a phase reads 0.64 of its current (2026-09-25).
- Unspanned, an emulated drive put 1.55 times its command through the plant:
  the world's legs 170-300 C under an observer's 100; unzeroed, the 8.7 mV
  zero is 0.85 A. An emulated MCU is spanned on the repl's 10.238 mV/A and
  zeroed at open: the world carries the regulated current within 0.2 %
  (2026-09-26).
- An id added without moving `BOARD_CAL_PARAM_COUNT` is held but never reported.
- Replies past 253 B page (ADC table, pins, parts).
- `BOARD_CAL_PARAM_COUNT` stayed 46 after the winding's ids 46-48
  (CAL_VERSION 12): settable, never reported by cal op 8. Now 49 (2026-09-23).

## Bootloader

- Not run on a board. 15 488 B Debug / 8 312 B Release (2026-09-23);
  host-tested core (64 checks) and master on a stand-in bus of four (24).
- Identity via a 32-byte DTCM slot, not the record (keeps the bench's
  CAL_VERSION 13 record valid).
- A node keeps an image whose size and CRC match: no erase, no programmed word.
- First bench act: `build_and_flash.py --boot`: without the bootloader in
  sector 0 nothing copies the store into RAM.
- The master sent chunks 50 ms after erase and sealed with a 0.5 s timeout;
  the node erases (~s) and programs in its receive path. Waits added.
- The application runs from D2 SRAM (0x30000000, 288 K, unused before): 201 K
  Debug, 135 K Release. Flash keeps a sealed copy, written only where its
  CRC differs; nothing runs unverified. The master's `missing` bitmap was
  1 K for the 1792 K flash image, past one reply; 165 B now (2026-09-23).
- Found by driving the host's `Boot` client through the C core: the
  bootloader echoed device and op in front of every 0x6E reply, where the
  application sends the fields alone; `missing`, `dump` called the
  `remaining` property (2026-09-23).
- Run on the emulator: a blank node on a 10 Mbit limb takes this host's build
  through `Coaxial63100.open()`, 139 K in 17 s at 475 MIPS. It answered its
  own RS485 echo (RE tied low) until the echo was drained after each reply
  (2026-09-25).
- On a bus the host looked for a blank node, and polled the application after
  `go`, at one rate: the bootloader listens at 10 Mbit, the application at its
  record's `link_baud`. Renode's rate-blind lines and a limb adapter set apart
  from the host's port hid it. The port moves to 10 Mbit for the bootloader
  and back for the application; the emulated adapter follows the port
  (2026-09-28).

## Host and tooling

| Subject | File |
| --- | --- |
| The gynoid walking: her plan, its shape and her look, the start from the squat | [findings/walk.md](findings/walk.md) |
| The gynoid kept up: the capture law and the side step, the floor's events and shoves, the scoreboard and its searches | [findings/balance.md](findings/balance.md) |
| The gynoid's build and drives, her buses, her falls and her get-up, her drawing and her clothes | [findings/body.md](findings/body.md) |
| The drive and its observers on the stand-in, the emulator and native://, the rotor's pages and their demos | [findings/drive.md](findings/drive.md) |
| Renode, native:// and CI: the image emulated, its peripherals, its speed | [findings/emulation.md](findings/emulation.md) |
| The host's pages, tooling, suites and the local model's runner | [findings/host.md](findings/host.md) |

## Local model

- Wrote "Mid-scale ... 25.00 C" from the warning text when refusing was tried.
- Invented a coaxial cable twice; guessed `ch=['phA']`, BUS_VOLT, A0; sent the
  left knee (asked in Swedish) as `right knee`.
- gemma4:12b answered a measurement with HARDWARE.md's table; denied being
  able to flash with the tool listed.
- qwen2.5:14b switched the AFE on four times in one turn; answered in
  Chinese, Japanese and Thai.
- A search hit without its chapter was reported as the explanation when it
  sat under Ruled Out: `find` reports the chapter.

## Ruled Out

Settled; do not investigate again.

### PCSEL accumulation explains the Phase V offset

Ruled out. PCSEL accumulation is real (0xC03) and every path clears it, but
it is not the Phase V offset.

### The NTC channel is not anomalous, it is quiet

The 15 nF node capacitor supplies the sample-and-hold charge; 1.5-cycle
sampling is ruled out on the quiet channels. Not ruled out for Cinj/Clevel.

### The rest

- Hot gate stages: pin speed, not hardware.
- JTAG `Unable to get core ID`: probe firmware; cabling proven.
- BNO085 needing longer after power-up: no; the feature set by hand works.
- BNO085 hardware hypotheses: all four failed; six firmware causes.
- MISO held on the IMU bus: the check's own CS floating low.
- Board halted with two sessions: the port was opened twice.
- H_INTN never asserting: it does; 15 ms round trips missed it.
