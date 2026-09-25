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
- 10 Mbit on RS485, emulated: one byte a pass lost to a byte a microsecond.
  The link drains up to LINK_TAKE_MAX a pass, a frame closed at its silence,
  one clock read a pass; 200 echoes of 240 B, none lost (2026-09-25).

## Gate stage

- 30 ns dead time truncated to 29.5 ns (7 DTG) tripped the supply's OCP
  (2026-08-29). Rounding is up: 8 counts = 33.7 ns.
- Two gate stages 15 C hotter than the third: gate pins at CubeMX LOW speed.
  VERY_HIGH since. Found by a 600-sample pin count and a register dump.
- Gate short probe: neighbour follows within 76 ns; pull-down ~40 k.
- Alternate (op 10) proven 2026-08-30: 12 mid-run reads, both triples, scope.
- STO interlock: Cinj 0.77 V, Clevel 0.06 V against 3 V (2026-08-27). The
  keepalive latch holds a few hundred microseconds.

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
- Squaring one synced sample per step aliased; `Board_SyncMeanSquare`
  accumulates per leg in counts.
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

## Host and tooling

- Model weights (7.6 GB) reloaded per suite were most of a run: loaded once,
  released once. A run killed from outside leaves 8.4 GB on the card.
- The offline gate was 400 s of sleeping on the stand-in's clock; suites run
  four at a time: 142 s.
- numpy's OpenBLAS pool costs ~499 MB commit per process; this laptop has no
  page file. Capped in `coaxial.model.blocks`.
- Renode 1.17.0's STM32H7_ADC has 19 inputs (a conversion on 19, Vgate, throws
  inside Renode) and drops ADC2's registers at +0x100; its terminal hands a TCP
  chunk over at once, so a 257 B ADU overran the 256 B ring. board/emu replaces
  the ADCs and paces the console at its baud: the image passes conformance
  110/110, 420 MB, 47 s (2026-09-25).
- Renode's H7 RCC assumes an 8 MHz HSE (the board's is 25): SysTick ran at
  0.32 of the clock set. The DWT counted a fixed 475 MHz, so the bootloader's
  (160 MHz) t1.5 was 253 us, and a chunk split across a 0.5 ms quantum was
  lost: 47 % of the stream. The DWT counts the core's clock now, and the
  adapter keeps t3.5 + 0.25 ms between the host's frames: none lost. Wall s a
  virtual s at 475 MIPS: app 28, bootloader 10 (2026-09-25).
- Renode's ADC keeps no JEXTSEL or JEXTEN in JSQR: HAL's InjectedStart refused,
  and seven papers stopped at "an injected group would not start". board/emu's
  ADC converts the injected group on TRGO2 (MMS2 1000, OC5REF): 6 032 triples
  in 2 s, no overrun. At Renode's 100 MIPS the drive's ISR outran its period;
  the emulator runs the part's 475 (2026-09-25).
- The gynoid's walk (`machine.gait`) at 0.85 strides/s: every joint's jerk
  under 4.2 times its rms, the knee's peak 375 deg/s, the head 3.5 mm up and
  down, 5 mm sideways, the pelvis 8 degrees each way. Past 1.0 strides/s the
  legs reach full length early in the swing and the knee snaps (2026-09-25).
- Toe-off with the heel's rise eased to a stop there: the foot stood still,
  the knee went -180 -> +409 deg/s. Rising through it (420 deg/stride) into a
  septic swing: the foot never under 0.93 m/s, the knee flexing throughout,
  the toes 0-3 mm over the floor early in the swing, 23 at mid (2026-09-25).
- The board's raster on a GTX 1080 Ti (wgpu, Vulkan): 392x224 100 ms -> 2.1 ms;
  the menu's turntable 64 -> 13 ms at 52x18. BOARD ATTITUDE 66 -> 64 ms a
  frame at 200x60: its ground (32 ms) and paint (15) are the frame now
  (2026-09-25).
- Conformance on `emulator://` on a Threadripper 1950X: 110/110 at 100 MIPS,
  109 at 200, 107 at 475. The three after the 257 B ADU go unanswered: the
  master's waits are wall seconds, unscaled by the port's `time_scale`
  (2026-09-25).
- Emulator speed: the image's MPU. tlib keeps no TLB entry for a page inside an
  enabled region whose subregion is disabled, and walks the MPU on every access
  there; CubeMX's 4 GB region (SRD 0x87) spans ITCM, DTCM and D2 SRAM. Its enable
  masked on the bus: idle 8.2 -> 1.4 wall s a virtual s at 100 MIPS, 23 -> 5.2
  at 475; the drive 43-62 -> 18-22; 8 idle boards in one Renode 37-49 -> 390 M
  instructions a wall second. On in the suites only. Under the drive half is the
  A1335 loop (20 -> 10.7 held): 128 k packets a virtual second, ~45 register
  accesses each, its 1 us settles spinning on CYCCNT; the quantum no factor
  there. Renode's own speed on
  this laptop: 1 200 M ALU instructions a wall second, a GPIO read 0.66 us, a
  BSRR write 1.4 us, a DWT read 1.3 us (0.56 board/emu's). Renode's TIM1 was: an
  event every update, 3.5 wall s a
  virtual s at 100 MIPS; board/emu's counts lazily (9.4 -> 5.9) at the tree's
  237.5 MHz where Renode's ran 250. The core runs 100 MIPS until an ADC waits on
  TRGO2, then the part's 475 (2026-09-25).
- The emulated board under the drive, MPU on: 43-62 wall s a virtual s at 475 MIPS, the
  plant's step 4.5 us of it; TIM1's compares written every period were rescheduling
  its timer (64-76 before). A page reading in its draw waited a round trip a frame:
  on a Feed the attitude page draws 10 fps (5.6), the rotor observer 7.7 (0.3),
  capture 120 frames in 35 s (536) (2026-09-25).
- The firmware's hold turns its vector at `accel` toward `omega_target` and stands
  still at none; the stand-in's turns at once. AFE_ON is refused under an armed
  stage, taken before it (2026-09-25).
- The monitor's tokenizer takes no exponent: `2e-05` in a world's line and
  Renode exited (2026-09-25).
- Renode rewrites `%APPDATA%\renode\history` after every monitor command: a
  body's five processes, each asked its load every 0.5 s, collided, and a limb
  exited on an IOException. Each process has its own `--config` and history;
  the humanoid idle, 20 boards: 2.4-2.6 wall s a virtual s (2026-09-25).
- An armed sync stays armed past drive.off: the meter is the injected group's
  until gate drivers op 3 gives it back (the emulated wire sweep, 2026-09-25).
- `UL` is 64-bit on Linux: `-Wconversion` warned on CI only.
- Ollama answered 500 from 2026-09-03 to 09-12: the runner failed to start.
- Front page model drawn at inner height - 2 and inside a 1-column padding:
  a blank row top and bottom, a column each wall. Now the box's full
  inside (2026-09-23, checked at 90x28, 120x36, 200x60).
- Rotor observer at a 200x60 terminal (can 95 dots, tuned at 21): ring stroke
  grew to 3.0 dots half-width, pulled teeth floated loose. Stroke capped
  at 1.0 (0.8 broke into dots), undriven tooth length drawn as track,
  kept out of any cell an area holds (2026-09-23).
- `env.ps1` dot-sourced into `coaxial_tty.ps1`: its `foreach ($name ...)` was
  the caller's `[ValidateSet] $Name` and failed it; the loop is `$bundle`
  (2026-09-23).
- One DC bus connector's screw hole drew as a box, then vanished into the
  connector (724f950 absorbs nested blocks): the circle test centred on the
  centroid and 14 unevenly spaced points read dev 0.047 (limit 0.03).
  Least-squares centre: a drum, like the other four; no other primitive moved
  (2026-09-23).

- `pole_pairs` counted shaft travel over a wall-clock walk against the
  nominal travel: 20.80 for 21 (the rotor pulling in from its rest angle),
  and past the 0.25 limit on a busy CI runner (red 834519a). Now the
  command's and the shaft's travels, sampled together, after the first
  quarter: 21.00 with six cores busy (2026-09-23).
- BOARD ATTITUDE face down drew the top's parts on the underside. The
  outline's grace adds the cell's depth span, and a tilted face spans
  0.036-0.08 a cell against a 0.032 slab: 533 of 535 drawn dots sat
  behind it. An edge on the slab's far face now gets the fixed grace
  only; the face art (the top's layout) is not read from behind
  (2026-09-23, rasters face down, up, tilted).
- The stand-in's bare rotor on a 2 A hold (k 0.735 N.m/rad, J 2e-5, b 1e-5)
  has zeta 0.0013: it rings at 30 Hz for seconds and a 25 Hz loop pumps it until
  poles slip (224 deg). A joint's damping, b 4e-3 (zeta ~0.5), holds 20 joints
  within 4.1 deg over three runs (2026-09-24). A 25 Hz loop cannot damp 30 Hz:
  that is the board's loop to do.
- Fitment by ring test, stand-in: a 2 A hold stepped 20 deg electrical rings at
  23.8 Hz (hip, J 3.2e-5) to 38.9 Hz (head, 1.2e-5); repeats within 1 %, J back
  within 3 %. The crossing count failed mid-suite (a read gap hid a pair: waist
  and neck swapped); the median half-period holds (2026-09-24).
- A model's answer at 40 tokens/s (3 lines, 2.5 s of motion): first move 0.21 s
  fed a line at a time, 0.66 s sent whole. A `wait` wake costs 15 tokens after
  the first full `now` (~120); the humanoid prompt 1 360 characters. Every pass
  wrote unset outputs as 0, opening the stand-in pack's contactor: unset now
  holds what the node reads (2026-09-24).
- Device 12, the board's loop (MINOR 20): a joint's feedback loaded as slots,
  rows of 0.8 s at 20 and -20 deg, 100 Hz on the stand-in: within 0.05 deg,
  the last held. Four legs with `node_hz=100`: the squat's down ends on its
  level at 0.65 s, the board's 90 deg/s slew over 58. On the board it ticks
  in the drive's sample; not yet run on the bench (2026-09-24).
- A step waits for its targets (1 % of their range) or its tests, its seconds a
  timeout; L and H alarm, logged as they come and go with a 1 % deadband against
  chatter; LL and HH trip; all in `machine.alarms`, beside the sequencer.
  The squat's down ends on arrival in 0.6-0.7 s of its 2; the scan's timeouts are
  answers, not alarms (it branches). The humanoid prompt: 1 434 characters
  (2026-09-24).

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
