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

## Host and tooling

- Entering `coaxial` through `coaxial.comm.session` broke the package's own
  import cycle: `rig` took `EMULATOR_URL` off a half-built session, and six
  pages threw on the emulator while the suite, which enters elsewhere, stayed
  green. `sessionmod.EMULATOR_URL` at call time (2026-09-28).
- A view drawing faster than the board answers repeats a reading: 24 of 60
  frames on the emulator, and the freshness note flickered live/stale every
  other frame. Staleness is elapsed stillness now, not one repeated frame
  (2026-09-28).
- SHAFT ANGLE's bob was not aliasing - 8 deg a frame at most, against a 180 deg
  fold - but the rotor hunting about a free-running vector: 3 deg at 1.2 Hz,
  46 reversals in 20 s. The vector's angle is commanded now, off a raised
  cosine: one turn each way, tracked to 0.6 deg (2026-09-28).
- The stand-in's plant pulls out and runs away held at 30 A, at every rate
  from 0.02 to 0.25 rev/s; at 5 A it follows the commanded angle (2026-09-28).
- ROTOR OBSERVER polled the thermals every 0.25 s on an emulated board and
  every 2 s on a real one - the slowest link took the most traffic. The three
  reads cost 18 ms of a 50 ms tick, over the 23 ms the other four already
  take, and the page froze and raced. Paced by the link (2026-09-28).
- What is left of that page's jitter is the draw, not the link - the same
  130 ms p99 on the stand-in. `cross_section.put` runs 3 800 times a frame and
  re-ranks the cell's owner at each dot; settling it once a frame is held up by
  `gauges` writing `frame.owner` from outside the class (2026-09-28).
- THERMAL OBSERVER said `AFE off` with AFE_ON high: it had no reading because
  the first sample is 30 s after opening. It says which now (2026-09-28).

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
- The gynoid in MuJoCo (55 kg, 26 drives, 1 kHz): on the line the legs stand
  in a V, 8 cm out at the hip for 3 at the foot, and need 0.773 m of 0.770:
  the IK clamped, a foot hung 3 mm off the floor and she tipped over it; the
  pelvis 12 mm lower. On one leg no stiffness held her (x5 4.8 s, x20
  unstable); the CoM fed back through the pelvis's target did (2026-09-25).
- Walking free in 3D: a swing leg reached from the pelvis's target put the
  foot 6 cm across the line; reached from the measured pelvis, a stance foot
  held where it landed and taking its weight from 0 over 0.04 of a stride,
  she walks 60 s at 0.6-1.0 strides/s. The sideways speed raw in the
  feedback, 500 Hz, or a sole turning at 0.03 m of torsion: she fell in 2 s
  (2026-09-25).
- Her torso on the pelvis pitched 8 degrees a stride, the head 9 cm fore and
  aft; the spine taking the pelvis's pitch back out: 2.4 and 3 cm. The
  walker, loop and world 0.63 ms a pass: x1.6 real time in her own process
  (2026-09-25).
- Her stance knee at its straightest stood at 27-31 degrees: the pelvis held
  level where the gait drops it on the swing side put the stance hip 2 cm
  low. The pelvis dropped as planned (5 degrees), the spine's roll set to
  take it back out (fed back from the measured roll she fell): 5.5, 9.0,
  12.7 degrees at 0.6, 0.75, 0.9 strides/s, 30 s each on her feet
  (2026-09-25).
- Landed on the ball, heel up, the landing knee stood at 42 degrees and the
  step struck 3.3 body weights: the stumble. On the heel, toes up 9 degrees,
  rolled flat by 0.13 of a stride: 29 -> 10 degrees in the plan (2026-09-26).
- The finest of 576 walks (16 cores, cross-entropy, 23 knobs), judged by the
  stance knee past 5 degrees, the soles' 20 ms peak, the pelvis's shake, the
  head's travel and the copper: bend 11.7 -> 5.7 degrees, peak 2.9 -> 2.2
  body weights, head 8.5 -> 6.0 cm, heat 0.47 -> 0.29, work 0.60 -> 0.35 m g
  d. Cost of transport 0.64: the cheapest, knees bent, 0.52; the old walk
  1.07. Straight legs cut the copper by 37 % (2026-09-26).
- Its sideways gains (STEP_D 0.01) held the walk and lost every start from
  standing: 0.057 back. Started as the right foot lands, on her standing
  stance gliding into the catwalk, she walks on at 0.85 strides/s; at 0.7
  and 0.9 she falls within 2.5 s. Stopping: a stride shortened at the pace
  kept threw her off the floor; carried in phase, slowed, she settles onto
  the front foot and runs on over its ball (2026-09-26).
- The virtual pendulum between her ears (`machine.pendulum`: 1.13 m, her
  weight, a spring of no length of its own): stir 3.2, 4.3, 4.9 mm at 0.65,
  0.85, 0.9 strides/s, 78 % of it on - the head's surge - peaking at each
  landing (2026-09-26).
- 26 knobs +-20 %: all but four within 0.25 mm of 4.2. The torso countering
  the surge twice a stride, 2.5 degrees at 0.125: 2.2 mm; with the head moved
  toward the bob (`SWAY_K` 1): 1.7; by its drift too, down in 2 s. The rise
  flips on 0.5 % of any knob, the committed gait's as much (10 of 24 held):
  one run a candidate scores chance (2026-09-26).
- Twelve trials a candidate (`tools/sim/gait_montecarlo.py`: three rises,
  three walks, six shoves of 14 N s), 19 s on 16 cores: she holds 85 % of the
  time, every side shove fells her. Shoved toward the standing foot, the catch
  steps the other across it; held off, she falls 1 s later; the standing foot
  lifts on the clock while the other still reaches. The capture point's law
  on the plan's reference fells her walking: the plan's sway is not the
  pendulum's - the DCM error in a steady walk 93 mm rms at liftoff
  (2026-09-26).
- The capture point from the pelvis's sideways speed, not the centre of
  mass's (the swing leg's speed is in that: a wide step put it 6 cm out past
  where it went); its course 0 -> 51 mm out over the swing at 0.65, 0.85 and
  0.9 strides/s; the ankle holds what the course leaves of the sole (at 0.65,
  25 mm off at 0.7 of the swing doubled); the foot 1.1 of what is off further
  out - at 1.3 the step back grew 1.35 a step (`machine.capture`,
  2026-09-26).
- Landings: the height's target from where the body is (from the plan's, 3 cm
  up, both legs threw her 5 cm into the air); the forward target moved no
  faster than 0.3 m/s (a foot 5 cm short snapped the knee straight); the phase
  at most twice its pace (it raced to 3.5 strides/s). The side step: the
  swapped foot down over 0.1 s ahead of the pelvis, the other out over 0.15 s
  to the capture point foreseen once, set down by a sine squared, eased on
  from where it stood; over once it bears her; the walk begun again on it at
  the speed she has, the other to step in beside. The first steps narrow over
  0.6 s: over 2.5 s the third lifted with the capture point 8 cm inside the
  standing foot. Twelve trials: held 73 % - the rises, the walks at 0.85 and
  0.9, the shoves along the line; the side shoves still fell her (the side
  step ends with the capture point 27 cm ahead and no foot there), and the
  walk at 0.65 from its own start pose drifts 7 cm across in its first second
  (2026-09-26).
- What does not help: a toe-off gate on the capture point (the legs driven
  3.5 cm across moved the pelvis 3 mm in 0.2 s; walking on, the frozen plan
  sank her 17 cm); the pelvis driven by the capture point's error (the foot
  slid 20 cm) or by the course's deviation alone (down in 1.4 s): the plan's
  sway servo stays (2026-09-26).
- The director's slip re-anchoring at 4 cm (at 2 cm the feet's slides under
  the ankle's drive at 0.65 re-anchored eight times in 2 s and she fell) and
  the side step's out foot set where the pelvis will be, kept there: twelve
  trials held 87 % - the walk at 0.65 and the shove toward the swinging foot
  at 0.85 hold too. Still felled: the shove toward the standing foot (the side
  step ends with the sagittal capture point 27 cm ahead and no foot there)
  and the shove at 0.9 as the swinging foot lands. Hurrying the phase on a
  catch latched the landing early and 9 cm short; a swinging foot held as
  landed once it bore mid-swing reset the phase at every touch: both out
  again (2026-09-27).
- The side step's foot–foot strikes: the stepping foot's toes struck the
  put-down foot's heel going by (5 cm between 10 cm wide feet, 1800 N), so
  that foot is put down 12 cm across at least and the other goes out before
  on; pitched from its swing, a put-down foot's toes took 1000 N 9 cm up, so
  borne means the ankle within 1 cm of the floor; a height still lowered for
  a catch shortened the trailing leg 3 cm as the step out began, so that leg
  takes the height from the body. A swap asked is a fresh judgement each
  pass (latched, one was done 0.2 s later on the wrong side), none in the
  first 0.15 s of a walk begun again, which blends in over 0.1 s (over 0.3
  the trailing foot stayed down and its share of the load drove the capture
  point on past the standing foot). A catch out of reach is a stomp: the
  foot put down at once where the capture point will be 0.1 s on. Still 87 %:
  the left shove at 0.85 zigzags into a second and third side step; slewing
  the attitude's correction at 0.5 rad/s let the roll grow through the step
  (2026-09-27).
- A search (CMA-ES, 144 candidates) over the torso's counter, the crane
  damping and the capture law's margin and gain: cost 8.2 -> 5.9 (stir 4.4
  -> 1.8 mm), but re-scored with the damping 0.556 -> 0.55 the walk at 0.9
  fell in 4 s, and a 0.1 % change of any knob flipped a shove: landed at 5 s,
  the shove met whatever stride phase the pace had brought her to (0.25 at
  0.85 strides/s, 0.48 at 0.65 and 0.9, 0.07 less with the counter). The
  shove now lands as the phase first crosses 0.30 after 5 s. There the
  counter alone, 3.9 degrees, takes the stir to 2.4 mm, held 86 % against
  84 (3.5-4.5 alike, one corner of margin and gain felled a walk); margin
  43 mm and gain 1.1-1.27 add 0.1 mm and noise; the damping stays off. At
  0.30 the shove toward the swinging foot fells her as it is: the landing
  latched at 0.7 of the swing with the capture point 9 cm out, it ran to 23
  and she toppled over the foot (2026-09-27).
- The pendulum between her ears, read by stride phase at 0.85 strides/s
  (fore, across and up alike, 0.8-0.9 mm each): nearly all of it goes in
  at touchdown. The pelvis fell into it at 0.15 m/s, 11 mm under the plan,
  the landing leg came down bent 23 degrees (the IK's own setpoint) and
  straightened to 7 within 0.16 of a stride, lifting her 26 mm at 0.29 m/s
  to a dead stop at the top: 6 m/s2 up then down at her ears, 3.5 aft as
  the front foot braked her, 4 across, the sole's peak 1.4 kN. A stance
  knee 16 degrees soft at its straightest leaves the leg room: 2.4 -> 2.0
  mm, and the plan's dip eroded over 0.05 of a stride (17 -> 20 mm, nearer
  the body's) 1.7; the peaks 3.7 and 2.5 m/s2. Left: 12 mm of sag at
  touchdown and a sole peak of 1.9 kN - the soles are rigid (2026-09-27).
- The soles given a little, as light sneakers (`machine.physics` SOLE_*):
  the contact's damping 1.5 of critical and its impedance easing in over 5
  mm take a touchdown's peak 1.9 -> 1.5 kN and the stir 1.7 -> 1.4 mm, a
  shove's parry 1.5 -> 1.1 kN. Every softer sole felled the first stride
  from standing, the walks and the shoves unharmed: settling over 0.035 s
  instead of 0.02 the body pitched on twice as fast in the first single
  support (the sole is a lag in the ankle's hold) and the foot landed 0.24
  s early; damped 1.75 or more the stance foot's load flickered to 70 N
  under the rolling foot (a damped contact kicks at its corners' speed) and
  she toppled sideways. MuJoCo's soft contact is a poor foam: its
  softness is a time constant on the whole body's mass, its damping acts on
  every corner's motion (2026-09-27).
- The walk begun from a lean (`machine.arrival`): shifted onto the left
  foot, her weight goes 7 cm ahead of the ankles over 0.6 s on both feet,
  then 5 cm further as the right foot lifts 6 cm over 0.3 s, and the walker
  takes her mid-swing at the phase where the plan has the pelvis where it
  stands over the left ball (`Walker.begin`, as after a side step), the
  foot landing on the walk's own track 4 cm from the standing one. Set down
  first in the walk's landing pose, still, the pelvis tipped back 2 degrees
  as the foot lifted and 5 forward as the walk took her. Handed on still,
  she hung back behind the landed foot and tipped over backwards (the phase
  pulled to a body standing still stands with it); landed on her standing
  stance, 16 cm out, the pelvis could not get over the foot and the next
  went 20 cm out to catch her; leant 10 cm in 0.5 s she was thrown off the
  left foot. The scoreboard as before, the rises held (2026-09-27).
- The look of the walk at 0.85 strides/s, by the trace: the stance knee
  16-29 degrees (the plan's own 16-24: the pelvis rides 10 mm under the
  plan, latched there at each landing as the trailing leg sags 9 mm into
  it, its ankle drooping 2 degrees under the push-off's torque, and raised
  at 0.1 m/s - a rush of 0.22 m/s to a dead stop); the ears 19 mm up and
  down a stride and 33 fore-aft, the torso nodding 7.6 degrees (the
  counter), 66 fore-aft with the torso still: the pelvis's own surge, the
  collision at every touchdown (-0.25 m/s over 0.13 s, +0.2 back before the
  next). The stance knee 8 degrees soft, the heel strike 15 degrees toes
  up (its peak 1.55 -> 1.1 kN), the recovery 0.03 m/s: the knee 9-29, the
  stir 1.4 -> 1.2 mm, held 86 %. The counter off for the eye: 3.5 mm.
  Straighter legs at the strike need the trailing heel up before it (the
  plan's height there is that leg's, flat, 22 cm behind): a heel rising
  in single support ran her ahead of the plan and she sank 35 mm before a
  landing 0.1 stride late (the phase, pulled to the body at 5 a stride,
  lags it); the heel to 50-55 degrees at toe-off felled the walks alike
  (2026-09-27).
- The floor's events (`physics.World.terrain`) in the shoves' place on the
  scoreboard, a shove hardly ever happening to a walker: a hole 3 cm deep
  and 40 cm long under the left foot's next landing (the floor a slab over
  a plane that deep, cut in two), a sill 4 cm high 15 cm ahead of its toes
  as it lifts, a patch at 0.06 under the landing, a loose rug (0.1 on the
  floor, the sole's own grip on it) its front edge 15 cm short of the
  landing. Her toes skim at 3 cm through the first 0.16 s of a swing: a 2
  cm sill they shoved at with 200 N and went over, at 4 with 350 N, the
  pelvis tipping 8 degrees, and the swing carries them over. The patch at
  0.15 let the stance foot creep 3 mm (the walk asks 0.17 of the floor), at
  0.06 it slides 10 cm back under the push-off and she walks on, tipped 6.
  The rug at 0.3 lay still (the sole's shear 100 N, its hold 165), at 0.1 it
  goes with the push-off and she falls at 7.7 s. The hole fells her at 6.6
  s: the foot finds no floor where the plan lands it, the leg holds it 3 cm
  short, the body falls 14 mm and runs on, a catch lifts the wrong foot. A
  geom moved or grown past its compiled bounds is missed by MuJoCo's
  broadphase (the rug fell through a slab grown 27 m, a box through a sill
  moved 1 m): the slabs are compiled over their whole span and cut, the
  sill and the patch are mocap bodies. Held 93 % (2026-09-27).
- A fall seen early (`machine.director`): the pelvis tipped past 12
  degrees and tipping on faster than 60 a second, or under 0.65 m, is past
  the walker's recovery; she curls into the squat's joints over 0.4 s with
  the arms out toward the fall (ahead: the hands out in front, the head up;
  behind: the arms down behind her, the chin tucked) and lies as she
  landed. Her seat, back, chest, skull, arms and thighs are contacts now
  (`figure.CONTACTS`): curled only at 35 degrees over 1.5 s, the legs
  walked on through the fall and she lay with her torso through the floor.
  Into the hole she goes at 13.7 degrees and 300 a second, 0.16 s before
  she is down at 40, the hands take the floor 0.12 s before the head
  touches once, and she lies still on her left side and seat from 7.4 s
  (0.7 s after); off the rug she sits down backwards onto her feet (1.5 kN
  each) and rolls onto her back. The walking stumbles tip her 6-8 degrees,
  under the trigger (2026-09-27).
- Getting up, tried in the joints alone (`tools/sim/getup_lab.py`): from
  her side, straightening out rolls her onto her back; from her back a leg
  crossed over, either way, or the right arm and leg swung up roll her onto
  her right side; from her front the push-up onto hands and knees and on
  into the dog (the pelvis 0.48 m up on hands and toes) work, but the
  squat's joints from there put the knees down with the torso on the
  floor, and a lunge with the right foot tips her onto her left side. Sat
  up from her back, she rolled onto her side. The arrival's rise takes a
  yaw now and the walker a heading (`Walker.heading`, the bus turned about
  the vertical), for a get-up facing as she lay (2026-09-27).
- A drive's glitch (`physics.World.glitch`, the peak torque cut to a share
  for a while) on the scoreboard: the left knee's gate dropped for 0.15 s
  at mid-stance ('cut') gives 19 degrees under 500 N, the pelvis 16 mm,
  and takes her weight again, tipped 4; the knee derated to a tenth for 2
  s ('hot', 25 N m) folds 18 -> 64 degrees in 0.3 s and the pelvis sinks
  20 cm, past the height latch's 12: the other leg, reaching from a target
  the body is no longer at, holds its foot in the air and she falls
  backwards at 5.7 s. Walked on straight (the pelvis at the weak leg's
  reach, its step 8 cm short) it fell sooner: a knee already folded cannot
  straighten under her, and the raised target hung the other foot; the cut
  knee, straightened the same way, fell too. The hip held to a quarter for
  a second changed nothing (a stance hip asks under 60 N m). The swinging
  foot lands on the capture point along the walk now as across it (the
  body's speed over the plan's, over omega, 25 cm at most, `walker.FORE_K`):
  the walks' stir 3.70 -> 3.52 mm and 3 % further, the rises held; slowed
  to a stop by the folding knee, the foot had come down where the plan
  had it, ahead of a body going nowhere (2026-09-27).
- The calf: the heel to 50 degrees at toe-off (`gait.TOE_DEG`, rate -300
  a stride, turning back at 4000 a stride squared), the rise scaled by the
  stride. With the foot landed on the capture point along the walk it
  holds: the walks' stir 3.35 -> 2.80 mm, the ears' up-and-down 18 -> 12 mm
  a stride, the plan's landing knee 29 -> 14 degrees, the hot knee walked
  out (tipped 12), held 90 %. Unscaled, the first short strides' push-off
  hopped her off the front foot (the rises fell); 55 and 60 degrees, an
  earlier heel-off and a slower rise all lose trials. After toe-off the
  toes' tips skim the floor (the sole's peak 1.4-1.9 kN there): curled up
  20 degrees they touched it still, the toe drive too slow (2026-09-27).
- The head's fore-and-aft, 61 mm a stride at the ears: the pelvis surges
  30 (its speed 0.76-0.99 m/s over a 1 m stride, the vault's exchange plus
  the stance leg braking it 2 m/s2 through half the stance and pushing 3-5
  before the landing) and the torso's pitch, 2.2 degrees the spine's
  counter does not take out, doubles it up there. Tried and out: the phase
  run as a pendulum (58-65), a lead on the pitch's rate for the spine (fell
  in 4 s at 0.05-0.15 s either sign), a shorter stride (0.9: the pelvis 38
  at 0.82 m/s), a planned landing dip (the body sags under any plan, the
  knees bent the same and the bob 19-24), the height latch let go of its
  first 15 mm (the first landing hopped), the bob's swing on the landing
  (`PEND_K` 1.5: fell in a second). The stance knee, 6 degrees soft: 4-7 at
  its straightest, 15-20 through mid-stance from the latch's 10-12 mm; at
  4 the walk at 0.9 fell in 0.6 s (2026-09-27).
- The lean before the first step is the body's, not the pelvis's: pushed
  7 cm ahead with the torso plumb (`PLUMB` 1.0) the pelvis went out under a
  vertical trunk and the trunk pitched back half a degree at the push - she
  read as leaning back before she stepped. Now the lean's frame tips the
  pelvis LEAN_DEG 4 forward and the torso with it, the neck keeping the
  head level; the walker takes the lean over at the hand-off and lets it
  out over LEAN_OUT_S 2 s. A lean kept through the walk, 2, 3 or 4 degrees,
  had her fall to the slip and the hot knee and the walk at 0.9: held 76,
  76 and 66 % against 90; let out, held 89.8 %. A stiffer spine steadies
  the torso, not the head: kp 800 -> 1600 -> 2400 N m/rad took the torso's
  pitch 2.1 -> 1.2 -> 0.8 degrees a stride and the ears' fore-and-aft
  58 -> 68 -> 63 mm, the pelvis's surge 24 -> 39 -> 33 - the spine's give
  is the filter between the hips' pulses and the head (2026-09-27).
- The drives' callouts (HUMANOID page): docked at the viewport's edges, a
  side's joints on its side, each a row - the joint, its angle, its torque
  as a five-cell bar and a number, its power as a bar, driving or braking -
  with a leader to the joint's pivot as it moves; beside her at 0.42 m they
  were three cells and crossed her stride (2026-09-27).
- Her boards on their buses, simulated (`machine.buses`): a bus a limb (the
  type's subsystems - the axis, each arm, each leg), a process each with its
  boards' PD loops, in lockstep with the world a step at a time over a
  shared block (q, qd, limit in; ctrl out; a byte on stdin a step), the
  host's frames real bytes on a TCP socket a bus (`socket://`, an emulated
  limb's port). Modbus RTU (`machine.rtu`): a pass one 0x10 broadcast of
  the bus's setpoints (9 bytes and an i32 mdeg a unit, SETPOINT_REG) and a
  0x03 poll a board (8 bytes; the reply 13: angle and rate, i32 mdeg,
  mdeg/s, STATE_REG), a frame landing its bytes after its stamp at 9 216 000
  baud (USART2/UART5's rate in the .ioc; 115 200 is the debug VCP's alone),
  8N1, a reply the board's 30 us turn after the poll; nothing on a bus
  still busy a pass on. A leg's seven boards hear a pass's setpoints 40 us
  on and the host their state 0.4 ms on: held 89.8 %, stir 3.55 mm, as the
  threads had it (89.8 %, 3.53). A lockstep of five processes: 19 us a step
  spinning on the block, 37 spinning then a semaphore, 46 the semaphore
  alone; the walk's step 1 431 us wall against the threads' 1 710 (the GIL
  hand-offs), the buses' share ~350 us a pass (a sendall 21 us a bus, a
  pipe byte 10 a process, CRC and parsing ~115); 0 bad frames in 10 000
  replies. The setpoint's rate a board carries on at is read between two
  frames, never from a hold: from the reset's hold to the first frame 37 us
  on it came to 150 rad/s and every drive slammed to its peak. A world
  with no buses runs the drives itself (`tools/sim/getup_lab.py`)
  (2026-09-27).
- The feet: 27 cm with the toes, outsize on 1.60 m, but the walk is tuned
  to them - at 23 the walk at 0.85 fell in 0.5 s from mid-stride (a catch
  at 0.2 s, both feet off the floor), at 25 the rises fell at 10.9 s, the
  walk at 0.9 in 0.4 s and the hot knee at 8.8 (2026-09-27).
- The start as the eye has it: taking her from the lean, the walker's plan
  stood the pelvis up, 7 degrees back in 0.2 s, the stance heel rising - on
  her toes and leaning back; the plan tips it as the lean now, LEAN_DEG 4 ->
  8 to read as one. Risen 1 cm under the stand, she stood on 20-degree knees
  with the hips behind them: risen to the stand's soft knee (4) and sunk 4
  mm as her weight goes left (`arrival.SINK_M`; unsunk the stance knee
  locked at -3), 7-10 through the step. The weight's 6 cm onto the left
  foot rolls the stance hip -4.4 degrees (-7.7 as the right lifts): a lift
  from standing needs her centre of mass over the left sole, and from the
  page's 60 degrees the column's lean reads as leaning back. The head
  nodded 6.1 degrees a step on the neck's 60 N m/rad; 300 at half critical:
  2.2, the pendulum's stir 3.55 -> 3.00 mm, held 89.7 % (2026-09-28).
- Her walk made hers: the arms swung from the forearm, near the body
  (`gait.ARM`: the shoulder 12 degrees a side, the elbow 24 +- 11, the
  wrist 10 +- 7). The hips already swayed 35 mm to the shoulders' 23, the
  pelvis rolling 10 and turning 39 degrees a stride; a deeper hip drop
  (ROLL_DEG 6, 8) swayed the shoulders 28 and 33 mm and nodded the head
  2.9 and 3.6 degrees. The pelvis tipped 6 degrees under an upright torso
  read as leaning back from the rise on: the page's spine at -5.9, the
  torso bent back over the hips; out again. The scoreboard cannot judge a
  look: the committed arms moved 2 % (the shoulder 16.3, the elbow's swing
  9.2) held 80 and 75 %, the slips and the hot knee flipping; every arm
  tried held 84.7-85.5 %, the chosen ones 89.7 (2026-09-28).
- The weight onto the left foot before the first step, partly as the
  right lifts (`arrival.SHIFT_IN` 3.5 -> 5.5 cm inside the left ankle):
  standing, the stance hip rolls -2.5 degrees, not -4.4, the pelvis 33 mm
  across, not 61, her centre of mass 30 mm left at the lift, not 52; -7.8
  through the step as before. Ten and sixteen perturbed starts (timings,
  gains, the lean's and the lift's reach): 3.5 held 10 and 15, 5.5 and 6
  held 10 and 14, 6.5 held 6 of 10 and 7.5 one, the first steps falling at
  9-11 s (2026-09-28).
- Leaning back before the first step, by the page's recordings (R): the
  pelvis sunk for the weight's shift bent the knees 4 -> 12 degrees under a
  plumb torso - the hips 26 mm behind the knees, the hips-to-shoulders line
  -0.5 degrees, the torso 7.2 behind the shins; rising, the torso came up
  first, 18.6 behind them. Now the torso leans as far as the shins
  (`arrival.with_shins`), rising through a keyframe at 80 % of the height,
  +5.7 standing, and the lean's 8 from the shift on: the hips-to-shoulders
  line +3 standing, +8 shifting, the sink 2 mm. The seam to the walk: the
  arrival rode the torso on the pelvis and the walker eased its spine in
  from the hand-off's, so the torso swung 10.8 -> 14.0 -> 7.5 degrees as
  the pelvis tipped; both now take the pelvis's tip back out of the spine
  (`walker.PLUMBED` not eased in): the torso 8.7-9.8 through it, the
  largest swing back in 0.3 s -6.6 -> -3.0. Held 89.7 %, 15 of 16
  perturbed starts (2026-09-28).
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
- The emulated board under the drive, MPU on: 43-62 wall s a virtual s at 475
  MIPS, the plant's step 4.5 us of it; TIM1's compares written every period
  were rescheduling its timer (64-76 before). A page reading in its draw waited
  a round trip a frame: on a Feed the attitude page draws 10 fps (5.6), the
  rotor observer 7.7 (0.3), capture 120 frames in 35 s (536) (2026-09-25).
- The firmware's hold turns its vector at `accel` toward `omega_target` and stands
  still at none; the stand-in's turns at once. AFE_ON is refused under an armed
  stage, taken before it (2026-09-25).
- The monitor's tokenizer takes no exponent: `2e-05` in a world's line and
  Renode exited (2026-09-25).
- Renode rewrites `%APPDATA%\renode\history` after every monitor command: a
  body's five processes, each asked its load every 0.5 s, collided, and a limb
  exited on an IOException. Each process has its own `--config` and history;
  the humanoid idle, 20 boards: 2.4-2.6 wall s a virtual s (2026-09-25).
- The A1335 by DMA, its read stepped from main() without a spin: the emulated
  drive 20 -> 17 wall s a virtual s (the angle loop 9 -> 6). main() in WFI with
  AFE_ON low and MOE clear: idle at 475 MIPS 5.2 -> 1.0, real time. Renode
  takes `WfiAsNop` only after `ClearTranslationCache`, and restarts
  `ExecutedInstructions` on a reset; FastDWT anchors on the CPU's TimeHandle at
  the first read after a wake. Asleep, Renode's load reads 1: the host waits
  by the awake scale, and a socket:// rig without it gave up on replies
  (2026-09-25).
- The thermal observer's borrow of AFE_ON (0.5 s every 30 s) took test_wire's
  AFE-off scan once the idle board kept real time: the test holds sampling
  off (2026-09-25).
- The A1335 and the BNO085 paused for that borrow with the host holding
  AFE_ON beside it: the host's angle and IMU dropped 0.5 s every 30 s
  (test_emulator's angle read, None). They pause when the observer holds the
  rail alone, `Board_PowerAlone` (2026-09-26).
- The ring test stamped each angle read before its ask: a stall under the
  gate's load swapped neck and head (1 Hz apart). Stamped mid-read
  (2026-09-26).
- The emulated drive, 50 kHz, costs a period 3 interrupts (Renode: 3.7 us an
  entry and exit, 4.8 with FPU state), ~30 register accesses (0.9 us each on
  board/emu's models) and ~2 950 instructions at -O0. Floor with the handlers
  whole: ~2 wall s a virtual s. main() skipped to 0.5 us short of each timer
  event: 16.5 -> 6; Board_PwmReady and ARR kept once read, ADC3's injected end
  without HAL's walk: 6 -> 2.5, 50 000 updates a virtual second, no overrun.
  Changing MIPS inside a Renode round lost a third of the periods; SkipTime
  keeps its accounting. CYCCNT and TIM1's count run on one virtual clock:
  instructions, sleeps and skips. The Release image ran 7: its main() polls
  more registers a slice (2026-09-25).
- The drive's triples straight into the compares (RCR 1 while it owns them,
  unskewed): no TIM1 update interrupt, 1 interrupt and ~18 register accesses a
  period. A/B, alternating on one quiet laptop: the emulated drive 4.8-5.7 ->
  3.0-3.5 wall s a virtual s; the same image read 2.5 hours earlier, so only
  alternated runs compare (2026-09-25).
- A script killed by `timeout` left its Renode spinning 4 h, skewing every
  measure since; each Renode now sits in a job that dies with its Python
  (2026-09-25).
- An armed sync stays armed past drive.off: the meter is the injected group's
  until gate drivers op 3 gives it back (the emulated wire sweep, 2026-09-25).
- native://: board_pwm.c, board_sync.c, board_adc.c and board_drive.c built for
  this host over board/native's TIM1, ADCs and AFE. The drive 1 virtual s in
  0.048 wall s flat out; paced, 1.000, 50 000 updates a wall second, no
  overrun; the conformance suite 96/96 over it. The pacer at Windows' 15.6 ms
  timer and 5 ms a wake ran a third of real time: timeBeginPeriod(1), caught up
  whole each wake.
  The host's `-Wconversion` found three narrowings in board_pwm.c and
  board_sync.c the target's flags pass (2026-09-26).
- native:// with the whole board layer (power, STO, A1335 by DMA, BNO085 over
  SHTP, the drive): 1 virtual s in 0.055 wall s a board. The humanoid, 20
  boards on 5 buses, every drive holding: each limb 1.0000 virtual s a wall
  s, 50 000 triples a virtual s a board, no overrun, 2.05-2.28 of 16 cores
  (2026-09-26).
- With `proven_dispatch` the host skipped t3.5 after any proven request: the
  addressed board closes a frame on its CRC, the rest of a limb on the
  silence, so a frame for another unit ran into the last and went unanswered.
  The gap is skipped for the same unit only. Rigs on one URL bus each hold a
  Transport, none owing another's gap: the native limb keeps t3.5 as the
  bus's adapter (2026-09-26).
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
- An armed sync leaves the meter the NTC and the DC link (`read_index`): a
  software-clocked sweep waited on phase U for ever, and METER BRIDGE's demo
  motor froze all ten meters on the emulator. The page runs no motor
  (2026-09-26).
- The rotor observer's demo on an emulated board ran the record's placeholders
  (J 2e-5, l1 0.1, l2 100): sensorless from standstill lost the rotor in 27 ms
  on the drive core, and the hold's hand-over started on a free-running
  estimate (-38 rad/s) and tripped at 70 A. Commissioning's arithmetic
  (l1 0.025, l2 7.9, 0.16 V at fs/2, crossover 180 rad/s) and the held frame as
  the estimate: 1 600-2 000 frames, no trip, native and emulator (2026-09-26).
- The emulated heat was one lumped node, the MCU die 8 K over it; the
  observer's model puts 0.666 W through 22.5 + 40.5 K/W, 42 K. Both dies read
  colder than modelled and the NTC's inversion (x12 at 30 s, x27 at 2 s)
  threw the V patch to 5 C. The plant runs thermal.c's network as truth
  (world_heat.c): the NTC within 0.25 K under the demo's 30 A, driver U 26-100 C,
  CONVERGING in 4 min on native (2026-09-26).
- The attitude's tumble turned 140 and 280 deg/s on the emulator and native
  (2.56 s); the stand-in stepped it a read, a turn in 1.28 s at 200 reads/s.
  25.6 s on the clock: 31 deg/s (2026-09-26).
- Renode prints a peripheral's C# compile error on its console only: the
  emulator "did not answer" and the rig fell back to the stand-in (2026-09-26).
- The emulated bench's rotor: a `free` load carries no inertia, so the 2e-5
  kg m^2 motor alone reached no-load in 0.2 s; the demo's 8e-3 flywheel is a
  drag-free `rotor` load (2026-09-26).
- ROTOR OBSERVER on the emulated MCU drives its plant through the converters
  (heat, NTC, SOA as on the stand-in). Sensorless held a 10 A reversal through
  zero; the clamp through zero ran the estimate to 1e5 rad/s, and the observer
  chain lost lock at the clamp's acceleration (0 at 2 500). I/f cannot carry
  the flywheel: on its current spring zeta is ~0.001, it rings and slips. The
  demo: aligned once, then sensorless, zero crossed at 10 A, the clamp above
  1.5 w_hi, the brake proportional: 224 s native, 1 700 frames on the
  emulator, no trip (2026-09-26).
- drive.c: in a command frame below the back-EMF's speed the estimate is the
  frame; into hold from sensorless above it, the frame starts on the estimate
  (a jump to the setpoint's angle at speed slipped poles) (2026-09-26).
- drive.c read the theta setpoint only as HOLD began: commissioning's three
  angles, the machine's ring test and the stepper and servo, all writing it
  mid-hold, held one angle on the firmware while the stand-in's frame
  followed. The frame moves by the setpoint's change now, the ramp's travel
  kept. The motion verbs waited on the wall's clock: at a 20th of real time
  the emulated flywheel was read mid-swing; the board's now - the servo lands
  30, 60, 0 deg within 0.2 on the emulator (2026-09-27).
- The observer box's error beside a newer estimate: the page's sample replaced
  the state, then the model, a request apart; one update now (2026-09-26).
- One heat clock for every world but the bench: `coaxial.model.thermal.HASTE`
  10 on the stand-in, the plant and the emulated observer (thermal op 13), a
  step 0.1 thermal s at any clock. On a 100 ms wall poll the clamp lagged a
  leg's 1.4 s and the derate cycled 0.12-0.9. A plant hasted from power-on
  ran the MCU's node a kelvin ahead of an observer set at the open, and the
  dies' anchor dragged the legs 1-6 K: the rig sets both at once. THERMAL
  OBSERVER's load is the demo motor's 30 A rms on both, 12 s on, 24 off; the
  MCU's die reads in the off phase. `tools/dev/ab.py`, the page's reads over
  the same board seconds: 67 faults against the stand-in to 3 on native
  (2026-09-26).
- The papers on the stand-in against the emulator (`make_notebooks.py
  --compare`, 2026-09-27): a software-clocked task gave no record while a
  drive held the converters - the meter reads the latched triple's phases
  now, as it did the link and the NTC; a record's codes were trimmed on the
  board and again on the host - a phase at rest read -8.8 A - once now, at
  the source, the TIM1 clock's too; the host's tare wrote a trimmed burst's
  mean as the whole offset, a second tare undoing the first (-51 A after
  one on the stand-in's offsets) - folded into the offset now; the host dropped
  MINOR 7's sensor rows from the layout; commissioning, the ring test and
  the machine's arming waited on the wall's clock, the emulated flux spin
  never reached 300 rad/s (lambda 0.00069 against 0.00546 V.s); the
  stand-in's link gave t1.5/t3.5 as 1750/4083, not the part's cycles, and the
  console's frames on all three ports.
- The thermal observer's NTC anchor inverts the whole miss through the
  element's lag (215 s) at every sample, a miss standing from sample to
  sample included: at the design's 30 s a standing miss moves the leg
  patches 11.9 times itself, at 1 s 26.7 times. On native a sample a
  thermal second left the legs 2-10 K under the world's, HEAD's firmware
  too. Inverting only the miss grown since the last sample moved the
  identification's air scale off its tuning (0.90 against 0.5 after four
  cycles). A sample was also folded in on every slice of a poll: once now
  (2026-09-27).
- native:// had no console: after 0x48 gave the line back a board went
  silent for every later session in the process, the tty's next page on the
  stand-in. The fake board takes 'm' again (2026-09-26).
- The emulated A1335s answered bare twelve bits and Clevel/Cinj read 0: the
  part's register identifiers (ANG 5, TSEN F, FIELD E, the stand-in's from
  the bench) and the unmodified board's 0.06 / 0.77 V now (2026-09-26).
- The stand-in wrote CubeMX's DTG 19, read its die channel at mid-scale
  (545 C) and its MCU die under the drive, gave the triple's codes and the
  trigger unarmed and the STO pilot with the AFE off; native read its 5 V
  rail at 0 V. Each now as the board reads it (2026-09-27).
- The emulator's cost is its register accesses and interrupts, not its
  models: 1.24 M accesses a virtual second idle at 0.9 us each, 17 a period
  under the drive at Renode's floor. Cut a pass: HAL_RCC_GetSysClockFreq's
  7 RCC reads (the IMU's poll), AFE_ON's 3 IDR reads (a shadow); the idle
  core at 50 MIPS, the quantum 0.5 ms. Idle with the AFE on 2.8 -> 1.0 wall
  s a virtual s, real time. The STO chain: a step an edge cost 3.4 us,
  50 000 a virtual second, now batched at 1 kHz with the edges' instants;
  the drive's ISR counted as the chain's time put the keepalive 46 us apart
  and the observer's slice 850 us (the part: 9 and 190), so the chain's
  clock is main()'s instructions stretched by the interrupts' share; the
  slice broke the hold's regime every 100 ms and the chain never held, so a
  longer gap in a held regime runs from the held state and the hold resumes:
  195 -> 40 ms a virtual s, the drive 3.1 -> 2.65. Nothing from
  SyncPCEveryInstructionDisabled or -O2 on the sample path's board files
  (2026-09-27).
- CI's runner ran the emulated board slower than the scale the host slept
  by: the STO test's 0.05 board s settle ended before the chain had let go,
  BIF stayed latched through the clear, and the Release image's board group
  ran past 240 s. An emulated board's sleep reads its virtual clock as it
  goes; a group has 480 s (2026-09-27).
- CI's firmware job red on and off since 2026-09-27, red four times from
  8112407: conformance's Renode exited on a DllNotFoundException for its
  world library (build/hosttest/world_emu_<pid>.so). Each test group's
  process swept every world library not locked before building its own;
  Windows locks a loaded one, Linux locks none, and a group's sweep
  unlinked another's before its Renode loaded it. The sweep removes only
  libraries whose process has ended (`tools.emu.world.sweep`); the log's
  tail is on the commit as a comment, readable without a login
  (2026-09-28).
- The drive's ISR called newlib's cosf and sinf ten times and atan2f four
  times a period (the observer: the PLL's angle twice over, the lead, the
  blend's two angles) and lrintf seven: drive_sincos, an atan2 polynomial
  (1.7e-6 rad) and an add-and-truncate put the emulated core's ISR at
  1 445 cycles a period from 1 876, 123 -> 99 M instructions a virtual s.
  The wall barely moved, 3.0 -> 2.95: a period's 57 us are its 17 register
  accesses (~1.3 us each on Renode's IO path), the exception (4.5), the
  plant's step and the conversions (3.5) and the instructions (~10); the
  skip 0.6, no translation flushes (~1 000 blocks a virtual s). Real time
  under the drive is out of the emulator's reach as the firmware stands:
  every access is the ISR's own (2026-09-27).
- ROTOR OBSERVER on an emulated board, A/B'd against the stand-in over the
  demo (tools/dev/ab.py, then the drive's state every 50 board ms through a
  cycle on all three): the can's wobble is the low-speed estimate. The
  innovation |eps| over the rock is 0.72 on native, 0.78 on Renode, 0.05 on
  the stand-in - whose sensorless mode is a stub (simulated/drive/plant.py:
  `_converge` pulls theta_hat onto the rotor, `_ih` a closed form) - and
  0.2 on all three at speed, where the back-EMF observer holds. The part
  samples at the counter's top (CCR5 = ARR - 15, the low switches on for
  the shunts); an fs/2 injection sampled there sits on its triangle's
  midpoints, and the demodulator's gain is (n - 1) / n of the model's:
  none at `drv_inj_periods` 1, which choose_injection picks (n >= 2 fails
  its f_inj >= 8 x the 2 500 Hz current loop). The rotor held on the d-axis
  and the injection frame turned +0.3 rad, eps_amps reads -0.003 on native
  at n 1, -0.012 at n 2, -0.013 on Renode, a frame ahead negative as the
  physics has it with Ld < Lq; the stand-in's form says +0.009, and both
  emulators carry an offset of 0.005 A at zero error (0.15-0.4 rad).
  The noise: the bench's phases 0.35-0.41 A rms, the world's 0.02 A, the
  commissioning's assumed `drv_sigma_i` 0.05. n 2 with the loop at
  1.5 kHz alone did not bring the rock's |eps| down on native (1.07). The
  Renode plant read the compares at its one step a period and showed them a
  period early, the demodulator's sign turned: it now steps half a period,
  the compares landing at the underflow and TRGO2 at the top, as
  board/native has it (2026-09-27).

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
