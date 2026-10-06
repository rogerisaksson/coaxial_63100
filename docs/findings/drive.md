# Findings: the drive

The drive and its observers on the stand-in, the emulator and native://, the
rotor's pages and their demos. The board's own are in
[FINDINGS](../FINDINGS.md).

- ROTOR OBSERVER's rotor is the flywheel (J 8e-3, b 5e-4), not the bare 2e-5,
  and the demo's loop is designed on it. The cycle, the bench's word: up each
  way against a propeller on the clamp, 0 -> 2 771 rpm at 50 A and 0.85 kW; a
  coast at iq 0 on the drag and the propeller; a brake; a load at the continuous
  rating held at 993 of 1 000 rpm on 25 A (2026-09-28).
- The stand-in's hold turned its vector at theta_sp + omega_target t: a new rate
  jumped it 1.001 rad. drive.c's command_frame now, ramped and integrated:
  0.0001. The angle page's stepped command kicked the held rotor, +-0.9 deg at 8
  Hz, 832 turnings in 65 s; a turning vector, 0 (2026-09-28).
- The stand-in faded torque linearly with speed, half of kt I at half the
  no-load speed and the rest in no account; its current is held by the link now,
  |v| <= vdc / sqrt 3, as the world core's: 1 771 -> 2 771 rpm, 0.61 -> 0.85 kW
  at 50 A. With the stage off its rotor stood (2 451 rpm for 18 s); it coasts
  (2026-09-28).
- With the stage off the firmware's observer lost the rotor on the emulator: 3
  308 -> 238 rpm while the flywheel turned on, stuck through the brake. The demo
  coasts at iq 0 with the bridge on. The stand-in's tracker follows the model
  off: a gap (2026-09-28).
- Near standstill the firmware's estimate swings 100-150 rpm a sample, on native
  and Renode alike: the estimator, not the emulator (2026-09-28).
- The rotor page's mark, the angle over the pole pairs, skipped a pitch (51.4
  deg) each electrical turn. The drive counts theta_hat's turns (op 0, MINOR
  24): STEPPER +57.8/-53.7 of 60, 3.2 a frame at most (2026-09-28).
- The thermal page's load on a board was the demo vector at the watcher's 0.14
  Hz: on native the current sat in a leg for seconds, the hottest leg changed 9
  times in 20 s, 91 % apart. At 50 Hz of the board's time the legs rise
  together, U and W within 1 K and V 4-5 K under, as on the stand-in
  (2026-09-28).
- The demo's STEPPER at 15 mechanical degrees a step - 105 electrical -
  lost its rotor (180 asked, 580 turned); 45 electrical, eased: 63 of 60
  (2026-09-28).
- The demo's loaded speed changes spool, waiting on the rotor, 800 rpm/s at most:
  164 rpm/s the first half second where a constant rate stepped 552 on. QUAD's
  0.4 s stabs made 49 changes a minute and a 48 A step; 16 and 22 now. The
  feed reads the drive every pass: 16.2 states a second of 8.8 (2026-09-28).
- QUAD on its four stand-ins: the third flight never left its hover, the
  hover's share 0.42 over a room of 0.4 since the dry loss's refit; the burn
  stopped 0.44 m up and crept 3.1 s to its mark; the lift trailed its ramp
  0.4 m; the landing dropped its last 3 cm; the fall's and the landed
  spool's rotors, braked to their idle at the clamp, spent 0.16-0.25 of the
  envelope. The room is the throttle's point less a flight's 0.3; one
  altitude loop, 5 rad/s about a height and its rate; the burn flown down a
  speed for each height over its mark, within 2 cm of it 1.6-1.8 s on; the
  thrust run down as the propellers do. Four flights on end, the lift at
  1.50 m, down still, the share flat on the floor (2026-10-05).
- QUAD's routine, `machine.aerobatics`' card on `machine.flying`'s one law: a
  pirouette, an orbit and a corkscrew to 8 m, a roll and a flip over a toss,
  the way back down, full tilt, the burn - 59.9 s on ideal rotors, 0.5 g at
  most outside its turns, the pole 2.7 m off; on four stand-ins no board
  throttling, SOA 0.50 at most in its figures. On the way: the heading's loop
  at 16/8 on the discs' drag swung the rotors 1 470-2 080 rpm at the clamp in
  a pirouette of 180 deg/s (4/4, the turn fed on, 120 deg/s: 16 A); the spot's
  at 4/3.2 rang an orbit's bank 19-31 degrees on 14 of margin (1/1.6, the air
  fed on: 24-25); a roll on rotors run down as their propellers do went 1.6 m
  aside whichever way the thrust was cut - the collective bears the weight
  over a turn begun and ended alike; the thrust slewed 15 N/s swung a hover
  0.4 m; a pass of the page 14 ms and one in twenty 50, the rows asked by the
  wall's clock: the flight's clock is its passes' sum (2026-10-05).
- The stand-in's thermal air took the rotor's rpm by the bench motor's 7 pole
  pairs: QUAD's 63100, 14, cooled its boards on 2 941 rpm at 1 471; by the
  record's now, as `board_thermal.c`. Its four observers were STABLE from
  49-55 s of a flight, 66-70 with the routine - air 1.00, capacity 1.00-1.01,
  the room 25.1 of 25.0 C; the page said `4 of 4 converging` whatever they
  were (2026-10-05).
- QUAD on its pack, 15 cells: 63 V full, 0.12 ohm, 0.22 A h - a demo's. A
  flight takes a fifth of it; full tilt and the burn 1.5-1.85 kW, the bus to
  56.2-56.9 V; spent at a fifth it comes down from wherever at 2 m/s and is
  changed on the floor in 3 s, the observers kept. At 63 V the gate stage's
  dump is the envelope: on the bench's still air, 8.33 K/W, a hover stood at
  0.65-0.72 and every board throttled by the corkscrew; the record says a
  propeller's wash, 2.5 K/W - an assumption - and the stand-in's truth is
  laid on it: 0.52-0.60 in the figures, the hottest node 77-116 C. The rotors
  are asked the share of their pull the envelopes leave - the least room 0.08
  under a throttle's point, of 0.3; what the frame is sped up on and a fall's
  stop is planned on, never what a stop may take: 0.71-0.81 at most over
  three flights, none throttling, flown from the first hover with the
  observers UNCERTAIN, the apex 36-39 m. Held to the collective itself, a
  burn spent its own stop and pressed the skids 2 cm into the floor; brought
  down the moment the share was none, it left its burn 3.5 m up. A row's
  height told from an eased one by what it moved a pass, a 50 ms pass took
  the landed row for 9.6 m/s and launched the frame 3 m: a row brings its
  height's rate and pull. A rotor's speed asked 700 rad/s^2 at most: stepped,
  a pass was the whole clamp and 0.2 of an envelope for a read (2026-10-05).
- QUAD's course, `machine.course`, flown after the routine has warmed the
  boards: 14 gates of 3.4 m on a line of 141 m - an alley of trees, a house's
  ridge, round a mast, between two cars, a slalom. On ideal rotors a lap
  21.6 s, 10.7 m/s and 69 degrees at the most, every gate within 0.43 m of
  its middle on passes of 10-50 ms; on four stand-ins 28 s, 0.20-0.28 m over
  three flights, the envelopes at 0.54-0.82, the rotors asked 58 % of their
  pull at the mean through a lap - never all of it, never none - none
  throttling. What it took of the law: a row's own pull (a bend's, asked
  0.2 s ahead) and a nose free of its heading - a turn of the frame is the
  discs' drag's, 10 N of thrust apart a N m: held on the heading the rotors
  were at their clamp 12 % of a lap and two boards throttled at 0.94, free
  3 % and 0.82; the discs leant against what is asked up, not all of
  gravity (at a crest a bend had 0.6 of its pull); the tilt's loop on the
  discs' axis alone - a nose 66 degrees behind its heading turned the tilt's
  torque round and the frame went over - its miss answered as 0.6 rad at the
  most: whole, a lean turned back asked more than the rotors slew and the
  page's frame swung 75, 87, 121 degrees over, 4 m lost. The lap's pull is
  the envelopes' share of its lean: planned on the law's reach it never
  eased. A flight is begun on 0.7 of the pull: an idle board is at
  0.52-0.56, not cool, and waited for all of it the page stood 120 s on the
  floor. The stand-ins' rotors and heat are stepped the pass's seconds
  (`SimulatedDrive.paced`, `fast_forward`): on the wall's clock a pass 0.25 s
  late turned the rotors those seconds under a setpoint meant for 50 ms, and
  under the gate's load a gate was passed 2.7 m off; paced, 0.22 m alone and
  0.25 m beside fourteen busy processes, the envelopes at 0.74 and 0.79. A
  pack gives the routine 21 % of itself and the course 27 %: spent on a lap
  of every fourth flight, ended at that lap's finish. Spent on a flight's
  way down the card went on to the next flight's, that flight never flown;
  the finish's gate stayed lit through the next flight's lift: a flight's
  own way down, the first gate (2026-10-06).
- A starved QUAD page's every pass is its longest, 50 ms, and stepped whole
  the frame rang on its rotors: the corkscrew at 30 A where 24 at 45 ms,
  the envelopes' share 0.22, `home` never held - 41 s on it with the feed
  every 60 ms, the pack at 67 % - and CI read full tilt's kilowatt as 814 W
  off a frame. A pass is its steps, 25 ms at the most (`flight.passed`):
  the same feed, the corkscrew 26.2 A on 0.75 of the pull and over, `home`
  3.0 s; full tilt's kilowatt is 0.15 s of its two, read a step
  (2026-10-06).
- The rotor page, the bench's word: the magnets blurred through a 1/80 s
  shutter (208-386 cells, 0.10 a frame at most; the smear's ring flipped
  0.29), the windings whole at their current's brightness and gone on a
  coast (0 cells), the mark one place among them (2026-09-28).
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
- The firmware's hold turns its vector at `accel` toward `omega_target` and stands
  still at none; the stand-in's turns at once. AFE_ON is refused under an armed
  stage, taken before it (2026-09-25).
- The ring test stamped each angle read before its ask: a stall under the
  gate's load swapped neck and head (1 Hz apart). Stamped mid-read
  (2026-09-26).
- An armed sync stays armed past drive.off: the meter is the injected group's
  until gate drivers op 3 gives it back (the emulated wire sweep, 2026-09-25).
- `pole_pairs` counted shaft travel over a wall-clock walk against the
  nominal travel: 20.80 for 21 (the rotor pulling in from its rest angle),
  and past the 0.25 limit on a busy CI runner (red 834519a). Now the
  command's and the shaft's travels, sampled together, after the first
  quarter: 21.00 with six cores busy (2026-09-23).
- The stand-in's bare rotor on a 2 A hold (k 0.735 N.m/rad, J 2e-5, b 1e-5)
  has zeta 0.0013: it rings at 30 Hz for seconds and a 25 Hz loop pumps it until
  poles slip (224 deg). A joint's damping, b 4e-3 (zeta ~0.5), holds 20 joints
  within 4.1 deg over three runs (2026-09-24). A 25 Hz loop cannot damp 30 Hz:
  that is the board's loop to do.
- Fitment by ring test, stand-in: a 2 A hold stepped 20 deg electrical rings at
  23.8 Hz (hip, J 3.2e-5) to 38.9 Hz (head, 1.2e-5); repeats within 1 %, J back
  within 3 %. The crossing count failed mid-suite (a read gap hid a pair: waist
  and neck swapped); the median half-period holds (2026-09-24).
- Device 12, the board's loop (MINOR 20): a joint's feedback loaded as slots,
  rows of 0.8 s at 20 and -20 deg, 100 Hz on the stand-in: within 0.05 deg,
  the last held. Four legs with `node_hz=100`: the squat's down ends on its
  level at 0.65 s, the board's 90 deg/s slew over 58. On the board it ticks
  in the drive's sample; not yet run on the bench (2026-09-24).
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
- At rest the injection's estimate ran away three ways, each measured on
  native against the world's shaft (2026-09-28). The loop fed w lambda forward
  off it: at 2 V and l2 261 the demodulator read the current it drove as
  angle, -8 000 rad/s and 16 A inside 20 ms; the loop's speed is the back-EMF
  weight's share of it now, held +-1.8 deg for 2 s. The weight came from the
  estimate: noise past w_hi handed over to a back-EMF of nothing, which read
  back the feed-forward; from the fundamental's |E| / lambda (2 ms) now, the
  core's case 2 243 -> 130 rad/s. The demodulator differenced currents each
  turned into its own frame: 12 A of the align's d current fed 4.4 of each
  correction back, a pole off, the demo's first up backwards to -527 rpm;
  stationary differences along the injection's axis, +-0.04 rad through it.
- Commissioning backed the injection to 20 dB after the filter: native's
  0.057 V left 2.5 rad an update, noise wrapped flat. It stops where an update
  keeps pi/6 now: 0.585 V on the record's noise, 30 rpm sd at rest; the
  demo's innovation near zero 1.08 -> 0.25 rad, its first up forward
  (2026-09-28).
- At no current the demo's flywheel swings +-20 deg mechanical over 10 s on
  native: the stiffness of ~10 mA held by the loop, an offset's (2026-09-28).
- The observer box's error beside a newer estimate: the page's sample replaced
  the state, then the model, a request apart; one update now (2026-09-26).
- The thermal observer's NTC anchor inverts the whole miss through the
  element's lag (215 s) at every sample, a miss standing from sample to
  sample included: at the design's 30 s a standing miss moves the leg
  patches 11.9 times itself, at 1 s 26.7 times. On native a sample a
  thermal second left the legs 2-10 K under the world's, HEAD's firmware
  too. Inverting only the miss grown since the last sample moved the
  identification's air scale off its tuning (0.90 against 0.5 after four
  cycles). A sample was also folded in on every slice of a poll: once now
  (2026-09-27).
- The stand-in wrote CubeMX's DTG 19, read its die channel at mid-scale
  (545 C) and its MCU die under the drive, gave the triple's codes and the
  trigger unarmed and the STO pilot with the AFE off; native read its 5 V
  rail at 0 V. Each now as the board reads it (2026-09-27).
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
