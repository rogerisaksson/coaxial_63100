# Findings: one stack for every drive

One motor, one gearbox and one inverter: the demand the stacks were sized
on, the candidates, what the one stack leaves open (`machine/drives.py`,
docs/DIMENSIONS.md). The drives' own sizing: [drives](drives.md).

## Demand

The demand the stacks were sized on is the controller's own (2026-10-04, the
walk from the squat, 20 s, each leg joint's torque by band):

| Joint | rms N m | under 50 Hz | under 5 Hz | Peak = servo clamp N m | At the clamp, % of passes |
| --- | --- | --- | --- | --- | --- |
| hip | 64.6 | 47.7 | 35.8 | 171 | 5.3 |
| knee | 60.4 | 36.2 | 19.3 | 171 | 7.4 |
| ankle | 44.6 | 30.8 | 17.9 | 132, 129 | - |

- 45 and 64 % of the hip's and knee's copper heat is over 50 Hz; every peak
  is its servo's clamp (171, 171, 132, 129, 52 N m).
- Its source is the setpoints: 422 of 6051 passes moved one over 0.5 deg; the
  knee's rate 914 deg/s rms, the hip's 653. 3-8 deg in a pass at the swap of
  feet (the landed foot's anchor, the pelvis lowered), 12 at a toe-off (the
  anchor released), and 1.5-7 deg a pass to and fro about them: a leg's
  stance share (`stance.legs`' b) following its sole's load read a pass at a
  time, flickering 0-900 N.
- Rebuilt from what was written: kd x the setpoint's rate 540 N m rms at the
  hip, the rotor's feed-forward 457, kd x the joint's own rate 89, kp x the
  error 65.

## Load band and governor

The soles' load through a sensor's band (`physics.LOAD_S` 20 ms, the lab's,
2026-10-04): the walk's touchdown 769 -> 253 N, its power 683 -> 560 W; the
hip's, knee's and hip roll's rms 63, 60 and 48 -> 56, 49 and 36 N m, at their
clamps 2.0, 3.6 and 1.3 % of the passes; 5 walks of 5 held (0.65-1.0 of the
pace) against 4.

A governor on the setpoints, each followed at 1000/s within 0.9 of its
drive's unloaded speed and the acceleration its clamp gives its rotor and
limb: took the torque over 50 Hz out (the hip's 48 of 48 N m under 50 Hz, at
its clamp 0.0 %). At 200/s, 5 ms behind in the loops closed through the
setpoints: 1 walk of 5 fell at 1.6 s; standing on the loads as read it fell
at 3.9 s.

## Candidates

One box and one inverter by smoke (2026-10-04): a candidate's stacks laid at
run time, the load's band, the rise and five walks as tuned at 0.65-1.0 of
the pace. Each rose and walked 5 of 5:

| Candidate | kg (drives) | Part types |
| --- | --- | --- |
| as built | 32.3 (16.2) | 23 |
| frame B's lamination at two stack lengths: 60 x 16, and 60 x 30 for the hips, knees, ankles, spine; KV 90; box B and the 50 A inverter on every drive | 28.5 (12.4) | 21 |
| the long stack on every drive | 30.1 (14.0) | 20 |
| the short stack on every drive | 27.7 (11.6) | 20 |

- Under the governor at 200/s the short stack and a middle one (64 x 22, a 74
  mm box, 75 A) fell at 1.6-1.7 s 5 of 5, the two lengths 3 of 5: its lag,
  not the stacks.
- The two lengths on their own demand (`drive_sizes`, the load's band): the
  hips', knees' and spine's peaks are the 124 N m clamp (A 1.50, the box's S
  1.45); the hip's and knee's rms 53 and 49 N m of the 69 the 50 A inverter's
  switches hold (T 1.36 and 1.13 at 1.5x, 0.60 and 0.50 at 1x; the short
  stack's winding holds 52); the knee's 1007 deg/s 1.17 of its volts.
- The margin as the clamp (2026-10-04): a peak that is its servo's clamp
  times 1.5 is over its drive by construction. Every servo's clamp its
  drive's peak / 1.5 instead (the two lengths' hips and knees at 83 N m, as
  built 136): rose and walked as far (9.5 m); the two lengths' hip and knee
  rms 40 and 33 N m (T 0.76 and 0.52 at 1.5x of the 69 held), at the clamp
  6.7 % of the passes, the knee's 729 deg/s 0.84 of its volts, the box's S
  0.97 at 1.5x. Falls, parries and standing within it: the scoreboard.
- Fit (`fit.py` on each, 2026-10-04): the two lengths stand 12 and 5 mm past
  the skin at the ankle and the knee against 21 and 14 as built; none past
  the shell but the elbow's 5 mm as before; the ankle's two drives clear of
  each other (6-7 mm into each other as built). Every inverter in its own
  stack, a 70 mm disc and no board apart: the elbow 8 mm past the shell, the
  knee 9 and the ankle 15 past the skin, no pair closer. The long stack on
  every drive: 13 mm past the shell at the elbow, 6 at the hip roll, 4 at the
  shoulder, its gearbox 9 mm into the upper arm's tube.
- A fall asks more of a box than any here takes (`gait_montecarlo`'s shove,
  the worst of each): as built the ankle's 2226 N m against its pair's 527,
  the knee's 658 against 289; on the two lengths the knee's 763 against 128,
  the ankle's 1050 against 233. The rotor spun up through 1:30 by the blow,
  not the servo's torque.
- Scoreboard (the load's band, the stepping foot by its landing, as tuned,
  2026-10-04), every rise and walk 100 % on each:

| Candidate | Score | Held % | Within peaks / 1.5 |
| --- | --- | --- | --- |
| as built | 229.0 | 86.4 | - |
| short stack on every drive | 264.6 | 84.8 | 282.8, 82.1 % |
| the two lengths | 533.6 | 85.7 | 526.5, 82.2 % |
| the long at KV 105 | 425.7 | 82.2 | - |

  The two lengths' cost is the fall's: the knee's box asked 5.96 times its
  momentary by the long stack's rotor (0.094 kg m^2 at the joint), the
  short's 0.062. The short stack's winding holds 52 N m: the hip's rms 45 at
  its peak and 39 within peak / 1.5 (T 1.26 at 1.5x, 0.56 at 1x), the knee's
  38 and 32. The governor on the setpoints at 1000/s: 911.4 and 61.0 %; at
  half its clamp's acceleration 960.2 and 56.5, the walks 35-72 %. Not taken.

- On the walk with the slip gone ([feet](feet.md), 2026-10-04; smoke at 0.85
  strides/s): as built the hip's, knee's and ankle's rms 39, 41 and 35 N m,
  332 W; the short stack on every drive 30, 25 and 26 (the hip roll's 42 of
  the 87 it holds), within its peaks / 1.5 28, 24 and 26. Sized on its own
  demand over the scenes (`drive_sizes`, the run at its 27.7 kg folded in):
  the hip's rms 37 of the winding's 52 N m (T 1.13 at 1.5x, 0.50 at 1x), the
  knee's 32 (0.84), the ankle's 33 of 95 (0.28), the hip roll's 46 of 87
  (0.64); the knee's 1059 deg/s 1.23 of its volts at KV 90 and 48 V; the
  peaks the 124 N m clamp.

## Stacks as built

`machine/drives.py`'s record of 2026-10-03 and -04: two frames.

| Frame | Motor | Box | Inverter | Joints |
| --- | --- | --- | --- | --- |
| A | 68 x 30 at KV 110 | 84 mm | 100 A | spine, hips, knees, ankles |
| B | 60 x 16 at KV 90 | 64 mm | 50 A | the rest |

- Each KV in its window of 1.5x on the walk's torque at its speed; each box's
  momentary 1.5x its peak, its rated over its rms.
- At 1:30 frame B's windows met at KV 76-95 but the ankles' (x1.43-1.48),
  their copper 1.05x their rms, the hip roll's 1.17x. A's spine on 100 A to
  KV 104, the hip 52-85, the knee none: x1.41 at KV 90, its first step's 144 N
  m at 704 deg/s, 2.0 kW, on 100 A at 48 V.
- The hip's and the knee's KV against the gait Monte Carlo (KV 120 is x1.09
  over the clamp, 960 deg/s):

| KV hip, knee | Held % | Knee run into its SOA |
| --- | --- | --- |
| 90, 70 | 70.7 | felled 2 of 3 |
| 120 | 74.2 | 1 of 3 |
| 125 | 71.8 | - |

- The spine at KV 40 on 50 A, 360 deg/s, fell in the knee's SOA.
- One KV for A: the spine's amps at its clamp 1.05 at 110, 1.15 at 120; the
  hip's and the knee's 1.26 at 110 (volts 1.15 and 1.23), 1.38 at 120 (1.06,
  1.13); at 100 their volts 1.31 against 1.19, a parry's 1259 deg/s binding,
  not the run.
- The waist's and the shoulder's 44 mm boxes stood 2.79 and 1.44 times their
  momentary rating; the 64 mm 0.91 and 0.47.
- The ankles on A for a run's headroom at 55 kg (on B 1.32 of its amps, 5.3
  of its heat, 1.27 of its box; on A 0.80, 0.73 and 0.56); the hip yaw, the
  hip roll and the ankle roll on B (0.62, 0.93 and 0.97 of their amps; on A
  the hip roll's stack stood 19 mm into the pelvis frame and the hip yaw's
  inverter 15 into the spine's); the rest on B with B's box, box C gone with
  frame C.

## One stack on every drive

Laid 2026-10-05: the most simplicity, the fewest gearbox and electronics
variants. 60 x 20 mm at KV 90, the 64 mm box at 1:30, the 70 mm 50 A inverter;
21 of each, 20 part types against 23; 28.4 kg against 32.3, the drives 12.3
of it against 16.2.

| Stack length | Score | Held % | Note |
| --- | --- | --- | --- |
| as built | 250.5 | 82.9 | - |
| 16 mm | 292.5 | 83.7 | the walk at 0.65 strides/s 76 %; within peaks / 1.5 299.9, 81.7 % |
| 18 mm | 325.0 | 85.8 | - |
| 20 mm (laid) | 318.7 | 84.9 | every rise and walk |
| the hips' pitch alone on 60 x 30 | 351.5 | 83.0 | - |

- The length is the hip's: its rms 37-42 N m over the scenes, the winding
  holding 52, 59 and 66 N m at 16, 18 and 20 mm (T 1.13 and 1.15 at 1.5x on 16
  and 18).
- On 20 mm, its own demand and the run at its weight (`drive_sizes`): T 0.90
  at the hip, 0.79 at the knee, 0.45 at the hip roll, 0.21 at the ankle; the
  hips', knees' and spine's peaks its 124 N m clamp (A 1.50, P 1.24-1.35, S
  1.45); the knee's 1034 deg/s 1.20 of its volts; the elbow, the neck and the
  head feel their rotors 7.8, 2.9 and 33 times their loads.
- The get-up as it stood stayed down: felled by the shove it came from the
  sit into the crouch pitched 9 deg where 24, the CoM 52 mm ahead of the
  pelvis where 134, bore 236 of its 279 N on the feet in the squat and fell
  forward out of it, 3 tries, from each of five lying states. The keyframes
  onto the feet searched on it (`getup_search --table f`, 96 candidates): 2
  walked, from all five, the rest from none; the best walking again at
  23.6-25.4 s, 1 try, hands out 5.3 s, bowed 5.5, the arms thrown up to 170
  deg. Folded to 170 deg the ankle roll's drum meets the femur where it met
  the knee's board (test_gynoid). Felled by the shove: up and walking at
  33.0 s, 2 tries.
- Fit: 9 and 3 mm past the skin at the ankle and the knee against 21 and 14;
  6 mm past the shell at the elbow against 5; 2 at the hip roll.
- On the walk of 2026-10-05 (the stance leg a strut, the catch at 7 cm, the
  lean before the shift; `drive_sizes`, 1.5x): T 0.87 at the hip, 0.67 at
  the knee, 0.32 at the hip's roll where 0.90, 0.79 and 0.45; the rms 41, 36
  and 41 N m where 42, 39 and 49; the knee's parry 1007 deg/s, 1.17 of its
  volts, where 1034; the peaks its clamp as before (A 1.50, P 1.24-1.35, S
  1.45 at the hips, knees and spine).

## Motors as bought

2026-10-05: complete units off the shelf, no frameless kit; two motors
(motors cost); one board and one gearbox (variants of those are the cost).
The frame before, 60 x 20 mm at KV 90 by a law fitted on T-Motor's pages, is
no part: at its 4.1 N m its rotor was 0.75e-4 kg m^2 where every outrunner
found with that torque has 1.6-2.6e-4.

| Motor | mm | g | KV | mohm | N m, 180 s or rated | Peak | USD |
| --- | --- | --- | --- | --- | --- | --- | --- |
| T-Motor U8 II Lite | 87 x 27 | 253 | 85, 100 | 134-141 at 100 | 3.0 at 31 A | - | 300 |
| T-Motor U8 Lite | 87 x 27 | 243 | 85 | 225 | 2.1 at 19.1 A | - | - |
| CubeMars R80 | 87 x 27.5 | 354 | 110 | 125 | 1.3 | 4.0 | 259 |
| CubeMars RO80, a kit | 93 x 26 | 352 | 105 | 120 | 1.3 at 15 A | 4.0 at 50 A | 115 |
| EaglePower LA8308 | 92 x 28.5 | 336 | 90 | 186 | 2.3 at 22 A | - | 45-90 |
| T-Motor MN6007 II | 67 x 26 | 159 | 160 | 178 | 1.2 at 23.7 A | - | 130 |
| CubeMars R60 | 69 x 26 | 248 | 115 | 300 | 0.8 | 2.4 at 40 A | 178 |

- The U8 II Lite at KV 100 on the 15 drives whose joint asks over 50 N m (the
  legs, the trunk); the MN6007 II on the arms' and the head's 6: USD 5 280 of
  motors where 21 U8s are 6 300; the LA8308 in the U8's place 1 680 and 1.2
  kg more.
- No T-Motor page gives a peak: the U8's 4 N m is CubeMars' for the same
  36N42P stator, the MN6007's 1.9 the RO60's by its stack.
- KV 100: the knee asks 860-1007 deg/s and at 1:30 has 960 at 48 V, at KV 85
  816. A joint's torque times its speed over the pack's volts is its amps
  whatever the winding; 42 poles at 5 000 rpm are 1.75 kHz under 50 kHz of
  PWM, no room for a higher ratio.
- On them (docs/DIMENSIONS.md): 108 N m and 0.177 kg m^2 at a hip where 124
  and 0.071, holding 74 N m where 66 (Km 0.26 against the law's 0.23); 51 N m
  at an arm's joint; 30.8 kg where 28.4, 14.7 of it drives.
- The 63100 on every drive (2026-10-05): 100 mm behind the U8's 87, 33 wider
  than the MN6007. Past the clothes standing (`tools/sim/fit.py`): the
  elbow's stack 20 mm, the shoulder's 15, the hip yaw's 6, where the 70 mm
  stacks had the elbow's 6 and nothing else. The spine's, the neck's and the
  head's lie in the back on its middle line, flush (a hand's breadth off it a
  100 mm disc is 17-20 mm out of the shell); the head's under the neck's: in
  the skull, 0.2 kg more of head; one fall of the suite put the head on the
  floor at 1.49 m/s.
- Their rotors (2026-10-05): fed forward whole by the board
  (`physics.ROTOR_FF` 1), a standing start's first 50 ms had the knees and
  hips at their clamps; it fell back in 3 starts of 12; with the old rotor on
  the same body, none. At a quarter of it: no parry in 5 starts; form at three
  paces: the knee 7.1, 6.9 and 9.6 deg behind the plumb line, landing at
  28.6, 29.7 and 26.4; 428, 388 and 623 J/m where the frame before had 393,
  384 and 604. At none, the landing knee 30.0 at 0.65 strides/s, the form's
  30. The run on them: [run](run.md).

## Heat and power

- In still air (2026-10-05; `drives.COOLING` 1.0 where the oil assumed is
  0.3; the boards as built, 10 min of heat a run, the windings the hottest
  node):

| Gait | Hip C | Knee C | Board switches C |
| --- | --- | --- | --- |
| walk 0.85 strides/s | 62 | 52 | 41 |
| walk 1.0 strides/s | 94 | 77 | - |
| run asked 1.5 m/s | - | 114, derated 0.64, down at 6 min | - |
| run asked 1.75 m/s | - | 113, 0.69, down at 4 min | - |
| run, cooling 0.6 | - | 89 | - |
| run, in oil | - | 57-64 | - |

  The walk runs on in air; a run lasts minutes, enough (the FETs' safe area
  the concern). The model means a switch's loss over an electrical turn:
  stalled, one carries all of it.

- What the walk draws (2026-10-05, motors as bought), from 2 s into it; on the
  page it reads natural, momentary power fairly low, a smooth flow at the
  lower pace:

| Strides/s | m/s | Mean W | Median | 95 % | Max kW | Rows over 1 kW | J/m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.65 | 0.48 | 193 | 129 | 570 | 2.66 | 7 of 591 | 405 |
| 0.85 | 0.65 | 276 | 177 | 894 | 4.1 | 22 | 427 |

- Air-cooled (2026-10-10): a board's back on an aluminium holder over a third
  of it, on its motor or gearbox, its only cooling (`drives.COOLING` 1,
  thermal_app_t's `joint` on the humanoid's worlds). The U8 holds 40 N m
  indefinitely where 74 in oil, the MN6007 19 where 35; the walk's smoke and
  the look suite identical to the digit as in oil.

## Which drives want another box and motor

`drive_sizes.py` on the rigid world, 2026-10-11 (the rise, 24 s of walk and the
scenes, each number 1 at a part's limit at the margin 1.5, the parries' 1.2):

| Kind | Peak / has, N m | RMS / holds | deg/s / has | A | P | T | J | S | V | Ratio window |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| hip | 108 / 108 | 39 / 40 | 886 / 960 | 1.50 | 1.33 | 2.10 | 0.18 | 1.27 | 0.92 | 43..70 |
| knee | 108 / 108 | 39 / 40 | 1009 / 960 | 1.50 | 1.35 | 2.14 | 1.20 | 1.27 | 1.05 | 44..27 |
| hip roll | 129 / 180 | 39 / 68 | 327 / 575 | 1.07 | 0.61 | 0.76 | 0.54 | 0.90 | 0.57 | 44..68 |
| ankle | 147 / 197 | 43 / 74 | 1025 / 1055 | 1.12 | 1.01 | 0.76 | 74 | 0.94 | 0.97 | 24..3 |
| ankle roll | 96 / 129 | 20 / 48 | 652 / 1611 | 1.12 | 0.37 | 0.38 | 93 | 0.94 | 0.40 | 11..2 |
| spine | 108 / 108 | 6 / 40 | 554 / 960 | 1.50 | 0.76 | 0.05 | 0.18 | 1.27 | 0.58 | 7..71 |
| hip yaw | 52 / 108 | 14 / 40 | 372 / 960 | 0.72 | 0.24 | 0.26 | 2.00 | 0.60 | 0.39 | 15..21 |
| shoulder | 40 / 51 | 4 / 19 | 447 / 1536 | 1.17 | 0.18 | 0.10 | 0.64 | 0.47 | 0.29 | 9..37 |
| elbow | 25 / 51 | 3 / 19 | 284 / 1536 | 0.73 | 0.12 | 0.04 | 6.95 | 0.29 | 0.18 | 6..11 |
| neck | 15 / 51 | 1 / 19 | 730 / 1536 | 0.44 | 0.21 | 0.01 | 2.69 | 0.18 | 0.48 | 3..18 |
| head | 0 / 51 | 0 / 19 | 1 / 1536 | 0.00 | 0.00 | 0.00 | 29 | 0.00 | 0.00 | 0..6 |

- Two categories would serve, never one a joint (the user, 2026-10-11): the
  four hips and knees, which ask their clamp (A 1.50) and twice their
  copper (T 2.1) with the knee at the pack's volts (V 1.05) - a motor of
  half again the torque at the one box, not a ratio: the knee's window
  (44..27) has no ratio, more of it heats less and drags more; and the four
  ankle drives, whose rotors stand 74-93 times the foot's inertia through
  their rods (windows 24..3 and 11..2): a lower ratio, with the pair's two
  motors carrying the 147 N m the parries ask. The spine's peak and amps
  bind on the rise (A 1.50, T 0.05): a moment, not heat. The rest fits the
  U8 and MN6007 at 1:30; the head's drive carries nothing (0 N m, 29x the
  head's inertia).
- The drives stay as bought (the user, 2026-10-05); this is where a second
  box or motor would go first, hips and knees before ankles.
