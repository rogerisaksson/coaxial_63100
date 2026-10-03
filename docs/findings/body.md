# Findings: her body

The gynoid's build and drives, her buses, her falls and her get-up, her
drawing and her clothes. The board's own are in [FINDINGS](../FINDINGS.md).

- The gynoid in MuJoCo (55 kg, 26 drives, 1 kHz): on the line the legs stand
  in a V, 8 cm out at the hip for 3 at the foot, and need 0.773 m of 0.770:
  the IK clamped, a foot hung 3 mm off the floor and she tipped over it; the
  pelvis 12 mm lower. On one leg no stiffness held her (x5 4.8 s, x20
  unstable); the CoM fed back through the pelvis's target did (2026-09-25).
- A fall seen early (`machine.director`): the pelvis tipped past 12
  degrees and tipping on faster than 60 a second, or under 0.65 m, is past
  the walker's recovery; curled into the squat's joints over 0.4 s, the
  arms toward the fall, she lay as she landed. Her seat, back, chest,
  skull, arms and thighs are contacts (`figure.CONTACTS`): curled only at
  35 degrees over 1.5 s, the legs walked on and she lay with her torso
  through the floor. Into the hole at 13.7 degrees and 300 a second, 0.16
  s before she was down at 40, the hands took the floor 0.12 s before the
  head touched once, still on her left side and seat from 7.4 s (0.7 s
  after); off the rug she sat down backwards onto her feet (1.5 kN each)
  and rolled onto her back. Walking stumbles tip her 6-8 degrees, under the
  trigger (2026-09-27).
- Getting up in the joints alone (`tools/sim/getup_lab.py`): sat up from
  her back, she rolled onto her side (2026-09-27).
- The drives' callouts (HUMANOID page) dock at the viewport's edges, a
  leader to each pivot: beside her at 0.42 m they were three cells and
  crossed her stride (2026-09-27).
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
- Her hair on two hinges under her crown, 0.06 kg at 1.8 Hz: 17 degrees
  fore and aft and 6 aside as she walks (2026-09-28).
- The lace fall's head: the lace held her trailing foot, she dove onto her
  hands with her legs straight behind, her arms gave, and with the chin
  down 40 degrees her head met the floor at 0.85 m/s. The waist's turn at 0,
  20, 35 degrees: 2.02, 1.41, 1.33 m/s; the arms straight or on the
  forearms, the head down sooner. The head held up, the neck back 20, 40 or
  60 (falls.HEAD_UP_DEG): the chest and the hips take the floor, the
  head never (2026-09-28).
- After a fall the walk began with the pelvis still lowered, 12 cm, 8-10
  cm crouched: `Walker.reset` on landing (2026-09-28).
- The jeans' legs, softly stopped at 14 degrees, swung to 19, the shin 35
  mm out through their sides, the sneaker 64; stopped stiffly at 5, 8, 3
  (physics.HEM_*), a 74 mm hem on the vamp: the shin inside, the sneaker
  5, the other foot 12 in where 16 (2026-09-28).
- Falling, the legs' and trunk's drives shorted (`falls.SHORT_FALLING`).
  Up on a woman's 40/25 N m arms, sat,
  folded over the knees, after the rug and the lace in 25 s. Risen
  facing where she lay she fell 0.2-1 s into the walk: MuJoCo's pyramid
  cone grips along the world's axes; elliptic at impratio 10 she walks 13 m
  every way (2026-09-30).
- Folded over her knees getting up her trunk went 42-45 mm into them, 33-37
  squatting; colliding, the fold failed 3 of 3 until the knees opened 30
  degrees: 7 mm. Up, the walk kept its catch and speed from before the fall
  and held 6 s 2 times in 11; reset, 10 (`Walker.reset`). Her head's IMU
  picks the side she rolls over. Walking back the way she came, a joint came
  2.38 m behind the lens (`gynoid.Follow`) (2026-09-30).
- Her whole body collides (`figure.BODY`). The get-up, a plan the observer
  checks (`machine.planner`): on her heels from 18 of 22 kneels
  (`observer.OVER_FEET_M`), onto her feet ended crouched (`observer.EARLY`),
  the floor's boxes deep (`floor.DEEP_M`): 13 of 13 falls walk again 20-25 s
  after, 12 padded (2026-10-01).
- Falling, her reach pointed 45 degrees off her tip (the hole); turned at
  the waist past 20 off her front (`falls.turn`), 5. In 24 falls her head
  touched in 12, 688 N at most; unturned 10, 1170 (2026-10-01).
- Pads where falls land, 5 mm of gel (`figure.PADS`,
  `tools/sim/landings.py`): stiff, it bounced, the median landing 19.6 kN,
  bare 6.0 (2026-10-01).
- Down after a sill the spine bent back 98-101 degrees; stopped
  (`physics.STOPS`) 31-37 (2026-10-01).
- The humanoid page threw at a zoom of 1.1^3: a callout placed at its
  joint's height at rest, the joint past the drawing's edge, was written
  past the last row (2026-09-28).
- Her buses: five processes (the axis's 5 boards, each arm's 4, each leg's
  7), 0 bad frames; with the heat in each reply (21 bytes) a leg's wire is
  busy 430 us of a 1 ms pass. The loop runs 0.63-0.7 of real time here,
  the director's Python most of it, the lockstep 8 %. Each board keeps its
  heat (`machine.heat`: thermal.c's envelope on three lumped nodes, the
  5230SL through a 1:64 cycloid, heat 10 times the clock): walked at 0.85
  strides/s the hips' laminate spent 0.95 at 60 s and derated to 0.5; at
  0.75 and 0.65 it held 0.76 and 0.74. The director eases the pace from
  0.7 spent: 0.79 over 110 s, never derated. A knee warmed to 100 C (0.93,
  derated 0.73) walked on, 0.98 2.5 s later; in its SOA (on-resistance 50
  times) it dropped its gates under 107 N m, armed again 54 ms later, its
  derate back to 1 in 2.8 s, walking on. Held 88.1 % (the SOA and the hot
  knee 100 %). A halt's settle fell at 15, 15.3, 15.6 and 16 s, and at 15
  and 16 s at every commit back to the director's first (898e224): as it
  takes over, her centre of mass stands 91 mm outside the line between the
  feet; with the weight shifted before the lift, and the rear foot kept on
  its ball, as well (2026-09-28).
- Her drives sized (`machine.drives`, `tools/sim/drive_sizes.py` against
  the rise and 20 s of walk): three assemblies, a board behind an outrunner
  on a cycloid, coaxial - L the 63100 board (its parts 92 x 93 mm, a 100 mm
  disc) on the 5230SL at 1:64, 251 N m, 791 deg/s at 44.4 V, 1.5 kg, 100 x
  95 mm, for the hips, the knees, the spine's pitch and roll and the ankles
  (the latter inside the calf); M 50 A on a 43 mm stator at 1:76, 101 N m;
  S 20 A on a 35 mm one at 1:101, 30 N m - M and S estimated from their
  size classes. Every joint within its peak (the hips' 250 at 250.7), its
  rms within what its envelope holds for ever (the hip 61 of 73.7 N m, the
  board bolted to its housing), its speed within the unloaded (the knee 616
  of 791 deg/s); the assemblies ride their segments within their masses,
  the shank 2.15 of 2.65 kg. The cycloid shows the rotor 0.49 kg m^2 at an
  L joint, the model carries 0.05: walked with it the head goes 44.9 ->
  81.3 mm fore and aft, the strike 1550 -> 1031 N, held 83.8 -> 71.8 %,
  whatever L's ratio (36, 48, 64: 73.9, 71.4, 71.8 %). The frame fixes the
  product of that inertia and the copper's heat at a torque (J N^2 against
  t^2 / Km^2 N^2): the hips' winding stands 83 C at 1:64, 129 at 1:48, 210
  at 1:36. Drawn, the ankle's drum on the calf's back stood 3 cm proud;
  inside a calf 112 mm round only its ends show, as the knee's and the
  elbow's do (2026-09-28).
- The page's callouts: 34 cells a joint -> 20 in the tty's frame, the name
  on a patch in its drive's heat colour (`ansi.thermal_rgb`). What she trips
  on, drawn in its own ink: laid under the lifting foot it showed 0.1 s
  before she met it; on the page it is laid a stride on at the same phase,
  1.3 m ahead of her pelvis. The frames the page draws at 15 a second showed the
  same state again 65 times in 235 and the rest 0.036 s of her time apart
  on the mean with 0.022 of spread: her process runs 0.7 of real time in
  slices of 0.05 s. Played back 0.2 s behind the newest state on a clock
  of its own, its pace eased over 0.5 s, the states between blended: the
  same state again once in 234, her time a wall second 0.22 of spread
  where it was 0.62. The drives' six-field readings had cost the loop's `flat` twice
  its time: a float taken as it is, 0.53 -> 0.64 of real time
  (2026-09-28).
- Down, the fall braked, then cut once still and checked a bus's boards one
  after another, 0.2 s each (`machine.down`): lying she draws 33.2 W, her 25
  boards at 1.33 W, 105 W at the check's first five; the page drew 136-163 W
  lying, her tuck holding her curled, and 157 W given up, stuck falling with
  her drives armed (2026-10-02).
- Running on cranks, not reversing: each rotor's reflected inertia given and
  taken twice a stride, 1/2 J w^2 at 1.4 strides/s, the hip +-40 deg, the
  knee 10-120, the ankle +-25 at J 0.163 kg m^2: 8.6, 16.2 and 3.4 W a leg,
  56 W both, against the 445 W of copper walking at 0.85 - not worth a
  crank's mass and fixed range (2026-10-02).
- L wound 1.25 times its turns, 137 -> 171 N m at 100 A, 1520 -> 1216 deg/s:
  from the squat she walked 30 s, on the catalogue's winding she fell at
  9.45 s (the page's 9.47); the hips at the clamp 3.4-3.6 % of walking
  either way, the knees 7.2-7.5 -> 6.1-6.8, copper 524 -> 606 W. The hip yaw
  at 1:60 beside it, 77 N m: she fell at 5.82 s (2026-10-02).
- The hip's drives kept on the pelvis's side, its pitch on its axis; on the
  thigh 0.12 m down with rods to the pelvis they would add 0.038 kg m^2 to
  the leg's 1.685 about the hip, its ~140 deg past a crank's dead point (the
  ankle's rod spans 95, 39-41 deg at worst) and ball-joint play where
  hysteresis shows. The femur and tibia hung from their drives' gearbox
  collars (`mechanism.HUNG`), no longer through the drums' centres, 40 mm
  into the hip's; the pelvis's boom 30 mm round, 170 N m on it 147 MPa
  (2026-10-02).
- A wobbling plate (nutation) ball reducer against the planar ball stage, on
  L's 61 mm ball circle: held by a Cardan or a Rzeppa, one stage gets the
  same balls a ratio - 1:12 11 of 12.9 mm, 1:36 4.1 mm; compounded, its two
  faces 6 and 6, 1:36 on six 20 mm balls but 42-59 % efficient through the
  difference, near locking backdriven. The planetary into the ball stage
  kept, 0.95 x 0.97 (2026-10-02).
- Her jeans as MuJoCo cloth (`flexcomp`, 3.14.0): her 1 ms step 0.31 ms,
  with two grids over her shanks of 400 vertices 19.2 ms, 100 1.06, 100
  touching her body alone 0.47, 48 so 0.31 - a coarse reference live, the
  denim not (2026-10-02).
- The ankle's rod bent down the calf's inner back (`linkage.BENDS`), its
  ball on the heel's back 20 mm in: clear of every part over fit's 45 poses,
  2 mm of its drive at full dorsiflexion; straight inside the leg it stood
  51 mm out of her shell, 15 of her jeans, now 52 and 5 behind her calf.
  Its arms at 33 and 34.5 mm, the hub kept, the walk from the squat fell at
  13 s; the drive 15 mm ahead, two falls put her head down at 1.6-1.9 m/s;
  its crank outside, the thigh's boards met it sat back, 15 mm (2026-10-02).
- The ankle a parallel pair (`linkage.PAIRS`, the user, 2026-10-02): two M
  at 1:68, 80 and 150 mm under the knee, each a 12 mm carbon rod crossed
  from a crank on its drum's gearbox end at the knee's front, bent down past
  the drums, to a horn on the heel's back 12 mm out or 16 in. Her ankle's
  shell 32-36 mm round leaves any rod's line within 25 mm of the axis: on
  48 mm arms the balls stood 26-29 mm out. The pitch both together, lever
  0.75-0.91, 160 N m, 652 deg/s; the roll one against the other, 0.60,
  105 N m; each board's current the pitch's share plus or less the roll's,
  within its 25 A (`buses`, `physics.paired`). Inside her shell and jeans
  standing, fit clean over its 45 poses; the scoreboard 495, held 68.3 %
  (the L and its rod's 535, 64.5); her skeleton colliding 542, 62.3 %,
  never the floor: inside her shell, outside her skins' capsules, the toes'
  belt sank to it with her sole and felled her, a knee's drum landed past
  its gel, 935 N against 102 bare. Colliding, shoved past saving into the
  crouch her head met the floor at 1.03-1.66 m/s in 3 of 64 falls, without
  at 0.66-0.81 in 2; one of them with nothing of the skeleton touching -
  chance (2026-10-03).
- Every drive's output on its drum's axis from its gearbox's end
  (`tools/sim/fit.py`'s outputs, the user, 2026-10-03): the ankle's L had
  lain along the shin turning its crank about the knee's axis; the hip
  yaw's gearbox faced up, away from the hip; the elbow's and the wrist's
  belts turned about x off drums along the arm - across it the drums stood
  3-26 mm out of her shell, so a bevel pair at each gearbox's end
  (`linkage.BEVELS`); the spine roll's M 90 mm off its joint, nothing
  between.
- The hip a gimbal of parts (`skeleton.gimbal`, the user, 2026-10-02): the
  yaw's M on the pelvis turns a fork - a steerer, a crown over the pitch's
  L, legs before it 20 mm in and behind it 20 out - to the roll's bearings
  55 mm either side of the hip's centre; the roll's M rides the fork, its
  spur pair into the cradle round the L. Each drive's stator on the stage
  before its joint, a body of its own in MuJoCo (`figure.STAGES`), 4.6 kg
  off the pelvis, her 35.95 kg kept - its walk from the squat fell at 5.8 s,
  the scoreboard 578 against 495, the rises the worse, so their masses ride
  the pelvis till the walk is retuned (`physics.STAGED`). The boom 60 mm across
  and an arm up to each yaw drive: across, it lay on the L, their inner
  corners 16 mm higher rolled 25 deg, no crown between. fit clean over its
  45 poses once the femur left its collar 34 mm out, not 28.5 - knees under
  her it met the fork's front bearing, 1 mm (2026-10-03).
- Her trunk's roll drive (2026-10-03): her shell at the spine's pivot 70 mm
  ahead and behind, 102 to the sides. On its own axis a roll M stood 47 mm
  out of it, 10 past her clothes ahead, 27 behind. A rod from the pelvis to
  the roll's stage crosses the pitch: its ball on the pitch axis, rolled 10
  deg and pitched 60 it read 10 of false roll, its slant took the roll's
  lever through nought near 58 of pitch. A bevel differential across the
  pivot fitted her, two M at x +-55 mm, but its bevels carry the drives'
  87.5 N m each, 4.4 kN on a 20 mm radius: steel wants about 70 mm across,
  the drums then past her shell; sat back on her heels, hips -163, her
  femurs met its drums 3-6 mm wherever the knee's board left them room. The
  roll's M on the pitch stage beside the pivot met the hip's yaw drum.
- Her ball stages for the wave reducer tools: one eccentric, ring fixed, cage
  out at lobes:1, the eccentric's race a stock bearing's outside, its
  eccentricity 80 % of the largest whose ball path stays a ball round
  plus 0.15 mm; pressure angle mean 13-15, at most 21-22 deg (2026-10-02):

  | size | balls | race | ball circle | e | web | lobes | ring | housing |
  | --- | --- | --- | --- | --- | --- | --- | --- | --- |
  | L | 11 x 12 mm | 6005 | 59 mm | 0.92 mm | 4.9 mm | 12 | 69.2-72.8 mm | 80 |
  | M | 11 x 8 mm | 6805 | 45 mm | 0.77 mm | 4.9 mm | 12 | 51.5-54.5 mm | 60 |
  | S | 9 x 7 mm | 6900 | 29 mm | 0.55 mm | 3.1 mm | 10 | 34.9-37.1 mm | 42 |

  M's on 9 mm balls left its ring 1.75 mm of wall in 60, S's on 8 mm
  0.4 in 42.
