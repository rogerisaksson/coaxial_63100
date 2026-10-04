# Findings: her drives

Her drives sized, wound and geared: the motors, the gearboxes, the
inverters, the numbers that size them (`machine.drives`,
`tools/sim/drive_sizes.py`). Her body's own are in [body](body.md), the
board's in [FINDINGS](../FINDINGS.md); the one stack for every drive, the
demand behind it and the candidates beside it in [stacks](stacks.md).

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
- A wobbling plate (nutation) ball reducer against the planar ball stage, on
  L's 61 mm ball circle: held by a Cardan or a Rzeppa, one stage gets the
  same balls a ratio - 1:12 11 of 12.9 mm, 1:36 4.1 mm; compounded, its two
  faces 6 and 6, 1:36 on six 20 mm balls but 42-59 % efficient through the
  difference, near locking backdriven. The planetary into the ball stage
  kept, 0.95 x 0.97 (2026-10-02).
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
  A ball on the 6005's cylindrical outside at L's 171 N m carries 4.66 kN,
  p0 7.46 GPa against ISO 76's static 4.2; at 40 N m 4.6: a grooved race
  or rollers in line contact (2026-10-03).
- One gearbox for all her drives, each outrunner and KV picked per joint
  for 1.5x on the walk's torque at its speed and on its rms copper
  (2026-10-03, 48 V): the hip and knee set the box - 257 N m momentary,
  81 mm; their 51 N m rms at a fifth of it, 92 mm. 96 mm round, 12 drives
  stood out of her skin (shoulder +36 mm, hip yaw +29, ankle and wrist +23,
  knee +22), the ankle pair's drums 24 mm into each other. Inside one box
  KV and frame met 1.5x from the wrist (MN3508 KV380) to the hip (U12 II
  KV60 at 1:24, x1.74, rms x2.1) but the knee: x1.45 at best (U12 II wound
  KV90, 1:30), its rise at 171 N m and first step 144 N m at 704 deg/s on
  100 A. Km, R line to line halved: 5230SL 0.205 (582 g), Hobbywing M8108
  85KV 0.229 (270 g), M8110 95KV 0.276 (315 g), U12 II KV60 0.581 (803 g).
  The walk asks 3.1 N m/kg at the hip and knee, their drives' clip.
- Her drives sized by two dimensionless numbers (2026-10-03, at the box's
  output, 1.5x): the power number 1.5 max(tau w) / (eta sqrt3/2 V I) picks
  the inverter - a KV exists under 1, its window 1/P wide - the knee 0.80
  on 100 A, the hip 0.37, the rest under 0.4 on 25 A; the power-rate
  number (1.5 T_rms)^2 J_rotor / ((eta T_cont)^2 J_load), the ratio gone,
  picks the frame - the knee asks 36 kW/s of T_cont^2 / J_rotor, a 107 mm
  U12 II gives 27, its rotor heavy, a 72 x 26 mm stator 40. One ratio
  then lies between each joint's heat's least and inertia's most.
- Her drives as coaxial stacks (`drives.STACKS`, 2026-10-03): frames
  68 x 30 and 60 x 16 mm, boxes 84, 64 and 44 mm, inverters 100 mm 100 A
  and 70 mm 50 A, 1:30. The gait Monte Carlo held 70.5 % on the drives as
  they stood; 61.3 on 72 x 28 / 60 x 12 at 1:30 - 2.05 of her shank's
  2.65 kg drives -, 74.6 on their old masses; 73.7 on these at 1:36, but
  felled by P she stayed down both ways (rotors seen 1.4-1.5x), at 1:30 up
  at 22.9 and 23.7 s; the knee and hip at KV 90 and 70 (1.5x torque)
  70.7, at 120 (x1.09, 960 deg/s) 74.2 - parrying asks speed, and on
  100 A at 48 V the knee has not both. Gynoid, falls and faults suites
  passed.
- Her drives sized by six numbers, each 1 at a part's limit at 1.5x the
  walk's demand (`tools/sim/drive_sizes.py`, 2026-10-03; the demand cached,
  an iteration 0.4 s): P the inverter, T the copper at the ratio, J the
  rotor felt through it, Q = T J the frame alone (the ratio cancels), S the
  box, V the volts; the ratio's window sqrt(T) N .. N / sqrt(J). Read on the
  stacks: the knee's window one number, 33..34 (Q 0.94, P 1.31, V 1.07);
  the hip's 32..82; the hip roll's T 1.57 at its spur's 1.67, 0.70 at the
  2.5 planned (window 63..126); the ankles' and the toes' J 21-775 the
  rotor against a light limb, not a limit (the foot's flick 0.12 of its
  peak), their S 0.42-0.94 the bound. The knee's continuous torque is the
  inverter's: at 68 x 30 the winding binds at 76 N m (41 A), at 68 x 45 the
  100 mm inverter's switches at 82 (44 A) through its 3.6 K/W housing path:
  a longer or wider frame buys nothing past it, J worsens (1.08-1.25).
  A six-bar for the knee, 360 deg of crank to 170 of shank: 43 close at
  33 deg transmission, the best x1.09 on torque at speed against direct
  drive's x1.12 at the same winding freedom, its lever 0.63-0.89 over
  15-45 deg where the knee asks its 171 N m and 1024 deg/s - no torque
  from it, only the dead centres. BOM: 2 frames, 3 boxes, 2 inverters, 3
  windings; the drives 15.2 kg of her 55; stacks 100/88/70/68 mm round.
- The hip roll on a crank-rocker (`linkage.PLANAR`, 2026-10-03): the drive
  on the fork behind, 48 mm from the roll axis, a 14 mm crank turning 360
  deg, a 45 mm rod, a 25 mm horn, the dead centres at -28 and +40 deg;
  lever 1.8 at 0, 2.2 at -14, 2.1 at +27, 3.8 at -24; transmission
  49 deg; the rod 7.7 kN at 1.5x. On the walk its heat 1.57 -> 1.14, amps
  0.93 -> 0.94, volts 0.31 -> 0.37. The linkage lives between the hip
  stack's end and the fork's arm: the stack's 100 mm inverter rim cut
  every rod at 34 mm from the pitch axis, so the stack went 20 mm in on
  its axis (and 16 mm into the pelvis boom squatting), the roll's
  bearings 5 mm out; in front the drum stood 68 mm out of her skin. The
  stroke against the lever: dead centres at -22 and +44 (a 12 mm crank, a
  22 mm horn) gave lever 2.0, 3.2 at -14, heat 0.86, the Monte Carlo
  75.8 % (74.2 on the spur), the SOA fault 100 %, up after P at 23.0 and
  23.5 s - but P's falls pin the adducting hip at -24 and swing the other
  to +35, and on the -22 stop her head met the floor at 4.5 and 6.0 m/s
  (`landings`: in 2 falls of 9 at 0.7 kN, 0 of 9 with the stop off, the
  nudge parry 1 of 8); at -28 and +40, the falls off its dead centres, no
  head landing, but the stance band's lever 2.2 for 3.2: the Monte Carlo
  67.2 %, the slip at 0.9 34 % (79), the fixed walk from the squat down
  at 13 s, the heat 3.11 with the parries; at -40 or -45 (the swing 84-94)
  the lever falls to 1.3-1.5 and the heat to 2.3. Neither stroke kept:
  the spur (1.67, no stops, 74.2 %, every suite) stays. A crank-rocker
  there wants a hip roll with twice the continuous torque, or dead
  centres a fall and a capture step never reach.
- The demand over five scenes - the walk, a slip, a nudge, a hole, P's
  shove - at 1.5x (`drive_sizes.SCENES`, 2026-10-03): one walk's knee asked
  1231 deg/s with a catch in it and 561 without. With the parries in, on
  the spur build: the hip T 1.94, P 1.33, V 1.06; the knee T 1.94, P 1.36,
  V 1.13, its window 42..34 inverted; the hip roll T 2.92; the ankles T
  1.4-1.5, V 1.15; the hip yaw T 1.01; the spine V 1.05. On the walk alone
  every number but the knee's P and V stood under 1; the five scenes'
  rms moves 20-30 % between runs of one build. A joint folded past a
  four-bar's dead centre or a rod's stroke read the far branch's lever
  (0.44 at the hip roll's -22, the demand 4.6x): clamped to the stroke.
- Each drive held rigid in turn on the scoreboard (`drives.WAYS`; the
  fewest drives, the user, 2026-10-03), against 730 and 74.1 % driven: the
  wrists 602 and 74.2, every rise and walk standing - held since, 23
  drives; the head's turn 699 and 69.4 (a walk fell), the neck 628 and
  71.2 (a rise), the waist 727 and 71.6 (a rise), the spine's roll 1293
  and 23.0, the hips' yaw 1337 and 12.1. One KV a frame (the fewest
  variants): A's spine 1.05 of its amps at KV 110, 1.15 at 120; its hips
  and knees 1.26 (volts 1.15, 1.23) and 1.38 (1.06, 1.13). The waist's and
  the shoulder's 44 mm boxes 2.79 and 1.44 times their momentary rating,
  the 64 mm 0.91 and 0.47 - B, the shoulder's stack +23 mm past her skin
  standing (+22 before). With the wrists held and the boxes B, A at KV
  110 scored 623 and 72.7 %, at 120 763 and 72.9: 110 kept, two windings.
- A run's demand folded into the sizing (`drive_sizes --run`, the
  literature's peaks a kg at 3-3.5 m/s: hip 2.7, knee 3.0, ankle 3.6 N m,
  the rms 0.4 of them, 7-12 W a kg; the aim, the user, 2026-10-04): the
  hips' and knees' numbers unchanged - the walk's parries ask more than a
  run -; the ankle binds, 147 -> 198 N m peak, on B A 1.32, T 5.26, S
  1.27, V 1.15, the hip roll's T 2.92 as before. On frame A with its box
  and inverter the ankle 0.80, 0.73, 0.56, 0.94, the ankle roll 0.59 and
  0.20, the hip roll 0.57 and 0.41: every leg joint on A is 3.9 kg more
  drive (14.2 -> 18.1 of 55).
- Two frames, two boxes, two inverters (the aim, the user, 2026-10-04):
  frame C and box C gone after a day, the elbow, the neck and the head on
  B direct - the elbow's drum 72 mm across the elbow with its collars, the
  shell there 80, +5 mm past it standing -; the ankles on A with its box
  and inverter (on B 1.32 of its amps and 1.27 of its box for a run), the
  stack 10 mm down the shank (its gearbox met the knee's by 7 mm with the
  knee folded), +3 past the calf's shell; the hip yaw, the hip roll and
  the ankle roll on B (on A the hip roll's stack stood 19 mm into the
  pelvis frame, the hip yaw's inverter 15 into the spine's). Her
  electronics in oil, each stator on its inverter through a thermal
  interface (`drives.COOLING` 0.3 on the windings' and the laminates' K/W,
  assumed): every heat number under 1 - the hips' and knees' 1.94 -> 0.64,
  the hip roll's 2.92 -> 0.88, the ankle's 0.24 -, the switches binding
  the A joints. Left over 1 at 1.5x the parries' peaks: the hips' and
  knees' amps 1.26, power 1.33-1.36, volts 1.15-1.23 (at 1.0x 0.84, 0.89,
  0.77-0.82), the spine's amps 1.05 - the margin's rule. Drives 17.3 kg
  of 55; parts 25 types (`bom.py`).
- A third frame, C, 40 mm round and 12 deep at KV 200, for the elbow, the
  neck, the head and the toes (the fewest parts, the user, 2026-10-04):
  on B the elbow's rotor reflected 4.2 times its load's inertia
  (`drive_sizes` J), the neck's 2.7, the toes' 775; on C 0.76 and 0.48,
  the elbow's amps at stall 0.91 (1.09 at a 10 mm stack), 0.7 kg off her
  drives. Across the elbow's 56 mm the C stack fits on the joint's own
  axis, -1 mm inside her shell, where B's 68 asked a bevel pair and a
  belt: both gone, the wrist's too (held), the elbow's inverter a ring
  round the humerus at mid-arm facing along it (0 mm). The humerus ends
  at a collar on the elbow's motor, the forearm hangs from one on its
  gearbox by a strut to its axis (`skeleton.HUNG`). Parts: 28 -> 27
  types (`tools/sim/bom.py`).
- Restarting after a fall she spasms and falls (the user, 2026-10-03,
  humanoid_20261003_231619, 232719): headless from the squat with P's
  shove at 60 s (`look.py --push 60 --to 100`) she collapses in the
  get-up's crouch at 76-80 s and again at 93-97 s, her right hip 117 C
  and derated to 0.82 there - the bowed torso (80-98 deg) hangs on the
  hips -; with the shove at 6 s the same restart stands. Her hottest
  drives: a knee 64-81 C through the walk, the left hip 106-109 in the
  catch, the spine 86-89 sat back on her heels. The test took her lying
  pose into a cold world: now the fall after 60 s of walking and the
  heat carried (`getup_search.SHOVE_S`, `World.glitch(celsius=)`). The
  get-up is her heavy lift: sitting back from the kneel draws 1.5 kW at
  its peak, lifting onto her feet 1.4, the crouch 0.1-0.5, and the hip
  goes 82 -> 117 C through them - not derated until the crouch, so a rest
  on her heels for a derated leg drive never fired, and was cut. The
  onto-feet keyframes re-searched hot stand her up 5 of 5
  (docs/findings/body.md).
- The gym's scenes in the sizing (the user, 2026-10-04; `drive_sizes.SCENES`
  'stand:' each rig of `events.STANDING`: standing on it, it befalling her
  at 8 s; the rms the walk's alone - a stand of milliseconds before a fall
  made it the peak, T 3.75): at 1.5x the hips' and knees' amps 1.26 and power
  1.20-1.23, the spine's amps 1.05 - the parries' peaks, unmoved -; the
  ankle roll's amps 1.08 and box 1.04, the gym's (0.93 and 0.90 before);
  the rest under 1. The ankle roll on A instead (`fit.py`): its stack 12
  mm past her shell and 11 into the ankle's gearbox (7 on B), so it stays
  on B - 0.72 and 0.69 at 1.0x, the margin rule's call (2026-10-04).
- One frame, one box, one inverter for every drive (the DFM extreme, the
  user's few variants, 2026-10-04; `drive_sizes --cached`, `fit.py`): all
  on A, 27.6 kg of drives against 17.3 (+10.3 of her 55), every joint
  under 1 but the hips and knees at A's own 1.26, and the stacks past her
  shell - the toes' 32 mm, the elbow's 19, the head's 14, the shoulder's
  and the hip roll's 13; all on B, 12.7 kg, the hips and knees at 2.07 of
  their amps and 2.4 of their power, the spine 1.72. Two it is: A for the
  spine, hips, knees and ankles, B for the rest (2026-10-04).
- What binds the hips and knees is a parry, not the run (`drive_sizes
  --cached --run`, 2026-10-04): their 171 N m and 1259 deg/s peaks are the
  scenes' - a catch's - and stand at 1.26 of A's amps, 1.21 of its power
  and 1.19 of its volts at the pack's lowest 48 V and 1.5x; the literature's
  3-3.5 m/s run asks less (148 N m), the 2 m/s jog `RUN` is now sized on
  less still. Amps x volts is the pack's volts x the inverter's amps: 1.15
  at 54 V and KV 100, 1.07 at 63 V and KV 93, no ratio or KV moves it. KV A
  100 tried: the volts 1.31 - back to 110. The toes' motors out: 21
  drives, 16.2 kg of 55; the elbow, the neck and the head feel their rotors
  6.8, 2.6 and 28.8 times their loads through box B's 1:30 (their windows
  3..11, 2..19, 0..6; B direct overheats the elbow, its rms 4 N m against
  1.7 held).
