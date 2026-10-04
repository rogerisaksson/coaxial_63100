# Findings: one stack for every drive

One motor, one gearbox and one inverter: the demand the stacks as built
were sized on, the candidates, and what the one stack leaves open
(`machine/drives.py`, docs/DIMENSIONS.md). Her drives' own sizing is in
[drives](drives.md).

- The demand the stacks were sized on is the controller's own (2026-10-04,
  the walk from the squat, 20 s, each leg joint's torque by band): the
  hip's rms 64.6 N m, 47.7 of it under 50 Hz and 35.8 under 5; the knee's
  60.4, 36.2 and 19.3; the ankle's 44.6, 30.8 and 17.9 - 45 and 64 % of the
  hip's and the knee's copper heat is over 50 Hz, and every peak is its
  servo's clamp (171, 171, 132, 129, 52 N m), the hip and the knee at them
  5.3 and 7.4 % of the walk's passes. Its source the setpoints: 422 of
  6051 passes moved one over 0.5 deg, the knee's rate 914 deg/s rms and the
  hip's 653 - 3-8 deg in a pass at the swap of feet (the landed foot's
  anchor, the pelvis lowered), 12 at a toe-off (the anchor let go), and
  1.5-7 deg a pass to and fro about them, a leg's stance share
  (`stance.legs`' b) following its sole's load read a pass at a time and
  flickering 0-900 N. Rebuilt from what was written, kd x the setpoint's
  rate is 540 N m rms at the hip and the rotor's feed-forward 457, kd x the
  joint's own rate 89, kp x the error 65.
- The soles' load through a sensor's band (`physics.LOAD_S` 20 ms, the
  lab's, 2026-10-04): the walk's touchdown 769 -> 253 N, its power 683 ->
  560 W, the hip's, knee's and hip roll's rms 63, 60 and 48 -> 56, 49 and
  36 N m, at their clamps 2.0, 3.6 and 1.3 % of the passes; 5 walks of 5
  held (0.65-1.0 of the pace) against 4. A governor on the setpoints - each
  followed at 1000/s, within 0.9 of its drive's unloaded speed and the
  acceleration its clamp gives its rotor and limb - took the torque over
  50 Hz out (the hip's 48 of 48 N m under 50 Hz, at its clamp 0.0 %); at
  200/s, 5 ms behind in the loops closed through the setpoints, 1 walk of
  5 fell at 1.6 s and standing on the loads as read she fell at 3.9 s.
- One box and one inverter by smoke (the user, 2026-10-04; a candidate's
  stacks laid at run time, the load's band, the rise and five walks as
  tuned at 0.65-1.0 of the pace): each rose and walked 5 of 5 - as built
  (32.3 kg, 16.2 of it drives, 23 part types); frame B's lamination at two
  stack lengths, 60 x 16 and 60 x 30 for the hips, knees, ankles and spine,
  KV 90, box B and the 50 A inverter on every drive (28.5 and 12.4 kg, 21
  types); the long stack on every drive (30.1 and 14.0, 20 types); the
  short on every drive (27.7 and 11.6, 20 types). Under the governor at
  200/s the short stack and a middle one (64 x 22, a 74 mm box, 75 A)
  fell at 1.6-1.7 s 5 of 5 and the two lengths 3 of 5: its lag, not the
  stacks. The two lengths on their own demand (`drive_sizes`, the load's
  band): the hips', knees' and spine's peaks are the 124 N m clamp (A
  1.50, the box's S 1.45), the hip's and knee's rms 53 and 49 N m of the
  69 the 50 A inverter's switches hold (T 1.36 and 1.13 at 1.5x, 0.60 and
  0.50 at 1x; the short stack's winding holds 52), the knee's 1007 deg/s
  1.17 of its volts.
- The margin as the clamp (2026-10-04): a peak that is its servo's clamp
  times 1.5 is over its drive by construction. Every servo's clamp its
  drive's peak / 1.5 instead - the two lengths' hips and knees at 83 N m,
  as built at 136 -, she rose and walked as far (9.5 m): the two lengths'
  hip and knee rms 40 and 33 N m (T 0.76 and 0.52 at 1.5x of the 69 held),
  at the clamp 6.7 % of the passes, the knee's 729 deg/s 0.84 of its volts,
  the box's S 0.97 at 1.5x. Her falls, parries and standing within it:
  the scoreboard.
- The candidates' fit (`fit.py` on each, 2026-10-04): the two lengths
  stand 12 and 5 mm past her skin at the ankle and the knee against 21 and
  14 as built, none past her shell but the elbow's 5 mm as before, the
  ankle's two drives clear of each other (6-7 mm into each other as
  built). Every inverter in its own stack, a 70 mm disc and no board
  apart: the elbow 8 mm past her shell, the knee 9 and the ankle 15 past
  her skin, no pair closer. The long stack on every drive: 13 mm past her
  shell at the elbow, 6 at the hip roll, 4 at the shoulder, its gearbox 9
  mm into the upper arm's tube.
- A fall asks more of a box than any here takes (`gait_montecarlo`'s
  shove, the worst of each): as built the ankle's 2226 N m against its
  pair's 527 and the knee's 658 against 289; on the two lengths the knee's
  763 against 128, the ankle's 1050 against 233 - the rotor spun up
  through 1:30 by the blow, not the servo's torque.
- The candidates on the scoreboard (the load's band, the stepping foot
  by its landing, as tuned, 2026-10-04): as built 229.0 and 86.4 %; the
  short stack on every drive 264.6 and 84.8, within its peaks / 1.5 282.8
  and 82.1; the two lengths 533.6 and 85.7, within their peaks / 1.5
  526.5 and 82.2, the long at KV 105 425.7 and 82.2 - every rise and walk
  100 % on each. The two lengths' cost is the fall's: the knee's box asked
  5.96 times its momentary by the long stack's rotor (0.094 kg m2 at the
  joint), the short's 0.062. The short stack's winding holds 52 N m: the
  hip's rms 45 at its peak and 39 within peak / 1.5 (T 1.26 at 1.5x, 0.56
  at 1x), the knee's 38 and 32. The governor on the setpoints at 1000/s:
  911.4 and 61.0 %, at half its clamp's acceleration 960.2 and 56.5, the
  walks 35-72 % - not taken.
- On the walk with the slip gone (feet.md, 2026-10-04; smoke at 0.85
  strides/s): as built the hip's, knee's and ankle's rms 39, 41 and 35 N m,
  332 W; the short stack on every drive 30, 25 and 26 (the hip roll's 42 of
  the 87 it holds), within its peaks / 1.5 28, 24 and 26. Sized on its own
  demand over the scenes (`drive_sizes`, the run at her 27.7 kg folded in):
  the hip's rms 37 of the winding's 52 N m (T 1.13 at 1.5x, 0.50 at 1x),
  the knee's 32 (0.84), the ankle's 33 of 95 (0.28), the hip roll's 46 of
  87 (0.64); the knee's 1059 deg/s 1.23 of its volts at KV 90 and 48 V; the
  peaks the 124 N m clamp.
- The stacks as built (`machine/drives.py`'s record of 2026-10-03 and -04):
  two frames, A 68 x 30 at KV 110 on an 84 mm box and the 100 A inverter
  for the spine, hips, knees and ankles, B 60 x 16 at KV 90 on a 64 mm box
  and the 50 A for the rest, each KV in its window of 1.5x on the walk's
  torque at its speed, each box's momentary 1.5x its peak and its rated
  over its rms. At 1:30 frame B's windows met at KV 76-95 but the ankles'
  (x1.43-1.48), their copper 1.05x their rms and the hip roll's 1.17x; A's
  spine on 100 A to KV 104, hip 52-85, knee none - x1.41 at KV 90, its
  first step's 144 N m at 704 deg/s, 2.0 kW, on 100 A at 48 V. The hip's
  and the knee's at KV 120, x1.09 over her clamp, 960 deg/s: at 90 and 70
  she held 70.7 % of the gait Monte Carlo, its knee run into its SOA
  felled her 2 of 3; at 120 74.2 %, 1 of 3; at 125 71.8 %. The spine at KV
  40 on 50 A, 360 deg/s, felled her in the knee's SOA. One KV for A: the
  spine's amps at its clamp 1.05 at 110, 1.15 at 120; the hip's and the
  knee's 1.26 at 110 (volts 1.15 and 1.23), 1.38 at 120 (1.06, 1.13); at
  100 their volts 1.31 against 1.19, a parry's 1259 deg/s binding, not the
  run. The waist's and the shoulder's 44 mm boxes stood 2.79 and 1.44 times
  their momentary rating: the 64 mm 0.91 and 0.47. The ankles on A for a
  run's headroom at 55 kg (on B 1.32 of its amps, 5.3 of its heat, 1.27 of
  its box; on A 0.80, 0.73 and 0.56); the hip yaw, the hip roll and the
  ankle roll on B (0.62, 0.93 and 0.97 of their amps; on A the hip roll's
  stack stood 19 mm into the pelvis frame and the hip yaw's inverter 15
  into the spine's); the rest on B with B's box, box C gone with frame C.
- One stack on every drive (2026-10-05, laid; the user: the most
  simplicity, the fewest gearbox and electronics variants): 60 x 20 mm at
  KV 90, the 64 mm box at 1:30, the 70 mm 50 A inverter - 21 of each, 20
  part types against 23, her 28.4 kg against 32.3, the drives 12.3 of it
  against 16.2. On the walk with the slip gone the scoreboard, a stack
  length each: 16 mm 292.5 and 83.7 % (the walk at 0.65 strides/s 76 %),
  18 mm 325.0 and 85.8, 20 mm 318.7 and 84.9, against 250.5 and 82.9 on
  the stacks as built; the hips' pitch alone on 60 x 30, 351.5 and 83.0;
  within the peaks / 1.5 on 16 mm, 299.9 and 81.7. The length is the
  hip's: its rms 37-42 N m over the scenes, the winding holding 52, 59 and
  66 N m at 16, 18 and 20 mm (T 1.13 and 1.15 at 1.5x on 16 and 18). On
  20 mm, its own demand and the run at her weight (`drive_sizes`): T 0.90
  at the hip, 0.79 at the knee, 0.45 at the hip roll, 0.21 at the ankle;
  the hips', knees' and spine's peaks its 124 N m clamp (A 1.50, P
  1.24-1.35, S 1.45); the knee's 1034 deg/s 1.20 of its volts; the elbow,
  the neck and the head feel their rotors 7.8, 2.9 and 33 times their
  loads. On it her get-up as it stood left her down: felled by the shove
  she came from the sit into the crouch pitched 9 deg where 24, her centre
  of mass 52 mm ahead of the pelvis where 134, bore 236 of her 279 N on
  her feet in the squat and fell forward out of it, 3 tries, from each of
  five lying states. The keyframes onto her feet searched on it
  (`getup_search --table f`, 96 candidates): 2 walked, from all five, the
  rest from none; the best walking again at 23.6-25.4 s, 1 try, her hands
  out 5.3 s, bowed 5.5, the arms thrown up to 170 deg. Folded to 170 deg
  the ankle roll's drum meets the femur where it met the knee's board
  (test_gynoid). Its fit: 9 and 3 mm past her skin at the ankle and the
  knee against 21 and 14, 6 mm past her shell at the elbow against 5, 2 at
  the hip roll. As laid the scoreboard 318.7 and 84.9 %, every rise and
  walk; felled by the shove she is up and walking at 33.0 s, 2 tries.
