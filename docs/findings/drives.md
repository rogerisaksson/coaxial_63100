# Findings: her drives

Her drives sized, wound and geared: the motors, the gearboxes, the inverters, the
numbers that size them (`machine.drives`, `tools/sim/drive_sizes.py`). Her body's own
are in [body](body.md), the board's in [FINDINGS](../FINDINGS.md).

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
