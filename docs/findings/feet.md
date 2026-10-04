# Findings: her feet

The gynoid's feet: the toes without a motor, the sole's give, the push-off
on them. Her walk is in [walk](walk.md); the board's own are in
[FINDINGS](../FINDINGS.md).

- The toes sprung, no motor (the user, 2026-10-04; `drives.WAYS` foot 1,
  `drives.TOE_K`): as the walker was, at 40 N m/rad, the scoreboard held
  15 % against 73.7 driven - every walk down at its first push-off
  (0.5-0.8 s), every rise at its first steps -, the heel rise 50 -> 30
  deg 33 %. The push-off's knobs searched on the look suite (8 x 12:
  TOE_DEG, HEEL_OFF, RISE_DEG, LAND_DEG, TOE_RATE, the arrival's lift and
  first stride): best 397 at TOE_DEG -28, HEEL_OFF 0.46, RISE_DEG -10,
  LAND_DEG 9.4, TOE_RATE -363, LIFT_UP_M 0.054, FIRST 0.58 - the rises
  45-79 %, the walks 82 and 89 at 0.65-0.85, 18 and 6 at 0.9-1.0, the
  stir 24 mm against 1.7 driven; rerun with the knobs rounded 41-100 and
  5-53 (chaos). Stiffer is worse: 100 N m/rad the rises 30-40 and the
  walks 4-66, 200 and 400 under 35 and 16 - a stiff toe turns with the
  foot at toe-off and stands her on the toe tips; a driven one
  counter-turns to lie flat. From the squat at 0.85 she walks 10 s on
  the found knobs with a catch at 6.5 s.
- The parry's hurry 0.29 -> 0.25 with the elbow's stack on its axis
  (2026-10-04): the test's 38 N held, left and right of 8, 0.25 2 and 3,
  0.27 1 and 3, 0.29 2 and 1, 0.32 2 and 2; the faults scoreboard 428 and
  65.2 % at 0.25, 482 and 62.2 at 0.32.
- Into her walk from the squat she staggered on the stacks (the user,
  2026-10-03): their ankles' and hip rolls' rotors reflect half the inertia
  the first step was tuned on (0.103 and 0.173 kg m^2 against 0.282 and
  0.367; the hips' and knees' 0.158 against 0.163), and the pelvis went 51
  mm down and 15 up in the walk's first second. `arrival.LIFT_UP_M` 0.06 ->
  0.04: 18 down and 4; the Monte Carlo held 74.1 % with every rise 100 %,
  `FIRST` 0.5 lost rises.
- The toes springy but damped (the user, 2026-10-04: a thin carbon-fibre
  sandwich with TPU, or TPU printed round carbon rods glued with silicone):
  `drives.TOE_C` = loss factor 0.3 x TOE_K / 20 rad/s = 0.375 N m s/rad.
  Sprung so, the look suite held 35.9 % against 69.9 driven - the rises
  44, 57 and 38 %, the walks 91, 12, 5 and 4 at 0.65-1.0, every fast walk
  down within 0.8 s; the toes stay driven until the push-off is reworked
  (TODO 5) (2026-10-04).
- The motors off the feet (the user, 2026-10-04; `drives.WAYS` foot 1, the
  damping from TOE_LOSS at the call): 21 drives, 16.2 kg of drives from
  17.3; the whole scoreboard 1503.8 and 39.6 % against 721.4 and 65.8
  driven. The walks and the events start dead from a stand
  (`walker.start()`, `gait_montecarlo.trial`) and fall in the first stride
  at 0.85 +-2 %: 0.7, 2.6 and 1.8 s in; at 0.65 they hold 90.8 %. From the
  squat through the arrival's lean (`look.py`, the page's way) she walks at
  0.85 and 1.0 with a catch at 8.3 and 7.7 s, the pelvis turned 79 and 49
  deg over 16 s against 25 driven; the toes bend to -23 and -27 deg at the
  push-off. The sole's give (`mjcf.SOLE_S` 0.02, 1.5 of critical, 5 mm) is
  the whole foot's foam and TPU; the carbon flex is the toe hinge alone
  (the user, 2026-10-04).
- The scoreboard's walks and events started standing through the
  arrival's shift, lean and step (`Director.begin(stage='stand')`,
  2026-10-04), as the page starts her: with the toes sprung 1452.7 and
  49.0 % against the dead start's 1503.8 and 39.6; the 0.65 walks 100 %,
  the 0.85-1.0 walks 4-6 s and 2.4-5.1 m before they fall, the events at
  0.85 falling at 5.4-6.1 s. The veer, not the launch, is the sprung
  walk's gap.
- The toes' spring softer (`drives.TOE_K`, 2026-10-04): at 25 N m/rad the
  toes bent 23-27 deg at the push-off and she veered; at 10 the scoreboard
  1009.6 and 67.1 % against 1452.7 and 49.0, the walks 100, 100, 66 and 31
  % at 0.65-1.0, the rises 79, 100 and 83; at a true 1.0 strides/s they
  fold 41 deg and she falls at 6.7 s; at 40 she falls at 6.8 s from the
  squat. Baked 10.
- The push-off on sprung toes (`gait.HEEL_OFF`, `TOE_DEG`; a grid on the
  look suite, 2026-10-04): as built for the driven toes - the heel off at
  0.36 of the stride, the foot 50 deg down at toe-off - 328 and 80.0 %;
  off at 0.5 and 30 deg 179 and 88.8 %, every rise standing, the walks 78,
  100, 100 and 43 % at 0.65-1.0; 0.5 and 40 deg 197 and 91.2 (two rises at
  0.9 down); 0.36 and 30, 227 and 81.6; 0.5 and 20, 301 and 86.7; 0.36 and
  20, 514 and 55.7. At 0.5 and 30, from the squat (`look.py`, 16 s), one
  walk with no catch where the built caught seven times and fell at 9.0 s.
  Six candidates on the faults suite beside the look: 0.5 and 35 deg look
  95.2 and 98.2 % - every rise and the walks to 0.9 standing, 1.0 at 87 -
  faults 285 and 68.9 %; 0.5 and 40, faults 217 and 71.5; 0.5 and 30,
  faults 475 and 58.4; the heel off at 0.45, look 70.6-85.0 %, faults
  38.6-45.4. Baked 0.5 and 40.
- She walks lower on sprung toes (2026-10-04): the heel off at 0.5 drops
  the heel's rise as the other foot lands (`gait.RISE_AT` 0.5), the
  trailing foot stays flat, and the landing knee stands at 35-38 deg, 23 at
  mid-stance, the pelvis 0.84-0.87 m up, where on driven toes she walked
  at 0.88 with the standing knee straight (2 to -6 deg). A knee's board
  0.45 s in its SOA under her weight then folds it: test_gynoid_faults'
  walk-on stands at 40 deg and with the heel off at 0.48, falls at 30, 33,
  35, 37, 38 and 42 deg and at 0.52 and 0.55 - chance; the suite's soa
  event 54 %. A derated knee taken as a shove (the catch armed, the phase
  25-80 % faster) fell every time.
- How low (`look.py` from the squat, the walk's row, 2026-10-04): on driven
  toes the pelvis 8.7-28.8 mm under her stand and the left knee to 52 deg;
  on sprung ones with the heel off at 0.5, 14.2-53.2 mm and 74 deg.
- A toe spring (`drives.TOE_REST`, the toes' rest up 5 and 10 deg) under
  the driven toes' push-off, the heel off at 0.36 and 50 deg (the lab, a
  grid of 8 on the look and the faults suites, 2026-10-04): the walk back
  at the driven height, 7.2-30.8 mm under her stand, 14 s with no catch,
  every rise standing; the look suite 176-189 and 88.4-90.5 % against the
  baked 197 and 91.2, but the faults 257-508 and 54.6-60.6 % against 217
  and 71.5 and the walk at 1.0 strides/s 33 % against 80. The rest alone,
  the heel off at 0.5: the dip stays, 51.6 mm at 10 deg. Not baked: the
  height and the faults both are the grinder's to find.
- A shorter stride (`gait.STRIDE_M`, `look.py` from the squat, 14 s,
  2026-10-04): 0.5 m at 1.0 strides/s the pelvis 5.7-19.2 mm under her
  stand against 14.2-53.2 at 0.85 m, but 36 catches; 0.45 m at 1.1, 6.0-17.1
  mm and 3 catches. The height is the stride's; the walk wants its knobs
  searched at it (the grinder's `gait.STRIDE_M`).
