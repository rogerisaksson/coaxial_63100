# Findings: feet

Passive toes, the sole's give, the push-off. The walk: [walk](walk.md); the
board's findings: [FINDINGS](../FINDINGS.md).

## Toes sprung, no motor

`drives.WAYS` foot 1, `drives.TOE_K` (2026-10-04).

- As the walker was, at 40 N m/rad: scoreboard held 15 % against 73.7 driven
  (every walk down at its first push-off, 0.5-0.8 s; every rise at its first
  steps); the heel rise 50 -> 30 deg: 33 %.
- The push-off's knobs searched on the look suite (8 x 12: TOE_DEG,
  HEEL_OFF, RISE_DEG, LAND_DEG, TOE_RATE, the arrival's lift and first
  stride): best 397 at TOE_DEG -28, HEEL_OFF 0.46, RISE_DEG -10, LAND_DEG
  9.4, TOE_RATE -363, LIFT_UP_M 0.054, FIRST 0.58. Rises 45-79 %; walks 82
  and 89 % at 0.65-0.85, 18 and 6 at 0.9-1.0; stir 24 mm against 1.7 driven.
  Rerun with the knobs rounded: 41-100 and 5-53 (chaos).
- Stiffer is worse: 100 N m/rad rises 30-40 %, walks 4-66; 200 and 400 under
  35 and 16. A stiff toe turns with the foot at toe-off and stands the body
  on its toe tips; a driven one counter-turns to lie flat.
- From the squat at 0.85, 10 s walked on the found knobs, a catch at 6.5 s.
- Damped: a thin carbon-fibre sandwich with TPU, or TPU printed round carbon
  rods glued with silicone. `drives.TOE_C` = loss factor 0.3 x TOE_K / 20
  rad/s = 0.375 N m s/rad. Sprung so, the look suite held 35.9 % against
  69.9 driven (rises 44, 57 and 38 %; walks 91, 12, 5 and 4 at 0.65-1.0;
  every fast walk down within 0.8 s).
- The motors off the feet, the damping from TOE_LOSS: 21 drives, 16.2 kg of
  drives from 17.3; the whole scoreboard 1503.8 and 39.6 % against 721.4 and
  65.8 driven.
  + Walks and events start dead from a stand (`walker.start()`,
    `gait_montecarlo.trial`) and fall in the first stride at 0.85 +-2 %:
    0.7, 2.6 and 1.8 s in; at 0.65 they hold 90.8 %.
  + From the squat through the arrival's lean (`look.py`, the page's way):
    walks at 0.85 and 1.0 with a catch at 8.3 and 7.7 s, the pelvis turned 79
    and 49 deg over 16 s against 25 driven; the toes bend to -23 and -27 deg
    at the push-off.
  + The sole's give (`mjcf.SOLE_S` 0.02, 1.5 of critical, 5 mm) is the whole
    foot's foam and TPU; the carbon flex is the toe hinge alone.
- Started standing through the arrival's shift, lean and step
  (`Director.begin(stage='stand')`), as the page starts: with the toes sprung
  1452.7 and 49.0 % against the dead start's 1503.8 and 39.6; the 0.65 walks
  100 %; the 0.85-1.0 walks 4-6 s and 2.4-5.1 m before they fall; the events
  at 0.85 falling at 5.4-6.1 s. The veer, not the launch, is the sprung walk's
  gap.

## Spring stiffness

`drives.TOE_K` (2026-10-04):

| N m/rad | Scoreboard | Walks at 0.65-1.0 | Rises | Note |
| --- | --- | --- | --- | --- |
| 40 | 1452.7, 49.0 % | - | - | falls at 6.8 s from the squat |
| 25 | - | - | - | toes bent 23-27 deg at the push-off; veered |
| 10 (baked) | 1009.6, 67.1 % | 100, 100, 66, 31 % | 79, 100, 83 % | at a true 1.0 strides/s they fold 41 deg, falls at 6.7 s |

## Push-off on sprung toes

`gait.HEEL_OFF`, `TOE_DEG`; a grid on the look suite (2026-10-04). As built
for the driven toes the heel lifts at 0.36 of the stride, the foot 50 deg
down at toe-off.

| Heel off | Toe-off deg | Look suite | Faults suite | Note |
| --- | --- | --- | --- | --- |
| 0.36 | 50 | 328, 80.0 % | - | as built |
| 0.5 | 30 | 179, 88.8 % | 475, 58.4 % | every rise standing; walks 78, 100, 100, 43 % at 0.65-1.0; from the squat (16 s) no catch where the built caught seven times and fell at 9.0 s |
| 0.5 | 35 | 95.2, 98.2 % | 285, 68.9 % | every rise and the walks to 0.9 standing, 1.0 at 87 % |
| 0.5 | 40 (baked) | 197, 91.2 % | 217, 71.5 % | two rises at 0.9 down |
| 0.36 | 30 | 227, 81.6 % | - | - |
| 0.5 | 20 | 301, 86.7 % | - | - |
| 0.36 | 20 | 514, 55.7 % | - | - |
| 0.45 | - | 70.6-85.0 % | 38.6-45.4 % | - |

## Walk height

- Lower on sprung toes (2026-10-04): the heel off at 0.5 drops the heel's
  rise as the other foot lands (`gait.RISE_AT` 0.5), the trailing foot stays
  flat; the landing knee 35-38 deg, 23 at mid-stance, the pelvis 0.84-0.87 m
  up, where on driven toes the walk ran at 0.88 with the standing knee
  straight (2 to -6 deg).
- A knee's board 0.45 s in its SOA under the weight then folds it:
  test_gynoid_faults' walk-on stands at 40 deg and with the heel off at
  0.48, falls at 30, 33, 35, 37, 38 and 42 deg and at 0.52 and 0.55: chance;
  the suite's soa event 54 %. A derated knee taken as a shove (the catch
  armed, the phase 25-80 % faster) fell every time.
- `look.py` from the squat, the walk's row: on driven toes the pelvis
  8.7-28.8 mm under the stand and the left knee to 52 deg; sprung, heel off
  at 0.5, 14.2-53.2 mm and 74 deg.
- A toe spring (`drives.TOE_REST`, the toes' rest up 5 and 10 deg) under the
  driven toes' push-off, heel off at 0.36 and 50 deg (a grid of 8 on the look
  and faults suites): the walk back at the driven height, 7.2-30.8 mm under
  the stand, 14 s with no catch, every rise standing; look suite 176-189 and
  88.4-90.5 % against the baked 197 and 91.2, but faults 257-508 and
  54.6-60.6 % against 217 and 71.5, the walk at 1.0 strides/s 33 % against
  80. The rest alone, heel off at 0.5: the dip stays, 51.6 mm at 10 deg. Not
  baked: the height and the faults are the grinder's to find.
- A shorter stride (`gait.STRIDE_M`, `look.py` from the squat, 14 s): 0.5 m
  at 1.0 strides/s, the pelvis 5.7-19.2 mm under the stand against 14.2-53.2
  at 0.85 m, but 36 catches; 0.45 m at 1.1: 6.0-17.1 mm, 3 catches. The
  height is the stride's; the walk needs its knobs searched at it (the
  grinder's `gait.STRIDE_M`).

## Rear foot slip

The rear foot slips back before its swing (`look.py`'s toes back at lift,
from where the toes stood as the foot last bore the weight alone;
2026-10-04).

- 92 mm as baked, 90 with the load's band; at 1 kHz 64-127 mm over 90 ms,
  the toes 14-34 mm up, the foot bearing nothing from 20 ms after the other
  lands (phase 0.52) to its toe-off at 0.66.
- Cause: the stance leg runs out of reach before the heel rises at 0.5. The
  knee set straight at 520 deg/s and the hip back at 350, the other foot
  lands, and the leg, released still extending, throws the foot back: the
  hip to 26 deg where 17 was asked, the knee to -7. A leg bearing nothing
  late in its stance solved from the pelvis as it is: 69 mm.

| Fix | Slip mm | Effect |
| --- | --- | --- |
| heel off at 0.40 (`gait.HEEL_OFF`) | 5 | landing knee 23 deg from 37; pelvis 7.6-29.4 mm under the stand from 13.4-48.2; scoreboard 798.8 and 75.4 % against 229.0 and 86.4; walks at 1.0 strides/s down at 4.5 s (0.38 and 0.42 alike; 0.46-0.49 erratic, 6-16 mm, one fall) |
| toe-off at 0.54-0.58 | - | down at 4.6-9.3 s |
| heel by reach (`stance.rolled`: a stance foot behind the hip rolled up on its ball until the ankle is within 0.99 of the leg's reach, 45 deg at most) | 8 | ball within 6 mm of its place from 0.42 to toe-off, bearing 20-170 N; the knee's setpoint still 16 deg; power 560 -> 369 W; hip and knee rms 56 and 49 -> 43 and 42 N m; touchdown 253 -> 222 N; walk at 1.0 strides/s down at 6.3 s (catches from 9 s, the baked from 10.4); feet landing 125 mm ahead of the hip where 175, lifted 272 behind where 359, 40 mm apart where 19: the stride was the dipping pelvis's |

### Baked

2026-10-04: the heel by reach (45 deg, 0.99); the stride 0.75 m
(`gait.STRIDE_M`; at 0.85 the walk at 1.0 strides/s falls, at 0.8 it holds
69 %); the foot released travelling with the pelvis from where it stood,
easing into its swing over 0.1 of the stride (`stance.LET_Q`: at toe-off its
leg was asked 16 -> 42 deg of knee in a pass and the toes still went back 10
mm); the swing 45 mm up (`gait.LIFT_M`).

- `look.py` from the squat: toes back at lift 0.8 mm from 92; head bob 21 mm
  from 24; strike 388 N from 430; landing knee 32 deg from 37; pelvis
  11.6-38.1 mm under the stand from 13.4-48.2.

| Strides/s | Slip mm (before) | Power W (before) |
| --- | --- | --- |
| 0.65 | 3 (31) | 317 (507) |
| 0.85 | 1 (85) | 342 (554) |
| 0.9 | 1 (92) | 356 (624) |
| 1.0 | 4 (92) | 487 (882) |

- A drive at its clamp 0.24-0.94 % of the passes, from 0.82-1.73.

- Scoreboard, the slip in its cost: 250.5 and 82.9 % against 265.9 and
  86.4; every rise and walk; a sill at 0.65 strides/s and a nudge walking
  felled it 2 of 3 each, none before; the rug 1 of 3 from 2.
- Its forms: let-go over 0.06, 11-20 mm, 368.0 and 85.5 %; the swing 25 mm
  up, the rug 3 of 3, 400.2 and 82.3; no let-go, 10 mm, 374.0 and 84.7; the
  reach at 0.97, 634-679 and 73-78.
- Felled by the scoreboard's shove at 5.0 s: up and walking at 28.4 s.

## Parry hurry

0.29 -> 0.25 with the elbow's stack on its axis (2026-10-04): the test's 38 N
held, left and right of 8: 0.25 2 and 3, 0.27 1 and 3, 0.29 2 and 1, 0.32 2
and 2; faults scoreboard 428 and 65.2 % at 0.25, 482 and 62.2 at 0.32.

## Stacks at the first step

Into the walk from the squat the gynoid staggered on the stacks
(2026-10-03): their ankles' and hip rolls' rotors reflect half the inertia
the first step was tuned on (0.103 and 0.173 kg m^2 against 0.282 and
0.367; the hips' and knees' 0.158 against 0.163); the pelvis went 51 mm down
and 15 up in the walk's first second. `arrival.LIFT_UP_M` 0.06 -> 0.04: 18
down and 4; Monte Carlo held 74.1 % with every rise 100 %; `FIRST` 0.5 lost
rises.
