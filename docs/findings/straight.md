# Findings: the straight leg

The stance leg extended, as a woman's, or where its drives pay instead
(docs/TODO.md, walk form). The walk as built: [walk](walk.md); the one law's:
[going](going.md); the band: [normal](normal.md).

## The walk as built

A straight leg costs more than a bent one, and where (steady at
0.85 strides/s, 0.63-0.66 m/s, the boards' own 53 W in each): the walk as
built 365 W, 561 J/m (work 151, copper 160).

| Stance knee | W | Hips' and knees' work and copper W |
| --- | --- | --- |
| 6 deg | 402 | 247 |
| 18 | 264 | 133 |
| 30 | 256 | 123 |

- By the left leg's phase, a twentieth each: where it only stands, the knee
  at 1.5 deg, the knee draws 1 W at 6 N m rms; landed bent 21-27 deg and
  straightened under load, hip and knee 620 W over 0.12-0.17; released at 6
  deg into a swing of 57, 895 W over one twentieth; the two legs solved for
  one pelvis in the double support, the hips' roll 73-78 N m rms each. The
  mechanics ask some 10 W.
- A leg carrying as its sole bears of what the two bear (lab): 364 -> 341 W
  as built, the knee at plumb 12.3 -> 3.4 deg, the toes back 2 mm, no parry.
  The power is steady from 2 s into the walk: 368, 372 and 373 W from 2, 4
  and 8 s.

## The law's strut

`strut`, deg: the standing knee the pelvis is never over, 6 the form's.
Bent more, the walk's row draws less:

| strut deg | 6 | 10 | 14 | 18 | 24 |
| --- | --- | --- | --- | --- | --- |
| J/m | 574 | 528 | 503 | 502 | 500 |

At 10 deg 0.25 off the band where 0.41, the knee 29-30 deg as its toes leave.

## Where the straight leg's drives pay

The law's walk from the squat - strut 6, landing knee 11.8 deg asked - and its
crouched manner, strut 24 and 25.8: both 0.85 m/s on a 1.15 s stride, 0.98 m,
her 31.1 kg. Every drive, a world step a row from 5 to 16 s, the work on each
joint's travel over its step, J/m (`look.py` reads the same two walks 485 and
390, its draw sampled 60 times a second):

| Walk | Work | Braked | Copper | Drawn, the boards' 63 with it |
| --- | --- | --- | --- | --- |
| straight | 242 | 170 | 190 | 495 |
| crouched | 196 | 135 | 134 | 393 |

- Of the straight walk's 102 more, 56 are copper and 46 work, 35 of that
  braked away again; the work less its braking is 72 and 61.
- The world's work and braking meters took a step's torque on the speed
  before it and read 225 and 186, 185 and 144: on the speed after it now
  (`World.advance`), the joint's travel, as `World.drawn` has it.

Both legs' hips, knees and ankles by the phase of each leg's own stride, the
other foot landing at 0.50, 8 strides; the two legs within 0.5 J/m of each
other. Knee deg; work / braking / copper J/m:

| Of the stride | Straight | Crouched | Over, work and copper |
| --- | --- | --- | --- |
| 0-0.1, the landing | 12; 38 / 31 / 44 | 27; 18 / 16 / 25 | +38, the knees +19 |
| 0.1-0.4, standing | 6; 15 / 9 / 17 | 24; 12 / 10 / 13 | +6 |
| 0.4-0.5, before the other foot lands | 7; 3 / 16 / 10, the ankles' work 0.3 | 25; 0 / 14 / 7, the ankles' 0.1 | +6 |
| 0.5-0.6, the other foot lands, this leg trailing | 5; 41 / 16 / 34, the ankles' work 26 | 24; 31 / 9 / 18, the ankles' 24 | +26, the knees +14 |
| 0.6-0.65, the leaving | 11; 62 / 8 / 27, the ankles' work 31 | 29; 63 / 9 / 27, the ankles' 33 | -1 |
| 0.65-0.8, the swing | 46; 23 / 37 / 5 | 63; 25 / 38 / 6 | -2 |
| 0.8-1.0, the swing's end | 21; 46 / 46 / 38 | 34; 33 / 35 / 25 | +26, the knees +9 |
| the stride | 228 / 164 / 175 | 183 / 130 / 121 | +98 |

- No push-off before a landing in either walk: the ankles' work comes after
  the other foot is down, a quarter of it under load and a third as the
  foot leaves.
- The first table of this page, 7800cbb5, called these one leg's: its probe
  counted a world step twice, the second time on the speed after it.

## The landing, a pass a row

The law's own targets and both legs' setpoints over a left landing
(`Going._carried` and `hold.lean` wrapped, the boards' torques beside them;
one landing a walk, a second alike), the pelvis going 0.98 m/s on and 0.30
down as the foot lands:

| When | What the law does | Straight | Crouched |
| --- | --- | --- | --- |
| the landing's pass, the trailing leg | the height's target steps from over the pelvis as read, the one standing leg's arc, to the pelvis as read, where the bounce is laid: 3.7 and 4.6 mm | 2.8 mm of leg: its knee asked 6.1 -> 11.5 deg in the pass | 3.8 mm: 24.0 -> 26.6 |
| the 12 ms after, the landing leg | handed from its swing's pose to its standing solution, the difference faded out over 30 ms | the solution out of reach, its knee 0: held at 11.8 by the difference for 3 passes, then 3.5 deg in one as the solution comes into reach; 20.4 at 12 ms, 8.1 mm of leg | 19 mm inside its reach: 25.8 -> 28.2, 3.8 mm |
| to 0.1 s | the bounce asks 5 mm down at 26 ms, then up onto the strut's arc: the pelvis dips 7 mm (8) at 40 ms and is up 20 by 0.1 s; both soles 632 N (642) at 60 ms and 183 (179) at 120, her weight 306 | the knees at their 108 N m clamp 21 % of the landing's passes, 16 % of the trailing leg's | 6 %, 2 % |
| the leaving's pass, 0.13 s on | the height's target falls 4.2 mm (3.8) from the bounce's end onto the standing leg's arc; the lean across from 10 mm to none | the fall taken by the ankle, 3.3 deg in the pass; the knee asked 9.1 where the strut's 6 from there on - the arc taken at the pelvis as read, the target the lean's 7.4 mm ahead of it; hip, knee and ankle at their clamps under 4 ms | 2.2 deg; 24.6 where 24; at their clamps alike |

- dL/dtheta = a b sin(theta) / L: 20 mm/rad at a 6 deg knee, 40 at 12, 80
  at 24 - 2.8, 1.4 and 0.7 deg of knee a millimetre of leg -, and at full
  extension the knee goes as the root of what the leg lacks: 0.4 mm short is
  3.7 deg. The straight leg lands 4 mm from its full length.
- The height's target steps 3.7-4.6 mm at a landing and 3.8-4.2 at a
  leaving, the lean 10 mm across, in both walks alike. A standing knee's
  setpoint steps 3.1-5.4 deg in the pass straight, 0.6-2.6 bent.
- Not shown: that these steps are the landings' 64 J/m - no run has one out.
  Not separated from them: the knee's rate term (35 N m s/rad on a 108 N m
  clamp, 177 deg/s of setpoint past its own) and its rotor (0.177 kg m^2: 113
  kg along the leg at 12 deg, 24 at 26). The swing's end, +26, has no pelvis
  target in it.

## Tried on the law

The walk's row from the squat, a run each. The first four by `look.py`,
where the row as it is reads 485 J/m; the rest by the world's meters over
5-16 s, where it reads 495 at 0.85 m/s:

| Change | Result |
| --- | --- |
| the landing knee asked 6 deg (the row's 11.8) | lands at 4.7-4.9; 566 |
| the landing pose reaching the floor, the row's knee its least bend | lands at 4.7-5.5; 599, 0.26 off the band |
| a push-off: the standing heel up 5, 10 or 20 deg over the last 0.12 s before the free foot lands, kept while it gives its load - on the strut's arc; with the floor-reaching pose; lifting the pelvis's target by the ankle's rise | down in 4.0-6.1 s each: the sole hung 9-13 mm up for 0.15 s; the heel rose under an unmoved target and the knee bent 10 -> 13; with the target lifted she lost her way from the first step, 0.08 m/s at 3.3 s |
| the fold's second half a raised cosine, no slope at its end | 532, 0.22 off; the knee's unfolding moved to 0.80-0.85 of the stride and braked there |
| each knee's board gains laid a pass from its own angle, kp = min(900, K J^2 + 30), kd = min(35, C J^2 + 2), J = dL/dtheta; K 125 kN/m, C 5.5 kN s/m - a knee's 900 N m/rad is 2.2 MN/m along its leg at 6 deg, 140 kN/m at 24 | 483 at 0.79 m/s on a 1.34 s stride, landing at 4.0 deg and standing at 0-2: the knees' copper at the landing a third, the hips' and ankles' doubled; the leg no softer, its hip and ankle holding 0.4 of its length's stiffness through the same lever. K 300: 536; K 60: down at 5.5 s; the stiffness alone 721 |
| the knee's damping alone so, in swing and stance | 487 at 0.80 m/s: 26 less at the two landings, 18 more at the swing's end |
| the knee's damping so in stance alone | floored at 2 N m s/rad down in 5.7-7.3 s at C 2.5-25; floored at 6: 469 at 0.80 m/s, the knees at their clamp 4.1 % where 6.8 |
| the row's `off` 8, 15, 25 deg (the standing heel's rise toward the free foot's landing) | 502, 510, 530 |
| the row's `bounce` 0.07, 0.16, 0.22 s | 521; 488 at 0.80 m/s; 511 |

- None lowers a metre's draw at her pace; the knee's damping by its lever in
  stance takes 5 % of it at a pace 6 % slower. Untried: the height's target
  begun at a landing where it was, not at the pelvis as read; a standing leg
  solved from the leg as it landed, so no hand-over from out of reach
  (docs/TODO.md, walk form).
