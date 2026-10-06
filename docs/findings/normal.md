# Findings: a woman's walk as her metric

A normal feminine walk as a band of measures, and her walk as approved as a
take (the user, 2026-10-05: a metric, an .fbx its reference, a test of what
broke; `tools/sim/normal.py`, `fbx.py`, `mocap.py`, tests/takes/walk.fbx).
Her walk as built is in [walk](walk.md), the one law's in
[going](going.md). Mocap packs stay outside git: reference and test only
(the user).

- The band (2026-10-05). 23 measures from joints' places alone - her
  rows, a recording, a mocap take and an animation alike -, a band each:
  textbook gait (a knee all but straight as it lands, 15-20 deg bent
  under her weight, 35-45 as its toes leave at 60 % of the stride, 60-70
  swinging; the pelvis turning 6-10 deg) and what tells a woman's walk
  from a man's, her pelvis rolling more, her cadence higher on a shorter
  step, her feet nearer her line. A foot lands with its ankle furthest
  ahead of her hips and lifts with its toes furthest behind them (Zeni
  2008), 3-6 % of a stride late: over ground, on the spot and on a
  treadmill alike. Lengths in legs, times in sqrt(leg / g).
- Walks off it, by the bands' widths (2026-10-05): six mocap takes'
  walks 0.00 four times, 0.12 (a pelvis rolling 3.8 deg, 5-15) and 0.20
  (feet 0.26 legs apart, 0.02-0.22); a studio's catwalk loop 0.26, a
  runway's take 1.37 - arms 82 deg, roll 23; hand-keyed loops 1-2, no
  roll in their rig's hips. Her walk as built 2.01: the knee landing at
  30 deg (0-12), both feet down a quarter of a stride a step (stance 75
  %, 56-70), its step 1.04 for its time (1.1-1.8), the pelvis rising and
  falling 12 mm (15-54). The one law's 4.10, on stilts (the user): the
  swinging knee 26 deg (52-78), no heel's rise, the pelvis level, her
  feet 0.29 legs apart.
- Her walk as a take (2026-10-05). tests/takes/walk.fbx, `look.py
  --built --to 12 --fbx`: 355 frames, 6 s, 134 kB, binary FBX 7400, a
  LimbNode a segment by HumanIK's names; read back 0.000 mm off over 20
  joints. Its stamp is fixed; its FileId and its footer's code are that
  stamp's as the FBX SDK's own files' are theirs, 48 of 48 - the
  footer's three passes over the stamp's digits, the FileId's one.
  Opened in no other program yet.
- Held (2026-10-05). test_gynoid_gait.py: her walk the take's, each
  measure within 0.1 of its band's width - two windows of one walk 0.03
  apart at most -, and on the band but the five known out, none further;
  test_gynoid_going.py: the law's arms on the band, its walk no further
  off than 4.5. `mocap.py A.fbx B.fbx ..`, a column a take: off the
  band, and from the first.
- In words (2026-10-05; the user: what is a stiff walk without a soft
  one to hold it to, and Groucho's, a catwalk, a run; `normal.WORDS`,
  `GAITS`). A word is how far its measures are out of their bands on its
  side, by the bands' widths, said from 0.1: stiff - the swinging knee,
  the knee and the heel as the toes leave, under; crouched - the
  standing knee never straight; landing bent; still-hipped, still-armed,
  arms carried; tripping and striding - the step for its time; wide and
  on a line; swaying, swinging; shuffling; flying - the pelvis lowest
  over the standing foot where a walk's is highest (`vault`); slow,
  brisk. A gait is its words: on stilts stiff, Groucho's crouched, a
  catwalk swaying and not wide, a run flying. Read off 19 walks: four
  mocap walks no word, a woman's walk; a crouch walk crouched 4.3,
  Groucho's; six jogs and runs flying 0.4-1.6, a run; a runway's take
  swaying 0.8 and swinging 0.5, a catwalk; two limping runs crouched 1.0
  and wide 0.8. Her walk as built: landing bent 1.45, shuffling 0.37, no
  gait's name. The one law's: stiff 2.38, shuffling 0.78, still-hipped
  0.48, landing bent 0.45, tripping 0.42, wide 0.37 - on stilts, the
  user's word for it.
- Asked in the same words (2026-10-05; the user: a middle layer that
  blends, concepts as tuples; `style.MANNERS`, `look.py --manner`, the
  body's 'manner' command). A manner is a line through the walk as
  built's constants from where they are tuned, asked ((manner, amount),
  ..) and summed: crouched 1 the standing knee at 24 deg - read back
  crouched 0.56, Groucho's, 465 J/m where 434; at 15 deg no word yet.
  Catwalk 1 the style's catwalk end - swaying 0.11, a catwalk; at 2,
  swaying 0.75. Swagger 1 its other end - still-hipped 0.13. No line yet
  for leaning, tripping and wide: her stride at 0.6 m fell at 6.6 s, her
  feet 2 cm wider drew 1008 J/m, the pre-swing at 8 deg fell at 10 s,
  her lean is her start's alone - the walk as built is as narrow an
  optimum as the one law's. A walk into the wind, by four takes of one:
  crouched 1.0-1.9, landing bent 1.5-2.5, tripping 0.9-1.1, wide
  0.5-0.8, leaning 0.4-1.2 - the trunk 8-22 deg ahead -, slow.
- The one law's walk since 2026-10-06 ([going](going.md)): 0.34 off the
  band where 4.10, stiff 0.31 and shuffling 0.24 where stiff 2.38; 0.20
  with its free foot's raise eased, stiff 0.17 - the heel 22 deg up as
  its toes leave where 28, the knee 29.7 where 30, the pelvis turning
  3.5 deg where 4. A gait is named from how much of its words (`GAITS`):
  on stilts from stiff 0.5, Groucho's from crouched 0.4, a run from
  flying 0.25. test_gynoid_going.py holds eleven of its measures on the
  band, the walk within 0.6 of it and its words under 0.5.
- Asked on the one law (2026-10-06; `gaits.MANNERS`, `go.py --manner`,
  the page's M, Z and X). A manner is a row's setpoints moved, or other
  manners, its amount 0 to 1 coming on and going over 2 s as she walks,
  the less the less a walk's the row is: leaning her trunk 9 deg more;
  crouched the standing knee - the row's `strut`, a setpoint since - 18
  deg more and the landing knee 14; wide her feet's track 36 mm;
  tripping a step 0.1 s shorter; catwalk the pelvis's list 1.9 deg more
  and its turn 12.9; into the wind leaning 1, crouched 0.8, tripping
  0.4. One walk through them and plain again, read back: leaning 0.50 at
  559 J/m; crouched 0.58, Groucho's, at 409; wide 0.68 at 495; tripping
  0.36 with shuffling 0.57 - a shuffle, the heel 13 deg up as its toes
  leave - at 666; catwalk swaying 0.40, a catwalk, the pelvis turning 29
  deg and rolling 11.5, at 570; into the wind leaning 0.48, crouched
  0.31, landing bent 0.77 at 529; plain again 0.21 off the band at 497 -
  up through all six on 12 timings of 12. Bent legs carry a lean: leaned
  12 deg more she fell from her stand, with the standing knee at 18 deg
  she walked so, at 24 deg leaned 15 more. The catwalk came with the
  free foot's raise eased and no foot hurried: before, its pelvis turned
  5 deg fell at 3.9 s; now 16 deg walks, and with the list at 6 falls
  from her stand. test_gynoid_going.py holds each manner's words and the
  plain walk after them; M picks a manner on the page, Z and X its
  amount, each keeping its own. The user's frame (2026-10-06): a base -
  a pose, a gait -, its accents by amount, a transition between two. As
  built: standing is the gait's row at no speed, so one base and its
  accents within what bears her; a transition a thing of its own where
  two bases' mix is no movement - her walk to her jog by the knots QUICK
  and EASE and a stay, 23 of 24 where 2 of 6 mixed straight - or where
  what bears her changes.
- A pose's accent and its way into a walk (2026-10-06; the user: a pose
  a base and its accents, a transition from it to a gait; `gaits.POSES`,
  `passed`). Two setpoints her stand alone reads: `weigh`, how far
  toward one foot her weight is, and `hang`, the pelvis rolled up over
  that leg. hip left, hip right: 77-80 % of her on that sole, the pelvis
  rolled 5.5 deg, the other knee 29 deg where 10, the pelvis 19 mm down;
  from one side to the other in 2 s, no step. Asked on from it at once
  she was down in 1.2 s, her capture point 63 mm outside the loaded foot
  as the roll went. A transition is then a gate: a pose's accent is let
  go, 2 s, before the row leaves her stand, and is hers again as she
  stands - asked on at 6 s her first step at 8.1, 0.85 m/s by 11 s;
  asked to a stand at 14 s, standing at 15.5 and on her left leg again
  by 17. test_gynoid_going.py holds it. My reading of the user's
  "line-up": the weight on one leg, the hip out.
- Her slow walks strolled (2026-10-06; `gaits.STROLLS`, the accent
  `strolling`, asked by a slow walk's row itself). A woman's step keeps
  its ratio to its time at any pace; the law's slow walks kept the
  walk's 0.52 s: at the row -0.3, 0.59 m/s, a step 0.39 legs, her walk
  ratio 0.73 where 1.1-1.8, 1.53 off the band - on stilts, a shuffle. On
  a step 0.14 s longer: 0.52 m/s, the ratio 1.08, 0.73 off (shuffling
  0.55, stiff 0.39), no gait's name, at 695 J/m; at the row -0.15, 0.66
  m/s and 0.39 off at 550. Longer still she falls - 0.72 s at 0.6 m/s,
  0.70 at 0.45 -, and the row -0.6, 0.32 m/s, stays a shuffle on stilts:
  the page's S steps -0.15 and -0.3 now. As a knot of her way between
  her stand and her walk it felled her stops, 7 of 10 where 10, a stop
  passing through its step's time: it is an accent, and let go before
  she stops - stopped on the long step, down 3 timings of 12. Her ways
  76 of 104, the page's presses under the director 66 of 72, as before
  it.
