# Findings: a woman's walk as the metric

A normal feminine walk as a band of measures, a reference take (.fbx), a test
of what broke: `tools/sim/normal.py`, `fbx.py`, `mocap.py`,
tests/takes/walk.fbx. The walk as built: [walk](walk.md); the one law's:
[going](going.md). Mocap packs stay outside git, reference and test only.

## The band

23 measures from joints' places alone, so rows, a recording, a mocap take and
an animation compare alike; a band each (2026-10-05):

- Textbook gait: the knee all but straight at landing, 15-20 deg bent under
  load, 35-45 as its toes leave at 60 % of the stride, 60-70 swinging; the
  pelvis turning 6-10 deg.
- A woman's walk against a man's: the pelvis rolling more, a higher cadence
  on a shorter step, the feet nearer the line.
- A foot lands with its ankle furthest ahead of the hips and lifts with its
  toes furthest behind them (Zeni 2008), 3-6 % of a stride late: over
  ground, on the spot and on a treadmill alike.
- Lengths in legs, times in sqrt(leg / g).

## Walks off the band

Distance off the band in the bands' widths (2026-10-05):

| Walk | Off | Out of band |
| --- | --- | --- |
| six mocap walks | 0.00 (four), 0.12, 0.20 | pelvis roll 3.8 deg (5-15); feet 0.26 legs apart (0.02-0.22) |
| a studio's catwalk loop | 0.26 | - |
| a runway take | 1.37 | arms 82 deg, roll 23 |
| hand-keyed loops | 1-2 | no roll in their rig's hips |
| the walk as built | 2.01 | knee landing at 30 deg (0-12); both feet down a quarter of a stride a step (stance 75 %, 56-70); step 1.04 for its time (1.1-1.8); pelvis rising and falling 12 mm (15-54) |
| the one law, on stilts | 4.10 | swinging knee 26 deg (52-78); no heel rise; pelvis level; feet 0.29 legs apart |

## Reference take

tests/takes/walk.fbx, `look.py --built --to 12 --fbx` (2026-10-05): 355
frames, 6 s, 134 kB, binary FBX 7400, a LimbNode a segment by HumanIK's
names; read back 0.000 mm off over 20 joints. Its stamp is fixed; its FileId
and its footer's code derive from that stamp as in the FBX SDK's own files,
48 of 48: the footer three passes over the stamp's digits, the FileId one.
Opened in no other program yet.

## Tests

- test_gynoid_gait.py (2026-10-05): the walk as the take, each measure within
  0.1 of its band's width (two windows of one walk 0.03 apart at most); on
  the band but for the five known out, none further.
- test_gynoid_going.py: the law's arms on the band, its walk no further off
  than 4.5.
- `mocap.py A.fbx B.fbx ..`: a column a take, off the band and from the
  first.

## Words

`normal.WORDS`, `GAITS` (2026-10-05). Each quality is read against its
opposite (stiff against soft). A word is how far its measures are out of
their bands on its side, in the bands' widths, said from 0.1:

| Word | Measures over (+) or under (-) their bands |
| --- | --- |
| stiff | knee swinging, knee at lift, heel at lift - |
| crouched | knee straightest + |
| landing bent | knee at landing + |
| still-hipped | pelvis roll, turn, bob - |
| still-armed | arm, elbow - |
| arms carried | elbow bent + |
| tripping, striding | walk ratio -, + |
| wide, on a line | feet apart +, - |
| swaying | pelvis roll, pelvis turn, hips wag + |
| swinging | arm, elbow, knee swinging + |
| shuffling | stance +; toes up at landing, heel at lift - |
| flying | vault - (the pelvis lowest over the standing foot where a walk's is highest) |
| leaning, leaning back | trunk lean +, - |
| slow, brisk | pace -, + |

A gait is its words: on stilts stiff; Groucho's crouched; a catwalk swaying
and not wide; a run flying. Read off 19 walks:

| Walk | Words | Gait |
| --- | --- | --- |
| four mocap walks | none | a woman's walk |
| a crouch walk | crouched 4.3 | Groucho's |
| six jogs and runs | flying 0.4-1.6 | a run |
| a runway take | swaying 0.8, swinging 0.5 | a catwalk |
| two limping runs | crouched 1.0, wide 0.8 | - |
| the walk as built | landing bent 1.45, shuffling 0.37 | - |
| the one law | stiff 2.38, shuffling 0.78, still-hipped 0.48, landing bent 0.45, tripping 0.42, wide 0.37 | on stilts |

## Manners on the walk as built

`style.MANNERS`, `look.py --manner`, the body's 'manner' command
(2026-10-05): a middle layer that blends, concepts as tuples. A manner is a
line through the walk as built's constants from where they are tuned, asked
((manner, amount), ..) and summed.

| Manner | Read back | J/m |
| --- | --- | --- |
| crouched 1: standing knee 24 deg | crouched 0.56, Groucho's | 465 (434 plain) |
| crouched, knee 15 deg | no word | - |
| catwalk 1, the style's catwalk end | swaying 0.11, a catwalk | - |
| catwalk 2 | swaying 0.75 | - |
| swagger 1, the other end | still-hipped 0.13 | - |

No line yet for leaning, tripping and wide: a 0.6 m stride fell at 6.6 s;
the feet 2 cm wider drew 1008 J/m; the pre-swing at 8 deg fell at 10 s; the
lean is the start's alone. The walk as built is as narrow an optimum as the
one law's. A walk into the wind, four takes of one: crouched 1.0-1.9,
landing bent 1.5-2.5, tripping 0.9-1.1, wide 0.5-0.8, leaning 0.4-1.2 (the
trunk 8-22 deg ahead), slow.

## The one law's walk since 2026-10-06

[going](going.md): 0.34 off the band where 4.10; stiff 0.31 and shuffling
0.24 where stiff 2.38. With its free foot's raise eased 0.20, stiff 0.17:
the heel 22 deg up as its toes leave where 28, the knee 29.7 where 30, the
pelvis turning 3.5 deg where 4. A gait is named from its words' amounts
(`GAITS`): on stilts from stiff 0.5, Groucho's from crouched 0.4, a run from
flying 0.25. test_gynoid_going.py holds eleven of its measures on the band,
the walk within 0.6 of it, its words under 0.5.

## Manners on the one law

`gaits.MANNERS`, `go.py --manner`, the page's M, Z and X (2026-10-06). A
manner moves a row's setpoints, or other manners; its amount 0 to 1 comes on
and goes over 2 s while walking, less the less a walk's the row is:

| Manner | Setpoints | Read back | J/m |
| --- | --- | --- | --- |
| leaning | trunk 9 deg more | leaning 0.50 | 559 |
| crouched | standing knee (`strut`) 18 deg more, landing knee 14 | crouched 0.58, Groucho's | 409 |
| wide | track 36 mm | wide 0.68 | 495 |
| tripping | step 0.1 s shorter | tripping 0.36, shuffling 0.57 (heel 13 deg up as the toes leave) | 666 |
| catwalk | pelvis list 1.9 deg more, turn 12.9 | swaying 0.40, a catwalk; pelvis turning 29 deg, rolling 11.5 | 570 |
| into the wind | leaning 1, crouched 0.8, tripping 0.4 | leaning 0.48, crouched 0.31, landing bent 0.77 | 529 |
| plain again | - | 0.21 off the band | 497 |

- One walk through all six and plain again: up on 12 timings of 12.
- Bent legs carry a lean: leaned 12 deg more it fell from its stand; with the
  standing knee at 18 deg it walked so; at 24 deg it leaned 15 more.
- The catwalk needed the free foot's raise eased and no foot hurried: before,
  a pelvis turned 5 deg fell at 3.9 s; now 16 deg walks, and with the list at
  6 it falls from its stand.
- test_gynoid_going.py holds each manner's words and the plain walk after
  them; M picks a manner on the page, Z and X its amount, each keeping its
  own.
- Frame (2026-10-06): a base (a pose, a gait), its accents by amount, a
  transition between two. As built: standing is the gait's row at no speed,
  so one base and its accents within what bears the weight; a transition is a
  thing of its own where two bases' mix is no movement (walk to jog by the
  knots QUICK and EASE and a stay: 23 of 24, where 2 of 6 mixed straight) or
  where what bears the weight changes.

## A pose and its way into a walk

`gaits.POSES`, `passed` (2026-10-06): a pose a base with accents, a
transition from it to a gait. Two setpoints the stand alone reads: `weigh`,
how far toward one foot the weight is, and `hang`, the pelvis rolled up over
that leg.

- Hip left, hip right: 77-80 % of the weight on that sole, the pelvis rolled
  5.5 deg, the other knee 29 deg where 10, the pelvis 19 mm down; one side to
  the other in 2 s, no step.
- Walk asked from it at once: down in 1.2 s, the capture point 63 mm outside
  the loaded foot as the roll went.
- A transition is therefore a gate: a pose's accent is released over 2 s
  before the row leaves the stand, and restored on standing. Asked on at 6 s:
  first step at 8.1, 0.85 m/s by 11 s; asked to stand at 14 s: standing at
  15.5, on the left leg again by 17. test_gynoid_going.py holds it.
- 'Line-up' read as: the weight on one leg, the hip out.

## Slow walks strolled

`gaits.STROLLS`, the accent `strolling`, asked by a slow walk's row itself
(2026-10-06). A woman's step keeps its ratio to its time at any pace; the
law's slow walks kept the walk's 0.52 s:

| Row | Step | m/s | Walk ratio | Off the band | J/m |
| --- | --- | --- | --- | --- | --- |
| -0.3 | as walking | 0.59 | 0.73 (1.1-1.8), 0.39 legs | 1.53, on stilts, a shuffle | - |
| -0.3 | 0.14 s longer | 0.52 | 1.08 | 0.73 (shuffling 0.55, stiff 0.39), no gait's name | 695 |
| -0.15 | 0.14 s longer | 0.66 | - | 0.39 | 550 |

- Longer still it falls (0.72 s at 0.6 m/s, 0.70 at 0.45); the row -0.6,
  0.32 m/s, stays a shuffle on stilts. The page's S steps -0.15 and -0.3.
- As a knot of the way between stand and walk it felled the stops, 7 of 10
  where 10, a stop passing through its step's time: it is an accent, released
  before stopping. Stopped on the long step: down 3 timings of 12.
- The ways 76 of 104, the page's presses under the director 66 of 72, as
  before it.
