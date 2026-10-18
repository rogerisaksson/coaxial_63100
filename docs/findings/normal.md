# Findings: a woman's walk as the metric

A normal feminine walk as a band of measures, a reference take (.fbx), and a
test of what broke: `tools/sim/normal.py`, `tools/sim/gvs.py`,
`tests/takes/walk.fbx`. Mocap packs stay outside git, reference and test only.
Her walk on the loop: [orthokinematics](orthokinematics.md).

## The band

25 measures from joints' places alone (`normal.BAND`), so rows, a recording, a
mocap take and an animation compare alike; a band each:

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

Distance off the band in the bands' widths:

| Walk | Off | Out of band |
| --- | --- | --- |
| six mocap walks | 0.00 (four), 0.12, 0.20 | pelvis roll 3.8 deg (5-15); feet 0.26 legs apart (0.02-0.22) |
| a studio's catwalk loop | 0.26 | - |
| a runway take | 1.37 | arms 82 deg, roll 23 |
| hand-keyed loops | 1-2 | no roll in their rig's hips |
| the loop, striding (`tools/sim/ortho.py --walk --to 40 --look`) | 2.46-2.49 | knee at landing 32.4 deg (0-12); walk ratio 1.02 (1.1-1.8); knee swinging 49.4 (52-78); knee at lift 20.0 (30-60); elbow 2.7 (5-45) |

## Reference take

`tests/takes/walk.fbx`: 355 frames, 6 s, 134 kB, binary FBX 7400, a LimbNode a
segment by HumanIK's names; read back 0.000 mm off over 20 joints. Its stamp is
fixed; its FileId and its footer's code derive from that stamp as in the FBX
SDK's own files, 48 of 48: the footer three passes over the stamp's digits, the
FileId one.

## Tests

- `tests/test_ortho.py`: her walk on the loop measured against `normal.BAND`
  and `tools/sim/gvs.py`.
- `tools/sim/gvs.py`: Hof-scaled geodesic Lie-algebraic SO(3) diff engine
  against biological ground truth, enforcing the CI guillotine gate (GPS).

## Doctrine: Mocap as Descriptive Truth, Not Prescriptive Law

1. **Descriptive Truth, Never Prescriptive Control:**
   Mocap data defines the normative destination, never runtime instructions.
   The robot never tracks a mocap trajectory; the controller remains
   completely unaware of biological gait data. Mocap serves strictly as an
   external answer key to verify whether the physical modeling is complete.

2. **Absolute Prohibition of Phenomenological Tuning:**
   When the solver settles into an unnatural minimum (e.g. crouching,
   wide-base waddling, or frantic tripping), adding ad-hoc damping filters,
   deadbands, or heuristic anti-wobble weights is strictly prohibited. An
   aberration proves that the cost function or constraint set in Lisp lacks
   a thermodynamic, frictional, or structural constraint present in nature.

3. **Deviation-Driven Development (GPS/GVS CI Guillotine):**
   Pure physics (KKT, dynamic Contact Wrench Cones, mass-inertia) is
   formulated in Lisp, compiled to deterministic DoD C code executed from
   STM32 DTCMRAM, and benchmarked against mocap via Hof-scaled Lie-algebraic
   SO(3) GVS (`tools/sim/gvs.py`). If the GPS metric exceeds tolerance, the
   build terminates. Corrections must occur exclusively in the high-level
   physical formulation, never via runtime parameter fudging.

4. **Discovery of Hidden Constraints:**
   Mocap demonstrates *what* biology executes; development systematically
   deduces *why* (the underlying mechanical or energetic cost). Constraints
   are identified and priced until the solver's most economical, least-effort
   solution is naturally isomorphic to normative biological motion. Once
   compiled and deployed to the MCU, mocap is completely erased from the
   equation—leaving only the first-principles physics it demanded.

## Words

`normal.WORDS`, `normal.GAITS`. Each quality is read against its
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
and not wide; a run flying:

| Walk | Words | Gait |
| --- | --- | --- |
| four mocap walks | none | a woman's walk |
| a crouch walk | crouched 4.3 | Groucho's |
| six jogs and runs | flying 0.4-1.6 | a run |
| a runway take | swaying 0.8, swinging 0.5 | a catwalk |
| two limping runs | crouched 1.0, wide 0.8 | - |
| the loop, striding | landing bent 1.70, stiff 0.32-0.43, tripping 0.11-0.35 | - |
