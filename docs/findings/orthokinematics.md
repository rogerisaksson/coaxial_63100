# Findings: orthokinematics

The robot's movement as text: compiled to rows of numbers, stepped in a latent
space with no branch, rendered on its body by its modes, solved by the
whole-body step without an active set ([wrench](wrench.md)).
`machine/texts/ortho.py` compiles, `machine/law/latent.py` steps,
`machine/law/impedance.py` renders - the robot's modes read by
`machine/law/modal.py`, its soles' polygon by `machine/law/poise.py`,
its stride by `machine/law/stride.py` -, `machine/process/sprung.py`
runs it: the HUMANOID page, where nothing else moves it, and
`tools/sim/ortho.py`. tests/test_ortho.py holds it, 43 of 43. The robot's moves
are `lisp/humanoid/gynoid_mesomorph_medium.lisp`; the language's first texts, as
given, `lisp/humanoid/examples.lisp` (`>` on the page).

## The text

| Clause | Says |
| --- | --- |
| `(stream_inject ..)` | a payload: one row, injected as it is said |
| `(defbehavior NAME (phase NAME (duration 400ms) ..) ..)` | rows in a table, each at its time, then the robot's rest over 200 ms |
| `(sequence_id N)` `(blend_duration 800ms)` | its id; how long the robot's state takes there, a straight line |
| `(latent_targets (MODE N) ..)` | a mode's target, -6 to 6: past 1 off the robot's soles' own reach |
| `(impedance (stiffness MODE N) (damping MODE N) ..)` | N/m and N s/m; `global` every mode; `MAX_SYS_K` 80 kN/m |
| `(cpg_config (enable true) (frequency 0.8Hz) (amplitude MODE N) (phase_offset MODE OF 90deg))` | the robot's rhythm: one angle, a sine a mode |
| `(cpg_override (halt_oscillators))` | the robot's rhythm off, in a phase |
| `(task_constraints (maintain_zmp (polygon support_polygon)) (limit_yaw_momentum N))` | what holds whatever is asked; or `dynamic_footprint` |

What a row leaves unsaid stays as it was. A row on the wire is `<4I59f`, 252
bytes: id, at ms, blend ms, flags (1: the robot's rest), then the 59 parameters
as its state lays them, NaN unsaid.

## The robot's modes

Each a mass on a spring and a damper to its reference.

| Mode | Of | At 1 | Moves |
| --- | --- | --- | --- |
| `axial_compression` | the robot's standing legs, hip to ankle, shorter than it stood on | the robot's centre of mass 0.115 m down, its hips 0.142 | the robot's 31.1 kg |
| `lateral_shift` | the robot's soles' places from its centre of mass, their sum, across | over the robot's first sole, 0.095 m | the robot's 31.1 kg |
| `sagittal_shift` | the sum, along | at the robot's toes' edge, 0.086 m | the robot's 31.1 kg |
| `sagittal_drive` | their difference, along | 0.6 m apart, the first behind | a limb's 5.95 kg |
| `limb_lift` | the difference, up | the robot's first sole 0.02 m over the other; its rhythm's amplitude a swinging sole's clearance, 0.02 m more | a limb's |
| `stance_width` | the difference, across | twice as wide | a limb's |
| `pitch_attitude` | the robot's trunk and pelvis | 0.5 rad ahead | the robot's trunk's 14.6 kg |
| `pelvis_list` | the robot's pelvis's roll | 0.15 rad, the robot's first hip up | the robot's trunk's |
| `pelvis_turn` | the robot's pelvis's turn | 0.2 rad to the robot's left | the robot's trunk's |
| `arm_swing` | the robot's shoulders, against each other | 0.35 rad, the first ahead; an elbow 0.4 of it | an arm's 1.2 kg |
| `foot_rock` | the robot's soles' pitch, against each other | a heel 0.75 rad up behind the robot, the toes 0.25 up ahead | a limb's |

## A pass

- `latent.step`: the row whose time has come laid over the robot's state by a
  mask, the state a lerp over the row's blend, its rhythm one angle; out
  go each mode's reference with its rate and acceleration, its K and D, its
  phase.
- `impedance.ask`: the sum's springs are the robot's centre of mass's
  acceleration. Along and across, its ZMP brought into the robot's soles
  (`poise`). Up, what the robot's height's spring asks of its standing
  legs' length, each leg's acceleration along itself with what its going
  over it gives taken off: over a leg held at its length the robot rises and
  comes down as its arc does.
- A sole in the air is held on the difference's impedance: up against the
  floor the standing soles touch, along and across against the robot's centre of
  mass, led by its speed past its cruise over omega; with none standing, in
  its pelvis's frame.
- `poise.stands`: a sole asked up leaves once the robot's ZMP as the robot's
  springs ask it lies in the other's box and the robot sways within 0.1 of what
  is asked.
- The robot's cruise is the speed its drive makes of its steps, no more
  than 0.15 m/s from what the robot's soles have made over the last second;
  along its way it is damped to it and held to no place.
- No branch: `latent.py`, `impedance.py`, `stride.py` and `poise.py` have no
  if, loop or dict, `modal.py` no if; a choice is a mask, a limit a minimum
  or a maximum, a change a blend (test_ortho.py reads the files' trees).
- The reasons for its constants stand beside them in those files.

## The robot's stride

`machine/law/stride.py`, from the robot's drive's phase alone.

| What | How |
| --- | --- |
| swing | 0.4 of a turn, about where its drive carries the limb on fastest; the robot's sway 108 deg ahead of its drive is forward, behind it back |
| letting go | from where the other's swing ends a sole stands over its ball, its heel rising, the robot's ZMP in the other sole; it leaves as its own swing begins and the other has landed |
| landing | where it meets the robot's capture point: a step's offset short of it along, `cruise T / (e^(omega T) - 1)`, its own side of it by `wide / (e^(omega T) + 1)`; further on by 0.5 of the robot's speed past its cruise over omega; 0.35 m from it at most, 0.1 m its own side of the other |
| getting there | a spring stiffening as it nears, 6 over the time left squared, damped 4 over it; over its landing 0.9 through its swing |
| clearance | up over the first 0.2 of its swing, down over the last 0.25, held between; what its pitch brings its heel or its toes down by added; past its swing and not down, on down at 0.3 m/s |
| rock | in the air its pitch is the robot's rock's; standing, its heel rises as far as its leg, at its length, needs for the robot's hip 6 mm under where it stood, 0.6 rad at most at 6 rad/s |
| the fold | a standing sole its stride has off folds its knee as its load leaves it, 0.26 rad with none on it: its leg that much shorter for its heel's rise, the robot's knees' row told |
| the edge | the whole-body step turns a standing sole about its ball (`settle`) with its centre of pressure there (`edge`), the robot's ZMP's box the ball; striding only, and what the floor cannot give there its inertia takes (`wrench._taken`) |
| the robot's knees | a soft stop in the drives' loop, 20 N m at straight fading over 0.1 rad, the observer told of it: the step alone left a straight standing knee locked at -5 deg |

## Measured

`python tools/sim/ortho.py NAME`, as the page runs it: the step solved every
third pass, 1.06 ms a pass walking, the page 0.9 of real time.

| Asked | Is | Stood |
| --- | --- | --- |
| `crouch`, axial 0.6 at 20 kN/m | 0.589 | yes, tilt 1.0 deg |
| `weight_shift`, lateral 1.0 | 1.00, the other sole 19 N of 306 | yes |
| `bow`, pitch 0.5 | 0.502 | yes |
| `one_sole`, lateral 1.0 and lift -1.0 in one row | 1.00, lift -0.98, the other sole 0 N | yes, 4 s on one |
| `hop`: loaded 0.85, thrown by a spring that lets go at 0.15, the robot's trunk free, landed at 0.25 | 1.10 m/s up, 0.21 s off the floor | yes, tilt 6.3 deg |
| `ballistic_walk`: APA 0.6 s at 0.83 Hz, stride 0.65 drive, 0.02 axial | 0.70 m/s, step 0.481 legs, walk ratio 1.03, GPS 14.15 deg | yes, 20 s |

### The robot's walk

201 to 204 of `gynoid_mesomorph_medium.lisp`: stand, sway at 0.83 Hz and 0.31
(2.50 cycles over the 3.0 s before `#203`, aligning the lateral weight shift
with step onset), step at a 400 ms blend and a drive of 0.58 (`GOING_S` 0.45 s,
`AHEAD_M_S` 0.22 m/s, sagittal damping 260 N s/m), walk on at 0.65 with axial
compression 0.02, the robot's rock 0.6, its list and turn 0.4, its arms 0.8. 51
s: 0.70 m/s, step 0.475 legs (in `normal.BAND`), walk ratio 1.02, initiation
time `init_tau` 7.2 (`sqrt(L/g)` to reach `dxi*/dtau >= 0.25`, down from 17.1),
backward swing kick 0.001 legs (0.8 mm, down from 41 mm), rig-invariant SO(3)
mocap GPS 13.82 deg (`<= 15.0 deg`, `tools/sim/gvs.py`,
`tools/sim/ballistic.py`).

Through the eye's measures (`python tools/sim/ortho.py --walk --to 40 --look`),
against a woman's band (`normal.BAND`, 25 measures):

| Walk | m/s | Off the band | By name |
| --- | --- | --- | --- |
| the walk as built, before the loop | 0.65 | 2.07 | landing bent 1.49, shuffling 0.38 |
| the one law's, before the loop | 0.82 | 0.26 | shuffling 0.25, stiff 0.21 |
| the loop, the robot's height level and its soles flat | 0.28 | 7.71 | on stilts, Groucho's, a shuffle |
| the loop, striding, the rows traded on a sole's edge | 0.61 | 2.60 | none: landing bent 1.62, tripping 0.48, stiff 0.37, shuffling 0.11 |
| the loop, striding (0.9 Hz, 0.2 step drive, 1.5 s blend) | 0.69 | 2.46 | none: landing bent 1.70, tripping 0.35, stiff 0.32 |
| the loop, ballistic initiation (0.83 Hz, 0.58 step, 0.65 walk) | 0.70 | 2.49 | none: landing bent 1.70, stiff 0.43, tripping 0.11 |
| two takes of women | 1.11, 1.14 | 0.00 | - |

A stagger, the band's blind spot: the robot's pelvis's fall into a landing and
its soles' loads then, of its 306 N.

| Walk | Falls into a landing, mm | Loads' sum then, N | After, at least |
| --- | --- | --- | --- |
| the one law's: its foot flat till the other landed, thrown back up in 0.11 s | 32 | 604 | 150 |
| the same on the page, recorded | 23 to 33 | 481 to 505 | 152 to 202 |
| the walk as built | 10 | 421 | 269 |
| the loop, striding | 17 | 485 | 183 |

By the eye's own measures (`tools/sim/ortho.py --look`: landing load, unloading,
dip, head nod), striding: the robot's soles bear 1.34 of its
weight about a landing (1.25) and 0.63 after (0.75), its pelvis falls 15
mm into it, the robot's head nods 3.5 deg (6) - 1.37, 0.70, 12 mm and 46 deg
with the rows traded.

Striding, 305 J/m, 19 of the 25 in (step now in `normal.BAND`):

| Measure | Is | A woman's |
| --- | --- | --- |
| knee at landing, deg | 32.4 | 12 and under |
| step, legs | 0.475 | in (0.45 to 0.90) |
| walk ratio | 1.02 | 1.1 and over |
| knee swinging, deg | 49.4 | 52 and over |
| knee at lift, deg | 20.0 | 30 and over |
| elbow, deg | 2.7 | 5 and over |
| heel at lift, deg | 30.9 | in |
| stance, % | 66.1 | in |
| knee straightest, deg | 14.5 | in |
| toes up at landing, deg | 8.7 | in |
| arm, deg | 27.9 | in |
| trunk lean, pelvis roll, turn, deg | 3.4, 5.3, 7.6 | in |
| pelvis bob, vault, hips wag, feet apart, legs | 0.027, 0.020, 0.052, 0.139 | in |

- Of the eye's 12 form bounds, 5 broken: knee behind plumb 20.6 deg (10), hip over
  stance 2.2 (3), head aside 46.8 mm (35), landing load 1.34 (1.25), unloading
  0.63 (0.75). Strike 412 N, head bob 24 mm.
- The robot's trunk turns 5.9 deg with its pelvis and its shoulders wag 36 mm,
  1.1 and 23 with the rows traded: its arms swing 27.6 deg, as asked, where 33.5
  as the step's wheel about it.
- With the rows traded on a sole's edge the robot's head nodded 46 deg, its neck
  -26 to +21. A swinging leg takes 3.6 N m s of the robot about its left
  as it leaves and gives 3.4 back before the floor, 0.75 at it: 16 to 27 N m
  either way. On its ball a standing sole's centre of pressure is that line
  (0.083 of 0.083 m) and the step's rows are one too many: traded among them, a
  row a joint, the robot's neck is asked -196 rad/s^2, 1.21 of its clamp, and
  the swinging sole gets -5.8 of the -10.9 m/s^2 asked. One mass at that sole
  tells 0.67 of the leg's moment (1.45 kg), the robot's mass matrix on the
  acceleration asked 0.2 to 0.5, on its twist as it went 0.95.
- A step's plan gains 0.14 m/s by itself: the robot's centre of mass past the
  standing sole's ball till the next lands, less the landing's brake till
  it is over its heel - in v^2, `omega^2 (vT - l - r)(vT - 2v/omega +
  2b - l + r)`, l 0.086, r 0.068, b 0.063 m. Asked 0.11 to 0.17 m/s a step, the
  robot gained none: the landing's blow took it. The robot's speed was its
  balance, held by nothing: every step that landed softer ran away till a
  landing was led by the robot's speed's error.
- The robot's knee lands bent because its hip is 1 to 2.5 cm under where
  a straight leg needs it: a sole lands 4 to 10 cm ahead of its hip, the other
  15 to 30 behind. 1.2 cm of hip is 15 deg of knee.
- Over a straight leg 16 cm ahead the robot rises 17 mm: 0.58 m/s to do it.
- 205 after 203: back 0.45 m in 12 s, -0.17 m/s.

The step under the robot's walk, 40 s each (`wrench.step`'s rounds; a sole's
lead its landing further on by that of (v - cruise) / omega):

| Step | Stood | m/s | Head nod, deg | Trunk pitch | Landing load | Strike, N | Knee as the robot's toes leave |
| --- | --- | --- | --- | --- | --- | --- | --- |
| the rows traded again round what was clipped | 51 s | 0.61 | 46 | 1.0 | 1.37 | 440 | 31 |
| what the floor cannot give taken by the robot's inertia - the least change of its kinetic energy to the faces its soles are past | down at 16.5 s | 0.82 | - | - | - | - | - |
| the same, the robot's centre of mass and the swinging sole held | down at 12.1 s | - | - | - | - | - | - |
| the same, the robot's centre of mass let go on a sole's ball alone | down at 38.7 s | 0.98 | - | - | - | - | - |
| the rows traded, the swing's reaction in the robot's centre of mass's ask | down at 14.2 s | - | - | - | - | - | - |
| the robot's inertia, the swinging sole held, a lead of 0.5 | 51 s | 0.69 | 2.0 | 1.8 | 1.33 | 389 | 18 |
| the same, a knee folding 0.26 rad before its swing | 51 s | 0.69 | 2.5 | 1.9 | 1.40 | 433 | 24 |
| the same, a lead of 1.0 | 51 s | 0.66 | 3.4 | 3.7 | 1.41 | 433 | - |
| the same, a lead of 0.25; none | down at 36 s; at 19 s | 0.76 | - | - | - | - | - |
| a lead of 0.5 and the robot's speed's error summed, 0.05 to 0.2 /s | 71 s | 0.62 | 7.5 to 9.0 | 6.4 to 7.9 | 1.40 | 457 to 484 | - |
| the rows traded, a lead of 0.5 | down at 16.5 s | - | - | - | - | - | - |
| the rows traded, the robot's upper body's rows at 3, 10, 30 | down within 18 s | - | - | - | - | - | - |

- By the step's own map a lead of 0.5 leaves 0.42 of a step's speed error
  to the next, 1.0 turns it -0.09, none 0.93.
- The fold. With the rows traded the robot's trailing knee folded before the
  robot's toes left because its heel lagged what was asked, 5.5 deg of 8.8 as
  the other landed; on a step that follows the ask the leg stays at its length,
  18 deg where a woman's is 30 and over: stiff 0.62, on stilts by name. Asked of
  one piece as its load leaves it - the knee, its leg that much shorter for its
  heel's rise, the robot's knees' row -: 0.17 rad stiff 0.41; 0.26, 0.32 and
  2.46 off; 0.35, its head 7 deg and strike 489 N; 0.44, 13 deg; 0.52,
  down at 14 s. Asked of the robot's heel alone, 0.2 to 0.5 rad past its rock's,
  down at 11 to 32 s; its rock at 0.9 and 1.2, at 19 s and before.
- Standing, the rows are traded as they were, bit for bit. By faces and
  the rows' own metric: 7, 5, 3, 2 and 0 of 8 at 38 to 120 N where 8, 6,
  4, 2 and 0, a hop's tilt 9.6 deg where 2.4; a throw's soles pressed
  nothing till the robot's height's band was decided before their faces.
- The robot's fall into a landing is asked: its height's law has the
  robot's centre of mass down at 1 to 2.7 m/s^2 from its hip's passing
  the standing sole's ball, 0.22 s before the next lands, till 0.1 s after - the
  robot's soles' sum 0.72 of its weight by the plan as it lands; the
  sole behind goes from 320 N to 59 in 45 ms, the robot's ZMP's polygon the
  landed sole's alone from that pass.
- The robot's legs weighed by what they bear in its height's reading, not one
  each standing: 0.68 after a landing where 0.65, a fall of 15 mm where
  16, 294 J/m, 2.42 off - and the first texts' walk 1.50 m in 9 s where
  1.5 and over are held: not kept. Each leg's rate its own, its hip from
  its ankle against the pass before, where the robot's centre of mass's against
  its sole's reads a leg 9 cm/s longer as its heel rises: the robot's height's
  ask -0.6 m/s^2 through late stance where -2.1, and its head 8 deg, the
  robot's trunk 7, strike 530 N. The misread rate is part of the robot's
  height's damping.
- A fall, standing, 120 N ahead for 0.12 s: down in 1.04 s; the robot's arms a
  turn to two round its shoulders, its hands on the floor at 5 to 16 m/s, its
  trunk at 3.1 with 2.6 kN, its head at 2.4 with 3.3 kN; 80 N: its pelvis first,
  2.4 m/s and 5.4 kN; 120 N across: its head in 1.45 s at 3.7 m/s. Once a part
  but a sole is on the floor the step asks the robot's neck 99 of its clamp, a
  hip's turn 85. With a slack - the robot's capture point past the robot's
  soles, 3 cm halving it - taking every mode's stiffness and the robot's
  weight's support off, tried beside the tree: its head at 1.6 m/s with
  1.3 kN, its arms still till the floor, and its thighs on it in
  0.8 s at 1.4 m/s with 6.8 kN; from there a hip's turn is asked 72 of its
  clamp, the robot's spine 46: the step knows no floor but under the robot's
  soles. That slack costs no shove the robot stood: on the loop, standing, 8, 7,
  4 and 2 of 8 ways at 38, 60, 80 and 100 N with it and without. Yielding from
  0.12 s after the shove the robot is 0.48 m past its toes 0.54 s after
  it, pressed on at 1.0 m/s and down at 0.8, 0.14 m lower, and the robot's soles
  bear nothing from 0.6 s: standing, its legs have half a second and no
  step - a fall is not theirs to damp where it stands.
- Shoved walking, 0.12 s at 15 s - 20, 30 and 40 N across either way, 10
  and 20 N along either way -: 6 of 10 stood on either step, the four
  down each across; the robot's speed after one 0.75 to 0.77 m/s where 0.70 to
  0.93.

What was tried on it, each 30 or 40 s:

| Tried | Measured |
| --- | --- |
| a drive of 0.8, 1.0, 1.2 | 0.75 to 0.82 m/s, down at 17 to 26 s: the robot's speed surges as a step lengthens, the swing lands short, the sole behind its 42 cm back |
| the robot's rhythm hastened with its speed | a drive of 1.0: 0.83 m/s for 40 s in one build, head bob 122 mm, strike 852 N; down at 12 to 29 s in the ten runs after |
| the swing's rows heavier in the step, 60 to 100 | 0.88 to 1.02 m/s for 40 s at 557 to 611 J/m, 3.2 to 4.1 off: arms 110 to 134 deg, head bob 166 to 187 mm, strike to 1 013 N |
| the robot's ankle fed forward what its leg's length asks | down at 22 s, strike 910 to 1 253 N for 467 |
| the robot's hips held where a straight landing needs them | knee at landing 27.2 deg for 31.8, 426 J/m for 337, head bob 81 mm for 21; at a drive of 0.8 down at 17 to 20 s |
| a sole placed half a step ahead of the robot | 0.2 m/s; down at 15.6 s |
| a sole placed on its sweep's place from the robot | 15 cm ahead of the robot at 0.31 m/s, 7 cm past where it could go over it: the robot stopped and went back |
| a sole placed where the robot's capture point was foreseen, e^(omega t) on | 0.33 m/s where 0.46 was asked, 0.06 m/s lost a landing |
| a sole let go by its ankle's place | down at 14.8 and 18.8 s |
| a sole let go only with the robot's ZMP in the other | a step 0.23 legs at 0.28 m/s |
| a sole flat till it left | 25 cm behind the robot, its hips 4 cm down as the other landed, on a knee at 36 deg |
| the robot's hip let no dip before a heel rises | down at 13.3 s |
| a heel 0.8 further up behind the robot, for 0.5 | down at 12.7 and 17.5 s |
| 3 cm clear, for 4 | down at 20.3 s |
| 1.0 Hz, sway 0.27 | 0.65 m/s, 345 J/m, 2.93 off, knee at landing 32.8 deg |
| the robot's height's spring 8 000 N/m | 0.57 to 0.61 m/s, 342 to 353 J/m, 2.64 to 2.75 off |
| axial 0.03 | 0.60 m/s, 300 J/m, 2.72 off |
| solved every second, fourth, fifth pass | 0.61 m/s; 326, 334, 338 J/m; 2.61, 2.57, 2.54 off |

### The first texts as given

`examples.lisp`:

| Text | On the robot |
| --- | --- |
| 101, stand relaxed | stands, 0.15 compressed |
| 102, the gait: sway 0.35 at 1.1 Hz, 1 200 N/m, 15 N s/m | the robot's ZMP asked 1.91 of the way to a sole; it steps, each sole to its capture point |
| 103 after it: the dynamic footprint | walks: 1.90 m 6 s on, 0.33 m/s, tilt 2.1 deg |
| `hop_asymmetric` | loaded to 0.80, 1.12 m/s up, 0.28 s off the floor; not stood |

- The hop as given asks full extension at `MAX_SYS_K` and has no landing:
  off the floor the robot's legs straighten at once, its pelvis goes back 10
  deg, its trunk 24 forward, and it lands with its soles 0.15 m ahead of it.

## The steps under it

| Whole-body step | The robot's walk at a level height, a drive of 0.5, 60 s | At 0.8 | About the robot at a hop's lift-off |
| --- | --- | --- | --- |
| the stack of QPs (`wbc.step`), every fourth pass | 8.43 m; down at a drive of 0.6 | - | 4.1 kg m^2/s, not stood |
| without an active set (`wrench.step`), every third | 8.29 m | 13.93 m | 0.8, stood |

| Drives' own loop | Sway asked 0.274 at 0.9 Hz, both soles down |
| --- | --- |
| PD round the reference integrated from the loop's accelerations | 0.359, 250 ms behind |
| each drive's disturbance observed at 100 rad/s and taken off | 0.271, 5 ms behind |
| the torque as asked | 0.255, 10 ms behind |

- Under the PD 40 to 60 % of a small acceleration asked was done; the loop
  runs on the observers.
- A pass on the stack costs 3.2 ms solved every pass - the stack 2.1, the
  renderer 0.32, `wbc.sense` 0.16, `latent.step` 0.07 -, a third of real
  time.

## The stunts as entries

The stunts were entries a case each (`sit_chair` to `controlled_fall`), measured
beside crutches since taken out: the chair moved under the robot's mid-hop,
obstacles and sills found by body id, a shove's newtons choosing `parry_shove`
or `controlled_fall`. Their outcomes stood on those and are void; TODO's
Scenarios carry what is open. The knee's soft stop is a quadratic barrier from
0.05 rad, over 200 N m at its end and no step in its slope.

- `law.held_back` waited two of the robot's rhythm's swings, for ever with the
  robot's rhythm at rest - every phase of sitting and rising sets its gain to 0:
  lowering, the robot's centre of mass behind its heels before the seat
  bore it, `seated` was held and the rise with it (CI, 3.12: none in 5
  and 6 s). At rest no swing comes and nothing waits for one.
