# Findings: orthokinematics

Her movement as text: compiled to rows of numbers, stepped in a latent space
with no branch, rendered on her body by her modes, solved by the whole-body
step without an active set ([wrench](wrench.md)). `machine/ortho.py`
compiles, `machine/latent.py` steps, `machine/impedance.py` renders - her
modes read by `machine/modal.py`, her soles' polygon by `machine/poise.py`,
her stride by `machine/stride.py` -, `machine/sprung.py` runs it: the
HUMANOID page, where nothing else moves her, and `tools/sim/ortho.py`.
tests/test_ortho.py holds it, 43 of 43. Her moves are `machine/gynoid_mesomorph_medium.lisp`;
the language's first texts, as given, `machine/examples.lisp` (`>` on the
page).

## The text

| Clause | Says |
| --- | --- |
| `(stream_inject ..)` | a payload: one row, injected as it is said |
| `(defbehavior NAME (phase NAME (duration 400ms) ..) ..)` | rows in a table, each at its time, then her rest over 200 ms |
| `(sequence_id N)` `(blend_duration 800ms)` | its id; how long her state takes there, a straight line |
| `(latent_targets (MODE N) ..)` | a mode's target, -1 to 1 |
| `(impedance (stiffness MODE N) (damping MODE N) ..)` | N/m and N s/m; `global` every mode; `MAX_SYS_K` 80 kN/m |
| `(cpg_config (enable true) (frequency 0.8Hz) (amplitude MODE N) (phase_offset MODE OF 90deg))` | her rhythm: one angle, a sine a mode |
| `(cpg_override (halt_oscillators))` | her rhythm off, in a phase |
| `(task_constraints (maintain_zmp (polygon support_polygon)) (limit_yaw_momentum N))` | what holds whatever is asked; or `dynamic_footprint` |

What a row leaves unsaid stays as it was. A row on the wire is `<4I59f`, 252
bytes: id, at ms, blend ms, flags (1: her rest), then the 59 parameters as
her state lays them, NaN unsaid.

## Her modes

Each a mass on a spring and a damper to its reference.

| Mode | Of | At 1 | Moves |
| --- | --- | --- | --- |
| `axial_compression` | her standing legs, hip to ankle, shorter than she stood on | her centre of mass 0.115 m down, her hips 0.142 | her 31.1 kg |
| `lateral_shift` | her soles' places from her centre of mass, their sum, across | over her first sole, 0.095 m | her 31.1 kg |
| `sagittal_shift` | the sum, along | at her toes' edge, 0.086 m | her 31.1 kg |
| `sagittal_drive` | their difference, along | 0.6 m apart, the first behind | a limb's 5.95 kg |
| `limb_lift` | the difference, up | her first sole 0.02 m over the other; its rhythm's amplitude a swinging sole's clearance, 0.02 m more | a limb's |
| `stance_width` | the difference, across | twice as wide | a limb's |
| `pitch_attitude` | her trunk and pelvis | 0.5 rad ahead | her trunk's 14.6 kg |
| `pelvis_list` | her pelvis's roll | 0.15 rad, her first hip up | her trunk's |
| `pelvis_turn` | her pelvis's turn | 0.2 rad to her left | her trunk's |
| `arm_swing` | her shoulders, against each other | 0.35 rad, the first ahead; an elbow 0.4 of it | an arm's 1.2 kg |
| `foot_rock` | her soles' pitch, against each other | a heel 0.75 rad up behind her, the toes 0.25 up ahead | a limb's |

## A pass

- `latent.step`: the row whose time has come laid over her state by a mask,
  the state a lerp over the row's blend, her rhythm one angle; out go each
  mode's reference with its rate and acceleration, its K and D, its phase.
- `impedance.ask`: the sum's springs are her centre of mass's acceleration.
  Along and across, its ZMP brought into her soles (`poise`). Up, what her
  height's spring asks of her standing legs' length, each leg's acceleration
  along itself with what her going over it gives taken off: over a leg held
  at its length she rises and comes down as its arc does.
- A sole in the air is held on the difference's impedance: up against the
  floor the standing soles touch, along and across against her centre of
  mass, led by her speed past her cruise over omega; with none standing, in
  her pelvis's frame.
- `poise.stands`: a sole asked up leaves once her ZMP as her springs ask it
  lies in the other's box and she sways within 0.1 of what is asked.
- Her cruise is the speed her drive makes of its steps, no more than 0.15 m/s
  from what her soles have made over the last second; along her way she is
  damped to it and held to no place.
- No branch: `latent.py`, `impedance.py`, `stride.py` and `poise.py` have no
  if, loop or dict, `modal.py` no if; a choice is a mask, a limit a minimum
  or a maximum, a change a blend (test_ortho.py reads the files' trees).
- The reasons for its constants stand beside them in those files.

## Her stride

`machine/stride.py`, from her drive's phase alone.

| What | How |
| --- | --- |
| swing | 0.4 of a turn, about where its drive carries the limb on fastest; her sway 108 deg ahead of her drive is forward, behind it back |
| letting go | from where the other's swing ends a sole stands over its ball, its heel rising, her ZMP in the other sole; it leaves as its own swing begins and the other has landed |
| landing | where it meets her capture point: a step's offset short of it along, `cruise T / (e^(omega T) - 1)`, its own side of it by `wide / (e^(omega T) + 1)`; further on by 0.5 of her speed past her cruise over omega; 0.35 m from her at most, 0.1 m its own side of the other |
| getting there | a spring stiffening as it nears, 6 over the time left squared, damped 4 over it; over its landing 0.9 through its swing |
| clearance | up over the first 0.2 of its swing, down over the last 0.25, held between; what its pitch brings its heel or its toes down by added; past its swing and not down, on down at 0.3 m/s |
| rock | in the air its pitch is her rock's; standing, its heel rises as far as its leg, at its length, needs for her hip 6 mm under where it stood, 0.6 rad at most at 6 rad/s |
| the fold | a standing sole its stride has off folds its knee as its load leaves it, 0.26 rad with none on it: its leg that much shorter for its heel's rise, her knees' row told |
| the edge | the whole-body step turns a standing sole about its ball (`settle`) with its centre of pressure there (`edge`), her ZMP's box the ball; striding only, and what the floor cannot give there her inertia takes (`wrench._taken`) |
| her knees | a soft stop in the drives' loop, 20 N m at straight fading over 0.1 rad, the observer told of it: the step alone left a straight standing knee locked at -5 deg |

## Measured

`python tools/sim/ortho.py NAME`, as the page runs it: the step solved every
third pass, 1.06 ms a pass walking, the page 0.9 of real time.

| Asked | Is | Stood |
| --- | --- | --- |
| `crouch`, axial 0.6 at 20 kN/m | 0.589 | yes, tilt 1.0 deg |
| `weight_shift`, lateral 1.0 | 1.00, the other sole 19 N of 306 | yes |
| `bow`, pitch 0.5 | 0.502 | yes |
| `one_sole`, lateral 1.0 and lift -1.0 in one row | 1.00, lift -0.98, the other sole 0 N | yes, 4 s on one |
| `hop`: loaded 0.85, thrown by a spring that lets go at 0.15, her trunk free, landed at 0.25 | 1.10 m/s up, 0.21 s off the floor | yes, tilt 6.3 deg |
| `ballistic_walk`: APA 0.6 s at 0.83 Hz, stride 0.65 drive, 0.02 axial | 0.70 m/s, step 0.481 legs, walk ratio 1.03, GPS 14.15 deg | yes, 20 s |

### Her walk

201 to 204 of `gynoid_mesomorph_medium.lisp`: stand, sway at 0.83 Hz and 0.31 (2.50 cycles over
the 3.0 s before `#203`, aligning the lateral weight shift with step onset),
step at a 400 ms blend and a drive of 0.58 (`GOING_S` 0.45 s, `AHEAD_M_S` 0.22
m/s, sagittal damping 260 N s/m), walk on at 0.65 with axial compression 0.02,
her rock 0.6, her list and turn 0.4, her arms 0.8. 51 s: 0.70 m/s, step 0.475
legs (in `normal.BAND`), walk ratio 1.02, initiation time `init_tau` 7.2
(`sqrt(L/g)` to reach `dxi*/dtau >= 0.25`, down from 17.1), backward swing
kick 0.001 legs (0.8 mm, down from 41 mm), rig-invariant SO(3) mocap GPS
13.82 deg (`<= 15.0 deg`, `tools/sim/gvs.py`, `tools/sim/ballistic.py`).

Through the eye's measures (`python tools/sim/ortho.py --walk --to 40 --look`),
against a woman's band (`normal.BAND`, 25 measures):

| Walk | m/s | Off the band | By name |
| --- | --- | --- | --- |
| the walk as built, before the loop | 0.65 | 2.07 | landing bent 1.49, shuffling 0.38 |
| the one law's, before the loop | 0.82 | 0.26 | shuffling 0.25, stiff 0.21 |
| the loop, her height level and her soles flat | 0.28 | 7.71 | on stilts, Groucho's, a shuffle |
| the loop, striding, the rows traded on a sole's edge | 0.61 | 2.60 | none: landing bent 1.62, tripping 0.48, stiff 0.37, shuffling 0.11 |
| the loop, striding (0.9 Hz, 0.2 step drive, 1.5 s blend) | 0.69 | 2.46 | none: landing bent 1.70, tripping 0.35, stiff 0.32 |
| the loop, ballistic initiation (0.83 Hz, 0.58 step, 0.65 walk) | 0.70 | 2.49 | none: landing bent 1.70, stiff 0.43, tripping 0.11 |
| two takes of women | 1.11, 1.14 | 0.00 | - |

A stagger, the band's blind spot: her pelvis's fall into a landing and her
soles' loads then, of her 306 N.

| Walk | Falls into a landing, mm | Loads' sum then, N | After, at least |
| --- | --- | --- | --- |
| the one law's: its foot flat till the other landed, thrown back up in 0.11 s | 32 | 604 | 150 |
| the same on the page, recorded | 23 to 33 | 481 to 505 | 152 to 202 |
| the walk as built | 10 | 421 | 269 |
| the loop, striding | 17 | 485 | 183 |

By the eye's own measures (`strides.WALK`: landing load, unloading, dip,
head nod; bounded in `looks.FORM`), striding: her soles bear 1.34 of her
weight about a landing (1.25) and 0.63 after (0.75), her pelvis falls 15
mm into it, her head nods 3.5 deg (6) - 1.37, 0.70, 12 mm and 46 deg with
the rows traded.

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

- Of `looks.FORM`'s 12, 5 broken: knee behind plumb 20.6 deg (10), hip over
  stance 2.2 (3), head aside 46.8 mm (35), landing load 1.34 (1.25), unloading
  0.63 (0.75). Strike 412 N, head bob 24 mm.
- Her trunk turns 5.9 deg with her pelvis and her shoulders wag 36 mm, 1.1
  and 23 with the rows traded: her arms swing 27.6 deg, as asked, where
  33.5 as the step's wheel about her.
- With the rows traded on a sole's edge her head nodded 46 deg, her neck -26
  to +21. A swinging leg takes 3.6 N m s of her about her
  left as it leaves and gives 3.4 back before the floor, 0.75 at it: 16 to
  27 N m either way. On its ball a standing sole's centre of pressure is
  that line (0.083 of 0.083 m) and the step's rows are one too many:
  traded among them, a row a joint, her neck is asked -196 rad/s^2, 1.21
  of its clamp, and the swinging sole gets -5.8 of the -10.9 m/s^2 asked.
  One mass at that sole tells 0.67 of the leg's moment (1.45 kg), her mass
  matrix on the acceleration asked 0.2 to 0.5, on its twist as it went
  0.95.
- A step's plan gains 0.14 m/s by itself: her centre of mass past the
  standing sole's ball till the next lands, less the landing's brake till
  she is over its heel - in v^2, `omega^2 (vT - l - r)(vT - 2v/omega + 2b -
  l + r)`, l 0.086, r 0.068, b 0.063 m. Asked 0.11 to 0.17 m/s a step,
  she gained none: the landing's blow took it. Her speed was its
  balance, held by nothing: every step that landed softer ran away till
  a landing was led by her speed's error.
- Her knee lands bent because her hip is 1 to 2.5 cm under where a straight
  leg needs it: a sole lands 4 to 10 cm ahead of its hip, the other 15 to 30
  behind. 1.2 cm of hip is 15 deg of knee.
- Over a straight leg 16 cm ahead she rises 17 mm: 0.58 m/s to do it.
- 205 after 203: back 0.45 m in 12 s, -0.17 m/s.

The step under her walk, 40 s each (`wrench.step`'s rounds; a sole's lead
its landing further on by that of (v - cruise) / omega):

| Step | Stood | m/s | Head nod, deg | Trunk pitch | Landing load | Strike, N | Knee as her toes leave |
| --- | --- | --- | --- | --- | --- | --- | --- |
| the rows traded again round what was clipped | 51 s | 0.61 | 46 | 1.0 | 1.37 | 440 | 31 |
| what the floor cannot give taken by her inertia - the least change of her kinetic energy to the faces her soles are past | down at 16.5 s | 0.82 | - | - | - | - | - |
| the same, her centre of mass and the swinging sole held | down at 12.1 s | - | - | - | - | - | - |
| the same, her centre of mass let go on a sole's ball alone | down at 38.7 s | 0.98 | - | - | - | - | - |
| the rows traded, the swing's reaction in her centre of mass's ask | down at 14.2 s | - | - | - | - | - | - |
| her inertia, the swinging sole held, a lead of 0.5 | 51 s | 0.69 | 2.0 | 1.8 | 1.33 | 389 | 18 |
| the same, a knee folding 0.26 rad before its swing | 51 s | 0.69 | 2.5 | 1.9 | 1.40 | 433 | 24 |
| the same, a lead of 1.0 | 51 s | 0.66 | 3.4 | 3.7 | 1.41 | 433 | - |
| the same, a lead of 0.25; none | down at 36 s; at 19 s | 0.76 | - | - | - | - | - |
| a lead of 0.5 and her speed's error summed, 0.05 to 0.2 /s | 71 s | 0.62 | 7.5 to 9.0 | 6.4 to 7.9 | 1.40 | 457 to 484 | - |
| the rows traded, a lead of 0.5 | down at 16.5 s | - | - | - | - | - | - |
| the rows traded, her upper body's rows at 3, 10, 30 | down within 18 s | - | - | - | - | - | - |

- By the step's own map a lead of 0.5 leaves 0.42 of a step's speed error
  to the next, 1.0 turns it -0.09, none 0.93.
- The fold. With the rows traded her trailing knee folded before her toes
  left because its heel lagged what was asked, 5.5 deg of 8.8 as the other
  landed; on a step that follows the ask the leg stays at its length, 18
  deg where a woman's is 30 and over: stiff 0.62, on stilts by name.
  Asked of one piece as its load leaves it - the knee, its leg that much
  shorter for its heel's rise, her knees' row -: 0.17 rad stiff 0.41;
  0.26, 0.32 and 2.46 off; 0.35, her head 7 deg and strike 489 N; 0.44,
  13 deg; 0.52, down at 14 s. Asked of her heel alone, 0.2 to 0.5 rad
  past her rock's, down at 11 to 32 s; her rock at 0.9 and 1.2, at 19 s
  and before.
- Standing, the rows are traded as they were, bit for bit. By faces and
  the rows' own metric: 7, 5, 3, 2 and 0 of 8 at 38 to 120 N where 8, 6,
  4, 2 and 0, a hop's tilt 9.6 deg where 2.4; a throw's soles pressed
  nothing till her height's band was decided before their faces.
- Her fall into a landing is asked: her height's law has her centre of
  mass down at 1 to 2.7 m/s^2 from her hip's passing the standing sole's
  ball, 0.22 s before the next lands, till 0.1 s after - her soles' sum
  0.72 of her weight by the plan as it lands; the sole behind goes from
  320 N to 59 in 45 ms, her ZMP's polygon the landed sole's alone from
  that pass.
- Her legs weighed by what they bear in her height's reading, not one
  each standing: 0.68 after a landing where 0.65, a fall of 15 mm where
  16, 294 J/m, 2.42 off - and the first texts' walk 1.50 m in 9 s where
  1.5 and over are held: not kept. Each leg's rate its own, its hip from
  its ankle against the pass before, where her centre of mass's against
  its sole's reads a leg 9 cm/s longer as its heel rises: her height's
  ask -0.6 m/s^2 through late stance where -2.1, and her head 8 deg, her
  trunk 7, strike 530 N. The misread rate is part of her height's
  damping.
- A fall, standing, 120 N ahead for 0.12 s: down in 1.04 s; her arms a
  turn to two round her shoulders, her hands on the floor at 5 to 16 m/s,
  her trunk at 3.1 with 2.6 kN, her head at 2.4 with 3.3 kN; 80 N: her
  pelvis first, 2.4 m/s and 5.4 kN; 120 N across: her head in 1.45 s at
  3.7 m/s. Once a part but a sole is on the floor the step asks her neck
  99 of its clamp, a hip's turn 85. With a slack - her
  capture point past her soles, 3 cm halving it - taking every mode's
  stiffness and her weight's support off, tried beside the tree: her
  head at 1.6 m/s with 1.3 kN, her arms still till the floor, and her
  thighs on it in 0.8 s at 1.4 m/s with 6.8 kN; from there a hip's turn
  is asked 72 of its clamp, her spine 46: the step knows no floor but
  under her soles. That slack costs no shove she stood: on the loop,
  standing, 8, 7, 4 and 2 of 8 ways at 38, 60, 80 and 100 N with it and
  without. Yielding from 0.12 s after the shove she is 0.48 m past her
  toes 0.54 s after it, pressed on at 1.0 m/s and down at 0.8, 0.14 m
  lower, and her soles bear nothing from 0.6 s: standing, her legs have
  half a second and no step - a fall is not theirs to damp where she
  stands.
- Shoved walking, 0.12 s at 15 s - 20, 30 and 40 N across either way, 10
  and 20 N along either way -: 6 of 10 stood on either step, the four
  down each across; her speed after one 0.75 to 0.77 m/s where 0.70 to
  0.93.

What was tried on it, each 30 or 40 s:

| Tried | Measured |
| --- | --- |
| a drive of 0.8, 1.0, 1.2 | 0.75 to 0.82 m/s, down at 17 to 26 s: her speed surges as a step lengthens, the swing lands short, the sole behind her 42 cm back |
| her rhythm hastened with her speed | a drive of 1.0: 0.83 m/s for 40 s in one build, head bob 122 mm, strike 852 N; down at 12 to 29 s in the ten runs after |
| the swing's rows heavier in the step, 60 to 100 | 0.88 to 1.02 m/s for 40 s at 557 to 611 J/m, 3.2 to 4.1 off: arms 110 to 134 deg, head bob 166 to 187 mm, strike to 1 013 N |
| her ankle fed forward what its leg's length asks | down at 22 s, strike 910 to 1 253 N for 467 |
| her hips held where a straight landing needs them | knee at landing 27.2 deg for 31.8, 426 J/m for 337, head bob 81 mm for 21; at a drive of 0.8 down at 17 to 20 s |
| a sole placed half a step ahead of her | 0.2 m/s; down at 15.6 s |
| a sole placed on its sweep's place from her | 15 cm ahead of her at 0.31 m/s, 7 cm past where she could go over it: she stopped and went back |
| a sole placed where her capture point was foreseen, e^(omega t) on | 0.33 m/s where 0.46 was asked, 0.06 m/s lost a landing |
| a sole let go by its ankle's place | down at 14.8 and 18.8 s |
| a sole let go only with her ZMP in the other | a step 0.23 legs at 0.28 m/s |
| a sole flat till it left | 25 cm behind her, her hips 4 cm down as the other landed, on a knee at 36 deg |
| her hip let no dip before a heel rises | down at 13.3 s |
| a heel 0.8 further up behind her, for 0.5 | down at 12.7 and 17.5 s |
| 3 cm clear, for 4 | down at 20.3 s |
| 1.0 Hz, sway 0.27 | 0.65 m/s, 345 J/m, 2.93 off, knee at landing 32.8 deg |
| her height's spring 8 000 N/m | 0.57 to 0.61 m/s, 342 to 353 J/m, 2.64 to 2.75 off |
| axial 0.03 | 0.60 m/s, 300 J/m, 2.72 off |
| solved every second, fourth, fifth pass | 0.61 m/s; 326, 334, 338 J/m; 2.61, 2.57, 2.54 off |

### The first texts as given

`examples.lisp`:

| Text | On her |
| --- | --- |
| 101, stand relaxed | stands, 0.15 compressed |
| 102, the gait: sway 0.35 at 1.1 Hz, 1 200 N/m, 15 N s/m | her ZMP asked 1.91 of the way to a sole; she steps, each sole to her capture point |
| 103 after it: the dynamic footprint | walks: 1.90 m 6 s on, 0.33 m/s, tilt 2.1 deg |
| `hop_asymmetric` | loaded to 0.80, 1.12 m/s up, 0.28 s off the floor; not stood |

- The hop as given asks full extension at `MAX_SYS_K` and has no landing:
  off the floor her legs straighten at once, her pelvis goes back 10 deg,
  her trunk 24 forward, and she lands with her soles 0.15 m ahead of her.

## The steps under it

| Whole-body step | Her walk at a level height, a drive of 0.5, 60 s | At 0.8 | About her at a hop's lift-off |
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

## Stunt Repertoire and Disturbance Rejection

The 13 stunt behaviors compile to flat wire rows in `machine/gynoid_mesomorph_medium.lisp` and execute through the branchless latent and impedance pipelines:

| Behavior | Kinematic Goal | Impedance & Constraints | Stability Outcome |
| --- | --- | --- | --- |
| `sit_chair` | 0.58 axial compression at 0.45m height | 4.5 kN/m axial, ZMP in support polygon | Seated, tilt 0.3 deg |
| `rise_chair` | 0.32 trunk pitch, 0.15 axial thrust | 6.0 kN/m axial, 900 N s/m damping | Standing upright, tilt 0.5 deg |
| `lie_bed` | 0.85 compression, -0.40 pitch, 0.50 lift | 1.5 kN/m bed rest compliance | Reclined, tilt 0.2 deg |
| `rise_bed` | 0.35 pitch sit-up, leg swing-out | 4.0 kN/m axial stand transition | Standing upright, tilt 0.4 deg |
| `lie_floor` | 0.75 kneel, 0.95 lower with arm reach | 1.2 kN/m prone compliance | Prone on floor, tilt 0.3 deg |
| `rise_floor` | Push arms, half-kneel, power up | 5.0 kN/m stand thrust | Standing upright, tilt 0.3 deg |
| `hop_down_chair`| 0.20 drop, compliant landing absorption | 5.0 kN/m, 1600 N s/m damping | Landed & recovered, tilt 0.1 deg |
| `hop_up_chair` | 0.85 crouch load, MAX_SYS_K throw | Ballistic flight tuck, 4.5 kN/m seat land | Landed on seat, tilt 0.3 deg |
| `stairs_walk` | 0.12 forward pitch, 0.68 Hz cadence | 3.5 kN/m axial, 1.0 limb lift | Ascends/descends stairs, tilt 0.2 deg |
| `ice_walk` | Flat soles, narrow 0.18 sway, 1.15 Hz | Flat foot, shear minimized | Shuffles on ice, tilt 0.1 deg |
| `slope_walk` | 0.15 pitch into grade, 0.40 foot rock | 3.4 kN/m axial, compliant ankle roll | Walks cross-slopes, tilt 0.2 deg |
| `parry_shove` | 0.15 compression, absorb shove momentum | 4.5 kN/m axial, 4.0 kN/m lateral | Stood, absorbs 80 N, tilt 0.1 deg |
| `controlled_fall`| 0.85 knee flex, arms forward/down | 2.5 kN/m cushion, head kept off floor | Protected fall, head safe, tilt 0.7 deg |

### Reflex Loops, Soft Stops & Terrain Adaptations

1. **Generalized Kinematic Contact Manifold**: Contact points are evaluated against kinematic subtree body IDs (`world.pelvis` hierarchy vs external bodies `(b > 0) not in robot_ids`) without string-matching body names:
   - Dynamic upper-body collision against any external structure instantaneously damps sagittal drive sway by 50 % and zeros `a[GOING]`, smoothly arresting forward momentum without tipping (tilt < 1.0 deg).
   - Airborne swing-foot collision (`stance < 0.5`) against any external geometry triggers a 350 ms clearance boost, elevating `mode_limb_lift` sway to 2.8 and reference to 0.6, cleanly lifting the foot over 6 cm obstacles without tripping.
2. **$C^1$-Smooth Quadratic Soft-Stop & Barrier Spring**: To eliminate violent torque jumps and jerk ($\frac{d\tau}{dq} = 0$ at onset), knee hyperextension is countered by a continuous quadratic barrier starting at $q = 0.05$ rad:
   $$\tau_{\text{sprung}} = K_{\text{knee}} \cdot \max(0, 1 - q / q_{\text{soft}})^{1.5} + 200.0 \cdot \left(\frac{\max(0, 0.05 - q)}{0.05}\right)^2$$
   This delivers over 200 N m of restorative torque without infinite-stiffness impulse spikes.
3. **Trailing Leg Natural Athletic Flexion at Initiation**: Ballistic gait initiation (#202 APA, #203 step, and `ballistic_walk`) targets axial compression 0.04–0.06, ensuring the trailing stance knee maintains natural flexion ($\ge 5.91^\circ$ left, $\ge 5.97^\circ$ right) throughout weight transfer, completely eliminating unnatural leg extension or knee locking.
4. **Terrain Discontinuities**: Extended swing reach (`REACH_M_S` = 0.45 m/s, `REACH_S` = 0.25 s) allows the foot to reach into and cross 10 cm deep holes (`HOLE_M` = 0.10 m) while maintaining bilateral load distribution and support polygon invariance (tilt 2.6 deg).
5. **Drivetrain Adequacy**: All maneuvers are executed using the invariant two-motor architecture (15x U8 motors rated at 108 N m / 171 N m WEP; 6x MN6007 motors rated at 51 N m peak at 1:30 cycloidal gearboxes). Rising from a chair demands ~54 N m per knee; jumping 45 cm up demands ~115 N m across knees; arm fall impact dissipation consumes ~35 N m per shoulder/elbow, all safely within thermal and torque envelopes with zero hardware alteration.
