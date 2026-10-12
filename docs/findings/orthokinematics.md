# Findings: orthokinematics

Her movement as text: compiled to rows of numbers, stepped in a latent space
with no branch, rendered on her body by her modes, solved by the whole-body
step without an active set ([wrench](wrench.md)). `machine/ortho.py`
compiles, `machine/latent.py` steps, `machine/impedance.py` renders - her
modes read by `machine/modal.py`, her soles' polygon by `machine/poise.py`,
her stride by `machine/stride.py` -, `machine/sprung.py` runs it: the page's
gym (`<` on HUMANOID, or `--ortho`) and `tools/sim/ortho.py`.
tests/test_ortho.py holds it, 43 of 43. Her moves are `machine/moves.lisp`;
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
| landing | where it meets her capture point: a step's offset short of it along, `cruise T / (e^(omega T) - 1)`, its own side of it by `wide / (e^(omega T) + 1)`; 0.35 m from her at most, 0.1 m its own side of the other |
| getting there | a spring stiffening as it nears, 6 over the time left squared, damped 4 over it; over its landing 0.9 through its swing |
| clearance | up over the first 0.2 of its swing, down over the last 0.25, held between; what its pitch brings its heel or its toes down by added; past its swing and not down, on down at 0.3 m/s |
| rock | in the air its pitch is her rock's; standing, its heel rises as far as its leg, at its length, needs for her hip 6 mm under where it stood, 0.6 rad at most at 6 rad/s |
| the edge | the whole-body step turns a standing sole about its ball (`settle`) with its centre of pressure there (`edge`), her ZMP's box the ball; striding only |
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

### Her walk

201 to 204 of `moves.lisp`: stand, sway at 0.9 Hz and 0.31, step at a drive
of 0.2, walk on at 0.6 with her rock 0.6, her list and turn 0.4, her arms
0.8, her pitch 0.06. 60 s: 32.30 m, 0.61 m/s, 102 landings, tilt 3.8 deg.

Through `tools/sim/look.py`'s eye (`tools/sim/ortho.py .. --look`), against a
woman's band (`normal.BAND`, 25 measures):

| Walk | m/s | Off the band | By name |
| --- | --- | --- | --- |
| the law on the page, her drives as built | 0.65 | 2.07 | landing bent 1.49, shuffling 0.38 |
| the law in the room | 0.82 | 0.26 | shuffling 0.25, stiff 0.21 |
| the loop, her height level and her soles flat | 0.28 | 7.71 | on stilts, Groucho's, a shuffle |
| the loop, striding | 0.61 | 2.59 | none: landing bent 1.61, tripping 0.47, stiff 0.37, shuffling 0.11 |
| two takes of women | 1.11, 1.14 | 0.00 | - |

Striding, 329 J/m, 20 of the 25 in:

| Measure | Is | A woman's |
| --- | --- | --- |
| knee at landing, deg | 31.4 | 12 and under |
| step, legs | 0.388 | 0.45 and over |
| walk ratio | 0.77 | 1.1 and over |
| knee swinging, deg | 45.3 | 52 and over |
| heel at lift, deg | 24.3 | 28 and over |
| stance, % | 64.3 | in |
| knee straightest, at lift, deg | 2.0, 31.5 | in |
| toes up at landing, deg | 9.2 | in |
| arm, elbow, deg | 33.7, 5.7 | in |
| trunk lean, pelvis roll, turn, deg | 3.7, 6.5, 8.3 | in |
| pelvis bob, vault, hips wag, feet apart, legs | 0.023, 0.020, 0.052, 0.136 | in |

- Of `looks.FORM`'s 10, 3 broken: knee behind plumb 13.5 deg (10), hip over
  stance 2.3 (3), head fore-aft 62 mm (45). Strike 428 N, head bob 23 mm.
- Her knee lands bent because her hip is 1 to 2.5 cm under where a straight
  leg needs it: a sole lands 4 to 10 cm ahead of its hip, the other 15 to 30
  behind. 1.2 cm of hip is 15 deg of knee.
- Over a straight leg 16 cm ahead she rises 17 mm: 0.58 m/s to do it.
- Shoved for 0.12 s at 15 s - 20, 30 and 40 N across either way, 10 and 20
  N along either way -, she stood 3 of 10: 30 and 40 N across one way, 10
  N back. In the rest she was down 3 to 10 s after.
- 205 after 203: back 0.45 m in 12 s, -0.17 m/s.

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

- Under the PD 40 to 60 % of a small acceleration asked was done; the gym
  runs on the observers.
- A pass on the stack costs 3.2 ms solved every pass - the stack 2.1, the
  renderer 0.32, `wbc.sense` 0.16, `latent.step` 0.07 -, a third of real
  time.
