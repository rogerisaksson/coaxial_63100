# Findings: orthokinematics

Her movement as text: compiled to rows of numbers, stepped in a latent space
with no branch, rendered on her body by her modes, solved by the whole-body
loop ([wbc](wbc.md)). `machine/ortho.py` compiles, `machine/latent.py` steps,
`machine/modal.py` renders, `machine/sprung.py` runs it: the page's gym (`<`
on HUMANOID, or `--ortho`) and `tools/sim/ortho.py`. tests/test_ortho.py
holds it, 32 of 32. Her moves are `machine/moves.lisp`; the language's first
texts, as given, `machine/examples.lisp` (`>` on the page).

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

What a row leaves unsaid stays as it was. A row on the wire is `<4I39f`, 172
bytes: id, at ms, blend ms, flags (1: her rest), then the 39 parameters as
her state lays them, NaN unsaid.

## Her modes

Her soles' places from her centre of mass, their sum and their difference;
each a mass on a spring and a damper to its reference.

| Mode | Of | At 1 | Moves |
| --- | --- | --- | --- |
| `axial_compression` | the sum, up | 0.115 m down | her 31.1 kg |
| `lateral_shift` | the sum, across | over her first sole, 0.095 m | her 31.1 kg |
| `sagittal_shift` | the sum, along | at her toes' edge, 0.086 m | her 31.1 kg |
| `sagittal_drive` | the difference, along | 0.6 m apart, the first behind | a limb's 5.95 kg |
| `limb_lift` | the difference, up | her first sole 0.02 m over the other | a limb's |
| `stance_width` | the difference, across | twice as wide | a limb's |
| `pitch_attitude` | her trunk and pelvis | 0.5 rad ahead | her trunk's 14.6 kg |

## A pass

- `latent.step`: the row whose time has come laid over her state by a mask,
  the state a lerp over the row's blend, her rhythm one angle; out go each
  mode's reference with its rate and acceleration, its K and D, its phase.
- `modal.ask`: the sum's springs are her centre of mass's acceleration, its
  ZMP clipped into her soles' polygon (her capture point kept in it while
  both soles stand). A sole in the air is held on the difference's
  impedance: across and up against the soles that stand, along against her
  centre of mass, led by her speed past her cruise over omega; with none
  standing, in her pelvis's frame.
- A sole is asked up while her ZMP, as her sway asks it, lies 1 cm inside
  the other's box: the swing's window follows the sway's amplitude and
  frequency, and a quarter turn between sway and drive is forward, three
  quarters back. It leaves the floor once her ZMP as her springs ask it is
  there and she sways within 0.1 of what is asked.
- Her cruise is the speed her drive makes of its steps, no more than 0.15 m/s
  from what her soles have made over the last second.
- No branch: `latent.py` has no if, loop or dict, `modal.py` no if; a choice
  is a mask, a limit a minimum or a maximum, a change a blend
  (test_ortho.py reads both files' trees).
- The reasons for its constants stand beside them in `machine/modal.py`.

## Measured

`python tools/sim/ortho.py NAME`, the loop solved every fourth pass as the
page runs it.

| Asked | Is | Stood |
| --- | --- | --- |
| `crouch`, axial 0.6 at 20 kN/m | 0.605 | yes, tilt 1.0 deg |
| `weight_shift`, lateral 1.0 | 0.994, the other sole 34 N of 305 | yes |
| `bow`, pitch 0.5 | 0.498 | yes |
| `one_sole`, lateral 1.0 and lift -1.0 in one row | 1.00, lift -0.97, the other sole 0 N | yes, 4 s on one |

Her walk, 201 to 204 of `moves.lisp` (stand, sway at 0.8 Hz and 0.37, step
at a drive of 0.2, walk on at 0.5), 60 s:

| Run | Went | m/s at the end | Landings | Tilt, deg |
| --- | --- | --- | --- | --- |
| as written | 8.43 m | 0.162 | 88, one a turn | 1.1 |
| drive 0.35 | 6.79 | 0.130 | 88 | 0.9 |
| her support polygon kept | 6.71 | 0.127 | 88 | 1.0 |
| axial 0.3 | 8.35 | 0.160 | 88 | 1.1 |
| lateral 1 500 N/m | 8.38 | 0.161 | 88 | 1.1 |
| 0.75 Hz, sway 0.39 | 7.78 | 0.150 | 83 | 1.1 |
| 0.85 Hz, sway 0.34 | 8.76 | 0.168 | 93 | 1.1 |
| shoved 20 N across at 12 s, either way | 7.72, 8.43 | 0.148, 0.162 | 88 | 1.2 |
| shoved 30 N across | 8.43 one way; the other she stops stepping and stands | - | 16 | 1.8 |
| drive 0.6 | down at 14.6 s | - | 17 | - |
| 205 after 203: the drive a quarter turn the other way | back 0.63 m in 13 s | -0.071 | 28 | 1.3 |

- One sole stands 45 % of the time; a swing is 0.34 s.
- The sway's amplitude is bounded by her soles: at 0.8 Hz a sway of 0.37 is
  her ZMP 1.25 of the way to a sole, whose box ends at 1.28.

The first texts as given (`examples.lisp`):

| Text | On her |
| --- | --- |
| 101, stand relaxed | stands, 0.15 compressed |
| 102, the gait: sway 0.35 at 1.1 Hz, 1 200 N/m, 15 N s/m | her ZMP asked 1.91 of the way to a sole: she sways 0.1 to 0.25, steps 16 times in 17 s |
| 103 after it: the dynamic footprint | down at 20.4 s |
| `hop_asymmetric` | loaded to 1.26, 1.4 m/s up, 0.27 s off the floor; not stood |

- The hop: her throw is asked no harder than 14 m/s^2; through it her soles
  press at her heels' edge and she leaves with 4.1 kg m^2/s about her
  ([flight](flight.md): 2.7 to 3.1 with a spring that lets go). In the air
  her trunk takes it.

## The loop under it

| Drives' own loop | Sway asked 0.274 at 0.9 Hz, both soles down |
| --- | --- |
| PD round the reference integrated from the loop's accelerations | 0.359, 250 ms behind |
| each drive's disturbance observed at 100 rad/s and taken off | 0.271, 5 ms behind |
| the torque as asked | 0.255, 10 ms behind |

- Under the PD 40 to 60 % of a small acceleration asked was done; the gym
  runs on the observers.
- A pass costs 3.2 ms solved every pass - the stack 2.1, `modal.ask` 0.32,
  `wbc.sense` 0.16, `latent.step` 0.07 -, a third of real time; solved every
  fourth, 1.0 ms, her crouch and her weight's shift the same to the third
  digit.
