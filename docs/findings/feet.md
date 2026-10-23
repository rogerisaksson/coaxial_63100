# Findings: feet

Passive toes, the sole's give, and the heel's rise on the loop
([orthokinematics](orthokinematics.md)). The board's findings:
[FINDINGS](../FINDINGS.md).

## Toes sprung, no motor

`drives.passive`, `drives.TOE_K`, `drives.TOE_LOSS`.

- A stiff toe (100 to 400 N m/rad) turns with the foot at toe-off and stands
  the body on its toe tips. At 10 N m/rad (`drives.TOE_K`, `DIMENSIONS.md`),
  damped by a thin carbon-fibre sandwich with TPU or TPU printed round carbon
  rods glued with silicone (`drives.TOE_LOSS` 0.3 at 20 rad/s, 0.15 N m s/rad),
  the forefoot bends under the push-off without a motor.
- The motors off the feet: 21 driven joints of 27 (`drives.passive`), 1.1 kg of
  drives saved across the two feet.
- The sole's give (`mjcf.SOLE_S` 0.02, `SOLE_DAMP` 1.5 of critical, `mjcf.SOLE_WIDTH_M`
  5 mm) is the sneaker's foam and gum sole; the carbon flex is the toe hinge
  alone. On rigid soles (1 mm give, 1.0 of critical) a touchdown peaked at
  1.9 kN, 3.5 times her weight; over 5 mm at 1.5 of critical: 1.5 kN. Softer
  (settling over 0.035 s) the body pitched on twice as fast, the sole a lag in
  the ankle's hold; damped at 1.75 the standing foot's load flickered to 70 N as
  the other swung. At a stand's sway's extreme the loaded sole rolls 1.3 deg into
  its foam and her pelvis lists 0.45 deg with it: the unloaded sole hangs 2.0 mm
  up on 1 N, the legs identical, and `leap.TOUCH_M` 2 mm lets it go; the parry
  then steps it in place ([stunts](stunts.md)). A band of the give as the
  stance's, for every sole or for one that stood, keeps the stack laying force
  on a sole that has left: the run's leap
  0.69 to 0.30 m, the seats and the snow wobble gone. A press of a standing
  sole to the floor at the contact bleed's spring moved the let-go 30 ms; at
  the sole's settling (2500/s^2) it pressed on the corner model's 0.9 mm bias
  all the time and stood her with a knee at 16 deg. Her pelvis listed by the
  soles' hover difference over her hips kept the sole (1.9 mm) and moved the
  stride leaps, a wall and the seats.

## Heel rise and the trailing knee on the loop

- A flat foot at her walking height reaches 0.124 m along the floor
  ([kinematics](kinematics.md)); her stride rolls the standing sole about its
  ball (`stride.rocks`, `stride.heels`), its heel rising as far as its leg at
  its length needs for her hip under where it stood, at most a woman's as her
  toes leave (36.2 deg, the reference take's), at 6 rad/s. Bus and DLS on, by
  the cap: 0.6 rad on stilts (2.87 off, knee at lift 18.5); 36.2 deg 0.71 m/s,
  411 J/m, 2.58 off (without them at 0.6, 0.70, 409, 2.55); 37.6 2.47; 40 deg
  2.42, 62 (the band's) 2.36 - past 37.6 her sprint, stutter, start and stop,
  rug, hole, snow and moves fell at 5-12 s, all eight standing at 36.2.
- As the trailing sole's load leaves it, its knee folds (`stride.FOLD_RAD`
  0.26 rad with none on it), its leg that much shorter for its heel's rise
  ([orthokinematics](orthokinematics.md)). The heel's cap at 40-46 deg, the
  fold bites: 0.45 knee at lift 23.4-23.6, stiff 0.23, 2.25 off, 459-462 J/m; 0.6
  24.1-26.5, 2.20-2.21, 443-479 J/m.
- Her pre-swing against a woman's take (`tests/takes/walk.fbx`), a stride's
  share from toe-off -0.12, -0.07, -0.02, 0: the woman's trailing knee 11.5,
  23.2, 25.0, 36.5 deg and her thigh behind 12.2, 8.9, 9.8, 5.2 - it folds and
  comes forward while her toes bear -; hers 10.1, 13.6, 15.2, 21.5 and 14.7,
  15.1, 16.5, 14.8, going on back till they leave. Her leading foot lands 0.16
  of a stride before, the woman's 0.25.
- On the stacks (`drives.md`, `stacks.md`), the ankles' and hip rolls' rotors
  reflect 0.103 and 0.173 kg m^2 through their rods and spur pairs, and the
  passive toes' rows sit in her dynamics under the rest (`wbc_wrench.c`,
  `machine/wrench.py`).
