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
- The sole's give (`mjcf.SOLE_S` 0.02, `SOLE_DAMP` 1.5 of critical, `figure.SOLE_GIVE_M`
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
  its length needs for her hip under where it stood, 0.6 rad at most at
  6 rad/s.
- As the trailing sole's load leaves it, its knee folds (`stride.FOLD_RAD`
  0.26 rad with none on it), its leg that much shorter for its heel's rise
  ([orthokinematics](orthokinematics.md)).
- On the stacks (`drives.md`, `stacks.md`), the ankles' and hip rolls' rotors
  reflect 0.103 and 0.173 kg m^2 through their rods and spur pairs, and the
  passive toes' rows sit in her dynamics under the rest (`wbc_wrench.c`,
  `machine/wrench.py`).
