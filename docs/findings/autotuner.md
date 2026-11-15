# Findings: autotuner

The text as given (lisp/humanoid/meta_ast_autotuner.lisp), read by `machine.texts.tune`:
six classes, 33 keys (32 in the suite's names); its step's numbers its own (lambda
0.08, a literal's move 0.15 of its scale, tol 1e-3, four places).

- The robot's texts, the suite's and its gym's (20, given to its session as an
  argument, `tune.texts`): 554 literals, its gym's 199 bound to the suite's
  name voicing reads into the slot their clause lays (the clause compiled
  alone); the grammar names 173, a root walks 159, 100 move it by
  evaluation: in 32 of its gym's 34 behaviours, 5 of 8 gaits, the 3
  schedules, the 2 stunts. The rest no voice or phase of the robot's own reads,
  their columns 0. Of the suite's alone (331): 75 named, 18 live.
- The robot's walk (its gym's four streams) reaches 5, 3 live: its stand's axial
  spring 3000 and damper 600 (compliance, damping ratio), its step's lift
  1.0; its lean's 0s have no scale. On the robot's loop: the centroidal pivot
  under 15 mm on 54 % of passes, the friction cone 0.40 used, its peak
  1.80 of its weight, its master bus's tank (wbc_bus.c) bled
  161.7 J in 16 s against its 12 J: 13.47 of |r| 13.7 - the anti-spasm insert's
  tank at its floor through the robot's walk.
- The classes now read the cone's use for shear (|F_t| over mu F_n, each
  contact's own mu; slip reported) and the tank's bleed over its capacity for
  passivity; the runway's and ice's numbers below were measured on slip and
  the robot's drives' J/m. The robot's vertical mode fitted on its centre's
  acceleration read its step's rate (1.86 Hz, zeta 0.003), not the contact's:
  not kept.
- The runway reaches 17, 3 live, each by its voice's rule: the nominal's
  stride on the robot's twist, its thoracic lean -2.0 on the robot's pitch (unit
  0.5), the catwalk's attack 1e-3 on its duck; a circumduction of 0 has
  no scale. The sit reaches 3, its hand-over's and descent's ms live.
- On the robot's loop (`tools/sim/tune.py`, the suite's harness, 16 s from 4 s,
  19 s a run) its walk with the runway's row: the centroidal pivot under
  the text's 15 mm on 55 % of passes, its median 12 mm - the robot rolls over
  the robot's toes each step, no walk's invariant -; its load never
  under 0.698 of its weight (0.35), its feet never nearer than
  12.6 mm (6.5), 274 mm of slip, 350 J/m:
  |r| 4.074, its energy 3.50 of it. A run repeats to the last digit; each live
  literal a unit of the text's fourth place off moves |r| 0.002-0.009: a
  step's floor.
- Each step that lowered |r| on the runway raised the robot's landing (1.424 ->
  1.433-1.450 of its weight): the pivot bought with its form. Held in the step
  (each form measure's measured gradient kept from growing, `qp.solve`), its
  trust region halved on a refusal, at half it kept 640 -> 688 mm stride, lean
  0.04 -> 0.0295, attack 105 -> 112.9 ms: the pivot's shortfall 1.73 -> 1.26,
  slip 274 -> 221 mm, landing 1.42 -> 1.41, 350 -> 360 J/m, |r| 4.074 ->
  3.926. From there no step keeps the robot's form (its landing's knee
  and load, its knee behind plumb) till the model's gain, 0.0079, is
  under its 0.009 floor.
- The stealth crouch on ice (22 walked, 7 live): the robot's 1 747 J/m at 0.13
  m/s is 17.47 of |r| 17.47, the text's energy class the whole norm. The
  crouch's first step kept by 0.125 over its 0.118 floor bought 1 734 J/m with
  slip 81 -> 94 mm, on ice; then none past its floor.
- Any root run (`tools/sim/telemetry.py`), the classes over whatever bears
  the robot, the impact the world's peak load, the energy its cost of transport
  (J a metre of its centre of mass's way over its model's 305.5 N): the
  runway's |r| 2.392, the same step kept (its peak 1.663 -> 1.596),
  converged. Run on every other root its literals reach, the omnidirectional
  schedule fell at 22.03 s: not written.
- The omnidirectional schedule (48 s, four morphs): 36 walked, 10 live, 5 of
  them felling the robot at 0.15 of their scale; its floor 0.209 over its
  model's best gain 0.173: nothing to resolve.
- The chair's sit: its descent's duration fells the robot at 0.15 either way;
  its hand-over's 320 ms moved 0.75-48 ms felled the robot 3 times in 7
  (6.46-9.87 s) and raised |r| the rest - its sit is chaotic in it -;
  under 0.4 ms the robot's law rounds it away, the robot's run the same bit for
  bit. The sit's |r| 8.23 is 8.23 of energy: a sit is no transport.
- At runtime (`machine.process.rcu`, `machine.process.tap`): the robot's walk
  linked passes as unlinked bit for bit, 6000 passes tapped, none dropped. The
  robot's raw pass copied in C beside its contacts' read, 26 us (numpy's gathers
  90, a per-pass sample 146); its hook 0.035 ms a pass median, p99 0.10, against
  its pass's 0.24 unlinked (interleaved on one load). Cooked off-core: force,
  slip and cone use as `mj_contactForce` and the robot's point's velocity give
  them, within 9e-16; the solver's velocity rows are its contacts' tangential
  velocity (2.7e-15 over 6384), the relative one on a moving prop.
- A table laid 0.04 s past the robot's lateral crossing (0.25 rad), both soles
  locked, its last blend done; minimum-jerk its target's rate leaving
  0.0042 of its mean and arriving 0.0003, linear 1.0002 both ways.
- The robot's walk tuned while it walks (`tools/sim/runtime.py`, the
  robot's loop a process of its own, a window 6 s from 2 s past a table's
  laying, 9-10 s on the wall): |r| 5.66 (pivot 2.29, cone 0.40, peak 0.41, tank
  5.14) to 5.06 in one step, the robot's stand's axial spring 3000 to 3450 and
  damper 600 to 510, its step's lift 1.0 to 0.85; the tank bled 54.7 J a
  window for 61.7, 379 J/m for 388. The next step not kept: the robot's hip over
  stance, then its landing load worse, then under its floor. Held
  elsewhere on 39 roots and written, the robot's page's walk (40 s) 401 J/m for
  408, off a woman's walk 2.42 for 2.51, its knee at landing 31.3 deg for 32.7,
  behind plumb 10.9 for 10.7; but the wall's scenario no longer met (it went
  -0.78 m): undone. A write is now held on the bench's scenarios and shoves too
  (`tune.benched`).
- Run again on main with the insert's comparator: the same step kept (|r|
  5.66 to 5.03), the next refused twice on the robot's unloading, then under its
  floor: converged; held on 39 roots and the bench (142 of 256 met as the
  texts are, all still met); written. The robot's page's walk 402 J/m for 408,
  off a woman's walk 2.42 for 2.51, knee behind plumb 10.1 for 10.7, unloading
  0.665 for 0.68; its chair wobbled 57/265/14 fell/ok/short for 79/237/20. But
  test_errands' viewport bed no longer rose (doing bed_rest, green 3/3 without
  it): undone. Three steps from the texts reached 4.82.
- Held strictly (no form measure growing at all) the step was 0: five form
  rows of noise-sized slopes over three literals; free it gained 0.47. A
  held measure now grows by its floor at most, as the verdict judges it, and
  a slope counts only where its pair moved it past that floor.
- A write is held on the suites its files reach (`tools/sim/gate.py`: a
  worktree at HEAD, the relay as the files are and with it, failing checks
  compared by name). On the bed-breaking step counts hid it: the bare
  worktree failed the chair's rise (seated 1.48 s, never stood), the step
  the viewport's bed instead, one each. Refused, the write halves its move.
