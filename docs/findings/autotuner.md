# Findings: autotuner

The text as given (machine/meta_ast_autotuner.lisp), read by `machine.tune`:
six classes, 33 keys (32 in the suite's names); its step's numbers its own (lambda
0.08, a literal's move 0.15 of its scale, tol 1e-3, four places).

- Her texts, the suite's and her gym's (20, given to her session as an
  argument, `tune.texts`): 554 literals, her gym's 199 bound to the suite's
  name voicing reads into the slot their clause lays (the clause compiled
  alone); the grammar names 173, a root walks 159, 100 move her by
  evaluation: in 32 of her gym's 34 behaviours, 5 of 8 gaits, the 3
  schedules, the 2 stunts. The rest no voice or phase of hers reads, their
  columns 0. Of the suite's alone (331): 75 named, 18 live.
- Her walk (her gym's four streams) reaches 5, 3 live: her stand's axial
  spring 3000 and damper 600 (compliance, damping ratio), her step's lift
  1.0; her lean's 0s have no scale. On her loop: the centroidal pivot under
  15 mm on 54 % of passes, the friction cone 0.40 used, her peak 1.80 of her
  weight, her master bus's tank (wbc_bus.c) bled 161.7 J in 16 s against its
  12 J: 13.47 of |r| 13.7 - the anti-spasm insert's tank at its floor through
  her walk.
- The classes now read the cone's use for shear (|F_t| over mu F_n, each
  contact's own mu; slip reported) and the tank's bleed over its capacity for
  passivity; the runway's and ice's numbers below were measured on slip and
  her drives' J/m. Her vertical mode fitted on her centre's acceleration read
  her step's rate (1.86 Hz, zeta 0.003), not the contact's: not kept.
- The runway reaches 17, 3 live, each by its voice's rule: the nominal's
  stride on her twist, its thoracic lean -2.0 on her pitch (unit 0.5), the
  catwalk's attack 1e-3 on her duck; a circumduction of 0 has no scale. The
  sit reaches 3, its hand-over's and descent's ms live.
- On her loop (`tools/sim/tune.py`, the suite's harness, 16 s from 4 s, 19 s
  a run) her walk with the runway's row: the centroidal pivot under the
  text's 15 mm on 55 % of passes, its median 12 mm - she rolls over her toes
  each step, no walk's invariant -; her load never under 0.698 of her weight
  (0.35), her feet never nearer than 12.6 mm (6.5), 274 mm of slip, 350 J/m:
  |r| 4.074, its energy 3.50 of it. A run repeats to the last digit; each live
  literal a unit of the text's fourth place off moves |r| 0.002-0.009: a
  step's floor.
- Each step that lowered |r| on the runway raised her landing (1.424 ->
  1.433-1.450 of her weight): the pivot bought with her form. Held in the step
  (each form measure's measured gradient kept from growing, `qp.solve`), its
  trust region halved on a refusal, at half it kept 640 -> 688 mm stride, lean
  0.04 -> 0.0295, attack 105 -> 112.9 ms: the pivot's shortfall 1.73 -> 1.26,
  slip 274 -> 221 mm, landing 1.42 -> 1.41, 350 -> 360 J/m, |r| 4.074 ->
  3.926. From there no step keeps her form (her landing's knee and load, her
  knee behind plumb) till the model's gain, 0.0079, is under its 0.009 floor.
- The stealth crouch on ice (22 walked, 7 live): her 1 747 J/m at 0.13 m/s is
  17.47 of |r| 17.47, the text's energy class the whole norm. Its first step
  kept by 0.125 over its 0.118 floor bought 1 734 J/m with slip 81 -> 94 mm,
  on ice; then none past its floor.
- Any root run (`tools/sim/telemetry.py`), the classes over whatever bears
  her, the impact the world's peak load, the energy her cost of transport (J
  a metre of her centre of mass's way over her model's 305.5 N): the runway's
  |r| 2.392, the same step kept (her peak 1.663 -> 1.596), converged. Run on
  every other root its literals reach, the omnidirectional schedule fell at
  22.03 s: not written.
- The omnidirectional schedule (48 s, four morphs): 36 walked, 10 live, 5 of
  them felling her at 0.15 of their scale; its floor 0.209 over its model's
  best gain 0.173: nothing to resolve.
- The chair's sit: its descent's duration fells her at 0.15 either way; its
  hand-over's 320 ms moved 0.75-48 ms felled her 3 times in 7 (6.46-9.87 s)
  and raised |r| the rest - her sit is chaotic in it -; under 0.4 ms her law
  rounds it away, her run the same bit for bit. Its |r| 8.23 is 8.23 of
  energy: a sit is no transport.
- At runtime (`machine.rcu`, `machine.tap`): her walk linked passes as
  unlinked bit for bit, 6000 passes tapped, none dropped. Her raw pass
  copied in C beside her contacts' read, 26 us (numpy's gathers 90, a
  per-pass sample 146); her hook 0.035 ms a pass median, p99 0.10, against
  her pass's 0.24 unlinked (interleaved on one load). Cooked off-core: force,
  slip and cone use as `mj_contactForce` and her point's velocity give them,
  within 9e-16; the solver's velocity rows are her contacts' tangential
  velocity (2.7e-15 over 6384), the relative one on a moving prop.
- A table laid 0.04 s past her lateral crossing (0.25 rad), both soles
  locked, her last blend done; minimum-jerk its target's rate leaving 0.0042
  of its mean and arriving 0.0003, linear 1.0002 both ways.
- Her walk tuned while she walks (`tools/sim/runtime.py`, her loop a process
  of its own, a window 6 s from 2 s past a table's laying, 9-10 s on the
  wall): |r| 5.66 (pivot 2.29, cone 0.40, peak 0.41, tank 5.14) to 5.06 in
  one step, her stand's axial spring 3000 to 3450 and damper 600 to 510, her
  step's lift 1.0 to 0.85; the tank bled 54.7 J a window for 61.7, 379 J/m
  for 388. The next step not kept: her hip over stance, then her landing
  load worse, then under its floor. Held elsewhere on 39 roots and written,
  her page's walk (40 s) 401 J/m for 408, off a woman's walk 2.42 for 2.51,
  her knee at landing 31.3 deg for 32.7, behind plumb 10.9 for 10.7; but the
  wall's scenario no longer met (she went -0.78 m): undone. A write is now
  held on the bench's scenarios and shoves too (`tune.benched`).
- Held strictly (no form measure growing at all) the step was 0: five form
  rows of noise-sized slopes over three literals; free it gained 0.47. A
  held measure now grows by its floor at most, as the verdict judges it, and
  a slope counts only where its pair moved it past that floor.
