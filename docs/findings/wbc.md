# Findings: her balance as one convex QP

The whole-body stack (`machine/wbc.py`) on its solver (`machine/qp.py`), the
MPC a second ahead (`machine/mpc.py`), the step it calls (`machine/balance.py`),
measured by `tools/sim/wbc.py` in a world without buses. The board's own are in
[FINDINGS](../FINDINGS.md); the law as built is in [standing](standing.md).

- The solver: Goldfarb and Idnani's dual active set, refactored by one QR a
  change - KKT 3.8e-12 on 1000 random QPs, warm 0.3 ms against 5 cold. On a
  stack it broke five ways, each now in `qp.py`: a level's tie-break at 1e-8
  (condition 1.6e8) found infeasible with z = 0 meeting every row (1e-6 of its
  curvature solves it); rows met to 1e-9 by the level above, past what the null
  space could move (clipped at zero); a row in the span of the held ones 2.4e-6
  over, its dual step driving the multipliers to 1e15 (met under 1e-5); rows
  5e-17 to 327 long failing the interior point (scaled to unit norm); and rows
  a level held with a positive multiplier left inequalities below it - the
  feasible set without interior, both methods failing - joined to the
  equalities, judged by the task's own multipliers, the tie-break the levels
  below at 1e-3 (2026-10-10).
- The levels as measured, each move with its failure (2026-10-10): the angular
  momentum bled at 10 /s on the level under the centre of mass pinned both
  soles' centres of pressure to their edges, flipping each 50 ms - 3 /s, with
  her form; the swinging sole above the turns pitched the pelvis 4 -> 25 deg
  in 0.1 s to throw the foot - the turns beside it; her height held on the
  balance's level straightened her standing knees 6 -> 1 deg - her height
  under the turns, her weight within 3 m/s^2 on the balance's level; the
  centres of pressure's margin there froze corners at their friction's edge
  and no sole rose - on the swing's level.
- Standing on pure torque the gearboxes' drag (1.2-2.2 N m, `physics.BACKDRIVE`)
  left her centre of mass creeping 5 mm/s, 24 mm of it an ankle's deadband at
  the law's gain; the drives' PD at half `physics.SERVO`'s kp round a
  reference integrated from the stack's accelerations: 0.3 mm in 2 s
  (2026-10-10).
- Shoved 0.12 s on the trunk from 8 ways, no step: 38 N 8 of 8, 60 N 8, 80 N 4
  (the sides and the back diagonals), 100 N 2 (the sides), 120 N 0 - where the
  law as built, stepping on boards as built, 12 of 12, 12, 12 (1-2 steps along
  her way at 80), 2 and 0. With the MPC's steps: 8 of 8, 8, 7, 7, 3
  (2026-10-10).
- War emergency power granted every drive: the head, the neck, the elbows and
  the shoulders ran at their stacks' 51 N m, 6.4, 3.4, 2.1 and 1.3 times their
  clamps, swung as weights; granted the legs and the trunk alone 100 N stood 7
  of 8 where 8, 120 N 3 where 4 (2026-10-10).
- The step (2026-10-10): its sole unloaded 50-200 ms before it rose still bore
  54 N - the soft contact (`mjcf.SOLE_S`) keeps a sole held still pressed - and
  took 280 N back as she leaned onto it: lifted at the call. Straight, its
  knee lifts it only to second order - asked up 17 m/s^2, the stack gave 1.5
  (0.09 of the row in the null space below the contacts): the knee folded 45
  deg over its line. Its path at 400 and 40 lagged 11 cm in a 0.2 s step; at
  1600 and 80 100 N stood 8 of 8 where 4, 900 and 60 7.
- A step on the stack costs 1.6-2.6 ms in Python, 11 at worst while the MPC
  weighs four steps (2026-10-10).
