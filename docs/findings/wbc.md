# Findings: balance as one convex QP

The whole-body stack (`machine/wbc.py`) on its solver (`machine/qp.py`), the
MPC a second ahead (`machine/mpc.py`), the step (`machine/balance.py`),
measured by `tools/sim/wbc.py` in a world without buses. The board's
findings: [FINDINGS](../FINDINGS.md); the law as built: [standing](standing.md).

## Solver

Goldfarb and Idnani's dual active set, one QR refactor a change: KKT 3.8e-12
on 1000 random QPs; warm 0.3 ms, cold 5 ms. Failures on the stack, each
handled in `qp.py` (2026-10-10):

| Failure | Handling |
| --- | --- |
| a level's tie-break at 1e-8 (condition 1.6e8) found infeasible, z = 0 meeting every row | tie-break at 1e-6 of the level's curvature |
| rows met to 1e-9 by the level above, past what the null space can move | clipped at zero |
| a row in the span of the held ones, 2.4e-6 over: its dual step drove the multipliers to 1e15 | met under 1e-5 |
| rows 5e-17 to 327 long failed the interior point | scaled to unit norm |
| rows a level held with a positive multiplier left as inequalities below it: no interior, both methods failed | joined to the equalities, judged by the task's own multipliers; the tie-break the levels below at 1e-3 |

## Levels

Each level's content and the failure that placed it (2026-10-10):

| Task | Placement | Failure elsewhere |
| --- | --- | --- |
| angular momentum, bled at 3 /s | form's level | at 10 /s under the CoM: both soles' CoPs pinned to their edges, flipping each 50 ms |
| swinging sole | beside the turns | above them: the pelvis pitched 4 -> 25 deg in 0.1 s to throw the foot |
| height | under the turns; the weight within 3 m/s^2 on the balance's level | on the balance's level: standing knees straightened 6 -> 1 deg |
| CoP margin | the swing's level | on the balance's level: corners frozen at their friction's edge, no sole rose |

## Standing on torque

Gearbox drag 1.2-2.2 N m (`physics.BACKDRIVE`) on pure torque: CoM creep 5
mm/s, 24 mm of it an ankle's deadband at the law's gain. The drives' PD at
half `physics.SERVO`'s kp round a reference integrated from the stack's
accelerations: 0.3 mm in 2 s (2026-10-10).

## Shoves

0.12 s on the trunk from 8 ways (2026-10-10), stood of 8; the law as built,
stepping on boards as built, of 12:

| N | stack, no step | stack, MPC's steps | law as built |
| --- | --- | --- | --- |
| 38 | 8 | 8 | 12 |
| 60 | 8 | 8 | 12 |
| 80 | 4 (the sides, the back diagonals) | 7 | 12, 1-2 steps along the way |
| 100 | 2 (the sides) | 7 | 2 |
| 120 | 0 | 3 | 0 |

Spread polar, each way at three moments 35 ms apart, 24 shoves a force: 60 N
24 of 24 with no step, 80 N 24, 100 N 20, 120 N 16 (2026-10-10).

| Variant | 60 / 80 / 100 / 120 N of 24 | Kept |
| --- | --- | --- |
| MPC soles 2 cm inside their edges (`mpc.MARGIN_M`) | 22 at 60 N, 44 steps | no: 5 mm, no step |
| end's pull to the support's middle 0.01 | 100 N 18, 120 N 9, 60 N 28 steps | no: 1 |
| ZMP 5 mm, capture 5 mm | 24 / 24 / 20 / 16, walk 0.31 m/s | yes |
| ZMP 1 cm, capture 5 mm | 24 / 23 / 21 / 21, walk 0.31 | no: lost test_wbc's 80 N from behind |
| ZMP 2 cm | 23 / 23 / 20 / 23, walk 0.31 | no |

- A sole the stack loaded that bore nothing for 50 ms swings
  (`balance.LOST_S`). Counted standing on 0 N for 0.6 s, the rear foot
  drifted 10 cm and the gynoid fell aside, the MPC seeing both feet down.
- WEP granted every drive: the thermal observer ignored, a blown MOSFET
  before a broken robot. The neck's and head's ranges the stack's own
  (`wbc.RANGES`): unbounded, the head, neck, elbows and shoulders ran at
  their stacks' 51 N m, 6.4, 3.4, 2.1 and 1.3 times their clamps, swung as
  weights. Granted the legs and trunk alone: 100 N stood 7 of 8 where 8, 120
  N 3 where 4 (2026-10-10).

## Step

2026-10-10:

| Fault | Measured | Fix |
| --- | --- | --- |
| sole unloaded 50-200 ms before rising | still bore 54 N (`mjcf.SOLE_S` keeps a still sole pressed); took 280 N back as the body leaned onto it | lifted at the call |
| straight knee lifts its sole only to second order | asked up 17 m/s^2, given 1.5 (0.09 of the row in the null space below the contacts) | the knee folded 45 deg over its line |
| swing path at 400 and 40 | lagged 11 cm in a 0.2 s step | 1600 and 80: 100 N stood 8 of 8 where 4; 900 and 60: 7 |

A step on the stack costs 1.6-2.6 ms in Python, 11 at worst while the MPC
weighs four steps.

## Walk

`tools/sim/wbc.py --walk`, 0.5 m/s at 0.5 s a step asked (2026-10-10).

- MPC: two landings ahead, each about the one before; its end the LIPM's
  periodic capture point, (w/2) tanh(wT/2) off the midline and l/(e^wT - 1)
  ahead, and the pace along the way. Planned to stop on its second step,
  the feet landed 0.26, -0.16, 0.32 m wide and the gynoid fell aside. Lifted at
  once from standing, the capture point ran 0.19 m out over the standing
  sole: 0.4 s on both soles first.
- Up through 16 steps in 8 s:

| Variant | m/s | J/m |
| --- | --- | --- |
| drives' PD at SERVO_SHARE everywhere | 0.25 | 705 |
| drives' PD at 0.1 of kp | 0.29 | 637 |
| no PD | 0.30 | 592 |
| pace weight 10, 100 or 1000 | same | same |
| double support 0.1 s | 0.30 | 1015 |
| double support 0.03 s, pace priced each interval | 0.36 | 746 |
| double support 0.03 s, pace priced at the end (kept) | 0.31 | 792 |
| 0.4 s a step | 0.26 | - |
| 0.35 s a step | 0.17 | - |

- A standing leg's PD round the integrated reference fought the stack (an
  ankle +15 N m against -11): kp at a tenth there.
- The steps are short: 0.18 m at 0.5 s where the law's 0.375. Braked ~20 N
  in the left foot's stance (the soles -9.9 and +6.8 N s along the way over
  2 s); the toes bear 16 N a sole, unmodelled; the knees on their -5 deg
  stops.

### The landing, by heel and toe-base cells

A cell under each heel and toe base, the floor's vertical load split by
lever between z = -HEEL and BALL, the toes' to the toe base; a probe on
`tools/sim/wbc.py --walk` (2026-10-10):

| t s | trailing sole, cells / plan N | landing sole, cells / plan N | landing sole m/s down |
| --- | --- | --- | --- |
| 3.316 | 274 + 23 / 299 | 0 / 0 | 0.43 |
| 3.320 | 17 + 4 / 135 | 0 + 529 / 162, 114 against the way | 0.39 |
| 3.324 | 5 + 0 / 127 | 439 + 379 / 131 | 0.27 |
| 3.332 | 6 + 0 / 121 | 332 + 232 / 78 | 0.09 |
| 3.348 | 0 / 138 | 180 + 94 / 42 | 0.04 |

- The landing sole comes down toe base first at 0.39-0.47 m/s, its swing's
  profile 0.25 -> 0; the impact lifts the body off the trailing sole within
  a millisecond.
- Both soles stay flat: the trailing heel 1.7 mm under its toe base, its toes
  at 0 deg. The roll onto the toes recorded here before was not measured.
- In single support the stack's plan flips from one millisecond to the next
  among three solutions, the turns' errors 0.6 and 0.4 deg and the angular
  momentum steady: the rows its levels hold change each step.

| Heel / toe base, N | Right ankle, N m | Rows held, L1 / L2 | L2 slack |
| --- | --- | --- | --- |
| 165 / 149 | +7.7 | 2 / 7 | 0 |
| 39 / 274 | +29.6 | 0 / 8 | 0 |
| 289 / 0 | -16.6 | 6 / 7 | 1.4e-2 |

## The chain in C

`wbc/` on the arrays `tools/cores/model.py` writes from the compiled figure
(34 links, a degree of freedom each, the pelvis floating: screws, spatial
inertias with the rotors' armatures, stops), against MuJoCo at 40 random
configurations (test_wbc_core.py, 2026-10-10):

| Quantity | Worst apart |
| --- | --- |
| a body's frame | 1.0e-15 m |
| mass matrix, the armatures on its diagonal | 4.6e-14 |
| bias C u + g | 1.7e-13 |
| a point's Jacobian, w and v in the world | 1.7e-15 |
| a step - pose, M, bias, both soles' Jacobians - on this host at -O2 | 14.5 us |

- The pelvis's twist is its own frame's (w, v); MuJoCo's free joint carries v in
  the world and w in the body, so M = T^T M_mj T and the bias takes M_mj dT u
  beside h_mj. Both presets build the chain at 0 warnings; nothing on the board
  calls it yet.

## The loop in C

`wbc_stack.c` over the chain: the torques of the drives and the held joints
the unknowns; the soles standing still and the dynamics make every
acceleration and each sole's wrench affine in them (M's Cholesky, the
contacts' Schur complement); the levels of `machine.wbc` in the null space of
those above - a small level by its Gram matrix damped at 1e-6, the last by its
normal equations; the polytope on the drives' loads, the lowest level scaled
back first, then a hard clamp. The inequalities of the python stack are not in
it: the friction pyramids, the sole's centre of pressure, the stops, the
height's band and WEP's share are tasks or clips, or absent. 2026-10-10:

| Measure | Result |
| --- | --- |
| standing, asked to stay: the loop's torques against `machine.wbc.step`'s | 0.01 N m apart, the largest 2.9 |
| a tick: pose, M, bias, centre, the contacts, 4 levels, the clip; this host at -O2 | 125 us |
| in MuJoCo on the loop, 3 s | stood, tilt 0.8 deg, drift 28 mm |
| shoved 60 N from behind, the MPC's steps | stood, tilt 1.5 deg, no step |
| shoved from 8 ways at 60 / 80 / 100 / 120 N, the MPC's steps, WEP: stood of 8 | the loop 7 / 4 / 0 / 1; the stack 8 / 7 / 7 / 3 |

### The drives' own loop under it

`machine.parts.Eso` and `CTRL_ESO`, twins (test_ctrl_core.py): Han's extended
state observer on a joint's angle about the acceleration the loop planned for
it, the drive's own residual taken off its torque, the inertia the loop's own
(`jeff`, 1 / B_jj under the body and the contacts). `tools/sim/wbc.py --inner
none|pd|eso [--eso-wo]`, standing 6 s on the loop, the centre of mass over the
last 2 s (2026-10-10):

| Inner loop | MPC's steps: creep mm/s, drift mm, tilt deg | capture point alone |
| --- | --- | --- |
| none, the torque as asked | 3.06, 33.6, 1.9 | 1.33, 16.4, 0.9 |
| PD round the integrated reference | 1.73, 36.2, 0.9 | 1.39, 24.7, 0.6 |
| ESO at 100 rad/s | 1.55, 21.5, 0.7 | 1.69, 18.6, 0.7 |
| ESO at 400 rad/s | 1.64, 20.6, 0.7 | 1.69, 18.6, 0.7 |

- Standing, the observers see 0.1 N m: the joints hardly move, the drag is
  stiction without a signal, and the MPC's own replanning wanders more than
  the friction. The measure that separates the three is a walk, where every
  joint reverses against the drag each step.
- The walk on the loop (`--walk 0.5 0.5 8 --core`) falls at its first step
  with each inner loop, tilt 25 deg at 1.7 s; the python stack walks 1.88 m,
  0.31 m/s, 792 J/m. The trace: as the swing begins the neck's torque sits at
  3.6 times its top, its WEP peak, and the swinging sole gets little - the
  clip scales a whole level back when one weak drive saturates, where the QP
  redistributes within the clamps. Next: saturation in the null space (the
  drive held at its bound, the level solved again in the rest), the centre of
  pressure, the friction cone and the stops as rows held at their bound.

### The bounds as inequalities

The loop's bounds - the loads, the vertical acceleration's band, a sole
pressing, its cone and its torsion, its centre of pressure, the stops - are
inequalities in every level, solved by a primal-dual active set (exact
equality-constrained solves by Schur complement, the set warm across ticks, at
most 20 changes a level); the four ways tried before it are in git (held rows
scaled, batch saturation, level scaling, ADMM at 30 iterations: 1.07 N m off
standing). 2026-10-10:

| Measure | Result |
| --- | --- |
| standing, asked to stay, against `machine.wbc.step` | 0.01 N m apart, no bound at its edge |
| a tick on this host at -O2 | 205 us |
| in MuJoCo, 3 s | stood, tilt 0.3 deg, drift 25 mm |
| shoved 60 N from behind | stood, tilt 5.8 deg, no step |
| the walk | 4 steps, down at 2.8 s; the python stack 16 steps |

- The python stack walked while the loop shadowed every tick on the same
  state and ask: in double support 0.8 N m apart (median; 3.3 at most), in
  single support 23 N m (median; 65 at most) with the swing beside the turns,
  29 under them, the standing sole's load 59 N apart at most. One tick dumped:
  the same net force on the sole, the centre of pressure 7 cm apart, and so the
  ankle's, knee's and hip's torques 17-28 N m apart; the stack's active rows all
  corner pyramids (one corner unloaded, three on their faces) - its corner
  forces are variables the 6D wrench has not. Next: the corners' forces as the
  loop's variables beside the torques, their pyramids its bounds.

### The corners' forces as variables

Each standing sole's four corner forces, as the python stack has them: the
wrench's least-norm split among the corners plus six internal forces, variables
beside the torques; the pyramids (four faces and pressing a corner) and the
centre of pressure's margin bounds on them; the forces' regularisation on the
twelve components. 2026-10-10:

| Measure | Result |
| --- | --- |
| standing, asked to stay, against `machine.wbc.step` | 0.00 N m apart |
| a tick on this host at -O2, 37 variables, 134 bounds | 415 us |
| shoved from 8 ways at 60 / 80 / 100 / 120 N, the MPC's steps, WEP: stood of 8 | 8 / 4 / 3 / 2; the stack 8 / 7 / 7 / 3 |
| the walk | 13 steps, 0.51 m, 0.12 m/s, down at 7.3 s; the stack 16 steps, 1.88 m, 0.31 m/s |
| shadowing the stack's walk, double support | 0.4 N m apart (median), 0.7 at most |
| the same, single support | 25 N m (median), 108 at most |

- The ask of one single-support tick cut down: as made 49 N m apart; without
  the swing's rows 4.9; without the fold 6.5; without both 4.5; the turns, the
  centre of mass, the posture, the momentum and WEP taken out in turn leave
  5-6. Both solvers meet the tasks alike there (the centre of mass's
  acceleration, the turns within 0.5 rad/s^2, 1 of 17 m/s^2 of the swing, the
  posture 141-146 rad/s^2 off): the 45 N m is in how the saturated swing's
  compromise is spread over the joints, decided by which pyramid faces are
  active - the stack's six at its centre-of-mass level (its 1e-3 tie-break
  toward the lower levels activates them, and it holds them below), the
  loop's none. Both of the stack's rules tried in the loop: the bounds held
  below the level that presses them, 35 N m apart (median); its tie-break,
  27.5, and the loop's own walk 6 steps where 13. Neither kept. Open.

### The active set given room

The active set's 20 changes a level ran out in every swing and left bounds
broken by up to 1 600 (unit rows); at 80 changes a level the bounds hold
(1e-9) and no level runs out. 2026-10-10:

| Measure | Result |
| --- | --- |
| the walk, 8 s at 0.5 m/s asked | 17 steps, 1.08 m, 0.18 m/s, 2944 J/m, stood; the stack 16 steps, 1.88 m, 0.31 m/s, 792 J/m |
| shoved from 8 ways at 60 / 80 / 100 / 120 N, stood of 8 | 7 / 5 / 2 / 0; the stack 8 / 7 / 7 / 3 |
| a tick standing | 410 us; a swing's tick 20-90 changes of the active set |

- Shadowing the stack's walk tick by tick, WEP off for both: 16 N m apart in
  single support (median; 21 with WEP: each solver's own WEP history differs),
  0.04 in double; the loop chatters less (0.75 N m a tick against the stack's
  2.9), achieves more of the swing's ask (7.8 of 34 m/s^2 against 5.6) and
  draws more torque (|tau| 40.5 against 33.3). The loop's lower speed on its
  own walk, and so most of its J/m (the housekeeping and the copper a second
  over fewer metres), is the next measure: each solver's own walk.

### The loop in the stack's own space

The loop reformulated with the python stack's variables - every acceleration
and each standing sole's corner forces in shares of her weight - the dynamics'
undriven rows, the held joints and the standing soles exact as level 0, the
levels under them in an orthonormal null-space basis cut per level, each
solved by Goldfarb and Idnani's dual active set (a bound in the span of those
held moves the duals alone), with the stack's two rules: the levels below
weighed 1e-3 into a level's flat directions, and a bound held with a dual over
1e-2 kept as an equality below (1e-7 felled her at 38 N, 1e-1 felled the
walk). 2026-10-11:

| Measure | Loop | The python stack |
| --- | --- | --- |
| the walk, 8 s at 0.5 m/s asked | 16 steps, 1.58-1.65 m, 0.26-0.27 m/s, 739-768 J/m, tilt 1.4 deg | 16 steps, 1.88 m, 0.31 m/s, 792 J/m |
| shoved from 8 ways, stood of 8: 38 / 60 / 80 / 100 / 120 N | 8 / 7 / 6 / 0 / 0 | 8 / 8 / 7 / 7 / 3 |
| WEP used over a polar's 8 shoves, 100 N | 1.1 s | 12.7 s |
| shadowing the stack's walk, single support | 8.2 N m apart (median), chatter 0.73 N m a tick, swing 4.6 of 34 m/s^2 | chatter 2.9, swing 5.6 |
| a tick on this host at -O2, 63 variables | 1470 us | - |
| standing, asked to stay | 0.00 N m apart, level 0 met to 5e-8 | - |

- What each change bought, measured on the loop's own walk: the stack's
  variables alone (the torque space left) quietened the arms' structure but
  fell at 4 steps; level 0 all but unregularised and the forces in shares of
  her weight, 7 steps at 0.39 m/s; the null basis in place of the leaky
  projector, exact levels but an explosion where a bound's reduced row vanished;
  Goldfarb-Idnani, convergence but 6 steps; the tie-break, the arms following
  the trunk (shoulders 4 N m, 1 % at the clamp, where 10-20 N m at 16-42 %)
  and the walk whole with the band back on.
- Tried and dropped, each measured: the band and the margins soft through
  slacks (the walk fell at 2.6 s); the tie-break at 1e-4 (both fell); no
  tie-break (both fell); bounds never held below (both fell); a bound freed
  barred from returning (bounds left broken).
- Open: 60 N from straight behind falls where the other seven ways stand; 100
  and 120 N fall, the loop granting WEP a tenth of what the stack does; the
  tick 1.5 ms here (the tie-break's rows and up to 160 active-set steps a
  level), its cycles on native and Renode unmeasured.

### The observer under the walking loop

The loop's walk, 8 s at 0.5 m/s asked, with each inner loop under it
(`--inner`), 2026-10-11:

| Inner loop | Walked | m/s | J/m | Tilt, deg |
| --- | --- | --- | --- | --- |
| the torque as asked | 1.87 m | 0.31 | 668 | 1.6 |
| PD round the reference integrated from the loop's accelerations | 1.58 m | 0.26 | 768 | 1.5 |
| ESO at 100 rad/s, the drive's residual taken off | 1.89 m | 0.32 | 655 | 1.2 |
| ESO at 400 rad/s | 1.91 m | 0.32 | 634 | 1.2 |

- Where the joints reverse against the drag every step the observer shows
  its worth: a tenth more speed and a sixth less energy than the PD the stack
  walks on, and under the stack's own 792 J/m. Standing it saw nothing
  (0.1 N m); the walk was its measure.
- Shoved from 8 ways with the observer under the loop, stood of 8: 38 N 8,
  60 N 8 (straight behind too, where the PD fell), 80 N 7, 100 N 4 (the PD 0);
  the stack 8 / 8 / 7 / 7.
