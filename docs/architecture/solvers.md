# Architecture: Solvers

The whole-body solvers, in Python and in C row for row: a dense QP and a
stack of them, and the step without an active set the robot's loop runs.
The chain they solve over: [kinematics](kinematics.md); what asks them:
[planners](planners.md). Findings: [wbc](../findings/wbc.md),
[wrench](../findings/wrench.md).

| Where | Is |
| --- | --- |
| `machine/stack/qp` | dense convex QP: dual active set, an interior-point fallback, a strict hierarchy by null space |
| `machine/stack/wbc` | the whole-body stack: accelerations and corner forces a step, torques after |
| `machine/stack/wrench` | the stack's step without an active set: each sole's wrench or acceleration and the robot's upper body's accelerations by one least squares, four times, clipped |
| `machine/stack/pressed` | a bearing patch's rows on the step: each contact held still, its split as felt (wbc_patch.c) |
| `machine/stack/chain` | the robot's step in closed chains, picked by its release alone - a leg let go or no sole standing, none asked to land: one QP in its driven accelerations and the contacts' forces, every other quantity affine in them through its dynamics (`chain_qp`); level 1 answers its turns (its pelvis's about its own up and way by the turn taking its left to the asked left, its pitch the robot's posture's), a swing and the twist told it with the held contacts' stick rows and the forces least, under the laws - a contact presses alone and never sinks into its support faster than its give says, friction within an eight-sided pyramid, the robot's torques within their clamps -; level 2 its posture in level 1's damped null space, projected off its held hands; its sled told a twist at its friction's edge against the way; the robot's torques those forces' (lisp/humanoid/kernel/closed_chain_qp.lisp) |
| `wbc/` | the loop over the chain (wbc_stack.c the levels and bounds, wbc_solve.c the dual active set), asks of R^k into torques by priority within the bounds; the step without an active set (wbc_wrench.c, machine.stack.wrench's rows; a sole's cone and the robot's stride through it, wbc_cone.c; what it hands out, wbc_shrunk.c); the closed chains' step (wbc_chain.c, machine.stack.chain's; its QP wbc_chain_qp.c and wbc_chain_rows.c on the dual active set wbc_qp.c, its rows above posture wbc_chain_tasks.c); host-tested; device 13 (`coaxial/devices/wbc`) |
| `aero/src/aero_share.c` | the quad's thrusts shared by an active set: [quad](quad.md) |
| `se3/` | a pose fitted to weighted points, a Gauss-Newton step about their centroid: the turn a 3 by 3 and the shift a division, no place in the world in the curvature; the arena eleven planes of a stride (points, goals about an anchor, weights, then each point's miss and weight kept the second pass), two passes of sixteen lanes, the same bits on every target; the turn h (h h + floor^2)^-1 (-b) in double, so a line of points or one point is not turned whatever way it lies; Rodrigues' turn by its series, laid back on SO(3) by a step of Newton's and Schulz's; soft weights w / (1 + soft r^2); no branch (se3_sums.c, se3_step.c), host only; `host/tools/cores/se3.py` its ctypes harness and numpy twin. Under friction (`se3_grip`) the same planes are a body's contact: a goal its point's anchor in the ground, a weight the load on the point, the miss a bristle's strain - a spring of the whole stiffness inside the reach mu load / stiffness, on the cone past it, mu between rest and sliding by the point's own speed off the body's rate, a drag on that speed; `se3_anchored` takes the anchors to where the pose leaves them, g = q - share (q - g), in their planes - or the step's own first pass does, told to (`take`), a tick in one call -: the contact's memory, held inside the reach, the reach behind a sliding point, under a point with no load |
| `host/tools/sim/wbc` | the whole-body stack standing, shoved |
| `host/tools/cores/wbc.py` | the wbc core's ctypes harness |

## Suites

| Suite | Holds |
| --- | --- |
| `test_qp.py` | the C dual active set beside `machine.stack.qp.solve` on random QPs, the interior point where it gives up (wbc_qp_interior.c) |
| `test_wbc.py` | the QP solver's KKT, two levels' priority, standing, a nudge held |
| `test_wrench.py` | the step without an active set: a step standing beside the stack's, the robot's stand, nudges held, a hop |
| `test_se3_core.py` | the pose's step in C: no branch in its source or its machine code and its passes packed, each of a step's numbers its numpy twin's bits in double and float on every target this processor runs, the closed form's best pose (Kabsch's) from 0.2 rad off, no turn about a line of points or of one point, a float's grain kept 13 km from the origin about an anchor, on SO(3) and finite over 10000 steps and 3000 wild arenas, a wild goal let go |
| `test_se3_grip.py` | the pose's step under friction in C: each of a step's numbers and each anchor its numpy twin's bits in double and float on every target this processor runs, the anchors taken in the step's own pass or by a call of their own, a point inside its reach a spring whose anchor keeps its bits over 2000 returns, past the reach the pull on the cone between rest and sliding by the speed and the anchor the reach behind, no pull with no load and the anchor under its point, a sole twisted to 0.08 rad and back pulled the other way where it started and at rest off its start once let go |
| `test_chain.py` | the closed chains' step in C beside its python bit for bit over eight contacts, its forces pressing alone within their pyramids and the robot's torques within their clamps wherever the laws were met, the QP turning it the asked way through the contacts it holds, the robot's seat told a twist sliding at its friction's edge and still bearing, a tick's time; the selector and the standing sole as a patch |
