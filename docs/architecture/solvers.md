# Architecture: Solvers

The whole-body solvers, in Python and in C row for row: a dense QP and a
stack of them, and the step without an active set the gynoid's loop runs.
The chain they solve over: [kinematics](kinematics.md); what asks them:
[planners](planners.md). Findings: [wbc](../findings/wbc.md),
[wrench](../findings/wrench.md).

| Where | Is |
| --- | --- |
| `machine/qp` | dense convex QP: dual active set, an interior-point fallback, a strict hierarchy by null space |
| `machine/wbc` | the whole-body stack: accelerations and corner forces a step, torques after |
| `machine/wrench` | the stack's step without an active set: each sole's wrench or acceleration and humanoid robot upper body's accelerations by one least squares, four times, clipped |
| `machine/chain` | her step in closed chains, picked by her release alone - a leg let go or no sole standing, none asked to land: one QP in her driven accelerations and the contacts' forces, every other quantity affine in them through her dynamics (`chain_qp`); level 1 answers her turns (her pelvis's about its own up and way by the turn taking its left to the asked left, its pitch her posture's), a swing and the twist told her with the held contacts' stick rows and the forces least, under the laws - a contact presses alone and never sinks into its support faster than its give says, friction within an eight-sided pyramid, her torques within their clamps -; level 2 her posture in level 1's damped null space, projected off her held hands; her sled told a twist at its friction's edge against the way; her torques those forces' (lisp/humanoid/kernel/closed_chain_qp.lisp) |
| `wbc/` | the loop over the chain (wbc_stack.c the levels and bounds, wbc_solve.c the dual active set), asks of R^k into torques by priority within the bounds; the step without an active set (wbc_wrench.c, machine.stack.wrench's rows; a sole's cone and humanoid robot stride through it, wbc_cone.c; what it hands out, wbc_shrunk.c); the closed chains' step (wbc_chain.c, machine.stack.chain's; its QP wbc_chain_qp.c and wbc_chain_rows.c on the dual active set wbc_qp.c, its rows above posture wbc_chain_tasks.c); host-tested; device 13 (`coaxial/devices/wbc`) |
| `aero/aero_share.c` | the quad's thrusts shared by an active set: [quad](quad.md) |
| `host/tools/sim/wbc` | the whole-body stack standing, shoved |
| `host/tools/cores/wbc.py` | the wbc core's ctypes harness |

## Suites

| Suite | Holds |
| --- | --- |
| `test_wbc.py` | the QP solver's KKT, two levels' priority, standing, a nudge held |
| `test_wrench.py` | the step without an active set: a step standing beside the stack's, humanoid robot stand, nudges held, a hop |
| `test_chain.py` | the closed chains' step in C beside its python bit for bit over eight contacts, its forces pressing alone within their pyramids and her torques within their clamps wherever the laws were met, the QP turning her the asked way through the contacts it holds, her seat told a twist sliding at its friction's edge and still bearing, a tick's time; the selector and the standing sole as a patch |
