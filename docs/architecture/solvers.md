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
| `wbc/` | the loop over the chain (wbc_stack.c the levels and bounds, wbc_solve.c the dual active set), asks of R^k into torques by priority within the bounds; the step without an active set (wbc_wrench.c, machine.wrench's rows; a sole's cone and humanoid robot stride through it, wbc_cone.c; what it hands out, wbc_shrunk.c); host-tested; device 13 (`coaxial/devices/wbc`) |
| `aero/aero_share.c` | the quad's thrusts shared by an active set: [quad](quad.md) |
| `host/tools/sim/wbc` | the whole-body stack standing, shoved |
| `host/tools/cores/wbc.py` | the wbc core's ctypes harness |

## Suites

| Suite | Holds |
| --- | --- |
| `test_wbc.py` | the QP solver's KKT, two levels' priority, standing, a nudge held |
| `test_wrench.py` | the step without an active set: a step standing beside the stack's, humanoid robot stand, nudges held, a hop |
