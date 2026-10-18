# Architecture: Kinematics

The gynoid's figure as numbers, laid once: segments, joints and masses; the
algebra of her chains; what a leg reaches; movements by name; the figure as
MuJoCo's XML and as the C core's static model. What moves her:
[planners](planners.md); what solves her torques: [solvers](solvers.md).
Findings: [kinematics](../findings/kinematics.md).

| Where | Is |
| --- | --- |
| `machine/figure` | the gynoid's segments, masses and legs' dimensions, humanoid robot stand and humanoid robot squat, leg IK |
| `machine/motors` | humanoid robot kinematics as dual quaternions: a pose a motor, a joint a screw, a chain their product |
| `machine/reach` | a leg's reach over its foot from static arrays (reach_rows, laid by tools/cores/reach.py): margins, the flat place landed, two feet's gap; a limb an index |
| `machine/terms` | humanoid robot movements by their orthopaedic names from static arrays (term_rows, laid by tools/cores/terms.py): a part and a movement, its joint kind, sign and stops |
| `machine/mjcf` | the figure as MuJoCo's XML: segments, a motor a joint, contacts, clothes, what hangs loose |
| `machine/skeleton` | the skeleton as MuJoCo collides it, a welded massless body a part riding its segment |
| `wbc/` | humanoid robot chain on its static model (`wbc_model.c`, written by `tools/cores/model.py`): poses, Jacobians, M, bias (`wbc_body.c`), twists and wrenches in body frames |
| `host/tools/cores/` | model: humanoid robot figure as static arrays (wbc/); reach and terms: the rows above; manifest: a robot's build and figure as its orthokinematic manifest (`machine/<robot>.manifest.lisp`) |

## Suites

| Suite | Holds |
| --- | --- |
| `test_reach.py` | humanoid robot leg's reach: its rows humanoid robot figure's, what they hold within humanoid robot stops, a place landed in MuJoCo by either limb |
| `test_terms.py` | humanoid robot movements' terms: its rows humanoid robot figure's, a term moving humanoid robot in MuJoCo as its name says by either limb, its stops, what she has not |
| `test_wbc_core.py` | humanoid robot chain in C against MuJoCo: frames, M, bias, Jacobians at random configurations; a step's time |
