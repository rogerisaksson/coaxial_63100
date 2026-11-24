# Architecture: Kinematics

The robot's figure as numbers, laid once: segments, joints and masses; the
algebra of its chains; what a leg reaches; movements by name; the figure as
MuJoCo's XML and as the C core's static model. What moves the robot:
[planners](planners.md); what solves its torques: [solvers](solvers.md).
Findings: [kinematics](../findings/kinematics.md).

| Where | Is |
| --- | --- |
| `machine/body/compile` | the static meta-compiler: the build's TOML (`humanoids/<robot>.toml`, `library/`) solved for subtree masses, levers, inertias, servos and gait, cached by content hash under `.cache/compiled/`; `peripherals` the bus, thermal SOA, cooling, harness, eyes, world and sysid |
| `machine/body/figure` | the robot's segments, masses and legs' dimensions, the robot's stand and squat, leg IK |
| `machine/body/bends` | how each driven joint goes a radian of the robot's modes: its knees' bend, its arms' swing, reach and fold |
| `machine/body/reach` | a leg's reach over its foot from static arrays (reach_rows, laid by tools/cores/reach.py): margins, the flat place landed, two feet's gap; a limb an index |
| `machine/body/terms` | the robot's movements by their orthopaedic names from static arrays (term_rows, laid by tools/cores/terms.py): a part and a movement, its joint kind, sign and stops |
| `machine/body/mjcf` | the figure as MuJoCo's XML: segments, a motor a joint, contacts, clothes, what hangs loose |
| `machine/body/skeleton` | the skeleton as MuJoCo collides it, a welded massless body a part riding its segment |
| `machine/body/footprint` | the robot's footprint by height from static arrays (footprint_rows, laid by tools/cores/footprint.py), the gap the robot leaves each thing |
| `machine/body/defrobot` | the robot's manifest's numbers: the (defrobot NAME ..) the robot's gym's text includes, `tools/cores/manifest.py`'s |
| `wbc/` | the robot's chain on its static model (`wbc_model.c`, written by `tools/cores/model.py`): poses, Jacobians, M, bias (`wbc_body.c`), twists and wrenches in body frames |
| `host/tools/cores/` | model: the robot's figure as static arrays (wbc/); reach, terms and footprint: the rows above; manifest: a robot's build and figure as its orthokinematic manifest (`lisp/humanoid/builds/<robot>.manifest.lisp`) |

## Suites

| Suite | Holds |
| --- | --- |
| `test_reach.py` | the robot's leg's reach: its rows the robot's figure's, what they hold within the robot's stops, a place landed in MuJoCo by either limb |
| `test_terms.py` | the robot's movements' terms: their rows the robot's figure's, a term moving the robot in MuJoCo as its name says by either limb, its stops, what the robot has not |
| `test_wbc_core.py` | the robot's chain in C against MuJoCo: frames, M, bias, Jacobians at random configurations; a step's time |
