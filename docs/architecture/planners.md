# Architecture: Planners

The gynoid's latent loop, the one way she moves: a text compiled to rows,
the latent state stepped, the modes on her body and their springs rendered
into the whole-body loop's ask ([solvers](solvers.md)), then her drives.
Run by her law ([robotics](robotics.md)). Findings:
[orthokinematics](../findings/orthokinematics.md),
[flight](../findings/flight.md).

| Module | Is |
| --- | --- |
| ortho | orthokinematics: humanoid robot movement as text - `(stream_inject ..)`, `(defbehavior ..)`: latent targets, a stiffness and a damping a mode, rhythm, task constraints - compiled to rows of numbers (gynoid_mesomorph_medium.lisp, examples.lisp) |
| latent | humanoid robot latent state stepped with no branch: a row laid by a mask, the state a lerp, humanoid robot rhythm one angle; each mode's reference and impedance out |
| modal | humanoid robot modes on the body, no branch: the soles' places from the centre of mass as sums and differences, the standing legs' compression, the turns; what is sensed, the statics and the loop's ask laid flat |
| impedance | humanoid robot modes' springs rendered, no branch: the centre of mass's acceleration, a sole in the air, the turns, arms and knees - the whole-body loop's ask |
| poise | humanoid robot ZMP into humanoid robot soles' polygon, humanoid robot capture point kept in it; which soles stand |
| stride | humanoid robot stride from humanoid robot drive's phase: when a limb swings, where its sole lands (humanoid robot capture point), its clearance, its rock heel to toe, a standing sole's heel rising |
| flight | on the whole-body stack: which soles stand, told from one pass - each as it bears, an unloaded one beside one that bears or with humanoid robot at rest; one that does not is asked to its landing (reach) |
| support | the whole-body ask shaped for a hop's flight, a hit's momentum and a swing kept clear |
| se2 | the twist on the loop: the gait it walks by, the heading turned by it, the rhythm by it |

- The texts: `gynoid_mesomorph_medium.lisp` (and `_r80`), `gait_base.lisp`,
  `postures_base.lisp`, `stunts_base.lisp`, `situations.lisp`,
  `examples.lisp`.
- `host/tools/sim/`: ortho (humanoid robot latent loop as the page runs it,
  traced; `--walk --look` humanoid robot walk's look); hop (a hop on it:
  thrown, flown, landed).

## Suites

| Suite | Holds |
| --- | --- |
| `test_ortho.py` | a text compiled to rows, none refused; no branch in the core or the renderer; the state blended and turned; crouched, shifted, bowed, on one sole, hopped; the walk on its own payloads; the first texts as given; the gym's worker, keys |
| `test_leap.py` | which soles stand, from one pass alone; a landing within reach; the loop in C against the python stack, no sole standing; a hop on both, three in a row on the loop |
