# Architecture: Planners

The loop in a synthesizer's terms, a term a thing: [synth](synth.md).

The gynoid's latent loop, the one way it moves: a text compiled to rows,
the latent state stepped, the modes on its body and their springs rendered
into the whole-body loop's ask ([solvers](solvers.md)), then its drives.
Run by its law ([robotics](robotics.md)). Findings:
[orthokinematics](../findings/orthokinematics.md),
[flight](../findings/flight.md).

| Module | Is |
| --- | --- |
| ortho | orthokinematics: humanoid robot movement as text - `(stream_inject ..)`, `(defbehavior ..)`: latent targets, a stiffness and a damping a mode, rhythm, task constraints - compiled to rows of numbers (gynoid_mesomorph_medium.lisp, examples.lisp) |
| latent | humanoid robot latent state stepped with no branch: a row laid by a mask, the state a lerp, humanoid robot rhythm one angle; each mode's reference and impedance out |
| modal | humanoid robot modes on the body, no branch: the soles' places from the centre of mass as sums and differences, the standing legs' compression, the turns; what is sensed, the statics and the loop's ask laid flat |
| impedance | humanoid robot modes' springs rendered, no branch: the centre of mass's acceleration, a sole in the air, the turns, arms and knees - the whole-body loop's ask |
| poise | humanoid robot ZMP into humanoid robot spanned support polygon, humanoid robot capture point kept in it; which soles stand, with no branch |
| stride | humanoid robot stride from humanoid robot drive's phase: when a limb swings, where its sole lands (humanoid robot capture point), its clearance, its rock heel to toe, a standing sole's heel rising |
| leap | on the whole-body stack: which soles stand, told from one pass - each as it bears, an unloaded one beside one that bears or with humanoid robot at rest; one that does not is asked to its landing (reach) |
| support | sparse upper-body admittance coupling between driven joint innovations and modal references, with no branch |
| se2 | the outer twist-to-setpoint gate and the inner branchless heading, analog insole floor-plane and CPG modulation pass |
| fours | on all fours, no branch, not yet on the loop: the kernel's torus clock (limb phases pulled to their lags), a morph's lags on the torus's shortest arc, the fifth limb antiphase to their swing, the virtual truss (a wrench met, each edge squeezed as told), a limb bent to a length |
| crawl | off its feet, no branch, not yet on the loop: a floor twist's coronal index and the turnout it asks, a sled's share of its weight over its axial phase, its anchors' friction use and the share of the vault they hold |
| ballistic | walk to run, a brake, a jump and a leap, not yet on the loop: a gait's Froude number and the text's duty map, a step's brake on a floor's friction and the steps to a walk, its hips leaving the floor at its knees' back-EMF bounds with their rotors, a flight's apex and span |
| bedside | at a seat's or a bed's edge, no branch, not yet on the loop: its skin's reach behind it within a band of heights, a slide's speed under its rim's barrier, a hung limb's swing, its drift rolling on a floor and the shrimp's counter-scoot; its thighs' share crossing a seat's edge, an edge's capstan drag, a stool's centre of pressure on the floor |
| substrate | on soft ground, no branch, not yet on the loop: a contact's work loop and its regime, the damping ratio returning an energy, a mass's compression stroke, flat capsules' area, a power law's stiffness, a bed's phase and a tick of it |
| groundwork | on the floor, no branch, not yet on the loop: its centre over the floor it rests on (its skin's support along its up), where it rests round a turn, the roll's circumferential ring and its girdles' lag, a substrate's sinkage and the text's torque out of its trough |

- The texts: `gynoid_mesomorph_medium.lisp` (and `_r80`), `gait_base.lisp`,
  `postures_base.lisp`, `motion.lisp`, `situations.lisp`,
  `examples.lisp`.
- `host/tools/sim/`: ortho (humanoid robot latent loop as the page runs it,
  traced; `--walk --look` humanoid robot walk's look); hop (a hop on it:
  thrown, flown, landed).

## Suites

| Suite | Holds |
| --- | --- |
| `test_ortho.py` | a text compiled to rows, none refused; no branch in the core or the renderer; the state blended and turned; crouched, shifted, bowed, on one sole, hopped; the walk on its own payloads; the first texts as given; the gym's worker, keys |
| `test_fours.py` | the kernel's multimodal texts read whole; each symmetry locked from 64 random starts and bearing as it pairs its limbs; the trot-to-crawl morph, two limbs bearing; the fifth limb through its limbs' swing; its toes-and-fists stance levelled on its figure; the truss meeting its weight, squeezing nothing |
| `test_groundwork.py` | the kernel's groundwork texts read whole, every manifold they name a patch; the ring's quarters, the girdles' twist within its waist; its roll onto its side on a hard floor, arms at its sides, slotted, levered; the trough's law at its thresholds; the six ground origins laid on its figure, the tall kneel along its axial column |
| `test_crawl.py` | the kernel's terrestrial texts read whole, the overlay bridged; the coronal index and its knee's and elbow's stroke sideways; a sled's anchors on the patches' frictions; the four origins laid on its figure; the supine ratchet on its drives |
| `test_bedside.py` | the kernel's bedside texts read whole, every frame they touch one of its patches' but its calves'; what meets the chairs, the lodge bed and a stool behind it; a stool's rim halt, its lean, its hung shank, its perch; the bed's pendulum on its length and width; its roll's drift, the escape and its stroke |
| `test_ballistic.py` | the kernel's ballistic texts read whole, the brisk walk's leap held on its world; their regimes on its leg, its run's stance on its knee; its jumps, platform and leap from its drives; its brake's steps on a floor's friction, its energy on its DC link |
| `test_vault.py` | the kernel's overheight texts read whole, every landmark a segment of its own; its dip hung from its fists, its lift, travel and torques; the edge's unweighting and drag, the stool's cone at its hips, its reach along a seat; its way down |
| `test_substrate.py` | the kernel's deformable texts read whole; the work loop against a linear contact's energy return, the regimes as damping ratios; its patches' areas and pressures against the texts' ceilings; a power law's stiffness and the tamp's gate; a bed under it still centre, the kip on its drives |
| `test_postures.py` | the seat's chain its landing's; its legs blended over what bears it, the soles' the crouch bit for bit, a let-go hip hung; the kernel's posture atlas read; sat on the chair, its ask its legs' |
| `test_leap.py` | which soles stand, from one pass alone; a landing within reach; the loop in C against the python stack, no sole standing; a hop on both, three in a row on the loop |
