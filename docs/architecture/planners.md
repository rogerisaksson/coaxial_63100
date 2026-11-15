# Architecture: Planners

The loop in a synthesizer's terms, a term a thing: [synth](synth.md).

The gynoid's latent loop, the one way it moves: a text compiled to rows,
the latent state stepped, the modes on its body and their springs rendered
into the whole-body loop's ask ([solvers](solvers.md)), then its drives.
Run by the robot's law ([robotics](robotics.md)). Findings:
[orthokinematics](../findings/orthokinematics.md),
[leap](../findings/leap.md).

| Module | Is |
| --- | --- |
| texts/ortho | orthokinematics: the robot's movement as text - `(stream_inject ..)`, `(defbehavior ..)`: latent targets, a stiffness and a damping a mode, rhythm, task constraints - compiled to rows of numbers (gynoid_mesomorph_medium.lisp, examples.lisp) |
| law/latent | the robot's latent state stepped with no branch: a row laid by a mask, the state a lerp, its rhythm one angle; each mode's reference and impedance out |
| law/modal | the robot's modes on the body, no branch: the soles' places from the centre of mass as sums and differences, the standing legs' compression, the turns; what is sensed and the loop's ask laid flat |
| law/statics | the statics of what is sensed standing: the frame, the mass each mode moves, the rest vector |
| law/impedance | the robot's modes' springs rendered, no branch: the centre of mass's acceleration, a sole in the air, the turns, arms and knees - the whole-body loop's ask |
| law/composer | the law's pass composed of its parts at work: each its run or its rest by its activity, no switch |
| law/poise | the robot's ZMP into its spanned support polygon, its capture point kept in it; which soles stand, with no branch |
| law/stride | the robot's stride from its drive's phase: when a limb swings, where its sole lands (its capture point), its clearance |
| law/rocker | how a sole rolls, no branch: its rock's pitch, a knee's fold before its swing, a standing heel's rise |
| law/treads | a floor told the robot: where each swing lands on its treads, the height under a place and between two |
| law/vault | the robot's hip over a standing leg: the arc about its ankle, or held where its heel rises for |
| law/surety | the robot's surety field, how sure it is of its stance: wbc_surety.c's python on the law's arrays |
| law/support | sparse upper-body admittance coupling between driven joint innovations and modal references, with no branch |
| law/upper | the robot's upper body's yield and the loop's ask in C: `support.couple` and `impedance.ask`, bit for bit |
| law/brace | the robot's brace on the solve: the tuck it contracts into on a topple, its cervical joints held |
| law/melt | the robot's legs' springs melted by the share of its weight told off soles told nothing or but to rest |
| law/se2 | the outer twist-to-setpoint gate and the inner branchless heading the twist turns, the analog insole floor plane |
| law/rhythm | the rhythm under a twist told: the gait's drive, turn and arms, and the rhythm's rate |
| moves/leap | on the whole-body stack: which soles stand, told from one pass - each as it bears, an unloaded one beside one that bears or with the robot at rest; one that does not is asked to its landing (reach) |
| moves/emancipation | the robot's legs released on purpose from what bears it: the gate, the cascade, the leg held till free |
| moves/cascade | the robot's legs' release in arrays alone: `emancipation`'s arithmetic, each compiled by `motion.laid` |
| moves/anatomy | the robot's moves' anatomy: each joint driven as its phase says, its trunk held by its hips |
| moves/sensed | the robot's moves whose phases end by what it senses, each a table of the suite's lisp (`defmove`) |
| moves/fours | on all fours, no branch, not yet on the loop: the kernel's torus clock (limb phases pulled to their lags), a morph's lags on the torus's shortest arc, the fifth limb antiphase to their swing, the virtual truss (a wrench met, each edge squeezed as told), a limb bent to a length |
| moves/crawl | off the robot's feet, no branch, not yet on the loop: a floor twist's coronal index and the turnout it asks, a sled's share of the robot's weight over its axial phase, its anchors' friction use and the share of the vault they hold |
| moves/scoot | the robot's scoot on the floor as its twist tells it: a crab on its seat, heels in turn, then a stroke |
| moves/clocked | the kernel's scoot segment on the robot's loop: its descent to the floor and its seat, legs out ahead |
| moves/ballistic | walk to run, a brake, a jump and a leap: the jump's crouch and the leap in stride (`bound`) on the loop through `leap`, the rest not yet: a gait's Froude number and the text's duty map, a step's brake on a floor's friction and the steps to a walk, the robot's hips leaving the floor at its knees' back-EMF bounds with their rotors, a flight's apex and span |
| moves/bedside | at a seat's or a bed's edge, no branch, not yet on the loop: the robot's skin's reach behind it within a band of heights, a slide's speed under its rim's barrier, a hung limb's swing, the robot's drift rolling on a floor and the shrimp's counter-scoot; its thighs' share crossing a seat's edge, an edge's capstan drag, a stool's centre of pressure on the floor |
| moves/bedexit | the robot's way out of a bed over its end: sat up on its arms, scooted to the edge, seated there |
| world/substrate | on soft ground, no branch, not yet on the loop: a contact's work loop and its regime, the damping ratio returning an energy, a mass's compression stroke, flat capsules' area, a power law's stiffness, a bed's phase and a tick of it |
| world/groundwork | on the floor, no branch, not yet on the loop: the robot's centre over the floor it rests on (its skin's support along its up), where the robot rests round a turn, the roll's circumferential ring and its girdles' lag, a substrate's sinkage and the text's torque out of its trough |

- The texts: `gynoid_mesomorph_medium.lisp` (and the builds' `_packs`,
  `_performance`, `_r80`, `_s95`, `_s105`), `gait_base.lisp`,
  `postures_base.lisp`, `motion.lisp`, `situations.lisp`,
  `examples.lisp`.
- `host/tools/sim/`: ortho (the robot's latent loop as the page runs it,
  traced; `--walk --look` its walk's look); hop (a hop on it: thrown, flown,
  landed).

## Suites

| Suite | Holds |
| --- | --- |
| `test_ortho_texts.py` | a text compiled to rows, what is no text refused, the law's code branchless |
| `test_ortho.py` | the state blended and turned; crouched, shifted, bowed, on one sole, hopped; the walk on its own payloads; the first texts as given; the gym's worker, keys |
| `test_fours.py` | the kernel's multimodal texts read whole; each symmetry locked from 64 random starts and bearing as it pairs the robot's limbs; the trot-to-crawl morph, two limbs bearing; the fifth limb through its limbs' swing; its toes-and-fists stance levelled on its figure; the truss meeting its weight, squeezing nothing |
| `test_groundwork.py` | the kernel's groundwork texts read whole, every manifold they name a patch; the ring's quarters, the girdles' twist within the robot's waist; its roll onto its side on a hard floor, arms at its sides, slotted, levered; the trough's law at its thresholds; the six ground origins laid on the robot's figure, the tall kneel along its axial column |
| `test_crawl.py` | the kernel's terrestrial texts read whole, the overlay bridged; the coronal index and the robot's knee's and elbow's stroke sideways; a sled's anchors on the patches' frictions; the four origins laid on its figure; the supine ratchet on its drives |
| `test_bedside.py` | the kernel's bedside texts read whole, every frame they touch one of the robot's patches' but its calves'; what meets the chairs, the lodge bed and a stool behind it; a stool's rim halt, its lean, its hung shank, its perch; the bed's pendulum on its length and width; its roll's drift, the escape and its stroke |
| `test_ballistic.py` | the kernel's ballistic texts read whole, the brisk walk's leap held on the robot's world; their regimes on its leg, its run's stance on its knee; its jumps, platform and leap from its drives; its brake's steps on a floor's friction, its energy on the robot's DC link |
| `test_vault.py` | the kernel's overheight texts read whole, every landmark a segment of the robot's own; its dip hung from its fists, its lift, travel and torques; the edge's unweighting and drag, the stool's cone at the robot's hips, its reach along a seat; its way down |
| `test_substrate.py` | the kernel's deformable texts read whole; the work loop against a linear contact's energy return, the regimes as damping ratios; the robot's patches' areas and pressures against the texts' ceilings; a power law's stiffness and the tamp's gate; a bed under it still centre, the kip on its drives |
| `test_postures.py` | the seat's chain the robot's landing's; its legs blended over what bears it, the soles' the crouch bit for bit, a let-go hip hung; the kernel's posture atlas read; sat on the chair, its ask its legs' |
| `test_leap.py` | which soles stand, from one pass alone; a landing within reach; the loop in C against the python stack, no sole standing; a hop on both, three in a row on the loop |
