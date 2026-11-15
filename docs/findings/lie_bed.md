# The robot's lie on the bed

The robot's lie on the lodge's bed, on the closed chains' step: the step, the
release, the lift, the plan through the robot's dynamics, the take's pull,
both fists down. The lie on the floor: [lie](lie.md).

## The closed chains' step

Lying on the bed the robot spasmed at the foam's 1.9 Hz and rolled 19.2 deg on
the long side (`tools.sim.scenarios bed`: lay not met, jerk pelvis 461,
head 609, hands 1573 / 999, feet 1346 / 1814): one solver ran every
regime. The upright step (`machine.stack.wrench`) pins every bearing point
at W_STILL and plans the support's push through its balance rows; on a
yielding multi-point support both are asks it cannot meet. The told row
parameter `felt` that folded the bed's push into the robot's bias (a SUPINE list
of chain ends in acts) was a per-text special case and is gone.

The robot's second step, `machine.stack.chain` with `wbc_chain.c` bit for bit
(test_chain.py, 1e-9 over 12 states), is picked by its stance alone: no
sole standing and what is not its soles bearing it. The robot's driven joints
follow its posture in the damped null space of the contacts' Jacobian
(Levenberg-Marquardt, Cholesky); the contacts' forces come from a
box-projected Gauss-Seidel over its Delassus matrix on the dofs no drive
holds - pressing alone, within their friction's box, warm from what is
felt, a mattress's give as a pass of lag -; its torques follow, shrunk to
its clamps. The regime's constants are the kernel's
(closed_chain_complementarity.lisp, `closed-chain-regime`), the solvers
written out there on flat double-float vectors.

| scene | before | after |
| --- | --- | --- |
| bed | lay not met, rolled 19.2, jerk 461 / 609 / 1573 / 999 / 1346 / 1814 | met, rolled 0.1, 392 / 599 / 829 / 999 / 775 / 1277 |
| mattress | jerk pelvis 1068, head 1682 | 740 / 972 |
| turning_in_bed | 373 / 501 | 281 / 431 |
| crawl_backwards | fell at 8.5 s | stands, floor borne 15.3 s, 377 / 615 |
| chair, kneel, crouch, falls, walks | | bit for bit the same |

Selectors measured on the bed (rolled, jerk pelvis / head / right foot):
stance alone 0.1, 392 / 599 / 1277; two bearing bodies or more 1.1, 570 /
896 / 2034 (the regime flickered as the robot's pelvis bore alone); latched on
the prop comparator's impact blanking 0.6, 526 / 816 / 2074 (the leg swap's hop
ran on the upright step and slammed its foot). A chair-edge bounce in the broken
rise from the chair now runs three passes on the chains' step; the errands
suite's four red rows are HEAD's own.

Two gotchas: `wbc_step_t`'s new regime fields sit after its workspace -
in front of it they shifted the workspace by 40 bytes and numpy's BLAS
kernels answered 47 of 12 668 solves a few bits apart from the python
step; and a test module parameter named `step` shadowed the glue module.

## The release, the turn and the rest

A page recording of the lodge's lie
(build/recordings/humanoid_20261101_130722.csv) showed three faults after the
closed chains' step came: a spasm as the gait "released" the robot, a lie across
the bed instead of along it, and a body held rigid on the mattress. The
recording's replay walks the robot into the bed (its commands all predate the
lie; the errand's say is internal), so the headless twin is
`tools.sim.house --one -1.0 1.8 3.14 lie_bed --said`.

- The spasm was the regime flipping on a bouncing sole or a vanishing
  mattress contact: 5 flips in 0.9 s, 87-108 N m a flip. `chain.chained`
  now reads the robot's own release - no sole standing and none asked to land,
  its legs let go - instead of its stance: 1 flip, the bed's lie the same, the
  chair's rise the upright step's throughout.
- The chain step ignored the robot's pelvis's and trunk's turn asks; the error
  grew to 3 rad and the robot turned by leg momentum alone. The chain step's
  first stage is now one damped least squares over every acceleration
  (`chain.tasks`): each patch's points held on its normal and its centroid on
  its tangents, torsion free, the turns, a swing, the robot's posture and base
  at rest, the held joints. The turn's weight swept against the posture's 1
  (bed: off the axis / rolled; lodge: off / trunk deg/s): 30 gave 88 / 7.1 and a
  fall off the lodge bed at 909; 2 gave 85 / 1.6 and 79 / 633; 1 gave 88 / 17.9
  and 28 / 151 at one spot but 88 at another; 0.5 gave 81 / 25.4; 0.3 gave 67 /
  0.6 and 62-66 / 163 with the calmest jerks of every variant (pelvis 338,
  head 561) but pumped against the robot's feet sat on the floor (test_floor:
  its seat 223-324 N of a weight read 328 for 306); 0.1 and 0 gave 65 / 32 and
  56 / 155. The text's is 0: two-joint arms on pinned fists turn the robot's
  little and the third stage's foot push undoes it; the rows stay, unit-tested
  weighed up, till the chain gives the robot a lever.
- The rest: `se2.limp` lets the robot go limp by state - its back told to bear
  it and its row's blend done (bed_rest's ends 1.4 s in) - where only the name
  floor_rest did. The robot's clamps read 0 at the end of the lodge's lie.

Still across: in the bed scene the robot turns from 90 to 35 deg off the axis
through the trochanter stages, then the third stage's leg stretch lifts it
off the bed on its right foot alone for 0.5 s (prop 0, right_foot the one
body bearing) and it lands across again; the lodge ends 62-66 deg off.
The standing-foot push at the leg swap is the next thing to take away.
Relaxed across the bed the robot rolls 20-30 deg under the robot's overhanging
legs, so the bed scene's `lay` judge (rolled within LIST_RAD) reads red while
the robot held flat across it rigid; the judge wants the robot along the bed
first.

## The lift's push and the rest's drop

The robot's own loop recording (`watch.recorded`,
recordings/lodge_lie_eased2.jsonl; `tools/sim/recording.py --overview 0.25`) and
a per-pass lift trace showed the spasm the page recording caught: with one leg
let go and one sole on the bed the upright step's balance rows planned 278-316 N
on the standing sole, the prop latch let go, its joints went 255 to 566 deg/s,
its pelvis rose 0.15 m, it was airborne and tumbled onto its stomach.
`chain.chained` now reads a leg let go too, and `chain.bearing` hands a standing
sole to the chain as one more patch, held in its friction's box, never planned
as a push: both lifts run on the chain, no sole push planned, the robot's hips
at most 70 deg/s, the robot's pelvis 0.57-0.59 m, no airborne pass, held on the
robot's back.

The rest dropped the robot's clamps in one pass at the row's blend end: its neck
fell 64 deg at 138 deg/s, its spine 52 at 153, its arms 109 at 142 (its
walk's bound 57 / 41 / 128). `se2.limp` now eases 0 to 1 over as long as
the row blended: 110 / 97 / 111, the 97 the robot's sit's spine at 13.9 s. What
drops is the robot's lying down itself: through the three stages its trunk stays
25-30 deg from upright and its feet on the floor, and gravity lays its back
when the clamps go, its head 0.6 m.

Why the chain's step does not lay the robot down, measured on a python twin
(session scratchpad, lie_chain2.py): `chain.tasks` meets the robot's pelvis's
and trunk's turn rows with the base's own accelerations, which the force stage
discards, so a turn asked of the robot's base reaches no drive; weighed 1, 3 and
10 it only adds violence at the rest (102, 198, 268 deg/s). A task stage made
dynamics-consistent - the robot's base's and passive joints' accelerations what
the held contacts and the robot's dynamics make of the robot's driven ones, a
contact the plan would pull on freed, one dragged past its box sliding with its
friction known - answers a turn through the robot's drives: with the robot's
hands told to bear (palmar 0.2 through the lie) the robot lifts its pelvis onto
them, scoots 0.3 m in 0.7 s and reclines braked at 15-20 deg/s on hands bearing
35-48 N. The stage still pushes on the let-go foot at the second lift (289-377
N, the robot's knee 161-171 deg/s, the clamps shrunk to 0.04-0.35), turns the
robot not at all (yaw error -31 to -113 deg: 27 N of friction a hand, the
ischial patch's two points), lays the robot's legs straight up (feet 1.15 m: the
seated chain pose at the ischial share 0.45 keeps the robot's hips flexed) and
the limp still drops the robot's legs (168-209 deg/s). Not ported: on these
measures it does not beat the step above. The next lever is the text's: hands
that bear through the lie, legs told along the bed, a turn the robot's hands let
through.

Two measuring faults: `recording.py` numbered its windows with `list.index`
over blocks holding arrays and threw on the second; a scripted recording
loses its last 1.5 s unless the gate's writer is drained before the process
ends (its piping thread is a daemon). The house's chair spot (0.32, 1.01,
1.37) falls on HEAD as here (14.2 s there, 13.4 here); the errands' four red
rows are HEAD's.

## The plan through the robot's dynamics

The chain's task stage is now the plan the twin proved (`chain.planned`,
wbc_chain.c, closed_chain_plan.lisp): the dofs no drive holds affine in the
robot's driven accelerations through the contacts held and its dynamics, so a
turn or a swing is answered by its drives; its posture in their damped null
space. Three things the twin had not shown came out in the port, each by its
numbers:

- Two contacts on one prescribed chain hold the robot in any split of its
  weight: the damped map planned a pull of 280 N on its pelvis against 630 N on
  its sole where it bore 200 and 100, then let the pelvis go. Penalising all
  force change either kept the pull (weight 0.005 and under) or forbade every
  lift (0.02); the fight is the forces with no effect on the robot's free dofs,
  null(Ja_P^T), and only that component is now kept at the felt split
  (`force-weight` 0.02).
- The Gauss-Seidel warm from what is felt, lagged by the mattress constant,
  never carried the planned push to the robot's drives: the base turn it
  realised was nil while the plan foresaw -3 to -6 rad/s^2. Warm from the plan's
  own forces, foreseen and realised agree to the decimal.
- The robot's gains were the gait's (100, 20): a hip yaw told 0.44 rad asked 44
  rad/s^2 and the plan pulled it off the seat. The robot's own are 4 / 4 for a
  turn and 14 / 7.5 for the posture; at 16 and 20 the robot's hands' arrival
  threw the robot's elbow 374-428 deg/s, at 12 the robot's floor sit's trunk sat
  10.7 deg off its pitch. A contact felt under 30 N starts free (`hold-n`): at
  10 the fingers' arrival at 2-4 N, held, still whipped the robot's elbow 324
  deg/s, at 20 its floor sit fell, at 60 it fell off the bed. The task damping
  is 0.3: at 0.5 and 1.0 the robot's floor sit fell over at 7.1 s. The robot's
  pelvis is two ischial points always: the pivot's single point let the plan
  spin it for free.

Measured on the lodge lie (C step): every joint under 93 deg/s through the
lie - the lifts 63 and 47 deg/s, no clamp shrink -, the house's fields arms
107 (bound 128), trunk 97 (the robot's sit's spine, 13.9 s), neck 69 (walk_back,
8.8 s). The parity suite runs eight contacts: a five-contact one missed the
C plan's row compaction overwriting unread rows whenever more than half the
rows stayed held, the bug that threw the robot's elbow 449 deg/s in the loop
alone.

What the lie still lacks is leverage, not calm: the robot sits through it (lean
16-17 deg, feet 0.09-0.12 m, yaw error -68 deg, across the bed). The robot's
hands never reach the mattress - the carpal seek leaves them 0.25-0.30 m above
it and the upright step does not recline the robot the -0.40 rad the carpal
stage asks (lean 3-5 deg) -, so the chain has no lever for the recline or the
turn; on the robot's pelvis's two points alone no plan can spin the robot, the
friction pins them. With the fingers held at 10 N the robot's hip yaw turned its
50 deg during the second lift before the arrival whipped it. The text's long
side now turns the robot's trunk a little first (an ischial stage, 600 ms, -0.35
rad), keeps the robot's hands told to bear through the lifts (0.30, 0.25, 0.25,
0.10), and a let-go leg turns at its hip toward the told heading
(`emancipation.hipped`). Next is the hands' seek onto the mattress behind the
robot; the floor sit's knee bound moved 124 to 130 deg (127 under the calmer
posture gain).

## The pull onto the bed

The take (lying/Lying Down from Sitting, 0.866 of the robot's leg): fists
planted 0.2 m behind, upright, by 0.9 s; the pelvis drawn 0.45 m up the bed in
1.3 s at 0.58 m/s at most, its height unchanged, both feet up 0.5 m as one; the
trunk down in 1.4 s; the legs stretched last. The bed scenario's `bed_behind` is
the bed's long side; `bed_foot_behind` its end, now `bed_foot`. At HEAD the
end-lie sat: lean 6-19 deg, fists 3-6 N on the foam, pelvis 0.06 m in, feet on
the floor, and the rest's limp folded the robot forward (head 0.52 m); the lay
judge called it met. Four mechanisms, each measured in memory before it was
written:

- The robot's fists hung 0.17 m over the mattress: `_recline` set them for the
  stage's end pitch (`seated.braced` at -0.40) while the robot's trunk stayed at
  -0.07 (the sit's pelvis 0.68 rad forward, its spine at -30). Planted
  upright 0.14 m behind (`carpal-ahead-m`) they touch; pressed 2-5 cm into
  the foam by position they bore 50-150 N and shrank the solve to 0.00-0.05
  (an infeasible fight of arm and contact rows): no press. They bear once
  the lean needs them.
- The scoot is the robot's feet: told plantar 0.45, ischial 0.15, palmar 0.40,
  knee 1.45 to 1.25, the robot's shin back 0.05 (`talocrural-rad`), the sagittal
  ask past the robot's heels so `risen_back` does not hold the row, the robot
  moves 0.16 m in 1.2 s calm (scale 1.0), the robot's pelvis rolling back to
  -0.6 rad on its ischia. A knee extension with the ask over the robot's heels
  is a rise and waits for ever on the bed; a pelvis-lift scoot (ischial 0) falls
  backward.
- The lean on the fists is stable to the feet's unloading (-0.35 asked, -31
  deg met, hands 93 N, feet 9 N) only with the arms a shade too straight
  (`FOLD_MARGIN` 0.15 on `seated.planted`): folded to the reach ahead of the
  robot's pitch they drew it back into a fall, 853-1128 deg/s at the elbow. A
  fist 0.20 m behind is the robot's own to pitch -0.3; at -0.6 within 0.06 m, at
  -1.2 beside it, flat 0.05 m ahead: the pull is the arms' reach, as the take's
  hands pass from 0.17 m behind to 0.07 ahead.
- At the chain's handover the robot was hauled forward 37 deg and sat up: a
  let-go leg's hip reference carried the ASKED pelvis pitch (`cascade.opening`'s
  aim, -20 deg) against the -57 the scoot had left, and the chain's turn task
  asked the pelvis the text's torso pitch. The aim follows the pelvis's sensed
  pitch as the robot's back is told to bear (`told[DORSAL]`, the word that the
  robot's legs are to lift) and the ask while the robot's legs rest or stand: on
  the sensed pitch alone the floor sit fell at 7.1 s, by the legs' lift share
  the haul reloaded the robot's soles and the ask came back, and let go with the
  back half told the robot hauled itself up - the lean stage tells it first. The
  pelvis's turn rows are about its own way and up (`chain.ways`,
  wbc_chain_tasks.c), its pitch the robot's posture's; a patch is held by what
  its segments feel together (hand and fingers 23 N each were free under
  `hold-n` 30 with 93 N on the fists). With all three the robot lies down on the
  fists at 62 deg/s (the take 64), 0.35 m up the bed (the take 0.39, the
  scenario's `went` -0.52), head 0.52 m, feet 0.36-0.61 along the mattress,
  soles 0.00; the handover still spikes its elbow 193 and waist 264 deg/s (the
  lodge's fields from the bed's foot: arms 271, neck 90, trunk 290 in the
  scoot), the next polish.

The long side keeps its stages on the new plant: the robot's legs lift now and
the robot sits up on them (met, soles 0.57, no fall); its own pull is open. The
dock is `:short`: from the long side's doorstep the walk round the bed's corners
falls as before (the corner brake, not the lie: csv_rec at 19.8 s, south_face at
12.6 s). Lying 0.35 m up the bed, `rise_bed` sticks at its lean - its sit-up
leaves the robot's feet on the mattress, the thrust wants the floor - so the
errands' F for bed fails on the rise; its chair rows and the rise leads (1.881,
3.781, 3.781) are red at HEAD alike.

## Both fists down

The lodge's end-lie failed where the scenario lay. Measured, the bed (top
0.44 m to 0.45, one box to mattress and frame), the room's physics, the
dock's 0.137 rad yaw and the turn ask zeroed changed nothing; one fist bore
alone. `seated.planted` set the fists for the stage's told torso pitch; sat,
the robot's pelvis is 0.68 rad forward of it, so they hung 0.11-0.15 m over the
foam, the first down bore alone (58-73 N against 0-10) while the other hung 4-13
cm up, the robot rolled and yawed 10-13 deg onto it, the lean stalled 8-10 deg
short of its -20 ask (feet 70 N) and the handover hauled the robot forward.
Three mechanisms, each measured first:

- `seated.kept`, per tick (`somatic.asked`): a fist told to bear is sought to
  the point the text planted (read back off the compiled ask by a two-link
  reach in the robot's sagittal plane, within 1.6 cm of `figure`) under the
  robot's pelvis's pitch as sensed, the robot's spine's bend and trunk's roll;
  one that has borne keeps that point, fixed along the robot's way as the
  robot's hips scoot past it. Held at its joints instead the arm went limp and
  the release dropped the robot at 75 deg/s; sought to the stage's own point, an
  unlatched fist swung beside the robot's hip at the scoot's first tick and the
  scoot stood the robot up: the text keeps one point, 0.14 m behind.
- The press is sought, the seat identified. Set on the foam the fists bore 3 N,
  no contact to the step, and the scoot's told soles stood the robot up; a fixed
  centimetre bore 50 N there and fought the rigid floor (the floor scoot fell
  back). Measured, the mattress gives 0 N for its first centimetre, 30-50 N over
  2-7 cm (a tangent near 400 N/m) and creeps 1-3 cm/s at steady force. `kept`
  identifies the seat per hand (felt force over sink since the touch, a secant)
  and presses by the kernel's admittance (seek 0.25 m/s, yield 45 N s/m) at a
  gain bounded by that stiffness: dead-beat on a floor, a sink on foam, the
  press a seek step ahead of the fist at most (led by the reach's 1.6 cm error
  it pressed the robot off the lean, both scenes fell). The point is fixed in
  the world in height too, off the robot's pelvis's middle (pelvis-relative, the
  chair's rise lifted the fists off its seat; the robot's hips' midpoint moves
  3-4 cm against the middle as the pelvis rolls back); a point pressed under the
  seat (the hitch's) is never taken up, one taken up is kept; one fist still
  hovers over the foam.
- The chain's pelvis rows take the turn of the robot's left axis to the asked
  left (`chain.ways`, wbc_chain_tasks.c): the log map's way and up parts carried
  0.04-0.09 rad of a 35 deg pitch mismatch, its way part the 17 deg the robot
  ended off the bed's line.

Now the lodge lies 0.33 m up the bed (the take 0.39), lean -91, 7 deg off its
line, and the scenario 0.13 m, -97, 6 deg, each by a drop onto the forearms at
the lean's end (0.38 m with the fixed press). Open: yawed to the bed the robot's
thighs meet its edge unevenly (23/54 N) and one fist takes the lean, so it ends
in a drop onto the forearms and the robot's elbows prop the recline at -63 to
-76 till the rest lays the robot's flat; the straightening is the chain's
heading row at turn-kp 4, slow; the shrimp needs two strokes.
