# Her lie on the floor

## Refused by her legs

`motion.lisp`'s lie_floor and rise_floor were axial-compression phases
mislabelled floor_rest: said on the floor mattress she stood on it at
pelvis 0.763 and the scenario's 'lay' read it met. Deleted; both route
through the suite's patches (`maneuver.PLAYED`). At the floor mattress's
0.00 m top the patch asks compression 4.10 where her legs realize 3.4 -
she froze mid-descent at 0.505 m - so a seat asking her pelvis under her
legs' floor is refused by name (`maneuver.text`): top + her ischia's 0.128
(`figure`'s seat capsules) against `figure.pelvis_at` at the knee drive's
154.7 deg, 0.299 m; the bed's 0.44 plays (0.568), the floor mattress
refuses (0.132) till the deep landing (item 13). A stunt said with nothing
among her things looks till a sighting is hers and refuses with none:
unseen, the patch's default 0.45 m seat played a descent onto nothing. The
mattress scenario holds the refusal; the bed scenario's miss (rolled 10.8
deg of LIST's 8.6) is the baseline's own, bit for bit at a31b9186.

Laid supine, the fall window was the spasm: fallen, the loop kept driving
(`motion.drives`, `edge.drive`) and the rest law fought the ground - 1 658
deg/s and 3 793 N into it on the mattress, 40 deg/s and 45 N with the
drives off - the thrash of every fall and tipped lie until the epoch
relays her. Fallen her drives let go now (`worker.stepped`, `edge.off`,
the bench's STO): torque clamps zero within the flag's 0.1 s, the window's
rest the laying transient's own scale (934 deg/s, 1 219 N against the
passive transient's 1 420, 2 936). The rise from the floor stays open: the
suite's wake patch at seat 0 compiles targets past the modes' own range
and the loop refuses them (compression 6.62, sagittal -2.06) - a floor
rise is a new patch over the tibial keel and carpal manifolds (kneel,
half-knee), item 13's landing its other half.

## The rise's true dependency

Asked a deep crouch straight (a text at mode_axial_compression 4.0 on the
flat floor), her ankle was driven to 215.8 deg of its -45..42 range by t
2 s and she fell at 3.0: the compression mode's one pattern (hip -K/2,
knee K, ankle -K/2) binds at the ankle's -45 near knee 90 deg, and past it
her feet fold. A setpoint stays within its drive's range now
(`motion.drives` clips `ref_q` into the manifest's, laid once in
`sprung.laid`): the same ask keeps every joint in range (ankle 40.2 of 42)
and physics itself refuses the depth - she overbalances at 4.7 s instead
of breaking at the ankle - and nothing out of range can reach the bench's
boards; her walk is unmoved to the digit (0.70 m/s, 408 J/m, 2.52).
The loop's realized floor is pelvis ~0.51; her legs' geometry allows
0.299 (knee 154.7) only with the heels up - the kernel's
:metatarsal-equinus-bilateral manifold, named in its texts, not yet among
the support buses (plantar, ischial, trochanter-lateral, scapular-dorsal,
occipital, palmar). The floor rise's chain, each measured: supine (quiet,
drives let go) -> roll on the trochanter (the pendulum act, in range) ->
no path below pelvis 0.51 without the equinus redistribution or a tibial
(kneeling) bus - item 13's deep landing is the same dependency from above.

## She lies down where she stands

`lie_floor` is her gym's own text again (motion.lisp): kneel (compression
3.2), tip (pitch -2.0 past what statics can hold, the ZMP's polygon let go
and the support panned off her soles - at -0.9 she balanced a reclined
crouch at 0.422 and stood there), floor_rest limp. Three mechanisms carried
it: `se2.GROUNDED` (kneel, tip, floor_rest) holds the fall judge off an
intended lie - unexcused she was epoch-reset mid-tip -; the grammar's
per-phase `(maintain_zmp (polygon 0))` and `(contact_manifolds ..)` let the
text itself release her balance law; and `se2.LIMP` (floor_rest) zeroes her
drives' clamps over the bus (`motion.drives`) - resting on the text's own
springs she churned on the floor, 200-1 400 deg/s, pitch rolling full
circle; limp she lies at 0-1 deg/s, 88-107 N, pelvis 0.198, no reset, 60 s.
Measured on the page itself ('lay on floor' in the prompt): down at T+12,
quiet. Her arms went as a propeller in the tip - the balance rows windmilled
against the backward fall, the shoulder sweeping 1 to 234.7 deg at 1 385
deg/s, the hand slamming 1 359 N - where a human sets the hands behind
beside the legs, four points: the kneel and tip place and hold them
(mode_arm_reach -1.4 then -3.9, `seated.braced`'s shoulder flexion at the
trunk's pitch over the unit's 0.35; arm stiffness 1200, the palmar
manifold panned 0.6): the shoulder stays -75..-27 deg at 207 deg/s and her
palms take the landing, 1 065 N, resting at 10.
The pose is a heap - folded back over her heels, pitch -122 - not a
tidy supine: the recline to flat (the bed's 'lay' bar, rolled within LIST)
and the rise are the rest; her walk unmoved to the digit.

## The clamps frozen by the limp

The limp's branch-free form rebound her clamps to their own product each
pass (`g['cmd']['limit'] = limit * (1 - limp)`): from the first pass on
they never took the step's live ones (`motion.drives`'s kept limit), and a
rise from the low chair - sat 22.4 s, stood - sat at 25.5 and fell at 31.9;
found by the live gynoid suite, bisected to the commit. The cmd's limit a
buffer laid once, the live clamps multiplied into it: sat 22.4, stood, the
floor lie as before (pelvis 0.200), her walk to the digit.

## The closed chains' step

Lying on the bed she spasmed at the foam's 1.9 Hz and rolled 19.2 deg on
the long side (`tools.sim.scenarios bed`: lay not met, jerk pelvis 461,
head 609, hands 1573 / 999, feet 1346 / 1814): one solver ran every
regime. The upright step (`machine.stack.wrench`) pins every bearing point
at W_STILL and plans the support's push through its balance rows; on a
yielding multi-point support both are asks it cannot meet. The told row
parameter `felt` that folded the bed's push into her bias (a SUPINE list
of chain ends in acts) was a per-text special case and is gone.

Her second step, `machine.stack.chain` with `wbc_chain.c` bit for bit
(test_chain.py, 1e-9 over 12 states), is picked by her stance alone: no
sole standing and what is not her soles bearing her. Her driven joints
follow her posture in the damped null space of the contacts' Jacobian
(Levenberg-Marquardt, Cholesky); the contacts' forces come from a
box-projected Gauss-Seidel over her Delassus matrix on the dofs no drive
holds - pressing alone, within their friction's box, warm from what is
felt, a mattress's give as a pass of lag -; her torques follow, shrunk to
her clamps. The regime's constants are the kernel's
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
896 / 2034 (the regime flickered as her pelvis bore alone); latched on the
prop comparator's impact blanking 0.6, 526 / 816 / 2074 (the leg swap's
hop ran on the upright step and slammed her foot). A chair-edge bounce in
the broken rise from the chair now runs three passes on the chains' step;
the errands suite's four red rows are HEAD's own.

Two gotchas: `wbc_step_t`'s new regime fields sit after its workspace -
in front of it they shifted the workspace by 40 bytes and numpy's BLAS
kernels answered 47 of 12 668 solves a few bits apart from the python
step; and a test module parameter named `step` shadowed the glue module.

## The release, the turn and the rest

A page recording of the lodge's lie (build/recordings/humanoid_20261101_130722.csv)
showed three faults after the closed chains' step came: a spasm as the gait
"released" her, a lie across the bed instead of along it, and a body held
rigid on the mattress. Its replay walks her into the bed (the recording's
commands all predate it; the errand's say is internal), so the headless twin
is `tools.sim.house --one -1.0 1.8 3.14 lie_bed --said`.

- The spasm was the regime flipping on a bouncing sole or a vanishing
  mattress contact: 5 flips in 0.9 s, 87-108 N m a flip. `chain.chained`
  now reads her own release - no sole standing and none asked to land, her
  legs let go - instead of her stance: 1 flip, the bed's lie the same, the
  chair's rise the upright step's throughout.
- The chain step ignored her pelvis's and trunk's turn asks; the error grew
  to 3 rad and she turned by leg momentum alone. Its first stage is now one
  damped least squares over every acceleration (`chain.tasks`): each
  patch's points held on its normal and its centroid on its tangents,
  torsion free, the turns, a swing, her posture and base at rest, the held
  joints. The turn's weight swept against the posture's 1 (bed: off the
  axis / rolled; lodge: off / trunk deg/s): 30 gave 88 / 7.1 and a fall off
  the lodge bed at 909; 2 gave 85 / 1.6 and 79 / 633; 1 gave 88 / 17.9 and
  28 / 151 at one spot but 88 at another; 0.5 gave 81 / 25.4; 0.3 gave 67 /
  0.6 and 62-66 / 163 with the calmest jerks of every variant (pelvis 338,
  head 561) but pumped against her feet sat on the floor (test_floor: her
  seat 223-324 N of a weight read 328 for 306); 0.1 and 0 gave 65 / 32 and
  56 / 155. The text's is 0: two-joint arms on pinned fists turn her little
  and the third stage's foot push undoes it; the rows stay, unit-tested
  weighed up, till the chain gives her a lever.
- The rest: `se2.limp` lets her go limp by state - her back told to bear her
  and her row's blend done (bed_rest's ends 1.4 s in) - where only the name
  floor_rest did. Her clamps read 0 at the end of the lodge's lie.

Still across: in the bed scene she turns from 90 to 35 deg off the axis
through the trochanter stages, then the third stage's leg stretch lifts her
off the bed on her right foot alone for 0.5 s (prop 0, right_foot the one
body bearing) and she lands across again; the lodge ends 62-66 deg off.
The standing-foot push at the leg swap is the next thing to take away.
Relaxed across the bed she rolls 20-30 deg under her overhanging legs, so
the bed scene's `lay` judge (rolled within LIST_RAD) reads red while
she held flat across it rigid; the judge wants her along the bed first.

## The lift's push and the rest's drop

Her own loop recording (`watch.recorded`, recordings/lodge_lie_eased2.jsonl;
`tools/sim/recording.py --overview 0.25`) and a per-pass lift trace showed
the spasm the page recording caught: with one leg let go and one sole on
the bed the upright step's balance rows planned 278-316 N on the standing
sole, the prop latch let go, her joints went 255 to 566 deg/s, her pelvis
rose 0.15 m, she was airborne and tumbled onto her stomach. `chain.chained`
now reads a leg let go too, and `chain.bearing` hands a standing sole to the
chain as one more patch, held in its friction's box, never planned as a
push: both lifts run on the chain, no sole push planned, her hips at most
70 deg/s, her pelvis 0.57-0.59 m, no airborne pass, held on her back.

The rest dropped her clamps in one pass at the row's blend end: her neck
fell 64 deg at 138 deg/s, her spine 52 at 153, her arms 109 at 142 (her
walk's bound 57 / 41 / 128). `se2.limp` now eases 0 to 1 over as long as
the row blended: 110 / 97 / 111, the 97 her sit's spine at 13.9 s. What
drops is her lying down itself: through the three stages her trunk stays
25-30 deg from upright and her feet on the floor, and gravity lays her back
when the clamps go, her head 0.6 m.

Why the chain's step does not lay her down, measured on a python twin
(session scratchpad, lie_chain2.py): `chain.tasks` meets her pelvis's and
trunk's turn rows with the base's own accelerations, which the force stage
discards, so a turn asked of her base reaches no drive; weighed 1, 3 and 10
it only adds violence at the rest (102, 198, 268 deg/s). A task stage made
dynamics-consistent - her base's and passive joints' accelerations what the
held contacts and her dynamics make of her driven ones, a contact the plan
would pull on freed, one dragged past its box sliding with its friction
known - answers a turn through her drives: with her hands told to bear
(palmar 0.2 through the lie) she lifts her pelvis onto them, scoots 0.3 m in
0.7 s and reclines braked at 15-20 deg/s on hands bearing 35-48 N. It still
pushes on the let-go foot at the second lift (289-377 N, her knee 161-171
deg/s, the clamps shrunk to 0.04-0.35), turns her not at all (yaw error -31
to -113 deg: 27 N of friction a hand, the ischial patch's two points), lays
her legs straight up (feet 1.15 m: the seated chain pose at the ischial
share 0.45 keeps her hips flexed) and the limp still drops her legs
(168-209 deg/s). Not ported: on these measures it does not beat the step
above. The next lever is the text's: hands that bear through the lie, legs
told along the bed, a turn her hands let through.

Two measuring faults: `recording.py` numbered its windows with `list.index`
over blocks holding arrays and threw on the second; a scripted recording
loses its last 1.5 s unless the gate's writer is drained before the process
ends (its piping thread is a daemon). The house's chair spot (0.32, 1.01,
1.37) falls on HEAD as here (14.2 s there, 13.4 here); the errands' four red
rows are HEAD's.

## The plan through her dynamics

The chain's task stage is now the plan the twin proved (`chain.planned`,
wbc_chain.c, closed_chain_plan.lisp): the dofs no drive holds affine in her
driven accelerations through the contacts held and her dynamics, so a turn
or a swing is answered by her drives; her posture in their damped null
space. Three things the twin had not shown came out in the port, each by
its numbers:

- Two contacts on one prescribed chain hold her in any split of her weight:
  the damped map planned a pull of 280 N on her pelvis against 630 N on her
  sole where she bore 200 and 100, then let the pelvis go. Penalising all
  force change either kept the pull (weight 0.005 and under) or forbade
  every lift (0.02); the fight is the forces with no effect on her free
  dofs, null(Ja_P^T), and only that component is now kept at the felt split
  (`force-weight` 0.02).
- The Gauss-Seidel warm from what is felt, lagged by the mattress constant,
  never carried the planned push to her drives: the base turn she realised
  was nil while the plan foresaw -3 to -6 rad/s^2. Warm from the plan's own
  forces, foreseen and realised agree to the decimal.
- Her gains were the gait's (100, 20): a hip yaw told 0.44 rad asked 44
  rad/s^2 and the plan pulled her off the seat. Her own are 4 / 4 for a turn
  and 14 / 7.5 for the posture; at 16 and 20 her hands' arrival threw her
  elbow 374-428 deg/s, at 12 her floor sit's trunk sat 10.7 deg off its
  pitch. A contact felt under 30 N starts free (`hold-n`): at 10 the
  fingers' arrival at 2-4 N, held, still whipped her elbow 324 deg/s, at 20
  her floor sit fell, at 60 she fell off the bed. The task damping is 0.3:
  at 0.5 and 1.0 her floor sit fell over at 7.1 s. Her pelvis is two ischial
  points always: the pivot's single point let the plan spin her for free.

Measured on the lodge lie (C step): every joint under 93 deg/s through the
lie - the lifts 63 and 47 deg/s, no clamp shrink -, the house's fields arms
107 (bound 128), trunk 97 (her sit's spine, 13.9 s), neck 69 (walk_back,
8.8 s). The parity suite runs eight contacts: a five-contact one missed the
C plan's row compaction overwriting unread rows whenever more than half the
rows stayed held, the bug that threw her elbow 449 deg/s in the loop alone.

What the lie still lacks is leverage, not calm: she sits through it (lean
16-17 deg, feet 0.09-0.12 m, yaw error -68 deg, across the bed). Her hands
never reach the mattress - the carpal seek leaves them 0.25-0.30 m above it
and the upright step does not recline her the -0.40 rad the carpal stage
asks (lean 3-5 deg) -, so the chain has no lever for the recline or the
turn; on her pelvis's two points alone no plan can spin her, the friction
pins them. With the fingers held at 10 N her hip yaw turned her 50 deg
during the second lift before the arrival whipped her. The text's long side
now turns her trunk a little first (an ischial stage, 600 ms, -0.35 rad),
keeps her hands told to bear through the lifts (0.30, 0.25, 0.25, 0.10),
and a let-go leg turns at its hip toward the told heading
(`emancipation.hipped`). Next is the hands' seek onto the mattress behind
her; the floor sit's knee bound moved 124 to 130 deg (127 under the calmer
posture gain).

## The pull onto the bed

The take (lying/Lying Down from Sitting, 0.866 of her leg): fists planted
0.2 m behind, upright, by 0.9 s; the pelvis drawn 0.45 m up the bed in 1.3 s
at 0.58 m/s at most, its height unchanged, both feet up 0.5 m as one; the
trunk down in 1.4 s; the legs stretched last. The bed scenario's `bed_behind`
is the bed's long side; `bed_foot_behind` its end, now `bed_foot`. At HEAD
the end-lie sat: lean 6-19 deg, fists 3-6 N on the foam, pelvis 0.06 m in,
feet on the floor, and the rest's limp folded her forward (head 0.52 m);
the lay judge called it met. Four mechanisms, each measured in memory before
it was written:

- Her fists hung 0.17 m over the mattress: `_recline` set them for the
  stage's end pitch (`seated.braced` at -0.40) while her trunk stayed at
  -0.07 (the sit's pelvis 0.68 rad forward, its spine at -30). Planted
  upright 0.14 m behind (`carpal-ahead-m`) they touch; pressed 2-5 cm into
  the foam by position they bore 50-150 N and shrank the solve to 0.00-0.05
  (an infeasible fight of arm and contact rows): no press. They bear once
  the lean needs them.
- The scoot is her feet: told plantar 0.45, ischial 0.15, palmar 0.40, knee
  1.45 to 1.25, her shin back 0.05 (`talocrural-rad`), the sagittal ask past
  her heels so `risen_back` does not hold the row, she moves 0.16 m in 1.2 s
  calm (scale 1.0), her pelvis rolling back to -0.6 rad on its ischia. A knee
  extension with the ask over her heels is a rise and waits for ever on the
  bed; a pelvis-lift scoot (ischial 0) falls backward.
- The lean on the fists is stable to the feet's unloading (-0.35 asked, -31
  deg met, hands 93 N, feet 9 N) only with the arms a shade too straight
  (`FOLD_MARGIN` 0.15 on `seated.planted`): folded to the reach ahead of her
  pitch they drew her back into a fall, 853-1128 deg/s at the elbow. A fist
  0.20 m behind is hers to pitch -0.3; at -0.6 within 0.06 m, at -1.2 beside
  her, flat 0.05 m ahead: the pull is the arms' reach, as the take's hands
  pass from 0.17 m behind to 0.07 ahead.
- At the chain's handover she was hauled forward 37 deg and sat up: a let-go
  leg's hip reference carried the ASKED pelvis pitch (`cascade.opening`'s
  aim, -20 deg) against the -57 the scoot had left, and the chain's turn
  task asked the pelvis the text's torso pitch. The aim follows the pelvis's
  sensed pitch as her back is told to bear (`told[DORSAL]`, the word that
  her legs are to lift) and the ask while her legs rest or stand: on the
  sensed pitch alone the floor sit fell at 7.1 s, by the legs' lift share
  the haul reloaded her soles and the ask came back, and let go with the
  back half told she hauled herself up - the lean stage tells it first. The
  pelvis's turn rows are about its own way and up (`chain.ways`,
  wbc_chain_tasks.c), its pitch her posture's; a patch is held by what its
  segments feel together (hand and fingers 23 N each were free under
  `hold-n` 30 with 93 N on the fists). With all three she lies down on the
  fists at 62 deg/s (the take 64), 0.35 m up the bed (the take 0.39, the
  scenario's `went` -0.52), head 0.52 m, feet 0.36-0.61 along the mattress,
  soles 0.00; the handover still spikes her elbow 193 and waist 264 deg/s
  (the lodge's fields from the bed's foot: arms 271, neck 90, trunk 290 in
  the scoot), the next polish.

The long side keeps its stages on the new plant: her legs lift now and she
sits up on them (met, soles 0.57, no fall); its own pull is open. The dock
is `:short`: from the long side's doorstep the walk round the bed's corners
falls as before (the corner brake, not the lie: csv_rec at 19.8 s,
south_face at 12.6 s). Lying 0.35 m up the bed, `rise_bed` sticks at its
lean - its sit-up leaves her feet on the mattress, the thrust wants the
floor - so the errands' F for bed fails on the rise; its chair rows and the
rise leads (1.881, 3.781, 3.781) are red at HEAD alike.

## Both fists down

The lodge's end-lie failed where the scenario lay. Measured, the bed (top
0.44 m to 0.45, one box to mattress and frame), the room's physics, the
dock's 0.137 rad yaw and the turn ask zeroed changed nothing; one fist bore
alone. `seated.planted` set the fists for the stage's told torso pitch; sat,
her pelvis is 0.68 rad forward of it, so they hung 0.11-0.15 m over the foam,
the first down bore alone (58-73 N against 0-10) while the other hung 4-13
cm up, she rolled and yawed 10-13 deg onto it, the lean stalled 8-10 deg
short of its -20 ask (feet 70 N) and the handover hauled her forward.
Three mechanisms, each measured first:

- `seated.kept`, per tick (`somatic.asked`): a fist told to bear is sought to
  the point the text planted (read back off the compiled ask by a two-link
  reach in her sagittal plane, within 1.6 cm of `figure`) under her pelvis's
  pitch as sensed, her spine's bend and trunk's roll; one that has borne
  keeps that point, fixed along her way as her hips scoot past it. Held at
  its joints instead the arm went limp and the release dropped her at 75
  deg/s; sought to the stage's own point, an unlatched fist swung beside her
  hip at the scoot's first tick and the scoot stood her up: the text keeps
  one point, 0.14 m behind.
- The press is sought, the seat identified. Set on the foam the fists bore
  3 N, no contact to the step, and the scoot's told soles stood her up; a
  fixed centimetre bore 50 N there and fought the rigid floor (the floor
  scoot fell back). Measured, the mattress
  gives 0 N for its first centimetre, 30-50 N over 2-7 cm (a tangent near
  400 N/m) and creeps 1-3 cm/s at steady force. `kept` identifies
  the seat per hand (felt force over sink since the touch, a secant) and
  presses by the kernel's admittance (seek 0.25 m/s, yield 45 N s/m) at a
  gain bounded by that stiffness: dead-beat on a floor, a sink on foam,
  the press a seek step ahead of the fist at most (led by the reach's 1.6 cm
  error it pressed her off the lean, both scenes fell). The point is fixed in
  the world in height too, off
  her pelvis's middle (pelvis-relative, the chair's rise lifted the fists off
  its seat; her hips' midpoint moves 3-4 cm against the middle as the pelvis
  rolls back); a point pressed under the seat (the hitch's) is never taken
  up, one taken up is kept; one fist still hovers over the foam.
- The chain's pelvis rows take the turn of her left axis to the asked left
  (`chain.ways`, wbc_chain_tasks.c): the log map's way and up parts carried
  0.04-0.09 rad of a 35 deg pitch mismatch, its way part the 17 deg she
  ended off the bed's line.

Now the lodge lies 0.33 m up the bed (the take 0.39), lean -91, 7 deg off
its line, and the scenario 0.13 m, -97, 6 deg, each by a drop onto the
forearms at the lean's end (0.38 m with the fixed press). Open:
yawed to the bed her thighs meet its edge unevenly (23/54 N) and one fist
takes the lean, so it ends in a drop onto the forearms and her elbows prop
the recline at -63 to -76 till the rest lays her flat; the straightening is
the chain's heading row at turn-kp 4, slow; the shrimp needs two strokes.
