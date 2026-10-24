# Findings: the dual-domain kernel

Its steps on her loop and its tests in the lodge ([protocol](../todo/protocol.md),
host/machine/ortho_kernel/).

## The momentum observer

`machine.observer`, blind, in C (wbc_gmo.c on her chain, wbc.h): her drives'
torques (`edge.TAU`), encoders, IMU and estimate over M(q), low-passed at the
text's 35 Hz, split over the registry's patches by 4 least squares, each
pulling normal pinned for the next. Against her skin
(`tools/sim/observed.py`, each law pass from 1 s; rms N, her weight 306 N;
ghost: the most read where her skin feels under 5 N):

| scene | soles | seat | hips | back | palms |
| --- | --- | --- | --- | --- | --- |
| walk | 21.4 | 4.8, ghost 41 | 1.3, 6 | 1.2, 11 | 5.9, 33 |
| chair, sit and rise | 15.1 | 30.5, 97 | 2.9, 23 | 7.8, 93 | 11.6, 174 |
| bed, lie | 15.8 | 14.7, 112 | 7.5, 36 | 19.2, 14 | 11.8, 24 |
| bed, rise | 13.7 | 12.7, 112 | 6.4, 36 | 15.9, 21 | 9.6, 24 |
| floor mattress | 6.5 | 1.4 | 1.8 | 1.2 | 6.6 |

Its python (an active-set QP over MuJoCo's Jacobians) against the C, the same
passes walking: 0.1-0.4 N rms a manifold, 9 at most.

The somatic manifolds (rms N, against her skin; the most her skin felt):
her forefoot - her toes and of each sole the share its centre of pressure has
gone from its middle to its ball - walking 33.9 (439), seated 81.9 (253); her
thighs' backs seated 72.0 (416), lying a 303 N ghost (her calves and heels
on the bed, no point of hers). Her toes a patch of their own split the sole's
sum badly (walking 61.4 N rms where 20.8).

- Her law's M, h against MuJoCo's own constraint force at the world's
  qacc: 0.14 N at most. Given the exact contact force the split reads
  0.2 N mean on the soles, at most 1.3 N elsewhere.
- Her drives' friction (MuJoCo's frictionloss, 1.2-2.2 N m a drive) is
  0.42 of its loss at 3-10 mrad/s, 0.85 at 30-100, 0.97 past: a hip roll's
  2 N m reads 20 N under a sole. Unmodelled the soles read 19.5 N light and
  the seat 44 N borne walking; as Coulomb, tanh(v / 30 mrad/s), 3.3 and 27.
  Taken as bounded unknowns instead: worse (28.2 light).
- Her pelvis's speed differenced over a law pass made the patches only her
  base rows see - the seat, the sacrum, a palm pushed along a straight arm -
  its slack: walking the palms read 707 N. Her IMU's specific force for her
  pelvis's acceleration: 88.
- Her edge reports the step just ended; summed after the law pass the drives'
  torques lagged the velocity's interval by one 1 ms step: the seat's ghost
  52 N, aligned 19.
- The two hips' pushes lie on one line: least norm by the QP's tie-break.
- A thigh's back bears the seat (91 N on the pelvis, 89 on the thighs
  seated): counted to the seat's patch along her pelvis's up, to the back's
  along its forward - the registry's boxes reach 110 mm ahead of the
  buttocks and 420 mm below the sacrum. Without it the chair's soles erred
  43.6 N rms, the palms 72.

## Her pass's cost and its null space

- On this host, a law pass's C: her pose, M, bias and two Jacobians 6.2 us,
  her whole-body step 20.0, the observer 31.6 (43.4 with its call) - 58 us
  of the text's 350. The observer in numpy: 599 us, vectorized 385.
- Her trunk's turn asked 0.3 rad about up, her chest at 30 rad/s^2: standing
  her standing soles move 7.6e-5, her centre of mass 3e-5 m/s^2, her ankles'
  torques 1.5 N m; walking, double support, the soles 1.3e-3, her centre of
  mass 0.12-0.15 of its 1.7 m/s^2 asked (the soles' bounds bind and her
  rows trade at W_COM against W_TURN), her ankles 3.4-4.6 N m. Its sole
  forces change 19 N standing: the yaw's couple on the floor.

## Weights by force

- Taken as the observer's alone (each manifold's normal force of her weight,
  slewed 4.5 up and 45 down a second), the chair's, the bed's and its rise
  fell (10.2, 8.4 s): rising, her law kept the seat's share till it
  unloaded, and it unloads only once her law moves her onto her soles. The
  text pans its rises by told keyframes and binds the force only in its
  blind settle.
- Told weights where her comparator (0.18 / 0.06 of her weight, 45 ms dwell,
  18 ms blanking) feels the manifold, gated at 4.5 up and 45 down, the rest
  her soles' (`observer.weighed`): chair, bed and bed rise met as before; her
  walk 0.70 m/s, 408 J/m, 2.50 to the bit. Rising from the chair the seat
  lets her go at 8.55 s where its told pan runs to 9.0: her pelvis's jerk
  peak 957 (3 492 before), its 99th percentile 409 (266). Gated by the
  force's share of the engage share instead: 416, the head 909.

## Her capture point's barrier

`machine.barrier`: her centre of mass's floor ask projected onto each edge's
h' + 14 h >= 0 (h her capture point's distance in, less 18 mm) over the hull
of her bearing soles and the observer's pressing points on felt manifolds;
past an edge, no further out.

- The chair pulled 1 m back over 0.3 s from under her sit (`chair`, sit
  at 1 s, the seat touched at ~2.05): at 1.5 or 1.8 s she sinks into a
  squat, her pelvis at 0.56 m, the seat taken 0 throughout. At 2.0 s the
  seat is taken 0.08 for 0.05 s; her sit's rows straighten her trunk (her
  pelvis +41 to -8 deg) and she falls back at 3.64 s, with the barrier at
  3.87 - her capture point held at the heel's edge less the margin for
  0.5 s while her trunk's turn took the soles' room.
- On in every stance not striding, it threw the chair's rise forward off
  the seat (pulled back from outside; her ask's stance one sole as both
  bore her). On as far as her rows lean on what her comparator does not
  feel: chair, bed and bed rise as before (the chair's pelvis 283, head 744
  m/s^3 against 266, 783), her walk to the bit.

## Her DC link

`machine.dclink`, each world step: her drives' electrical power (work,
braking negative, copper, each board's housekeeping) from her two 15S2P packs
(48 V, 75 mOhm, their BMS's charge 19.6 A, estimated) into her 21 boards' DC
link (33 x 10 uF X7S each, 6.93 mF nominal), implicit in its volts.

| scene | peak V | least V | bled J |
| --- | --- | --- | --- |
| walk | 48.23 | 45.75 | 0 |
| chair, sit and rise | 48.54 | 45.30 | 0 |
| bed rise | 48.00 | 46.79 | 0 |
| sprint | 48.49 | 43.99 | 0 |
| hop down | 58.93 | 38.63 | 10.5 |

- A hop down's landing pushed the link to 60.78 V with the text's sinks
  alone (spine, shoulders, elbows: 880 W of d-axis copper); every board
  bleeding the current its drive's peak leaves its torque: 58.93.
- The bleed's copper is not yet in her drives' heat.

## The blind field

`machine.field`: while a seat's contact rows wait on a seat that does not bear
her and her rows' height has landed, her height seeks down at the text's
0.30 m/s through its 43 ms; held while her rows lean on a manifold, its floor
her knee where her thigh meets her shank less the text's 0.28 rad.

- The chair dropped 4 cm after she saw it: she seeks, finds it and sits
  (pelvis 0.52 m, the seat 0.85); without the field she hovers 4 cm over it.
  Rising from it she falls (11.7 s): the rise's rows are laid for the seen
  height.
- The chair pulled before it bears her (1.5 s): she seeks to a squat,
  pelvis 0.50 m, the seat taken 0. Its seek back (0.12 m/s) dropped: sought,
  her capture point went 0.18 m behind her soles' middle and she fell; with
  the barrier on as it sought, the chair's rise fell (9.88 s); seeking while
  her rows still descended pressed the seat, her pelvis's jerk 283 -> 490.
- Crouched at 0.47 m the observer reads 61-99 N on the seat her skin does
  not feel; a rise said there takes the latched ghost 0.15.

## The body schema

`machine.schema`: the text's 18 capsules by its names, frames and 13 pairs on
her skin (`figure.BODY`; her sole a capsule as wide as it from heel to ball,
her knee where the text lays it). Its own are another body's: its shank 0.41 m
and 68 mm round where hers 0.38 and 50, its thighs 95 mm round, through each
other at her hips 95 mm off her middle. Her posture's ask held to each near
pair's ECBF (11 Hz, 1.05), least in her mass's metric on the pair's barrier's
evading joint, within its drive's peak; each pair's floor raised by its
braking distance through `machine.bemf`.

- Walking (`play_walk`, 4 334 law passes, her law's pose): her feet 22.8 mm
  apart at least (floor 7.5), shanks 41.9 (12), knees 40.8 (12), forearm and
  hips 30.3 (15), hand and thigh 66.3.
- The text's log-sum-exp at 25 /m lies up to log(13) / 25 = 103 mm under the
  least: walking it read -43 mm with every pair clear.
- Her law's model is posed elsewhere within a pass after it is sensed (0.167
  rad off at 8.008 s walking): read there, her pairs' rates came out at metres
  a second. The shield poses data of its own.
- Its change on all 21 of her joints, unbounded: the wall's wobble fell at
  6.94 s (her forearm closing on her hips at metres a second), her 5 ms jump
  barely left the floor; on the text's evading DOFs with its limb synergies
  (waist, spine, both shoulders) within their peaks: the turning replay at
  +0.2 mm fell at 10.90 s, her left forearm held 228 passes closing on her
  hips 49 -> 21 mm, her posture moved up to 1.1 rad; on each pair's barrier's
  own DOF on its side, the forearm's abduction taken as her shoulder's flexion:
  the replay stood (87 passes held), her charged jump's forearms held at 32-88
  mm off her hips, her shoulder's target moved 3.15 rad, and she fell at 2.43
  s. Her shoulder only flexes: that pair's demand spilled whole, all three
  stand, as her base.
- Her smoke walk (`ortho.py --walk`) her base's to the digit: 2.50 off a
  woman's walk, landing load 1.39.
- A pass idle: 170 us in Python (22 Jacobians 67, the nearest points 52, the
  braking reach 35).

## The room on her shield

Her things (her map's boxes, read at the text's 120 Hz) against her arms'
capsules through their elbows alone (the text's limb synergy she has), each
box's patches by the text's affordance kernels, open by the approach window
where scheduled or felt: tried and undone. At the doorway's jambs
(test_errands' measure, deterministic) her arm struck at 95.5 N with it and
94.0 without on the left, 240 and 208 on the right; her shoulder yielded
-0.31 rad with it where -0.53 without on the left (the test's -0.40), -0.75
where -0.56 on the right: the elbow spared no strike and halved a yield. The
room's boxes stay her gauge's (`machine.wayfinder`); the text's shield holds
her trunk by her balance bus and her legs by her footholds, neither hers yet.

- Her legs held off the room through her hips (the self-collision barriers'
  DOFs): walking into the wall at +0.2 mm her knee was held 466 passes 31 mm
  into it and she fell at 7.13 s where her base meets it. Her arms alone, she
  meets it.
- A synergy's trunk joint moved for an arm (her waist's yaw): the Sit To
  Stand take's replay (falling on her base at 7.12 s) fell at 9.23; her own
  side's, 6.19; her arms alone, 5.83 - her forearms held 6 mm off the chair's
  back behind her 19 passes.
- Its cost a law pass on the lodge: 2.2 ms with bounding spheres (a 3 m wall's
  reaches every capsule: 73 pairs) and 40 golden steps on none; 0.54 ms with
  the box's distance to the capsule's middle (0 pairs walking the lodge), 21
  steps (a thousandth of the skin), the boxes laid once a refresh, the kernels
  as a matrix. test_errands ran past its 300 s wall on the first.
- A capsule's axis against a box by golden section: 24 random pairs within
  2.1e-5 m of a 20 001-point walk, the bracket's share of the segment.

## Her map

`machine.registry`: the text's 256 slots of what her looks told her - her eyes'
or their stand-in (the world's boxes), those her eyes' frustum shows at the
box's point nearest her eye -, matched to a slot of its kind in the
Wasserstein metric over the slot's and the look's variances (chi2 5.99), the
text's log-odds (+0.85 a match, -0.22 a miss before her eyes, 0 in a blind
spot 45 s, pruned under -3.5, her buses' at 1.8); a slot's drift the still
model's summed innovation, past 16.27 a mover at its velocity.

- Her bed errand through the lodge (29 s): 45 of its 53 boxes in her map,
  each once, none moving; the lamp, the roof's slabs, the switch, two rail
  posts and a timber she never looked at; 36 swollen as her eyes left them,
  32 mm at most.
- Every prop a look each refresh: all 53 in 25 ms, nothing behind her a
  blind spot. Her eyes' frustum on the looks.
- Associated over a thing's own size (the Wasserstein gate on its half
  sizes): a pane's gate reached its neighbour 1.1 m off, the slot jumped,
  read as moving and its velocity carried it away - 9 movers, the door 9
  slots, 50 for 53 props. Over the position's precision and within the
  kind: none.
- The chair shoved at 1 m/s under 8 ms looks: one slot, the still model's
  drift past 16.27 at its 28th look (0.22 s), its velocity 1.00 m/s.
- The door ajar at 0.8 passed 3 of 3 on her map's boxes.

## Her sit's dock

Why she sits on the chair's front edge leaning forward (`seats.py --sim
chair_behind sit_chair`, a human's band from the takes): her hips land 0.218 m
behind her ankles, 0.23 legs where a human's 0.31-0.47, her trunk seated 13.1
deg forward where within 7, her soles coming and going 14 times in 8 s.
`seated.landed` caps her hips a buttock past the seat's edge (0.140 + 0.078 m)
and has her thighs clear the edge where a human's rest on it. With her thighs
let rest the press her hips land with and the cap gone: 0.29 legs, 1.5 deg,
knee 97.7 (94-98), 6 come-and-goes - and her centre of mass at her heels'
edge the constraint left (0.32 legs at the text's 0.75 rad lean); risen from
the deeper seat she fell at 31.2 s where she stood in 0.94. Her rise needs
the seat's slide forward first (the kernel's Step 4): undone till then.

Her bed the suite's way (`house.py --one 0.32 1.01 1.37 lie_bed --said`): the
supine sleep's dock, 70 mm, fell her off the mattress's edge in the settle 3
of 3 - the bed soft, rigid, raised to the chair's 0.47 m - where the chair's
85 mm held, and a chair lowered to the bed's 0.44 held (neck 213 deg/s).
Docked at 85 the side-lie (side_lie.lisp) sat her and laid her nowhere: her
list and turn modes at their reach, 0.9 and 1.2 rad, tilt her pelvis against
her standing legs; at rest her soles bore all of her, 305 N, the bed 0, her
pelvis 0.64 m up, her neck swung 1 363 deg/s. Her gym's text (motion.lisp)
kept: arms 535 deg/s, trunk 279, neck 834 against her walk's 174, 88, 110. A
roll of her base onto her trochanter with her legs off the floor is no mode
of hers (Step 4).

## The wayfinder

`machine.wayfinder`: her trunk twisted through the gap her way meets 0.6 s on
among what her high band meets; its backward coupling (her flow bent by the
rows a shield holds against the room, Sherman-Morrison at eta 18.5) waits on
such rows.

- Her chest 364 mm across her upper arms' skin, 194 deep: a gap at 0.8 of it
  takes 73 deg of twist; 0.99, 57 (past twice atan(depth / across)); under
  0.3, the text's most (75).
- Test 1's door (`tools/sim/doorway.py`): the lodge's door locked ajar at
  46.2 deg, 291 mm (0.8 of her shoulders) between its edge and the far post,
  her exit said from 1.3 m inside: through at 3.0 s, no bump, her waist
  twisted 28 deg 0.15 m past the door's line and 51 deg at 0.29 m, her arms
  55 mm clear of it at the line.

## Her drives' back-EMF bounds

`machine.bemf`, the text's quadrant law on her link's volts, her boards' own.
On 48 V motoring falls under her law's clamps past 9.2 rad/s (hip roll), 14.7
(hip, knee), 16.8 (ankle), 24-26 (shoulder, elbow, neck); braking keeps its
peak. Her walk never reaches it. Her law each pass (`machine.shrunk`,
wbc_step.c, wbc_shrunk.c): her ask shrunk as one to her clamps as before,
each drive clipped to these bounds.

- Bounded at each drive's nominal peak, her war emergency power was taken
  (63 N m at 3.02 s walking): her walk 2.50 -> 2.54 off a woman's. The peak
  is her law's own limit.
- Her ask shrunk as one into these bounds: a fast joint's motoring bound took
  her whole ask down and felled her charged run leap's landing (6.64 s), where
  her base and the clip stand; her step has no box a drive to trade in.

## The discrete phase margin

`machine.margin`, `tools/sim/margin.py`: the text's lock-in at 15, 22, 30 and
38 Hz on her loop broken at a drive's torque, standing, 0.5 N m. The four at
once beat 7-8 Hz apart through its 250 ms leak, 9 % of each in the others':
each frequency in its turn, 1.5 s. A known loop (a PD on an inertia, its
torque 3 ms late) read within 0.11 dB and 1.7 deg.

| loop | 15 Hz | 22 Hz | 30 Hz | 38 Hz |
| --- | --- | --- | --- | --- |
| ankle | -7.9 dB, 1 deg | -0.6, 164 | -5.0, -177 | -7.8, -174 |
| knee | -17.0, 149 | -22.5, 25 | -23.7, 25 | -25.2, 25 |
| hip | -21.4, 65 | -26.0, 52 | -26.0, 22 | -26.9, 14 |
| shoulder | -26.3, 92 | -30.1, 87 | -36.0, 73 | -40.5, 70 |
| spine | -15.5, 122 | -21.5, 50 | -25.5, 35 | -29.2, 27 |
| hip walking, gearboxes in | +9.0, -115 | +2.7, -125 | -0.7, 174 | -9.5, -116 |
| hip walking, rigid | -21.6, -78 | -20.0, -124 | +0.1, 161 | -2.3, 161 |
| ankle walking, gearboxes in | -7.4, -178 | +7.1, -131 | +0.2, 159 | -4.1, 170 |
| knee walking, gearboxes in | -9.1, -25 | -6.1, -160 | +1.2, 167 | -10.1, -127 |
| shoulder walking, gearboxes in | -0.1, -111 | -5.7, -151 | -11.0, -159 | -20.3, -166 |
| spine walking, gearboxes in | -6.2, -34 | -4.0, -109 | -3.7, -176 | -0.8, 136 |

- Rigid, every loop past -3 dB keeps its margin; her ankle's phase turns
  through 180 deg between 15 and 22 Hz, its gain margin there 7.9 dB.
- Her gearboxes in (`physics.BOXED`): her hip's loop at 22 Hz +3.9 dB with
  24.8 deg - the text's rollback, K_d and gamma to 0.65. At 1 N m the same
  hip +0.2 dB, 88.8 deg: her gearboxes' play, the loop's gain rising as the
  probe sinks into it.
- Walking (`--walk`, her walk told after the settling): her hip's loop
  crosses 0 dB at 30 Hz with -3 to -8 deg of margin, gearboxes in, read
  alike at 0.25, 0.5 and 1 N m (her gait's own torques would scale it);
  rigid, +0.1 dB at 161 deg there and -2.3 at 161 at 38 Hz. The text's
  rollback is live as she walks: K_d 0.35, gamma 0.50.
- Every leg loop walking crosses 0 dB at 30 Hz with its phase past 180
  (ankle -21 deg, knee -13, hip -6), her spine at 38 Hz (-45); her shoulder
  keeps 14 deg or more. One feature in all her legs' loops, rigid too: not
  her gearboxes' resonance.

## The lowering

`machine.lowering`: a gait folded into a bank of the text's ABI (wbc_soa.h, 2 176
bytes on 34 lines) over her nominal gait's, its fields' offsets held by
`static_assert`s; the room's schedule four banks.

- The text writes every field at `offset + 4 joint`: a node's track (0x80)
  would land on the stride (0x84) and the clearance (0x88). Each node's only
  where her nominal gait says a field of several (`dc-offset-rad`).
- The thorax's 8 Hz low-pass at 1 kHz in float32: its DC gain 1.6e-5 over
  unity, its rest wandering 1.1e-6 at 0.3 over a second. Seeded on the
  double coefficients it drifted 4e-6 from its seed; on the float32 ones its
  loop runs, its first output its rest.
- Swapped settled at 0.3: seeded, a step of 8.6e-7; unseeded, 0.2998 - back
  within 1 % after 130 ms.
- A bank rides her link's table as its words, bit for bit; her loop's
  biquad in C (wbc_soa.c) Python's bit for bit.

## Her right-invariant EKF

`machine.inekf` beside her estimate: SE_2+2(3), her IMU's mean over each law
pass carrying it, each sole's point under its insoles' centre of pressure -
foot and toes by their loads - measured last pass and re-laid each, weighted
by the observer's share of her on it, the text's Mahalanobis gate (11.34) a
slide (`tools/sim/drifted.py`; m and m/s, estimate / EKF):

| scene | went | floor drift | height drift | speed p90 |
| --- | --- | --- | --- | --- |
| walk | 7.16 | 0.060 / 0.039 | +0.041 / +0.004 | 0.026 / 0.006 |
| turns | 3.81 | 0.038 / 0.021 | +0.049 / +0.022 | 0.025 / 0.006 |
| moves | 3.88 | 0.037 / 0.019 | +0.080 / +0.043 | 0.028 / 0.007 |
| strafes | 0.19 | 0.010 / 0.003 | +0.039 / +0.026 | 0.013 / 0.006 |

- Its soles' points walked as the slip gate's 0.08 m/s: it took her IMU over
  her soles, 1.02 m off in 13.5 m. Gated on 0.24 mm a pass, every contact a
  slide: 1.41 m. Her IMU read at the pass's end, not its mean: 0.25 m/s off.
- The foot's insole alone: her pelvis sank 0.39 m in 13.5 m as her soles
  rolled onto her toes.
- Her eyes' innovation (`visual-innovation-law`) is not in it.
