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
