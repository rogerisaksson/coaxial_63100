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
