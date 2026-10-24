# Findings: the dual-domain kernel

Its steps on her loop and its tests in the lodge ([protocol](../todo/protocol.md),
host/machine/ortho_kernel/).

## The momentum observer

`machine.observer`, blind: her drives' torques (`edge.TAU`), encoders, IMU and
estimate over her law's M(q), low-passed at the text's 35 Hz, split over the
registry's patches by least squares, each normal pushing. Against her skin
(`tools/sim/observed.py`, each law pass from 1 s; rms N, her weight 306 N;
ghost: the most read where her skin feels under 5 N):

| scene | soles | seat | hips | back | palms |
| --- | --- | --- | --- | --- | --- |
| walk | 22.3 | 4.5, ghost 47 | 1.4, 6 | 1.4, 10 | 7.8, 36 |
| chair, sit and rise | 13.9 | 24.7, 87 | 6.4, 16 | 12.0, 95 | 29.4, 205 |
| bed, lie | 18.6 | 11.2, 94 | 6.6, 44 | 31.6, 8 | 31.4, 85 |
| bed, rise | 15.8 | 9.7, 94 | 5.8, 44 | 25.8, 20 | 25.7, 85 |
| floor mattress | 7.0 | 1.4 | 2.0 | 1.2 | 6.9 |

Seated steady (chair, 3.6-8 s): soles 126-130 N against 126, seat 137-170
against 141-144, the palms a steady 40-45 ghost.

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
