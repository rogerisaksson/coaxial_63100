# Findings: her kinematics

The lowest level's geometry: the figure's recipes (`machine/figure.py` -
a leg's joints from its ankle back to its hip, a strut's reach) and what
might stand in their place. Her going's are in [going](going.md).

- Dual quaternions in the recipes' place (2026-10-06; the user: a higher
  algebra as the method, not a recipe a limb; a prototype outside git,
  build/prototypes/motors.py). A pose a motor - a unit dual quaternion,
  the even part of 3D projective geometric algebra -, a joint a screw
  about a line of `figure.SEGMENTS`, forward kinematics their product,
  inverse one damped least-squares solve on the twist between two
  motors: no trig by limb, no branch of an atan2. Forward against
  `figure.foot_of`, 2 000 poses: 1e-15 m. Inverse against `figure.leg`
  from 0.05 rad off: 2.3e-6 rad, 5.5 passes meaned. A call, pure Python:
  the recipe 8 us, the chain of motors 43, a pass of the solve 109 and
  482 to 1e-9. Near the straight knee, 6 deg, the pelvis asked a mm down
  and a mm up: the recipe turns the knee 2.4 and 4.6 deg, and straight
  past its reach; the solve at a damping of 1e-3 1.5 and 1.4 deg, the
  hip left 0.4-0.6 mm off. In the law for her standing legs, warm from
  her last setpoints, 2 passes: her walk the same - 500-507 J/m where
  501, 0.19-0.25 off a woman's band where 0.21, stood, walked and stood
  10 of 10 - and on the page's keys' way down, 46 s in, 0 of 12 where
  11: warm, with no posture to come back to, it drifts. So: the method
  for what has no recipe - her arms, a hand on a knee, a seat, several
  contacts at once -, with a posture term and a compiled core before it
  takes her legs'; no cure of its own for her walk's draw or her
  balance. Conformal algebra's spheres and circles would say a strut's
  reach and its heel's rise as meets: a square root each as they stand,
  not worth its 32 numbers.
- Her jeans' seams blended as motors (2026-10-06; the user: Kavan's
  skinning with dual quaternions; a prototype outside git,
  build/prototypes/skin.py). A corner within 7 cm of a knee's or a hip's
  pivot shares the frame across it by its height at rest alone, whatever
  part it is of - the shells that overlap there and her body under them
  move as one -, the two frames' motors from rest blended and normed. A
  shin shell's corner off the thigh shell's it lay nearest, the knee at
  60 deg: 78.5 mm at the most (36.5 meaned) as drawn, a stiff shell a
  segment, 24.0 (8.3) blended; at 120 deg 136.7 (72.7) and 48.4 (12.7).
  The thigh shell's ring at the hip, the hip turned out 90 deg: its 690
  mm kept, where the two moved points blended linearly leave 488. On
  close PNGs: at a walk's 40 deg the lit band across the knee and its
  step in her outline gone; sat, the seat's bowl under the thigh gone;
  at 120 deg an ear of some 7 mm on the thigh's front, the shell's rings
  5 cm apart - the method's bulge, where a stiff shell's corner reads as
  a knee. Her jeans alone blended, her stiff body shows through them, 91
  dots of two close PNGs at 90 deg and 445 at 120; blended with them,
  none. A pose of her 9 416 corners 1.2 ms as drawn and 2.9 blended, 1
  536 of them; of the 291 880 dots drawn without a card 7 and 41. Not in
  her drawing (TODO 32).
- Read, the user's two (2026-10-06). Villa-Uriol, Perez-Gracia and
  Kuester, Humanoid synthesis using Clifford algebra (ICRA 2006): a limb
  a serial chain, its kinematics the product of its joints' screws as
  unit dual quaternions off a reference pose - the prototype's chain -,
  equated to a mocap's frames and solved by Levenberg-Marquardt for the
  joints' places, axes and angles at once: 13 joints, 27 degrees of
  freedom, 1.3-6.4 ms a joint on a 1.4 GHz laptop; a ball joint at the
  hip and the shoulder fits a human twice as badly as two axes at the
  knee and the elbow. Here: a take laid on her own joints by that
  residual - what her legs cannot say of a woman's walk, a number a
  joint; her rows read off a take where they were searched. Pitt,
  Hildenbrand, Stelzer and Koch, Inverse kinematics of a humanoid robot
  based on conformal geometric algebra using optimized code generation
  (Humanoids 2008): a leg's joints as the meets of spheres, planes and
  lines, flattened to C by a generator, ten times the multiplications
  and additions of the recipe written by hand (some 200 and 300), and
  one construction a leg's order of joints: a recipe still, in a clearer
  hand. So the motors, with a solve; the conformal algebra to derive a
  closed form where one is wanted.
