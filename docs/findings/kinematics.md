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
