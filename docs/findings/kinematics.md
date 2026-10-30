# Findings: kinematics

The lowest level's geometry: the figure's recipes (`machine/body/figure.py`: a
leg's joints from its ankle back to its hip, a strut's reach) and what might
replace them.

## Dual quaternions in the recipes' place

A higher algebra as the method, not a recipe a limb; prototype outside git,
build/prototypes/motors.py.

- A pose is a motor (a unit dual quaternion, the even part of 3D projective
  geometric algebra); a joint a screw about a line of `figure.SEGMENTS`;
  forward kinematics their product; inverse one damped least-squares solve
  on the twist between two motors. No trig by limb, no atan2 branch.

| Check | Result |
| --- | --- |
| forward against `figure.foot_of`, 2 000 poses | 1e-15 m |
| inverse against `figure.leg` from 0.05 rad off | 2.3e-6 rad, 5.5 passes mean |
| a call, pure Python | recipe 8 us; chain of motors 43; one solve pass 109; 482 to 1e-9 |
| near the straight knee (6 deg), the pelvis asked 1 mm down and 1 mm up | recipe: the knee turns 2.4 and 4.6 deg, and straight past its reach; solve at damping 1e-3: 1.5 and 1.4 deg, the hip 0.4-0.6 mm off |

- In the law for the standing legs, warm from the last setpoints, 2 passes:
  the walk unchanged (500-507 J/m where 501; 0.19-0.25 off a woman's band
  where 0.21; stood, walked and stood 10 of 10); on the page's keys' way
  down, 46 s in, 0 of 12 where 11: warm, with no posture to return to, it
  drifts.
- Verdict: the method for what has no recipe (the arms, a hand on a knee, a
  seat, several contacts at once), with a posture term and a compiled core
  before it takes the legs'. No cure of its own for the walk's draw or the
  balance.
- Conformal algebra's spheres and circles would state a strut's reach and its
  heel's rise as meets: a square root each as they stand, not worth its 32
  numbers.

## Seams blended as motors

Kavan's skinning with dual quaternions; prototype outside git,
build/prototypes/skin.py.

- A corner within 7 cm of a knee's or a hip's pivot shares the frame across
  it by its height at rest alone, whatever part it belongs to (the shells
  that overlap there and the body under them move as one); the two frames'
  motors from rest blended and normalised.

| Measure | Drawn (stiff shell a segment) | Blended |
| --- | --- | --- |
| shin shell corner off the nearest thigh shell corner, knee 60 deg, max (mean) | 78.5 mm (36.5) | 24.0 (8.3) |
| the same, knee 120 deg | 136.7 (72.7) | 48.4 (12.7) |
| thigh shell ring at the hip, hip turned out 90 deg | - | 690 mm kept; linear blend of two moved points 488 |
| stiff body showing through the jeans, dots on two close PNGs at 90 / 120 deg | 91 / 445, the jeans alone blended | 0, the body blended with them |
| a pose of 9 416 corners | 1.2 ms | 2.9 ms (1 536 blended) |
| of the 291 880 dots drawn without a card | 7 | 41 |

- On close PNGs: at a walk's 40 deg the lit band across the knee and its step
  in the outline gone; seated, the seat's bowl under the thigh gone; at 120
  deg an ear of ~7 mm on the thigh's front, the shell's rings 5 cm apart:
  the method's bulge, where a stiff shell's corner reads as a knee.
- Not in the drawing (the latent stack, docs/TODO.md).

## Two papers read

- Villa-Uriol, Perez-Gracia, Kuester, Humanoid synthesis using Clifford
  algebra (ICRA 2006): a limb a serial chain, its kinematics the product of
  its joints' screws as unit dual quaternions off a reference pose (the
  prototype's chain), equated to a mocap's frames and solved by
  Levenberg-Marquardt for the joints' places, axes and angles at once: 13
  joints, 27 DOF, 1.3-6.4 ms a joint on a 1.4 GHz laptop; a ball joint at the
  hip and the shoulder fits a human twice as badly as two axes at the knee
  and the elbow. Use here: a take laid on the gynoid's own joints by that
  residual (what its legs cannot express of a woman's walk, a number a
  joint); its rows read off a take where they were searched.
- Pitt, Hildenbrand, Stelzer, Koch, Inverse kinematics of a humanoid robot
  based on conformal geometric algebra using optimized code generation
  (Humanoids 2008): a leg's joints as the meets of spheres, planes and lines,
  flattened to C by a generator; ten times the multiplications and additions
  of the hand-written recipe (some 200 and 300); one construction a leg's
  joint order: a recipe still, in a clearer hand.
- So: the motors with a solve; conformal algebra to derive a closed form
  where one is wanted.

## Lumbar curve on the law

No lumbar curve, front-heavy.

| Variant | Upper body ahead of the hips | Spine holds | J/m |
| --- | --- | --- | --- |
| as is: pelvis tipped 5.2 deg by the row's lean, spine straight, trunk 6.3 deg on | 25.2 mm | 2.5 N m | - |
| the walk as built | 0.3 mm | 0.1 N m | - |
| the spine taking the lean out | 9.5 mm | 0.4 N m | 484 |
| plus the trunk's seat 15 mm back on the pelvis | -3.8 mm | 0.6 N m | 439 |

The last walked its walk row backward at 0.26 m/s (docs/TODO.md, lumbar curve).

## A leg's reach over its foot

Static arrays (`machine/body/reach_rows.py`, laid by `tools/cores/reach.py`) read
by `machine/body/reach.py`, a limb an index; tests/test_reach.py holds them.

- A hip's place in its foot's frame is the ankle's roll, the ankle's pitch
  and the knee; the hip's three joints take the pelvis's turn. The pelvis
  level and the foot flat, the hip's yaw is the foot's from the pelvis's
  heading.
- Stops: the figure compiles the knee's (-5..165) and the hip's yaw's (25
  either side); the ankle's are its rods' strokes, -50..45 in pitch and 15
  either side in roll (`linkage.ROLLED`), no stop in the model. The rods'
  dead centre lies further: at -50 in pitch, 43 in roll.

| Hip over its ankle, the foot flat | What bounds it along the toes |
| --- | --- |
| 0.64 m and up | the leg's length alone: a circle, 0.124 m at 0.76 |
| 0.50-0.635 | ahead, the ankle past -50: at 0.60 m the hip out from 0.131 to 0.451 m ahead, a strip under the length beyond |
| under 0.545 | behind, the straight leg leant past the ankle's 45: a strip to 3 mm deep under the length |
| under 0.505 | the hip behind its ankle, 4-99 mm |

- Across, 0.268 of the hip's height either side: 0.20 m at 0.76.
- A flat foot at her walking height reaches 0.124 m: her steps of 0.49 m
  are on a risen heel and a landing toes up (`stride.heels`). `land` lays a
  foot flat; a pitched one is read in its own frame (`over`).
- Rising from the squat and walking, 16 s at 0.85 m/s, her joints ran:
  ankle -41.3..10.8, its roll -0.8..4.2, knee -0.8..128.6, hip's yaw
  -6.7..1.8. Over 9 s every pass inside the rows: 50 mm at the least
  (rising, 1.22 s), the yaw 18.3 deg inside, the leg at its length as she
  stands up (3.63 s).
- The rows, 109 from 0.10 to 0.64 m: none of 60 000 hips they hold is past
  a stop; of what the stops allow 3.5 % is left out (a row's caution, and
  all under 0.10 m: sat on her heels), at the leg's length 23 % (the strip
  beyond the island). 2 768 places landed stand in MuJoCo to 1e-15 m, every
  stopped joint inside its stop; her right is her left mirrored to 0.
- Not in it: her legs above their soles clear of each other (`clear` is the
  soles'), a hip's roll and pitch (no stop), a landing pitched.

## Her movements' orthopaedic terms

Static rows (`machine/body/term_rows.py`, laid by `tools/cores/terms.py`) read by
`machine/body/terms.py`; tests/test_terms.py holds them.

- Anatomy says what a movement does - hip flexion brings the knee forward,
  abduction takes it from the midline, dorsiflexion raises the toes; which
  joint makes it, and with which sign, is found by turning each joint of
  the segment in her figure and keeping the one nine tenths of whose motion
  goes that way. A limb's terms are found on both limbs and agree: a limb
  is an index, its joints' names data.

| Part | Movement, the joint's angle times its sign | Stops, deg |
| --- | --- | --- |
| hip | flexion -hip, abduction +hip_roll, lateral rotation +hip_yaw | rotation 25 either way; flexion and abduction none |
| knee | flexion +knee | 165, extension 5 |
| ankle | dorsiflexion -ankle | 50, plantarflexion 45 |
| foot | eversion +ankle_roll | 15 either way |
| trunk | flexion +spine; toward limb 0's side: lateral flexion -spine_roll, rotation +waist | 85, extension 30; 35; 50 |
| neck, head | flexion +neck; rotation toward limb 0's side +head | 60, extension 45; 80 (the whole-body stack's own) |
| shoulder, elbow | flexion +shoulder, +elbow | none; 160, extension 5 |

- 23 movements; 21 that anatomy names are absent, no driven joint of hers
  making them: her shoulder's abduction and rotation, her forearm's
  pronation, her wrist's (held), her fingers' (held), her toes' (sprung),
  her neck's lateral flexion.
- Held to MuJoCo: 46 asks of 10 deg, either limb, each term and its
  opposite - 0.995 of the motion goes the term's way at the least. The
  leg's own IK (`figure.leg`) bends each joint its term's way, 12 of 12.
