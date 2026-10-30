# Findings: drives

Motors, gearboxes, inverters and the numbers that size them
(`machine.body.drives`). The body's findings:
[body](body.md); the board's: [FINDINGS](../FINDINGS.md); one stack for
every drive, its demand and the candidates: [stacks](stacks.md); each
drive's demand on her loop and the stunts against it: [demand](demand.md).

## Assemblies

Three assembly classes, a board behind an outrunner on a cycloid, coaxial:

| Size | Build | Peak | Joints |
| --- | --- | --- | --- |
| L | the 63100 board (parts 92 x 93 mm, a 100 mm disc) on the 5230SL at 1:64; 791 deg/s at 44.4 V, 1.5 kg, 100 x 95 mm | 251 N m | hips, knees, spine pitch and roll, ankles (inside the calf) |
| M | 50 A on a 43 mm stator at 1:76, estimated from its size class | 101 N m | - |
| S | 20 A on a 35 mm stator at 1:101, estimated from its size class | 30 N m | - |

- Every joint within its peak (the hips' 250 at 250.7), its rms within what
  its envelope holds indefinitely (the hip 61 of 73.7 N m, the board bolted to
  its housing), its speed within the unloaded (the knee 616 of 791 deg/s);
  the assemblies within their segments' masses (the shank 2.15 of 2.65 kg).
- The cycloid shows the rotor at 0.49 kg m^2 at an L joint; the model carried
  0.05.
- The frame fixes the product of that inertia and the copper's heat at a
  torque (J N^2 against t^2 / Km^2 N^2): the hips' winding 83 C at 1:64, 129
  at 1:48, 210 at 1:36.
- Drawn, the ankle's drum on the calf's back stood 3 cm proud; inside a calf
  112 mm round only its ends show, as the knee's and the elbow's do.

## Cranks

Running on cranks, not reversing: each rotor's reflected inertia
given and taken twice a stride, 1/2 J w^2 at 1.4 strides/s, the hip +-40 deg,
the knee 10-120, the ankle +-25 at J 0.163 kg m^2: 8.6, 16.2 and 3.4 W a
leg, 56 W both, against 445 W of copper walking at 0.85. Not worth a crank's
mass and fixed range.

## Winding

L wound 1.25 times its turns: 137 -> 171 N m at 100 A, 1520 ->
1216 deg/s. The hips at the clamp 3.4-3.6 % of walking either way, the knees
7.2-7.5 -> 6.1-6.8 %, copper 524 -> 606 W.

## Reducers

- A wobbling-plate (nutation) ball reducer against the planar ball stage on
  L's 61 mm ball circle: held by a Cardan or a Rzeppa, one
  stage gets the same balls a ratio (1:12 11 of 12.9 mm, 1:36 4.1 mm);
  compounded, its two faces 6 and 6, 1:36 on six 20 mm balls but 42-59 %
  efficient through the difference, near locking backdriven. Kept: the
  planetary into the ball stage, 0.95 x 0.97.
- Ball stages for the wave reducer tools: one eccentric, ring
  fixed, cage out at lobes:1; the eccentric's race a stock bearing's outside;
  its eccentricity 80 % of the largest whose ball path stays a ball round
  plus 0.15 mm; pressure angle mean 13-15, at most 21-22 deg.

| Size | Balls | Race | Ball circle | e | Web | Lobes | Ring | Housing |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| L | 11 x 12 mm | 6005 | 59 mm | 0.92 mm | 4.9 mm | 12 | 69.2-72.8 mm | 80 |
| M | 11 x 8 mm | 6805 | 45 mm | 0.77 mm | 4.9 mm | 12 | 51.5-54.5 mm | 60 |
| S | 9 x 7 mm | 6900 | 29 mm | 0.55 mm | 3.1 mm | 10 | 34.9-37.1 mm | 42 |

- M on 9 mm balls left its ring 1.75 mm of wall in 60; S on 8 mm 0.4 in 42.
- A ball on the 6005's cylindrical outside at L's 171 N m carries 4.66 kN,
  p0 7.46 GPa against ISO 76's static 4.2; at 40 N m 4.6: a grooved race or
  rollers in line contact.

## One gearbox

Each outrunner and KV picked per joint for 1.5x on the walk's torque at its
speed and on its rms copper (48 V):

- The hip and knee set the box: 257 N m momentary, 81 mm; their 51 N m rms at
  a fifth of it, 92 mm. At 96 mm round 12 drives stood out of the skin
  (shoulder +36 mm, hip yaw +29, ankle and wrist +23, knee +22), the ankle
  pair's drums 24 mm into each other.
- Inside one box KV and frame met 1.5x from the wrist (MN3508 KV380) to the
  hip (U12 II KV60 at 1:24, x1.74, rms x2.1), but the knee: x1.45 at best (U12
  II wound KV90, 1:30), its rise at 171 N m and first step 144 N m at 704
  deg/s on 100 A.
- Km, R line to line halved: 5230SL 0.205 (582 g), Hobbywing M8108 85KV
  0.229 (270 g), M8110 95KV 0.276 (315 g), U12 II KV60 0.581 (803 g).
- The walk asks 3.1 N m/kg at the hip and knee, their drives' clip.

## Dimensionless sizing

At the box's output, 1.5x:

- Power number 1.5 max(tau w) / (eta sqrt3/2 V I) picks the inverter: a KV
  exists under 1, its window 1/P wide. The knee 0.80 on 100 A, the hip 0.37,
  the rest under 0.4 on 25 A.
- Power-rate number (1.5 T_rms)^2 J_rotor / ((eta T_cont)^2 J_load), the
  ratio gone, picks the frame: the knee asks 36 kW/s of T_cont^2 / J_rotor; a
  107 mm U12 II gives 27, its rotor heavy; a 72 x 26 mm stator 40.
- One ratio then lies between each joint's heat's least and inertia's most.

## Coaxial stacks and frames

- Frames 68 x 30 and 60 x 16 mm, boxes 84, 64 and 44 mm, inverters 100 mm
  100 A and 70 mm 50 A, 1:30.
- The knee's continuous torque is the inverter's: at 68 x 30 the winding
  binds at 76 N m (41 A); at 68 x 45 the 100 mm inverter's switches at 82 (44
  A) through its 3.6 K/W housing path. A longer or wider frame buys nothing
  past it; J worsens (1.08-1.25).
- A six-bar for the knee, 360 deg of crank to 170 of shank: 43 close at 33
  deg transmission, the best x1.09 on torque at speed against direct drive's
  x1.12 at the same winding freedom; its lever 0.63-0.89 over 15-45 deg where
  the knee asks its 171 N m and 1024 deg/s. No torque from it, only the dead
  centres.
- Hip roll on a crank-rocker (`linkage.PLANAR`): the drive on the fork behind,
  48 mm from the roll axis; a 14 mm crank turning 360 deg, a 45 mm rod, a
  25 mm horn; dead centres at -28 and +40 deg; lever 1.8 at 0, 2.2 at -14,
  2.1 at +27, 3.8 at -24; transmission 49 deg; the rod 7.7 kN at 1.5x. The
  linkage lives between the hip stack's end and the fork's arm: the stack's
  100 mm inverter rim cut every rod at 34 mm from the pitch axis, so the
  stack went 20 mm in on its axis (16 mm into the pelvis boom squatting),
  the roll's bearings 5 mm out; in front the drum stood 68 mm out of the skin.
  Neither stroke kept: the spur (1.67, no stops) stays.
- Electronics in oil, each stator on its inverter through a thermal
  interface (`drives.COOLING` 0.3 on the windings' and the laminates'
  K/W, assumed): every heat number under 1 (the hips' and knees' 1.94 ->
  0.64, the hip roll's 2.92 -> 0.88, the ankle's 0.24), the switches
  binding the A joints.
- Across the elbow's 56 mm the stack fits on the joint's own axis, -1 mm
  inside the shell, where a 68 mm stack needed a bevel pair and a belt: both
  gone, the wrist's too (held); the elbow's inverter a ring round the humerus
  at mid-arm facing along it (0 mm). The humerus ends at a collar on the
  elbow's motor; the forearm hangs from one on its gearbox by a strut to
  its axis (`skeleton.HUNG`). Parts: 28 -> 27 types (`tools/sim/bom.py`).

## A printed box's play

The gearboxes as bodies of their own (`machine.body.gearbox`, `physics.BOXED`,
play and hysteresis as a good printed box has them):
each rotor runs free across the play, winds the box and the structure up past
it against the mesh's friction, and the link is turned by that wind-up
alone; the rotor's inertia off the joint's armature; the rotors stepped five
times a world step (at one the knee's 187 rad/s mode outgrew its damping).
The numbers, estimated: 0.5 deg of play at the output (a wave drive's few
arcmin before), the roller box's 9 kN m/rad as before (at a printed cycloid's
1.5 the stance ankle sagged 3-4 deg under its 60 N m), the mesh's friction
2 % of the drive's peak, the wind-up's damping ratio 0.3; at the knee 4.4 mrad,
6.2 kN m/rad, 2.2 N m on 0.177 kg m^2. Fed 3 N m the waist's rotor crosses its
play in 23 ms with the link still to 2e-9 rad, winds up 6.3 mrad (6.5 by the
numbers), and reversed meets the other flank 85 ms later before the link turns
back (`tests/test_gynoid.py`).

- A loop closed on the joint's angle through the box fell standing in 0.6 s
  (800 N m/rad on a 1.4 kN m/rad spring with the rotor behind it). The servo
  and the observer read the motor's encoder (`physics.WOUND` 1), as a board
  does; the whole-body law the joint's. The observer on the joint took the
  play for 84 N m of disturbance and she fell standing.
- The wind-up alone loses the ankle 2 x 60 / 6 200 = 1.2 deg at each torque
  reversal, the play 0.5 more.
- The loop on its torques walks the boxes: with its PD 2.52 m at 0.42 m/s
  and 265 J/m, with its observer 2.17 m at 0.36 m/s and 326 J/m (rigid 2.31
  and 265, 1.94 and 325), and the observer stands 8 of 8 at 100 N where the
  PD 2 and the stack 7 (`tests/test_wbc_core.py`, `tools/sim/wbc.py`).
- The drives' speed, back-EMF and copper are the rotor's; the boards see the
  rotor's angle past the joint's on the block (`delta`, `delta_d`, `ahead`,
  `gearbox.Boxes`).

## As data

- Her build's parts as files (`machine/library`, `machine/humanoids`): every
  drive's kt, peak, armature, speed, drag, shock, ohm, mass, length and heat
  the same to the bit for all 27 joints; `docs/DIMENSIONS.md` unchanged;
  `drives.py` 5.8 k -> 4.7 k tokens.
- Her manifest of them reads her 31.14 kg (305.5 N), her centre of mass 0.916 m
  up, omega 3.27/s: her text's header had said 55 kg.
