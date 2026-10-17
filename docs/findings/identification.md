# Findings: identification

Her body identified of what a robot of her build measures (`machine.sysid`,
`tests/test_sysid.py`, `tools/sim/sysid.py`): the drives' torques are linear in
each link's ten inertial parameters and each drive's armature, viscous and
Coulomb friction, tau = Y(q, v, a) theta; Y is Newton-Euler down her tree from
her pelvis's IMU and her encoders, only her drives' rows measured. The same
arrays come off MuJoCo and off a robot on its stand, its boards' currents its
torques. What her law senses: [sensing](sensing.md); the drives:
[drives](drives.md).

## The regressor

- Her drives' rows against MuJoCo's inverse dynamics at 40 random states, the
  pelvis anywhere, the loose dofs moving: 5e-14 N m apart of 8. The pelvis's
  frame set still, its specific force the base acceleration: gravity rides in
  the IMU's reading and nowhere else.
- 28 links, 33 hinges, 21 drives: 343 columns. Her pelvis's own ten are zero
  in every drive's row; her hair's and hems', unsensed and at rest in her
  law's model, fold into her head's and shanks'.
- A joint's axes and anchors are MuJoCo's per joint (`xaxis`, `xanchor`), not
  per dof.

## The campaign

- On the stand (`physics.STAND`: her pelvis welded 1.3 m up, `solref` 0.005):
  every drive swung at once met herself - feet on feet 417 steps of 12 000,
  forearm on thigh 372, shank on thigh 290. A limb at a time
  (`sysid.STAGES`), its joints swung 35 % of their range 5 % from either stop
  and the unlimited 0.5 rad, the hip's roll abducted, her arms held out before
  her (`sysid.POSE`, shoulders flexed 90 deg): 0 steps touched in 24 s.
- MuJoCo's `frictionloss` is Coulomb past 0.25 rad/s and a dead band under it:
  read everywhere, a drive's viscous friction came out 0.7-2.1 N m s/rad for
  0.3 and its Coulomb 0.1-0.4 for 1.2-2.2. A drive's row read only where it
  slides faster than `STICK` 0.25 rad/s: Coulomb within 0.5-5 % on the arms,
  the trunk, the knees and the ankles, viscous within 0.02, a knee's armature
  0.162-0.182 for 0.177, an ankle's 0.292-0.304 for 0.293; residual 0.06-0.9
  N m rms a drive.
- A limb's stage of 4 s holds no period of the 0.23 Hz tone: a hip's Coulomb
  friction came out 5-22 % off by the phases drawn, two seeds. At 8 s a limb
  (48 s) the hips' 9-10 % and a hip yaw's 13-14 % at either seed, viscous
  within 0.16 N m s/rad, the residual under 1 N m: the suite's campaign, its
  rows read at 50 Hz.
- The base parameters are the campaign's own: laid from random states with
  the pelvis free, masses stood apart from first moments that on the stand,
  its pelvis still, never do - their fits 1e14 with their doubt. Found in the
  campaign's regressor by pivoted Gram-Schmidt, a column kept at 1 % of
  itself clear, one under 1e-9 of the largest dropped as rounding: 237 of
  343, condition 1e-3; 92 trusted (under 10 % of themselves), 48 within 5 %
  of the model's, 65 within 20.
- Untold on the stand: a hip's yaw armature (its swing 8.75 deg within its
  stops: 3.5-4.8 for 0.177) and its roll's (swung with the whole leg about its
  axis: 1.0-2.6 for 0.49); a foot's and its toes' tensors, the toes' sprung
  hinge barely moving, split between them (hz -0.71 and 0.74 for 0.02 and
  0.002, their sum right).
- 0.5 kg in her left hand: her forearm's first moment -0.031 -> -0.156 kg m,
  read -0.155; her upper arm's -0.271 -> -0.411, read -0.412; her trunk's
  3.585 -> 3.756, read 3.751.

## The corrections

- `identified.correction` moves her build the least that fits the campaign: each
  link's mass and first moment and each drive's armature and friction, least
  squares with a ridge (0.3 of the regressor's mean column) on each move in
  its kind's unit (0.1 kg, 0.02 kg m, 0.01 kg m^2, 0.05 N m s/rad, 0.1 N m);
  a link's inertia about its centre as built, its inertia about its origin
  following its mass and centre, so every link stays a body; no mass under a
  quarter of itself, no drive parameter under none. `identified.save` writes it
  (`machine/robots/<robot>.identified.toml`, `identified.IDENTIFIED`); `wbc.Body`
  lays it over her law's model whole, so the C arrays
  (`tools/cores/model.py`) follow.
- A base parameter is a sum over neighbouring links: laid on one column
  apiece the links' own tensors were no body's (the trunk's principal
  moments 0.044, 0.078, 0.178; a hip roll's -896); laid in part, a hip's
  residual rose 6.5 N m. Priced of each parameter's own size the hand's
  0.5 kg cost 14 units and landed as the trunk's mass and the arms' first
  moments, the other arm's residual up 0.31 N m.
- The 0.5 kg in her hand, on a third campaign: her hand and fingers 0.19 and
  0.22 kg heavier (their columns one, the gripper held), her forearm 0.06; her
  shoulder's residual 3.37 -> 0.05 N m, her elbow's 1.93 -> 0.05, her trunk's
  1.5-4.5 -> 0.1; no other drive's up by more than 0.06.
