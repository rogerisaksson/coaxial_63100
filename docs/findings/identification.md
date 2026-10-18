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

- A draft beside this one (`host/machine/sysid.py` of 2430ad8, no suite) hung
  her in a harness of springs (`World.hang`) and read her pelvis's rows too;
  hung, 7 contacts on her gave 25-32 N m at the hips that no sensor reads.
  Superseded; its DOD is docs/TODO.md's item 24.
- On the stand (`mjcf.STAND`: her pelvis welded 1.3 m up, `solref` 0.005):
  every drive swung at once met herself - feet on feet 417 steps of 12 000,
  forearm on thigh 372, shank on thigh 290. A limb at a time
  (`campaign.STAGES`), its joints swung 35 % of their range 5 % from either stop
  and the unlimited 0.5 rad, the hip's roll abducted, her arms held out before
  her (`campaign.POSE`, shoulders flexed 90 deg): 0 steps touched in 24 s.
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
- Hung in a harness (`World.hang`) where the stand would hold her, a robot's
  rig: her pelvis 79 mm down on it, bouncing at 1.8 Hz under her limbs; 48
  s, 0 steps touched, residual 0.24 N m rms a drive (the stand's 0.28), 110
  base parameters trusted (101), 70 within 10 % (69). 237 base parameters
  either way: the bounce tells no mass from its first moment at the base's
  1 %. A load cell in the harness (`World.pull`, `sysid.hung`: its force on
  her and its moment about her pelvis's origin, six rows beside the drives')
  adds her pelvis's ten: 247 base parameters, 115 trusted, 83 within 10 %,
  her mass 31.142 for 31.143 kg; the force rows' residual 0.07 N m at the 0.1
  m lever, the moment rows' 0.3-0.57.

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
- A world built 15 % off the figure, every link's mass, centre and inertia
  and every drive's armature and friction drawn apart: Coulomb read back
  within 0.6 % at the median, 10.4 % at worst; a knee's and an ankle's
  armature within a few %, viscous 11.5 % at the median (0.3 N m s/rad is
  little torque); 107 base parameters trusted, 61 within 10 % of the world's.
  The correction laid over the build: a fresh campaign's residual 0.60 ->
  0.36 N m rms a drive, the world's own model 0.30 (the friction band, the
  hems); the trunk's rows 1.19 and 2.9 -> 0.28 and 0.18.
- The world with its gearboxes (`physics.BOXED`): the rotors ring at their
  mode (187 rad/s at a knee) through the campaign, in the motor's encoder and
  the torque that drives it; the armature read off its second differences
  came out negative on 7 drives and the links' residual 0.3-2.8 N m. The
  arrays low-passed both ways at 10 Hz (`campaign.smoothed`): residual 0.05-0.42,
  every armature positive, the hips' within 3 %, the rest 3-46 % off.
- Two of the world's own under it: the box's wind-up integrated against the
  link's start-of-step speed where MuJoCo moves the link at its end's - dt qd
  on every wind-up, 2 mrad of a 4.4 mrad play at 2 rad/s, and the link's
  jerk on the rotor's second difference -, and its damping and mesh against
  that stale speed, (c + mesh / 0.02) dt qdd, 0.13 kg m^2 at the waist. Gone
  (`gearbox.moved`, the step's middle's speed): the gearbox's play test
  crosses in 89 ms where 142.
- The rotor's inertia on the steps inside the play, where the box passes
  nothing: the motor angle's second difference is its torque's mean over the
  two steps about it, over J (`campaign.boxes`): within 1.2 % on all 21
  drives. Each box with that rotor: the wind-up within 7 % and the play
  within 1 % on 17 drives, the knees' and elbows' wind-up 5-13 %, the
  ankles' 28-35 % low and their play 12 % (their rotors ring across the
  play, the lowest mode, 93 rad/s); the damping 10-40 % low. The mesh's
  friction is not read: it acts through its rounding's slope, mesh / 0.02
  rad/s, 5 times the damping, and the ring swings the sliding across the
  rounding within a step. Not kept: the mesh from the steps sliding past 0.04
  rad/s - within 10 % at the hips and the neck, 60-70 % off at the knees,
  nothing at the arms; the rotor's inertia fitted with the box on the wound
  rows - the fit wandering as the rows are chosen anew by it; the box read
  on arrays low-passed at 3-20 Hz, every wind-up 20-90 % low: the wind-up's
  signal is the ring's size.

## On her walk

- Her walk on the gym's world 15 % off the figure, 0.69 m/s for 9 s, her
  edge arrays and her drives' torques a step: a row is read where the model
  holds (`sysid.free`) - no joint of the limb within 2 deg of a stop, nothing
  touching its skin from the sample before to the one after. Against the
  world's own model the trunk's rows read 0.02-0.03 N m rms, a hip's 1.3-2.3
  and a knee's 0.9-1.2: the edge's differences (the same rows off the world's
  own state 0.3), a swinging hip's row 0 within its swing and 2-6 N m the
  sample its sole landed, before the guard. The arms' rows fall to the elbow's
  stop, 2.7 deg flexed.
- Judged on the walk's second half, N m rms a drive in the mean: the build
  0.53, the world's own 0.35; corrected on the stand's campaign alone 0.62 -
  her hips' and knees' 1.8 and 1.3 -> 3.1 and 2.4: her pelvis still on the
  stand, masses and first moments never stand apart -; on the walk's first
  half alone 0.44; on both 0.42, the trunk's 0.47, 0.45, 0.11 -> 0.30, 0.17,
  0.03, no drive's up by more than 0.02 (`test_her_model_holds_on_her_walk`).

## Standing on her insoles

- Her pelvis's six rows (`sysid.regressor`'s last: the force on her, world, and
  the moment about her pelvis's origin in its frame) are MuJoCo's inverse
  dynamics to 3e-13 N m at 20 random states. Standing on her law and swaying
  (#202) 8 s, her insoles' pressures along their soles' normals at their
  centres of pressure against them, shear-free about the point under her
  pelvis at her insoles' height (`sysid.borne`): her weight 305.2 for 305.5 N,
  0.034 N m rms apart at a 0.1 m lever, the moments 0.13 and 0.45 N m rms
  apart on swings of 2 and 18.
- Those three rows beside the stand's campaign add her pelvis's own to the base
  parameters, 237 -> 239: her mass 31.14 kg to 0.02 %, and a first moment of her
  pelvis (hz -0.137 for -0.147 kg m, 6 % of itself), which her drives' rows never
  show.

## On her boards

A drive on its board, through the wire alone (`coaxial.control.identify`,
`tools/bench/sysid_drives.py`, tests/test_native_sysid.py): held as a stepper
at 2 A, swung by the board's own loop (device 12, a Direct regulator on
`theta`, rows of 10 ms), its DAQ streaming two phase currents and the A1335
in one stamped record at 100 Hz; the rotor frame's q current times kt the
torque, `sysid.motor` the least squares for J, b, f and the gravity on the
shaft.

- The leg's four boards on `native://` (`humanoid_leg`, 36:1 joints), 16 s
  each, against the world's own J, b and m g arm / gear:

| drive | J kg m^2 | b N m s | gravity N m |
| --- | --- | --- | --- |
| hip | 2.37e-4 for 2.13e-4 | 1.35e-3 for 1.19e-3 | 0.339 for 0.327 |
| knee | 1.77e-4 for 1.59e-4 | 1.10e-3 for 1.03e-3 | 0.200 for 0.191 |
| ankle | 1.25e-4 for 1.24e-4 | 1.03e-3 for 0.88e-3 | 0.032 for 0.026 |
| foot | 1.09e-4 for 1.21e-4 | 1.05e-3 for 0.84e-3 | 0.0074 for 0.0065 |

- The torque left over 0.12 to 0.24 of the torque's rms; the Coulomb term
  under 1.5e-4 N m where the world has none. Seven campaigns on the hip read
  J 1.87 to 2.39e-4, b 1.35 to 1.42e-3, gravity 0.339 to 0.355.
- A record's mean current turns with the shaft: the A1335 is taken at the
  window's middle (half a window before its snapshot) and half the A1335's
  poll period later (the snapshot is the last poll, 0.23 ms old in the
  mean), the vector grown back by the sine over its turn; each tone at one
  speed, the first's swing 1.2 rad, the 2.9 Hz tone the inertia's (without
  it the hip's J read 1.65e-4). Half a window off, b read -6e-3; the poll
  left out, +20 to +30 %.
- Not kept: the log ring (device 5, angle and drive at the link's share)
  overflows at this host's 12 ops a second to the native limb, its records
  bursts; single-sweep records are the PWM's ripple, J 2e-5.
- A profile loaded whole on a native rig lays its model on the world's plant,
  and the emulator clears the load's inertia and damping with a motor's j or
  b laid live: the hip's J read 1.2e-4, the bench flywheel rang at 30 Hz.
  The record takes the profile's drive part alone.
- The current vector's angle from two phases and the A1335 agree to 0.26 rad
  el rms on the hip's swing: the hold's lag, 0.3 rad el at 2.9 Hz, is what is
  read; the A1335's 12 bits are 0.088 deg.
