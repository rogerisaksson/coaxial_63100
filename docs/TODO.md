# TODO

Open work. An item is a gap and its DOD (done when); its measurements are in
the files under docs/FINDINGS.md. An item is deleted when its DOD is met and
gets a line at the end of [DONE](DONE.md). Subjects off the gynoid's list:
the table under [Subjects](#subjects).

## Goal

- A running biped of minimal mechatronic complexity with headroom in its
  dynamic range: the fewest motor types and gearboxes; every joint
  air-cooled, the board's back on an aluminium holder over a third of it, on
  its motor or gearbox, no other cooling.
- Drives sized in the gym's scenes for simplicity, DFM and printing: few
  variants of toleranced parts (gearboxes, electronics); stock lengths,
  linkages and ball bearings.
- End to end: the LLM writes text of balance, rhythm and elasticity, the gynoid
  humanoid robot latent loop runs it, its torques go over the bus to the boards'
  firmware and drives, emulated or on the bench.
- Under the text no rule a case and no branch: flat static arrays, a mode a
  spring, a landing read from the gynoid humanoid robot capture point.
- A special case means the problem needs a more general form.

## Gynoid, in order

- **A hop falls** (`tools/sim/told.py hop`): said from a stand on her worker
  she lands with her centre of mass 15 cm ahead of her soles - her trunk turned
  on in the air and took them back - and is down at 2.4 s. A second law,
  `flight.shape_hop`, governs it, not her text. Left out, her loop's own hop
  stands from a stand (still at 4.16 s) and the turn's replay at 7 of 7
  wobbles; after a stop, still stepping, the hold lets the load go at a pass
  with no need and she throws on one sole, down at 8.8 s at every wobble. DOD:
  no second law; still within 3 s from a stand and after a stop, 3 of 3
  wobbles; `test_ortho`'s hop to 5 s.
- **Up the stairs on W she fell at the first step** at 0.6 m/s (the page's
  own keys, `test_errands.py`). DOD: climbed on W, a test.
- **HUMANOID freezes as the mouse moves** (reported): not seen here, 300
  frames in its own console under 6 070 sweeps, 37.5 s against 37.3 still.
  A frame past 3 s appends every thread's stack to
  `%LOCALAPPDATA%/coaxial_63100/stalls.txt` (`screen.STALL_LOG`). DOD: seen,
  its stack dumped, fixed.
- **Seated she still falls or does not rise** (CI's test_errands, `F sits her`,
  some runners): here 1 run in 16-48 perturbed. Off the seat she rocks side
  to side, the ZMP her springs ask 0.23 m out (the chair -0.9 um). A sole let
  go hovers 42 mm up, asked -6.8 m/s^2 down and given -0.2: her centre of mass
  asked 2.95 m/s^2 down through the seat outweighs it (W_COM, seed 9). Not
  kept: that ask stopped at the seat (seed 9 still down, the rise 2.50 -> 2.71
  s); a prop kept within 2 mm of where it bore (4 and 6 of 48 risen). DOD:
  64 of 64 perturbed runs seated and risen.

1. **Orthokinematics, the one way she moves** (orthokinematics.md): the
   humanoid robot movement is text of balance, rhythm and elasticity, compiled
   to rows; the humanoid robot latent state is stepped and rendered with no
   branch, the whole-body step under it. On the page she walks from rest at
   0.70 m/s, 2.49 off a woman's band (step 0.48 legs in the band, SO(3) mocap
   GPS 13.82 deg), and crouches, shifts, bows, stands on one sole and hops by
   digit. What is not below is not hers yet, and nothing else does it.
   + Humanoid robot walk's form: the humanoid robot knee lands at 32 deg (12) -
     the humanoid robot hip 1 to 2.5 cm under where a straight leg needs it -,
     the humanoid robot knee swinging 49.4 deg (52) and 20 as the humanoid
     robot toes leave (30), the humanoid robot elbow 2.7 (5); 5 of
     `looks.FORM`'s 12 broken. Two takes of women:
     1.1 m/s, a step 0.8 legs, the knee landing at 2-11 deg, 50-57 as
     the toes leave, 68-73 swinging, the heel 38-57 up.
   + The floor as a contact's wrench cone: a sole's moments and its
     friction coupled faces of its push, never a part clipped alone,
     never a centre of pressure by division. Its room a continuous
     function of the sole's pitch - a soft sole rolls -, narrowing
     along and across as the contact shrinks to an off-centre point
     and never to a line; no change of state coded. As it is: on an
     edge from 1.7 deg of pitch, the room a line there; a part clipped
     alone and pinned in the world's frame, the room across gone with
     the room along.
   + A landing sole's push rising no faster than its foam takes, the
     pendulum's slow fall the time for it: strike 433 N (450), landing
     load 1.40 (1.25), 0.65 after (0.75), no rate bounded.
   + A narrow track: the humanoid robot feet 108 mm apart, the humanoid robot
     thighs 24 mm, in jeans 16 mm through each other. The lateral ZMP a
     corridor toward the humanoid robot midline, narrow and never a line; the
     humanoid robot pelvis's turn locked to the humanoid robot drive carrying
     the swinging leg round the standing one, no branch for the humanoid robot
     knees; the lateral shift's stiffness dropped through the swing, the
     humanoid robot centre of mass falling to the landing; the humanoid robot
     thorax turned against the humanoid robot pelvis, the humanoid robot
     shoulders and the humanoid robot head's frame still (torso turn 5.9 deg,
     shoulders 36 mm, head 32 mm aside; 1.1, 23 and 22 while the humanoid robot
     arms were the step's wheel about the humanoid robot).
   + A straight leg under a lateral sway: the humanoid robot height's target a
     soft spring under the lateral ZMP and the floor's faces, the knee giving
     what the pelvis's list and turn take.
   + At toe-off, the room a point: the humanoid robot thorax and the humanoid
     robot arms taking the residual. Measured as the whole payer: 9 to 10 deg
     of trunk pitch, or down at 12 s; the humanoid robot head and arms held to
     the humanoid robot trunk on the step as it is, down within 18 s.
   + Humanoid robot walk's speed: a drive past 0.6 surges and falls at 17 to 26
     s; 1.0 m/s only with the humanoid robot arms at 130 deg and the humanoid
     robot head bobbing 0.19 m. Humanoid robot run: none.
   + Shoves, hops, stunts and the floor's events: as entries a case each
     (`parry_shove` chosen by the shove's newtons, the chair moved under her
     mid-hop, obstacles found by body id); their general laws are under
     Scenarios below.
   + The loop in C: `latent.step` and `impedance.ask` as cores beside
     `wbc/`, `ortho_row_t` a header a test holds to `ortho.ROW`; the page
     on `wbc_wrench.c`. Solved every pass, every second, third (the
     page's), fourth and sixth she walks 51 s alike: 0.68-0.69 m/s, 295-329
     J/m, 2.43-2.56 off the band.
   + From the page: a line to type a text into; the model composing
     rows.
   + On the front page's stand she stands: the humanoid robot walk there from a
     take of the loop's.
   + Humanoid robot legs inside their reach's rows (`machine.reach`) walking:
     unmeasured.
   + DOD: from the page she rises, walks at 1.0 m/s in the humanoid robot form
     - on the band, none of its names, beside a take of a woman's, on fewer J/m
     than the same walk crouched -, turns, stops, runs 2 m/s, stands on one
     sole 10 s by micro-hops, hops and gets up from a fall, each a text; 80 N
     shoves parried by a payload; each of the floor's events held 3 times in 4.
2. **A real sneaker**: a TPU sole with air pockets under a shoe.
   + DOD: a size 37-38's length, width, springs, give, bend, grip, roll and
     mass from a shoe's data; the humanoid robot walk held; strike N and
     touchdown m/s quoted before and after.
3. **Shell as armour** (body.md): printed plates over gel pads
   (`figure.PADS`) where a fall lands.
   + DOD: floor force on a drum, a board or a tube 0 over a spread of
     falls; the plates under the clothes read as shape only, judged on a
     PNG.
4. **Fewest parts** (`tools/sim/bom.py`, body.md): every holder and lever a
   2.5D print.
   + DOD: one bearing size, one rod end; the hip roll's spur pair and the
     ankles' bent rods kept only where removing them costs the humanoid robot
     walk; no flex > 0.25 deg; every bought part a vehicle maker's stock part.
5. **Structural flex** (drives.md, body.md): the drive's loop on the motor's
   encoder, the whole-body step on the joint's sensor (`physics.WOUND` 1).
   + DOD: `drives.BOX_K` and the toes' `TOE_K`, `TOE_LOSS` measured on
     prototypes; the humanoid robot walk on the loop at `physics.BOXED` 1 in
     its form; every member < 0.25 deg.
6. **Gearboxes: one stage, hollow, printable**: a roller wave round the
   motor's bore, stock rollers, consumer-printer tolerances, backdrivable
   (<https://mevirtuoso.com/wave-reducer-simulator/>,
   <https://smorygo.com/wave_reducer>).
   + DOD: worst roller under 7075's yield at each size's peak; the humanoid
     robot walk held at the stages' caps; joint breakaway < 10 N at its
     segment's end; a prototype's torque and backlash measured.
7. **Gearboxes past their momentary ratings in falls** (body.md): each
   drive's compliance a motor-side degree of freedom, else torque limiters.
   + DOD: none > 1.0 over a spread of falls.
8. **Humanoid robot drives against what the loop asks** (`tools/sim/demand.py`,
   demand.md): the humanoid robot rms, speeds and heat inside 2/3 of every
   part's; the humanoid robot peaks not - walking and hopping the humanoid
   robot hips, knees and spine at their clamps (48.4 A, their gearboxes at 0.84
   of their momentary), a hip 2.7 % of the passes. Walking it is the humanoid
   robot leg's lift: its sole asked 33 m/s^2 the pass it stops standing,
   bearing 50 N still, the hip -108 N m for 15 ms, the knee 80. An ask past
   the clamps the step shrinks as one (jerk.md).
   + DOD: each drive's peak, rms, speed and heat on the humanoid robot walk,
     the humanoid robot hop and a shove on the loop, none past 2/3 of a part's
     limit.
9. **Skeleton collisions** (`physics.SKELETON`, body.md): on the loop she
   walks with it on as without; the humanoid robot falls there are not taken
   yet.
   + DOD: head on the floor in 0 of 64 falls.
10. **Obstacle contacts**: toes into the sill, fingers into the floor at
    MuJoCo's 0.02 s; overlap or force, decision open.
    + DOD: the decision; the set measured on it.
11. **Carbon shells and clothes** over the structure as built (body.md).
    + DOD: `fit.py` 0 mm past them standing, in the squat and over a
      recorded walk; seat soft; jeans a coarse cloth < 0.5 ms a step; no
      seam on a PNG.
12. **Boards on their buses** (`machine.buses`): the loop writes its
    torques into the world, no board hears them.
    + DOD: the humanoid robot walk on the loop on emulated boards end to end, a
      feedforward torque register on the wire (PROTOCOL MINOR), each
      board's heat in the humanoid robot state.
13. **Tubes, corners and flanges**: carbon tubes epoxied into printed
    corners; a gearbox on a flange; motor, box and board outermost; a
    grille or a heat sink on the FETs; a quick-release per limb.
    + DOD: no pair closer than before; each tube in its holder; each board
      on its flange; each quick-release's give modelled; a shoulder stop.
14. **A dance as a text**: on the toes and tripping, down, a turn with the
    skirt swinging, seated legs crossed, up again.
    + DOD: twice through on the page, no fall.
15. **Movement language** (`normal.WORDS`): a word of the eye an accent of
    a text.
    + DOD: tripping without a shuffle; 100 % of a take's measure; a walk
      into 20 N of wind with the asked accent; sitting, rising.
16. **Lumbar curve, no or minimal holding torque for the pose** (kinematics.md).
    + DOD: the humanoid robot walk's spine and hip torques < 0.5 N m on their
      stride means.
17. **The latent stack** (wbc.md): the humanoid robot physics as static arrays,
    one C loop over them (`wbc/`), R^k the only thing upward, the model feeding
    R^k as data.
    + Leap (`machine/leap.py`, leap.md): the strike at a landing,
      1.8-2.8 kN at 0.8 m/s; the loop in C 15-17 deg over a hop of 48 mm;
      a hop on one sole.
    + Reach (`machine/reach.py`, kinematics.md): the humanoid robot stride
      reading it, a limb an index; a landing pitched and a heel risen in
      `land`; the humanoid robot legs above their soles clear of each other.
    + Loop: a tick 24.9 M instructions on the Release image, 52 ms on the
      M7 where the DOD's 1. The step without an active set in C
      (`wbc_wrench.c`, `wbc_cone.c`, wrench.md): 0.25 M instructions
      standing, 0.52 ms; a sole swung 0.34 M, striding 0.35; setting
      out, the rounds beside the stride, 0.44 M, 0.93 ms (device 13 op
      2); on a board, by its own cycles: unmeasured.
    + Bottom, each board: the ESO (`parts.Eso`, `CTRL_ESO`) onto the boards
      as built on the motor's encoder; torque and acceleration registers on
      the wire.
    + Top: the model writes K and the phases, nothing else.
    + DOD: the humanoid robot stand and the humanoid robot walk on the C loop
      with their measures kept (J/m, band, shoves); a step < 1 ms on a 475 MHz
      M7; `machine/` in subpackages, each file under 5 k tokens,
      `test_structure` on it.
18. **Knowledge and models of the humanoid robot surroundings** (sight.md): a
    stereo pair on the humanoid robot head, the room's things on a map she
    keeps, a way planned round them. After a look round the chair is told where
    it stands, the table 0.07 m off and the bed 0.05 m off (0.00 m off when
    moved to (-0.9, 1.2)), 697 of 725 cells within 0.15 m of a thing; she maps
    what is within 3.5 m.
    + DOD: the map's boxes against the room's within 0.1 m; a chair moved into
      the humanoid robot way walked round on the loop; the humanoid robot view
      on the page.
19. **No transition with a jerk**, whatever happens (Scenarios 2,
    `tools/sim/jerk.py`). DOD: every segment's peak through every scenario
    no higher than in her steady walk.
20. **Identification's rest** (identification.md): her drives, links,
    rotors, the boxes' wind-up and play, her walk's rows, her insoles' rows,
    a harness and its load cell read. Left: each box's damping and mesh, the
    ankles' boxes; a link's mass apart from its first moment.
    + DOD: each measured in `test_sysid.py`.

## One command, either driver

- **Told as a dynamic system** (asked): keys, the tour and an agent send the
  same twist and verbs; a command moves her latent targets, the twist her
  rhythm's drive and a verb her modes', and when she moves follows from her
  state: her capture point, her swings, her rhythm's phase and gain. As it is
  the worker's say path is a switchboard of speed gates, delays by name
  (`BRAKE_DELAYS`, `RISES`, `INIT_DELAYS`, `off_c`'s 1.35 s) and gaits picked
  by thresholds: W 2.0 s into her stand after a stop waits 1.35 s, 2.01 s to
  0.3 m/s (0.89, 0.90, 0.67 at 0, 1, 4 s). DOD: no gate, delay or branch by
  name in it; W at any moment of a stop past 0.3 m/s within 1 s; the kept
  replays, the errands and her walk standing.

## Subjects

| Subject | File |
| --- | --- |
| Scenarios, by the general case: each a test in MuJoCo, in order | [todo/scenarios.md](todo/scenarios.md) |
| The bench: what the board has still to show | [todo/bench.md](todo/bench.md) |
| Flight pages: the crafts in wind, the aerobat's landing | [todo/flight.md](todo/flight.md) |
| Host: native://, identification on the robot, her law compiled | [todo/host.md](todo/host.md) |
