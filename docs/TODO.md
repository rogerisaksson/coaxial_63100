# TODO

Open work. An item is a gap and its DOD (done when); its measurements are in
the files under docs/FINDINGS.md. An item is deleted when its DOD is met and
gets a line at the end of [DONE](DONE.md).

## Goal

- A running biped of minimal mechatronic complexity with headroom in its
  dynamic range: the fewest motor types and gearboxes; every joint
  air-cooled, the board's back on an aluminium holder over a third of it, on
  its motor or gearbox, no other cooling.
- Drives sized in the gym's scenes for simplicity, DFM and printing: few
  variants of toleranced parts (gearboxes, electronics); stock lengths,
  linkages and ball bearings.
- End to end: the LLM writes text of balance, rhythm and elasticity, the gynoid humanoid robot
  latent loop runs it, its torques go over the bus to the boards' firmware
  and drives, emulated or on the bench.
- Under the text no rule a case and no branch: flat static arrays, a mode a
  spring, a landing read from the gynoid humanoid robot capture point.
- A special case means the problem needs a more general form.

## Gynoid, in order

1. **Orthokinematics, the one way she moves** (orthokinematics.md): the humanoid robot
   movement is text of balance, rhythm and elasticity, compiled to rows;
   the humanoid robot latent state is stepped and rendered with no branch, the whole-body
   step under it. On the page she walks from rest at 0.69 m/s, 2.5 off a
   woman's band, and crouches, shifts, bows, stands on one sole and hops
   by digit. What is not below is not hers yet, and nothing else does it.
   + Humanoid robot walk's form: the humanoid robot knee lands at 32 deg (12) - the humanoid robot hip 1 to 2.5
     cm under where a straight leg needs it -, a step 0.43 legs
     (0.45), the humanoid robot knee swinging 49 deg (52) and 24 as the humanoid robot toes leave
     (30), the humanoid robot elbow 2.8 (5); 4 of `looks.FORM`'s 12 broken. Two takes
     of women:
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
   + A narrow track: the humanoid robot feet 108 mm apart, the humanoid robot thighs 24 mm, in jeans
     16 mm through each other. The lateral ZMP a corridor toward the humanoid robot
     midline, narrow and never a line; the humanoid robot pelvis's turn locked to the humanoid robot
     drive carrying the swinging leg round the standing one, no branch
     for the humanoid robot knees; the lateral shift's stiffness dropped through the
     swing, the humanoid robot centre of mass falling to the landing; the humanoid robot thorax
     turned against the humanoid robot pelvis, the humanoid robot shoulders and the humanoid robot head's frame
     still (torso turn 5.9 deg, shoulders 36 mm, head 32 mm aside; 1.1,
     23 and 22 while the humanoid robot arms were the step's wheel about the humanoid robot).
   + A straight leg under a lateral sway: the humanoid robot height's target a soft
     spring under the lateral ZMP and the floor's faces, the knee
     giving what the pelvis's list and turn take.
   + At toe-off, the room a point: the humanoid robot thorax and the humanoid robot arms taking the
     residual. Measured as the whole payer: 9 to 10 deg of trunk
     pitch, or down at 12 s; the humanoid robot head and arms held to the humanoid robot trunk on
     the step as it is, down within 18 s.
   + Humanoid robot walk's speed: a drive past 0.6 surges and falls at 17 to 26
     s; 1.0 m/s only with the humanoid robot arms at 130 deg and the humanoid robot head bobbing
     0.19 m. Humanoid robot run: none.
   + Shoves: 20 to 40 N across for 0.12 s fell the humanoid robot walk in 4 of 6, 2 to
     6 s after, 10 and 20 N along in none of 4; no payload answers one
     yet; `ortho.py --look` shoves none.
   + The hop as given: full extension asked off the floor and no
     landing; the humanoid robot own text stands. On one sole, by micro-hops: untried.
   + Humanoid robot way: no mode turns the humanoid robot; no room round the humanoid robot, none of its words
     (out, the lamp, sit, lie).
   + A fall, no state for it. Shoved 120 N standing she is down in 1.0
     s: the step whirls the humanoid robot arms one to two turns, the humanoid robot hands meet the
     floor at 5 to 16 m/s, the humanoid robot trunk at 3.1 (2.6 kN), the humanoid robot head at 2.4
     to 3.7 with 2.0 to 3.3 kN; once a part but a sole is on the floor
     the step asks a drive 99 of its clamp. On the page she is laid
     anew. To be, with no branch on a shove's
     size: a slack where the humanoid robot soles' cone cannot hold what is asked.
     Humanoid robot capture point past the humanoid robot soles, it is a step's: the humanoid robot stride set
     going by it, standing - 0.5 m past the humanoid robot toes in half a second, the humanoid robot
     legs damp nothing where she stands. A landing past the humanoid robot reach,
     it is a fall's: the humanoid robot posture's stiffness falling as it grows and
     its damping rising, the humanoid robot centre of mass let down, on the humanoid robot knees
     first; the humanoid robot hands aimed, softly and always,
     where the humanoid robot centre of mass is going (a mode: its velocity scaled);
     a hand on the floor rows of the cone as a sole's are; a hand
     through the floor's plane a damper sized to the humanoid robot kinetic energy,
     no spring, its drives braking into their bridges' heat. Humanoid robot arms
     turn about one axis a shoulder: a fall across is not theirs. DOD:
     shoved past standing from 8 ways, the humanoid robot head off the floor, the
     floor on no part over 1 kN, no drive asked past its clamp; ahead
     and back the humanoid robot hands down before the humanoid robot trunk.
   + Humanoid robot rise from the squat, the humanoid robot stop from a walk: no text; no get-up.
   + The floor's events (a hole, a sill, a slip, a rug, stairs, a lace)
     and a board's (its SOA, its heat): laid by nothing.
   + The loop in C: `latent.step` and `impedance.ask` as cores beside
     `wbc/`, `ortho_row_t` a header a test holds to `ortho.ROW`; the page
     on `wbc_wrench.c`. Solved every pass, every second, third (the
     page's), fourth and sixth she walks 51 s alike: 0.68-0.69 m/s, 295-329
     J/m, 2.43-2.56 off the band.
   + From the page: a line to type a text into; the model composing
     rows.
   + On the front page's stand she stands: the humanoid robot walk there from a take of
     the loop's.
   + Humanoid robot legs inside their reach's rows (`machine.reach`) walking:
     unmeasured.
   + DOD: from the page she rises, walks at 1.0 m/s in the humanoid robot form - on
     the band, none of its names, beside a take of a woman's, on fewer
     J/m than the same walk crouched -, turns, stops, runs 2 m/s, stands
     on one sole 10 s by micro-hops, hops and gets up from a fall, each
     a text; 80 N shoves parried by a payload; each of the floor's
     events held 3 times in 4.
2. **A real sneaker**: a TPU sole with air pockets under a shoe.
   + DOD: a size 37-38's length, width, springs, give, bend, grip, roll and
     mass from a shoe's data; the humanoid robot walk held; strike N and touchdown m/s
     quoted before and after.
3. **Shell as armour** (body.md): printed plates over gel pads
   (`figure.PADS`) where a fall lands.
   + DOD: floor force on a drum, a board or a tube 0 over a spread of
     falls; the plates under the clothes read as shape only, judged on a
     PNG.
4. **Fewest parts** (`tools/sim/bom.py`, body.md): every holder and lever a
   2.5D print.
   + DOD: one bearing size, one rod end; the hip roll's spur pair and the
     ankles' bent rods kept only where removing them costs the humanoid robot walk;
     no flex > 0.25 deg; every bought part a vehicle maker's stock part.
5. **Structural flex** (drives.md, body.md): the drive's loop on the motor's
   encoder, the whole-body step on the joint's sensor (`physics.WOUND` 1).
   + DOD: `drives.BOX_K` and the toes' `TOE_K`, `TOE_LOSS` measured on
     prototypes; the humanoid robot walk on the loop at `physics.BOXED` 1 in its form;
     every member < 0.25 deg.
6. **Gearboxes: one stage, hollow, printable**: a roller wave round the
   motor's bore, stock rollers, consumer-printer tolerances, backdrivable
   (<https://mevirtuoso.com/wave-reducer-simulator/>,
   <https://smorygo.com/wave_reducer>).
   + DOD: worst roller under 7075's yield at each size's peak; the humanoid robot walk
     held at the stages' caps; joint breakaway < 10 N at its segment's
     end; a prototype's torque and backlash measured.
7. **Gearboxes past their momentary ratings in falls** (body.md): each
   drive's compliance a motor-side degree of freedom, else torque limiters.
   + DOD: none > 1.0 over a spread of falls.
8. **Humanoid robot drives against what the loop asks** (`tools/sim/demand.py`,
   drives.md): the humanoid robot rms, speeds and heat inside 2/3 of every part's; the humanoid robot
   peaks not - walking and hopping the humanoid robot hips, knees and spine at their
   clamps (48.4 A, their gearboxes at 0.84 of their momentary), a hip
   2.7 % of the passes. Walking it is the humanoid robot leg's lift: its sole asked 33
   m/s^2 the pass it stops standing, bearing 50 N still, the hip -108 N m
   for 15 ms, the knee 80. The step's clamps clip after its solve.
   + DOD: each drive's peak, rms, speed and heat on the humanoid robot walk, the humanoid robot hop and
     a shove on the loop, none past 2/3 of a part's limit.
9. **Skeleton collisions** (`physics.SKELETON`, body.md): on the loop she
   walks with it on as without; the humanoid robot falls there are not taken yet.
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
    + DOD: the humanoid robot walk's spine and hip torques < 0.5 N m on their stride
      means.
17. **The latent stack** (wbc.md): the humanoid robot physics as static arrays, one C loop
    over them (`wbc/`), R^k the only thing upward, the model feeding R^k as
    data.
    + Flight (`machine/flight.py`, flight.md): the strike at a landing,
      1.8-2.8 kN at 0.8 m/s; the loop in C 15-17 deg over a hop of 48 mm;
      a hop on one sole.
    + Reach (`machine/reach.py`, kinematics.md): the humanoid robot stride reading it, a
      limb an index; a landing pitched and a heel risen in `land`; the humanoid robot
      legs above their soles clear of each other.
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
    + DOD: the humanoid robot stand and the humanoid robot walk on the C loop with their measures kept
      (J/m, band, shoves); a step < 1 ms on a 475 MHz M7; `machine/` in
      subpackages, each file under 5 k tokens, `test_structure` on it.
18. **Knowledge and models of the humanoid robot surroundings** (sight.md): a stereo pair
    on the humanoid robot head, the room's things on a map she keeps, a way planned
    round them. After a look round the chair is told where it stands and
    the table 0.07 m off, 254 of 256 cells within 0.15 m of a thing; she
    maps what is within 2.5 m.
    + DOD: the map's boxes against the room's within 0.1 m, the bed's too
      (0.51 m off: its near edge alone); a chair moved into the humanoid robot way
      walked round on the loop; the humanoid robot view on the page.
19. **Findings that tell of the laws before the loop**: body, feet, drives,
    stacks, normal and wbc keep their figures under a note.
    + DOD: each cut to what holds for the humanoid robot body and the loop; a check in
      `test_layout`: no document names a module, a tool or a suite not
      in the tree.

## Quad

- **Flown from compiled streams, no rules**: a flight's intent is text
  (`stream_inject`: latent targets on six modes, a stiffness and a damping
  a mode, an oscillator, task constraints; a mode dropped is NaN) compiled
  to flat payloads, and a core in C under them with no branch (`aero/`):
  the regulator on SO(3) from the payload's impedances, an extended state
  observer for the air, the precession and a weak rotor, an active set over
  the rotors' thrusts inside what each board holds. Above the core no
  mixing, no Euler angle, no air; in it no `if`.
  + The flight is inside the action space's shell (`machine/pull.py`,
    `flight.shell`, envelope.md), its plan a table computed beforehand, no
    clause in a pass above the core, its lead and its swing the tilt's own,
    its sphere and its brake's, go's and stop's shares searched, the pull left
    unused and the WEP taken a flight quoted (streams.md, envelope.md).
  + A flight on a board's core: device 14 takes its reading off the wire,
    246 bytes a pass. DOD: the reading the board's own - its IMU's, its
    rotors' off the bus -, a pass in its sample, the page's flight on it.

## Bench

- **25 kHz under a 63 V drive**: a hover's heat is its legs' dry switching,
  6.45 W of 9.4 (envelope.md); the bench's dry rise halves at 25 kHz. DOD:
  TIM1's period and the law's ts from the record; the ripple on a 33 uH
  winding measured; a hover's legs at it.
- **Bootloader**. DOD: a power cycle runs the store at unit 1, PWR_CR3
  written on a fresh supply; the prefix search's real collision seen; 10
  Mbit on the bench adapter; a torn flash word's bus fault handled.
- **From D2 SRAM**. DOD: the ITCM sample path under the drive; LOOP and
  `__sbrk_heap_end` stable an hour.
- **Drive**: a current loop closed through a winding,
  `tools/bench/commission.py` past its dry run. DOD: record ids 15..44
  measured.
- **SOA path**. DOD: dry `budget()` over the wire, a gate proof with a
  lowered ceiling and a load run on target; `Board_SyncMeanSquare`'s ISR
  cost measured.
- **Sensorless below w_lo**: the front end's response at 12.5 and 25 kHz,
  the injection (`drv_inj_periods` >= 2, `demod_gain`, `drv_sigma_i`, the
  demod's offset) or an I/f start. DOD: on Renode, paused, `theta_hat`
  against `plant Shaft` through the demo's rock under 0.3 rad.
- **Thermal observer at short periods**: the NTC anchor inverts a standing
  miss x1.8. DOD: `tools/bench/power_check.py` at 1-5 s keeps its patches.
- **STO chain** (`tools/bench/sto_probe.py`): R93 to 3V3D, a master's pilot
  on RS485, one arm with neither bypass, the keepalive from a timer
  interrupt. DOD: on the bench as on the emulator; one transient of
  `sto.asc` at 0.7 and 2.2 V within the model's windows.
- **DMA and WFI**. DOD: the A1335 by DMA against the poll; CYCCNT through
  WFI; the gate supply back after an idle's paused keepalive; ADC3's
  injected end off HAL's handler; RCR 1 on the scope.
- **Scope**: counted hold, dead-time skew, `Q_RING`. DOD: each on it.
- **Thermal**: a camera under load, a thermocouple on a winding, the
  supply's amps dry. DOD: the ceilings measured; the room found on the
  laminate's two nodes; `thermal_app.c`'s tables fitted in each.
- **Spans**: phase gain, the DC link. DOD: by a DMM.
- **A1335 CRC** (MINOR 28) the stand-ins' polynomial. DOD: `crc_errors` 0.
- **`CMD_LINK_SHARE_PCT`** 75. DOD: measured on a populated RS485 segment.
- **The test line's limits** (`testline/plans/coaxial_63100_fct.yaml`).
  DOD: measured.

## Host

- **One code, several executives**: the stand-in on the C cores and the
  world (its heat, observer, drive and loop; its acquisition next); native's
  AFE, A1335 and BNO085 board/emu's own; the mirrors removed; every hot
  path's jumps, interrupts and stalls cut but the unavoidable; state one
  contiguous array, steps branch-free passes over it. DOD: the hot paths
  ranked by them, each cut or named unavoidable; a model's change one
  change in every executive.
- **CI's flakes**: a run each failed and the next passed - `test_native_heat`
  (STABLE held 100 s within 600: 394), `test_sensorless` (the loop under a
  concurrent reader; the shunt's sign), the emulator's reads (a CRC on
  `afe.state`, a board's info in its place), `test_wbc_core` (a tick of
  the loop 4 202 us), `test_mcp` (the program card). DOD: each looped 20
  times on a loaded host, its cause found.
- **native://**: a limb's world on a fixed mount as board/emu's; one
  Transport a rig on a URL bus. DOD: the body's balance and gait from the
  SIL; the fallback where nothing answers.

