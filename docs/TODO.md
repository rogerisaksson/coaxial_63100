# TODO

Open work. An item is a gap and its DOD (done when); its measurements are in
the files under docs/FINDINGS.md. An item is deleted when its DOD is met.

## Goal

- A running biped of minimal mechatronic complexity with headroom in its
  dynamic range: the fewest motor types and gearboxes; every joint
  air-cooled, the board's back on an aluminium holder over a third of it, on
  its motor or gearbox, no other cooling.
- Drives sized in the gym's scenes for simplicity, DFM and printing: few
  variants of toleranced parts (gearboxes, electronics); stock lengths,
  linkages and ball bearings.
- End to end: the LLM picks a prose pattern, the director drives it over
  the bus to the boards' firmware and drives, emulated or on the bench.
- Under the patterns: classical IK (the arrival's keyframes, capture-point
  foot placement) and a learned residual on its setpoints, a table the
  director reads.
- A special case means the problem needs a more general form.

## Gynoid, in order

1. **Walk form, held by test.** Requirement: a low-loss walk rolls over the
   hip, the stance leg extended past plumb. `looks.FORM` in
   test_gynoid_gait.py; smoked on the armada (walk.md, balance.md).
   + Now: landing knee 25-28 deg; a crouched walk costs less; stride 0.75
     m; starts open.
   + DOD: parries per spread <= 1 on any host; clean starts, standing and
     from the squat, over a spread of side gains, roll peak 0.27; speed
     within 5 % of plan at 1.0 strides/s; `looks.FORM` landing knee 15 deg;
     test_gynoid_gait.py asserts fewer J/m than the same walk crouched to
     12, 18, 24 and 30 deg at each pace; stance knee 4.5 deg or on its stop;
     stride >= 1.0 m.
2. **Drives as bought**: off-the-shelf motors, one gearbox, one board,
   air-cooled (docs/DIMENSIONS.md, stacks.md).
   + DOD: every joint's A, P, T and V < 1 on the run and the gym's scenes
     at the settled margin, no joint's J > 1; pack voltage, cooling and each
     joint's ratio fixed in the model.
3. **Balance by micro-steps, as on stilts** (standing.md): a point contact
   under each ball; a quick step toward the capture point whenever it
   leaves, standing and walking alike; one reflex law (`machine.dcm`); a
   small move vocabulary the model picks from.
   + Next: 120 N pushes; stance restored after a step; the trunk thrown
     into the fall; a ready crouch; then a hole, a sill, a slope, a tilt, a
     missing brick.
   + DOD: a 'stand' suite on the scoreboard 100 % standing (nudges, bricks,
     board), point feet and box sole alike; the walk's events plus slope,
     tilt and brick, scoreboard >= its 2026-10-04 value; first stride
     standing with heel and toes moved +-1 cm; the patterns driven over the
     bus to emulated boards.
4. **Running** (`machine/runner.py`, run.md).
   + Now: 0.72-1.65 m/s held. Open: 2.0 m/s, the foot sliding at landing,
     turns.
   + DOD: 2 m/s standing on the scoreboard; no strike > 2 kN; the biped
     suite's parries, falls and get-ups as walking.
5. **Push-off without toe motors**: passive toes, a flexing carbon/TPU
   sandwich (feet.md).
   + DOD: every rise and walk standing; standing knee < 10 deg at
     mid-stance; the knees' T in `drive_sizes` as with driven toes; damping
     from the sandwich's data; toes back within 2 mm at lift; scoreboard >=
     86.4 %.
6. **A real sneaker**: a TPU sole with air pockets under a shoe.
   + DOD: a size 37-38's length, width, springs, give, bend, grip, roll and
     mass from a shoe's data; scoreboard held; strike N and touchdown m/s
     quoted before and after.
7. **Shell as armour**: printed plates over gel pads (`figure.PADS`) where a
   fall lands.
   + DOD: floor force on a drum, a board or a tube 0 in the falls suite;
     the plates under the clothes read as shape only, judged on a PNG.
8. **Fewest parts** (`tools/sim/bom.py`): every holder and lever a 2.5D
   print.
   + DOD: one bearing size, one rod end; the hip roll's spur pair and the
     ankles' bent rods kept only where removing them costs the scoreboard;
     no flex > 0.25 deg; every bought part a vehicle maker's stock part.
9. **Structural flex**: a joint-side sensor on every drive, or the wind-up
   fed forward; decision open.
   + DOD: the decision; `drives.BOX_K` measured on a prototype; the walk
     standing at `physics.WOUND` 1; every member < 0.25 deg.
10. **Gearboxes: one stage, hollow, printable**: a roller wave round the
    motor's bore, stock rollers, consumer-printer tolerances, backdrivable
    (<https://mevirtuoso.com/wave-reducer-simulator/>,
    <https://smorygo.com/wave_reducer>).
    + DOD: worst roller under 7075's yield at each size's peak; scoreboard
      held at the stages' caps; joint breakaway < 10 N at its segment's
      end; a prototype's torque and backlash measured.
11. **Gearboxes past their momentary ratings in falls**: each drive's
    compliance a motor-side degree of freedom, else torque limiters.
    + DOD: none > 1.0 in the scoreboard's falls.
12. **Arms down while getting up**: hands on the knees, or more hip torque;
    decision open.
    + DOD: the decision; 5 of 5 hot; hands within 620 mm of the chest
      through the lift.
13. **Get-up end**: rocking before standing, feet split fore and aft.
    + DOD: split rms 0.25; feet 40-80 mm apart.
14. **No abrupt get-up moves**.
    + DOD: every joint < 400 deg/s, every foot < 2 m/s; fall to walking <
      15 s; the loop at 1.0.
15. **Falls past saving**.
    + DOD: a hand and a knee take the fall, called 0.3 s before the floor;
      the body drawn in; head < 2 kN.
16. **Scoreboard events**.
    + DOD: each > 75 %; the 1.0 walk standing; the sill's spread the
      page's; stairs climbed; 8 halts of 8.
17. **Softer walk**.
    + DOD: strikes < 1000 N; ears < 8 and 40 mm; scoreboard held.
18. **Walk retuned per build**.
    + DOD: the knobs re-searched after each build change, the scoreboard
      quoted.
19. **Skeleton collisions** (`physics.SKELETON`).
    + DOD: head on the floor in 0 of 64 falls.
20. **Obstacle contacts**: toes into the sill, fingers into the floor at
    MuJoCo's 0.02 s; overlap or force, decision open.
    + DOD: the decision; the set measured on it.
21. **Carbon shells and clothes** over the structure as built.
    + DOD: `fit.py` 0 mm past them in every pose; seat soft; jeans a coarse
      cloth < 0.5 ms a step; no seam on a PNG.
22. **Fewer drives**: the fingers a fist (`drives.WAYS`).
    + DOD: their boards removed; the quick-releases' give modelled; a
      shoulder stop.
23. **Boards on their buses** (`machine.buses`).
    + DOD: the walk on emulated boards end to end.
24. **R in the tty crashes it**.
    + DOD: reproduced from the reported traceback, fixed, a test on it.
25. **Planner server** after LOCAL_TRIES local failures.
    + DOD: measured on a fall.
26. **MuJoCo Warp**: the scoreboard's worlds batched on the GPU.
    + DOD: its step measured against the CPU's 0.31 ms first.
27. **`machine/` in subpackages**, every file < 5 k tokens (`director.py`
    6.0, `arrival.py` 5.6).
    + DOD: `test_structure` on the layout.
28. **One going law, S and F on it** (going.md).
    + DOD: F up to the fastest run and S back on the page, 10 of 10; its
      walk on `looks.FORM` and `normal.BAND` at no more J/m; shoved,
      standing as the walk as built.
29. **Tubes, corners and flanges**: carbon tubes epoxied into printed
    corners; a gearbox on a flange; motor, box and board outermost; a
    grille or a heat sink on the FETs; a quick-release per limb.
    + DOD: no pair closer than before; each tube in its holder; each board
      on its flange.
30. **A dance from the move set**: on the toes and tripping, down, a turn
    with the skirt swinging, seated legs crossed, up again.
    + DOD: twice through on the page, no fall.
31. **Movement language** (`normal.WORDS`, `gaits.MANNERS`).
    + DOD: tripping without a shuffle; 100 % of a take's measure; a walk
      into 20 N of wind with the asked accent; sitting, rising.
32. **Kinematics on dual quaternions** (kinematics.md).
    + DOD: a posture term, a C core; the legs, the arms, a take and the
      jeans' seams on it.
33. **Lumbar curve, no holding torque for the pose**.
    + Now, on the one law (`look.py`: torso ahead, pelvis tilt, spine bent,
      mass ahead of hips, spine holds): pelvis tipped 5.2 deg, spine
      straight, upper body 25 mm ahead of the hips, spine holding 2.5 N m;
      the walk as built 0.3 mm, 0.1 N m.
    + Tried (build/wbc/*_hollow.py): the trunk's seat back on the pelvis by
      (0.12 + HIP_DROP) tan 5 deg, the spine back 5 deg, the walk as
      built's pelvis tipped 5 deg as its start's lean decays: spines 0.0-0.7
      N m, mass over the hips, the law 486 -> 439-449 J/m. Reverted: the
      law, placing feet from the hip with the mass ahead of it, walked its
      walk row backward at 0.26 m/s and fell in 4 more checks; 3 falls
      checks failed (head down at 2.7-3.5 m/s, a get-up late).
    + DOD: both walks' spine and hip torques < 0.5 N m on their stride
      means; going and falls suites as on 2026-10-10 (54 of 56, 44 of 44).
34. **Balance as one convex QP per step** (wbc.md): an MPC over a
    whole-body stack. Held: friction cones, drive and joint limits, the
    contacts. Soft: the ZMP in the support, collisions. Priorities by null
    space; WEP where a fall is near. `machine/qp.py`, `wbc.py`, `mpc.py`,
    `balance.py`, `tools/sim/wbc.py`; no buses (`World.feed`).
    + Now, shoved from 8 ways at 3 moments: 60/80/100/120 N stood 24/24/20/16
      of 24 (the law 12/12/2/0 of 12); 3-7 steps where one would do.
    + Now, walk asked 0.5 m/s: 0.31 m/s, 792 J/m, steps 0.16 m (the law
      0.375). Touchdown at 0.39 m/s down, 818 N on the landing sole against
      162 planned, the trailing sole 0-19 N against 122-141; in single
      support the stance sole's load flips heel <-> toe base every 4 ms.
    + Next: a pressure cell under each heel and toe base, the held contact
      and the landing judged from them; a sole rolling on its ball as a
      contact; leg collisions; 1.6-11 ms a step in Python.
    + DOD: 120 N from 8 ways stood, <= 2 steps; 10 m walked at 0.5-1.0 m/s,
      J/m against the law's; the stack over the buses (a torque register,
      PROTOCOL MINOR); a step < 1 ms (a C core: M, the bias and the
      Jacobians from the figure's own screws, MuJoCo the reference).
35. **Docs as technical documents**: no attributions, no narrative; tables
    and numbers. Done: TODO.
    + DOD: docs/, the READMEs and the code comments outside vendor code.
36. **The latent stack** (32, 34; wbc.md): her physics offline into static
    arrays, one C loop over them, R^k the only thing upward, the model
    feeding R^k as data.
    + Offline: `tools/cores/model.py` writes `wbc/inc/wbc_model.h` from the
      compiled figure - a DOF a link (parent, frame, screw, spatial inertia
      with the rotor's), stops, clamps; no logic.
    + Loop: `wbc/`, C11, host-tested against MuJoCo (FK, J, M, bias):
      the contacts' orthogonal decomposition, the tasks by priority in the
      free space, the torques clipped to the polytope level by level; a
      step's cycles on native and Renode.
    + Upward: the law's rows as asks - balance point, contacts, clearances,
      turns, hands - and the leg recipes gone; the get-up's words rows of
      asks; the model picks and fills rows (`decide`), never code.
    + Bottom, each board: an ESO/ADRC `ctrl` part against the FOC loop - the
      drive's own friction, backlash, cogging and drift estimated around the
      loop's predicted acceleration and cancelled, the actuator an ideal
      torque source; the body's dynamics and the contacts are not its
      disturbance. Measured need: creep 5 mm/s on pure torque, an ankle's
      deadband 24 mm (wbc.md); a virtual spring's stick-slip on the boards
      as built, with and without it, the measure.
    + Behaviour: impedance (K, D a task, inputs) and a CPG's phases in R^k
      in balance.py's and going.py's place; dynamic equations, no stages.
    + Top: the model writes K and the phases, nothing else.
    + Wire: 10 Mbit proven; a feedforward torque register (PROTOCOL MINOR).
    + Done: the arrays; the chain's poses, Jacobians, M and bias in C against
      MuJoCo; the loop (`wbc_stack.c`) 0.01 N m off the python stack standing,
      125 us a tick here, stands 3 s and a 60 N shove in MuJoCo
      (test_wbc_core.py); both presets build it at 0 warnings.
    + Done: `parts.Eso` and `CTRL_ESO`, twins, `--inner eso` in the stand;
      standing they see 0.1 N m, no signal (wbc.md).
    + Done: the bounds as inequalities in every level, an active set (`wbc_solve.c`);
      the stand and a 60 N shove held, the walk 4 steps (wbc.md).
    + Next: the soles' corner forces as variables beside the torques, their
      pyramids the bounds, as the python stack has them - its single-support
      answer is 23 N m off the loop's for want of them; then the polar and
      the walk against the stack's, and the observer's worth on the walk.
    + DOD: the stand and the walk on the C loop with their measures kept
      (J/m, band, polar); a step < 1 ms on a 475 MHz M7.

## Bench

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
- **Motion papers on the emulator**: `motion` and `applications` set J and
  load through `drive.model`. DOD: both run on the emulator.
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
  world (its heat and observer since 2026-10-10, the drive next); native's
  AFE, A1335 and BNO085 board/emu's own; the mirrors removed; every hot
  path's jumps, interrupts and stalls cut but the unavoidable; state one
  contiguous array, steps branch-free passes over it. DOD: the hot paths
  ranked by them, each cut or named unavoidable; a model's change one
  change in every executive.
- **native://**: a limb's world on a fixed mount as board/emu's; one
  Transport a rig on a URL bus. DOD: the body's balance and gait from the
  SIL; the fallback where nothing answers.
- **The MCU's die under the drive**: 9 us of sampling in a 20 us period.
  DOD: read.
- **Debug `-O0`**. DOD: `-Og` measured (LOOP, keepalive gap).
- **`heat.py`'s dry loss** 1.2 W, the bench's fit 2.4-4.4 at 24-44 V. DOD:
  it.
- **`coaxial_63020`** has no pin table in `boot_main.c`. DOD: one.
