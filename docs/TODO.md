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
9. **Structural flex**: the drive's loop on the motor's encoder, the
   whole-body law on the joint's sensor (`physics.WOUND` 1, drives.md); on
   the gearboxes as bodies the walk on setpoints stands but drags its toes,
   the loop on its torques walks.
   + DOD: `drives.BOX_K` measured on a prototype; the page's walk on the
     loop at `physics.BOXED` 1 on its form; every member < 0.25 deg.
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
    + Now (`gait_montecarlo --suite all`, 2026-10-11): rises and walks 100 %
      (1.00 off its form, knee behind plumb 10.6); sill, slip, hot, nudge
      100 %, rug 79, hole 77, soa 76, lace 31 (26 on 2026-10-04,
      balance.md); the shoves past saving 25-46 %; stairs not in it.
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
24. **Planner server** after LOCAL_TRIES local failures.
    + DOD: measured on a fall.
25. **MuJoCo Warp**: the scoreboard's worlds batched on the GPU.
    + DOD: its step measured against the CPU's 0.31 ms first.
26. **`machine/` in subpackages**, every file < 5 k tokens (`director.py`
    6.0, `arrival.py` 5.6).
    + DOD: `test_structure` on the layout.
27. **One going law, S and F on it** (going.md).
    + DOD: F up to the fastest run and S back on the page, 10 of 10; its
      walk on `looks.FORM` and `normal.BAND` at no more J/m; shoved,
      standing as the walk as built.
28. **Tubes, corners and flanges**: carbon tubes epoxied into printed
    corners; a gearbox on a flange; motor, box and board outermost; a
    grille or a heat sink on the FETs; a quick-release per limb.
    + DOD: no pair closer than before; each tube in its holder; each board
      on its flange.
29. **A dance from the move set**: on the toes and tripping, down, a turn
    with the skirt swinging, seated legs crossed, up again.
    + DOD: twice through on the page, no fall.
30. **Movement language** (`normal.WORDS`, `gaits.MANNERS`).
    + DOD: tripping without a shuffle; 100 % of a take's measure; a walk
      into 20 N of wind with the asked accent; sitting, rising.
31. **Kinematics on dual quaternions** (kinematics.md).
    + DOD: a posture term, a C core; the legs, the arms, a take and the
      jeans' seams on it.
32. **Lumbar curve, no holding torque for the pose**.
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
33. **Docs as technical documents**: no attributions, no narrative; tables
    and numbers. Done: TODO.
    + DOD: docs/, the READMEs and the code comments outside vendor code.
34. **The latent stack** (31; wbc.md, the Python stack before it): her
    physics as static arrays, one C
    loop over them (`wbc/`), R^k the only thing upward, the model feeding R^k
    as data.
    + Loop: 120 N shoves (1 of 8 where the stack 3); the tick 2.1 ms here;
      its state 1 174 KB in doubles where the H753 has 1 MB - a diet
      (floats, a level's rows at a time, the Schur over the active set),
      then its cycles on Renode (wbc.md).
    + Bottom, each board: the ESO (`parts.Eso`, `CTRL_ESO`; on the gearboxes
      8 of 8 at 100 N where the PD 2, drives.md) onto the boards as built,
      reading the motor's encoder: the play's stick-slip there, its torque
      and acceleration registers on the wire.
    + Behaviour: impedance (K, D a task, inputs) and a CPG's phases in R^k
      in balance.py's and going.py's place; dynamic equations, no stages.
      The page's walk onto the loop: on the gearboxes with their play
      (`physics.BOXED`, drives.md) the walk on setpoints drags its toes 7-19
      mm at lift where its form's 2, the loop walks them at 265-326 J/m.
    + Upward: the law's rows as asks - balance point, contacts, clearances,
      turns, hands - and the leg recipes gone; the get-up's words rows of
      asks; the model picks and fills rows (`decide`), never code.
    + Top: the model writes K and the phases, nothing else.
    + Wire: 10 Mbit proven; a feedforward torque register (PROTOCOL MINOR).
    + DOD: the stand and the walk on the C loop with their measures kept
      (J/m, band, polar); a step < 1 ms on a 475 MHz M7.

35. **The law's turn, then the room's words** (room.md): the one law turns
    0.05 rad/s walking and not on the spot - a 10 m curve, no target in the
    6 x 5 m room reached; a turn row (the landing yawed, the pelvis after it)
    measured on the form, then sit, lie, out and the lamp (`machine.errands`,
    keys U I E Y) walked to and done, and tests of each.

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
