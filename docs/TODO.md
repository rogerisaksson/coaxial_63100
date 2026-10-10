# TODO

The aim (the user, 2026-10-04): she runs - a biped of absolute mechatronic
simplicity with headroom in its dynamic range: as few motor types and
gearboxes as can be, her joints air-cooled - a board's back on an aluminium
holder over a third of it, on its motor or gearbox, its only cooling (the
user, 2026-10-10). Sized in the gym for simplicity, DFM and printability:
few variants of the toleranced parts - gearboxes, electronics -; stock
lengths, linkages and ball bearings are cheap. A DOD runs from the LLM to
the metal: a pattern in prose, picked by the model, driven by the director
over the bus to the boards' firmware and drives - on the emulator or the
bench. Under the patterns classical IK - the arrival's keyframes, the
capture point placing each foot - with a learned residual on its setpoints,
a table the director reads. A special case is a sign the problem wants a
more general form. A line is a gap and its DOD (done when); the
measurements live in docs/FINDINGS.md's files; a line goes when its DOD is
met.

## Gynoid, in order

1. **Her walk a human's, held by test** (the user, 2026-10-05: low-loss
   walking rolls with the hip, the stance leg extended on past the plumb
   line). Held by test_gynoid_gait.py (`looks.FORM`), smoked on the armada
   (walk.md, balance.md). Left: her knee lands at 25-28 deg; crouched she
   still walks for less; her stride 0.75 m; her starts. DOD: a spread's
   parries at 1 or under on any host; her starts clean, standing and from
   the squat, over a spread of side gains, the roll's peak at 0.27; her
   speed within 5 % of her plan's at 1.0 strides/s; `looks.FORM`'s landing
   knee at 15; test_gynoid_gait.py asserting fewer J/m than the same walk
   crouched to 12, 18, 24 and 30 deg at each pace; the stance knee at 4.5
   or on its stop; the stride at 1.0 m or over.
2. **Her drives as bought** (the user, 2026-10-05: motors off the shelf,
   one gearbox, one board, air-cooled since 2026-10-10; docs/DIMENSIONS.md,
   stacks.md). DOD: every
   joint's A, P, T and V under 1 on the run and the gym's scenes at the
   settled margin, no joint's J over 1; the pack's volts, the cooling and
   each joint's ratio baked.
3. **Balance by micro-steps, as on stilts** (the user, 2026-10-04): a point
   under each ball, a quick step toward the capture point whenever it
   leaves, standing and walking alike; one reflex law (`machine.dcm`), a
   small vocabulary of moves the model picks from (standing.md). Next: the
   120 N pushes, the stance back up after a step, the trunk thrown into the
   fall, a ready crouch; then a hole, a sill, a slope, a tilt, a brick gone.
   DOD: a 'stand' suite on the scoreboard 100 % standing - the nudges, the
   bricks, the board -; point feet and the box sole alike; the walk's
   events joined by the slope, the tilt and the brick, the scoreboard at or
   over today's; the first stride standing with the heel and toes moved
   +-1 cm; the patterns driven over the bus to the emulated boards.
4. **Running** (`machine/runner.py`, run.md): 0.72-1.65 m/s held. Left: 2.0
   m/s, the foot's slide at landing, her turns. DOD: 2 m/s standing on the
   scoreboard, no strike over 2 kN, the biped suite's parries, falls and
   get-ups held as walking.
5. **The push-off without toe motors** (the user, 2026-10-04: the toes alone
   flex, a carbon and TPU sandwich; feet.md). DOD: every rise and walk
   standing; the standing knee under 10 deg at mid-stance; the knees' T in
   `drive_sizes` as on driven toes; the damping from the sandwich's numbers;
   the toes back at lift under 2 mm, the scoreboard at or over 86.4 %.
6. **A real sneaker** (the user, 2026-10-04: a TPU sole with air pockets
   under a shoe). DOD: a 37-38's length, width, springs, give, bend, grip,
   roll and mass from a shoe's numbers; the scoreboard held; the strike's N
   and the touch's m/s quoted before and after.
7. **Her shell as armour** (the user, 2026-10-04): printed plates over gel
   pads (`figure.PADS`) where a fall lands. DOD: the floor's force on a
   drum, a board or a tube in the falls suite 0; the plates under her
   clothes read as shape only, judged on a PNG.
8. **Fewest parts** (`tools/sim/bom.py`): every holder and lever a 2.5D
   print. DOD: one bearing size, one rod end; the hip roll's spur pair and
   the ankles' bent rods kept only where cutting them costs the scoreboard;
   no flex past 0.25 deg; every part off the shelf as a vehicle maker's.
9. **Her flex** (the user, 2026-10-03): a joint-side sensor on every drive,
   or the wind-up fed forward. DOD: the user's pick; `drives.BOX_K`
   measured on a prototype, the walk standing at `physics.WOUND` 1; every
   member under 0.25 deg.
10. **Gearboxes one stage, hollow, printable** (the user, 2026-10-03): a
   roller wave round the motor's bore, stock rollers, a consumer printer's
   tolerances, backdriven (<https://mevirtuoso.com/wave-reducer-simulator/>,
   <https://smorygo.com/wave_reducer>). DOD: the worst roller under 7075's
   yield at each size's peak; the scoreboard held at the stages' caps; a
   joint's breakaway under 10 N at its segment's end; a prototype's torque
   and backlash measured.
11. **Gearboxes past their momentary ratings** in falls: each drive's
   compliance a motor-side degree of freedom, torque limiters where not.
   DOD: none over 1.0 in the scoreboard's falls.
12. **Arms down getting up** (the user, 2026-10-03): hands on the knees, or
    the hip's torque up. DOD: the user's pick; 5 of 5 hot, the hands under
    620 mm from the chest through the lift.
13. **The get-up's end**: she rocks before she stands, her feet apart fore
    and aft. DOD: the split's rms at 0.25; the feet 40-80 mm apart.
14. **No abrupt moves getting up**. DOD: every joint under 400 deg/s, every
    foot under 2 m/s; under 15 s from the fall to walking; the loop at 1.0.
15. **Past saving**. DOD: a hand and a knee take the fall, called 0.3 s
    before the floor, the body drawn in, the head under 2 kN.
16. **The scoreboard's events**. DOD: each over 75 %, the 1.0 walk standing,
    the sill's spread the page's, the stairs climbed, 8 halts of 8.
17. **A softer walk**. DOD: strikes under 1000 N, the ears under 8 and 40
    mm, the scoreboard held.
18. **Her walk retuned per build**. DOD: the knobs re-searched after each
    build change, the scoreboard quoted.
19. **Her skeleton colliding** (`physics.SKELETON`). DOD: her head on the
    floor in 0 of 64 falls.
20. **The obstacles' contacts**: toes into the sill, fingers into the floor
    at MuJoCo's 0.02 s. DOD: overlap or force, the user's call, the set
    measured on it.
21. **Carbon shells and clothes** over the structure as built. DOD:
    `fit.py` 0 mm past them in every pose; her seat soft; her jeans a coarse
    cloth under 0.5 ms a step; no seam on a PNG.
22. **Fewer drives**: the fingers a fist (`drives.WAYS`). DOD: their boards
    gone with them, the quick-releases' give modelled, a shoulder stop.
23. **Her boards on their buses** (`machine.buses`). DOD: the walk on
    emulated boards end to end.
24. **R in the tty crashes it** (the user, 2026-10-03). DOD: reproduced
    from the user's traceback, fixed, a test on it.
25. **The planner's server** after LOCAL_TRIES local failures. DOD: measured
    on a fall.
26. **MuJoCo Warp** (the user, 2026-10-02): the scoreboard's worlds batched
    on the GPU. DOD: its step against the CPU's 0.31 ms measured first.
27. **`machine/` in subpackages**, every file under 5 k (`director.py` 6.0,
    `arrival.py` 5.6). DOD: `test_structure` on the layout.
28. **One law for her going, S and F on it** (the user, 2026-10-05;
    going.md). DOD: F to her fastest run and S back on the page, 10 of 10;
    its walk on `looks.FORM` and `normal.BAND` at no more J/m; shoved, up
    as the walk as built.
29. **Her tubes, corners and flanges** (the user, 2026-10-05): carbon tubes
    epoxied into printed corners, a gearbox on a flange, motor, box and
    board outermost, a grille or a heat sink on the FETs, a limb a
    quick-release. DOD: no pair closer than before, each tube in its
    holder, each board on its flange.
30. **A dance of her moves** (the user, 2026-10-05): on her toes and
    tripping, down, round with her skirt swishing, sat legs crossed, up
    again. DOD: twice round on the page, no fall.
31. **A language for how she moves** (the user, 2026-10-05, 06;
    `normal.WORDS`, `gaits.MANNERS`). DOD: tripping no shuffle; 100 % a
    take's measure; a walk into 20 N of wind, the accent asked of it;
    sitting, rising.
32. **Her kinematics on dual quaternions** (the user, 2026-10-06;
    kinematics.md). DOD: a posture term, a C core; her legs, her arms, a
    take and her jeans' seams on it.
33. **Her back hollowed, nothing held for the pose** (the user,
    2026-10-10): on the one law her pelvis tips 5.2 deg, her spine
    straight, her upper body 25 mm ahead of her hips, the spine holding
    2.5 N m (`look.py`: torso ahead, pelvis tilt, spine bent, mass ahead
    of hips, spine holds); the walk as built 0.3 mm, 0.1 N m. Tried
    (build/wbc/*_hollow.py): the trunk's seat back on the pelvis by
    (0.12 + HIP_DROP) tan 5 deg, the spine back 5 deg, the walk as
    built's pelvis tipped 5 deg as its start's lean goes - the spines
    0.0-0.7 N m, the mass over her hips, the law 486 -> 439-449 J/m; but
    the law, its feet laid from the hip on her mass ahead of it, went its
    walk row backward at 0.26 m/s and fell in 4 checks more, and 3 falls
    checks failed (the head down at 2.7-3.5 m/s, a get-up late). DOD: both
    walks' spine and hips under 0.5 N m held on their strides' means; the
    going and falls suites as on 2026-10-10 (54 of 56, 44 of 44).
34. **Her balance one convex QP a step** (the user, 2026-10-10: an
    MPC over a whole-body stack, the constraints the design - friction
    cones, the drives' and joints' limits, the contacts held; the ZMP in
    the support, collisions, priorities by null space; WEP where a fall
    is near). `machine/qp.py`, `wbc.py`, `mpc.py`, `balance.py`,
    `tools/sim/wbc.py`, bus-less (`World.feed`). Standing without a
    step: 38 N 8 of 8, 60 N 8, 80 N 4 - the sides, the back diagonals -,
    100 N 2, 120 N 0, where the law with its steps 12 of 12, 12, 12, 2,
    0. Left: a step's sole does not rise, its knee straight (a vertical
    acceleration only to second order; the fold, `balance.FOLD_DEG`);
    the legs' collisions; 2-11 ms a step in Python. DOD: 120 N from 8
    ways stood on steps; 10 m walked at 0.5-1.0 m/s, J/m against the
    law's; the stack over the buses (a torque register, PROTOCOL MINOR);
    a step under 1 ms (a C core).

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

- **Full DOD, the whole** (the user, 2026-10-10): one code, other
  executives: the stand-in on the C cores and the world (its heat and
  observer since 2026-10-10, the drive next), native's AFE, A1335 and
  BNO085 board/emu's own, the mirrors gone; every hot path's jumps,
  interrupts and stalls cut but the unavoidable; state one contiguous
  array, steps branch-free passes over it. DOD: the hot paths ranked by
  them, each cut or named unavoidable; a model's change one change in
  every executive.
- **native://**: a limb's world on a fixed mount as board/emu's; one
  Transport a rig on a URL bus. DOD: the body's balance and gait from the
  SIL; the fallback where nothing answers.
- **The MCU's die under the drive**: 9 us of sampling in a 20 us period.
  DOD: read.
- **Debug `-O0`**. DOD: `-Og` measured (LOOP, keepalive gap).
- **`heat.py`'s dry loss** 1.2 W, the bench's fit 2.4-4.4 at 24-44 V. DOD:
  it.
- **`coaxial_63020`** has no pin table in `boot_main.c`. DOD: one.
