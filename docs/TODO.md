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
   + Now (2026-10-11, look.py from the squat, the armada): the walk as
     built lands at 26-30 deg, 0 parries over its starts; the law's at
     12.2, 485 J/m, 0.20 off the band, stride 0.89 m. The crouched manner
     (strut 18) measures 390 for three faults in the driving, a leg a
     stride (straight.md): the knee landed at 15 deg and driven to 5 under
     load (+37 J/m), the trailing knee held stiff through the other foot's
     landing with no push-off before it (+25), the swing's knee braked by
     its servo into its landing (+26); the standing twentieths alike.
   + Next: the knee landed straight and yielding under load, the trailing
     ankle's push-off before the other foot lands, the swing let go to its
     pendulum; each measured on the same table.
   + DOD: parries per spread <= 1 on any host; clean starts, standing and
     from the squat, over a spread of side gains, roll peak 0.27; speed
     within 5 % of plan at 1.0 strides/s; `looks.FORM` landing knee 15 deg;
     test_gynoid_gait.py asserts fewer J/m than the same walk crouched to
     12, 18, 24 and 30 deg at each pace; stance knee 4.5 deg or on its stop;
     stride >= 1.0 m.
2. **Running** (`machine/runner.py`, run.md).
   + Now: 0.72-1.65 m/s held. Open: 2.0 m/s, the foot sliding at landing,
     turns.
   + DOD: 2 m/s standing on the scoreboard; no strike > 2 kN; the biped
     suite's parries, falls and get-ups as walking.
3. **A real sneaker**: a TPU sole with air pockets under a shoe.
   + DOD: a size 37-38's length, width, springs, give, bend, grip, roll and
     mass from a shoe's data; scoreboard held; strike N and touchdown m/s
     quoted before and after.
4. **Shell as armour**: printed plates over gel pads (`figure.PADS`) where a
   fall lands.
   + Now (`gait_montecarlo`, 2026-10-11): the shove's three falls 0.00 kN
     on a drum, a board or a tube, the hands' landing 4.4.
   + DOD: floor force on a drum, a board or a tube 0 in the falls suite;
     the plates under the clothes read as shape only, judged on a PNG.
5. **Fewest parts** (`tools/sim/bom.py`): every holder and lever a 2.5D
   print.
   + Now (`bom.py`, 2026-10-11): 21 part types; one bearing size, one rod
     end, tubes of 7 stock sizes; the spur pair and the bent rods in.
   + DOD: one bearing size, one rod end; the hip roll's spur pair and the
     ankles' bent rods kept only where removing them costs the scoreboard;
     no flex > 0.25 deg; every bought part a vehicle maker's stock part.
6. **Structural flex**: the drive's loop on the motor's encoder, the
   whole-body law on the joint's sensor (`physics.WOUND` 1, drives.md); on
   the gearboxes as bodies the walk on setpoints stands but drags its toes,
   the loop on its torques walks. 2 of 61 members flex past 0.25 deg at
   0.6 of the peak (`members.py`, 2026-10-11): the shank's first run 0.29,
   the spine roll's horn 0.26.
   + DOD: `drives.BOX_K` and the toes' `TOE_K`, `TOE_LOSS` measured on
     prototypes; the page's walk on the loop at `physics.BOXED` 1 on its
     form; every member < 0.25 deg.
7. **Gearboxes: one stage, hollow, printable**: a roller wave round the
    motor's bore, stock rollers, consumer-printer tolerances, backdrivable
    (<https://mevirtuoso.com/wave-reducer-simulator/>,
    <https://smorygo.com/wave_reducer>).
    + DOD: worst roller under 7075's yield at each size's peak; scoreboard
      held at the stages' caps; joint breakaway < 10 N at its segment's
      end; a prototype's torque and backlash measured.
8. **Gearboxes past their momentary ratings in falls**: each drive's
   compliance a motor-side degree of freedom, else torque limiters.
   + Now (P's shove on the walk as built, 2026-10-11): 3 of 21 past it
     over the walk, the fall and the get-up: the left hip 1.53, the right
     1.41, the left hip yaw 1.20; the spine 0.99, a knee 0.97.
   + DOD: none > 1.0 in the scoreboard's falls.
9. **Arms down while getting up**: hands on the knees, or more hip torque;
   decision open.
   + Now (P's shove on the walk as built, 2026-10-11): the hands 0.86 m
     from the torso at the farthest through the lift and the crouch, thrown
     at 6.4-7.3 m/s.
   + DOD: the decision; 5 of 5 hot; hands within 620 mm of the chest
     through the lift.
10. **Get-up end**: rocking before standing, feet split fore and aft.
    + Now (2026-10-11): the feet 178 mm apart fore and aft as she first
      stands, 168 as she walks again.
    + DOD: split rms 0.25; feet 40-80 mm apart.
11. **No abrupt get-up moves**.
    + Now (2026-10-11): 9 joints of 27 past 400 deg/s (the elbows 1 190,
      the hips 640, the sprung toes 2 000), the feet 2.9 and 3.2 m/s;
      walking again 22.9 s after the fall.
    + DOD: every joint < 400 deg/s, every foot < 2 m/s; fall to walking <
      15 s; the loop at 1.0.
12. **Falls past saving**.
    + Now (`gait_montecarlo --suite all`, 2026-10-11): the shove's three
      falls land 4.4 kN, the head 0.00; of 16 crouch falls the head on the
      floor in 2, at 0.7-0.9 m/s (balance.md, 2026-10-04).
    + DOD: a hand and a knee take the fall, called 0.3 s before the floor;
      the body drawn in; head < 2 kN.
13. **Scoreboard events**.
    + Now (`gait_montecarlo --suite all`, 2026-10-11): 88.5 % held; rises
      and walks 100 % (1.00 off its form, knee behind plumb 10.6); sill,
      slip, hot, nudge 100 %, rug 79, hole 77, soa 76, lace 31 (26 on
      2026-10-04, balance.md); the 120 N shoves 25 % walking, 44-46
      standing; the stand's nudges, bricks, boards and rockers 100; stairs
      not in it.
    + DOD: each > 75 %, the 120 N shoves and the lace among them, a slope
      and a tilt among the events; the 1.0 walk standing; the sill's
      spread the page's; stairs climbed; 8 halts of 8.
14. **Skeleton collisions** (`physics.SKELETON`).
    + Now (2026-10-11): at 1 she falls in the rise from the squat and gets
      up into it again, never walking; at 0, P's shove's fall puts the head
      0.08 m over the floor, on it.
    + DOD: head on the floor in 0 of 64 falls.
15. **Obstacle contacts**: toes into the sill, fingers into the floor at
    MuJoCo's 0.02 s; overlap or force, decision open.
    + DOD: the decision; the set measured on it.
16. **Carbon shells and clothes** over the structure as built.
    + Now (`fit.py`, 2026-10-11): standing, the elbow 20, the shoulder 15
      and the hip yaw 6 mm past the clothes; over 45 poses 8 pairs overlap
      13-19 mm - the pelvis's frame and the hip roll's drum rolling onto
      her front, the arm bones and the ankles' rods sitting back.
    + DOD: `fit.py` 0 mm past them in every pose; seat soft; jeans a coarse
      cloth < 0.5 ms a step; no seam on a PNG.
17. **Boards on their buses** (`machine.buses`).
    + DOD: the walk on emulated boards end to end.
18. **Planner server** after LOCAL_TRIES local failures.
    + DOD: measured on a fall.
19. **MuJoCo Warp**: the scoreboard's worlds batched on the GPU.
    + DOD: its step measured against the CPU's 0.31 ms first.
20. **One going law, the page's own** (going.md). Met by measure: its walk
    on `looks.FORM`, 0.20 off `normal.BAND` at 485 J/m where the walk as
    built draws 623 at 0.90 m/s; the keys 12 of 12, the presses 66 of 72;
    shoved, standing as the walk as built. The page's default stays the
    walk as built till the law's is approved by eye.
    + DOD: the law the page's default; presses 72 of 72.
21. **Tubes, corners and flanges**: carbon tubes epoxied into printed
    corners; a gearbox on a flange; motor, box and board outermost; a
    grille or a heat sink on the FETs; a quick-release per limb.
    + DOD: no pair closer than before; each tube in its holder; each board
      on its flange; each quick-release's give modelled; a shoulder stop.
22. **A dance from the move set**: on the toes and tripping, down, a turn
    with the skirt swinging, seated legs crossed, up again.
    + DOD: twice through on the page, no fall.
23. **Movement language** (`normal.WORDS`, `gaits.MANNERS`).
    + DOD: tripping without a shuffle; 100 % of a take's measure; a walk
      into 20 N of wind with the asked accent; sitting, rising.
24. **Lumbar curve, no holding torque for the pose**.
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
25. **Docs as technical documents**: no attributions, no narrative; tables
    and numbers. Now: docs/, the READMEs and the code comments done but 7
    in arrival.py, graphics/gynoid.py and humanoid_keys.py (`grep "the
    user"`).
    + DOD: docs/, the READMEs and the code comments outside vendor code.
26. **The latent stack** (wbc.md, the Python stack before it): her
    physics as static arrays, one C
    loop over them (`wbc/`), R^k the only thing upward, the model feeding R^k
    as data.
    + Loop: 120 N shoves (1 of 8 where the stack 3); the tick 1.7 ms here,
      26.8 M instructions on the image (device 13, wbc.md): 56 ms on the
      M7 against the DOD's 1 - the formulation's cost, not its memory.
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
      (J/m, band, polar); a step < 1 ms on a 475 MHz M7; `machine/` in
      subpackages, each file under 5 k tokens, `test_structure` on it.

27. **The room's words** (room.md): the law turns (turns.md: walking
    0.3 rad/s, standing 0.5 stepping round); out walks through the door;
    test_room.py holds the three.
    + Lamp: her hand placed at a point - the arm's two joints solved for the
      switch from the pelvis as read, or the reach as she walks the last
      0.4 m: the arc from a set angle crosses the switch's 8 cm only
      0.39-0.44 m off it, the halt scatters her 0.1-0.3 m.
    + Sit and lie: a step back to the seat, or the pelvis let down on the
      law; stepping round wants 0.5 m clear, the keyframes reach 0.45 m
      back; lain along the bed from its foot she tumbled.
    + DOD: U, I, E, Y from the page, each done and up again, in
      test_room.py.

28. **Knowledge and models of her surroundings** (sight.md; she walked into
    the door post on the page, 2026-10-11): classical image analysis and
    stereo vision - a camera pair on her head rendered from the world,
    disparity to depth, the depth classified into the room's things (walls,
    door, furniture) as boxes on a map she keeps, and her errands' ways
    planned round them; the world's own geometry read by nothing of hers. On
    the page a picture in the viewport's corner: her eyes' view, each thing she
    has told hulled and labelled in its kind's colour.
    + Now: the stereo within 1.7 % of the renderer's depth at the median; 93 %
      of the map's cells within 0.15 m of the room's things after a look
      round, the chair told 0.16 m off, the table 0.05; out through the door
      0.30-0.41 m from the post both ways the page asks it; the picture on
      the page.
    + DOD: the map's boxes against the room's within 0.1 m; out through the
      door clearing its posts by 0.25 m, 10 of 10 from anywhere in the room;
      a chair moved into her way walked round; on the page.

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
