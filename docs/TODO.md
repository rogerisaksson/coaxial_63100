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
- End to end: the LLM picks a prose pattern, the director drives it over
  the bus to the boards' firmware and drives, emulated or on the bench.
- Under the patterns: classical IK (the arrival's keyframes, capture-point
  foot placement) and a learned residual on its setpoints, a table the
  director reads.
- A special case means the problem needs a more general form.

## Gynoid, in order

1. **Walk form, held by test.** A low-loss walk rolls over the hip, the
   stance leg extended past plumb: `looks.FORM` in test_gynoid_gait.py,
   smoked on the armada. The law's straight walk draws 495 J/m where its
   crouched manner 393, at the landings and the swing's end (straight.md).
   + Next: a standing leg solved from the leg as it landed, the height's
     target begun where it was; the knee's damping by its leg's lever in
     stance; each over a spread of starts.
   + DOD: parries per spread <= 1 on any host; clean starts, standing and
     from the squat, over a spread of side gains, roll peak 0.27; speed
     within 5 % of plan at 1.0 strides/s; `looks.FORM` landing knee 15 deg;
     test_gynoid_gait.py asserts fewer J/m than the same walk crouched to
     12, 18, 24 and 30 deg at each pace; stance knee 4.5 deg or on its stop;
     stride >= 1.0 m.
2. **Running** (`machine/runner.py`, run.md): 0.72-1.65 m/s held; 2.0 m/s,
   the foot sliding at landing and turns open.
   + DOD: 2 m/s standing on the scoreboard; no strike > 2 kN; the biped
     suite's parries, falls and get-ups as walking.
3. **A real sneaker**: a TPU sole with air pockets under a shoe.
   + DOD: a size 37-38's length, width, springs, give, bend, grip, roll and
     mass from a shoe's data; scoreboard held; strike N and touchdown m/s
     quoted before and after.
4. **Shell as armour** (body.md): printed plates over gel pads
   (`figure.PADS`) where a fall lands.
   + DOD: floor force on a drum, a board or a tube 0 in the falls suite;
     the plates under the clothes read as shape only, judged on a PNG.
5. **Fewest parts** (`tools/sim/bom.py`, body.md): every holder and lever a
   2.5D print.
   + DOD: one bearing size, one rod end; the hip roll's spur pair and the
     ankles' bent rods kept only where removing them costs the scoreboard;
     no flex > 0.25 deg; every bought part a vehicle maker's stock part.
6. **Structural flex** (drives.md, body.md): the drive's loop on the motor's
   encoder, the whole-body law on the joint's sensor (`physics.WOUND` 1); on
   the gearboxes as bodies the walk on setpoints stands but drags its toes,
   the loop on its torques walks.
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
8. **Gearboxes past their momentary ratings in falls** (body.md): each
   drive's compliance a motor-side degree of freedom, else torque limiters.
   + DOD: none > 1.0 in the scoreboard's falls.
9. **Arms down while getting up** (body.md): hands on the knees, or more hip
   torque; decision open.
   + DOD: the decision; 5 of 5 hot; hands within 620 mm of the chest
     through the lift.
10. **Get-up end** (body.md): rocking before standing, feet split fore and
    aft.
    + DOD: split rms 0.25; feet 40-80 mm apart.
11. **No abrupt get-up moves** (body.md).
    + DOD: every joint < 400 deg/s, every foot < 2 m/s; fall to walking <
      15 s; the loop at 1.0.
12. **Falls past saving** (balance.md).
    + DOD: a hand and a knee take the fall, called 0.3 s before the floor;
      the body drawn in; head < 2 kN.
13. **Scoreboard events** (balance.md): 88.5 % held; the lace 31 %, the
    120 N shoves 25-46; stairs not in it.
    + DOD: each > 75 %, the 120 N shoves and the lace among them, a slope
      and a tilt among the events; the 1.0 walk standing; the sill's
      spread the page's; stairs climbed; 8 halts of 8.
14. **Skeleton collisions** (`physics.SKELETON`, body.md): on, each thigh's
    bone meets its hip's gimbals as she rises, and she never walks.
    + DOD: head on the floor in 0 of 64 falls.
15. **Obstacle contacts**: toes into the sill, fingers into the floor at
    MuJoCo's 0.02 s; overlap or force, decision open.
    + DOD: the decision; the set measured on it.
16. **Carbon shells and clothes** over the structure as built (body.md).
    + DOD: `fit.py` 0 mm past them in every pose; seat soft; jeans a coarse
      cloth < 0.5 ms a step; no seam on a PNG.
17. **Boards on their buses** (`machine.buses`).
    + DOD: the walk on emulated boards end to end.
18. **Planner server** after LOCAL_TRIES local failures.
    + DOD: measured on a fall.
19. **One going law, the page's own** (going.md, normal.md): 0.24 off
    `normal.BAND` and on stilts to the eye, indoors, where she walks on
    it: beside the take its pelvis turns 3.5 deg where 13.4, a foot leaves
    with the knee 28 and the heel 23 deg where 36, her feet are 153 mm
    apart where 27. Risen from a fall she is on the walk as built.
    + DOD: the law the page's one walk, risen from a fall too; no stilts
      by `normal.GAITS` at a gate the eye agrees with; presses 72 of 72.
20. **Tubes, corners and flanges**: carbon tubes epoxied into printed
    corners; a gearbox on a flange; motor, box and board outermost; a
    grille or a heat sink on the FETs; a quick-release per limb.
    + DOD: no pair closer than before; each tube in its holder; each board
      on its flange; each quick-release's give modelled; a shoulder stop.
21. **A dance from the move set**: on the toes and tripping, down, a turn
    with the skirt swinging, seated legs crossed, up again.
    + DOD: twice through on the page, no fall.
22. **Movement language** (`normal.WORDS`, `gaits.MANNERS`).
    + DOD: tripping without a shuffle; 100 % of a take's measure; a walk
      into 20 N of wind with the asked accent; sitting, rising.
23. **Lumbar curve, no holding torque for the pose** (kinematics.md).
    + DOD: both walks' spine and hip torques < 0.5 N m on their stride
      means; going and falls suites at 54 of 56 and 44 of 44.
24. **The latent stack** (wbc.md): her physics as static arrays, one C loop
    over them (`wbc/`), R^k the only thing upward, the model feeding R^k as
    data. No rule a case and no branch on the leg she stands on: what a
    limb reaches is laid in advance and a landing read from it.
    + Reach (`machine/reach.py`, kinematics.md): the law reading it, a
      limb an index and the standing one landing again a hop; a landing
      pitched and a heel risen in `land`; her legs above their soles
      clear of each other.
    + Loop: a tick 26.8 M instructions on the image, 56 ms on the M7 where
      the DOD's 1; 120 N shoves 1 of 8 where the stack 3.
    + Bottom, each board: the ESO (`parts.Eso`, `CTRL_ESO`) onto the boards
      as built on the motor's encoder; torque and acceleration registers on
      the wire.
    + Behaviour: impedance (K, D a task) and a CPG's phases in R^k in
      balance.py's and going.py's place; the page's walk onto the loop.
    + Upward: the law's rows as asks - balance point, contacts, clearances,
      turns, hands; the model picks and fills rows (`decide`), never code.
    + Top: the model writes K and the phases, nothing else.
    + Wire: a feedforward torque register (PROTOCOL MINOR).
    + DOD: the stand and the walk on the C loop with their measures kept
      (J/m, band, polar); a step < 1 ms on a 475 MHz M7; `machine/` in
      subpackages, each file under 5 k tokens, `test_structure` on it.

25. **The room's words** (room.md): out from where the page stands her
    passes the door (test_room.py); from within 0.3 m of there 6 of 12,
    from anywhere in the room 1 of 12.
    + Door: her other hand meets the leaf at its hinge and stops her in
      the doorway; opened before she walks through it, or passed nearer
      its latch.
    + Stopped while a speed is asked she goes over backward on both feet:
      a step back as her capture point leaves her hold behind, whatever
      is asked.
    + Corners: a way laid with none she cannot walk at 0.3 rad/s, or a
      corner met standing without its metre past.
    + Lamp: her hand placed at a point - the arm's two joints solved for
      the switch from the pelvis as read, or the reach as she walks the
      last 0.4 m.
    + Sit and lie: a step back to the seat, or the pelvis let down on the
      law; lain along the bed from its foot she tumbled.
    + DOD: U, I, E, Y from the page, each done and up again, 10 of 10
      from anywhere in the room, in test_room.py.

26. **Knowledge and models of her surroundings** (sight.md): a stereo pair
    on her head, the room's things on a map she keeps, her ways planned
    round them, her view on the page. After a look round the chair is
    told where it stands and the table 0.07 m off, 254 of 256 cells
    within 0.15 m of a thing; she maps what is within 2.5 m.
    + DOD: the map's boxes against the room's within 0.1 m, the bed's too
      (0.51 m off: its near edge alone); a chair moved into her way
      walked round.

27. **Her kinematic language** (room.md, The words): a movement is text,
    compiled to rows one law reads; a capability is the law's to gain,
    never a case beside it. As it is: lines of go, face, stand, pace,
    hand, reach and play on the room's frames (`machine.kin`).
    + Form: s-expressions - a behaviour its phases, a phase its
      objectives, its constraints and what it waits on.
    + Terms: orthopaedic. A movement by its plane - flexion, extension,
      dorsiflexion, plantarflexion, abduction, adduction, lateral
      flexion, inversion, eversion, medial and lateral rotation - on a
      joint of a limb named by its role (stance, swing; no left or
      right), its amount degrees or a share of its range; weight bearing
      a limb's share of her; the gait cycle's events; the centre of
      pressure's place under a sole and the margin of stability; rates
      and temperatures as limits. A term on a joint she has not is
      refused.
    + Compiles to: a posture a manner on the row (knee flexion the
      strut, trunk flexion the lean, weight bearing `weigh`); a gait the
      row itself, `gaits.WALK` bit for bit; a free limb's joints and a
      limb bearing nothing, new terms of the row.
    + The law gains: a limb asked to bear nothing leaves as it is light
      and is held as asked; on one foot she is held over it across too;
      her capture point out of its hold, that foot lands again under it
      - a hop, the same landing as any (`reach.land`).
    + DOD: walked, stood, her weight onto one limb, the other lifted (hip
      30 deg, knee 60), 10 s on it by micro-hops, her knees soft and her
      trunk a little ahead, down on both again - a test; no branch on a
      side in the law (`test_structure`).

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
  world (its heat, observer, drive and loop; its acquisition next); native's
  AFE, A1335 and BNO085 board/emu's own; the mirrors removed; every hot
  path's jumps, interrupts and stalls cut but the unavoidable; state one
  contiguous array, steps branch-free passes over it. DOD: the hot paths
  ranked by them, each cut or named unavoidable; a model's change one
  change in every executive.
- **`Velocity`'s start** (`coaxial.control.motion`): sensorless at rest, as
  a rotor node's was - right while the estimate lies within a quarter turn
  of the rotor. DOD: `motion.started`, its ramp from where that leaves the
  rotor; test_sensorless's from any rest angle.
- **L map at no bias** reads L / 2.3 on the cores (drive.md): the
  dead-time table and the model's error on the sampled current's sign under
  an fs/2 ripple. DOD: Ld and Lq within 5 % at no bias.
- **The chain beside the loop** (`obs`) loses a rotor spooled at 81 000
  rad/s^2. DOD: it follows a QUAD's spool, or is cut.
- **A date refused**: `test_layout` holds the tree's text free of dates.
  DOD: the last 15 files stripped, the check in `--structure`.
- **native://**: a limb's world on a fixed mount as board/emu's; one
  Transport a rig on a URL bus. DOD: the body's balance and gait from the
  SIL; the fallback where nothing answers.
- **The MCU's die under the drive**: 9 us of sampling in a 20 us period.
  DOD: read.
- **Debug `-O0`**. DOD: `-Og` measured (LOOP, keepalive gap).
- **`heat.py`'s dry loss** 1.2 W, the bench's fit 2.4-4.4 at 24-44 V. DOD:
  it.
- **`coaxial_63020`** has no pin table in `boot_main.c`. DOD: one.
