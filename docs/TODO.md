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
- End to end: the LLM writes text of balance, rhythm and elasticity, her
  latent loop runs it, its torques go over the bus to the boards' firmware
  and drives, emulated or on the bench.
- Under the text no rule a case and no branch: flat static arrays, a mode a
  spring, a landing read from her capture point.
- A special case means the problem needs a more general form.

## Gynoid, in order

1. **Orthokinematics, the one way she moves** (orthokinematics.md): her
   movement is text of balance, rhythm and elasticity, compiled to rows;
   her latent state is stepped and rendered with no branch, the whole-body
   step under it. On the page she walks from rest at 0.61 m/s, 2.6 off a
   woman's band, and crouches, shifts, bows, stands on one sole and hops
   by digit. What is not below is not hers yet, and nothing else does it.
   + Her walk's form: her knee lands at 31 deg (12) - her hip 1 to 2.5
     cm under where a straight leg needs it -, a step 0.39 legs
     (0.45), her knee swinging 45 deg (52), her heel 24 up as her toes
     leave (28); her neck nodding -26 to +21 deg a stride; 3 of
     `looks.FORM`'s 9 broken. Two takes of women:
     1.1 m/s, a step 0.8 legs, the knee landing at 2-11 deg, 50-57 as
     the toes leave, 68-73 swinging, the heel 38-57 up.
   + A stagger: she falls 17 mm into each landing, her soles 485 N then
     of her 306 and 183 after; the walk as built before the loop 10 mm,
     421 and 269. No measure of it in `strides.WALK`, no bound in
     `looks.FORM`.
   + Her walk's speed: a drive past 0.6 surges and falls at 17 to 26
     s; 1.0 m/s only with her arms at 130 deg and her head bobbing
     0.19 m. Her run: none.
   + Shoves: 10 to 40 N for 0.12 s fell her walk in 7 of 10, 3 to 10 s
     after; no payload answers one yet; `ortho.py --look` shoves none.
   + The hop as given: full extension asked off the floor and no
     landing; her own text stands. On one sole, by micro-hops: untried.
   + Her way: no mode turns her; no room round her, none of its words
     (out, the lamp, sit, lie).
   + Her rise from the squat, her stop from a walk: no text. Down, she
     is laid anew: no fall taken, no get-up.
   + The floor's events (a hole, a sill, a slip, a rug, stairs, a lace)
     and a board's (its SOA, its heat): laid by nothing.
   + The loop in C: `latent.step` and `impedance.ask` as cores beside
     `wbc/`, `ortho_row_t` a header a test holds to `ortho.ROW`; the page
     on `wbc_wrench.c`. In python the page solves every third pass;
     solved every pass her walk falls within 6 s, on either step.
   + From the page: a line to type a text into; the model composing
     rows.
   + On the front page's stand she stands: her walk there from a take of
     the loop's.
   + Her legs inside their reach's rows (`machine.reach`) walking:
     unmeasured.
   + DOD: from the page she rises, walks at 1.0 m/s in her form - on
     the band, none of its names, beside a take of a woman's, on fewer
     J/m than the same walk crouched -, turns, stops, runs 2 m/s, stands
     on one sole 10 s by micro-hops, hops and gets up from a fall, each
     a text; 80 N shoves parried by a payload; each of the floor's
     events held 3 times in 4.
2. **A real sneaker**: a TPU sole with air pockets under a shoe.
   + DOD: a size 37-38's length, width, springs, give, bend, grip, roll and
     mass from a shoe's data; her walk held; strike N and touchdown m/s
     quoted before and after.
3. **Shell as armour** (body.md): printed plates over gel pads
   (`figure.PADS`) where a fall lands.
   + DOD: floor force on a drum, a board or a tube 0 over a spread of
     falls; the plates under the clothes read as shape only, judged on a
     PNG.
4. **Fewest parts** (`tools/sim/bom.py`, body.md): every holder and lever a
   2.5D print.
   + DOD: one bearing size, one rod end; the hip roll's spur pair and the
     ankles' bent rods kept only where removing them costs her walk;
     no flex > 0.25 deg; every bought part a vehicle maker's stock part.
5. **Structural flex** (drives.md, body.md): the drive's loop on the motor's
   encoder, the whole-body step on the joint's sensor (`physics.WOUND` 1).
   + DOD: `drives.BOX_K` and the toes' `TOE_K`, `TOE_LOSS` measured on
     prototypes; her walk on the loop at `physics.BOXED` 1 in its form;
     every member < 0.25 deg.
6. **Gearboxes: one stage, hollow, printable**: a roller wave round the
   motor's bore, stock rollers, consumer-printer tolerances, backdrivable
   (<https://mevirtuoso.com/wave-reducer-simulator/>,
   <https://smorygo.com/wave_reducer>).
   + DOD: worst roller under 7075's yield at each size's peak; her walk
     held at the stages' caps; joint breakaway < 10 N at its segment's
     end; a prototype's torque and backlash measured.
7. **Gearboxes past their momentary ratings in falls** (body.md): each
   drive's compliance a motor-side degree of freedom, else torque limiters.
   + DOD: none > 1.0 over a spread of falls.
8. **Her drives against what the loop asks**: the stacks were sized on the
   laws before the loop (drives.md, stacks.md); DIMENSIONS.md's holds are
   thermal alone.
   + DOD: each drive's peak, rms, speed and heat on her walk, her hop and
     a shove on the loop, none past 2/3 of a part's limit.
9. **Skeleton collisions** (`physics.SKELETON`, body.md): her walk on the
   loop with it on is untried.
   + DOD: she walks with it on; head on the floor in 0 of 64 falls.
10. **Obstacle contacts**: toes into the sill, fingers into the floor at
    MuJoCo's 0.02 s; overlap or force, decision open.
    + DOD: the decision; the set measured on it.
11. **Carbon shells and clothes** over the structure as built (body.md).
    + DOD: `fit.py` 0 mm past them standing, in the squat and over a
      recorded walk; seat soft; jeans a coarse cloth < 0.5 ms a step; no
      seam on a PNG.
12. **Boards on their buses** (`machine.buses`): the loop writes its
    torques into the world, no board hears them.
    + DOD: her walk on the loop on emulated boards end to end, a
      feedforward torque register on the wire (PROTOCOL MINOR), each
      board's heat in her state.
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
16. **Lumbar curve, no holding torque for the pose** (kinematics.md).
    + DOD: her walk's spine and hip torques < 0.5 N m on their stride
      means.
17. **The latent stack** (wbc.md): her physics as static arrays, one C loop
    over them (`wbc/`), R^k the only thing upward, the model feeding R^k as
    data.
    + Flight (`machine/flight.py`, flight.md): the strike at a landing,
      1.8-2.8 kN at 0.8 m/s; the loop in C 15-17 deg over a hop of 48 mm;
      a hop on one sole.
    + Reach (`machine/reach.py`, kinematics.md): her stride reading it, a
      limb an index; a landing pitched and a heel risen in `land`; her
      legs above their soles clear of each other.
    + Loop: a tick 26.8 M instructions on the image, 56 ms on the M7 where
      the DOD's 1. The step without an active set in C (`wbc_wrench.c`,
      wrench.md): 0.70 M instructions standing, 0.81 M a sole swung and
      a moment clipped, 1.71 ms; her mass matrix 0.25 M of it.
    + Bottom, each board: the ESO (`parts.Eso`, `CTRL_ESO`) onto the boards
      as built on the motor's encoder; torque and acceleration registers on
      the wire.
    + Top: the model writes K and the phases, nothing else.
    + DOD: her stand and her walk on the C loop with their measures kept
      (J/m, band, shoves); a step < 1 ms on a 475 MHz M7; `machine/` in
      subpackages, each file under 5 k tokens, `test_structure` on it.
18. **Knowledge and models of her surroundings** (sight.md): a stereo pair
    on her head, the room's things on a map she keeps, a way planned
    round them. After a look round the chair is told where it stands and
    the table 0.07 m off, 254 of 256 cells within 0.15 m of a thing; she
    maps what is within 2.5 m.
    + DOD: the map's boxes against the room's within 0.1 m, the bed's too
      (0.51 m off: its near edge alone); a chair moved into her way
      walked round on the loop; her view on the page.
19. **Findings that tell of the laws before the loop**: body, feet, drives,
    stacks, normal and wbc keep their figures under a note.
    + DOD: each cut to what holds for her body and the loop; a check in
      `test_layout`: no document names a module, a tool or a suite not
      in the tree.

## Quad

- **The page's flight a fifth of this host**: four stand-in boards a pass
  of 2 ms are 0.20 s a flight's second where 0.12 at 20 ms, 0.51 beside
  the drawing's thread - three calls a rotor a pass, each a board's whole
  state read; 16 frames a second where 17.8, and under 50 suites its clock
  half the wall's. DOD: a board's pass one call; 20 frames a second.
- **The heat's clock in flight**: ten thermal seconds a flight's
  (`HASTE`): full tilt's 2 s are 20 of heat, the legs at their throttle's
  point 0.2 s in (envelope.md). DOD: flown on the flight's own seconds,
  the observers out of UNCERTAIN before a first flight; the race on it.
- **The plan on boards that hold their pull**: `course.PULL` gridded alone
  to 0.37 (envelope.md); BRAKE, GO and the crossings are the search's on
  boards that had 34-60 % of it and a law since gone, the crossings laid
  again by flight (streams.md). DOD: `quad_race.py --search` over them on
  the core, the laps against 14.3-14.8 s, every flight whole.
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
    clause in a pass above the core (streams.md). Left: the lap's sphere,
    its brake's and its swing's shares from its rotors' reach, where they
    were searched.
  + The axis asked brings its own rate: the force asked turns 7.2 rad/s
    rms on a lap, the frame 4.6, its axis 29 degrees rms off; that turn fed
    forward struck every flight (streams.md). DOD: a row's force one the
    frame turns with - its turn and that turn's change inside the reach -,
    its rate in the payload; the axis's miss on a lap quoted.
  + On four boards in a wind the rows have 76-82 % of the clamp's pull at
    the least on a second lap, the law before 89-91 %: what the core's
    passes of 2 ms spend.
  + WEP's reach: the loops' slew is the flight's clamp's; on WEP's a rotor
    near its idle is lost, braked or risen (envelope.md). DOD: the
    estimate's reach by speed on the stand-in, the loops' slew that.
  + A room too hot for a lap (toasty, 45 C): a board holds none in the
    dive and the floor is struck. DOD: what the boards hold a flight long
    in their room, known before it lifts (thermal op, a horizon), `fit` on
    it.
  + A flight on a board's core: device 14 takes its reading off the wire,
    246 bytes a pass. DOD: the reading the board's own - its IMU's, its
    rotors' off the bus -, a pass in its sample, the page's flight on it.
  + DOD: the pull left unused and the WEP taken a flight quoted; the race
    whole on it.

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
- **The law's vector a period late**: laid on the angle it sampled at, its
  duties wait a period and meet the rotor that much turned; its integrators
  take it up and its v . i is 21 % under the stage's watts at 680 rad/s
  (envelope.md). The rotor page's power (`views/rotor/thermal.watts`) and
  `notebooks/motion` count it. DOD: the vector laid on the angle it is
  applied at; v . i the copper's and the shaft's within 1 % there on the
  stand-in.
- **CI's flakes**: a run each failed and the next passed - `test_native_heat`
  (STABLE held 100 s within 600: 394), `test_sensorless` (the loop under a
  concurrent reader; the shunt's sign), the emulator's reads (a CRC on
  `afe.state`, a board's info in its place), `test_wbc_core` (a tick of
  the loop 4 202 us), `test_mcp` (the program card). DOD: each looped 20
  times on a loaded host, its cause found.
- **`Velocity`'s start** (`coaxial.control.motion`): sensorless at rest, as
  a rotor node's was - right while the estimate lies within a quarter turn
  of the rotor. test_controller's velocity loop on the stand-in faulted
  overcurrent in 8 starts of 384, 32 at a time, and once on CI's runner:
  within three passes, the estimate at -9 000 to -12 800 rad/s as the ramp
  crosses the hand-over; a pass 0.1-1.0 s late faulted none of 15. DOD:
  `motion.started`, its ramp from where that leaves the rotor;
  test_sensorless's from any rest angle; none in 400 starts.
- **The ring test's J** reads 30-63 % over the stand-in's (drive.md): the
  hold's stiffness taken as 1.5 pp^2 lambda I, the board's current loop
  softer at the ring's rate. DOD: within 5 %.
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
