# TODO

The aim (the user, 2026-10-04): she runs. A biped of absolute mechatronic
simplicity with headroom in its dynamic range - as few motor types and
gearboxes as can be, every special part a sourcing and logistics nightmare
-, her electronics liquid-cooled (an enclosure in transformer oil or the
like), so a drive's continuous rating is the cooling's, not the air's.
Sized in the gym (the user, 2026-10-04: `drive_sizes`' scenes, the page's
rigs) for the most mechatronic simplicity, DFM and printability: few
variants of the many-part, toleranced components - gearboxes, electronics
-, which drive complexity and cost; stock lengths, linkages and ball
bearings are cheap. A DOD runs from the LLM to the metal (the user,
2026-10-04): a pattern written in prose, picked by the model, driven by
the director over the bus to the boards' firmware and the drives - on the
emulator or the bench, not the simulated director alone. The platform
layer under the patterns (the user, 2026-10-04): classical IK - the
arrival's keyframes, the capture point placing each foot - hybrid with
reinforcement learning, the learned part a residual on the IK's setpoints
trained against MuJoCo by the grinder's loop (item 3), a table the
director reads, not a controller a hypothesis. Every special case is a
sign the problem wants a more general form - a space it lives in, the
capture point's plane for standing (the user, 2026-10-04). Every item
below serves that; the first first. A line is a gap and its DOD
(done when), numbers where they are the criterion; the measurements behind
each live in docs/FINDINGS.md's files. A line goes when its DOD is met.

## Gynoid, in order

1. **Her walk a human's, held by test** (the user, 2026-10-05: human
   walking of low loss rolls with the hip; the stance leg going back is
   extended and goes on past the plumb line; the Groucho walk is left to
   the others; a biped on straight legs draws less than one crouched, and
   moving its legs does not change that). Back since 2026-10-05
   (`stance.rolled`, `stance.carrying`), held by test_gynoid_gait.py
   (`looks.FORM`), smoked on the armada; its numbers in
   docs/findings/walk.md and balance.md. Left: her knee lands at 25-28
   deg (24 on the walk approved 2026-10-03) and is straightened under her
   weight - the roll's peak at 0.27 of the stride lands it at 21-24, and
   her starts from the squat parry with it; crouched she still walks for
   less, 371 J/m at a stance knee of 12 deg against 398 at 6; the knee's
   stop at -5 deg unused, the leg held off it by its drive; a kneel is
   called a fall (item 15); at a stance knee of 4.5, her recordings', she
   parries 5 times in three strides at 1.0 strides/s; the stride 0.75 m
   where a woman's is 1.29; her speed is her own, 0.96 m/s at 1.0
   strides/s where the plan has 0.83, and glided to 1.02 from standing
   she falls in 1 start of 6; `looks.PARRIES` 3 since CI's runner parried
   twice in a spread at 4 cm; at 0.65 strides/s the swinging foot clears
   the floor by 0.1 mm. DOD: a spread's parries at 1 or under on any
   host; her starts clean, standing and from the squat, over a spread of
   side gains, the roll's peak at 0.27; her speed within 5 % of her
   plan's at 1.0 strides/s; `looks.FORM`'s landing knee at 15;
   test_gynoid_gait.py asserting fewer J/m than the same walk crouched to
   12, 18, 24 and 30 deg at each pace; the stance knee at 4.5 or on its
   stop; the stride at 1.0 m or over.
2. **Her drives as bought** (the user, 2026-10-05: motors off the shelf,
   complete, two for their cost; one gearbox and one board, their variants
   the pain). Laid 2026-10-05 (docs/DIMENSIONS.md, docs/findings/stacks.md):
   the U8 II Lite at KV 100 on the legs' and the trunk's 15, the MN6007 II
   on the arms' and the head's 6, a 64 mm box at 1:30, the 63100 on all
   21; her 30.8 kg. Left, at 1.5x (`drive_sizes`): the hips', knees' and
   spine's peaks their 108 N m clamp (A 1.50, S 1.27), the shoulder's 40
   of 51 (A 1.17); the knee's parry 1028 deg/s and the ankle's 1073, 1.07
   and 1.02 of their volts at 48; T 0.94 at the hip walking, 0.99 at the
   knee running - in oil (`drives.COOLING` 0.3, assumed), where a grille
   or a heat sink on the FETs in air is now the thought (item 29); the
   U8's peak, 4 N m, is CubeMars' for its stator, on no page of its own;
   the rotors seen at their joints 74 and 93 times the foot at the ankle,
   29 the head, 7 the elbow - a ratio a joint; past her clothes the
   elbow's stack 20 mm, the shoulder's 15, the hip's yaw's 6 - a smaller
   box and board for the arms, if it comes to it (the user); a fall asked
   a knee's box 763 N m of its 128 (item 11).
   DOD: every joint's A, P, T and V under 1 on the run and the gym's
   scenes at the settled margin, no joint's J over 1; the pack's volts,
   the cooling and each joint's ratio baked.
3. **Balance by micro-steps, as on stilts** (the user, 2026-10-04): the support
   a point under each stance ball, nothing of the sole's shape to the
   controller, a small quick step toward the capture point whenever it leaves,
   standing and walking alike - else the control is too sensitive to initial
   conditions (a 3 mm move of the heel's spheres felled the first stride; the
   sneaker's heel spheres 797 and 49.8 % where the box's sole 604 and 73.7).
   Standing first (the user, 2026-10-04), the floor perturbed a little under
   her, then walking. Standing rigged and scored (`events.STANDING`, the
   'stand' suite, test_gynoid_stand.py; docs/findings/standing.md). The reflex
   one law (`machine.dcm`, 2026-10-04), measured on the push polar
   (`events.PUSH_DEG`, 48 pushes over 8 ways): 25 stood. The soles' load
   through its sensor's band and the stepping foot one that can land where the
   capture point goes (2026-10-04): 30 - 60 N 12 of 12, 80 N 11, 100 N 7, 120 N
   none -, the bricks 100 and 84 %, the rockers 100. Next, in the same plane:
   the 120 N pushes (the share's rate against the band, standing.md); the
   stance back at her standing height after a step - she stays 5-12 cm down on
   bent knees; the trunk thrown into the fall (the angular momentum, a CMP past
   the sole, 60 N m 17 cm on her 35 kg) as a torque - a lean on the spine's
   setpoints did nothing; a ready crouch as the capture point strays - her
   standing legs are straight, 3.8 cm of reach along the floor. Then walking: a
   hole, a sill, a slope, a tilt, a brick gone. The way (the user, 2026-10-04):
   little Python - reflexes over a small vocabulary of moves, a table the local
   model and the observer pick from, each hypothesis a line. The aim a
   meta-control-law that learns a hard surface and keeps her balance on it.
   DOD, from the LLM to the metal: a 'stand' suite on the scoreboard - the
   nudges, the bricks, the board - 100 % standing, on the board still but for
   micro-adjustments under the walk's stir; point feet (`figure`'s contacts a
   sphere at each ball, a knob) and the box sole alike; the walk's events
   joined by the slope, the tilt and the brick, the scoreboard at or over
   today's; the first stride stands with the heel and the toes moved +-1 cm;
   the sprung toes (item 6) held by it; the patterns driven over the bus to the
   emulated boards (`emulator://`, `native://`), the drives turning, not the
   simulated director alone.
4. **Running** (`machine/runner.py`, test_gynoid_run.py,
   docs/findings/run.md): from a flight at her speed she holds 0.72-1.65
   m/s half a minute on her motors as bought, 357-267 J/m, a sole 665 N at
   most. Left: asked 2.0 she is down in 8-25 s (1.94 m/s on a rotor 0.4 of
   theirs) - the stance wants to shorten with her speed, 0.26 s ran 1.91;
   her foot lands 0.3-1.1 m/s on over the floor and slides, her weight on
   it 60 ms late; every smoother swing tried has her down; her turns.
   DOD: 2 m/s standing on the scoreboard, no strike over 2 kN, the
   parries, falls and get-ups of the biped's suite held as walking.
5. **The push-off without toe motors** (the user, 2026-10-04: the motors
   off the feet, 21 drives; the toes alone flex, at the ball, a thin
   carbon-fibre sandwich with TPU or TPU round carbon rods glued with
   silicone, `TOE_K` 10 and `TOE_LOSS` 0.3; the whole sole's foam and TPU
   the contact's give, `mjcf.SOLE_S`): the heel off at 0.5 of the stride
   and the foot 40 deg down at toe-off, the scoreboard 641.7 and 78.6 %
   against 721.4 and 65.8 on driven toes - the walks 100, 100, 90 and 80 %
   at 0.65-1.0, the rises 100, 100 and 69 (docs/findings/feet.md). She
   walks lower for it: no heel's rise as the other foot lands, the landing
   knee at 35-38 deg where it stood straight, the pelvis 2-3 cm down - a
   knee's board in its SOA folds it (the soa event 54 %), and the knees'
   heat is unmeasured. A toe spring (`drives.TOE_REST` 5-10 deg up) under
   the driven push-off walks at the driven height, every rise standing,
   and loses the faults suite, 54.6-60.6 % against 71.5 (feet.md): the
   grinder on `gait.HEEL_OFF`, `TOE_DEG`, `drives.TOE_REST`, `TOE_K`,
   `TOE_LOSS` for both; else a shorter stride at a higher cadence. DOD:
   every rise and walk standing; the
   standing knee under 10 deg at mid-stance; the knees' T in `drive_sizes`
   as on driven toes; the damping from the sandwich's numbers. The rear
   foot slipped back 92 mm before its swing (the user, 2026-10-04): gone
   to 0.8 mm (`look.py`'s toes back at lift) with the heel rising by the
   leg's reach, the stride 0.75 m, the foot let go easing into its swing
   and the swing 45 mm up - the walks' power 317-487 W from 507-882
   (feet.md). Left: a sill at 0.65 strides/s and a nudge walking fell her
   2 of 3 each, none before (the scoreboard 250.5 and 82.9 % against 265.9
   and 86.4, the slip in its cost); the slip 3 and 4 mm at 0.65 and 1.0
   strides/s. DOD: the toes back at lift under 2 mm on every walk, the
   scoreboard's held at or over 86.4 %; the grinder on the faults suite
   with `stance.LET_Q`, `gait.LIFT_M`, `stance.PRE_SWING_DEG`.
6. **A real sneaker** (the user, 2026-10-04; the sole printed in TPU with
   air pockets, the shoe over it so nothing breaks): a 37-38's length, width,
   heel and toe spring; its sole's give, its forefoot's bend as the sprung
   toe, its grip, its heel's roll at the strike, its mass. DOD: each from a
   shoe's numbers; the scoreboard held; the strike's N and the touch's m/s
   quoted before and after.
7. **Her shell as armour** (the user, 2026-10-04): plates and cops over
   what a fall lands on - the knees', hips', elbows' and shoulders' stacks,
   the seat, the head -, each a printed panel over its gel pad
   (`figure.PADS`), the joints covered. DOD: the floor's force on a drum, a
   board or a tube in the falls suite and the scoreboard's falls measured
   and 0; the look practical and technological, no chrome, no lit lines,
   the plates under her jeans and T-shirt and read through them as shape
   only (the user, 2026-10-04), judged on a PNG.
8. **Fewest parts** (the user; `tools/sim/bom.py` 25 types): every holder
   and lever a 2.5D print. DOD: one bearing size, one rod end; the hip
   roll's spur pair and the ankles' bent rods each kept only where its cut
   costs the scoreboard; no flex past 0.25 deg (`members.py`); a part
   number a part, sourced as a vehicle maker's (rod ends, cardan and
   Rzeppa joints, gear pairs, bearings off the shelf).
9. **Her flex** (the user, 2026-10-03): a joint-side sensor on every drive,
   or the wind-up fed forward (a motor-side loop fell her walk). DOD: the
   user's pick; `drives.BOX_K` measured on a prototype and the walk
   standing at `physics.WOUND` 1; the marginal members (the tibia's lower
   run 0.31 deg, the femur's 0.25, the roll's horn 0.26) under 0.25; the
   pelvis's back member 9 mm into the hip roll's holders and the folded
   femur 6 into the fork's arm cleared.
10. **Gearboxes one stage, hollow, printable** (the user, 2026-10-03): a
   roller wave - catalogue needle rollers between an NA49/NA69 bearing on
   the eccentric and a 7075 lobed ring, A 1:36-41, B 1:40-43 - round the
   motor's bore, standard rollers, pins or balls, tolerances a consumer
   printer's, backdriven. DOD: the worst roller under 7075's yield at each
   size's peak; the scoreboard held at the stages' caps (fell at them
   alone, held with the spur pair and the four-bar); a joint's breakaway
   under 10 N at its segment's end (estimated: the elbow 0.80 N m, the
   ankle 2.48); the ball stages from the user's tools
   (<https://mevirtuoso.com/wave-reducer-simulator/>,
   <https://smorygo.com/wave_reducer>) printed, a prototype's torque and
   backlash measured, the races grooved or the balls rollers (a stock
   race's ball 7.46 GPa).
11. **Gearboxes past their momentary ratings** in falls (an ankle 3.7, a
   knee 3.4, a hip roll 1.9): each drive's compliance a motor-side degree
   of freedom, torque limiters where it is not. DOD: none over 1.0 in the
   scoreboard's falls.
12. **Arms down getting up** (the user, 2026-10-03; the throw stands her
    up 5 of 5, held low none of 5 hot or cold): hands pushing on the
    knees, or the hip's torque up. DOD: the user's pick; 5 of 5 hot with
    the hands under 620 mm from the chest through the lift.
13. **The get-up's end** (the user, 2026-10-03): she rocks on her feet
    before she stands (each foot 85-246 N every 0.5 s, the split's rms
    0.44-0.67 against 0.21 standing); her feet apart fore and aft as she
    rolls onto them (174-186 mm across, -2..+22 fore and aft). DOD: the
    split's rms at 0.25; the feet a minimal step apart, 40-80 mm.
14. **No abrupt moves getting up**: a hand 860 deg/s, a shoulder 720, an
    elbow 711, a foot 4.9 m/s; 20.9 s from the fall to walking, the loop
    0.82 x real time. DOD: every joint under 400 deg/s and every foot
    under 2 m/s; under 15 s; the loop at 1.0.
15. **Past saving** (P's 120 N, held 0 of 48): the fall is called
    0.06-0.25 s before the floor; a lace dives her onto her hands; the tuck
    (`falls.TUCK`) brought her head to the floor at 3.6 kN; lying her arms
    point straight out. DOD: a hand and a knee take the fall, called 0.3 s
    before the floor, the body drawn in, the head under 2 kN.
16. **The scoreboard's events**: the hole 77 %, the rug 29, the lace 26,
    the nudge 54; the walk at 1.0 strides/s falls 3 of 3; the page's sill 3
    cm on fells 12 of 12 where the scoreboard's spread holds; the stairs'
    first riser fells her (met at 0.85-0.9 of a swing, 1.4 kN); the halt
    falls in its settle. DOD: each event over 75 %, the 1.0 walk standing,
    the sill's spread the page's, the stairs climbed, 8 halts of 8.
17. **A softer walk** (strikes 1272 N, work 254 W, copper 160 of 482
    drawn; the ears bob 11 mm, 56 fore and aft): a stiffer spine or a gyro
    lead on the torso, a shorter stride at a higher cadence, a landing on
    the ball, softer soles, the pendulum placing the step (PEND_K fell at
    1.5); the pelvis dropped about the stance hip (a beam engine's beam,
    the user; undone at 583 and 59.8 % where 535 and 64.5) retuned with.
    DOD: strikes under 1000 N, the ears under 8 and 40 mm, the scoreboard
    held.
18. **Her walk retuned per build** (a scoreboard scores chance:
    near-identical builds 150 apart). DOD: the walk's knobs re-searched
    after each build change, the scoreboard quoted.
19. **Her skeleton colliding** (`physics.SKELETON`): the crouch past
    saving put her head on the floor at 1.03-1.66 m/s in 3 of 64 falls.
    DOD: 0 of 64.
20. **The obstacles' contacts**: toes 20 mm into the sill, fingers 26 into
    the floor at MuJoCo's 0.02 s; at 0.01 the head 10 kN. DOD: overlap or
    force, the user's call (asked 2026-10-01), the set measured on it.
21. **Carbon shells and clothes** shaped over the structure as built
    (`fit.py` 0 mm past her shell and clothes in every pose); her seat
    soft, not two spheres; the panels' seams finer than 20 corners a ring;
    her jeans as a coarse cloth (~6 x 5 vertices a leg, her body's
    contacts alone, the step under 0.5 ms). DOD: each measured; no seam on
    a PNG.
22. **Fewer drives**: the fingers a fist (`drives.WAYS`), their boards
    still on the arms' buses; the quick-releases' give not modelled; the
    shoulder has no stop (219 deg asked). DOD: the boards gone with the
    drives, the give modelled, a shoulder stop measured.
23. **Her boards on their buses** (`machine.buses`): an emulated limb in a
    process's place; the firmware's map wanting the walk's registers
    (`machine.rtu`); the IMU on the axis bus. DOD: the walk on emulated
    boards end to end.
24. **R in the tty crashes it** (the user, 2026-10-03; not headless). DOD:
    reproduced from the user's traceback, fixed, a test on it.
25. **The planner's server** after LOCAL_TRIES local failures. DOD: a
    failed local plan asks the server, measured on a fall.
26. **MuJoCo Warp** on the RTX 4080 SUPER (the user, 2026-10-02): the
    scoreboard's worlds batched. DOD: its step against the CPU's 0.31 ms
    measured first; the boards at 1 kHz beside it.
27. **`machine/` in subpackages**; `gait_montecarlo.py` stands at its 6 k
    cap, `director.py` at 5.9 (2026-10-04; `arrival.py` 5.2): the trial
    into its own module. DOD: `test_structure` on the layout, each file
    under 5 k.

28. **One law for her going, S and F on it** (the user, 2026-10-05). In, under
    J (findings/going.md). Left: its walk the page's own - 0.20 off
    `normal.BAND`, the user to say - at 502 J/m; her ways 76 of 104; a shove's
    parry, a reflex under it (balance.md): a leg out at once, by impulse and
    standing foot; her turns; a walk under 0.5 m/s; the floor's events on the
    law; a run past 1.5 m/s; the runner and the walk gone into it. DOD: F to
    her fastest run and S back on the page, 10 of 10; its walk on `looks.FORM`
    and `normal.BAND` at no more J/m; shoved, up as the walk as built.
29. **Her tubes, corners and flanges** (the user, 2026-10-05): carbon
    tubes cut to length, epoxied into printed corners; a gearbox on a
    flange, its output through it; motor, box and board outermost, a
    grille or a heat sink on the FETs; every print 2.5D; a quick-release
    a limb, one interface, the hands their own; carbon panels over it.
    The holders are the 70 mm stacks': on the U8s her frame's parts are
    19 mm into each other at the hips where 9 (`tools/sim/fit.py`). DOD:
    no pair closer than before, each tube in its holder, each board on
    its flange.
30. **A dance of her moves** (the user, 2026-10-05): up on her toes and
    tripping on them, down again, round in a circle - her skirt swishing -, sat
    on a chair one leg over the other, up and the sequence again. DOD: twice
    round on the page, no fall.
31. **A language for how she moves** (the user, 2026-10-05, 06: concepts on
    references, as tuples; a base, its accents, a transition). In:
    `normal.WORDS`, `gaits.MANNERS`, M Z X. DOD: tripping no shuffle; 100 % a
    take's measure; a walk into 20 N of wind, the accent asked of it; sitting,
    rising.
32. **Her kinematics on dual quaternions** (the user, 2026-10-06: an algebra
    round the hacking; findings/kinematics.md). DOD: a posture term, a C core;
    her legs, her arms, a take and her jeans' seams on it.

## Bench

- **Bootloader**, on the board since 2026-10-05. DOD: a power cycle runs
  the store at unit 1, PWR_CR3 written on a fresh supply; the prefix
  search's real collision seen; 10 Mbit on the bench adapter measured; a
  torn flash word's bus fault handled.
- **From D2 SRAM**. DOD: the ITCM sample path under the drive (a wrong
  copy hard-faults on the first ADC interrupt); LOOP cycle counters and
  `__sbrk_heap_end` stable an hour.
- **Drive**: a current loop closed through a winding,
  `tools/bench/commission.py` past its dry run. DOD: record ids 15..44
  (motor R, L, lambda, gains, injection, dead-time table) measured.
- **SOA path**: dry `budget()` over the wire, a gate proof with a lowered
  ceiling, a load run. DOD: all three on target; `Board_SyncMeanSquare`'s
  ISR cost measured.
- **Sensorless below w_lo** (the estimate never converges on a physical
  plant; the stand-in's mode a stub): (1) the front
  end's response at 12.5 and 25 kHz, one AC run of
  electronic_simulations/afe/amplifiers.asc; (2) the injection:
  `drv_inj_periods` >= 2, `demod_gain` carrying (n - 1) / n, the current
  loop under f_inj / 8 (max_frac 0.05 -> 0.03), `drv_sigma_i` measured in
  `budget()`, the 0.005 A demod offset run down, or an I/f start; (3) the
  stand-in's drive on the C core. DOD: on Renode, paused, `theta_hat`
  against `plant Shaft` through the demo's rock under 0.3 rad (0.7-1.1).
- **Motion papers on the emulator**: `motion` and `applications` set J and
  load through `drive.model`, which an emulated drive keeps. DOD: the
  world's flywheel takes a host-set load; both run on the emulator.
- **Thermal observer at short periods**: the NTC anchor inverts a standing
  miss x1.8. DOD:
  `tools/bench/power_check.py` at 1-5 s keeps its patches.
- **STO chain**: R93 to 3V3D, a master's pilot on RS485, Cinj/Clevel with
  and without it, one arm with neither bypass (`tools/bench/sto_probe.py`);
  the keepalive from a timer interrupt (the identification's shadow step
  holds main() 130 us). DOD: on the bench as on the emulator; one LTspice
  transient of `sto.asc` at 0.7 and 2.2 V within the model's windows.
- **DMA and WFI**. DOD: the A1335 by DMA measured against the poll; CYCCNT
  through WFI with DBGSLEEP_D1 (`clock.probe`); the gate supply back after
  an idle's paused keepalive before MOE; ADC3's injected end off HAL's
  handler (`Board_SyncIrq`) against the drive's cycle count; RCR 1 on the
  scope, pulses symmetric about the underflow 30 us after the sample, a
  skew set mid-run falling back to RCR 0.
- **Scope**: counted hold (MINOR 8), dead-time skew (record holds 0),
  `Q_RING` in `inverter.py`. DOD: each on the scope.
- **Thermal**: a camera under load (`board_to_ambient` at high dT, per-leg
  `to_board`, the NTC's share under load), a thermocouple on a
  winding, the supply's amps dry. DOD: the ceilings measured; the laminate's
  21 J/K and its parts' 28 two nodes, the room found on them; `f_sw` a gate op;
  `thermal_app.c`'s tables fitted on a test cycle in each.
- **Spans**: phase gain, the DC link. DOD: by a DMM.

## Host

- **native://**: a limb's world on a fixed mount as board/emu's; the AFE,
  A1335 and BNO085 repeat board/emu's C#; one Transport a rig on a URL bus.
  DOD: the body's balance and gait from the SIL (`Limb.imu`,
  `emu_world_motor`); one source for both; the fallback where nothing
  answers.
- **The meter under the drive**: `read_index` serves the NTC and the DC
  link from the latched sample. DOD: the MCU's die and the phases too;
  a sweep over a locked channel says so.
- **Debug `-O0`**. DOD: `-Og` measured (LOOP, keepalive gap).
- **`heat.py`'s dry loss** 1.2 W, the bench's fit 2.4-4.4 at 24-44 V. DOD: it.
- **QUAD raw** (the user, 2026-10-10): laps 14.1-17.0 s on boards, every
  tip 0.07 m inside. DOD: an emulated drone's heat its airspeed; gynoid
  boards in oil.
- **A1335 CRC** counted (MINOR 28) on the stand-ins' polynomial. DOD:
  `crc_errors` 0 on the bench.
- **`testline/plans/coaxial_63100_fct.yaml`** limits are placeholders.
  DOD: measured.
- **`CMD_LINK_SHARE_PCT`** 75 unmeasured on a populated RS485 segment.
- **`coaxial_63020`** has no pin table in `boot_main.c`.
