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
trained against MuJoCo by the grinder's loop (item 2), a table the
director reads, not a controller a hypothesis. Every item below
serves that; the first first. A line is a gap and its DOD
(done when), numbers where they are the criterion; the measurements behind
each live in docs/FINDINGS.md's files. A line goes when its DOD is met.

## Gynoid, in order

1. **Two motor types, two gearboxes, two inverters** (the user,
   2026-10-04): the elbow, the neck and the head on frame B direct, frame C
   and box C gone with the toes' and the wrists' drives; the leg stacks
   sized for running with headroom - a running scene in `drive_sizes` (the
   literature's peaks a kg, hip 2.5, knee 3, ankle 3.5 N m, 8 W a joint)
   at the user's 1.5x; the thermal path the cooling's K/W, not the
   housing's 3.6; the gym's scenes among the sizing's (the ankle roll's
   amps 1.08 and box 1.04 from them). DOD: `tools/sim/bom.py` 2 motors, 2
   gearboxes, 2 inverters; every leg joint's A, P and V under 1 on the
   running scene and the gym's, T under 1 with the cooling; the margin's
   rule settled by it (1.5x on the walk, the get-up, the run and the gym;
   the user's call on parries and falls).
2. **The gym's grinder: the local model and a dumb searcher run my
   hypotheses** (the user, 2026-10-04, <https://youtu.be/J5ax5LrOovA>): a
   prompt that gives the local model her kinematics - the capture point,
   the support, the vocabulary of moves, the knobs, the scoreboard's
   suites and their numbers - and asks for hypotheses as knob spans or a
   rule line with a suite to score them on; a loop that scores each on
   the relay (CMA-ES sized to it, RL against MuJoCo later), logs the
   record and feeds it back; I reason and smoke-test, the tokens mine
   stay few. The grinder's choices - the suite, the knobs, a repeat, how
   she lies - a decision model's (Cloudflare's Clef 27B as `clef:4k`,
   Ollama's `/v1/systemone`: a state and typed questions in, a probability
   an option out in one pass, 5.3 s; the 9B refuses on 0.35.1;
   docs/MODELS.md; the user, 2026-10-04), the hypothesis text a chat
   model's. DOD: `tools/sim/gym.py` runs unattended an hour on the look
   suite from a prompt, every candidate a line in its log with the
   scoreboard's numbers, the best quoted in balance.md.
3. **Balance by micro-steps, as on stilts** (the user, 2026-10-04): the
   support a point under each stance ball, nothing of the sole's shape to
   the controller, a small quick step toward the capture point whenever it
   leaves, standing and walking alike - else the control is too sensitive
   to initial conditions (a 3 mm move of the heel's spheres felled the
   first stride; the sneaker's heel spheres 797 and 49.8 % where the box's
   sole 604 and 73.7). Standing first (the user, 2026-10-04), the floor
   perturbed a little under her, then walking. Standing rigged and scored
   (`events.STANDING`, the 'stand' suite, test_gynoid_stand.py; the
   board stiff and free, the bricks abreast and staggered): the nudges and
   the stiff board 100 %, the shoves 46-49, the bricks 48-56, the free
   rocker 21-26 (docs/findings/balance.md). The step (`machine.stand`)
   earns nothing yet: a shove's cross-over cannot reach in 0.25 s, the
   brick's step down lands and she leans back off both feet, the rocker
   fells the rise. P's shove at phase 0.14 to her left: the catch
   overshoots and her head strikes at 1.74 m/s (the falls suite's crouch,
   red since the merge; balance.md). Then walking: a hole, a sill, a
   slope, a tilt, a brick gone. The way (the user, 2026-10-04): little
   Python - the reflexes a prose stream encoded into meta-movement
   patterns over a small vocabulary of moves (step, lean, crouch, hold,
   catch; `machine.planner`'s way), a table of when and what the local
   model and the observer pick from, each hypothesis a line, not a
   controller. The aim a meta-control-law that learns a hard surface and
   keeps her balance on it. DOD, from the LLM to the metal: a 'stand'
   suite on the
   scoreboard - the nudges, the bricks, the board - 100 % standing, on the
   board still but for micro-adjustments under the walk's stir; point feet
   (`figure`'s contacts a sphere at each ball, a knob) and the box sole
   alike; the walk's events joined by the slope, the tilt and the brick,
   the scoreboard at or over today's; the first stride stands with the
   heel and the toes moved +-1 cm; the sprung toes (item 5) held by it;
   the patterns driven over the bus to the emulated boards (`emulator://`,
   `native://`), the drives turning, not the simulated director alone.
4. **Running**: a gait with flight, from the walk's search. DOD: 2 m/s
   standing on the scoreboard, no strike over 2 kN, the parries, falls and
   get-ups of the biped's suite held as walking.
5. **The toes' motors out** (the user, 2026-10-04: they break at once,
   weigh the step down, keep ordinary shoes off): a sprung forefoot
   (`drives.WAYS` foot 1, `TOE_K` 25 - a sneaker's forefoot; stiffer stood
   her on her toe tips), springy but damped - a thin carbon-fibre sandwich
   with TPU, or TPU printed with carbon rods poked into the print and
   glued with silicone (the user, 2026-10-04): `TOE_C` from TPU's loss
   factor -, the
   push-off reworked: searched, the rises 45-79 % and the walks 82, 89, 18
   and 6 at 0.65-1.0 (docs/findings/walk.md). DOD: the scoreboard at or
   over 73.7 % with the toes sprung; every rise and walk standing; the
   damping from the sandwich's numbers.
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
8. **Fewest parts** (the user; `tools/sim/bom.py` 27 types): every holder
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
16. **The scoreboard's events**: the hole 53 %, the rug 51, the lace 26,
    the nudge 33; the walk at 1.0 strides/s falls 3 of 3; the page's sill 3
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
27. **`machine/` in subpackages**. DOD: `test_structure` on the layout.

## Bench

- **Bootloader**: `build_and_flash.py --boot`, the app's sealed store, a
  boot into D2 SRAM, an image loaded over the ST-Link's port and persisted.
  DOD: the prefix search's real collision seen (CRC error, timeout or both);
  10 Mbit on the bench adapter measured.
- **First flash since 2026-09-16**. DOD: the ITCM sample path boots (a
  wrong copy hard-faults on the first ADC interrupt); `test_bench.py` at
  its baseline; LOOP cycle counters and `__sbrk_heap_end` stable an hour.
- **Drive**: a current loop closed through a winding,
  `tools/bench/commission.py` past its dry run. DOD: record ids 15..44
  (motor R, L, lambda, gains, injection, dead-time table) measured.
- **SOA path**: dry `budget()` over the wire, a gate proof with a lowered
  ceiling, a load run. DOD: all three on target; `Board_SyncMeanSquare`'s
  ISR cost measured.
- **Sensorless below w_lo** (the estimate never converges on a physical
  plant, native or Renode; the stand-in's mode is a stub): (1) the front
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
- **Thermal observer at short periods** (the NTC anchor re-inverts a
  standing miss every sample; under 30 s the leg patches wind away). DOD: a
  derivation inverting only the unexplained growth, or a floor on the
  period; `tools/bench/power_check.py` at 1-5 s keeps its patches.
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
  `Q_RING` in `inverter.py`. DOD: each seen on the scope.
- **Thermal**: a camera under load (`board_to_ambient` at high dT, per-leg
  `to_board`), a power step and the NTC's slope, a thermocouple on a
  winding. DOD: the drivers', regulators', AFE's and laminate's ceilings
  measured, not estimated.
- **Spans**: phase gain. DOD: spanned as the DC link is.

## Host

- **native://**: a limb's world on a fixed mount as board/emu's; the AFE,
  A1335 and BNO085 repeat board/emu's C#; one Transport a rig on a URL bus.
  DOD: the body's balance and gait from the SIL (`Limb.imu`,
  `emu_world_motor`); one source for both; the fallback where nothing
  answers.
- **`tools/sim/montecarlo.py`** (the FOC loop's) runs a pool of its own.
  DOD: its jobs as shards on the relay, the library loaded once a shard.
- **The meter under the drive**: `read_index` serves the NTC and the DC
  link from the latched sample. DOD: the MCU's die and the phases join
  them; a sweep over a locked channel says so.
- **Debug `-O0`**. DOD: `-Og` measured (LOOP counters, keepalive gap).
- **`intent.py` has no thermal kind**: warmth questions become an NTC read.
  DOD: answered from the live model, measured against it.
- **`test_native_heat`** lost the cold room on CI 3.12 (925a173). DOD:
  STABLE again within FIND_S on CI.
- **`test_sensorless`** overpowered-servo check flakes ~1 in 4 in the full
  gate. DOD: 20 gates green.
- **A1335 CRC** reported, not checked. DOD: the polynomial known, checked.
- **`testline/plans/coaxial_63100_fct.yaml`** limits are placeholders.
  DOD: measured.
- **`CMD_LINK_SHARE_PCT`** 75 unmeasured on a populated RS485 segment.
- **PE15** reads 0 with the AFE on: driver not established.
- **Gate op 10** (alternate) has no period count.
- **`coaxial_63020`** has no pin table in `boot_main.c`.
