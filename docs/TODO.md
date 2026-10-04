# TODO

Open work: a line a gap and its DOD (done when), numbers where they are the
criterion; the measurements behind each live in docs/FINDINGS.md's files. A
line goes when its DOD is met.

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

## Gynoid, the biped first

- **The toes' motors out** (the user, 2026-10-04: they break at once, weigh
  the step down, keep ordinary shoes off): a sprung forefoot (`drives.WAYS`
  foot 1), the push-off reworked. DOD: the look scoreboard at or over 73.7 %
  with the toes sprung (15 as the walker is, 33 with the heel rise 50 -> 30
  deg); every rise and walk standing.
- **A real sneaker** (the user, 2026-10-04): a 37-38's length, width, heel
  and toe spring; its sole's give, its forefoot's bend as the sprung toe,
  its grip, its heel's roll at the strike, its mass. DOD: each from a
  shoe's numbers; the scoreboard held; the strike's N and the touch's m/s
  quoted before and after.
- **Her shell as armour** (the user, 2026-10-04): plates and cops over what
  a fall lands on - the knees', hips', elbows' and shoulders' stacks, the
  seat, the head -, each a printed panel over its gel pad, the joints
  covered. DOD: the floor's force on a drum, a board or a tube in the falls
  suite and the Monte Carlo's falls measured and 0; the look practical and
  technological, no chrome, no lit lines, judged on a PNG.
- **Fewest parts** (the user, 2026-10-03/04; `tools/sim/bom.py` 27 types):
  every holder and lever a 2.5D print. DOD: one bearing size, one rod end;
  the hip roll's spur pair, the foot's belt and the ankles' bent rods each
  kept only where its cut costs the scoreboard; no flex past 0.25 deg
  (`members.py`).
- **Arms down getting up** (the user, 2026-10-03; the throw stands her up 5
  of 5, held low none of 5 hot or cold): hands pushing on the knees, or the
  hip's torque up. DOD: the user's pick; 5 of 5 hot with the hands under
  620 mm from the chest through the lift.
- **The get-up's end** (the user, 2026-10-03): she rocks on her feet before
  she stands (each foot 85-246 N every 0.5 s, the split's rms 0.44-0.67
  against 0.21 standing); her feet apart fore and aft as she rolls onto
  them (174-186 mm across, -2..+22 fore and aft). DOD: the split's rms at
  0.25; the feet a minimal step apart, 40-80 mm fore and aft.
- **R in the tty crashes it** (the user, 2026-10-03; not headless). DOD:
  reproduced from the user's traceback, fixed, a test on it.
- **Past saving** (P's 120 N, held 0 of 48): the fall is called 0.06-0.25 s
  before the floor; a lace dives her onto her hands, legs straight. DOD: a
  hand and a knee take the fall, called 0.3 s before the floor.
- **The scoreboard's events**: the hole 53 %, the rug 51, the lace 26, the
  nudge 33; the walk at 1.0 strides/s falls 3 of 3; the page's sill 3 cm on
  fells 12 of 12 where the scoreboard's spread holds. DOD: each event over
  75 %, the 1.0 walk standing, the sill's spread the page's.
- **The obstacles' contacts**: toes 20 mm into the sill, fingers 26 into
  the floor at MuJoCo's 0.02 s; at 0.01 the head 10 kN. DOD: overlap or
  force, the user's call (asked 2026-10-01), the set measured on it.
- **The stairs**: each riser met at 0.85-0.9 of a swing, the other foot
  striking the step at 1.4 kN; kneeling over the hole's edge rolls her
  50-89 deg, 5 of 5. DOD: the stairs climbed on the scoreboard, a step up
  for a late stub, the get-up on the stairs.
- **A softer walk** (strikes 1272 N, work 254 W, copper 160 of 482 drawn;
  the ears bob 11 mm, 56 fore and aft): a stiffer spine or a gyro lead on
  the torso, a shorter stride at a higher cadence, a landing on the ball,
  softer soles, the pendulum placing the step (PEND_K fell at 1.5). DOD:
  strikes under 1000 N, the ears under 8 and 40 mm, the scoreboard held.
- **The halt** falls in its settle: her centre of mass off the feet's line
  as it takes over. DOD: 8 halts of 8 standing.
- **Lying**: her arms point straight out; the tuck (`falls.TUCK`) 0.2 s
  after she is down brought her head to the floor at 3.6 kN. DOD: a fall
  taken on arms, legs, knees and seat, the body drawn in, the head under 2
  kN.
- **No abrupt moves getting up**: a hand 860 deg/s, a shoulder 720, an
  elbow 711, a foot 4.9 m/s. DOD: every joint under 400 deg/s and every
  foot under 2 m/s through the get-up.
- **The get-up faster**: 20.9 s from the fall to walking, the roll 5.4; the
  loop 0.82 x real time through it. DOD: under 15 s; the loop at 1.0.
- **Her flex** (the user, 2026-10-03): a joint-side sensor on every drive,
  or the wind-up fed forward (a motor-side loop fell her walk). DOD: the
  user's pick; `drives.BOX_K` measured on a prototype and the walk standing
  at `physics.WOUND` 1; the marginal members (the tibia's lower run 0.31
  deg, the femur's 0.25, the roll's horn 0.26) under 0.25; the pelvis's
  back member 9 mm into the hip roll's holders and the folded femur 6 into
  the fork's arm cleared.
- **The margin's rule** (the user's 1.5x): with the parries and P's shove in
  the demand every leg joint's copper is 1.4-1.7 over and its inverter
  1.2-1.5; the get-up heats a hip 82 -> 117 C. DOD: the user's pick - 1.5x
  on the walk and the get-up with 1.0x on parries and falls, or every leg
  stack at 1.5x continuous - and the stacks resized to it, every
  `drive_sizes` number under 1.
- **The knee's continuous torque**: its switches at 44 A through 3.6 K/W.
  DOD: the housing's K/W as a heatsink measured, or a lower RDS; the knee's
  P 1.31 and V 1.07 under 1.
- **Her drives as stacks**, open: the hip roll's copper 1.17x (a
  crank-rocker measured and not kept); the hip's 100 mm inverter 16 mm into
  the pelvis boom in the squat; the toes' and the head's 70 mm stacks out
  of her shell. DOD: every stack inside her shell in every pose
  (`fit.py`), every `drive_sizes` number under 1.
- **Hollow-shaft gearbox** (the user, 2026-10-03): the rotor and stator in
  the gearbox's bore, a printable shell, standard rollers, pins or balls,
  tolerances a consumer printer's; backdriven. DOD: its type found and
  modelled for A, B and C; a joint's breakaway under 10 N at its segment's
  end (estimated: the elbow 0.80 N m, the ankle 2.48).
- **Gearboxes one stage** (the user, 2026-10-03): a roller wave, catalogue
  needle rollers between an NA49/NA69 bearing on the eccentric and a 7075
  lobed ring, A 1:36-41, B 1:40-43, C 1:29. DOD: the worst roller under
  7075's yield at each size's peak; the scoreboard held at the stages' caps
  (fell at them alone, held with the spur pair and the four-bar).
- **Gearboxes past their momentary ratings** in falls (an ankle 3.7, a knee
  3.4, a hip roll 1.9): each drive's compliance a motor-side degree of
  freedom, torque limiters where it is not. DOD: none over 1.0 in the
  Monte Carlo's falls.
- **The ball stages** from the user's tools
  (<https://mevirtuoso.com/wave-reducer-simulator/>,
  <https://smorygo.com/wave_reducer>) with docs/findings/drives.md's
  inputs. DOD: printed, a prototype's torque and backlash measured; the
  races grooved or the balls rollers (a stock race's ball 7.46 GPa).
- **Transmissions sourced** as a vehicle maker's (the user, 2026-10-02):
  rod ends, cardan and Rzeppa joints, gear pairs, belts, bearings off the
  shelf. DOD: a part number a part in `bom.py`.
- **Drives hot geared lower** (copper as 1/ratio^2). DOD: each joint's
  ratio traded against its transmission's and its room, `drive_sizes` T
  under 1.
- **Her legs without stops**; the toes' gearbox 1.3-1.7 times its shock
  rating shoved past saving. DOD: no joint meets a stop on the scoreboard.
- **Fewer drives**: the fingers a fist (`drives.WAYS`), their boards still
  on the arms' buses; the quick-releases' give not modelled; the shoulder
  has no stop (219 deg asked). DOD: the boards gone with the drives, the
  give modelled, a shoulder stop measured.
- **Her boards on their buses** (`machine.buses`): an emulated limb in a
  process's place; the firmware's map wanting the walk's registers
  (`machine.rtu`); the IMU on the axis bus. DOD: the walk on emulated
  boards end to end.
- **Her skeleton colliding** (`physics.SKELETON`): the crouch past saving
  put her head on the floor at 1.03-1.66 m/s in 3 of 64 falls. DOD: 0 of
  64.
- **Her walk retuned per build** (a scoreboard scores chance: near-identical
  builds 150 apart). DOD: the walk's knobs re-searched after each build
  change, the scoreboard quoted.
- **The pelvis dropped about the stance hip** (a beam engine's beam, the
  user, 2026-10-02; undone at 583 and 59.8 % where 535 and 64.5). DOD:
  retuned with it in the walk's search, at or over the scoreboard without.
- **Bare look panels** (`coaxial.graphics.panels`): the seams want finer
  meshes than 20 corners a ring. DOD: no seam visible on a PNG.
- **Carbon shells** shaped over the structure as built, clothes cut to
  them; her seat soft, not two spheres. DOD: `fit.py` 0 mm past her shell
  and clothes in every pose; the seat's give in the sit measured.
- **Her jeans as a coarse cloth** (the user, 2026-10-02): ~6 x 5 vertices a
  leg pinned at the waist, her body's contacts alone. DOD: her step under
  0.5 ms with it; drawn from its vertices.
- **MuJoCo Warp** on the RTX 4080 SUPER (the user, 2026-10-02): the Monte
  Carlo's worlds batched. DOD: its step against the CPU's 0.31 ms measured
  first; the boards at 1 kHz beside it.
- **The planner's server** after LOCAL_TRIES local failures. DOD: a failed
  local plan asks the server, measured on a fall.
- **A fast walk, then running**. DOD: 1.2 strides/s standing on the
  scoreboard, no strike over 1 kN.
- **`machine/` in subpackages**. DOD: `test_structure` on the layout.

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
