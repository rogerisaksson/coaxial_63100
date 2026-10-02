# TODO

Open work. Measured results are in FINDINGS.

## Needs the bench

- **Bootloader**: flash it (`build_and_flash.py --boot`), then the app's
  sealed store; boot it into D2 SRAM (the first run from RAM), load an image
  over the ST-Link's port, persist it; see the prefix search's real collision
  (CRC error, timeout or both). 10 Mbit on the bench adapter unproven.
- **First flash since 2026-09-16**: ITCM sample path (a wrong copy
  hard-faults on the first ADC interrupt), `test_bench.py` vs baseline,
  LOOP cycle counters, `__sbrk_heap_end` stable over an hour.
- **Drive**: a current loop closed through a winding;
  `tools/bench/commission.py` beyond its dry run. Record ids 15..44 (motor R,
  L, lambda, gains, injection, dead-time table) are placeholders.
- **SOA path** on target: dry `budget()` over the wire, gate proof with a
  lowered ceiling, a load run. `Board_SyncMeanSquare` ISR cost
  unmeasured.
- **Sensorless below w_lo**: the injection's estimate does not converge on
  a physical plant, native or Renode (FINDINGS 2026-09-27); the stand-in's
  sensorless mode is a stub, so it never showed. To decide: (1) the front
  end's response at 12.5 and 25 kHz - one AC run of
  electronic_simulations/afe/amplifiers.asc on the phase input, the
  transfer is static so far; (2) the injection: `drv_inj_periods` >= 2 with
  `demod_gain` carrying the top-sampled (n - 1) / n and the current loop
  under f_inj / 8 (`current_loop`'s max_frac 0.05 -> 0.03), `drv_sigma_i`
  measured in `budget()` instead of assumed, the 0.005 A demod offset at
  zero error run down - or an I/f start and no injection; (3) the stand-in's
  drive on the C core the world library already builds, so the reference is
  one. The instrument: on Renode with the emulation paused, `theta_hat` read
  out of RAM against `plant Shaft` through the demo's rock, under 0.3 rad -
  the drive's `eps` alone says it now, 0.7-1.1 rad on either emulator.
- **Motion papers on the emulator**: `motion` and `applications` set their
  rotor's J and load through `drive.model`, which an emulated drive keeps to
  itself; the world's flywheel needs a load the host sets. They run on the
  stand-in until then (their `MODE`).
- **Thermal observer at short sample periods**: the NTC anchor re-inverts a
  standing miss through the lag every sample (FINDINGS 2026-09-27); under
  30 s it winds the leg patches away (`tools/bench/power_check.py` samples
  at 1-5 s). A derivation that inverts only the unexplained growth, net of
  the model's own response to the last push, and the identification
  retuned on it; or a floor on the period.
- **STO chain**: circuit change (R93 to 3V3D), a master sending the pilot on
  RS485, Cinj/Clevel with and without it, one arm with neither bypass
  (`tools/bench/sto_probe.py`) - on the emulator since 2026-09-27. The
  keepalive from a timer interrupt: the thermal identification's shadow
  step holds main() 130 us. One LTspice transient of `sto.asc` at 0.7 and
  2.2 V against the model's windows.
- **DMA and WFI**: the A1335's reads by DMA against the old poll, CYCCNT
  through WFI with DBGSLEEP_D1 (`clock.probe`), the gate supply back after an
  idle's paused keepalive before MOE; ADC3's injected end off HAL's handler
  (`Board_SyncIrq`) against the drive's cycle count; the drive at RCR 1 on the
  scope - pulses symmetric about the underflow, 30 us after the sample - and
  a skew set mid-run falling back to RCR 0.
- **Scope**: counted hold (MINOR 8), dead-time skew (record holds 0),
  `Q_RING` in `inverter.py`.
- **Thermal**: camera under load (`board_to_ambient` at high dT, per-leg
  `to_board`), a power step and the NTC's slope (leg capacity: burst budget
  is 0.22-0.67 s), a thermocouple on a winding. Ceilings for drivers,
  regulators, AFE and laminate are estimates.
- **Spans**: phase gain; DC link is the only spanned channel.

## Host

- native://: a limb's world stands on a fixed mount, as board/emu's: the
  body's balance and gait come from the SIL (`Limb.imu`, the world's
  `emu_world_motor`); the AFE, A1335 and BNO085 repeat board/emu's C# (the
  heat is world_heat.c's), one source for both wanted; one Transport a rig on
  a URL bus, the limb keeping t3.5 for them; not yet the fallback where
  nothing answers (Renode is).
- Gynoid, open (a line goes when done):
  + Past saving (P's 120 N, held 0 of 48) she crouches as she goes, a shank
    first (`falls.crouch`), but ends on a side: the fall is called 0.06-0.25
    s before the floor, too late for a hand and a knee to take her - an
    earlier verdict wanted; the capture point past a step's reach comes
    0-0.45 s sooner, `capture`'s `need` foretells nothing. A lace holding
    her trailing foot she dives onto her hands, legs straight.
  + The scoreboard's events held: the hole 53 %, the rug 51 %, the lace 26 %,
    the nudge 33 % (2026-10-01). The walk at 1.0 strides/s falls in 3 runs
    of 3 (HEAD 2 of 3). A sill laid as the page lays it, a stride on, 3 cm
    apart over 33 cm, fells her 12 times of 12; the scoreboard's three, 3 cm
    either side of one place, hold: the spread too narrow.
  + The obstacles' contacts stiffer: toes 20 mm into the sill for 35 ms,
    fingers 26 mm into the floor (MuJoCo's 0.02 s give). Her body's set at
    0.01 s, the head 28 -> 20 mm in and 3.8 -> 10.0 kN, the shoves' landing
    6.2 -> 11.6 kN; at 0.005 10 mm and 16.8 kN: overlap or force, the
    user's call (asked 2026-10-01).
  + The stairs fell her at the first riser since 855c87c: each riser met at
    0.85-0.9 of a swing, past TRIP_LATE, the foot put down short, the other
    striking the step at 1.4 kN; a step up for a late stub wanted; the get-up
    on the stairs untried. Kneeling over the hole's edge, one knee 3 cm down,
    knees under and sitting back roll her 50-89 degrees, 5 tries of 5.
  + A softer walk: fewer strikes (1272 N), less power (work 254 W, copper
    160 W of the 482 drawn). The ears bob 11 mm a stride and go 56 mm fore
    and aft, the pelvis's 30 doubled by the torso's 3 degrees of pitch; the
    stance knee 15-20 degrees through mid-stance, latched 10-12 mm low at
    each landing. To try: a stiffer spine drive or a rightly signed gyro lead
    on the torso; a shorter stride at a higher cadence; a landing that does
    not sag, on the ball (LAND_DEG below 0 under the landing's cost);
    softer soles, then a compressible sole layer; the pendulum between the
    ears placing the next step (PEND_K fell at 1.5, 2026-09-27).
    The stance legs' weight by load flickers about LANDED_N, a hip's setpoint
    2-5 deg a pass 4-10 times a second: rate-limited over 0.1 s the strike
    1231 -> 1024 N and the jumps 109 -> 44 in 16 s, but shoves past saving
    then brought her head down 2 times of 16 and a shank first 9, not 14.
  + The halt falls in its settle wherever tried: her centre of mass stands
    off the feet's line as it takes over.
  + Lying, her arms point straight out. A fall taken on the arms, legs,
    knees and seat to spread its blows, then the body drawn in so nothing
    breaks if she tumbles on, down a slope: the tuck (`falls.TUCK`) drawn in
    0.2 s after she is down brought her head to the floor at 3.6 kN, at
    1.0 s as without it; lying, her hands' reach unmeasured since.
  + No abrupt moves getting up: lifting onto her feet a hand 860 deg/s, the
    squat's shoulder 720, the fall's own elbow 711 and a foot 4.9 m/s.
  + The get-up faster: 20.9 s from the fall to walking, the roll 5.4 of it;
    the arrival's runaway guard. The loop at 0.82 x real time through a fall
    and its get-up, 0.73 walking: the host spins 0.20 ms of a 1.22 ms pass
    on the boards' tick, the walker takes 0.46 ms of a walking pass.
  + Fewer drives, for weight and BOM: of the 27 on 5 buses (the axis 5, an
    arm 4, a leg 7): the head's turn held rigid changed nothing in 5
    scenarios. The fingers a fist without drives (`drives.FINGERS`), their
    boards still on the arms' buses, their limit 0. The toes on a spring
    every walk falls within 0.8 s: the walker's push-off asks them and its
    legs' reach counts on them - reworked for a passive toe. The elbow's M
    drive, 70 mm, stands wider than her 56 mm arm: at the shoulder, a rod to
    the forearm. M's and S's motors are estimates; the quick-releases' give
    at the shoulders and hips is not modelled. The shoulder has no stop: the
    roll re-searched as built asks 219 degrees. High torque through a
    gearbox and a rod; the rest direct drive where its torque stays
    reasonable: as built only the head's turn asks little enough (2.7 N m
    getting up).
  + Her boards on their buses (`machine.buses`): an emulated limb in a
    process's place on its port and block; the firmware's map wanting the
    walk's registers (`machine.rtu`); the IMU on the axis bus, not the
    world's own reading.
  + The planner's server after LOCAL_TRIES local failures.
  + `machine/` in subpackages.
  + A fast walk, then running: no strikes, no blows, quiet and smooth.
  + The get-up on her build as made: the roll pushed by
    the arms takes her face down and knelt from all ten starts; from the five
    falls she reaches the squat and the rise and tips sideways there, 0 of 5
    walking again (HEAD's build 13 of 13): 20 s after a fall the hips at
    115-120 C, the right derated to 0.40 (`heat.HASTE` 10).
  + The pelvis dropped about the stance hip, not its middle (a beam engine's
    beam, the user, 2026-10-02): undone, its walk caught 5-9 times in its
    first 3-5 s, the scoreboard 583, held 59.8 % where 535, 64.5
    (docs/findings/walk.md); retuned with it in the walk's search.
  + MuJoCo Warp on the RTX 4080 SUPER (the user, 2026-10-02): the Monte
    Carlo's worlds batched on it, a fine cloth with them - its step against
    the CPU's 0.31 ms measured first, each world's boards at 1 kHz beside it.
  + Her jeans as a coarse MuJoCo cloth, a reference for her motion, not a
    simulation of denim (the user, 2026-10-02): a tube of ~6 x 5 vertices a
    leg pinned at its waist, its contacts her body's alone (contype 2,
    conaffinity 0) - her 1 ms step 0.31 ms, with 48 such vertices 0.31, 100
    0.47, 100 colliding with everything 1.06 - drawn from its vertices.
  + Her walk retuned for her build as made - drives as modules, L wound
    1.25: 535 on the scoreboard, held 64.5 % (HEAD's build 214, 82.0 %;
    before the rewind 635, 57.3); test_gynoid_falls' parry 1 of 8 where 2,
    the offline gate red on it, unpushed. Searching the arrival's, the
    capture's, the side step's and the parry's knobs (2026-10-02). One
    scoreboard a build scores chance: near-identical builds 150 apart.
  + Her gearboxes past their momentary ratings in six falls of six, by
    their peaks (`World.geared` over `drives.shock`): an ankle 3.7, a knee
    3.4, a hip roll 1.9, the toes 2.2 - each rotor's reflected inertia
    stopped at the impact through a rigid gearbox. A gearbox's compliance K
    caps it near the impact's speed times sqrt(K J): a knee at 10 rad/s, an
    estimated 5000 N m/rad and 0.163 kg m^2, 285 N m. Each drive's
    compliance modelled, a motor-side degree of freedom; torque limiters (a
    motorcycle's slipper clutch, a ball-detent coupling) where it is not.
  + Her gearboxes' ball stages exported from the user's tools
    (<https://mevirtuoso.com/wave-reducer-simulator/>,
    <https://smorygo.com/wave_reducer>) with docs/findings/body.md's
    inputs, printed, a prototype's torque and backlash measured.
  + Her bare look printable panels over her skeleton - carbon tubes, rods,
    drives, boards and generic parts between them -, ordinary clothes over
    it (the user, 2026-10-02): the limbs' panels light, their joint ends, the
    neck and the waist open (`coaxial.graphics.panels`); the seams between
    panels want finer meshes than 20 corners a ring (`shapes.AROUND`).
  + The rods asked: the spine roll's, from its M on the pelvis's top to the
    roll's yoke over the spine's pitch, and the ankle roll's, from its M
    before the tibia to the foot's side over the ankle's pitch.
  + No belt where its give would show in her walk (the user, 2026-10-02):
    the toes' S pulls its belt 1.2 kN at 12 N m on 10 mm pulleys - a rod;
    the elbow's 1.4 kN and the wrist's 1.0 at their peaks.
  + Her transmissions sourced as a bicycle's, a motorcycle's or a car's
    maker would (the user, 2026-10-02): rod ends, cardan and Rzeppa joints,
    gear pairs, belts and bearings off the shelf, a part list a joint.
  + Her drives running hot geared lower, a little larger, the gearbox's
    ratio traded against its transmission's to size and place it (the
    user, 2026-10-02): copper goes as 1/ratio^2 - an M at 1:60 sheds 22.5
    N m rms for good, the waist's 21.5 and the hip yaw's 16.2 under it.
  + Past her clothes standing: the ankle roll's drum 2 mm through the
    jeans' shin (`tools/sim/fit.py`); the spine roll's 3 mm past her skin.
  + Her legs without stops; the toes' S gearbox 1.3-1.7 times its shock
    rating shoved past saving.
  + Her carbon shells shaped over the structure as built, and clothes cut
    to fit them; her seat soft - a body with give, cloth over it - not two
    spheres a cheek.
- The meter under the drive: `read_index` serves the NTC and the DC link
  from the latched sample; the MCU's die (an identification anchor, unread
  under load) and the phases could join them; a software sweep over a channel
  the drive locks out waits without a word.
- Debug is `-O0`; `-Og` is a measurement away (LOOP counters, keepalive gap).
- `intent.py` has no thermal kind: warmth questions become an NTC read.
  Measure against the live model before landing.
- `test_sensorless` overpowered-servo check flakes ~1 in 4 inside the full
  gate only.
- A1335 CRC polynomial unknown (CRC reported, not checked).
- `testline/plans/coaxial_63100_fct.yaml` limits are placeholders.
- `CMD_LINK_SHARE_PCT` 75 unmeasured on a populated RS485 segment.
- PE15 reading 0 with the AFE on: driver not established.
- Gate op 10 (alternate) has no period count.
- `coaxial_63020` has no pin table in `boot_main.c`.
