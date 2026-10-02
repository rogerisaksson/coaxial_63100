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
  + Her a little sturdier, not as slight, every drive and rod hidden under
    her carbon and her clothes, the clothes' size the give (the user,
    2026-10-02): the hips a gimbal - the pitch's L centred on the hip, the
    roll's L on its axis behind it, the yaw's M above -, the ankle's L
    upright in the calf on a right-angle stage, the rest grown over. Her
    joints over 21 runs: the hip, knee, ankle and hip roll at their L's
    141 N m clamp, rms 50, 47, 40, 37; the waist's and the hip yaw's M at
    53, rms 21.5 and 16.2 past an M's 15 shed for good.
  + The leg's quick-release (`build.RELEASES`) stands 15 mm into the knee
    drive's top: the drive 35 mm lower, 0.012 kg m^2 more on her swing, the
    rise to the walk fell at 6.5 s - lowered with the hip's gimbal and the
    walk retuned for her new build.
  + Her drives running hot geared lower, a little larger (the user,
    2026-10-02): copper goes as 1/ratio^2 - an M at 1:60 sheds 22.5 N m rms
    for good, the waist's 21.5 and the hip yaw's 16.2 under it; the hip
    roll's L at 1:45 0.64 of its copper - speed and reflected inertia the
    price.
  + The seat's drives in closer to the pelvis, nothing standing out of her:
    clothes bought off the rack fit her (the user, 2026-10-02); standing,
    the hip's reaches 42 mm past her skin, its roll's 48, and the roll's
    drum stands 44 mm into the yaw's. A gimbal hip - the pitch's L centred
    on the hip carrying the roll's trunnions, the roll's M behind, the
    yaw's M above - 12, 42 and 23 mm, nothing touching over 21 runs' yaws
    and rolls. Her skin is an EU 32: hips 80.8 cm, bust 70. Her roll asks
    an L (rms 36.6, 141 peaks): behind the hip it swings with the yaw into
    the other's past 28 degrees - the yaw stopped at 25 (`mjcf.HIP_YAW_DEG`)
    held the scoreboard (248 against 238, held 81.0 %) and 3 get-ups of 3,
    the L rolls then 9 mm into each other and 58 out of her seat. Roll,
    pitch, yaw locks at 90 degrees of hip flexion: the order stays.
  + The mechanism realizable: each rod's crank, length and horn giving its
    stroke curve (`drives.STROKES`, a first cut from where the joints asked
    torque) over the joint's range, and the drives, rods, bones and shells
    clear of each other over every stroke - checked by geometry, not drawn.
    Drawn, a crank turns by its joint's angle times a lever, the rod's
    length free; the knee's rod to the tuberosity passes its dead point
    near 47 degrees, the ankle's lever 9 mm at -50; the hip's pair on the
    pelvis a spatial linkage, each crank by the yaw, roll and pitch; the
    ankle's roll, the wrist's, the gripper's and the toes' drives off their
    axes with nothing to them; the legs without stops; the toes' S gearbox
    1.3-1.7 times its shock rating shoved past saving. The hip's roll 109.7
    C after 30 s of walking, its throttle at 110.5. A four-bar's best over
    her ranges as 21 runs used them: the knee's -5..165 12 degrees at
    worst, to 130 24 at a lever of 0.9; the hip's -145..35 only at the joint;
    the ankle's 38 at 0.9, 27 at 1.2; the hip's roll 45 at 1.4 - the knee's
    way (a rod to 130, on its axis, a belt from the thigh) the user's call
    (asked 2026-10-02). Standing, 14 of her 16 kinds of drive reach 9-49 mm
    past her skin.
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
