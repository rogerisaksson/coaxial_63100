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
- Gynoid (`machine.walker`): the swing foot lands on the capture point
  (`machine.capture`), a swap is a side step, the walk begun again after
  it. The rises and the walks hold. Scored by `tools/sim/gait_montecarlo.py`
  on the floor's events (`physics.World.terrain`): the hole and the loose
  rug fell her, the sill and the slip patch tip her 6-8 degrees: held 93 %,
  stir 3.5 mm. Reflexes, on the head's and the strong joints' inertial
  measurements: a foot that finds no floor reaches down and the floor is
  where it found it (the anchor's y is 0 now); a stubbed toe lifts higher
  and the body's fall is caught by the next step; a sliding foot
  re-anchored at once, the other foot down early. A knee folding under
  her (a drive derated hot): sink onto the strong leg and kneel on the
  weak one - a bent knee on the floor asks its drive nothing - and rise on
  the strong leg when it holds again; the cut knee she rides out. Fallen
  she gets up (`machine.getup`) on a plan the observer checks step by step
  (`machine.observer`, `machine.planner`: a local model's, a server's after
  LOCAL_TRIES - no server client yet - else its own): onto her front, the
  knees under, back on her heels, onto her feet, knees together; 13 of 13
  falls walk again 20-25 s after, the stairs untried. Falling, the waist
  turns her arms toward the fall (`falls.turn`); pads at the elbows, hips
  and knees (`figure.PADS`). The
  scoreboard counts the time down. The shoves' findings stand: toward the
  standing foot the side step ends with
  the capture point 27 cm ahead and no foot there; shoved at 0.30 of the
  stride the landing latched at 0.7 of the swing sits 14 cm inside the
  capture point. The torso's counter and the damping are off (the counter
  nods the torso, the damping fells the walk at 0.9); the soles light
  sneakers. The look: the ears bob 11 mm a stride and go 56 mm fore and
  aft, the head nodding 2.2 degrees - the pelvis 30 (the vault of a 1 m
  stride) doubled by the torso's 3 degrees of pitch; the stance knee 15-20
  degrees through mid-stance from
  the height latched 10-12 mm low at each landing (the sole's and the
  joints' give under the strike). Left to try: a stiffer spine drive or a
  rightly signed lead on the gyro for the torso; a shorter stride at a
  higher cadence for the same speed; a landing that does not sag; and the
  pendulum between the ears as the observer whose swing places the next
  step - a trip swings the bob ahead, the step goes out under it, a stomp
  or two, then the walk again (asked 2026-09-27; PEND_K fell at 1.5 - the
  sign and the gain against the bob's 2 s period to be worked out). A lace
  holding her trailing foot, she dives onto her hands with her legs straight
  behind: the knees under her before the hands, a catch on all fours. The
  walk lands on the ball, softly: the scoreboard's landing cost (the impact over
  30 ms, the touchdown's speed) searched with LAND_DEG below 0. The
  scoreboard's trials split as the suites are (`test_gynoid.py` on fantasy
  boards, `test_gynoid_faults.py`), the look's measures in its cost. Her
  boards run
  on their buses, a process a limb (`machine.buses`): an emulated limb takes
  a process's place on its port and block when the emulation is ready, and
  the firmware's map wants the walk's registers (`machine.rtu`: SETPOINT_REG,
  STATE_REG, GATE_REG); the IMU's reading is the world's own still, not a
  frame on the axis bus. Each board keeps its heat and says it (`machine.heat`,
  one drive for every joint until the drives are sized); the director eases
  the pace on it and arms a dropped board again. The halt falls in its
  settle wherever tried: her centre of mass stands off the feet's line as
  it takes over - stopped by the walker's own capture of it, a stop to cool
  a drive could stand. A lace snagged shoe to shoe (`World.lace`) trips her,
  a catch step tried and 5 of 6 taut ones fell her (`look.py --event lace`).
  Prone, the push-up and
  the dog leave the head and the torso on the floor, the arms too weak to
  lift them (`tools/sim/getup_lab.py` heels, bearwalk): the model's 40 and
  25 N m are a woman's, kept against the drives' 101 and 30 (the user,
  2026-09-30). The model carries a tenth of the rotors the cycloids show
  (`physics.REFLECTED`): the walk to be tuned on them; M's and S's motors
  are estimates. The walk begins from a lean, the body 8 degrees ahead of
  plumb, the walker taking her mid-swing and letting the lean out over 2 s,
  the pelvis with the torso; the weight goes 3 cm onto the left foot before
  the right lifts and the rest as it lifts, the stance hip rolled -2.5
  standing: a lift on the capture point (her centre of mass still moving
  left) failed past 6.5 cm, the first steps falling. The softer soles that
  felled the old first stride are
  untried on it, then a compressible sole layer for more give than a
  contact's. The scoreboard's spread leaves 0.5 of cost to chance; the arms
  from the forearm cost 1.0 more, on the slip at 0.9 strides/s.
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
