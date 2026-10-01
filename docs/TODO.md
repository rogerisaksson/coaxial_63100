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
  + The scoreboard in three: a walk scored on its smoothness and energy, a
    fall ignored; a stumble, slip or push on the parry, its fall the
    heaviest cost; a fall past saving on the landing's impulse.
  + Past saving - the page's P push, 120 N for 0.12 s, held 0 of 48 - a leg
    out and down into a superhero crouch, caught on a hand and a knee, not
    falling stiff; its landing in a suite test. A lace holding her trailing
    foot she dives onto her hands, legs straight: the knees under first.
  + The scoreboard's events held: the hole 53 %, the rug 51 %, the lace 26 %
    (82.5 % in all, HEAD 75.8; 2026-10-01). The walk at 1.0 strides/s falls
    in 3 runs of 3 (HEAD 2 of 3).
  + The obstacles' contacts stiffer: toes 20 mm into the sill for 35 ms,
    fingers 26 mm into the floor (MuJoCo's 0.02 s give).
  + The stairs fell her at the first riser since 855c87c: each riser met at
    0.85-0.9 of a swing, past TRIP_LATE, the foot put down short, the other
    striking the step at 1.4 kN; a step up for a late stub wanted; the get-up
    on the stairs untried. Kneeling over the hole's edge, one knee 3 cm down,
    knees under and sitting back roll her 50-89 degrees, 5 tries of 5.
  + Lying fallen she draws 794 W, walking 482: each ankle 135 W of copper
    at 135 N m, its peak, though `falls.SHORT_FALLING` shorts it.
  + A softer walk: fewer strikes (1272 N), less power (work 254 W, copper
    160 W of the 482 drawn). The ears bob 11 mm a stride and go 56 mm fore
    and aft, the pelvis's 30 doubled by the torso's 3 degrees of pitch; the
    stance knee 15-20 degrees through mid-stance, latched 10-12 mm low at
    each landing. To try: a stiffer spine drive or a rightly signed gyro lead
    on the torso; a shorter stride at a higher cadence; a landing that does
    not sag, on the ball (LAND_DEG below 0 under the landing's cost);
    softer soles, then a compressible sole layer; the pendulum between the
    ears placing the next step (PEND_K fell at 1.5, 2026-09-27).
  + The halt falls in its settle wherever tried: her centre of mass stands
    off the feet's line as it takes over.
  + No abrupt moves getting up: the neck 921 deg/s sitting back and 823
    lifting, a shoulder 577-622 rolling and in the squat; setpoints jump at
    the hand-offs, the waist 1.5 deg a pass into the unfold, a knee 8 at the
    first step.
  + The get-up faster (21.5 s from the fall to walking); the loop
    profiled and made data-oriented; the arrival's runaway guard.
  + The model carries a tenth of the rotors the cycloids show
    (`physics.REFLECTED`): the walk to be tuned on them; M's and S's motors
    are estimates.
  + Her boards on their buses (`machine.buses`): an emulated limb in a
    process's place on its port and block; the firmware's map wanting the
    walk's registers (`machine.rtu`); the IMU on the axis bus, not the
    world's own reading.
  + The planner's server after LOCAL_TRIES local failures.
  + `machine/` in subpackages.
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
