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
  the strong leg when it holds again; the cut knee she rides out. Fallen,
  she lies curled
  as she landed: get up - onto her front (a roll, `tools/sim/getup_lab.py`
  has the moves that roll her), the push-up onto hands and knees, and from
  the kneel into the squat with the feet brought under her one at a time
  on a hand's support, the centre of mass placed over what bears her at
  each move (`arrival.over`), then the arrival's rise facing as she lay
  (its yaw and the walker's heading are ready). The scoreboard counts the
  time down. The arms take a forward fall late (the head touches once):
  the elbows to yield under the shoulders instead of the shoulders folding.
  The
  shoves' findings stand: toward the standing foot the side step ends with
  the capture point 27 cm ahead and no foot there; shoved at 0.30 of the
  stride the landing latched at 0.7 of the swing sits 14 cm inside the
  capture point. The torso's counter and the damping are off (the counter
  nods the torso, the damping fells the walk at 0.9); the soles light
  sneakers. The look: the pelvis surges 65 mm fore-aft and bobs 18 mm a
  stride - the collision at touchdown; push off before the landing (the
  trailing heel up as the other foot strikes) with the landing come as the
  body falls, not on the phase's clock; the stance knee 18-21 degrees at
  mid-stance from the height target latched 10 mm low at each landing (the
  trailing ankle's 2-degree droop under the push-off: a feedforward, or a
  dead band on the latch). The walk begins from a lean, the walker taking
  her mid-swing; the softer soles that felled the old first stride are
  untried on it, then a compressible sole layer for more give than a
  contact's.
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
