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
- **STO chain**: circuit change, pilot tone sent on RS485, Cinj/Clevel with and
  without it, interlock thresholds from those readings, one arm with neither
  bypass (`tools/bench/sto_probe.py`).
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
- Gynoid (`machine.walker`): balance is feedback patched onto a kinematic
  plan the pendulum does not follow, the rise chaotic, every side shove
  fatal. A reference that is the pendulum's: the CoM from the footsteps by
  the divergent component of motion (Englsberger's DCM walking), stance and
  swing switched on touchdown, the steps adjusted by the DCM; then the
  torso's counter and the damping (off: stir 4.3 -> 1.7 mm). Scored by
  `tools/sim/gait_montecarlo.py`: held 85 %, stir 4.1 mm to beat.
- The meter under the drive: `read_index` serves the NTC and the DC link
  from the latched sample, the phases could join them; a software sweep over a
  channel the drive locks out waits without a word.
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
