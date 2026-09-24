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
- **Drive**: no current has closed a loop through a winding.
  `tools/bench/commission.py` has run dry only. Record ids 15..44 (motor R, L,
  lambda, gains, injection, dead-time table) are placeholders.
- **SOA path** has never run on target: dry `budget()` over the wire, gate
  proof with a lowered ceiling, a load run. `Board_SyncMeanSquare` ISR cost
  unmeasured.
- **STO chain**: circuit change, pilot tone sent on RS485, Cinj/Clevel with and
  without it, interlock thresholds from those readings, one arm with neither
  bypass (`tools/bench/sto_probe.py`). All sessions so far armed with both
  bypasses.
- **Scope**: counted hold (MINOR 8), dead-time skew (record holds 0),
  `Q_RING` in `inverter.py`.
- **Thermal**: camera under load (`board_to_ambient` at high dT, per-leg
  `to_board`), a power step and the NTC's slope (leg capacity: burst budget
  is 0.22-0.67 s), a thermocouple on a winding. Ceilings for drivers,
  regulators, AFE and laminate are estimates.
- **Spans**: phase gain never spanned; DC link is the only spanned channel.
- Nothing has run near 63 V or 100 A.

## Host

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
