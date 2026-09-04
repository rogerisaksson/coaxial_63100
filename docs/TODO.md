# TODO

What is done and measured, what is written and dry-run only, and what
is still arithmetic. Every item names the file or record it lives in.

## Done and measured

* Every duty 1 to 100 % with the drivers powered, no trip, no overruns
  (2026-08-27).
* Pulses into an 8 ohm load at 25 and 31 V, 2 to 50 %, 26 runs, both
  directions (2026-08-30). `tools/pulse.py`.
* The alternate op, both triples on the scope (2026-08-30).
* The DC link spanned against a DMM, -32 418 ppm, saved (2026-08-30).
* Dead time 30 ns trimmed against the supply's OCP; 29.5 ns tripped it
  (2026-08-29).
* The thermal network fitted from the camera in four states
  (2026-08-28).
* The IMU stream, the angle sensor's registers, the three ports'
  counters, the request-length oracle, the reader thread's rates, the
  gate short probe.
* The drive ISR cost with the instruction cache and -O2: 2 922 cycles
  a period on the model source (2026-08-31).

## Written, dry-run only

* **The drive.** `drive/` behind `0x6E` device 10 is a dq current
  loop, HF injection, a Kalman-form PLL, I/f and a polarity pulse,
  host-tested against a motor model (`test_drive_core.py`) and stepped
  on the board with the drivers unpowered. No current has closed a loop
  through a winding. `tools/commission.py` is the procedure for when
  one can; every step has run dry on this bench and none against a
  motor.
* **The counted hold** (gate op 2 with a period count, MINOR 8): built
  2026-09-02, dry only. No counted hold has been scoped.
* **The dead-time skew** (`deadtime_skew`, gate op 9): a DTG rewrite
  each half-period. Not measured on a scope; the record holds 0.
* **The thermal envelope** acts (drops MOE at a ceiling) and the
  ceilings for the drivers, regulators and AFE are estimates - those
  datasheets are not in this tree. The board's 105 C is an estimate
  for the laminate. The FET's IS in the tree since 2026-09-04:
  `datasheets/mosfet/` gives Tj 175 C and Rth JC 0.69 K/W, so a 125 C
  ceiling on the copper is about 131 C at the junction - 44 K of margin,
  and the ceiling is conservative rather than optimistic.
* **The whole SOA path has never run on the target.** It builds clean -
  the derate, the soak joules, the reaction window, `Board_DriveDerate`,
  `Board_SyncMeanSquare` - and has never been flashed. Of the five-step
  validation only the build and the host-side suite are done; the dry
  `budget()` read over the wire, the gate proof with a lowered ceiling on
  a cold board, and a real load run are not.
* **`Board_SyncMeanSquare` costs an unmeasured amount of ISR.** Three
  int64 multiply-accumulates in the injected callback, against an
  interrupt the LOOP panel reports at 1620 cycles. `test_bench.py` at the
  bench is what would confirm it.
* **The motion verbs** (`stepper`, `servo`, `velocity`) and the four
  `app_*` notebooks run against the stand-in.
* **The commissioning's outputs** - `motor_r`, `ld`, `lq`, `lambda`,
  the gains, the injection, the dead-time table, `sigma_i`,
  `trigger_ticks` (record ids 15 .. 44) - are placeholders until a
  motor is on the bench. The injection is off and the trip sits at the
  rating.

## Still arithmetic

* The phase gain (3.5 mΩ x 4.5455) is traced off the schematic and has
  never been spanned; the DC link is the only spanned channel.
* `Q_RING` = 1.0 in `inverter.py` is assumed; the scope is the answer.
* `r_hotswap` 5 mΩ in the thermal losses is not measured.
* `die_over_node` for the MCU, 27 K, is assumed.
* The per-leg thermal spreading (45.6 K/W) is three times the lumped
  15.2 the camera saw; no measurement separates the legs. **Three
  lines of evidence now disagree about it** and FINDINGS has the
  arithmetic: the camera's zone tripled says 45.6, the NTC's own rise
  needs above 48 if the sensor is to sit below its source, and the
  datasheet's whole junction-to-air on a lesser board is 25.9. The
  likeliest odd input is the camera's board reference in the switching
  state, read off mixed copper and soldermask through an uncorrected
  emissivity.
* **The leg nodes' heat capacity is not measured and it sets the whole
  burst budget.** `thermal.c` always said so - "the parts' own are not
  measured" - and added "they only affect the settling", which stopped
  being true when the envelope started dividing by them. Silva 2022 puts
  the effective transient capacity at up to a third of the physical, so
  the 100 A burst on a driver node is a BAND: 0.22 s to 0.67 s, soak
  4.08 J to 12.25 J. A power step and the NTC's slope would settle it -
  the only one of these a transient can reach rather than an
  equilibrium, and `tools/pulse.py` already makes the step.
* The thermistor's element fraction (`ntc_sees_drivers` 0.30) comes off
  the pick and place by two-dimensional radial spreading, not off a
  measurement, and the campaign cannot measure it: its one switching state
  implies 1.05, which no passive body between two others can have. The
  same state now shows an 11.04 K residual, which is that inconsistency
  made visible instead of absorbed into a coupling.
* `NTC_TAU_S` is the geometric mean of the leg node's constant and the
  board's, 46.6 s. The model has no node for the local laminate the part
  is soldered into, so this is the pair it sits between standing in for
  a lag nobody measured.
* `board_to_ambient` is a correlation now - convection as the fourth
  root of the rise, radiation as `(T^2+T0^2)(T+T0)` - but it is still
  anchored at ONE measured point, 1.2 W over 10 K, and the 35 % radiation
  share at that point comes from a paper rather than this board.
* `RDS_ON` is the datasheet TYPICAL, 1.8 mOhm against a 2.1 max, so the
  envelope under-books a worst-case part by 17 %. The LTspice model this
  tree traces is the typical one, which is why the two are left agreeing.
* The lumped R-C class this model belongs to is worth about +/-10 %
  (`docs/papers`, against +/-5 % for a Fourier hybrid and +/-2 % for full
  3D CFD). Every unmeasured constant above is outside that band, so the
  method is not the limit here.
* `test_sensorless`'s overpowered-servo check returns instead of raising
  about one run in four, but only inside the full offline gate and never
  in six runs of that suite alone. Raising the load to 1.2 N.m did not
  fix it and twelve runs under CPU contention all raised correctly, so
  the wall-clock hypothesis is not established. The check now reports the
  angle that passed for a hold.
* The 5230SL's `r`, `ld`, `lq`, `j` and `b` in `motor.py` are
  estimates; the propeller curve is Hobbywing's stand, not this one.
* `BENCH_MOTOR` in `motor.py` and the drive defaults in the record are
  placeholders.
* The A1335's CRC polynomial is not in the datasheet in this tree, so
  the CRC is reported and not checked; the register map came from a
  reference implementation and the polled register is settable for
  that reason.
* The SH-2 report lengths came from CEVA's reference, not this
  tree's datasheet.
* `testline/plans/coaxial_63100_fct.yaml` carries placeholder limits.
* `CMD_LINK_SHARE_PCT` = 75 is for one host on the debug port; a
  populated RS485 segment has not been measured.
* The gate op 10 alternate has no period count; only op 2 does.
* PE15 reading 0 with the front end powered: what drives it is not
  established.
* `electronic_simulations` is a submodule with an SSH key on the bench
  machine and is not checked out here; `inverter.py` carries its
  traced constants.
* Nothing has run near 63 V or 100 A. No measured value at either is
  recorded anywhere in this tree.
