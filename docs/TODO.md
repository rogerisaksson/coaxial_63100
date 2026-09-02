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
  for the laminate.
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
  15.2 the camera saw; no measurement separates the legs.
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
* The occasional quiet link: open after 600 requests and four causes
  ruled out.
* `electronic_simulations` is a submodule with an SSH key on the bench
  machine and is not checked out here; `inverter.py` carries its
  traced constants.
* Nothing has run near 63 V or 100 A. No measured value at either is
  recorded anywhere in this tree.
