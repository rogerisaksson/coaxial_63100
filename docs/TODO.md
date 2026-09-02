# TODO

State as of 2026-09-02.

| | Value |
|---|---|
| `run_tests.ps1 -All` | 2357 checks, 25 suites |
| Debug build | 0 warnings; the drive's interrupt path and the HAL ADC files at `-O2`; the I-cache on, the D-cache off |
| FLASH / DTCMRAM | 158 728 B (8 %) / 49 856 B (38 %) - `build_and_flash.py` prints it |
| Protocol | MAJOR 2, MINOR 6 |
| Firmware | 1.6.0 |
| Calibration record | CAL_VERSION 8, 45 parameters, op 8 pages them |

## What runs

Every number here lives in the document that owns it - this says what works,
not what it measured.

| | Where the working is |
|---|---|
| TIM1 armed on request, all six gates at their idle level, break on PE15 | HARDWARE, *Gate drive* |
| The synced current path: TRGO2 to three injected groups, sample point anywhere in the period, the DC link on ADC3 rank 2 | FINDINGS, *The injected triple*; *Two injected ranks need scan mode* |
| KEEPALIVE from the main loop, above every branch | HARDWARE, *STO* |
| `0x6E` device 7 ties the cycle counter to a host clock, against UTC rather than this PC | PROTOCOL, *Device 7* |
| The link at 89.8 % of its bitrate for a full block, 4.5 % for a ping | FINDINGS, *What the transport was spending* |
| The thermal observer: ten nodes, drivers and phases per leg, an SOA budget in flash | PROTOCOL, *Device 8* |
| Rails reference counted, and who holds them on the wire | PROTOCOL, *Device 9* |
| Switching into a load: one leg at a duty against another held low, or the pair swapped every PWM period by the board (`0x6E` device 4 op 10), 2-50 %, 25-31 V, up to 60 s | CLAUDE.md *Scope*; FINDINGS, *The pair alternates* |
| The drive: current loop, injection, observer, I/f, polarity pulse - idle 1 780 cycles a period; sensorless on the model, spinning, the interrupt ends 12.3 us of 20 after the trigger | PROTOCOL, *Device 10*; FINDINGS, *The caches were off* |
| The model as the drive's source, and ROTOR OBSERVER on the chooser: the observer watched against a rotor whose angle is known, AFE off, no motor - locked to 0.002 rad, spun to 441 rad/s with 0.009 rad of error | PROTOCOL, *Device 10*; `tools/show_rotor_observer.py` |
| One machine in one place: the PMSM, the propeller law, the 5230SL off its two datasheets (24N28P, and 3.0 A at 44.4 V giving b = 1.71e-4, 4.3x the guess it replaced) and the stand-in's own | ARCHITECTURE, *Host*; `coaxial/motor.py` |
| System identification with a **per-parameter** trust: R, Ld, Lq, lambda by least squares in dq, and Lq flagged untrusted on a V/f run at -73 % because its column is collinear with lambda | ARCHITECTURE, *Host*; `coaxial/sysid.py` |
| The firmware's own observer driven to its limit on this machine: 45 A of startup torque holds, 50 A stalls in the handover, and the PLL works between 150 and 332 Hz - the Kalman fixed point sits at the top edge, not the middle | FINDINGS, *The rotor observer's limit*; `tools/observer_run.py` |
| The stand-in's virtual source turning a real rotor, so a chain built against it can watch an estimate track: torque from its own dq solution, and a type-2 PLL's `alpha / wn^2` lag - a tenth of the torque lags a tenth as much, a fifth of the bandwidth 25.0x | `coaxial/simulated.py`; `test_simulated.py`, *virtual rotor* |
| The propeller from rest to 6717 rpm and back, against Hobbywing's own 22 points - and the disagreement it turns up: 28.3 V of phase demand where linear SVM off 10S makes 21.4, on a stand that reached those rpm on 10S | `notebook_examples/propeller_sweep.ipynb` |
| The host speed loop closed over the model and identified back out of its own run: R, Ld, lambda recovered inside 9 %, and the two alignment lessons the fit taught (half a period of angle advance, sample before advance) written into `loop.py` | `notebook_examples/speed_loop.ipynb`; ARCHITECTURE, *Host* |
| The firmware's control law Monte Carlo'd over the 23-63 V link sweep, 6 240 runs, one process per core: a controller schedule per link voltage, zero trips over 48 fresh plants each, and the sensorless floor measured - back-EMF alone loses the rotor at a median 24-69 rpm; with injection every descent reaches rest locked | `notebook_examples/foc_montecarlo.ipynb`; `tools/montecarlo.py` |
| The NTC and the DC link ride the injected sequence as rank 2, so the thermal observer keeps its thermometer under the drive | PROTOCOL, *Device 4* |
| The commissioning: AFE noise floor, sample point, offsets, gain mismatch, dead time, L map, lambda, budget, gains, decision, verification - on the stand-in end to end, on the bench as far as the AFE | ARCHITECTURE, *Host*; `tools/commission.py` |
| The drive as three motion verbs on the stand-in's own rotor: a stepper that slews and rings, a servo over the A1335 correcting between moves (the per-pass loop pumps the aliased ring - measured), a sensorless velocity loop; two position notebooks and four application missions - quad lane, wing cruise power, a two-joint arm on one bus, the precision hold's resolution/repeatability/stiffness | `coaxial/motion.py`; `notebook_examples/position_servo.ipynb`, `position_and_sensorless.ipynb`, `app_*.ipynb` |
| The bench-day pipeline as one notebook: commission, identify, Monte Carlo a robust tune for exactly that machine at its measured link, write the record, and the drive verifies itself - the stand-in's observer at 2.3 deg under the searched tune against 11.2 under the defaults | `notebook_examples/auto_tune.ipynb` |
| The clock-closed daq record: the converter free-running, a record closed on the interval carrying its own sample count - 33 to 89 sweeps a window, and the mean 0.008 % off an independent burst | FINDINGS, *The accumulator closes on the clock* |
| The anti-alias chain: a tone generated on the board, filtered, decimated and read back - in band within 0.2 % of the design, an alias folded onto the same output frequency stopped at -261.6 dB, nothing dropped | FINDINGS, *A known tone through the whole path*; `tools/daq_integrity.py` |
| The link at **73 % of 115200** with the chain running: a read that stops on its own known length, a reader that waits for a reply's worth, and a board that samples while the UART drains | FINDINGS, *Where the 115200 line actually went*; *The board spent 72 % of its loop* |
| A rung change that does not show in the data: the new coefficients primed to where the old ones left the signal - 8 changes, nothing past 59 codes in a run spanning 36742 | FINDINGS, *A rung change was visible in the data* |
| The acquisition front door by name: `catalogue()`, `configure('phaseU', 'NTC')` or a sliced list, `read(-1)`, and records with `start_time`, `dt`, `samples` and `channel_name` | `host/README.md`, *Acquisition, end to end* |
| The host stack at **87 % of an emulated 10 Mbit/s** - 44 us of host per transaction, so the library is not what limits a fast link | FINDINGS, *The stand-in was its own benchmark, twice* |
| The IMU's three vectors beside the quaternion, and four features held at once instead of one | PROTOCOL, *Devices 0 and 1* |

**USB is configured and nothing sits on it.** OTG_FS device, no device class,
so a host sees one that fails enumeration. Nothing depends on it.


## What blocks the gate drivers

**The STO chain has not released, so PWM cannot be enabled.** Proven: clearing
the break latch and enabling in the same round trip leaves the latch set again,
because PE15 is still low. Two independent conditions are needed and neither is
met - a pilot tone from a master on RS485, and the KEEPALIVE pump. Only the
second runs.

On the bench the break is bypassed instead - `switch.py` and `pulse.py` do it
by name - which is how 2026-08-30's switching into an 8 ohm load ran, at
25-31 V and 2-50 %. The chain itself still has not released.

**Switching and measuring are mutually exclusive on this board** until the
AFE gate is patched: AFE_ON high unpowers the drivers. Every step of the
commissioning past the AFE noise floor therefore runs dry here, and says
`measured: False` when no current answered.

Nothing has run near 63 V or 100 A. One number this board reports has been
measured against an instrument: the DC link, spanned against a DMM on
2026-08-30 (31.04 read, 30.05 true, gain -32 418 ppm on channel 5, saved).
Nothing else has - invariant 7.

## Next, in order

0.5. **The link above 115200.** The `.ioc` carries 9216000 on the RS485
   pair and the firmware sets 115200 at init; every streaming figure in
   FINDINGS is line-limited, and the write-class floor after the ack
   shape is the two spec silences. In order: the THVD1450's rated ceiling
   off its DATASHEET (not in this repository yet), the debug probe VCP's
   real ceiling measured, a `link_baud` calibration parameter applied at
   init with 115200 the default, and both ends' t3.5 re-derived - the
   spec fixes 1.75 ms above 19200, which at 921600 is 150 character
   times of silence per frame, paid twice per transaction. A baud change
   with no board attached is blind, so this waits for the bench.

0. **Sensor fields on hardware.** The wire format exists now (MINOR 7,
   2026-09-02): a second appended `u16` mask, four-`i16` snapshots per
   sensor between the pins and the count, software clock only. Built,
   zero-warning compiled, and host-tested against the stand-in - the
   shaft angle streams beside the currents off the same virtual rotor,
   and `frame(scaled=True)` puts degrees beside amperes. NOT measured:
   like the IMU's MINOR 6 work below, the board has had no power since
   2026-09-01, so the first bench day runs `test_daq_api`'s sensor check
   against real records and the position_and_sensorless notebook past
   its link-rate Nyquist ceiling. **And none of the IMU work is
   measured**: the three vectors, the multi-feature re-apply and the
   MINOR 6 reply are built and host-tested only. That is the subsystem
   CLAUDE.md records as six defects and four dead hardware hypotheses,
   so the numbers wait for the rail.
0b. **Prove `tare()` end to end on hardware.** `board.calibration.tare()`
   measures and calls `compensate()`, which writes the record where
   invariant 7 says a conversion lives, and `frame(scaled=True)` applies
   `offset_raw` then `gain_ppm`. It now works against the stand-in - a
   phase reads exactly 0.00 A after a tare, and the mean of the three fell
   from 43 A to about 1 - but the stand-in is a model this repository also
   wrote. What is left there is the phases genuinely rotating between the
   tare and the read, which is a live input and not something a tare
   chases.

0c. **A flake with no reproduction.** `test_simulated`'s `show_desk draws
   simulated` failed once under a full offline run and passed every
   attempt since, including three in isolation and several full gates.
   `plan()` now spends two measurement passes of several seconds at
   startup, which is the obvious suspect and is NOT evidence. Nothing has
   been changed for it: fixing an unreproduced flake is guessing, and this
   file is where a guess would otherwise become a habit.
1. **The drive on a motor.** Everything past the AFE step of
   `tools/commission.py` needs current through a winding: the AFE patch
   first, then a motor on the bench, then the sign check, the dead-time
   sweep, the L map, lambda, and a verification run. The board's numbers
   are placeholders until then (board_cal.c says so).

   **What is ready for that day, and what it is worth.** The machine is
   described (`motor.py`), the recovery is written and reports its own
   uncertainty (`sysid.py`), and the observer has been driven to its limit
   against the firmware's C (`tools/observer_run.py`). All of it rests on
   `PLATINUM_5230SL`, whose R, Ld, Lq and J are size-class estimates -
   `measured=False`, and `Lq` is both the least trustworthy and the one
   the injection observer lives on. Two independent results already point
   at it: `sysid` cannot see it without `di/dt`, and the notebook's
   voltage demand exceeds what the thrust stand evidently managed on the
   same pack. **The first current through a winding settles both.**
2. **Move the sensor polls off the blocking path.** The worst gap is 163 µs,
   inside the latch's ~200-400 µs, but the edge rate is 36 kHz against the 100
   kHz asked for. Measured by holding each in turn: the **A1335 costs 42 µs per
   loop iteration, the IMU 0.5 µs**. Converting the angle packet to
   interrupt-driven SPI is the cheap half - a fixed 4-byte frame with chip
   select held across it. The IMU's SHTP path is header-then-body with a
   variable length and has already cost six bugs; it deserves its own change.
3. **Cinj and Clevel cannot be sampled asynchronously** - apparent duty tracks
   the sample rate. Take them through the injected group, or with a longer
   sampling time. FINDINGS has the table.
4. **Replace the conformance check `PE15 follows AFE_ON`.** It reads a pin the
   MCU does not drive and changes meaning the moment the STO chain releases.
   Replace it before the supply is switched on, not after.
5. **Dead time on a scope.** The OCP trim (FINDINGS) bounded it from one
   side - 33.7 ns held, 29.5 tripped - but that is one data point at one
   duty on a cold dry board, against a simulated worst-corner need of 65
   ns. The skew parameter is untested and 0: the one experiment ran on a
   board that was resetting between steps.
6. **Double-pulse at the voltage ladder.** An inductor on the bench, a
   scope on the phase node, `pulse.py`'s path: t_r/t_f, overshoot
   against the FETs' 100 V class, dead-time adequacy and body-diode
   recovery, stepped 20 -> 31 -> 45 -> 63 V - one stress variable at a
   time. Same rig as item 5, three measurements in one probe setup;
   op 10 already makes the alternating train.
7. **The observer's losses do not scale with temperature.** `rds_on`
   is 1.8 mOhm flat (`Thermal/Src/thermal.c`); a 100 V Si FET's tempco
   is ~+0.6-0.8 %/K, so a 100 C junction conducts at ~1.5-1.7x the
   model - under-estimated exactly where margins thin. First order:
   `rds_25 * (1 + alpha * (Tj - 25))` fed back from the leg's own node
   estimate, alpha off the datasheet, verified against the camera in a
   soak.
8. **The data cache.** Off, with the instruction cache on since
   2026-08-31: `Board_CalSave` reads the sector back through a pointer
   after programming and would need `SCB_InvalidateDCache_by_Addr` on it
   first. Everything in DTCM is uncached either way; the win is flash
   literal pools and .rodata on the interrupt path. Measure before and
   after with device 10 op 0's `cyc_*` and `exit_ticks_max`.
9. **The anti-alias chain on real converter samples.** Proven end to
   end against a generated tone (FINDINGS, *A known tone through the
   whole path*) - what it has never seen is the ADC: the tone stands
   in for it, so nothing here says the front end's noise folds the way
   the design predicts. Needs the AFE on, a signal on a channel, and
   the same two passes.
10. **A USB device class**, if USB is to do anything.
11. **A cycle-counted duty.** A hold's length is the link's: the shortest is
    one write round trip, ~800 periods, and 100 ms asked for is 93-108 at the
    FETs. A period count on the duty op, decremented in TIM1's update ISR and
    zeroing the compares at zero, makes 10 ms exactly 500 cycles. Op 10
    `alternate` (2026-08-30) shows the ISR already owns the compares.

## Standing

- `electronic_simulations/motor_inverters/half_bridge.asc` has three `mc()`
  calls with no `run`/`val` wrapper, unlike `amplifiers.asc` and `hot_swap.asc`.
  Running that file randomises trace inductance and the shunt.
- CubeMX rewrites `Core/Src/main.c` to CRLF on every generation. Git normalises
  it back; byte-level edits must match `\r\n`.
- Phase W's noise floor is a third above U and V (74.9 against 55 codes rms,
  2026-08-31). Not chased.
