# TODO

State as of 2026-09-01.

| | Value |
|---|---|
| `run_tests.ps1 -All` | 2187 checks, 24 suites |
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
| The NTC and the DC link ride the injected sequence as rank 2, so the thermal observer keeps its thermometer under the drive | PROTOCOL, *Device 4* |
| The commissioning: AFE noise floor, sample point, offsets, gain mismatch, dead time, L map, lambda, budget, gains, decision, verification - on the stand-in end to end, on the bench as far as the AFE | ARCHITECTURE, *Host*; `tools/commission.py` |
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

0. **The IMU and the shaft angle inside a DAQ record, and the IMU on
   hardware.** `catalogue()` lists orientation, acceleration, rotation
   rate, magnetic field and shaft angle and refuses them: they read
   through their own subsystems, and carrying them in a record is a wire
   format that does not exist. The constraint is named - the channel mask
   is a `u16` with ten analog channels in it, so fourteen more fields need
   a SECOND appended mask, not a wider one, which would be a MAJOR - and
   they have to be snapshots rather than sums, since a summed quaternion
   means nothing. **And none of the IMU work is measured**: the three
   vectors, the multi-feature re-apply and the MINOR 6 reply are built and
   host-tested only, because the board has had no power since
   2026-09-01. That is the subsystem CLAUDE.md records as six defects and
   four dead hardware hypotheses, so the numbers wait for the rail.
1. **The drive on a motor.** Everything past the AFE step of
   `tools/commission.py` needs current through a winding: the AFE patch
   first, then a motor on the bench, then the sign check, the dead-time
   sweep, the L map, lambda, and a verification run. The board's numbers
   are placeholders until then (board_cal.c says so).
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
