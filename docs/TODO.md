# TODO

State as of 2026-08-29.

| | Value |
|---|---|
| `run_tests.ps1 -All` | 1965 checks, 21 suites |
| Debug build | 0 warnings |
| FLASH / DTCMRAM | 146 064 B (7 %) / 48 688 B (37 %) - `build_and_flash.py` prints it |
| Protocol | MAJOR 2, MINOR 0 |
| Firmware | 1.5.0 |

## What runs

Every number here lives in the document that owns it - this says what works,
not what it measured.

| | Where the working is |
|---|---|
| TIM1 armed on request, all six gates at their idle level, break on PE15 | HARDWARE, *Gate drive* |
| The synced current path: TRGO2 to three injected groups, sample point anywhere in the period | FINDINGS, *The injected triple* |
| KEEPALIVE from the main loop, above every branch | HARDWARE, *STO* |
| `0x6E` device 7 ties the cycle counter to a host clock, against UTC rather than this PC | PROTOCOL, *Device 7* |
| The link at 89.8 % of its bitrate for a full block, 4.5 % for a ping | FINDINGS, *What the transport was spending* |
| The thermal observer: ten nodes, drivers and phases per leg, an SOA budget in flash | PROTOCOL, *Device 8* |
| Rails reference counted, and who holds them on the wire | PROTOCOL, *Device 9* |
| Switching into a load: one leg at a duty against another held low, or the pair swapped every PWM period by the board (`0x6E` device 4 op 10), 2-50 %, 25-31 V, up to 60 s | CLAUDE.md *Scope*; FINDINGS, *The pair alternates* |

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

Nothing has run near 63 V or 100 A. One number this board reports has been
measured against an instrument: the DC link, spanned against a DMM on
2026-08-30 (31.04 read, 30.05 true, gain -32 418 ppm on channel 5, saved).
Nothing else has - invariant 7.

## Next, in order

1. **Move the sensor polls off the blocking path.** The worst gap is 163 µs,
   inside the latch's ~200-400 µs, but the edge rate is 36 kHz against the 100
   kHz asked for. Measured by holding each in turn: the **A1335 costs 42 µs per
   loop iteration, the IMU 0.5 µs**. Converting the angle packet to
   interrupt-driven SPI is the cheap half - a fixed 4-byte frame with chip
   select held across it. The IMU's SHTP path is header-then-body with a
   variable length and has already cost six bugs; it deserves its own change.
2. **Cinj and Clevel cannot be sampled asynchronously** - apparent duty tracks
   the sample rate. Take them through the injected group, or with a longer
   sampling time. FINDINGS has the table.
3. **Replace the conformance check `PE15 follows AFE_ON`.** It reads a pin the
   MCU does not drive and changes meaning the moment the STO chain releases.
   Replace it before the supply is switched on, not after.
4. **Dead time on a scope.** The OCP trim (FINDINGS) bounded it from one
   side - 33.7 ns held, 29.5 tripped - but that is one data point at one
   duty on a cold dry board, against a simulated worst-corner need of 65
   ns. The skew parameter is untested and 0: the one experiment ran on a
   board that was resetting between steps.
5. **A USB device class**, if USB is to do anything.
6. **A cycle-counted duty.** A hold's length is the link's: the shortest is
   one write round trip, ~800 periods, and 100 ms asked for is 93-108 at the
   FETs. A period count on the duty op, decremented in TIM1's update ISR and
   zeroing the compares at zero, makes 10 ms exactly 500 cycles. Op 10
   `alternate` (2026-08-30) shows the ISR already owns the compares.
7. **`0x41`'s description still says "no timer, no PWM, no gate drive"** -
   `cmd_board.c`, one string, stale since TIM1 armed. The board should not
   describe itself wrongly; a reflash fixes it.

## Standing

- `electronic_simulations/motor_inverters/half_bridge.asc` has three `mc()`
  calls with no `run`/`val` wrapper, unlike `amplifiers.asc` and `hot_swap.asc`.
  Running that file randomises trace inductance and the shunt.
- CubeMX rewrites `Core/Src/main.c` to CRLF on every generation. Git normalises
  it back; byte-level edits must match `\r\n`.
