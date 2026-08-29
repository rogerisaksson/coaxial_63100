# TODO

State as of 2026-08-29.

| | Value |
|---|---|
| `run_tests.ps1 -All` | 1769 checks, 18 suites |
| Debug build | 0 warnings |
| FLASH / DTCMRAM | 145 204 B (7 %) / 48 544 B (37 %) - `build_and_flash.py` prints it |
| Protocol | MAJOR 1, MINOR 28 |
| Firmware | 1.4.1 |

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
| The thermal observer: one measurement, five estimates, an SOA budget in flash | PROTOCOL, *Device 8* |
| Rails reference counted, and who holds them on the wire | PROTOCOL, *Device 9* |

**USB is configured and nothing sits on it.** OTG_FS device, no device class,
so a host sees one that fails enumeration. Nothing depends on it.


## What blocks the gate drivers

**The STO chain has not released, so PWM cannot be enabled.** Proven: clearing
the break latch and enabling in the same round trip leaves the latch set again,
because PE15 is still low. Two independent conditions are needed and neither is
met - a pilot tone from a master on RS485, and the KEEPALIVE pump. Only the
second runs.

Nothing has run near 63 V or 100 A. No number this board reports has been
measured against an instrument - invariant 7.

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
4. **Dead time is simulated, not measured.** 80 ns comes from a 59.4 ns
   worst-corner gate overlap plus the 2EDL8034's 6 ns TDMOFF. Nothing has been
   on a scope.
5. **A USB device class**, if USB is to do anything.

## Standing

- `electronic_simulations/motor_inverters/half_bridge.asc` has three `mc()`
  calls with no `run`/`val` wrapper, unlike `amplifiers.asc` and `hot_swap.asc`.
  Running that file randomises trace inductance and the shunt.
- CubeMX rewrites `Core/Src/main.c` to CRLF on every generation. Git normalises
  it back; byte-level edits must match `\r\n`.
