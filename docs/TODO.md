# TODO

State as of 2026-08-27.

| | Value |
|---|---|
| `run_tests.ps1 -All` | 1663 checks, 17 suites |
| Debug build | 0 warnings |
| FLASH / DTCMRAM | 120 732 B / 14 984 B |
| Protocol | MAJOR 2, MINOR 7 |

## What runs

**TIM1 is configured and the bridge is held off.** Centre-aligned, ARR 2375
off 237.5 MHz = 50.000 kHz exact, RCR 1, DTG 19 = 80.0 ns, break on PE15
active low, AOE off. `Board_PwmInit()` starts the counter with MOE clear and
CCxE set, so OSSI drives all six outputs to their idle level. Read back off
the silicon over SWD, not inferred.

**The synced current path works.** TIM1 TRGO2 (OC4REF) → three injected
groups → ADC3's interrupt latches the triple. Measured 49976 triples/s
against 50 kHz, zero overruns, and the injected values agree with the meter
inside its own noise. The sample point is CCR4 and moves anywhere in the
period; the handler reads `CNT` a constant 4.06 µs later. FINDINGS has the
sweep.

**KEEPALIVE runs from the main loop.** PA10, toggled above every branch.
214 kHz idle, 124 kHz while the host polls Modbus.

**USB is configured, and nothing sits on it.** OTG_FS device, interrupt at
priority 5, 48.000 MHz from PLL3Q off the crystal. `MX_USB_OTG_FS_PCD_Init`
runs at boot; there is no device class, so a host sees a device that fails
enumeration.

**`0x6E` device 4 puts the bridge on the wire.** State, PWM on/off, duty,
sync arm/disarm, sample point, clear break — `Comms/Src/cmd_bridge.c`,
`host/coaxial/bridge.py`, mirrored in `simulated.py`.

## What blocks the bridge

**The STO chain has not released, so PWM cannot be enabled.** Proven:
clearing the break latch and enabling in the same round trip leaves the
latch set again, because PE15 is still low. Two independent conditions are
needed and neither is met — a pilot tone from a master on RS485, and the
KEEPALIVE pump. Only the second is running.

Nothing has run near 63 V or 100 A. No number this board reports has been
measured against an instrument — invariant 7.

## Next, in order

1. **Move the sensor polls off the blocking path.** The worst gap is now
   163 µs, inside the latch's ~200-400 µs, but the edge rate is 36 kHz
   against the 100 kHz asked for. Measured by holding each in turn: the
   **A1335 costs 42 µs per loop iteration, the IMU 0.5 µs**. Converting the
   angle packet to interrupt-driven SPI is the cheap half — a fixed 4-byte
   frame with chip select held across it. The IMU's SHTP path is
   header-then-body with a variable length and has already cost six bugs;
   it deserves its own change, not a rider on this one.
2. **Cinj and Clevel cannot be sampled asynchronously** — apparent duty
   tracks the sample rate. Take them through the injected group, or with a
   longer sampling time. FINDINGS has the table.
3. **Replace the conformance check `PE15 follows AFE_ON`.** It reads a pin
   the MCU does not drive, and changes meaning the moment the STO chain
   releases. Replace it before the supply is switched on, not after.
4. **Dead time is simulated, not measured.** 80 ns comes from a 59.4 ns
   worst-corner gate overlap plus the 2EDL8034's 6 ns TDMOFF. Nothing has
   been on a scope.
5. **A USB device class**, if USB is to do anything.

## Standing

- `electronic_simulations/motor_inverters/half_bridge.asc` has three `mc()`
  calls with no `run`/`val` wrapper, unlike `amplifiers.asc` and
  `hot_swap.asc`. Running that file randomises trace inductance and the
  shunt.
- CubeMX rewrites `Core/Src/main.c` to CRLF on every generation. Git
  normalises it back; byte-level edits must match `\r\n`.
