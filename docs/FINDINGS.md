# Hardware & Firmware Findings

## Resolved Defects

* **`ADC_PCSEL` Accumulation:** The HAL blindly ORs `PCSEL` without clearing it, leaving every configured channel permanently connected. DC bus noise dropped 7x after explicitly clearing it pre-configuration.
* **Silent Zero on a Failed Conversion:** `ADC_ReadOneChannel` returned void and left `*outRaw` at 0 when `ConfigChannel`, `Start` or a 10 ms `PollForConversion` failed. On a differential channel code 0 is 0 V, so a failure was indistinguishable from a measurement. Now `bool` through all six `Board_*` readers; the noise and burst loops abort rather than fold a zero into the mean. `h_adc_burst` took on the value checks `h_adc_noise` already had, so its refusal is SERVER DEVICE FAILURE rather than ILLEGAL DATA VALUE.
* **Blind Differential Reads:** `ADC_ReadDifferentialVolts` hijacked whatever state the ADC was left in rather than configuring its own channel. The function was purged.
* **Modbus Qty 0 Ignored:** A valid request for zero items failed silently due to a lazy 7-byte minimum PDU check. Lowered to 6.
* **HSE Boot Warning:** Blindly rejected PLL1 even when sourced directly from HSE. Fixed.
* **String Formatting Exception:** Missing tuple parentheses in the MCP renderer swallowed outputs. Caught solely because the exception handler was cynically narrow. Keep it that way.
* **`port_state` Unstubbed in `test_link_diagnose`:** The suite stubbed `comports`, `connect` and `check_power` but not `port_state`, which opens a real port. The result depended on the bench: it passed with the probe connected and failed with it out, where COM4 read `busy` and the checklist stopped one step short of what the check asserted. Stubbed; the BUSY branch it had been reaching by accident now has its own check.
* **Calibration Rollback That Rolled Back Nothing:** `Board_CalSetParam`
  assigned first and, on a value that would divide by zero, reverted by
  reloading flash. On a board whose record has never been saved that reload
  fails and changes nothing, so the refused value stayed - `vref_uv` left at
  zero, every reading garbage, and the command still answering ILLEGAL DATA
  VALUE. Found by `test_conformance.py` re-reading the record after every
  refusal it provokes. Now validated before the assignment.

## LLM & Host Infrastructure

* **LLM OOM Crashes:** `llama-server` throws `std::bad_alloc` due to its own bloated prompt cache and checkpoints, not prompt length. Fixed by disabling `LLAMA_ARG_CACHE_RAM` and `LLAMA_ARG_CTX_CHECKPOINTS` in the daemon.
* **`.port` Read as a COM Port:** `SimulatedSession.port` is a bus label (`AX`), never `None`. `link_diagnose` guarded on `configured is None`, so a session that fell back to the stand-in spent 15 s on an SWD probe and then reported "Configured port AX: not among the ports above - the cable may be unplugged". The `simulated` marker that fixed the same mistake in `_interface` already existed. Step 4's closing advice also opened with "Powered" whatever step 1 concluded.
* **`check_power` Discarded Its Own Reading:** With no target the programmer takes 30.3 s - a second connect attempt at 8 MHz - against a 15 s budget, and `TimeoutExpired.stdout` already holds `Voltage: 0.00V`. The handler returned `None`. Now parsed from the killed run.
* **`-Ask` Pinned the Card:** A one-shot was exempt from the unload on prompt exit while the same script passes `--keep-alive 30m`, so four smoke tests left 8.4 GB resident with nobody at the prompt. `-Ask` now takes a list: one load, N questions, one release.
* **`--sections` Read Only Under `--match`:** `run_tests.py --live --sections tools` ran all three - `tools` and `all` both returning 176 checks in 255 s. Coverage tiers were unaffected; they set the sections directly rather than through the flag.
* **Model Hallucinations:** `llama3.1:8b` fabricates telemetry when tools fail and invents physical constants (NTC B=3950 instead of 3380). `gemma4:12b` remains the default; it is slower but declines to lie.

## Confirmed Behaviors (Not Defects)

* **JTAG Connect-Under-Reset:** Fails because ST-Link probes neglect TAP re-initialization. Hardware is innocent. Workaround: SWD or `SWrst`.
* **Halted Core Silences USART3:** An aborted JTAG/SWD reset leaves the core halted, killing serial comms. Use `mode=HOTPLUG`.
* **Phase V 0.85V Offset:** Isolated op-amp failure on a single board. Do not calibrate around broken hardware. Since the phase channels report amperes it reads as -52 A with nothing connected, which is what makes it hard to ignore - and `0x6E` device 3 op 3 would now make it vanish. Do not zero Phase V on this board.
* **NTC Bit-Exact "Anomalies":** Johnson noise is 120x below LSB. Bit-exact readings are physics, not a frozen register.
* **UART Overrun (ORE):** Latches and kills RX permanently. Now explicitly cleared via `ICR`.
* **ADC Offset Calibration:** Drifts ~100mV across boots because it runs against an unpowered reference (`AFE_ON` low).
* **Probe Readings Under Concurrent Use:** `--power` drives the ST-Link and `port_state` opens the VCP; neither says anything about the hardware while another process holds them. A concurrent `run_tests.ps1 -All` makes `--power` report `ST-LINK error (DEV_CONNECT_ERR)` in 2.3 s where the same call reads `0.00V` in 15.2 s once the bench is idle. A leftover `dbg.py --repl` makes `port_state` report `busy`, which is what the checklist then correctly says.

## Confirmed Behaviors (Not Defects) - continued

**`AFE_ON` (`PB2`) powers the IMU.** Measured 2026-08-26: with it off the
BNO08X answers reads and never acts on a write. With it on, then reset, then
`Set Feature 0x05` at 20 ms: 135 rotation vectors in 4 s, 51 distinct.

What the fault looked like on the way there, all of it wrong:

| Suspected | Ruled out by |
|---|---|
| SPI mode or bitrate | the advertisement reads back clean at 1.48 MHz, mode 3 |
| chip select not reaching the part | clocking with CS high gives `ff ff ff ff`, with CS low `14 01 00 00` |
| a shorted or held SPI2 pin | PB12..PB15 each drive high, drive low and follow both internal pulls |
| the buffer truncating the advertisement | 276 bytes into 320 |
| `PS0`/`WAKE` not wired | H_INTN answers a wake in 0-3 ms |
| the part in the bootloader | it reports SH-2 3.2.0, part 10004148 |

`product_id` looked like proof the link was two-way and was not: the part
sends an unsolicited product id response after every reset, so the answer was
in the queue whether or not the request arrived.

**A1335 on SPI4, brought up 2026-08-26.** Three things cost time, all of
them in code:

| Symptom | Actual cause |
|---|---|
| `HAL_SPI_Init` returned `HAL_ERROR` | `IS_SPI_HIGHEND_INSTANCE` names SPI1..3 only, and SPI4 refuses a data size above 16 bits. The 20-bit packet goes out as four 5-bit words - exactly twenty clock edges under one chip select |
| every register returned the previous one's value | the answer lags a frame. The address arrives on MOSI bits 17..12 while MISO has already shifted out bits 19..16, so a read is two packets. Asking TSEN, FIELD, TSEN in turn returned the previous register every time |
| the angle wandered with the board still | `FIELD` reads 3 gauss - there is no magnet in front of the part, and the angle is then noise. Not a fault |

Confirmed once the framing was right: `TSEN` 2471 counts = 308.9 K = 35.7 C,
`FIELD` 2 gauss, the poll loop at ~22,000 reads a second with no errors.

The register map is not in `datasheets/AngleSensor` - that datasheet defers
it to the Programming Manual. Addresses come from
`github.com/ScranchNew/Allegro-A1335-Sensor-library`, and `0x6E` device 1
op 5 sets which one the loop reads so a better address needs no rebuild. The
CRC field's width is documented and its polynomial is not, so it is reported
and never checked.

## Ruled Out

* **`Chat` Decomposes into Turn, Steering and Budget:** All three touch `client`, `history`, `io_log`, `language` and `last_channels`; steering owns one attribute alone and budget none, so three objects need a shared state struct all three hold. `debug.py` is 1544 lines of which `Chat` is 1088; moving the wording (112) and module helpers (105) out leaves 1327. No class ceiling in the structure suite either: a mixin split defeats one, and a module ceiling would only demand this refactor.
* **`-Wconversion` Finds Silent Truncations Here:** 113 warnings across all seventeen of our sources, 112 inside HAL/CMSIS headers they merely include, one ours - a `0U` ternary narrowing to `uint8_t`. The integer-heavy files are heavy in explicit casts. The flag now sits on the ten HAL-free sources, where it also guards invariant 1.
* **NTC Sample Time Too Short:** The 15nF capacitor provides the necessary charge.
* **DIFSEL Ignored:** It is written safely while the ADC is disabled.
* **VREF Sag at 475 MHz:** A coincidental 1% shift on the DC bus; other channels shifted completely asynchronously (up to 9.7%).
* **Prompt Length Causing OOM:** See cache mechanics above.

## Open Anomalies

* **Unpowered Calibration:** The proposed fix (waiting for `AFE_ON`) remains untested.
* **IN11 (`Cinj`) 9.7% Shift:** Explained by the schematic, not by the ADC. `Cinj` is the output of the RS485 pilot detector - a 1 kHz to 10 kHz band pass into a TLV3492 comparator pair (HARDWARE.md, Safe Torque Off). With no master injecting a pilot there is no signal, so the channel reads whatever the band pass makes of the board's own noise, and a frequency-dependent reading is what that looks like. Re-measure with a pilot present before treating any of it as drift.
* **DC Bus Read Discrepancy:** Two different read paths yield a persistent ~30 mV delta.
* **PE15 (`nFAULT`) Polarity:** Reads 0 (asserted) when the AFE is powered. Three explanations were open; the intended logic is now stated - high is normal, low is a fault - which **rules out inverted logic** and leaves a real fault or a supply pull. The firmware inverts nothing: `Board_Pe15` returns raw IDR and both the Modbus discrete input and `0x6D` pass it through, so the measurement is the pin's electrical level.

  Explained: the gate drivers are not powered yet. An open-drain output
  cannot pull low without a supply - it goes high impedance - so the pin
  floats, and a floating input next to a switching rail follows whatever is
  nearest. "Real fault" and "supply pull" turn out to be the same answer.

  The earlier reading, now confirmed rather than proposed: the pin is
  floating. A fault output is normally open drain, PE15 is configured `GPIO_NOPULL`, and nothing else on the board is known to pull it up - a floating input next to a switching supply will follow whatever rail is nearest, which is what "tracks `AFE_ON` inversely" would look like. **The conformance suite's "independent witness" that a coil write reached the pin may therefore be built on a floating input.**

  **The conformance suite's "independent witness" is therefore worthless as
  written** - it reads a floating pin and will change meaning the moment the
  drivers are powered, at which point nFAULT starts driving the line for
  real. That check needs replacing before the supply is switched on, not
  after it starts failing.

  What is missing to close it, from the schematic: **no MCU pin powers the
  drivers, and none should.** The supply is released by the Safe Torque Off
  chain on `STO.SchDoc`, unlocked by a common-mode pilot tone the master
  injects on the RS485 pair - see HARDWARE.md. Until a master is sending
  that tone the chain never releases, the drivers stay unpowered, and their
  open-drain nFAULT cannot pull the line either way.

  The 2EDL8034s and the FETs are still not in `s_parts`, so the bridge is
  invisible to `board_info kind=parts`.
