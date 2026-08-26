# Hardware & Firmware Findings

## Resolved Defects

* **`ADC_PCSEL` Accumulation:** The HAL blindly ORs `PCSEL` without clearing it, leaving every configured channel permanently connected. DC bus noise dropped 7x after explicitly clearing it pre-configuration.
* **Silent Zero on a Failed Conversion:** `ADC_ReadOneChannel` returned void and left `*outRaw` at 0 when `ConfigChannel`, `Start` or a 10 ms `PollForConversion` failed. On a differential channel code 0 is 0 V, so a failure was indistinguishable from a measurement. Now `bool` through all six `Board_*` readers; the noise and burst loops abort rather than fold a zero into the mean. `h_adc_burst` took on the value checks `h_adc_noise` already had, so its refusal is SERVER DEVICE FAILURE rather than ILLEGAL DATA VALUE.
* **Blind Differential Reads:** `ADC_ReadDifferentialVolts` hijacked whatever state the ADC was left in rather than configuring its own channel. The function was purged.
* **Modbus Qty 0 Ignored:** A valid request for zero items failed silently due to a lazy 7-byte minimum PDU check. Lowered to 6.
* **HSE Boot Warning:** Blindly rejected PLL1 even when sourced directly from HSE. Fixed.
* **String Formatting Exception:** Missing tuple parentheses in the MCP renderer swallowed outputs. Caught solely because the exception handler was cynically narrow. Keep it that way.
* **`port_state` Unstubbed in `test_link_diagnose`:** The suite stubbed `comports`, `connect` and `check_power` but not `port_state`, which opens a real port. The result depended on the bench: it passed with the probe connected and failed with it out, where COM4 read `busy` and the checklist stopped one step short of what the check asserted. Stubbed; the BUSY branch it had been reaching by accident now has its own check.

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
* **Phase V 0.85V Offset:** Isolated op-amp failure on a single board. Do not calibrate around broken hardware.
* **NTC Bit-Exact "Anomalies":** Johnson noise is 120x below LSB. Bit-exact readings are physics, not a frozen register.
* **UART Overrun (ORE):** Latches and kills RX permanently. Now explicitly cleared via `ICR`.
* **ADC Offset Calibration:** Drifts ~100mV across boots because it runs against an unpowered reference (`AFE_ON` low).
* **Probe Readings Under Concurrent Use:** `--power` drives the ST-Link and `port_state` opens the VCP; neither says anything about the hardware while another process holds them. A concurrent `run_tests.ps1 -All` makes `--power` report `ST-LINK error (DEV_CONNECT_ERR)` in 2.3 s where the same call reads `0.00V` in 15.2 s once the bench is idle. A leftover `dbg.py --repl` makes `port_state` report `busy`, which is what the checklist then correctly says.

## Ruled Out

* **`Chat` Decomposes into Turn, Steering and Budget:** All three touch `client`, `history`, `io_log`, `language` and `last_channels`; steering owns one attribute alone and budget none, so three objects need a shared state struct all three hold. `debug.py` is 1544 lines of which `Chat` is 1088; moving the wording (112) and module helpers (105) out leaves 1327. No class ceiling in the structure suite either: a mixin split defeats one, and a module ceiling would only demand this refactor.
* **`-Wconversion` Finds Silent Truncations Here:** 113 warnings across all seventeen of our sources, 112 inside HAL/CMSIS headers they merely include, one ours - a `0U` ternary narrowing to `uint8_t`. The integer-heavy files are heavy in explicit casts. The flag now sits on the ten HAL-free sources, where it also guards invariant 1.
* **NTC Sample Time Too Short:** The 15nF capacitor provides the necessary charge.
* **DIFSEL Ignored:** It is written safely while the ADC is disabled.
* **VREF Sag at 475 MHz:** A coincidental 1% shift on the DC bus; other channels shifted completely asynchronously (up to 9.7%).
* **Prompt Length Causing OOM:** See cache mechanics above.

## Open Anomalies

* **Unpowered Calibration:** The proposed fix (waiting for `AFE_ON`) remains untested.
* **IN11 (`Cinj`) 9.7% Shift:** Unexplained frequency-dependent drift between 75 and 475 MHz.
* **DC Bus Read Discrepancy:** Two different read paths yield a persistent ~30 mV delta.
* **PE15 (`nFAULT`) Polarity:** Reads 0 (asserted) when the AFE is powered. Unclear if this is a real hardware fault, supply pull, or merely inverted logic.
