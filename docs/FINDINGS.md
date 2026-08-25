# Hardware & Firmware Findings

## Resolved Defects

* **`ADC_PCSEL` Accumulation:** The HAL blindly ORs `PCSEL` without clearing it, leaving every configured channel permanently connected. DC bus noise dropped 7x after explicitly clearing it pre-configuration.
* **Silent Zero on a Failed Conversion:** `ADC_ReadOneChannel` returned void and left `*outRaw` at 0 when `ConfigChannel`, `Start` or a 10 ms `PollForConversion` failed. On a differential channel code 0 is 0 V, so a failure was indistinguishable from a measurement. Now `bool`, propagated through all six `Board_*` readers; the noise and burst loops abort rather than fold a zero into the mean. `h_adc_burst` took on the value checks `h_adc_noise` already had, so its refusal can mean SERVER DEVICE FAILURE instead of ILLEGAL DATA VALUE.
* **Blind Differential Reads:** `ADC_ReadDifferentialVolts` hijacked whatever state the ADC was left in rather than configuring its own channel. The function was purged.
* **Modbus Qty 0 Ignored:** A valid request for zero items failed silently due to a lazy 7-byte minimum PDU check. Lowered to 6.
* **HSE Boot Warning:** Blindly rejected PLL1 even when sourced directly from HSE. Fixed.
* **String Formatting Exception:** Missing tuple parentheses in the MCP renderer swallowed outputs. Caught solely because the exception handler was cynically narrow. Keep it that way.

## LLM & Host Infrastructure

* **LLM OOM Crashes:** `llama-server` throws `std::bad_alloc` due to its own bloated prompt cache and checkpoints, not prompt length. Fixed by disabling `LLAMA_ARG_CACHE_RAM` and `LLAMA_ARG_CTX_CHECKPOINTS` in the daemon.
* **Stand-in Diagnosed as a Dead Cable:** `SimulatedSession.port` is a bus label (`AX`), never `None`, so `link_diagnose`'s `configured is None` guard never fired for a session that fell back on its own: 15 s of SWD probing, then "Configured port AX: not among the ports above - the cable may be unplugged". Second consumer to read `.port` as a COM port; the `simulated` marker that fixed `_interface` already existed. Hidden by a test double - `test_ollama.py`'s own `SimulatedSession` has no `port` attribute at all, so it reached the branch however the branch was written. Step 4's advice also opened with "Powered" whatever step 1 concluded.
* **`check_power` Discarded Its Own Reading:** With no target the programmer takes 30.3 s - a second connect attempt at 8 MHz - against a 15 s budget, and `TimeoutExpired.stdout` already holds `Voltage: 0.00V`. The handler returned `None`, i.e. "unknown" for the one case the check exists to answer. Now parsed from the killed run.
* **`-Ask` Pinned the Card:** A one-shot was exempt from the unload on prompt exit, on the grounds that "dbg.py already holds it for two minutes rather than thirty" - which the same script disproves twenty lines up by passing `--keep-alive $KeepAlive`, i.e. 30m. Four smoke tests left 8.4 GB resident with nobody at the prompt. `-Ask` now takes a list: one load, N questions, one release.
* **`--sections` Read Only Under `--match`:** `run_tests.py --live --sections tools` silently ran all three - measured, `tools` and `all` returning the same 176 checks in the same 255 s. Coverage tiers were unaffected; they set the sections directly rather than through the flag.
* **Model Hallucinations:** `llama3.1:8b` fabricates telemetry when tools fail and invents physical constants (NTC B=3950 instead of 3380). `gemma4:12b` remains the default; it is slower but declines to lie.

## Confirmed Behaviors (Not Defects)

* **JTAG Connect-Under-Reset:** Fails because ST-Link probes neglect TAP re-initialization. Hardware is innocent. Workaround: SWD or `SWrst`.
* **Halted Core Silences USART3:** An aborted JTAG/SWD reset leaves the core halted, killing serial comms. Use `mode=HOTPLUG`.
* **Phase V 0.85V Offset:** Isolated op-amp failure on a single board. Do not calibrate around broken hardware.
* **NTC Bit-Exact "Anomalies":** Johnson noise is 120x below LSB. Bit-exact readings are physics, not a frozen register.
* **UART Overrun (ORE):** Latches and kills RX permanently. Now explicitly cleared via `ICR`.
* **ADC Offset Calibration:** Drifts ~100mV across boots because it runs against an unpowered reference (`AFE_ON` low).

## Refuted Hypotheses (Dead Ends)

* **`-Wconversion` Would Find Silent Truncations:** False. Measured across all seventeen of our sources: 113 warnings, 112 of them inside HAL/CMSIS headers the sources merely include, one ours - a `0U` ternary narrowing to `uint8_t`. The integer-heavy files are heavy in *explicit* casts, which is why they look dense and warn about nothing. The flag now sits on the ten HAL-free sources, where it also guards invariant 1: add a HAL include there and the build fills with someone else's conversions.
* **NTC Sample Time Too Short:** False. The 15nF capacitor provides the necessary charge.
* **DIFSEL Ignored:** False. It is written safely while the ADC is disabled.
* **VREF Sag at 475 MHz:** False. A coincidental 1% shift on the DC bus; other channels shifted completely asynchronously (up to 9.7%).
* **Prompt Length Causing OOM:** False. See cache mechanics above.

## Open Anomalies

* **Unpowered Calibration:** The proposed fix (waiting for `AFE_ON`) remains untested.
* **IN11 (`Cinj`) 9.7% Shift:** Unexplained frequency-dependent drift between 75 and 475 MHz.
* **DC Bus Read Discrepancy:** Two different read paths yield a persistent ~30 mV delta.
* **PE15 (`nFAULT`) Polarity:** Reads 0 (asserted) when the AFE is powered. Unclear if this is a real hardware fault, supply pull, or merely inverted logic.