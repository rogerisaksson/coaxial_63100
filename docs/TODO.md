# TODO

QA and hardening pass, from 2026-08-26.

| | Baseline | Now |
|---|---|---|
| `run_tests.ps1 -All` | 1540 passed, 9 suites, 413 s | **1640 passed, 0 failed, 17 suites** |
| Largest single suite | 733 checks | **218** |
| Debug build | 0 warnings | 0 warnings |
| FLASH / DTCMRAM | 92 684 B / 13 488 B | 97 708 B / 13 616 B |

## Done

**Calibration on the board** — `0x6E` device 3, in flash, survives a reset.

- Nine scalars and one offset/gain pair per channel; `zero`, `span`, `save`, `load`, `defaults`
- `Board/Src/board_cal.c`, `Comms/Src/cmd_cal.c`, `host/coaxial/calibration.py`
- 15 conformance checks, from a master sharing no code with the library
- Verified live: save → hardware reset → record intact
- `span` refused on the NTC (logarithmic) and on any channel with no unit
- Caught a real bug: the rollback reloaded flash, which does nothing on a board that never saved. Left `vref_uv` at zero. Validates first now

**Scaling, traced off `electronics/` — none of it measured.**

- DC link: R12 49.9 kΩ / R11 2.2 kΩ → 78.15 V full scale
- NTC: R100 10.0 kΩ / NCU18XH103**D60**RB → B = 3380 K, ratiometric
- Phase: RU1‖RU2 3.5 mΩ (two 7 mΩ WSHM2818) × THS4551 1.5k/330 → 15.909 mV/A, 207.4 A full scale
- Reference: U2 REF2033 drives `+3V3_ref` **and** `+1V65_bias`
- Phase V reads −52 A with nothing connected — the known 0.85 V op-amp fault, now in a unit that makes it obvious

**Tests split by subject** — one file each, 12 to 218 checks.

- `ollama_support.py` holds the fixtures; `select()` and `run_file()` take the roster as an argument
- Any 5 % step is a tier; suites join cheapest-per-check first
- The model may narrow within a tier, never past it. Measured: it put `live:all` back on the 25 % tier — 398 s of which 352 were that suite
- The path map settles cheap changes without asking the model at all
- Ctrl+C is `STOPPED`, exit 130, and hands the model back

**Views** — `demo.ps1` picks; `demos/imu.ps1`, `angle.ps1`, `adc.ps1`.

- IMU: AsciiEffect ported from `AndrewSink/STL-to-ASCII-Generator`, fills the window, wheel and right-drag zoom
- 120 000 surface points: at 45 000 the buffer held 0.6 points per pixel and the board drew as a disc of speckle
- Meter bridge: every channel its own scale in its own unit. No dB, no raw codes
- Q closes, ESC returns to the menu, both put the front end back

## Next

- **Span every channel against an instrument.** Nothing here has been measured; every ampere is arithmetic off a PDF
- **What the ADA4891 quad does.** The shunt bounds the chain at 9.43 V/V for 100 A to be representable. THS4551's 4.5455 fits; the ×18.5 and ×10 a net dump appeared to show do not. Changes no reported ampere
- MCP tool for the calibration record — firmware and library have it, `coaxial_mcp` does not

## Open — from the inventory

- **No CI anywhere.** 151 commits in 5 days; the gate is `run_tests.ps1 -All` run by hand
- Datasheets for 2 of 45 active parts. Missing: 2EDL8034, IAUCN10S7N021, THS4551, ADA4891, REF2033, TPS3840, LM5069, TLV3492, THVD1450
- No layout or gerbers in `electronics/` — schematic PDF and BOM only

## Open — from reading the code

- [board_imu.c:174](../Board/Src/board_imu.c#L174), [dev_uart.c:320](../Comms/Src/dev_uart.c#L320) — deadline as `HAL_GetTick() + ms`. Three other waits in the same files use the wrap-safe form. Breaks at 49.7 days
- [board_io.c:27](../Board/Src/board_io.c#L27) — `s_digital` omits PA1/PA2/PA3 and PC8/PC12/PD2, so `testrig_port_write` masks USART3 and JTAG but neither RS485 port
- `BOARD_BURST_MAX_US` — 5 s against a 9.04 s CYCCNT wrap at 475 MHz. Correct today, undocumented coupling
- No watchdog; fault handlers are bare `while(1)`. U4 TPS3840 supervises the rail; nothing supervises the firmware
- Calibration has no wear levelling — one sector, erased whole on every save
