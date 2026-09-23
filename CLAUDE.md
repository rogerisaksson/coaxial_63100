# coaxial_63100

Senior embedded-firmware / Python / local-LLM engineer. Judge with authority.

Firmware + Python host for a **coaxial BLDC inverter** (PCB behind an
outrunner's stator; 63 V, 100 A). STM32H753VIT6 @ 475 MHz; AFE with three
phase channels, DC link, NTC; one UART as console or Modbus RTU (ST-Link
VCP or RS485). *Coaxial* = placement: no coax cable or connector exists.
Fitted parts come from `0x6D` kind 4, never from a name.

## State

- TIM1 50 kHz centre-aligned, break PE15. `Board_PwmInit`: MOE clear, CCxE
  set. Only `rig.gates.arm()` sets MOE; refuses DTG 0 (2EDL8034 has no
  interlock). Host silent 10 s -> stage and rail claims dropped.
- Gate supply is released by the STO chain, not the MCU. AFE_ON high
  unpowers the drivers: no current is measured while switching here.
- Measured: duty 1-100 % dry; 26 pulse runs into 8 ohm at 25/31 V,
  3.1-3.75 A. Nothing near 63 V/100 A.
- `drive/` (0x6E dev 10) host-tested; 2 922 cycles/period on target,
  drivers off. Bootloader built, not run. Open work: docs/TODO.md.

| Read | Before |
| --- | --- |
| docs/ARCHITECTURE.md | layout, tests |
| docs/PROTOCOL.md | anything on the wire |
| docs/BOOT.md | bootloader, flash map |
| docs/HARDWARE.md | interpreting a measurement |
| docs/MODELS.md | the local model |
| docs/FINDINGS.md | **investigating anything** |
| docs/TODO.md | picking up work |

## Commands

```bash
cd host
python -X utf8 tests/<suite>.py          # one suite
python tools/run_tests.py --offline      # offline gate, ~2.5 min
python tools/build_and_flash.py          # --build-only | --flash-only | --boot
python -m coaxial_mcp --port COM4        # MCP server
python dbg.py -m auto -q "read the NTC"  # local model, one question
```

`.\host\run_tests.ps1` (~25 %; `-All`, `-Structure`, `-Scope X.py`).
`. .\env.ps1` puts the ST toolchain (`%LOCALAPPDATA%\stm32cube\bundles`) on
PATH. Firmware: 0 warnings, Debug and Release.

## Host

`Coaxial63100` (`host/coaxial/rig.py`) is the front door; `device.daq`,
`.imu`, `.angle`, `.thermal`, `.gates`, `.drive`, `.motion`, `board.boot`.
Interfaces `Acquisition`, `PolledSensor`, `GateControl`, `BootControl` have
real + simulated implementations: add a method to both or neither.
`simulated_device=True` needs no cable. Refusals are the board's words
(`u8 took` + text).

## Rules

- **Short.** Code comments, docstrings and docs: the fact, the number, the
  date. No narrative; git history holds it. No worker agents (3 tries,
  0.8 M tokens, nothing returned). A change is a cut, not a rewrite.
- **CubeMX keeps regenerating.** Generated files (`core/`, `startup_*.s`,
  `cmake/stm32cubemx/`) are edited only inside `USER CODE` blocks; sources
  go in the root `CMakeLists.txt`.
- **Narrowest test first** while a bug is live. **Green before the next
  item**, pre-existing failures included.
- **Suspect your own code before the hardware**: reference implementation,
  init order (MSP callbacks reset pins), widths and byte order, worst-case
  buffer, and verify the fix ran. BNO085: 6 firmware bugs, 0 hardware.
- A measurement taken while something else drives the bench is not one.

## Routine, per item

1. Narrow suite; `-Structure` after host/ edits; offline gate before
   pushing `coaxial/` changes.
2. Braille output: judge a PNG (`tools/ansi2png.py`), then the bench.
3. One dated FINDINGS line if something was measured or settled;
   PROTOCOL for wire changes (MINOR per appended field).
4. Commit, push, check CI (`curl -s https://api.github.com/repos/rogerisaksson/coaxial_63100/actions/runs?per_page=3`,
   5-6 min). Fix red first.
5. End every item with exactly:

> **Continue, or commit and push?**
> *Continue* — keep working in this session
> *Commit and push* — stage, commit, push to origin/main

A mid-turn message from the bench is the next item.

## Local model

Routine board readings go to the local model. Before touching the board to
answer a question, ask:

> **Local model, or here?**
> *Local model* — board_chat
> *Here* — I drive the library

On *Local model*: say *Terminal > v beside + > Board chat* (Ctrl+Shift+B for
one question) and stop; never `Start-Process`. Design questions are yours.
`ollama ps` before loading (16 GB card). If Bash is blocked: write
`host/claude_do_it.ps1`, the user's `claude_watch.ps1` runs it, read
`claude_do_it.log`; comment out physical steps once run.

## Invariants

1. `modbus_crc/slave/rtu.c`: std headers only. Only `dev_uart.c` touches a
   USART.
2. RTU timing in raw `DWT->CYCCNT` ticks.
3. `0x41` payload append-only (append = MINOR, else MAJOR).
4. Codec chosen on protocol MAJOR only.
5. No printf while the binary link is open.
6. Every ADC read calls `HAL_ADC_ConfigChannel` and clears `PCSEL`.
7. Scaling lives once, in the calibration record. Only the DC link is
   spanned (2026-08-30, -32 418 ppm ch 5).
8. Python library: a result or a raise from `coaxial.errors`.
9. AFE_ON off: mid-scale, NTC 25.00 C - labelled; cooked readings refuse.
10. No limits or expected values in firmware or tests, except `self_test`
    and the thermal envelope (drops MOE at the record's ceiling).
11. DC link divider 49.9k/2.2k = 78.15 V FS: headroom on purpose.

## Traps

- JTAG connect-under-reset fails: use SWD, end with `--start`.
- AFE_ON off: BNO085 answers reads, ignores writes.
- Bash heredocs mangle backslashes: write scripts with the Write tool.
- Most files CRLF, `core/src/main.c` LF; replace scripts assert matches.
- `#include "x.h"` looks beside the includer first: a same-named header
  includes itself. In linker scripts `*dir/*.o` contains `/*`.
- `UL` is 64-bit on Linux CI: use `U`.
- PowerShell variables are case-insensitive.
