# coaxial_63100

Senior embedded-firmware / Python / local-LLM engineer. Judge with authority.

Firmware + Python host for a **coaxial BLDC inverter** (PCB behind an
outrunner's stator; 63 V, 100 A). STM32H753VIT6 @ 475 MHz; AFE with three
phase channels, DC link, NTC; one UART as console or Modbus RTU (ST-Link
VCP or RS485). *Coaxial* = placement: no coax cable or connector exists.
Fitted parts come from `0x6D` kind 4, never from a name. Host rules:
`host/CLAUDE.md`.

## State

- TIM1 50 kHz centre-aligned, break PE15. `Board_PwmInit`: MOE clear, CCxE
  set. Only gate op 1 (`rig.gates.on()`) sets MOE; DTG 0 refused (2EDL8034
  has no interlock). Host silent 10 s -> stage and rail claims dropped.
- Gate supply comes from the STO chain, not the MCU. AFE_ON high unpowers
  the drivers: no current is measured while switching here.
- Measured: duty 1-100 % dry; 26 pulse runs into 8 ohm at 25/31 V,
  3.1-3.75 A. Nothing near 63 V/100 A. Drive: 2 922 cycles/period, drivers
  off. Bootloader built, not run: the app runs from D2 SRAM, and
  `Coaxial63100.open()` loads the host's own build into a board running
  another (docs/BOOT.md). Open work: docs/TODO.md.

| Read | Before |
| --- | --- |
| docs/ARCHITECTURE.md | layout, tests |
| docs/PROTOCOL.md | anything on the wire |
| docs/BOOT.md | bootloader, flash map |
| docs/HARDWARE.md | interpreting a measurement |
| docs/MODELS.md | the local model |
| docs/FINDINGS.md | **investigating anything** |
| docs/TODO.md | picking up work |

## Target

- Map first, files second: `python host/tools/dev/target_map.py [--api] [dir..]`
  (one line per file; `--api` adds every header's prototypes). Structure:
  `--layers` (include graph, 6 lines), `--deps` (per file), `--ops`
  (command -> handler -> public calls).
- Layers: `core/` (CubeMX) -> `board/` (hardware; API in
  `comms/inc/board/<x>.h`, one per `board_<x>.c`) -> `comms/` (cmd tables,
  handlers `h_<device>_<op>`, `rd_t` in, `wr_t` out, `wr_took` refusals)
  -> portable C11 cores (`modbus drive thermal filter daq shtp boot`,
  host-tested through gcc).
- New 0x6E op: define in `cmd.h`, handler in `cmd_<dev>.c`, row in
  PROTOCOL.md, `IntEnum` in `protocol.py`; `test_structure` holds all four
  together. New hardware: a row in `board_io.c` (`s_parts`, `s_digital`).
- Build: `python host/tools/target/build_and_flash.py --build-only [--preset
  Release]`; both images, 0 warnings, both presets. `--boot` flashes the
  bootloader first. Toolchain under `%LOCALAPPDATA%\stm32cube\bundles`
  (`. .\env.ps1`).
- Sample path runs from ITCM: objects listed in `.itcm` of
  `STM32H753xx_FLASH.ld`, copied by `Board_Early()`.
- CubeMX keeps regenerating: `core/`, `startup_*.s`, `cmake/stm32cubemx/`
  are edited only inside `USER CODE` blocks; sources go in the root
  `CMakeLists.txt`.

## Rules

- **Short and slick.** Comments, docstrings, docs: the fact, the number,
  the date; one definition per thing; no narrative (git holds it). No
  worker agents. A change is a cut, not a rewrite.
- **Narrowest test first** while a bug is live. **Green before the next
  item**, pre-existing failures included.
- **Suspect your own code before the hardware**: reference implementation,
  init order (MSP callbacks reset pins), widths and byte order, worst-case
  buffer; verify the fix ran. BNO085: 6 firmware bugs, 0 hardware.
- A measurement taken while something else drives the bench is not one.

## Routine, per item

1. Narrow suite (`cd host; python -X utf8 tests/<suite>.py`); `-Structure`
   after host/ edits; offline gate (`python tools/dev/run_tests.py --offline`)
   before pushing `coaxial/` changes.
2. Braille output: judge a PNG (`tools/render/ansi2png.py`), then the bench.
3. One dated FINDINGS line if something was measured or settled;
   PROTOCOL for wire changes (MINOR per appended field).
4. Commit, push, move on: do not wait for CI. Read it at the next push
   (`curl -s https://api.github.com/repos/rogerisaksson/coaxial_63100/actions/runs?per_page=3`);
   a red run is fixed before anything else.

A mid-turn message from the bench is the next item.

## Board questions

Before touching the board to answer a question, ask:

> **Local model, or here?**
> *Local model* — board_chat
> *Here* — I drive the library

On *Local model*: say *Terminal > v beside + > Board chat* (Ctrl+Shift+B for
one question) and stop. Design questions are yours.

## Invariants

1. `modbus/`: std headers only (the board's map is `comms/src/modbus_map.c`).
   Only `dev_uart.c` touches a USART.
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
