# coaxial_63100

Senior embedded-firmware / Python / local-LLM engineer. Judge with authority.

Firmware + Python host for a **coaxial BLDC inverter** (PCB coaxially behind
an outrunner's stator; 63 V, 100 A). STM32H753VIT6 @ 475 MHz, AFE (3 diff
phase channels, DC link, NTC), one UART: text console or Modbus RTU over the
ST-Link VCP or RS485. *Coaxial* = placement: no coax cable or connector
exists (a local model invented one twice). Fitted parts come from `0x6D`
kind 4, never from a name.

## State

- TIM1 centre-aligned 50 kHz (ARR 2375 @ 237.5 MHz), break PE15 active low.
  Dead time from the calibration record. `Board_PwmInit()`: MOE clear, CCxE
  set - both FETs of every leg held off. Only `rig.gates.arm()` sets MOE; it
  refuses DTG 0 (2EDL8034 has no interlock). Host silent 10 s -> claims and
  stage dropped; the broker answers for an attached client every 3 s.
- Gate drivers' supply is released by the STO chain (pilot tone on RS485),
  not the MCU. AFE_ON high unpowers the drivers, so no current is measured
  while switching on this bench.
- Measured: every duty 1-100 % dry (2026-08-27); into ~8 ohm U-V at 25/31 V,
  26 runs, 3.1-3.75 A, 0 overruns (2026-08-30, `tools/pulse.py`). Nothing
  near 63 V/100 A.
- `drive/` (0x6E dev 10): dq current loop, HF injection, Kalman PLL, I/f,
  polarity pulse; host-tested; 2 922 cycles/period on target, drivers off.
  `tools/commission.py` is the bench procedure.
- Bootloader (`boot/`, docs/BOOT.md): built, host-tested, not yet run.
- Open work: docs/TODO.md.

VREFBUF off; U2 REF2033 drives `+3V3_ref` and `+1V65_bias` via AFE_ON.
Devices, channels and parts come from the bus (`0x41`, `0x6D`), never from
docs. Add hardware = one row in `board/src/board_io.c` (`s_parts`,
`s_digital`, a probe case).

| Read | Before |
| --- | --- |
| docs/ARCHITECTURE.md | touching the source layout or the tests |
| docs/PROTOCOL.md | changing anything on the wire |
| docs/BOOT.md | bootloader, flash map, node images |
| docs/HARDWARE.md | interpreting a measurement |
| docs/MODELS.md | the local model, its tag, its tools |
| docs/FINDINGS.md | **investigating anything** - what is settled |
| docs/TODO.md | picking up work |

## Commands

```powershell
.\host\run_tests.ps1                # ~25 % tier; -AutomaticMedium 50, -AutomaticHigh 75, -All
.\host\run_tests.ps1 -Structure     # after any host/ edit, 4 s
.\host\run_tests.ps1 -Scope X.py    # one suite
. .\env.ps1                         # PATH + board_chat, dbg, cbuild, cflash
```

```bash
cd host
python -X utf8 tests/<suite>.py          # one suite
python tools/run_tests.py --offline      # the offline gate, ~2.5 min
python tools/build_and_flash.py          # --build-only | --flash-only | --boot
python tools/flash_nodes.py --store DIR --simulated --bus LL
python tools/pulse.py -d 0.05 -H U -L V -n 1 --on 30
python tools/commission.py --simulated   # --arm --port COM4 at the bench
python -m coaxial_mcp --port COM4        # MCP server
python dbg.py -m auto -q "read the NTC"  # local model, one question
python tools/ansi2png.py frame.txt frame.png   # judge braille in a raster
```

Firmware must build with 0 warnings, Debug and Release. The ST toolchain is
under `%LOCALAPPDATA%\stm32cube\bundles\`, not on PATH (`env.ps1` adds it).

## Host

`Coaxial63100` (`host/coaxial/rig.py`) is the front door: AFE preflight,
supply restored on close (Ctrl+C too). Subsystems: `device.daq`, `.imu`,
`.angle`, `.thermal`, `.gates`, `.drive`, `.motion` (`stepper`, `servo`,
`velocity`), `board.boot`. Interfaces `Acquisition`, `PolledSensor`,
`GateControl`, `BootControl` each have a real and a simulated
implementation - add a method to both or neither. `simulated_device=True`
needs no cable. Refusals are the board's words (`u8 took` + text); the host
validates only what stops a request being formed.

```python
from coaxial import Coaxial63100
with Coaxial63100(port='COM4') as device:
    daq = device.daq
    daq.open(); daq.enable(); device.set_time_from_pc()
    daq.configure('phaseU', 'NTC'); daq.start()
    for r in daq.read(-1):
        print(r.start_time, r.dt, [(s.name, s.value) for s in r.samples])
```

## Rules

- **Surgical, token-light.** No worker agents (3 tries, 0.8 M tokens,
  nothing returned). A change is a cut, not a rewrite. One subsystem = one
  file, one seam, one suite. Docs short: facts, numbers, dates - no
  narrative. Git history holds the long form.
- **CubeMX must keep regenerating.** Edit generated files (`core/`,
  `startup_*.s`, `cmake/stm32cubemx/`) only inside `USER CODE` blocks.
  Sources go in the root `CMakeLists.txt`. Leave `.ioc`/`.mxproject` case.
- **Narrowest test first.** While a bug is live, run what can disprove the
  hypothesis (a register read, one suite), never `-All`.
- **Green before the next item**, pre-existing failures included.
- **Suspect your own code before the hardware.** BNO085 bring-up: six
  firmware defects, four hardware hypotheses, none survived. Read the
  reference implementation, the init order (MSP callbacks reset pins),
  widths and byte order, the worst-case buffer; verify the fix ran.
- A measurement taken while something else drives the bench is not one.

## Routine, per item

1. Test the narrow thing; `-Structure` after host/ edits; offline gate
   before pushing changes to `coaxial/`.
2. Braille output is judged in a raster PNG (`ansi2png.py`), then the bench.
3. Document: one dated FINDINGS line if something was measured or settled;
   PROTOCOL for wire changes (MINOR per appended field).
4. Commit (one-sentence title, measured facts in the body), push, check CI:
   `curl -s https://api.github.com/repos/rogerisaksson/coaxial_63100/actions/runs?per_page=3`
   (5-6 min; a red run's log tail is a commit comment). Fix red first.
5. End every item with exactly:

> **Continue, or commit and push?**
> *Continue* — keep working in this session
> *Commit and push* — stage, commit, push to origin/main

A mid-turn message from the bench is the next item.

## Local model

Routine board questions go to the local model (free): `board_chat -Ask
"..."` or VS Code: *Terminal > v beside + > Board chat* (Ctrl+Shift+B for
one question). Before touching the board to answer a question, ask:

> **Local model, or here?**
> *Local model* — board_chat
> *Here* — I drive the library

On *Local model*: give the click, stop. Never `Start-Process`. Design
questions (why this C, protocol MAJOR?) are yours - the model invents
hardware constants. Check `ollama ps` before loading (16 GB card).
If the permission classifier blocks Bash: `host/claude_watch.ps1` runs
`host/claude_do_it.ps1` on change, log in `claude_do_it.log`; comment out
physical steps once run.

## Layout

```text
core/      CubeMX-generated; main.c = CubeMX + USER CODE calls only
board/     this hardware, behind comms/inc/board.h (+ comms/inc/board/*.h)
comms/     cmd over link over dev_uart; console
modbus/ drive/ thermal/ filter/ daq/ shtp/ boot/   portable C11 cores, host-tested
electronics/  schematic, BOM, pick-place - authority on what is fitted
host/      coaxial/ (graphics/ renderer), coaxial_mcp/, coaxial_ollama/,
           terminal/ (loader + pages), tests/, tools/
notebook_examples/  nine executed papers, from host/tools/notebooks/
docs/      this documentation
```

## Invariants

1. `modbus_crc/slave/rtu.c` include only std headers; host-tested
   (`test_modbus_core.py`). Only `comms/src/dev_uart.c` touches a USART.
2. RTU timing in raw `DWT->CYCCNT` ticks, never microseconds.
3. `0x41`'s payload is append-only: append = MINOR, anything else = MAJOR.
4. A host picks its codec on protocol MAJOR only.
5. No printf while the binary link is open (latched overrun kills RX).
6. Every ADC read path calls `HAL_ADC_ConfigChannel` and clears `PCSEL`.
7. Every scaling parameter lives once, in the calibration record (0x6E dev
   3). Only the DC link is spanned against an instrument (2026-08-30,
   -32 418 ppm ch 5); all else is schematic arithmetic.
8. The Python library returns a result or raises `coaxial.errors`; never a
   status code or None-for-failure.
9. AFE_ON off: channels read exact mid-scale, NTC exactly 25.00 C - labelled,
   not refused; cooked readings refuse.
10. The board is a dumb slave: no limits or expected values in firmware or
    tests. Exceptions: `self_test` (own registers/flash) and the thermal
    envelope (drops MOE at the record's ceiling, holds 70 % for 30 min).
11. DC link divider 49.9k/2.2k = 78.15 V FS on 63 V, headroom deliberate.

## Traps

- JTAG connect-under-reset fails (`Unable to get core ID`): use SWD or
  `mode=Normal reset=SWrst`; end with `--start`.
- AFE_ON (PB2) off: analog reads mid-scale, BNO085 answers reads but ignores
  writes. Enable it before believing an analog or IMU reading.
- Bash heredocs mangle backslashes: write scripts with the Write tool.
- Most files are CRLF; `core/src/main.c` is LF. Replace scripts must assert
  their patterns matched.
- A quoted `#include "x.h"` resolves next to the including file first: a
  header named like its target includes itself.
- `UL` is 64-bit on Linux CI: use `U` in C shared with host tests.
- PowerShell variables are case-insensitive (`$Asked` == `$asked`).
