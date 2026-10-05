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
- Gate supply comes from the STO chain, not the MCU: the master's pilot on
  A1/B1, PA10's keepalive, FAULTOUT on PE15 (docs/HARDWARE.md). The bench
  board is unmodified (R93 on +5): AFE_ON high unpowers its drivers, and no
  current is measured while switching there. Emulated and simulated boards
  follow the schematic: the pilot heard with AFE_ON up supplies them.
- Measured: duty 1-100 % dry; three legs switching dry at 24-60.8 V, 25
  and 50 kHz, the thermal observer within 0.2 K of the thermistor after 120
  s blind at 53.8 V (2026-10-05); 26 pulse runs into 8 ohm at 25/31 V,
  3.1-3.75 A. Drive: 2 922 cycles/period, drivers off. Bootloader on the
  bench board (2026-10-05, the ST-Link's port, COM3 on the laptop): the app
  runs from D2 SRAM, and `Coaxial63100.open()` loads the host's own build
  into a board running another or waiting blank (docs/BOOT.md); a power
  cycle and 10 Mbit unproven. Open work: docs/TODO.md.
- Emulated: the image on Renode's STM32H753 (`board/emu`, `host/tools/emu`),
  its front end from the schematic and LTspice (`electronic_simulations`,
  `afe_spice.py`), the A1335, the BNO085 and the STO chain on the master's
  pilot (`world/src/world_sto.c`, `sto.asc`'s circuit) modelled, 475 MIPS:
  conformance 110/110; a limb of N boards on one RS485 bus;
  `emulator://?body=humanoid` the stand-in's fleet. Where no board answers:
  the emulator, else the stand-in (`COAXIAL_FALLBACK=simulated` skips it;
  the offline gate sets it).
  `native://`, for real-time SIL/HIL (the robot that balances and walks):
  board/src's board layer built for this host over `board/native`'s chip -
  TIM1, ADCs, SPI and DMA, the front end, A1335, BNO085 - 18x headroom a
  board with the drive on; `?body=humanoid` its 20 boards on 5 buses, a
  thread a limb, 2.2 cores. Validating the firmware stays on Renode; the
  tty's pages default to `native://` (`screen.PORT`): Renode runs 3.8-4.5
  wall s a board s under the drive, its translator's cost, not ours.

| Read | Before |
| --- | --- |
| docs/ARCHITECTURE.md | layout, tests |
| docs/PROTOCOL.md | anything on the wire |
| docs/BOOT.md | bootloader, flash map |
| docs/HARDWARE.md | interpreting a measurement |
| docs/MODELS.md | the local model |
| docs/DIMENSIONS.md | a drive, an axis, a stack |
| docs/FINDINGS.md | **investigating anything**: the board's, the rest by subject in docs/findings/ |
| docs/TODO.md | picking up work |

## Target

- Map first, files second: `python host/tools/dev/target_map.py [--api] [dir..]`
  (one line per file; `--api` adds every header's prototypes). Structure:
  `--layers` (include graph, 6 lines), `--deps` (per file), `--ops`
  (command -> handler -> public calls).
- Layers: `core/` (CubeMX) -> `board/` (hardware; API in
  `comms/inc/board/<x>.h`, one per `board_<x>.c`) -> `comms/` (cmd tables,
  handlers `h_<device>_<op>`, `rd_t` in, `wr_t` out, `wr_took` refusals)
  -> portable C11 cores (`modbus drive thermal filter daq shtp boot ctrl`,
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
  the date; one definition per thing; no narrative (git holds it). Prose
  at a minimum, technical and terse: no markers (capitals, labels, asides,
  remarks on progress) and no filler. A change is a cut, not a rewrite.
- **Data-oriented**, target and host (docs/ARCHITECTURE.md): structs or
  structured arrays; `step(state, in) -> out` touching its arguments only;
  memory laid once; tables of steps; the hardware at one edge.
- **Local first.** Scripts, lint, suites on this host before agents or
  workflows, ultracode or not; agents only for what cannot run here. Two
  swarms (2026-09-24) burned tokens and ran slower than local work.
- **The relay maxes the host.** Suites only through the baton relay
  (`tools/dev/focus.py`: a baton a physical core, the longest first, the
  long ones as shards): `python tools/dev/run_tests.py --file test_a.py
  --file test_b.py:word` (`--smart` what the diff reaches, `--offline` the
  gate); Monte Carlos through `tools/sim/gait_montecarlo.py`. Never a
  suite run straight nor a pool of one's own (`relay_first.py` denies the
  first).
- **Narrowest test first** while a bug is live. **Green before the next
  item**, pre-existing failures included.
- **Work goes into docs/TODO.md** the moment it is asked or found, and its
  line is deleted when done: the list, not the conversation, carries it.
  Unclear or self-contradicting: ask.
- **Suspect your own code before the hardware**: reference implementation,
  init order (MSP callbacks reset pins), widths and byte order, worst-case
  buffer; verify the fix ran. BNO085: 6 firmware bugs, 0 hardware.
- A measurement taken while something else drives the bench is not one.
- **What the user sees is so.** They point it out, you measure, the
  numbers say what to change, you change it and measure again:
  `host/tools/sim/look.py` (from the squat; `--last` their newest HUMANOID
  recording, R), quoted before any explanation - a hook says so on every
  such message (`tools/dev/measure_first.py`). Nothing is fixed until the
  second measurement shows it gone. Measure against the body, not the
  plumb line: a torso plumb over bent knees leans back. A running page
  keeps the code it started with.
- **Her walk is a human's; its form is a condition, never a price** (the
  user, 2026-10-05). On a flat, smooth floor one walk passes: the least
  energy a metre, the pelvis rolling over a stance leg whose knee is all
  but straight from its landing till it is behind her. A crouch falls
  less and so does crawling: a fall is met by a parry - where the foot
  lands -, never by the gait's shape, stride or height; the scoreboard
  rewards parries and rejects a walk off its form. A straight leg costing
  more than a bent one is a fault in how it is driven: found, not tuned
  round. What is solved stays: a fix that moves a solved measure is
  undone. Mocap is a reference, not a search space. `looks.FORM` is the
  form, test_gynoid_gait.py holds it; docs/TODO.md item 1.
- **Smoke her walk first**: after any change reaching it,
  `python tools/sim/armada.py` (12 s on the armada kept up, `--up`) - her
  form, J/m and parries at three paces - and
  `python tools/sim/gait_montecarlo.py --suite look` (36 s: her rises, her
  walks begun standing at four paces), quoted before anything else. A
  candidate is tried from a walking robot's mark (`--grid`, `--search`),
  never from the squat.
- **Simulated before emulated.** *Emulated* is the image on Renode: SIL.
  *Simulated* is a pure software model claiming no hardware, real or
  emulated. Nothing goes on the emulator until a plausible simulated model
  of it exists; the gynoid stays in the simulated world (2026-09-27).

## Routine, per item

1. Narrow suites on the relay (`cd host; python tools/dev/run_tests.py --file
   test_x.py[:word]`, every suite the item reaches in one call); `--structure`
   after host/ edits; offline gate (`python tools/dev/run_tests.py --offline`)
   before pushing `coaxial/` changes. Lint (`host/tools/dev/lint.py`:
   markdownlint, pyright basic) runs as a hook after every edit and at stop.
2. Braille output: judge a PNG (`tools/render/ansi2png.py`), then the bench.
3. One dated line in its subject's findings if something was measured or settled;
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
7. Scaling lives once, in the calibration record. Nothing is spanned: the
   DC link's -32 418 ppm was its converter's offset (2026-10-05).
8. Python library: a result or a raise from `coaxial.errors` (`machine.errors`).
9. AFE_ON off: mid-scale, NTC 25.00 C - labelled; cooked readings refuse.
10. No limits or expected values in firmware or tests, except `self_test`
    and the thermal envelope (drops MOE at the record's ceiling).
11. DC link divider 49.9k/2.2k = 78.15 V FS: headroom on purpose.
12. Emulation is transparent: the target builds nothing of `board/fake`,
    `board/native`, `board/emu`; no firmware source asks where it runs
    (`test_structure`). CubeMX regenerates and the bench flashes as before.

## Traps

- JTAG connect-under-reset fails: use SWD, end with `--start`.
- AFE_ON off: BNO085 answers reads, ignores writes.
- Bash heredocs mangle backslashes: write scripts with the Write tool.
- Most files CRLF, `core/src/main.c` LF; replace scripts assert matches.
- `#include "x.h"` looks beside the includer first: a same-named header
  includes itself. In linker scripts `*dir/*.o` contains `/*`.
- `UL` is 64-bit on Linux CI: use `U`.
- PowerShell variables are case-insensitive.
- The editable install resolves `machine`, `coaxial` to the main checkout:
  a script in a git worktree runs its own code only with
  `PYTHONPATH=<worktree>/host`.
