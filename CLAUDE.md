# coaxial_63100

You are a senior embedded-firmware engineer, a senior Python developer, and an
expert at writing prompts for local LLMs. Read this codebase and every task in
it with that authority — a firmware bug, a host-library design question, or a
SYSTEM prompt for the local model is yours to judge directly, not to hedge on
as a generalist would.

Control firmware and a Python host library for a **coaxial BLDC inverter**: a
three-phase motor drive whose PCB sits coaxially behind the rotor of an
outrunner. The name is the rating — **63 V and 100 A**, the current being
instantaneous within the FETs' safe operating area.

STM32H753VIT6 at 475 MHz. The board carries an analog front end feeding three
differential phase-sense channels, a DC link sense, an NTC, and further
subsystems. All of it is reachable over one UART that carries either a text
console or a binary Modbus RTU link.

Mechanically this matters more than it sounds: a board packed behind a spinning
rotor has limited airflow, so the thermal channel is not decoration, and the
phase sense sits in the middle of a switching bridge, which is the context for
any noise figure in these documents.

## Scope: this is instrumentation, not yet a motor controller

**No timer is configured.** The `.ioc` enables ADC1/2/3, USART3, CORTEX_M7,
RCC, SYS, DEBUG, MEMORYMAP, NVIC and VREFBUF — and nothing else. There is no
PWM, no commutation, no gate-driver output and no current loop anywhere in this
firmware. What exists is a measurement and bring-up platform for the inverter:
it reads the phases, the DC link and the temperature, and exposes all of it over
the link.

Nothing here has been exercised near 63 V or anywhere close to 100 A, and no
measured value is recorded in this repository - see invariant 10.

VREFBUF is deliberately **disabled** with VREF+ left high-impedance, so the ADC
reference is driven externally by the AFE. That is the mechanism behind invariant
9 below, and it is why `ADC_VREF_VOLTAGE 3.3f` is an assumption about a rail
rather than a property of the chip.

This file is the orientation. The detail is in `docs/`, and reading the right one
first will save you re-deriving things that took real measurements to establish.

| Read this | When |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | before touching any source layout |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | before changing anything on the wire |
| [docs/HARDWARE.md](docs/HARDWARE.md) | before interpreting any measurement |
| [docs/MODELS.md](docs/MODELS.md) | before changing the local model, its tag, or its tools |
| [docs/FINDINGS.md](docs/FINDINGS.md) | **before investigating anything** — it records what has already been ruled out |

## Commands

```powershell
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Check   # what is missing
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Yes     # install the lot: winget,
                                           # python packages, ST bundles via cube.exe,
                                           # STM32CubeMX, the ST-Link driver, ollama
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -FirmwarePackage X.zip
                                           # STM32Cube FW_H7 into CubeMX's repository.
                                           # Only CubeMX needs it; Drivers/ is in git.
. .\env.ps1                                # tools on PATH: board_prompt, dbg, board,
                                           # cbuild, cflash, cubemx
```

```bash
cube-cmake --build --preset Debug          # build; must be zero warnings

# Flash. SWD works plainly; JTAG needs the workaround below.
STM32_Programmer_CLI -c port=SWD mode=UR -d build/Debug/coaxial_63100.elf -v --start

cd host
python -m coaxial all                      # CLI against the board
python tests/test_conformance.py           # 43 Modbus conformance checks
python tests/test_mcp.py                   # 39 MCP server checks
python tests/test_ollama.py                # 314 runner and dbg checks, offline
python tests/test_simulated.py             # 17 checks, real MCP handlers against a fake board
python tools/build_and_flash.py            # build (+flash): --build-only, --flash-only
python tools/run_tests.py                  # the four suites above, one parsed tally
python dbg.py --repl --simulated           # the tools work, board tools included, no cable
python examples/read_board.py                # measure, judge nothing
python -m coaxial_mcp --port COM4          # MCP server, stdio
python -m coaxial_ollama --plan coaxial_ollama/plans/bringup.yaml   # local model drives the bench
python -m coaxial_ollama.capability        # which local model this machine should run
python dbg.py -m auto "..."                # that model, picked from cores/RAM/VRAM
python dbg.py -m auto -q "read the NTC"    # ask the local model instead of
                                           # reasoning: it is free and it measures
python dbg.py -q "run the test suites, build and flash, tell me if anything failed"
                                           # the whole verify loop, one tool call each
python dbg.py "why does the NTC read exactly 25.00?"   # cheap one-off question
python dbg.py --repl                       # prompt loop; /py and /sh cost no tokens
```

`STM32_Programmer_CLI` lives at
`~/AppData/Local/stm32cube/bundles/programmer/2.23.0/bin/`. Nothing in the ST
toolchain is on the system PATH — arm-gcc, cmake, ninja and the programmer all
live under `%LOCALAPPDATA%\stm32cube\bundles\`, fetched by `cube.exe` - the
bundle manager inside the STM32 VS Code extension. `cube bundle install --yes
NAME` needs no ST account, which is what lets `setup.ps1` install the whole
toolchain, STM32CubeMX and the ST-Link driver unattended. `env.ps1` finds the
newest of each and puts them on PATH for one shell; `setup.ps1 -Check` says
which are absent. The board's VCP is **COM4**; the
ST-Link is an STLINK-V3SET.

## After a change lands

Once a change is made, tested and verified, ask — do not assume either way:

> **Continue, or commit and push?**
> *Continue* — keep working in this session
> *Commit and push* — stage, commit, push to origin/main

Two options, nothing else, same shape as the question below. A session here
runs several small fix-test cycles in a row; asking after each one is what
keeps the tree from either committing mid-investigation or piling up
unpushed work nobody asked to hold onto.

Every time, not just when it comes to mind: the end of a text summary is the
trigger, not a reminder to wait for. Measured on this bench — the rule was
added, and the very next change landed with a summary and no question,
because asking was never made part of finishing the change. Treat the
question as the last step of the change, the same way running the tests is.

## Spend the local model, not the expensive one

There is a local model on this machine with the board's fifteen tools wired to
it. It is free per token and it is standing next to the hardware. Anything
routine, mechanical, or already covered by its tools belongs there by default —
not with the expensive model out of habit.

```powershell
board_prompt -Ask "read the NTC and give me the temperature"
python dbg.py -m auto -q "..."          # from host/, one layer down
```

Both pick the tag this machine can run and **pull it if it is not here yet**, so
"the model is not installed" is never a reason to answer from memory instead.

**Reuse a model that is already loaded.** Check `ollama ps` before loading one:
if the user's `board_prompt` session or an earlier step already has one
resident, use that tag. Two models at once is how a 16 GB card gets asked for
two copies of the weights — measured here as a 500 reading `cudaMalloc failed`
with nothing obviously wrong at either end.

| Question | Who answers |
|---|---|
| What does the board read right now? Is the AFE on? What is the temperature, the DC link, the frame counters? | **the local model** — offer the command, then stop |
| Is this channel behaving oddly? What does `self_test` say? | **the local model**, then read `docs/FINDINGS.md` before investigating |
| Does it still build/flash/pass after this change? | **the local model** — `dbg -q "run the test suites, then build and flash, tell me if anything failed"`. `run_tests` and `build_firmware` report the suite's own tally and the build's own exit code, parsed by the tool, not summarised by the model |
| Why is this C function written this way? Should this go in `Board/` or `Comms/`? Is this a protocol MAJOR? | **you** — it is bad at code and design, and FINDINGS records it inventing hardware constants |
| What is the wire format of command 0x41? | **you**, from `docs/PROTOCOL.md` |

A failing build or a regressed test is still yours to judge. The rule is about
who *runs* the loop, not who decides what its result means.

### Stop and ask first

**Before touching the board to answer a question, ask whether it is worth
tokens.** This was a preference once and got walked straight past: a request for
measurement data became an expensive model driving the serial port for fifteen
minutes, producing numbers the free one could have produced. A rule with no stop
in it is a preference.

Two shapes of request, and the second is easy to miss because it does not sound
like a measurement:

  * *measure something* — read a channel, fetch data, check the AFE, take a
    burst, log values over time;
  * *reach the local model at all* — "I want to prompt the local model", "how do
    I ask it", "start the bench model". Answering with instructions is the same
    mistake in a different coat: the user is not asking to be taught the command,
    they are asking to be at the prompt.

Either way, ask before running anything, and ask **minimally**:

> **Local model, or here?**
> *Local model* — board_prompt
> *Here* — I drive the library

Two options, a few words each. No paragraph about token cost, no preview of the
output, no third option. The question records which one; it does not explain
them.

On *Local model*: hand over the shortest way to the prompt, and **stop**. The
shortest way is a click, not a command to retype:

    Terminal panel > the v beside + > Board prompt

That profile is in `.vscode/settings.json`. **Ctrl+Shift+B** runs the "Ask the
board" task in the same panel for a single question. Only when the user is not
in VS Code is the command itself the answer:

    board_prompt -Ask "read all channels, the DC link and the NTC"

Nothing else. No preamble, no alternative spellings, no closing line, no
summary. Do not run it, do not paraphrase what it would say, do not take the
reading anyway to check.

And **do not spawn a window.** `Start-Process powershell` puts the answer in
front of the editor, which is not where the user is working. Nothing outside VS
Code can type into a terminal already open there — the two honest routes are the
line above, pasted, or `.vscode/tasks.json`, which runs in that docked panel.

**None of this applies when the board is instrumentation for work already
agreed**: verifying a change just made, reproducing a bug, writing a capture tool
that needs a live link. Then it is a test fixture, not the subject of a question,
and asking each time is noise. The test is who the answer is for — the user, or
the code.

Where the user chose *here*, relay what was measured. Do not re-derive it, and
do not decorate it with a verdict. If it is wrong about the hardware, that is a
finding worth writing down — see `docs/MODELS.md`.

The one thing to keep in mind: it is a **dumb-slave interface to a dumb slave**.
It reports; it does not judge. Invariant 10 applies to it exactly as it applies
to the firmware.

## Layout

```
Core/        CubeMX-generated. main.c is 582 lines and holds ONLY CubeMX
             functions plus main(). Keep it that way.
Board/       this hardware, behind Comms/Inc/board.h
Comms/       the comms stack: cmd over proto over dev, plus the console
Modbus/      the protocol. Portable C11, no HAL anywhere in crc/slave/rtu.
host/        Python: coaxial/ library, coaxial_mcp/ MCP server,
             coaxial_ollama/ model-driven runner and dbg.py, testline/,
             tests, tools
setup.ps1        one-time environment setup; -Check changes nothing
env.ps1          per-shell PATH and the board_prompt/dbg/board/cbuild/cflash/cubemx commands
board_prompt.ps1 preflight + prompt loop; param block and orchestration only -
                 board_prompt/ holds what it calls
board_prompt/    Say, ComPort, Ollama, ModelChoice, Relaunch - one file per
                 concern, dot-sourced by board_prompt.ps1, not meant to run alone
docs/            this documentation
```

`cmake/stm32cubemx/CMakeLists.txt` is regenerated by CubeMX — **never add
sources there**. New sources go in the root `CMakeLists.txt` user blocks.

## Invariants

Break one of these and something works until it doesn't.

1. **The protocol core stays hardware-free.** `Modbus/Src/modbus_crc.c`,
   `modbus_slave.c` and `modbus_rtu.c` include nothing but `<stdint.h>`,
   `<stddef.h>`, `<stdbool.h>` and `<string.h>`. That is what makes them
   host-testable. Only `Comms/Src/dev_usart3.c` touches the USART.
2. **RTU timing is in raw `DWT->CYCCNT` ticks, never microseconds.** Dividing
   cycles down to microseconds moves the wrap point off a power of two, and the
   unsigned elapsed-time arithmetic then breaks silently across the wrap.
3. **Command 0x41's payload is append-only.** It is the frozen version record.
   Appending a field is a protocol MINOR bump; moving, resizing or repurposing
   one is a MAJOR whether you meant it or not.
4. **A host selects its codec on the protocol MAJOR alone**, never the firmware
   version — otherwise every firmware rebuild breaks the host.
5. **No printf while the binary link is open.** A blocking transmit inside a
   frame corrupts RTU framing and stalls reception long enough to latch a UART
   overrun, which on this silicon kills reception permanently.
6. **Every ADC read path must call `HAL_ADC_ConfigChannel` and clear `PCSEL`.**
   See FINDINGS; two separate bugs came from paths that did not.
7. **The host never reports a sensed quantity for the phase channels.** They sit
   behind AFE gain that neither side knows. Volts at the ADC pin, and no
   further.
8. **Nothing in the Python library returns a status code or None-for-failure.**
   Every call produces its result or raises from `coaxial.errors`.
9. **AFE_ON decides what a reading means**, because it powers the ADC reference
   and not just the signal path. With it off, channels read exact mid-scale and
   the NTC reports exactly 25.00 °C — a plausible number that is not a
   measurement. The gate is a **label, not a refusal**: `analog_read` returns
   the codes either way, under a line that cannot be mistaken for one of them.
   Refusing was tried and was worse — asked for the raw codes with the AFE
   deliberately off, a model with no numbers to report wrote "Mid-scale …
   25.00 C" out of the warning text itself, so the refusal caused the invented
   reading it was meant to prevent. The library's cooked readings
   (`read_all`, `ntc_temperature`, `dcbus_voltage`) still refuse, because those
   claim a physical quantity and there is none to claim.
10. **The board is a dumb slave. No limits, no expected values, anywhere in
    the firmware or in this repository's tests.** It reports raw codes; pass/fail
    against real thresholds belongs to a test executive on the line, beside the
    calibrated instruments. The single exception is `self_test`, which judges
    only what it can prove from its own registers and flash.
11. **The DC link divider's headroom is deliberate.** 49.9k/2.2k gives 78.15 V
    full scale against a 63 V rating, 24 % of margin. Do not "optimise" it away;
    on an inverter the over-rating transient is exactly what you want recorded
    rather than clipped.

## Two things that will waste your time if you do not know them

**JTAG connect-under-reset does not work on this setup.** Any connect that
asserts NRST fails with `Unable to get core ID`. Use
`-c port=JTAG mode=Normal reset=SWrst`, or just use SWD. This is the probe
firmware, not the board — the cabling is fine and was proven so. Also: end a
programmer invocation with `--start`, not `-hardRst`, or the core is left halted.

**The AFE switch (PB2) powers the ADC reference.** With it off, every channel
reads exact mid-scale and the NTC reports exactly 25.00 °C — a plausible-looking
number that is not a measurement. Enable it before believing anything analog.

## Tooling traps in this environment

- Writing C escape sequences through a Python string inside a bash heredoc
  mangles them: `\r\n` arrives as a real CR+LF. Build the backslash with
  `chr(92)`, or write the C to a file with a quoted heredoc and splice it.
- `Core/Src/main.c` is LF-terminated. Python `open(...)` without
  `newline=''`/`newline='\n'` will convert it to CRLF on write.
- Long `cat > file <<'EOF'` heredocs get truncated. Split them.
