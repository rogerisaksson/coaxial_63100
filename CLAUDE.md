# coaxial_63100

You are a senior embedded-firmware engineer, a senior Python developer, and an
expert at prompting local LLMs. Judge a firmware bug, a host-library design or a
SYSTEM prompt directly, with that authority — do not hedge as a generalist.

Control firmware and a Python host library for a **coaxial BLDC inverter**: a
three-phase drive whose PCB sits coaxially behind the stator of an outrunner. The
name is the rating — **63 V, 100 A**, the current instantaneous within the FETs'
SOA. STM32H753VIT6 at 475 MHz; an analog front end feeding three differential
phase-sense channels, a DC link sense and an NTC; one UART carrying either a
text console or binary Modbus RTU.

Behind a spinning rotor there is little airflow, so the thermal channel is not
decoration, and the phase sense sits inside a switching bridge — which is the
context for every noise figure in these documents.

## Scope: instrumentation, not yet a motor controller

**No timer is configured.** The `.ioc` enables ADC1/2/3, USART3, CORTEX_M7, RCC,
SYS, DEBUG, MEMORYMAP, NVIC and VREFBUF, and nothing else: no PWM, no
commutation, no gate drive, no current loop. This is a measurement and bring-up
platform. Nothing has run near 63 V or 100 A, and no measured value is recorded
here — invariant 10.

VREFBUF is deliberately **disabled**, VREF+ high-impedance, so the AFE drives the
ADC reference. That is the mechanism behind invariant 9, and why
`ADC_VREF_VOLTAGE 3.3f` is an assumption about a rail, not a property of the chip.

| Read | Before |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | touching the source layout |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | changing anything on the wire |
| [docs/HARDWARE.md](docs/HARDWARE.md) | interpreting any measurement |
| [docs/MODELS.md](docs/MODELS.md) | changing the local model, its tag or its tools |
| [docs/FINDINGS.md](docs/FINDINGS.md) | **investigating anything** — it records what is already ruled out |

## Commands

```powershell
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Check    # what is missing
                            # -Yes installs the lot (winget, python packages,
                            # ST bundles, CubeMX, ST-Link driver, ollama);
                            # -FirmwarePackage X.zip adds FW_H7 to CubeMX.
. .\env.ps1                 # PATH + board_prompt, dbg, board, cbuild, cflash, cubemx
```

```bash
cube-cmake --build --preset Debug        # must be zero warnings
STM32_Programmer_CLI -c port=SWD mode=UR -d build/Debug/coaxial_63100.elf -v --start

cd host
python -m coaxial all                    # CLI against the board
python examples/read_board.py            # measure, judge nothing
python tools/run_tests.py                # every suite, one parsed tally
python tools/run_tests.py --offline      #   ...minus the ones needing the board
python tools/build_and_flash.py          # build (+flash): --build-only, --flash-only
python -m coaxial_mcp --port COM4        # MCP server, stdio
python -m coaxial_ollama --plan coaxial_ollama/plans/bringup.yaml
python -m coaxial_ollama.capability      # which local model this machine should run
python dbg.py --repl                     # prompt loop; /py and /sh cost no tokens
python dbg.py --repl --simulated         # the same, no cable
python dbg.py -m auto -q "read the NTC"  # one question, the model this machine fits
python dbg.py -q "run the test suites, build and flash, tell me if anything failed"
```

Suites: `test_ollama.py` (386, offline), `test_simulated.py` (26, offline),
`test_mcp.py` (39, **needs the board**), `test_conformance.py` (43, needs the
board, `--conformance`).

The ST toolchain is not on the system PATH — arm-gcc, cmake, ninja and
`STM32_Programmer_CLI` live under `%LOCALAPPDATA%\stm32cube\bundles\`, fetched by
`cube.exe` (the bundle manager in the STM32 VS Code extension; `cube bundle
install --yes NAME` needs no ST account). `env.ps1` puts the newest of each on
PATH for one shell. The board's VCP is **COM4**; the probe is an STLINK-V3SET.

## After a change lands

Once a change is made, tested and verified, ask — every time, as the last step of
the change, the same way running the tests is:

> **Continue, or commit and push?**
> *Continue* — keep working in this session
> *Commit and push* — stage, commit, push to origin/main

Two options, nothing else. A session here runs several small fix-test cycles, and
asking after each is what keeps the tree from either committing mid-investigation
or piling up unpushed work nobody asked to hold onto. Measured: the rule was
added, and the very next change landed with a summary and no question.

## Spend the local model, not the expensive one

There is a local model on this machine with the board's fifteen tools wired to
it. It is free per token and standing next to the hardware. Anything routine,
mechanical, or already covered by its tools belongs there by default.

```powershell
board_prompt -Ask "read the NTC and give me the temperature"
python dbg.py -m auto -q "..."          # from host/, one layer down
```

Both pick the tag this machine can run and **pull it if absent**, so "the model
is not installed" is never a reason to answer from memory. `board_prompt` also
tunes the ollama daemon so it stops crashing mid-session (docs/MODELS.md).

**Reuse a model already loaded.** Check `ollama ps` first: two models at once is
how a 16 GB card is asked for two copies of the weights.

| Question | Who answers |
|---|---|
| What does the board read now? Is the AFE on? Temperature, DC link, frame counters? | **the local model** — offer the command, then stop |
| Is this channel odd? What does `self_test` say? | **the local model**, then read FINDINGS before investigating |
| Does it still build/flash/pass? | **the local model** — `dbg -q "run the test suites, then build and flash, tell me if anything failed"`; the tools report the suite's own tally and the build's own exit code, parsed, not summarised |
| Why is this C function written this way? `Board/` or `Comms/`? Is this a protocol MAJOR? | **you** — it is bad at code and design, and FINDINGS records it inventing hardware constants |
| What is the wire format of command 0x41? | **you**, from docs/PROTOCOL.md |

A failing build or a regressed test is still yours to judge. The rule is about
who *runs* the loop, not who decides what its result means.

**And when the answer is for you, not the user, skip the model.**
`tools/run_tests.py` and `tools/build_and_flash.py` already print a parsed
tally and a real exit code in four lines. Verifying your own change mid-edit by
asking the local model to run them adds a model load, a turn and a paraphrase
risk for a result the script states directly. Route it by who reads the answer:

| The answer is for | Do |
|---|---|
| the user, who asked | the local model — it is free and it is at the bench |
| you, mid-change | run the script yourself; it is fewer tokens than asking |
| nobody yet (exploring) | neither — read FINDINGS first |

### Stop and ask first

**Before touching the board to answer a question, ask whether it is worth
tokens.** Two shapes of request, and the second is easy to miss:

* *measure something* — read a channel, fetch data, check the AFE, take a burst;
* *reach the local model at all* — "I want to prompt the local model", "how do I
  ask it". Answering with instructions is the same mistake in a different coat:
  they are not asking to be taught the command, they are asking to be at the
  prompt.

Either way, ask **minimally**:

> **Local model, or here?**
> *Local model* — board_prompt
> *Here* — I drive the library

No paragraph about token cost, no preview of the output, no third option.

On *Local model*: hand over the shortest way to the prompt and **stop**. The
shortest way is a click, not a command to retype:

    Terminal panel > the v beside + > Board prompt

That profile is in `.vscode/settings.json`; **Ctrl+Shift+B** runs the "Ask the
board" task in the same panel for one question. Only when the user is not in VS
Code is the command itself the answer:

    board_prompt -Ask "read all channels, the DC link and the NTC"

Nothing else. Do not run it, do not paraphrase what it would say, do not take the
reading anyway to check. And **do not spawn a window** — `Start-Process
powershell` puts the answer in front of the editor, not where the user is
working.

**None of this applies when the board is instrumentation for work already
agreed**: verifying a change just made, reproducing a bug, writing a capture tool
that needs a live link. Then it is a test fixture, not the subject of a question.
The test is who the answer is for — the user, or the code.

Where the user chose *here*, relay what was measured. Do not re-derive it and do
not decorate it with a verdict. If it is wrong about the hardware, that is a
finding worth writing down — docs/MODELS.md.

It is a **dumb-slave interface to a dumb slave**. It reports; it does not judge.
Invariant 10 applies to it exactly as it applies to the firmware.

## How to write here

Terse, friendly, and never at the cost of a fact. A comment earns its place by
saying something the code cannot: a measured number, why an obvious approach was
rejected, a failure that has already happened. It loses its place by restating
the code, telling the story twice, or hedging.

```python
# Measured: ch=['bus'] was refused with the channel it meant in the refusal.   # good
# We should probably consider that a model might, in some cases, spell it...   # cut
```

Three lines is a long comment. A docstring says what the function is for and
what a caller has to know — not a narrative of how it came to be. Keep the
measurement, drop the paragraph around it.

## Layout

```
Core/        CubeMX-generated. main.c is 582 lines and holds ONLY CubeMX
             functions plus main(). Keep it that way.
Board/       this hardware, behind Comms/Inc/board.h
Comms/       the comms stack: cmd over proto over dev, plus the console
Modbus/      the protocol. Portable C11, no HAL in crc/slave/rtu.
host/        Python: coaxial/ library, coaxial_mcp/ server, coaxial_ollama/
             runner and dbg.py, testline/, tests, tools
setup.ps1        one-time environment setup; -Check changes nothing
env.ps1          per-shell PATH and the board_prompt/dbg/board/cbuild/cflash aliases
board_prompt.ps1 preflight + prompt loop; orchestration only
board_prompt/    Say, ComPort, Ollama, ModelChoice, Relaunch — one concern per
                 file, dot-sourced, not meant to run alone
docs/            this documentation
```

`cmake/stm32cubemx/CMakeLists.txt` is regenerated by CubeMX — **never add sources
there**. New sources go in the root `CMakeLists.txt` user blocks.

## Invariants

Break one and something works until it doesn't.

1. **The protocol core stays hardware-free.** `modbus_crc.c`, `modbus_slave.c`
   and `modbus_rtu.c` include only `<stdint.h>`, `<stddef.h>`, `<stdbool.h>` and
   `<string.h>`. That is what makes them host-testable. Only
   `Comms/Src/dev_usart3.c` touches the USART.
2. **RTU timing is in raw `DWT->CYCCNT` ticks, never microseconds.** Dividing
   cycles down moves the wrap off a power of two, and the unsigned elapsed-time
   arithmetic then breaks silently across it.
3. **Command 0x41's payload is append-only.** It is the frozen version record.
   Appending a field is a protocol MINOR; moving, resizing or repurposing one is
   a MAJOR whether you meant it or not.
4. **A host selects its codec on the protocol MAJOR alone**, never the firmware
   version — otherwise every rebuild breaks the host.
5. **No printf while the binary link is open.** A blocking transmit inside a
   frame corrupts RTU framing and latches a UART overrun, which on this silicon
   kills reception permanently.
6. **Every ADC read path must call `HAL_ADC_ConfigChannel` and clear `PCSEL`.**
   Two separate bugs came from paths that did not — FINDINGS.
7. **The host never reports a sensed quantity for the phase channels.** They sit
   behind AFE gain neither side knows. Volts at the ADC pin, no further.
8. **Nothing in the Python library returns a status code or None-for-failure.**
   Every call produces its result or raises from `coaxial.errors`.
9. **AFE_ON decides what a reading means**, because it powers the ADC reference,
   not just the signal path. With it off, channels read exact mid-scale and the
   NTC reports exactly 25.00 °C — plausible, and not a measurement. The gate is a
   **label, not a refusal**: `analog_read` returns the codes either way under a
   line that cannot be mistaken for one of them. Refusing was tried and was
   worse: asked for raw codes with the AFE deliberately off, a model with no
   numbers to report wrote "Mid-scale … 25.00 C" out of the warning text itself.
   The cooked readings (`read_all`, `ntc_temperature`, `dcbus_voltage`) still
   refuse, because those claim a physical quantity and there is none to claim.
10. **The board is a dumb slave. No limits, no expected values, anywhere in the
    firmware or in this repository's tests.** It reports raw codes; pass/fail
    against real thresholds belongs to a test executive beside the calibrated
    instruments. The one exception is `self_test`, which judges only what it can
    prove from its own registers and flash.
11. **The DC link divider's headroom is deliberate.** 49.9k/2.2k gives 78.15 V
    full scale against a 63 V rating, 24 % of margin. Do not optimise it away; on
    an inverter the over-rating transient is what you want recorded, not clipped.

## Two things that will waste your time

**JTAG connect-under-reset does not work here.** Any connect asserting NRST fails
with `Unable to get core ID`. Use `-c port=JTAG mode=Normal reset=SWrst`, or SWD.
This is the probe firmware, not the board — the cabling was proven fine. End a
programmer invocation with `--start`, not `-hardRst`, or the core is left halted.

**The AFE switch (PB2) powers the ADC reference.** With it off every channel
reads exact mid-scale and the NTC reports exactly 25.00 °C. Enable it before
believing anything analog.

## Tooling traps

- Writing C escape sequences through a Python string inside a bash heredoc
  mangles them: `\r\n` arrives as a real CR+LF. Build the backslash with
  `chr(92)`, or write the C to a file with a quoted heredoc and splice it.
- `Core/Src/main.c` is LF-terminated. Python `open(...)` without
  `newline=''`/`newline='\n'` converts it to CRLF on write.
- Long `cat > file <<'EOF'` heredocs get truncated. Split them.
