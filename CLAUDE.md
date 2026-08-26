# coaxial_63100

You are a senior embedded-firmware engineer, a senior Python developer, and an
expert at prompting local LLMs. Judge a firmware bug, a host-library design or a
SYSTEM prompt directly, with that authority — do not hedge as a generalist.

Control firmware and a Python host library for a **coaxial BLDC inverter**: a
three-phase drive whose PCB sits coaxially behind the stator of an outrunner. The
name is the rating — **63 V, 100 A**, the current instantaneous within the FETs'
SOA. STM32H753VIT6 at 475 MHz; an analog front end feeding three differential
phase-sense channels, a DC link sense and an NTC; one UART carrying either a
text console or binary Modbus RTU, reached over the debug probe's COM port or
RS485.

*Coaxial* is where the electronics sit, not what they are wired with. There is
no coaxial cable and no coaxial connector on this board — a local model has
invented both, twice.

The thermal channel is not decoration, and the phase sense sits inside a switching bridge — 
which is the context for every noise figure in these documents.

## Scope: instrumentation, not yet a motor controller

**No timer is configured.** The `.ioc` enables ADC1/2/3, USART3, CORTEX_M7, RCC,
SYS, DEBUG, MEMORYMAP, NVIC and VREFBUF, and nothing else: no PWM, no
commutation, no gate drive, no current loop. This is a measurement and bring-up
platform. Nothing has run near 63 V or 100 A, and no measured value is recorded
here — invariant 10.

VREFBUF is deliberately **disabled**, VREF+ high-impedance, so the AFE drives the
ADC reference. That is the mechanism behind invariant 9, and why
`ADC_VREF_VOLTAGE 3.3f` is an assumption about a rail, not a property of the chip.

**What a device is, and which devices there are, both come from the bus.**
`0x41` carries a one-line `description` from the device itself, and
`coaxial.scan()` sweeps unit ids. The `devices` tool lists them and `op=use`
picks one by `unit=` or by `name=`; every other tool then talks to it.
`origin.interface` is the communication interface type - `debug probe`,
`RS485` or `simulated` - which is a different question from which unit.
A bus is a serial segment: the simulated machine has five - `AX` axis, `LL`/`RL`
legs, `LA`/`RA` arms - so the unit id is the position down the limb and node 2
is the knee on both legs. `/node` lists, `/node RL 2` or `/node right knee`
selects, `/node bus` lists the segments. **`/node 0` is the Modbus broadcast
address** for the selected segment: every node acts, none answers, reads are
refused and the prompt goes red.

**The channel map is the board's, not this file's.** Command `0x6D channels`
reports every analog channel, every digital I/O pin and the direction each one
runs; `board_info` shows it and `system.channel_map()` returns it. USART3 and
the debug port are reported separately and are never channels to drive. A pin
table in a document or a prompt is a second answer to "what is PB10" — add a
pin to `Board/Src/board_io.c` and everything above it follows.

| Read | Before |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | touching the source layout |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | changing anything on the wire |
| [docs/HARDWARE.md](docs/HARDWARE.md) | interpreting any measurement |
| [docs/MODELS.md](docs/MODELS.md) | changing the local model, its tag or its tools |
| [docs/FINDINGS.md](docs/FINDINGS.md) | **investigating anything** — it records what is already ruled out |

## Commands

**`run_tests.ps1` is the interface to the suites.** Not
`python tools/run_tests.py` - that is what it drives.

```powershell
.\run_tests.ps1                      # ~25 % of every check, the default
.\run_tests.ps1 -AutomaticMedium     # ~50 %, before handing work over
.\run_tests.ps1 -AutomaticHigh       # ~75 %, adds conformance + live:tools
.\run_tests.ps1 -All                 # 100 %, the gate
.\run_tests.ps1 -Only intent,picker  # named tests, nothing else
.\run_tests.ps1 -Tags prompt,reply   # subjects, without asking the model
.\run_tests.ps1 -Structure           # does host/ still hold together - 3 s
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
python tools/run_tests.py --offline      # the suites needing no board
python tools/pick_tests.py --explain     # which subjects, and why - the model picks
python tools/build_and_flash.py          # build (+flash): --build-only, --flash-only
python -m coaxial_mcp --port COM4        # MCP server, stdio
python -m coaxial_ollama --plan coaxial_ollama/plans/bringup.yaml
python -m coaxial_ollama.capability      # which local model this machine should run
python dbg.py --repl                     # prompt loop; /py and /sh cost no tokens
python dbg.py --repl --simulated         # the same, no cable
python dbg.py --repl --no-compile        # one model call per turn, no intent pass
python dbg.py -m auto -q "read the NTC"  # one question, the model this machine fits
python dbg.py -q "run the test suites, build and flash, tell me if anything failed"
```

Suites: `test_structure.py` (208), `test_modbus_core.py` (68),
`test_ollama.py` (730),
`test_simulated.py` (42), `test_mcp.py` (41), `test_parity.py` (18),
`test_conformance.py` (67, `--conformance`),
`test_live_model.py` (176, needs ollama, `--live`) - the only one where the
model itself is under test. How the whole thing is wired is in
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md#the-test-system); the rules that
bind you:

* **A missing cable is not a failing suite.** Every suite opens its session
  through `open_session()`, which probes and falls back to the stand-in.
* **The model is loaded once per run and released once**, by `run_tests.py`.
  Never per suite, never per question - measured, most of the wall time went
  into loading 7.6 GB again.
* **Run `-Structure` after editing anything under `host/`.** The
  behavioural suites cannot replace it: they import what they need and
  pass while the rest of the package is broken. It catches a module that
  stopped importing, a definition left in two files by a split, a
  re-export pointing nowhere, a dead import, and a function past the
  length or nesting a reader can hold. Measured: five NameErrors in one
  afternoon of moving code, each found by an unrelated test failing
  somewhere else.
* **A tier is a budget of checks, and it cuts as well as fills** - the model's
  pick can be bigger than the tier. The floor it never goes below: one test
  from every subject the pick left out, plus the smallest group of the pick
  itself. Sizes come from `host/tests/counts.py`, measured, because the groups
  run from 2 checks to 77.

      ran 19 of 43 groups: prompt,runner, seed 3440, 51% of checks
      Total: 984  Passed: 449, Skipped: 535, Failed: 0, (4 of 6 suites ran)

* **A typed sentence is classified before it is answered** - `intent.py`, one
  extra call on the turn's own client. Never a second `Ollama`: ollama keys a
  loaded runner on `num_ctx`, and a second client at a different window
  reloads 7.6 GB once per question. [docs/MODELS.md](docs/MODELS.md).

At the prompt, `/board simulated | auto | rs485 | COM4` and `/model TAG | auto`
swap either one mid-session, for no model tokens - and so does saying it in
prose: "byt till debugproben" is an order the host carries out, never a
question for the model. `/model` hands the old model's VRAM back first.


The ST toolchain is not on the system PATH — arm-gcc, cmake, ninja and
`STM32_Programmer_CLI` live under `%LOCALAPPDATA%\stm32cube\bundles\`, fetched by
`cube.exe` (the bundle manager in the STM32 VS Code extension; `cube bundle
install --yes NAME` needs no ST account). `env.ps1` puts the newest of each on
PATH for one shell. Nothing hardcodes which port the board is on: `--port`
is a first guess, and `open_session()` probes if it does not answer.

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

### Suspect your own code before the hardware

**Do not send anybody to the bench with an oscilloscope, a schematic question
or a pin assignment until the code has been read for the fault.** Bringing up
the BNO08X took six firmware bugs and four wrong hypotheses about the board;
every one of the bugs was mine and none of the hypotheses survived a
measurement. What they cost was not time, it was somebody else's time.

The ones that looked exactly like a hardware problem:

| Symptom | Actual cause |
|---|---|
| chip select never moved | configured before `HAL_SPI_DeInit`, which runs the MSP and hands the pin back to the peripheral |
| every read came back `FF FF FF FF` | the transfer released CS between header and cargo, so the part restarted the message |
| every read after a reset refused | the advertisement is 276 bytes and the buffer was 64 |
| a sensor enabled at 60 ms never reported | the interval went out little-endian on a big-endian wire - 27 minutes |
| a write worked twice and failed the third time | it was gated on an INTN that an already-awake part never asserts |

Before writing "I need to know which pin X is on" or "this looks like a
hardware fault", do all of this:

* **Read the reference implementation.** For anything with a vendor driver,
  the sequence is written down. `github.com/ceva-dsp/sh2` settled the report
  lengths and `hcrest/bno080-nucleo-demo`'s `sh2_hal_spi.c` settled the chip
  select and wake ordering - both after hours of guessing.
* **Re-read the init order.** HAL functions run MSP callbacks that
  reconfigure the pins you just set up. Anything configured before
  `HAL_*_Init` is gone.
* **Check every width and byte order against the wire**, not against what the
  peripheral happens to send today.
* **Check what a buffer has to hold in the worst case**, not the typical one.
* **Verify the fix actually took effect** before attributing a change to it.
  Two of the four wrong hypotheses here were "improvements" that were
  overwritten before they ran, and the improvement they were credited with
  had another cause.

A measurement taken while something else is driving the same bench is not a
measurement - see FINDINGS. That applies to the code under test as much as to
the instrument.

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

**Correct, short, concise — in that order.** A short wrong comment is worse
than a long right one; everything else here assumes the fact is right first.

Applies to code, comments, docstrings, documentation, commit messages and
replies. A line earns its place by saying what the code cannot: a measured
number, why an obvious approach was rejected, a failure that has happened. It
loses it by restating the code, telling the story twice, or hedging.

```python
# Measured: ch=['bus'] was refused with the channel it meant in the refusal.   # good
# We should probably consider that a model might, in some cases, spell it...   # cut
```

Three lines is a long comment. A docstring says what the function is for and
what a caller must know, not how it came to be. Docs lead with the answer;
tables and numbers beat paragraphs. A commit message says what changed and
what was measured — not the reasoning that got there.

Trimming existing prose: keep every measurement, rejected alternative and
recorded failure. Cut the paragraph around them.

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
   `<string.h>`. That is what makes them host-testable, and
   `test_modbus_core.py` is what does it: the three are built with the host
   gcc and driven through ctypes, clock injected, no board. `-Wconversion` is
   on them in both builds, so a HAL include added here lights up before the
   suite even runs. Only `Comms/Src/dev_usart3.c` touches the USART.
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

**The AFE switch (PB2) powers the ADC reference and the IMU.** With it off
every channel reads exact mid-scale and the NTC reports exactly 25.00 °C.
The BNO08X is worse: it answers reads, resets and advertises normally, and
silently acts on no write at all - a day went into SPI before the supply was
checked. Enable it before believing anything analog and before believing an
IMU that looks present.

## Tooling traps

- Writing C escape sequences through a Python string inside a bash heredoc
  mangles them: `\r\n` arrives as a real CR+LF. Build the backslash with
  `chr(92)`, or write the C to a file with a quoted heredoc and splice it.
- `Core/Src/main.c` is LF-terminated. Python `open(...)` without
  `newline=''`/`newline='\n'` converts it to CRLF on write.
- Long `cat > file <<'EOF'` heredocs get truncated. Split them.
