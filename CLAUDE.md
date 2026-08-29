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
no coaxial cable and no coaxial connector on this board. **Known failure mode:**
a local model fills the gap and reports one anyway - seen twice. **Guard:** the
fitted parts come from `0x6D` kind 4, so what is on the board is answerable
without inferring it from the name.

The thermal channel is not decoration, and the phase sense sits inside a
switching power stage — the context for every noise figure in these documents.

## Scope: instrumentation, not yet a motor controller

**TIM1 is armed on request, and there is still no commutation.** The `.ioc`
enables sixteen IPs - ADC1/2/3, SPI2, SPI4, USART2, USART3, UART5, **TIM1**,
CORTEX_M7, RCC, SYS, DEBUG, MEMORYMAP, NVIC and VREFBUF. TIM1 is centre-aligned
at **50 kHz** (ARR 2375 off 237.5 MHz), dead time **DTG 19 = 80.0 ns**, break on
PE15 active low, AOE off so nothing re-arms itself. `Board_PwmInit()` starts the
counter with MOE clear and CCxE set, driving all six outputs to their idle
level: both FETs of every leg held off in hardware.

`rig.gates.arm()` is the only thing that sets MOE, and a duty write is
refused until it has been called - arming a power stage should be asked for by
name, not fall out of writing a level. It re-reads BDTR DTG first and refuses a
stage with no dead time, because the 2EDL8034 has no interlock of its own.

Measured 2026-08-27 with the drivers powered: every duty from 1 % to 100 %, no
supply trip, no overruns. All three legs at the same duty, so no volts between
phases and no phase current. **There is still no commutation and no current
loop** - what exists is a gate driver stage that can be switched and measured,
not one that can turn a motor.

The gate drivers and the FETs are fitted (2EDL8034 x3, IAUCN10S7N021 -
`electronics/`) and **their supply is not the MCU's to switch** - the Safe Torque
Off chain releases it, unlocked by a pilot tone on RS485. Nothing has run near
63 V or 100 A, and no measured value is recorded here — invariant 10.

VREFBUF is deliberately **disabled**, VREF+ high-impedance, so the AFE drives the
ADC reference. That is the mechanism behind invariant 9. Its source is U2, a
REF2033, driving `+3V3_ref` **and** `+1V65_bias` - one part sets the reference
and the differential mid-point, which is why they track. The 3.3 V is a specified
part rather than a rail nobody measured, and it lives in the calibration record
anyway: a rig with a calibrated meter beats a datasheet tolerance.

**What a device is, and which devices there are, both come from the bus.** `0x41`
carries a one-line `description` from the device itself, and `coaxial.scan()`
sweeps unit ids. The `devices` tool lists them and `op=use` picks one by `unit=`
or by `name=`; every other tool then talks to it. `origin.interface` is the
interface type - `debug probe`, `RS485` or `simulated` - a different question
from which unit.

A bus is a serial segment: the simulated machine has five - `AX` axis, `LL`/`RL`
legs, `LA`/`RA` arms - so the unit id is the position down the limb and node 2 is
the knee on both legs. `/node` lists, `/node RL 2` or `/node right knee` selects,
`/node bus` lists the segments. **`/node 0` is the Modbus broadcast address** for
the selected segment: every node acts, none answers, reads are refused and the
prompt goes red.

**The channel map is the board's, not this file's.** Command `0x6D channels`
reports every analog channel, every digital I/O pin and the direction each one
runs; `board_info` shows it and `system.channel_map()` returns it. USART3 and the
debug port are reported separately and are never channels to drive. A pin table
in a document or a prompt is a second answer to "what is PB10" — add a pin to
`Board/Src/board_io.c` and everything above it follows.

**What is fitted comes from the board too.** `0x6D channels` kind 4 is the parts
list — name, what it does, where it sits, **what powers it**, and whether it
answered. `board_info kind=parts`, `system.channel_map()['parts']` and the local
model's `parts` kind all read it off the wire.

That last column is not decoration. **Problem:** AFE_ON powers the BNO08X as
well as the analog front end, and with it off the part answers reads, resets and
advertises normally while acting on no write at all - so every symptom points at
SPI, and a day was spent there before the supply was checked. **Fix:** `power`
is a column in the parts list, and `Board_ImuInit` refuses while PB2 is low
rather than half-working.

**Adding hardware is one row, and nothing else.** A new part means a row in
`s_parts` in `Board/Src/board_io.c`, its pins in `s_digital` beside it, and — if
it needs one — a probe case so `state` is measured rather than asserted. Do not
then add it to a document, a prompt, a host table or a tool description: those
are second answers to a question the firmware already settles, and they are the
ones that go stale. Check it landed with:

```powershell
python -c "import coaxial; [print(p) for p in coaxial.connect([1])[0].system.channel_map()['parts']]"
board_prompt -Ask "vad sitter på kortet?"    # the model, off the same wire
```

| Read | Before |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | touching the source layout |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | changing anything on the wire |
| [docs/HARDWARE.md](docs/HARDWARE.md) | interpreting any measurement |
| [docs/MODELS.md](docs/MODELS.md) | changing the local model, its tag or its tools |
| [docs/FINDINGS.md](docs/FINDINGS.md) | **investigating anything** — it records what is already ruled out |
| [docs/TODO.md](docs/TODO.md) | picking up work — what is done and measured, and what is still arithmetic |

## Commands

**`run_tests.ps1` is the interface to the suites.** Not
`python tools/run_tests.py` - that is what it drives.

```powershell
.\run_tests.ps1                      # ~25 % of every check, the default
.\run_tests.ps1 -AutomaticMedium     # ~50 %, before handing work over
.\run_tests.ps1 -AutomaticHigh       # ~75 %, adds conformance + live:tools
.\run_tests.ps1 -All                 # 100 %, the gate
.\run_tests.ps1 -Depth 40            # any 5 % step, when none of the four fits
.\run_tests.ps1 -Scope test_mcp.py   # those files only, whatever the depth
.\run_tests.ps1 -Only intent,picker  # named tests, nothing else
.\run_tests.ps1 -Tags prompt,reply   # subjects, without asking the model
.\run_tests.ps1 -Structure           # does host/ still hold together - 3 s
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Check    # what is missing
                            # -Yes installs the lot (winget, python packages,
                            # ST bundles, CubeMX, ST-Link driver, ollama);
                            # -FirmwarePackage X.zip adds FW_H7 to CubeMX.
. .\env.ps1                 # PATH + board_prompt, dbg, board, cbuild, cflash, cubemx
```

**Refusals come from the board.** Anything taking parameters answers `u8 took`
and, on a refusal, the board's own words for what is wrong and what to do. The
host validates only what stops a request being formed and repeats the rest.
Adding a check means adding its sentence beside it - not a code the host maps,
and not a list of causes in a docstring. docs/PROTOCOL.md has the wire.

**`Coaxial63100` is the front door.** `host/coaxial/rig.py`, and what all four
views use. It owns the AFE preflight (invariant 9) and puts the supply back the
way it found it, Ctrl+C included.

**The host is three interfaces and the parts that answer them** -
`Acquisition` (`coaxial/acquisition.py`), `PolledSensor` (`sensor.py`) and
`GateControl` (`gates.py`). Each has a real implementation and a simulated one,
so a name drifting between them fails at construction instead of on the first
call that reaches for it. Add a method to a subsystem that has a stand-in and
add it to both, or put it on neither. `GateStage` beside them is concrete: the
arming policy, and there is one of that. docs/ARCHITECTURE.md has the table.

```python
from coaxial import Coaxial63100
with Coaxial63100(port='COM4') as daq:          # simulated_device=True: no cable
    daq.set_time_from_pc()                      # the board counts cycles, not time
    daq.configure(['Phase U', 'NTC'], accumulate=8)
    daq.write(digital={'UART5_TERM': True})
    daq.start()
    for block in daq.blocks(20):
        r = block[-1]
        print(r['time'], r['NTC'] / r['samples'])   # a value is a SUM of `samples`
```

`python_examples/daq_session.py` is that flow as a notebook, in 91 lines.

```bash
cube-cmake --build --preset Debug        # must be zero warnings
STM32_Programmer_CLI -c port=SWD mode=UR -d build/Debug/coaxial_63100.elf -v --start

cd host
python -m coaxial all                    # CLI against the board
python examples/read_board.py            # measure, judge nothing
python tools/run_tests.py --offline      # the suites needing no board
python tools/pick_tests.py --explain     # which subjects, and why - the model picks
python tools/build_and_flash.py          # build (+flash): --build-only, --flash-only
python tools/switch.py --sweep 5,95 -p 10 -s 120  # switch now, in the
                                        # background; --stop disarms and exits
python -m coaxial_mcp --port COM4        # MCP server, stdio
python -m coaxial_ollama --plan coaxial_ollama/plans/bringup.yaml
python -m coaxial_ollama.capability      # which local model this machine should run
python dbg.py --repl                     # prompt loop; /py and /sh cost no tokens
python dbg.py --repl --simulated         # the same, no cable
python dbg.py --repl --no-compile        # one model call per turn, no intent pass
python dbg.py -m auto -q "read the NTC"  # one question, the model this machine fits
python dbg.py -q "run the test suites, build and flash, tell me if anything failed"
```

Eighteen suites, 1767 checks, sized from `host/tests/.counts.json` and so measured
rather than remembered: `test_structure.py` (374), `test_ollama_tools.py`
(218), `test_ollama_runner.py` (214), `test_simulated.py` (189),
`test_live_model.py` (146, needs ollama, `--live`), `test_ollama_prompt.py`
(113), `test_conformance.py` (110, `--conformance`), `test_ollama_link.py`
(96), `test_modbus_core.py` (68), `test_mcp.py` (44), `test_shtp_core.py` (38),
`test_ollama_render.py` (32), `test_parity.py` (30), `test_ollama_board.py`
(28), `test_ollama_bus.py` (28), `test_ollama_reply.py` (23),
`test_ollama_language.py` (12), `test_bench.py` (4, the board's loop rates
against a recorded baseline). How it is wired is in
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md#the-test-system); the rules that bind
you:

* **A missing cable is not a failing suite.** Every suite opens its session
  through `open_session()`, which probes and falls back to the stand-in.
* **The model is loaded once per run and released once**, by `run_tests.py`.
  Never per suite, never per question - measured, most of the wall time went
  into loading 7.6 GB again.
* **Run `-Structure` after editing anything under `host/`.** The behavioural
  suites cannot replace it: they import what they need and pass while the rest
  of the package is broken. It catches a module that stopped importing, a
  definition left in two files by a split, a re-export pointing nowhere, a dead
  import, and a function past the length or nesting a reader can hold. Measured:
  five NameErrors in one afternoon of moving code, each found by an unrelated
  test failing somewhere else.
* **A tier is a budget of checks, and it cuts as well as fills** - the model's
  pick can be bigger than the tier, and does not get to spend past it. The floor
  it never goes below: one test from every subject the pick left out, plus the
  smallest group of the pick itself. Sizes come from `host/tests/counts.py`,
  measured, because the groups run from 2 checks to 77.

      ran 19 of 43 groups: prompt,runner, seed 3440, 51% of checks
      Total: 984  Passed: 449, Skipped: 535, Failed: 0, (4 of 6 suites ran)

  Measured, and the reason the clamp exists: on the 25 % tier the tier had
  already dropped the live suite, the model's pick put `live:all` back, and the
  cheapest run there is took 398 s of which 352 were that one suite. It now says
  what it refused - `the 25% tier does not stretch to: live:all`.

* **Any 5 % step is a tier.** Suites join in order of seconds per check, so the
  first of a budget buys the cheapest checks there are - measured, per check:
  simulated 0.003 s, ollama 0.019, core 0.03, parity 0.13, mcp 0.14, conformance
  0.29, live 4.6. The `test_ollama_*` suites are in from the first tier and
  narrow *themselves*; that is where the fine resolution lives, because 763 of
  this tree's 1767 checks are in those nine files.

* **The model is not asked when the path map already knows.** Where every
  changed file matched an explicit rule and the answer is `CHEAP` - structure,
  core, shtp, simulated, none of which need a board or ollama - the pick is
  settled without a model. Asking costs a 7.6 GB load to be told what the map
  said, and the answer can only come back wider. Editing a demo wrapper is three
  seconds, not seven minutes.

* **Ctrl+C is `STOPPED`, exit 130, not `FAILED`** - and the `finally` hands the
  model back. Killing the run from outside does not: measured, 8.4 GB stayed on
  the card until it was released by hand.

* **A typed sentence is classified before it is answered** - `intent.py`, one
  extra call on the turn's own client. Never a second `Ollama`: ollama keys a
  loaded runner on `num_ctx`, and a second client at a different window reloads
  7.6 GB once per question. [docs/MODELS.md](docs/MODELS.md).

At the prompt, `/board simulated | auto | rs485 | COM4` and `/model TAG | auto`
swap either one mid-session, for no model tokens - and so does saying it in
prose: "byt till debugproben" is an order the host carries out, never a question
for the model. `/model` hands the old model's VRAM back first.

The ST toolchain is not on the system PATH — arm-gcc, cmake, ninja and
`STM32_Programmer_CLI` live under `%LOCALAPPDATA%\stm32cube\bundles\`, fetched by
`cube.exe` (the bundle manager in the STM32 VS Code extension; `cube bundle
install --yes NAME` needs no ST account). `env.ps1` puts the newest of each on
PATH for one shell. Nothing hardcodes which port the board is on: `--port` is a
first guess, and `open_session()` probes if it does not answer.

## Do not run the suites to look busy

**While a bug is live, run the narrowest thing that could disprove the current
hypothesis - never the full suite.** `-All` is eight minutes and answers a
question nobody asked. The suites are the gate *after* a change, not a step in
finding one.

**Problem, measured:** chasing why two of three gate driver stages ran 15 C
hotter than the third, the full suite was started three times. None of the 1736
checks could have said anything about it - the difference was on the bench.
**What worked instead:** a 600-sample pin count and a register dump.

The narrow thing is usually one of: read the register, count the samples, run the
one suite whose name matches what changed.

## Green before the next thing

**Fix the demo code and get the suites passing before moving on to the next item
on a list.** Not after it, not once the list is done. A failing check carried
forward stops being information: by the third item nobody can tell which change
broke it, and the run that would have said so is the one that was skipped.

This includes failures that were already there when the work started. Say they
are pre-existing, then fix them - reporting a red suite and carrying on is how it
stays red. **Problem, measured:** nine failures in `test_ollama_render` and
`test_ollama_reply` were labelled pre-existing and carried through four more
items, by which point the change that broke them was no longer identifiable.
**Fix:** the label is a note on the way to the fix, not a substitute for it.

## After a change lands

Once a change is made, tested and verified, ask — every time, as the last step of
the change, the same way running the tests is:

> **Continue, or commit and push?**
> *Continue* — keep working in this session
> *Commit and push* — stage, commit, push to origin/main

Two options, nothing else. A session here runs several small fix-test cycles, and
asking after each is what keeps the tree from either committing mid-investigation
or piling up unpushed work nobody asked to hold onto. **Known failure mode:** the
question gets replaced by a summary and the session carries on - it happened on
the first change after this rule was written, which is why it is a rule and not
a preference.

## Spend the local model, not the expensive one

There is a local model on this machine with the board's twenty tools wired to it.
It is free per token and standing next to the hardware. Anything routine,
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
| Why is this C function written this way? `Board/` or `Comms/`? Is this a protocol MAJOR? | **you** — measured failure mode: on design questions it substitutes plausible hardware constants (FINDINGS), and a wrong one here is not visibly wrong |
| What is the wire format of command 0x41? | **you**, from docs/PROTOCOL.md |

A failing build or a regressed test is still yours to judge. The rule is about
who *runs* the loop, not who decides what its result means.

**And when the answer is for you, not the user, skip the model.**
`tools/run_tests.py` and `tools/build_and_flash.py` already print a parsed tally
and a real exit code in four lines. Asking the local model to run them mid-edit
adds a model load, a turn and a paraphrase risk for a result the script states
directly. Route it by who reads the answer:

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
  ask it". Answering with instructions misses this the same way: they are not
  asking to be taught the command, they are asking to be at the prompt.

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

**Do not send anybody to the bench with an oscilloscope, a schematic question or
a pin assignment until the code has been read for the fault.** Bringing up the
BNO08X turned up six firmware defects and four hypotheses about the board, none
of which survived a measurement. The cost of a wrong hypothesis here is somebody
else's bench time, which is why the order matters.

Each of these presented as a hardware fault and was fixed in firmware:

| Symptom | Actual cause |
|---|---|
| chip select never moved | configured before `HAL_SPI_DeInit`, which runs the MSP and hands the pin back to the peripheral |
| every read came back `FF FF FF FF` | the transfer released CS between header and cargo, so the part restarted the message |
| every read after a reset refused | the advertisement is 276 bytes and the buffer was 64 |
| a sensor enabled at 60 ms never reported | the interval went out little-endian on a big-endian wire - 27 minutes |
| a write worked twice and failed the third time | it was gated on an INTN that an already-awake part never asserts |

Before writing "I need to know which pin X is on" or "this looks like a hardware
fault", do all of this:

* **Read the reference implementation.** For anything with a vendor driver, the
  sequence is written down. `github.com/ceva-dsp/sh2` settled the report lengths
  and `hcrest/bno080-nucleo-demo`'s `sh2_hal_spi.c` settled the chip select and
  wake ordering - both after hours of guessing.
* **Re-read the init order.** HAL functions run MSP callbacks that reconfigure
  the pins you just set up. Anything configured before `HAL_*_Init` is gone.
* **Check every width and byte order against the wire**, not against what the
  peripheral happens to send today.
* **Check what a buffer has to hold in the worst case**, not the typical one.
* **Verify the fix actually took effect** before attributing a change to it.
  Two of the four hypotheses here were "improvements" that were overwritten
  before they ran, and the improvement credited to them had another cause.

A measurement taken while something else is driving the same bench is not a
measurement - see FINDINGS. That applies to the code under test as much as to the
instrument.

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

`~/.claude/CLAUDE.md` says it and this file does not repeat it. What is specific
here: **keep every measurement, rejected alternative and recorded failure** when
trimming prose - cut the paragraph around them, never the number. FINDINGS is a
record, not documentation, and is not shortened.

## Layout

```
Core/        CubeMX-generated. main.c holds ONLY CubeMX functions, main(),
             the two poll calls the sensors need, and the STO keepalive
             toggle. Keep it that way.
electronics/ schematic and BOM - the authority on what is fitted
render/      the CAD export the attitude view draws from
Board/       this hardware, behind Comms/Inc/board.h
Comms/       the comms stack: cmd over proto over dev, plus the console
Modbus/      the protocol. Portable C11, no HAL in crc/slave/rtu.
host/        Python: coaxial/ library, coaxial_mcp/ server, coaxial_ollama/
             runner and dbg.py, testline/, tests, tools
demo.ps1         picks one of the live views; -Simulated for no cable
demos/           imu.ps1 attitude, angle.ps1 shaft angle, adc.ps1 meter bridge
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
   `test_modbus_core.py` is what does it: the three are built with the host gcc
   and driven through ctypes, clock injected, no board. `-Wconversion` is on them
   in both builds, so a HAL include added here lights up before the suite even
   runs. Only `Comms/Src/dev_uart.c` touches a USART.
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
7. **A conversion is named where it is defined, and defined once.** Every scaling
   parameter this board uses lives in the calibration record behind `0x6E`
   device 3 - reference, phase shunt and gain, DC link divider, four thermistor
   constants - never as a literal at a call site and never as a second copy in a
   host. The phase channels used to be exempt because their gain was unknown; it
   was traced off the schematic on 2026-08-26, so they report amperes now. What
   has not changed: **no number this board reports has been measured against an
   instrument.** Span before believing one.
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
    instruments. Two exceptions, both narrow:

    * `self_test`, which judges only what it can prove from its own registers
      and flash.
    * **The thermal envelope**, because a board that cooks itself is not a
      measurement problem. The distinction that keeps this honest: the board
      never calls a reading good or bad, it *acts* - at a ceiling it drops MOE
      and every gate goes to its idle level, the same path the break uses. The
      ceilings are not the firmware's opinion either; they live in the
      calibration record beside the thermistor constants (invariant 7), so the
      board holds a limit it was given rather than one it invented. The margin
      is reported; the verdict still is not.
11. **The DC link divider's headroom is deliberate.** 49.9k/2.2k gives 78.15 V
    full scale against a 63 V rating, 24 % of margin. Do not optimise it away; on
    an inverter the over-rating transient is what you want recorded, not clipped.

## Two things that will waste your time

**JTAG connect-under-reset does not work here.** Any connect asserting NRST fails
with `Unable to get core ID`. Use `-c port=JTAG mode=Normal reset=SWrst`, or SWD.
This is the probe firmware, not the board — the cabling was proven fine. End a
programmer invocation with `--start`, not `-hardRst`, or the core is left halted.

**The AFE switch (PB2) powers the ADC reference and the IMU.** With it off every
channel reads exact mid-scale and the NTC reports exactly 25.00 °C. The BNO08X is
worse: it answers reads, resets and advertises normally, and silently acts on no
write at all, so the fault presents as SPI - a day was spent there before the
supply was checked. Enable it before believing anything analog and before
believing an IMU that looks present.

## Tooling traps

- Writing C escape sequences through a Python string inside a bash heredoc
  mangles them: `\r\n` arrives as a real CR+LF. Build the backslash with
  `chr(92)`, or write the C to a file with a quoted heredoc and splice it.
- `Core/Src/main.c` is LF-terminated. Python `open(...)` without
  `newline=''`/`newline='\n'` converts it to CRLF on write.
- Long `cat > file <<'EOF'` heredocs get truncated. Split them.
