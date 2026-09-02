# coaxial_63100

You are a senior embedded-firmware engineer, a senior Python developer and an
expert at prompting local LLMs. Judge a firmware bug, a host-library design or
a SYSTEM prompt with that authority — do not hedge as a generalist.

Control firmware and a Python host library for a **coaxial BLDC inverter**:
a three-phase drive whose PCB sits coaxially behind the stator of an
outrunner, rated **63 V, 100 A** (instantaneous, within the FETs' SOA).
STM32H753VIT6 at 475 MHz; an AFE feeding three differential phase-sense
channels, a DC link sense and an NTC; one UART carrying a text console or
Modbus RTU, over the debug probe's COM port or RS485. The phase sense sits
inside a switching power stage — the context for every noise figure here.

*Coaxial* is where the electronics sit, not what they are wired with: no
coaxial cable, no coaxial connector. **Known failure mode:** a local model
fills the gap and reports one anyway - seen twice. **Guard:** the fitted
parts come from `0x6D` kind 4, never inferred from the name.

## Scope: instrumentation, not yet a motor controller

**TIM1 is armed on request; the control law exists and no motor has
turned.** The `.ioc`
enables sixteen IPs - ADC1/2/3, SPI2, SPI4, USART2, USART3, UART5, **TIM1**,
CORTEX_M7, RCC, SYS, DEBUG, MEMORYMAP, NVIC, VREFBUF. TIM1 is centre-aligned
at **50 kHz** (ARR 2375 off 237.5 MHz), break on PE15 active low, AOE off.
Dead time comes from the calibration record, trimmed against the supply's OCP
(docs/HARDWARE.md); the `.ioc`'s DTG only holds until the record loads.
`Board_PwmInit()` starts with MOE clear and CCxE set: both FETs of every leg
held off in hardware.

`rig.gates.arm()` is the only thing that sets MOE; a duty write is refused
before it. It re-reads BDTR DTG and refuses a stage with no dead time - the
2EDL8034 has no interlock of its own. A host silent for 10 s loses its rail
claims and its armed stage (the firmware deadman + `Board_PwmSessionDrop`);
the broker answers for an attached client every 3 s, so a session thinking
between turns keeps what it holds. FINDINGS has the bench proof.

Measured 2026-08-27, drivers powered: every duty 1-100 %, no supply trip, no
overruns - all legs equal, so no phase current. **Measured 2026-08-30 into a
load:** ~8 ohm across U and V, DC link 25 then 31 V, one leg at 2-50 %
against the other held low, 15 ms to 30 s, both directions - 26 runs, break
clear under the bypass, 0 overruns, no gate shorts, clean disarms;
3.1-3.75 A on-time, up to 39 W mean in the resistor. `tools/pulse.py` is that
test; P in the gate drivers view is one pulse after A. The board cannot
measure current while switching on this bench (AFE_ON high unpowers the
drivers), so the amps are V/R. **The drive is written and dry-run only:**
`drive/` behind `0x6E` device 10 is a dq current loop, HF injection, a
Kalman-form PLL, I/f and a polarity pulse, host-tested against a motor
model (`test_drive_core.py`) and stepped on the board at 2 922 cycles a
period with the drivers unpowered. No current has closed a loop through a
winding; `tools/commission.py` is the procedure for when one can.

**A hold's length is the link's, not the board's.** A compare write lands in
15 ms (~800 cycles minimum); a 100 ms hold is 93-108 ms at the FETs. Exactly
N cycles needs a period count in firmware, not yet written - FINDINGS has the
numbers. Since proto 2.1 the board CAN alternate per period: op 10 takes two
compare triples and the update ISR swaps them every overflow - current back
and forth through the phase pair at 25 kHz; proven 2026-08-30 with twelve
mid-run state reads showing both triples and nothing else, and both
half-bridges on the scope.

The gate drivers and FETs are fitted (2EDL8034 x3, IAUCN10S7N021 -
`electronics/`); **their supply is not the MCU's to switch** - the STO chain
releases it, unlocked by a pilot tone on RS485. Nothing has run near 63 V or
100 A; no measured value is recorded here - invariant 10.

VREFBUF is deliberately **disabled**, VREF+ high-impedance: the AFE drives
the ADC reference (the mechanism behind invariant 9). Its source is U2, a
REF2033, driving `+3V3_ref` **and** `+1V65_bias` - one part sets reference
and mid-point, which is why they track. The 3.3 V lives in the calibration
record: a rig with a calibrated meter beats a datasheet tolerance.

**Devices, channels and parts all come from the bus, never from this file.**
`0x41` carries each device's one-line `description`; `coaxial.scan()` sweeps
unit ids; the `devices` tool lists and `op=use` picks. `origin.interface`
(`debug probe`, `RS485`, `simulated`) is a different question from which
unit. A bus is a serial segment - the simulated machine has five (`AX`,
`LL`/`RL`, `LA`/`RA`), unit id = position down the limb, node 2 is the knee
on both legs. `/node` lists, `/node RL 2` or `/node right knee` selects.
**`/node 0` is Modbus broadcast**: every node acts, none answers, reads are
refused, the prompt goes red.

`0x6D channels` reports every analog channel and digital pin with direction;
kind 4 is the parts list - name, role, place, **what powers it**, answered.
A pin table in a document or prompt is a second answer to "what is PB10":
add a pin to `board/src/board_io.c` and everything above it follows.
**Problem:** AFE_ON powers the BNO08X too; off, the part answers reads,
resets and advertises normally while acting on no write - every symptom
pointed at SPI and a day was spent there before the supply was checked.
**Fix:** `power` is a parts-list column, and `Board_ImuInit` refuses while
PB2 is low. Adding hardware is one row in `s_parts` (+ pins in `s_digital`,
+ a probe case so `state` is measured); nothing else, or it goes stale.
Check it landed:

```powershell
python -c "import coaxial; [print(p) for p in coaxial.connect([1])[0].system.channel_map()['parts']]"
board_chat -Ask "vad sitter på kortet?"    # the model, off the same wire
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

**`run_tests.ps1` is the interface to the suites**, not
`python tools/run_tests.py` (what it drives).

```powershell
.\host\run_tests.ps1                      # ~25 % of every check, the default
.\host\run_tests.ps1 -AutomaticMedium     # ~50 %, before handing work over
.\host\run_tests.ps1 -AutomaticHigh       # ~75 %, adds conformance + live:tools
.\host\run_tests.ps1 -All                 # 100 %, the gate
.\host\run_tests.ps1 -Depth 40            # any 5 % step
.\host\run_tests.ps1 -Scope test_mcp.py   # those files only, whatever the depth
.\host\run_tests.ps1 -Only intent,picker  # named tests, nothing else
.\host\run_tests.ps1 -Tags prompt,reply   # subjects, without asking the model
.\host\run_tests.ps1 -Structure           # does host/ still hold together - 3 s
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Check    # what is missing
                            # -Yes installs the lot; -FirmwarePackage X.zip
                            # adds FW_H7 to CubeMX.
. .\env.ps1                 # PATH + board_chat, dbg, board, cbuild, cflash, cubemx
```

**Refusals come from the board.** Anything taking parameters answers `u8
took` and, on refusal, the board's own words for what is wrong and what to
do. The host validates only what stops a request being formed. A new check
is a sentence beside it - not a host-mapped code, not a docstring list.

**`Coaxial63100` is the front door** (`host/coaxial/rig.py`): it owns the
AFE preflight (invariant 9) and puts the supply back as found, Ctrl+C
included. **The host is three interfaces** - `Acquisition`, `PolledSensor`,
`GateControl` - each with a real and a simulated implementation, so a name
drifting between them fails at construction. Add a method to both or
neither. `GateStage` is concrete: the arming policy, one of it.

```python
from coaxial import Coaxial63100
device = Coaxial63100(port='COM4')       # simulated_device=True: no cable
daq = device.daq                         # the data acquisition subsystem
daq.open()
daq.enable()                             # powers the analog front end
device.set_time_from_pc()                # the board counts cycles, not time
daq.configure('phaseU', 'NTC')           # names in any spelling, or a list
daq.start()                              # host and target both buffer
for r in daq.read(-1):                   # blocks for the first, takes the lot
    print(r.start_time, r.dt, [(s.name, s.value) for s in r.samples])
daq.stop()                               # buffering stops at target
daq.close()                              # the acquisition released
device.close()                           # the port, and the supply as found
```

Subsystems hang off it by name - `device.daq`, `.imu`, `.angle`, `.thermal`,
`.gates`, `.drive` - and `device.motion` is the drive as three verbs:
`stepper` (HOLD as a microstepper), `servo` (position over the A1335,
corrected between moves - a per-pass loop at link rate samples the
load-angle ring aliased and pumps it), `velocity` (sensorless under
`coaxial.loop`). Notebooks: `position_servo`, `position_and_sensorless`,
and the four `app_*` missions. `notebook_examples/daq_session.ipynb` is the flow, executed;
`notebook_examples/propeller_sweep.ipynb` is the 5230SL and its propeller from
rest to 6717 rpm and back, checked against Hobbywing's own thrust stand;
`speed_loop.ipynb` closes `coaxial.loop`'s chain over the model and
identifies it back out; `foc_montecarlo.ipynb` Monte Carlos the firmware's
own control law over the 23-63 V link sweep (`tools/montecarlo.py`, one
process per core) and puts a number on the sensorless floor;
`auto_tune.ipynb` is the bench-day procedure - commission, identify,
search a robust tune for exactly that machine, write the record, and the
drive verifies itself.

**`daq.catalogue()` is what the board can record**, each row saying its
kind and whether `configure()` may ask for it - since MINOR 7 the sensor
fields (orientation, acceleration, rotation rate, magnetic field, shaft
angle) ride any software-clocked record as four-word SNAPSHOTS beside the
sums: `configure('phaseU', 'shaft angle')`, and the frame scales them.
On older firmware the rows are listed and refused with the reason. A `Record` is a `dict` underneath, so `r['NTC']` is
still the SUM and `r['samples']` still the count; `r.value('NTC')` is one channel's mean and
`r.sample('NTC')` the struct behind it; `r.samples` is the ARRAY
and `r.count` the count. `daq.channel_names()` and `daq.columns(values)`
are the two helpers around it.

**`start()` puts a reader thread on the link** and it is the only thing
that touches the transport while it lives - a `print` in a loop never sits
between two round trips. Measured: 84.4 to 134.6 records/s with 4 ms of
work a block. Every read answers its own backlog, so pacing costs no round
trip.

```bash
cube-cmake --build --preset Debug        # must be zero warnings
STM32_Programmer_CLI -c port=SWD mode=UR -d build/Debug/coaxial_63100.elf -v --start

cd host
python -m coaxial all                    # CLI against the board
python tools/run_tests.py --offline      # the suites needing no board
python tools/pick_tests.py --explain     # which subjects, and why
python tools/build_and_flash.py          # build (+flash): --build-only, --flash-only
python tools/session.py --status         # who is sharing the board's port
python tools/switch.py --sweep 5,95 -p 10 -s 120  # background; --stop disarms
python tools/pulse.py -d 0.05 -H U -L V -n 1 --on 30   # one leg against another
python tools/commission.py --simulated   # the eight steps on the stand-in;
                                         # --arm --port COM4 at the bench
python -m coaxial_mcp --port COM4        # MCP server, stdio
python -m coaxial_ollama.capability      # which local model this machine runs
python dbg.py --repl                     # prompt loop; /py and /sh cost no tokens
python dbg.py -m auto -q "read the NTC"  # one question, the model that fits
```

Twenty-five suites, 2370 checks, sized from `host/tests/.counts.json` and so
measured rather than remembered: `test_structure.py` (545),
`test_ollama_tools.py` (218), `test_ollama_runner.py` (216),
`test_simulated.py` (201), `test_live_model.py` (212, needs ollama, `--live`),
`test_ollama_prompt.py` (113), `test_conformance.py` (110, `--conformance`),
`test_ollama_link.py` (96), `test_drive_core.py`
(70, the control law against a motor model through the host gcc, the Monte
Carlo's job included), `test_modbus_core.py` (68), `test_sensorless.py`
(79, the design arithmetic - the power stage's too - the commissioning
and the motion verbs, dangerous paths included, against the stand-in), `test_mcp.py` (46),
`test_shtp_core.py` (38), `test_filter_core.py` (42, the anti-alias
chain against the transfer function it was designed from), `test_ollama_render.py` (32), `test_parity.py` (30),
`test_ollama_board.py` (28), `test_ollama_bus.py` (28), `test_render.py`
(27, the 3D engine stage by stage against an analytic oracle -
`render/render_demo.ps1` is its bench), `test_ollama_reply.py` (23), `test_broker.py`
(33, the shared session and the reply shapes on a scripted port, no board), `test_views.py`
(24, every view and the front page drawn twice, no board), `test_ollama_language.py` (12),
`test_daq_api.py` (75, the acquisition front door against the
stand-in - naming, reading, the record shape, the buffers),
`test_bench.py` (4, the board's loop rates against a recorded baseline).
Wiring: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md#the-test-system). The
rules that bind you:

* **A missing cable is not a failing suite** - every suite opens through
  `open_session()`, which probes and falls back to the stand-in.
* **The model is loaded once per run and released once**, by `run_tests.py`.
  Measured: most of the wall time went into loading 7.6 GB again.
* **Run `-Structure` after editing anything under `host/`.** It catches a
  module that stopped importing, a definition split into two files, a dead
  re-export, a function past what a reader can hold. Measured: five
  NameErrors in one afternoon of moving code, each found by an unrelated
  test elsewhere.
* **A tier is a budget of checks, and it cuts as well as fills.** Floor:
  one test from every subject the pick left out, plus the pick's smallest
  group. Sizes from `host/tests/counts.py` - groups run 2 to 77 checks.

      ran 19 of 43 groups: prompt,runner, seed 3440, 51% of checks
      Total: 984  Passed: 449, Skipped: 535, Failed: 0, (4 of 6 suites ran)

  Why the clamp exists, measured: on the 25 % tier the model's pick put
  `live:all` back and the cheapest run took 398 s, 352 of them that suite.
  It now says what it refused: `the 25% tier does not stretch to: live:all`.
* **Any 5 % step is a tier.** Suites join by seconds per check - measured:
  simulated 0.003 s, ollama 0.019, core 0.03, parity 0.13, mcp 0.14,
  conformance 0.29, live 4.6. The `test_ollama_*` suites narrow themselves;
  766 of this tree's 2370 checks are in those nine files.
* **The model is not asked when the path map already knows.** Every changed
  file on an explicit rule with a `CHEAP` answer - structure, core, shtp,
  simulated, views, render; no board, no ollama - settles without a model.
  Asking costs a 7.6 GB load to be told what the map said. A demo wrapper
  edit is three seconds, not seven minutes.
* **Ctrl+C is `STOPPED`, exit 130, not `FAILED`** - the `finally` hands the
  model back. Killing from outside does not: measured, 8.4 GB stayed on the
  card until released by hand.
* **A typed sentence is classified before it is answered** (`intent.py`), on
  the turn's own client. Never a second `Ollama`: a client at a different
  `num_ctx` reloads 7.6 GB per question. [docs/MODELS.md](docs/MODELS.md).

At the prompt, `/board simulated | auto | rs485 | COM4` and `/model TAG |
auto` swap either mid-session for no tokens - so does prose: "byt till
debugproben" is an order the host carries out. `/model` hands VRAM back
first.

The ST toolchain is not on PATH - arm-gcc, cmake, ninja,
`STM32_Programmer_CLI` live under `%LOCALAPPDATA%\stm32cube\bundles\`
(`cube bundle install --yes NAME`, no ST account). `env.ps1` puts the newest
on PATH for one shell. No port is hardcoded: `--port` is a first guess,
`open_session()` probes.

## Do not run the suites to look busy

**While a bug is live, run the narrowest thing that could disprove the
current hypothesis - never the full suite.** `-All` is eight minutes and
answers a question nobody asked. **Problem, measured:** chasing two gate
driver stages 15 C hotter than the third, the full suite was started three
times; none of the 1970 checks could say anything - the difference was on
the bench. **What worked:** a 600-sample pin count and a register dump. The
narrow thing: read the register, count the samples, or run the one suite
whose name matches the change.

## Green before the next thing

**Fix and get the suites passing before the next item on a list** - a
failing check carried forward stops being information. Pre-existing failures
too: say they are pre-existing, then fix them. **Problem, measured:** nine
failures in `test_ollama_render`/`test_ollama_reply` were labelled
pre-existing and carried through four more items; the change that broke them
was no longer identifiable. **Fix:** the label is a note on the way to the
fix, not a substitute.

## After a change lands

Once a change is made, tested and verified, ask - every time, as the last
step:

> **Continue, or commit and push?**
> *Continue* — keep working in this session
> *Commit and push* — stage, commit, push to origin/main

Two options, nothing else. **Known failure mode:** the question gets
replaced by a summary and the session carries on - it happened on the first
change after this rule was written.

## Spend the local model, not the expensive one

A local model on this machine has the board's tools wired to it, free per
token, at the bench. Anything routine, mechanical or covered by its tools
goes there by default.

```powershell
board_chat -Ask "read the NTC and give me the temperature"
python dbg.py -m auto -q "..."          # from host/, one layer down
```

Both pick the tag this machine runs and **pull it if absent** - "not
installed" is never a reason to answer from memory. `board_chat` also tunes
the ollama daemon (docs/MODELS.md). **Reuse a loaded model** - check
`ollama ps` first: two models is two copies of weights on a 16 GB card.

| Question | Who answers |
|---|---|
| What does the board read now? Is the AFE on? Temperature, DC link, frame counters? | **the local model** — offer the command, then stop |
| Is this channel odd? What does `self_test` say? | **the local model**, then read FINDINGS before investigating |
| Does it still build/flash/pass? | **the local model** — `dbg -q "run the test suites, then build and flash, tell me if anything failed"`; the tools report parsed tallies, not summaries |
| Why is this C function written this way? `board/` or `comms/`? Is this a protocol MAJOR? | **you** — measured failure mode: on design questions it substitutes plausible hardware constants (FINDINGS) |
| What is the wire format of command 0x41? | **you**, from docs/PROTOCOL.md |

A failing build or a regressed test is still yours to judge - the rule is
about who *runs* the loop. And when the answer is for you, not the user,
skip the model: `run_tests.py` and `build_and_flash.py` print a parsed tally
and a real exit code in four lines.

| The answer is for | Do |
|---|---|
| the user, who asked | the local model — free, at the bench |
| you, mid-change | run the script yourself |
| nobody yet (exploring) | neither — read FINDINGS first |

### Stop and ask first

Before touching the board to answer a question, ask whether it is worth
tokens. Two shapes: *measure something*, and *reach the local model at all*
("how do I ask it" means they want to be at the prompt, not taught the
command). Ask minimally:

> **Local model, or here?**
> *Local model* — board_chat
> *Here* — I drive the library

On *Local model*, hand over the shortest way and **stop** - a click, not a
retyped command:

    Terminal panel > the v beside + > Board chat

(**Ctrl+Shift+B** runs "Ask the board" for one question; only outside VS
Code is `board_chat -Ask "..."` the answer.) Do not run it, paraphrase it,
or take the reading anyway. **Do not spawn a window** - `Start-Process`
puts the answer where the user is not.

### When the permission classifier says no

Auto mode's classifier can refuse a Bash call that arms the stage - and then
everything Bash for a while, `run_tests.ps1` included; reads still went
through the same afternoon. The hand-off, three git-ignored files under
`host/`:

* `claude_watch.ps1` - the user starts it once; forks hidden (PID in
  `claude_watch.pid`), hashes `claude_do_it.ps1` every 0.5 s, runs it after
  a 1 s settle. `-Stop` kills, `-Status` asks.
* `claude_do_it.ps1` - what you want run, as `Step 'what' 'look for' {...}`
  blocks FROM `host/`; stops at the first non-zero exit. Rewrite, wait for
  `WATCH finished`, read the log.
* `claude_do_it.log` - UTF-8, timestamped: WATCH/RUN/STEP, output, exit.

**The watcher runs whatever the file holds** - comment a physical step out
the moment its run is read, or the next rewrite pulses the stage unasked.

### Suspect your own code before the hardware

**No oscilloscope, schematic question or pin assignment until the code has
been read for the fault.** The BNO08X bring-up produced six firmware defects
and four hardware hypotheses; none of the latter survived a measurement.

| Symptom | Actual cause |
|---|---|
| chip select never moved | configured before `HAL_SPI_DeInit`, which runs the MSP and hands the pin back |
| every read came back `FF FF FF FF` | CS released between header and cargo; the part restarted the message |
| every read after a reset refused | the advertisement is 276 bytes, the buffer was 64 |
| a sensor enabled at 60 ms never reported | the interval went out little-endian on a big-endian wire - 27 minutes |
| a write worked twice, failed the third | gated on an INTN an already-awake part never asserts |

Before "which pin is X" or "looks like hardware", all of:

* **Read the reference implementation** - `github.com/ceva-dsp/sh2` settled
  the report lengths; `bno080-nucleo-demo`'s `sh2_hal_spi.c` settled CS and
  wake ordering, both after hours of guessing.
* **Re-read the init order** - MSP callbacks reconfigure pins; anything set
  before `HAL_*_Init` is gone.
* **Check widths and byte order against the wire**, not today's traffic.
* **Check the worst-case buffer**, not the typical one.
* **Verify the fix took effect** - two of the four hypotheses were
  "improvements" overwritten before they ran, credited with another cause's
  improvement.

A measurement taken while something else drives the same bench is not a
measurement (FINDINGS) - code under test included. None of this applies when
the board is instrumentation for work already agreed: then it is a test
fixture. The test is who the answer is for. Where the user chose *here*,
relay what was measured - no re-derivation, no verdict. It is a **dumb-slave
interface to a dumb slave**; invariant 10 applies to it too.

## How to write here

`~/.claude/CLAUDE.md` says it; this file does not repeat it. Specific here:
**keep every measurement, rejected alternative and recorded failure** when
trimming - cut the paragraph, never the number. FINDINGS is a record, not
documentation, and is not shortened.

## Layout

```
core/        CubeMX-generated. main.c holds ONLY CubeMX functions, main(),
             the two poll calls the sensors need, and the STO keepalive
             toggle. Keep it that way.
electronics/ schematic and BOM - the authority on what is fitted
render/      the CAD export the attitude view draws from
board/       this hardware, behind comms/inc/board.h
comms/       the comms stack: cmd over proto over dev, plus the console
modbus/      the protocol. Portable C11, no HAL in crc/slave/rtu.
drive/       the control law. Portable C11, host-tested against a motor model
shtp/ thermal/ filter/  the other portable cores: the BNO08X transport, the
             ten-node observer, the anti-alias chain - each host-tested
host/        Python: coaxial/ library, coaxial_mcp/ server, coaxial_ollama/
             runner and dbg.py, testline/, tests, tools
notebook_examples/  executed notebooks, checked in with the stand-in's
             outputs - root README.md tables them
electronic_simulations/  LTSpice, a git submodule (SSH key on the bench
             machine); coaxial/inverter.py carries its traced constants
coaxial_tty.ps1  the chooser: session, seven views (the gate drivers and the
                 rotor observer under MOTOR CONTROLLER), the board chat
terminal/        imu.ps1 attitude, angle.ps1 shaft angle, adc.ps1 meter bridge,
                 gate_drivers.ps1, thermal_observer.ps1, rotor_observer.ps1
setup.ps1        one-time environment setup; -Check changes nothing
env.ps1          per-shell PATH and the board_chat/dbg/board/cbuild/cflash aliases
host/board_chat.ps1  preflight + prompt loop; orchestration only — also the
                 chooser's BOARD CHAT page, which asks who answers: this,
                 or claude with the coaxial MCP server. board_chat/ beside
                 it holds Say, ComPort, Ollama, ModelChoice, Relaunch: one
                 concern per file, dot-sourced, not meant to run alone
docs/            this documentation
```

`cmake/stm32cubemx/CMakeLists.txt` is regenerated by CubeMX — **never add
sources there**. New sources go in the root `CMakeLists.txt` user blocks.
Regeneration also writes ST's `Core/`/`Drivers/` case back into it; Windows
resolves that against the lowercase tree, so the bench still builds —
re-lowercase the paths when the file is next touched. `.ioc` and
`.mxproject` are CubeMX's own bookkeeping: leave their case alone.

## Invariants

Break one and something works until it doesn't.

1. **The protocol core stays hardware-free.** `modbus_crc.c`,
   `modbus_slave.c`, `modbus_rtu.c` include only `<stdint.h>`, `<stddef.h>`,
   `<stdbool.h>`, `<string.h>` - host-testable, and `test_modbus_core.py`
   does it: built with host gcc, driven through ctypes, clock injected.
   `-Wconversion` is on them in both builds. Only `comms/src/dev_uart.c`
   touches a USART.
2. **RTU timing is raw `DWT->CYCCNT` ticks, never microseconds** - dividing
   moves the wrap off a power of two and the unsigned elapsed arithmetic
   breaks silently across it.
3. **Command 0x41's payload is append-only.** Appending a field is a MINOR;
   moving, resizing or repurposing one is a MAJOR whether you meant it or
   not.
4. **A host selects its codec on the protocol MAJOR alone**, never the
   firmware version.
5. **No printf while the binary link is open** - a blocking transmit inside
   a frame corrupts framing and latches a UART overrun, which on this
   silicon kills reception permanently.
6. **Every ADC read path calls `HAL_ADC_ConfigChannel` and clears `PCSEL`.**
   Two separate bugs came from paths that did not - FINDINGS.
7. **A conversion is named where it is defined, and defined once** - every
   scaling parameter lives in the calibration record behind `0x6E` device 3,
   never a literal at a call site, never a second copy in a host. The phase
   gain was traced off the schematic 2026-08-26, so they report amperes.
   **One number has been measured against an instrument**: the DC link,
   spanned against a DMM 2026-08-30 (31.04 read, 30.05 true, -32 418 ppm on
   channel 5, saved). Every other number is the schematic's arithmetic -
   span before believing one.
8. **Nothing in the Python library returns a status code or
   None-for-failure** - a result, or a raise from `coaxial.errors`.
9. **AFE_ON decides what a reading means** - it powers the ADC reference.
   Off, channels read exact mid-scale and the NTC exactly 25.00 °C:
   plausible, not a measurement. The gate is a **label, not a refusal**:
   `analog_read` returns codes either way under an unmistakable line.
   Refusing was tried and was worse - asked for raw codes with the AFE
   deliberately off, a model wrote "Mid-scale … 25.00 C" out of the warning
   text itself. Cooked readings (`read_all`, `ntc_temperature`,
   `dcbus_voltage`) still refuse: they claim a physical quantity.
10. **The board is a dumb slave: no limits, no expected values, in firmware
    or this repository's tests.** Pass/fail belongs to a test executive
    beside calibrated instruments. Two narrow exceptions: `self_test`
    (judges only its own registers and flash), and **the thermal envelope**
    (a board that cooks itself is not a measurement problem) - the board
    never calls a reading good, it *acts*: at a ceiling it drops MOE, the
    same path the break uses, and the ceilings live in the calibration
    record, a limit it was given, not invented. The margin is reported; the
    verdict is not.
11. **The DC link divider's headroom is deliberate.** 49.9k/2.2k gives
    78.15 V full scale on a 63 V rating, 24 % margin - the over-rating
    transient is what you want recorded, not clipped.

## Two things that will waste your time

**JTAG connect-under-reset does not work here.** Any connect asserting NRST
fails with `Unable to get core ID` - probe firmware, not the board; cabling
proven fine. Use `-c port=JTAG mode=Normal reset=SWrst`, or SWD. End with
`--start`, not `-hardRst`, or the core is left halted.

**The AFE switch (PB2) powers the ADC reference and the IMU.** Off: every
channel exact mid-scale, NTC exactly 25.00 °C. The BNO08X is worse - answers
reads, resets and advertises normally while acting on no write, so the fault
presents as SPI; a day was spent there before the supply was checked. Enable
it before believing anything analog or an IMU that looks present.

## Tooling traps

- C escape sequences through a Python string inside a bash heredoc get
  mangled: `\r\n` arrives as a real CR+LF. Build the backslash with
  `chr(92)`, or write the code to a file and splice it.
- `core/src/main.c` is LF-terminated; Python `open(...)` without
  `newline=''` converts it to CRLF on write.
- Long `cat > file <<'EOF'` heredocs get truncated. Split them.
- Most files here are CRLF. A multi-line `str.replace` pattern written in a
  heredoc has LF newlines and silently matches nothing - five no-op edits in
  one afternoon, each discovered a test later. Single-line replaces are
  safe; multi-line edits go through the edit tool, and a replace script must
  assert its patterns matched before writing.
- PowerShell variable names are case-insensitive: `$Asked` and `$asked` are
  one variable. A list named `$Asked` beside the `$asked` view overwrote it
  and the chooser opened BOARD CHAT on every start (2026-08-31).
