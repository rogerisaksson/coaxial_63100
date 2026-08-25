# The model in the loop

Which tag runs, how much sits on the GPU, what it may not conclude, and which
failures are already measured.

Every number here was measured on **one reference bench** — a 16 GB card, a
32-core desktop CPU, 64 GB RAM, Q4_K_M weights, `num_ctx` 8192 — and is
recorded so nobody re-derives it. What generalises is the **ratio and the
conclusion**, not the absolute figure: a different card moves every number in
this document and none of the decisions.

## Why local

A transcript of a board under test is measurement data: register dumps, pin
names, unreleased hardware. `client.py` refuses a non-loopback host or a
`:cloud` tag unless `--allow-remote` says otherwise, and `require_model` drops
cloud tags so a bare stem cannot resolve onto one.

## Which tag

**The largest tools-capable model that fits the card whole**, keeping enough
VRAM back for the desktop. `capability.py` measures and picks:

```powershell
python -m coaxial_ollama.capability            # what this machine gets
dbg -m auto "what is the board temperature?"   # same choice, from the prompt
```

| Measured | Used for | How |
|---|---|---|
| cores, threads | whether a CPU-bound choice is slow or hopeless | `Win32_Processor.NumberOfCores` |
| CPU busy % | a warning, never the tag | `Win32_Processor.LoadPercentage` |
| RAM installed **and free** | which models can be held at all | `GlobalMemoryStatusEx` |
| VRAM total | the budget | `nvidia-smi`, else registry `qwMemorySize` |
| VRAM **used by anything else** | the reserve | `nvidia-smi memory.used`, minus ollama's own |

Two of those are the traps. **Free RAM, not installed**: 64 GB with 8 free
cannot hold a 42 GB model, and it swaps rather than erroring. **VRAM in use
minus ollama's own**: the probe usually runs with the last model resident, so
counting our own weights as the desktop's made the reserve grow by the size of
whatever was loaded, and the picker then chose something smaller — which became
the next baseline. A ratchet, measured before it was fixed.

CPU load is deliberately not an input: a snapshot says nothing about the next
ten minutes, and the tag is chosen for the session. Only tools-capable tags
are candidates. `-Model TAG` overrules everywhere.

Two ways in from the editor, both checked in: the **Board prompt** terminal
profile (`.vscode/settings.json`), and **Ctrl+Shift+B** for one question
(`.vscode/tasks.json`).

### Layers on the GPU

`num_gpu` is an ordinary `options` entry on `/api/chat` — **no Modelfile**,
which would be a second tag to keep in step. Measured on a 12B model at 48
layers:

| `num_gpu` | VRAM | tok/s |
|---|---|---|
| default (all 48) | 7.8 GB | **64.3** |
| 24 — half | 4.3 GB | 12.7 |
| 0 — CPU only | 0 GB | 6.7 |

**Five times the speed to hand back half the VRAM**, and unnecessary whenever
the model fits whole. Hybrid is what happens when nothing fits, not a target;
`--prefer capability` opts into it deliberately.

### The desktop's reserve

VRAM the picker does not spend: the largest of 2 GB, a quarter of the card, or
**what the card already holds plus 2 GB to grow into**. The third rule is the
one that earns its place — on the reference bench the desktop alone held
2.6 GB of 16 with nothing of ours running, so a flat quarter would have left
it 1.4 GB for a second 4K surface. Too little shows as a hang while the driver
evicts, not an error. Counting what is already there gave 4.6 GB instead.

Raising the reserve steps the tag down a size, which costs less than it looks:
a 14B and a 12B both landed within half a degree of the board's NTC.
`board_prompt -Reserve N` for one run, `COAXIAL_VRAM_RESERVE_GB` for a
machine.

### Priority and threads

`board_prompt.ps1` drops ollama to BelowNormal unless `-Normal`, and starts
the daemon with `OLLAMA_MAX_LOADED_MODELS=1`, `OLLAMA_NUM_PARALLEL=1`.
Priority is **CPU** scheduling — with the model wholly on the card it only
deprioritises tokenisation, sampling and serial I/O. The single-model limits
are memory: two contexts is how one card is asked for two copies of the
weights, measured as a 500 reading `cudaMalloc failed`.

Raising `num_thread` bought nothing, CPU-only, on a 64-thread machine:

| threads | default | 16 | 32 | 64 |
|---|---|---|---|---|
| tok/s | 6.4 | 6.4 | 6.3 | 5.7 |

Decode is bandwidth-bound, not core-bound, and filling both SMT siblings of
every core makes it worse. Nothing sets `num_thread`; the daemon's own choice
beat everything tried against it.

## The prompt line

```
«🤖💤»Coaxial 63100(JTAG and COM4, node 1)>
```

The bookend group is the only thing that moves: the icon turns while the model
works, green idle / yellow working / red failed, and the tag names the link and
the node. Two `>` prompts in one docked panel are otherwise identical.

An earlier version span the "1" in the name itself. It wrote `Coaxial 63-00`
and `Coaxial 63\00` into the operator's transcript, twice, and read as a
corrupted board name. A name is not a widget — `spinner.py`.

The session opens with one line in the machine's language, printed by the host:

```
Jag är gemma4:12b och är experten i det här projektet. Skriv /help.
```

Asking a model to write its own greeting costs a load, a turn and the chance
of getting it wrong. Everything a session used to print on the way in — role,
tool list, detail level, per-turn cost — is `/help`, built live.

An answer that hits `--words` says so. Measured: a seven-channel table stopped
mid-row at 180 generated tokens, which reads as complete to everyone except a
reader counting rows. `done_reason` marks it *[cut off at --words 180]*.

## Two passes: classify, then answer

A typed sentence is classified before it is answered. `intent.py` asks for
`{intent, kind}` against seven named intents, and the turn gains one line —
*"the operator is asking for the present value of channels or pins — answered
by analog_read"*.

One turn used to do both jobs, and the failures were the first showing up in
the second: *"ge mig en lista över de analoga värdena"* carries the word for a
map and the word for a read, and a single pass took the verb. Measured on
gemma4:12b: **11 of 12**, ~2.75 s each. The twelfth is `byt till debugproben`,
which `board_switch()` carries out for no model tokens before anything is
compiled.

**The noun decides, never the verb**, in the classifier's prompt and in
`SYSTEM` both: channels/pins/inputs is the map; values/readings/measurements
is a read. Without that line, *"lista över alla analoga mätvärdena"*
classified as `map` and put the channel table on screen for the second
question running.

It goes through the turn's **own** client, with only the schema, `think=False`
and the token budget overridden. A second `Ollama` was tried first and was
wrong: ollama keys a loaded runner on `num_ctx`, so a second client asking for
the same tag at a different window unloads and reloads 7.6 GB **once per
question**. `--no-compile` runs the one-call behaviour.

Every failure — no ollama, not JSON, an intent this repo has no name for —
adds no hint and leaves the turn exactly as it was.

## The language of the answer

**Decided by the host, not the model.** Told to "answer in the language the
question was asked in", it must work out the language *and* answer, and the
first is where it drifts — `qwen2.5:14b` answered European questions in
Chinese, Japanese and Thai.

A session starts in the machine's language (`language.system_language()`, the
Windows locale). A question in another language moves it; so does asking for
one. `--lang NAME` per run, `/lang` mid-session, `/lang auto` back to
detection. Units and channel names stay as the board prints them — `NTC`,
`DCbus`, `V`, `C` — because a translated channel name is one nobody can grep.

`language.py` decides in two stages: script ranges settle Chinese, Japanese,
Korean, Thai, Greek, Cyrillic, Hebrew, Arabic; a stop-word count separates the
Latin ones. It **abstains when the winner is not strictly ahead** — Danish and
Norwegian score alike, and telling a Dane to answer in Norwegian is worse than
saying nothing.

Every locale it can name has a greeting; a test fails if a recognised locale
has none. A console that cannot encode the one it is owed gets English — a
bare `python dbg.py` on cp1252 renders Japanese as question marks.

### One screen, one language

The screen carries more than the answer: the AFE-off warning,
`link_diagnose`'s checklist, "link is down, not answered", a refused channel
name. A Swedish question answered in Swedish under an English warning is one
screen in two languages.

`language.PHRASES` keys those sentences by the English at the call site;
`localise()` turns them on the way to the screen. What the model reads, what
the transcript keeps and what MCP serves stay English: one canonical text for
the reader that is not a person. Whole sentences only, so anything the board
said passes through untouched.

Swedish only, deliberately — a translation nobody here can check is
worse than none. A test fails if any key stops matching its call site.

`Chat.language` holds for the session. Rebuilding it every turn flipped the
instruction back and forth on a one-word follow-up like "tabellera", which
detects as nothing — a real prefix change, so a KV cache miss every time. Two
things move it: `detect()` disagreeing, or the question naming a language.

| Naming a language | Example | Why it is not a mention |
|---|---|---|
| a verb that asks for one, beside the name | "förklara på japanska", "byt språk till svenska" | "the German firmware bug" has no such verb |
| the name, in a message `detect()` places nowhere | "svenska tack" | "varför är dokumentationen på engelska?" is Swedish on `är`/`på`, left to the verb rule |

Both were measured as misses. With only "answer" verbs, "förklara på japanska"
went out under *Answer in Swedish and in no other language*. With no second
rule, a session locked to Korean answered "byt språk till svenska" — which
scores no stop word anywhere — with a refusal, in Korean.

A message asking for *nothing but* the switch (`bare_switch()`) is answered by
the host with one word from `language.OKAY` and never reaches the model. It
used to cost a turn answering "Jag har ändrat språket till svenska. Hur kan
jag hjälpa dig…" under a host line reading `språk: bytt till Swedish (låst)` —
the same fact three times, in two languages, one of them a lock nobody asked
to be told about. **The answer being in the new language is the
acknowledgement.** A request with a question attached is still the model's
turn; the leftover-word test is the line between them.

Tests build a `Chat` with no session language: a suite reading the Windows
locale passes on one machine and fails on the next. The locale is read in
`build()`, at the entry point.

A Windows console encodes with its codepage, cp1252 here. A Polish `ł`, an ohm
sign or anything Cyrillic does not fit, and the default handler raises
`UnicodeEncodeError` *after* the measurement. `dbg.py` sets `errors='replace'`
and leaves the codepage alone — forcing UTF-8 would hand a legacy console
mojibake for everything it could have shown. A redirected stream does get
UTF-8: a file has no codepage to mismatch.

## Keeping the model loaded

`keep_alive` goes on **every** request. Ollama caches the KV state of a prefix
it has processed — that is what makes turn nine as quick as turn two — and
throws it away when the model unloads.

| | hold | why |
|---|---|---|
| prompt loop (`--repl`) | 30 min | the next turn reuses the prefix |
| one question (`-Ask`, `-q`) | 2 min | enough for a follow-up, then the card is somebody else's |
| **leaving the prompt** | **released at once** | the cache has no further job |
| **entering the prompt** | **anything else unloaded first** | a card with two models is how a load fails |

Measured before that last row existed: a finished session held **9.69 GB of a
16 GB card for 27 minutes at 1 % utilisation**. Unloading on the way out
returns the card to 2.1 GB; a reload costs about seven seconds. `-Hold` keeps
it, `--keep-alive 0` hands it back on any path.

Entry matters as much. A killed window or somebody's `ollama run` leaves
weights resident until keep_alive expires, and the next load then asks a full
card — measured as a 500 reading `cudaMalloc failed`. `board_prompt.ps1`
sweeps first and says what it freed:

```
  ok    unloaded    llama3.1:8b was still resident, 4.9 GB freed
  ok    model       gemma4:12b  loaded in 7.0 s, held 30m
```

A model already the one this run wants is kept, not reloaded — `loaded in
0.0 s`. `-KeepOthers` leaves everything alone.

`preload()` sends `options` with an empty message list. Without `num_ctx` the
daemon loads the model's own default — 128k on llama3.1, a 7 GB KV buffer, a
500. Worse when it succeeds: resident at one context size, questioned at
another, so it reloads and the preload bought a wait instead of saving one.

**The same rule inside the tests.** `tools/run_tests.py` loads the tag once
before the first suite that needs it, holds it across every row of every such
suite, and releases in `finally`. Releasing between runs put most of the wall
time into loading 7.6 GB again.

### When the runner dies

Measured repeatedly: llama-server terminates with `std::bad_alloc`, usually
while saving its prompt cache, and ollama answers 500 `model runner has
unexpectedly stopped`. `client._chat_once` retries `RUNNER_RETRIES` (2) times
with a growing wait and says nothing — a retry that worked is not news. A
request ollama *refused* is never retried.

The cause is llama-server's own memory, not this loop's prompt: it saves a
~340 MiB slot state whenever a prefix diverges early from the cached one —
every question here — and keeps up to 32 checkpoints of 320.013 MiB each,
sized by the context window, not the prompt. Evidence in
[FINDINGS.md](FINDINGS.md).

```
LLAMA_ARG_CACHE_RAM       = 0   # no prompt cache: nothing here re-asks an old prompt
LLAMA_ARG_CTX_CHECKPOINTS = 0   # not 32, and not 2 either
```

Capping at 2 was not enough: restoring a 311.575 MiB checkpoint threw
`std::bad_alloc` on its own. Off entirely costs nothing, because `debug.Chat`
clears history after every answered turn. Measured off: ten questions, thirty
model calls, zero `std::bad_alloc`, one model load. `board_prompt.ps1` sets
them and restarts the daemon once if it was already running — an existing
daemon keeps the environment it started with. `-NoTune` opts out.

### When there really is no room

`_out_of_memory` tells a full card from a crashed runner — a crashed runner
comes back, a full card stays full — and `_make_room` climbs one rung at a
time, ordered by what being wrong costs:

1. unload every *other* resident model, then this one and its caches, retry;
2. halve `num_ctx` down to `MIN_NUM_CTX` (2048), for the rest of the session;
3. give up, naming `--num-ctx`, `--num-gpu` and a smaller tag.

Each rung goes into `client.notes` rather than the terminal — a library that
writes to somebody's terminal is one nobody can embed — and `Chat` drains them
into the same trace the tool results use.

---

## What the model may not conclude

The board's invariants, restated as limits on the model, and in `SYSTEM` for
that reason:

- **The AFE switch powers the ADC reference.** Off, every channel reads exact
  mid-scale and the NTC reads exactly 25.00 °C: plausible, and not a
  measurement. [HARDWARE.md](HARDWARE.md).
- **Phase channels sit behind unknown gain.** Volts at the ADC pin, never a
  sensed current or phase voltage.
- **No verdict.** It reports a number through `report`, against a schema the
  daemon enforces, and `plan.Limit` decides pass or fail in Python.

## Structured output: tools, not JSON mode

`format='json'` constrains the *content* field — the one part this loop does
not parse. Every number reaching a verdict arrives as a `report` argument. A
model told to answer in JSON tends to describe a tool call in its content
rather than making one, so json mode competes with the tool path.

Available for callers outside the runner — `Ollama(fmt='json')`, `dbg.py
--format json`, usually with `-t none` — and off everywhere else. The one
internal use is the intent pass, which constrains an enum rather than prose.

## Arguments arrive in whatever shape the model felt like

The schema says array of strings; a smaller model sends `ch="ntc"`, the string
`"['NTC']"`, or `samples="100"`. Unhandled, `for item in "ntc"` iterates
characters and the tool answers `unknown channel 'n'` — which tells the model
nothing it can act on, and what it does next is answer from memory.

`coaxial_mcp.tools.coerce` converts every argument to the type the tool's own
`inputSchema` declares, and refuses what will not convert **by field name and
wanted type**. Ollama path only: MCP gets the same protection from the
protocol library.

Names get four rules, in `_alias`:

* punctuation is stripped — `dc_bus`, `dc-bus`, `DC bus`, `dcbus` are one key;
* A/B/C and U/V/W alias each other, but only onto channels the board has;
* a word that can only mean one channel resolves to it (`bus` is inside
  `dcbus` and nothing else, `temp` is a `SIGNAL_ALIASES` entry for `ntc`). One
  that could mean several says *channel 'phas' could be phaseu or phasev or
  phasew — say which*, rather than "unknown", which reads as "no such thing";
* a name built out of words is read as its words: `BUS_VOLT`, `NTC_TEMP`,
  `ADC_CH3`, `PhaseAVolt`. Separators and camelCase both split. A single
  letter counts as a phase only beside the word that says so — without that,
  `not_a_channel` resolved to PhaseU through its bare `a`. `A0` stays refused:
  it is a pin name, and guessing which channel hangs off it invents one.

Measured: `['ntc','dc_bus','phase_a','phase_b','phase_c']` lost all five
readings to the two spelled the other way; `['bus']` was refused with `dcbus`
listed in its own refusal; `BUS_VOLT` was invented outright.

`BUS_VOLT` had a second cause, here rather than in the model. Terse mode
dropped `analog_read`'s *omit for all* — the only line saying how to ask for
every channel — so the model started naming them itself and read five of
seven. `detail._is_schema` now keeps a property description carrying
enumerated values, a spelling, **or how to leave the field out**.

## Reading these documents from a prompt

The `docs` tool reaches this file and its neighbours — **off by default**:

```powershell
dbg -t docs "what does FINDINGS say about PCSEL?"     # or /tools docs
```

```
docs()                           the index: every document and its headings
docs(doc='FINDINGS')             headings of one document
docs(doc='MODELS', section='Threads')
docs(find='25.00')               where a phrase appears, with its heading
```

It is out of `read`, `code`, `pins` and `build` for a stronger reason than
cost. Asked to *measure* the channels, `gemma4:12b` called `docs`, pulled
thousands of tokens of HARDWARE.md into context and answered with that
document's channel table — no measurement in it. Removing the tool cut the
question from 6229 prompt tokens to 2645 and turned the answer back into a
reading.

### Terse and full

Every description is re-sent every turn, and the readers are not alike: Claude
over MCP has hundreds of thousands of tokens, `gemma4:12b` has 8192 shared
with the conversation and the readings. `detail.py` picks the length from one
spec carrying both forms, **from the model, not a flag**:

| Reader | Level | Why |
|---|---|---|
| `gemma4:12b`, `qwen2.5:14b`, `llama3.1:8b` | terse | parameter count under `FULL_MODEL_B` (30 B) |
| a tag naming no size (`qwen3.6:latest`) | terse | the local daemon is where unnamed tags live |
| an ollama `:cloud` tag | full | somebody else's hardware |
| the MCP server, no model reading | full | the reader on that pipe is not the one paying |

`--detail terse|full|auto` per run, `/detail` mid-session, `COAXIAL_DETAIL`
per machine. Measured on this tool surface:

| Set | full | terse |
|---|---|---|
| `read` | 620 tok/turn | 439 |
| `code` (default) | 943 | 662 |
| `all` | 1435 | 1085 |

Terse also clips a `docs` section at 1200 characters instead of 4000, halves
the search hits, and drops subsection headings from the index — keeping
chapter names, because a chapter name is how the next call is spelled.

**Not** gated on this: the behavioural hints in `debug.py`. Each exists
because a small model needed telling, so trimming them for small models would
delete them exactly where they earn their place.

## Tools beyond the board

Beyond the board tools shared with the MCP server, three narrow ones, each
wrapping a fixed script rather than handing the model a command line:

| Tool | Wraps | Gated by |
|---|---|---|
| `build_firmware` | `tools/build_and_flash.py` — build, flash, or both | `--confirm` (always a write) |
| `run_tests` | `tools/run_tests.py` — the suites' own tally, never a paraphrase | nothing |
| `link_diagnose` | `tools/find_board.py` — an ordered checklist | nothing |

Each has a conditional `SYSTEM` line appended only when it is actually offered
(`BUILD_FIRMWARE_HINT`, `BUILD_HINT`, `LINK_DIAGNOSE_HINT`) — existing in the
schema is not enough, see the failures below.

`link_diagnose` stops at whichever step explains the silence:

1. Target power over SWD (`find_board.check_power()`) — the one check the
   serial side cannot make, and the most fundamental. Measured live, an
   unplugged ST-Link read `Voltage: 0.00V` where the serial side only ever
   said "silence".
2. COM ports Windows sees. 3. Whether the configured one is among them.
4. Whether the board answers right now. 5. `probe_other_ports` tries the rest.

`link_diagnose` (in-process) and `board_prompt/ComPort.ps1` (subprocess, since
PowerShell cannot import Python) call the same `find_board.py` — one
implementation of "does this port answer", not two that can drift.

## What a session leaves behind

`Chat.prompt_history` is every question typed, independent of `self.history`,
which the REPL clears after each answered turn. `/history` lists it,
`/clear_history` empties it, and `trim()` folds the last five into `SYSTEM` as
"steps already tried", so a multi-turn "why won't it connect" reads as one
investigation.

`IOLog` writes `host/prompt_io.tmp`, hidden and overwritten each session:
every question, every call (including the ones `_trace()` skips) and every
answer. Off on a bare `Chat()`.

---

## The live suite

`tests/test_live_model.py` is the only suite that does not script the model.
It crosses the two axes the model kept confusing — **list or read, analog or
digital** — and asserts which tool was called *and which was not*. An answer
that is right after the wrong call is not this suite passing.

`--sections tools|language|all`, `--no-compile` for the one-call behaviour,
`-m TAG` for another model, `--simulated` for the model half without a cable.
Deterministic: `temperature 0`, `seed 7`.

| Measured | Result |
|---|---|
| 2026-08-25, gemma4:12b, COM4 | **106 passed, 0 failed**, 408.6 s |
| `--sections tools`, with and without the intent pass | 98 / 98 both ways |
| before the *noun decides* rewrite | 6 of 24 failing |

Determinism is what makes it a gate. **A tool schema that crowded out a tool
choice**: adding `devices` took the tool list from ~1032 to ~1210 tokens a
turn, and `vad läser NTC:n?` stopped calling `analog_read` at all — it
answered from nothing, the same row twice. Trimming that one schema's property
descriptions, 157 tokens to 120 with no behaviour changed, brought back 82 of
82. A verbose `description` on a property nobody needed cost a tool call
somewhere else entirely, and nothing about the failure pointed at the cause.

Run against a second family to see what is the host's and what is the model's.
Measured 2026-08-25, `--simulated`:

| Tag | | |
|---|---|---|
| `gemma4:12b` | 94 passed, 0 failed | |
| `qwen2.5:14b` | 93 passed, 1 failed | `link_diagnose` on a question about what the board *is* |

**Two of qwen's three original failures were the host, not the model**, and
calling them a family difference was wrong. It answered *beskriv hårdvaran…*
with a description naming all seven channels — because describing them is the
question — and `is_retype` deleted it to an empty screen, which the suite
reported as the model failing to answer. The rule was "every channel named"
with nothing to tell a list from an explanation. Length tells them apart:
every measured retype has 3 to 12 words outside the table and that
description had 38, so `RESTATE_MAX_EXTRA` is 15. A markdown table is still
caught at any length.

An earlier row demanded **no board call at all** for that question, and both
families failed it by reaching for `board_info`. Two families agreeing is an
expectation that is wrong: describing this board from its own map beats
describing it from training. The row forbids a measurement now and allows the
map.

## Measured failure modes

> **Telling the model not to do something does not stop it. A fact the loop
> already holds, that the model gets no vote on, does.**

Every backstop in `debug.py` exists because a `SYSTEM` sentence was tried
first and did not hold.

| SYSTEM said | It did anyway | What settles it |
|---|---|---|
| never restate a tool's rows | retyped the whole table as prose, three sessions running (`qwen2.5:14b`) | `replies.is_retype` — an answer naming every channel just read is replaced with silence |
| never answer from an old reading | invented a full table one round trip after the ST-Link was unplugged, values a few counts off | `link_error` override, `self.last_channels` kept across turns |
| afe_power only when asked | turned the AFE back on to "serve" a reading, the turn after being told to turn it off | `Toolbox._permit` refuses it when the question never said "afe" |
| a call error is reported | answered "kortet har byggts och flashats" after declining that call at the `--confirm` prompt | `code_error` override |
| (schema) build_firmware exists | "Nej, jag programmerar inte firmwaren själv" — training beat the schema | `BUILD_FIRMWARE_HINT`, sent only when offered |
| (nothing said the swap exists) | "Jag kan inte byta till simulerad hårdvara" — true about itself, and a dead end | `debug.board_switch` swaps the board itself, no turn spent. One SYSTEM line naming `/board` stopped the refusal but did **not** buy the right answer — re-measured twice, the same question reached for `link_diagnose`, then `analog_read`, and wrote nothing |
| (schema) link_diagnose exists | called `build_firmware` when asked why the board was silent — a guess at a fix, not a diagnosis | `LINK_DIAGNOSE_HINT` |
| (schema) run_command's allowlist | tried `python3` and the wrong directory | `BUILD_HINT` |
| (nothing said which model it is) | answered the tag question from training | the turn's system message names the tag the daemon was asked for |

Two things that table is not. It is not an argument against writing the rule
down — every rule is still in `SYSTEM`, and the backstop only catches what is
left. And a backstop is not always available: `is_retype` can only fire
because the loop already knows which channels were read.

### llama3.1:8b answered from memory when a call failed

Faster — three questions in 24.0 s against 31.3 for a 12B, two model calls per
question against three, 1.2k prompt tokens against 2.9k. Then it asked for the
board temperature, called `analog_read` with `ch="ntc"`, got `unknown channel
'n'`, and said **"The board temperature is 25.00 C"** — three runs running, for
a board reading 36.3. The AFE-off number: invented, wrong by 11 °C, in the one
shape a reader is least likely to question. `coerce` came out of that.

After it, llama3.1:8b reads correctly and still loses: it passes
`ntc_beta=3950` where the part is B=3380 (`coaxial/scaling.py`), a silent
1.7 °C bias from a plausible constant the model supplied itself. The 12B
overrode nothing, landed within 0.5 °C, and when asked whether the AFE was on
answered by reasoning that the NTC was *not* reading exactly 25.00.

### A rule satisfied literally while broken

`SYSTEM` said "never a markdown table" and "no second list". The model complied
with both — comma-separated prose is neither — while doing exactly what the
rule existed to prevent. Reworded to name the act, not its shapes.

**The same line, read the other way.** *A table or list means analog_read once*
was written about tabulating readings; read plainly it says a **list** means
`analog_read`, so "ge mig en lista över alla analoga kanaler" fetched a full
table, in both languages, every time. Not disobedience — obedience to a
sentence that said the wrong thing. Three rewrites later it is **the noun
decides, never "list"**, after *A list of channels is board_info* still lost to
`lista … värdena` and `lista … mätvärdena`.

Building that backstop caught a bug in itself: the row regex keyed on a leading
digit and a word, which is also the shape of the header `64 smp @2000Hz` —
`smp` was briefly a channel. Anchoring on the mode column (`diff`/`SE`) fixed
it. Which is why the reproduction ran against a real render, not a string
shaped like one.

### The same call, asked again

Asked to tabulate every channel, a 14B turned the front end on, saw `on=1
pe15=0`, and turned it on again — three more times, identical call, each a full
round trip for a fact it already had.

`Chat.ask` remembers, per question, the (name, arguments) of every call and its
result; a call outside `REPEATABLE` that repeats exactly is answered from
memory. `analog_read`, `run_python` and `run_command` stay out — a second
reading is a new sample, and the DC bus does not hold still for a cache.

The trap: a repeat of a *failed* call must not read as a fresh success. The
dedup keeps the original result under the wrapping sentence and classifies the
link from that — otherwise `unchanged this turn, already asked: ERR
ConnectError: …` would not match the `ERR ` prefix the link-down override looks
for, and a cable pulled mid-turn would go quiet where it must not.

### A call written as text, more than one at a time

`dbg.py` recovers a tool call typed into `content` instead of `tool_calls` — a
JSON parse is cheaper than a wasted turn. The first version recovered exactly
one call, from a message that was *nothing but* that call. Asked "vad ar
temperaturen", the model sent two, and the prompt printed all four lines as the
answer — which at a bench reads as the board having stopped giving values.

`replies.salvage_calls` takes every call and matches braces rather than running
a non-greedy regex (`{.*?}` ends at the brace closing `arguments`, so a nested
argument parses as half of itself). The veto makes it safe: with the tags and
call objects removed, anything left must be marker noise — `tool`, `call`,
`function`, `check`, split at capitals so `CallCheckFunction` counts as three.
One word of real prose and the message is printed as the answer. Turning a
sentence that merely quotes JSON into a board command is far worse than showing
the JSON.

### The call header ate its own result

`_trace` printed the call above its result. Asked for five channels, `clip` cut
the arguments and landed a newline inside the header, gluing `->` and the
table's first row to a truncation notice. The header restated what the table
says better anyway, one channel per row. It is gone.

### Describe: a table, silence, then a cable

"Beskriv hårdvaran i detta projektet för en novis" → `analog_read {"ch":
["all"]}`, the table, and `A:` with nothing after it.

The silence was `_ask_inner`: its comment said *a blank answer is never valid
and needs no such gate*, and the code had `not answer` inside the
`not last_channels` gate — closed by the very call that had just succeeded. A
turn now nudges once for words and ends on a line rather than on nothing.

The table was `SYSTEM` saying when to call `analog_read` and never when not to.
One line added — *Describe, explain or compare means words* — and the
re-measurement had no tool call and an answer in words.

That answer also placed the serial link "över koaxialkabel", and the next
session did it again. Twice is not carelessness: `SYSTEM`'s own first line read
*an expert with a serial link to a coaxial BLDC inverter*, two words adjacent
with no relation stated, and the model fused them. The line now says what the
word means and what the link is — 16 tokens a turn against a false statement
about the product in every description it wrote. That moved the prompt from 184
to 200 tokens, past the `< runner/3` guard; the guard is now `< runner/2.5`.

`docs/HARDWARE.md` was wrong in the same direction — it said the PCB sits
behind the **rotor**. On an outrunner the rotor is the spinning can. A model
reading that document would have got it wrong from the document.

### A nudge stole the language lock

`trim()` read `asked` as the last `role=='user'` message — which is also the
shape of a mid-turn nudge. Measured: a Swedish question that triggered one had
its session language flip to English on the very next `trim()`. It now reads
`prompt_history[-1]`, appended exactly once per turn.

### The dead link, explained twice

With the board unplugged, every board question printed `link_diagnose`'s
checklist **twice** — once as the trace, once inside the answer — and paid the
ST-Link's fifteen-second timeout for both, because `_link_down_message` called
the tool again instead of reusing what the model had already called. The copies
were not even the same text: the trace cut each row at 96 characters, mid-word.

The answer now says only what the trace does not (`shown=`): one line, the
error class and its first clause. The trace wraps rather than cutting, and no
row may take more than `TRACE_LINES`. `-q` keeps the whole thing: with no trace
on screen, the checklist has nowhere else to be.

### The eager connect that ended a turn before it started

`main()` used to open `session.board` before the model was asked anything, so a
dead link failed loudly before tokens were spent. `link_diagnose` and the
`link_error` override do that job better: the model gets a real turn to help
instead of a one-shot question exiting with code 2 before it was ever asked.
