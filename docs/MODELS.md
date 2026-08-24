# The model in the loop

Read this before changing anything about the local model: which tag runs, how
much of it sits on the GPU, what it is allowed to conclude, and which failure
modes have already been measured on this bench.

Everything here was measured on one machine — RTX 4080 SUPER 16 GB, Threadripper
3970X (32 cores / 64 threads), 64 GB RAM, Q4_K_M weights, `num_ctx` 8192 — and
the numbers are recorded so nobody re-derives them. They are that machine's
numbers, not a specification.

---

## Why a local model at all

A transcript of a board under test is measurement data. It has register dumps,
pin names, and whatever a plan says about hardware that is not released. That
stays on the bench PC, which is why `client.py` refuses a non-loopback host or a
`:cloud` tag unless `--allow-remote` says otherwise, and why `require_model`
drops cloud tags from the candidate list so a bare stem cannot resolve onto one.

## Which tag, and who decides

Two ways to the prompt from inside the editor, both in the docked panel and
both checked in: the **Board prompt** terminal profile for a conversation
(`.vscode/settings.json`), **Ctrl+Shift+B** for one question
(`.vscode/tasks.json`). Nothing outside VS Code can type into a terminal that
is already open, which is why these exist.

`capability.py` measures the machine and picks. Ask it:

```powershell
python -m coaxial_ollama.capability                     # what this machine gets
python -m coaxial_ollama.capability --prefer capability
dbg -m auto "what is the board temperature?"            # same choice, from the prompt
```

The rule is **the largest tools-capable model that fits the graphics card
whole**, keeping enough VRAM back that the desktop still has somewhere to live.
What gets measured, every run:

| Measured | Used for | How |
|---|---|---|
| physical cores, threads | whether a CPU-bound choice is slow or hopeless | `Win32_Processor.NumberOfCores` |
| CPU busy % | a warning, never the tag | `Win32_Processor.LoadPercentage` |
| RAM installed **and free** | which models can be held at all | `GlobalMemoryStatusEx` |
| VRAM total | the budget | `nvidia-smi`, else the registry's `qwMemorySize` |
| VRAM **in use by anything else** | the reserve | `nvidia-smi memory.used`, minus what ollama itself holds |

Two of those matter. **Free RAM, not installed**: 64 GB with 8 free cannot
hold a 42 GB model, and the failure is swapping rather than an error anyone can
read. **VRAM in use minus ollama's own**: the probe usually runs with the last
model still resident, and counting our own 7.8 GB as the desktop's reserved
12.8 GB of a 16 GB card and picked something smaller — which became the next
baseline. A ratchet, measured before it was fixed.

CPU load is deliberately not an input: a snapshot says nothing about the next
ten minutes, and the tag is chosen for the session. A busy machine gets a
warning instead. Only tools-capable tags are candidates — a tag without them
describes a measurement instead of taking one.

`setup.ps1` and `board_prompt.ps1` both pull whatever the picker chooses;
`-Model TAG` overrules it everywhere. Anything driving this from outside,
Claude Code included (see [../CLAUDE.md](../CLAUDE.md)), should reach for
`board_prompt -Ask` or `dbg -m auto -q` rather than reason from memory.

### Layers on the GPU, and why hybrid is a fallback

`num_gpu` is an ordinary entry in `options` on a normal `/api/chat` call — **no
Modelfile needed**, which is better than one because a Modelfile is a second tag
to keep in step with the first. Measured with `gemma4:12b`, 48 layers:

| `num_gpu` | VRAM | tok/s |
|---|---|---|
| default (all 48) | 7.8 GB | **64.3** |
| 24 — half | 4.3 GB | 12.7 |
| 0 — CPU only | 0 GB | 6.7 |

A split model costs about **five times** the speed to hand back half the VRAM.
On a 16 GB card it is also unnecessary: the whole 12B fits in 7.8 GB and still
leaves 8 GB free. So hybrid is what happens when nothing fits, not a target.
`--prefer capability` opts into it deliberately and says what it costs.

### Leaving the desktop somewhere to live

The reserve is VRAM the picker does not spend: the largest of 2 GB, a quarter
of the card, or **what the card already holds plus 2 GB to grow into**. The
third matters on a workstation — measured with nothing of ours running, the
desktop alone was 2.6 GB of 16, so a flat quarter would leave it 1.4 GB for a
second 4K surface or a video starting. Too little shows up as a momentary hang
while the driver evicts, not as an error. Counting what is there gives 4.6 GB.

Still not much on a busy machine, so it is settable:

| | reserve | model | free with the desktop counted |
|---|---|---|---|
| default here | 4.6 GB | qwen2.5:14b, 9.7 GB | ~3.6 GB |
| `-Reserve 8` | 8.0 GB | gemma4:12b, 7.8 GB | ~5.6 GB |
| `-Reserve 11` | 11.0 GB | qwen2.5:7b, 4.7 GB | ~8.7 GB |

`board_prompt -Reserve N` for one run; `COAXIAL_VRAM_RESERVE_GB` in the
environment for a machine, once, honoured by every entry point. On a card of
this size the step from 14B to 12B costs little — both were within half a degree
of the board's own NTC reading, and 12B is the tag that checks the AFE before it
answers.

### Priority

`board_prompt.ps1` drops the ollama processes to BelowNormal unless `-Normal`
says otherwise, and starts the daemon with `OLLAMA_MAX_LOADED_MODELS=1` and
`OLLAMA_NUM_PARALLEL=1` when it starts it at all.

What each buys: the priority is **CPU** scheduling, so with the model wholly on
the card it deprioritises tokenisation, sampling and serial I/O — responsive
*around* a question, not during one. The single-model limits are about memory:
two contexts is how a 16 GB card is asked for two copies of the weights,
measured as a 500 reading `cudaMalloc failed`. The lever that actually reduces
GPU contention is the model's size, in the table above.

### Threads

Raising `num_thread` on 64 threads bought nothing and cost a little, CPU-only:

| threads | default | 16 | 32 | 64 |
|---|---|---|---|---|
| tok/s | 6.4 | 6.4 | 6.3 | 5.7 |

Decode is bandwidth-bound, not core-bound, and filling both SMT siblings of
every core makes it worse. Nothing in this repository sets `num_thread`; the
daemon's own choice beat everything tried against it.

### The prompt, and what stops an answer short

A session opens with one line, in the machine's own language:

```
Jag är gemma4:12b och är experten i det här projektet. Skriv /help.
```

`language.system_language()` reads the Windows locale, `language.greeting()`
picks the sentence, and anything without a translation gets English. Printed by
the host, not generated — asking a model to write its own greeting costs a load,
a turn and the chance of getting it wrong.

Everything a session used to print on the way in — the role, the tool list, the
detail level, the per-turn cost — is `/help`, which builds it live from the set
this session actually started with. Three lines nobody read twice is worse than
one line and a pointer.

Two more things a reader of a transcript will notice.

The prompt spins in the board's own name — `|`, `/`, `–`, `\` in one column —
**and keeps spinning while you type**, which says which of two `>` prompts in a
docked panel is waiting for a question. The first version stopped at the first
keypress, since a spinner that redraws its line repaints under what is being
typed; `spinner.py` repaints only its own cell and puts the cursor back (`ESC
7`, column, glyph, `ESC 8`), so the typed text is never written to. A line long
enough to wrap still defeats it, and a bench question is not that long.
Redirected output gets one static prompt, no escapes and no thread.

The bar is an en dash: a hyphen is a third the width of `|` at the size a
terminal draws them, and the spinner visibly limps. cp1252 has it at 0x96; a
console that cannot encode it is asked first and gets the ASCII set.

An answer that hits `--words` says so. Measured: a seven-channel table stopped
mid-row at 180 generated tokens, which reads as complete to everyone except a
reader counting rows. `done_reason` marks it *[cut off at --words 180]*.

### The language of the answer

A session starts in **the machine's language** - `language.system_language()`
reads the Windows locale, and the operator is answered in their own language
from the first word without a question having to prove it first. A question in
another language moves it, and so does asking for one; `--lang NAME` sets it
for a run and `/lang` changes it mid-session. Units and channel names stay as the board prints them
— `NTC`, `DCbus`, `V`, `C` — because those are what appears in `board_info`,
in the CSVs and in these documents, and a translated channel name is one
nobody can grep for.

Every locale this module can name has a greeting, Latin script or not:
Chinese, Japanese, Korean, Thai, Greek, Hebrew, Arabic and Russian alongside
the twelve European ones, and a test fails if a recognised locale has none. A
console that cannot encode the one it is owed gets English instead — a bare
`python dbg.py` on cp1252 renders Japanese as a row of question marks, and a
greeting nobody can read is worse than one in the wrong language.
`board_prompt.ps1` sets the console to UTF-8, so there the alphabet arrives.

**The language is decided here, not by the model.** Told to "answer in the
language the question was asked in", it has to work out the language *and*
answer, and the first is where it drifts — `qwen2.5:14b` answered European
questions in Chinese, Japanese and Thai.

`language.py` decides instead and the system message says it plainly: *The
question is in Swedish. Answer in Swedish and in no other language.* Two small
stages — script ranges settle Chinese, Japanese, Korean, Thai, Greek, Cyrillic,
Hebrew and Arabic; a stop-word count separates the Latin ones. It abstains
when the winner is not strictly ahead, because Danish and Norwegian score
alike and telling a Dane to answer in Norwegian is worse than saying nothing.

### One screen, one language

The answer follows the question, but the screen has more on it than the
answer: the AFE-off warning, `link_diagnose`'s checklist, "link is down, not
answered", a refused channel name. All of that is text this project wrote, and
a Swedish question answered in Swedish under an English warning is one screen
in two languages.

`language.PHRASES` holds those sentences keyed by the English they are written
as at the call site, and `localise()` turns them on the way to the screen -
`Chat._trace` for tool output, `Chat.ask` for the answer. What the model reads,
what the transcript keeps and what the MCP server serves stay English: one
canonical text for the reader that is not a person, another for the one that
is. Whole sentences only, so a channel name, a unit or anything the board said
passes through untouched.

Swedish only, deliberately. English is the fallback and the language of these
documents, and a translation nobody at this bench can check is worse than no
translation. Adding a language is adding a dict; a test fails if any key stops
matching its call site, so a translation cannot quietly go dead.

`Chat.language` holds it for the session rather than being rebuilt every
turn. That matters for more than tidiness: a one-word follow-up like
"tabellera" detects as nothing on its own, and rebuilding from that alone
flipped the instruction back and forth — a real prefix change, so a KV cache
miss on every short follow-up. Two things move it: the question switching
language for real (`detect()` disagrees), or the question naming one outright
— "svara på engelska", "förklara på japanska" — via
`language.requested_language()`, independent of what language it is itself
written in. Two shapes count as naming one:

| Shape | Example | Why it is not a mention |
|---|---|---|
| a verb that asks for a language, next to the name | "förklara på japanska", "byt språk till svenska" | "the German firmware bug" has no such verb |
| the name, in a message `detect()` places in no language at all | "svenska tack" | "varför är dokumentationen på engelska?" is Swedish on `är` and `på`, so it is left to the verb rule |

Both were measured as misses. With only "answer" verbs, "förklara på japanska"
went out under *Answer in Swedish and in no other language*. With no second
rule, a session locked to Korean answered "byt språk till svenska" — which
scores no stop word in any list — with a refusal, in Korean. The prompt now
also names the operator's request as the one thing that overrides the lock, so
the next phrasing the host misses costs a wrong language rather than a trap.
`/lang [NAME]` sets it by hand, `/lang auto` hands it back to detection.

A message that asks for *nothing but* the switch — `language.bare_switch()`,
which is `requested_language()` plus "and there is no other word in it" — is
answered by the host with one word from `language.OKAY`, and never reaches the
model at all. It used to cost a model turn that answered "Jag har ändrat
språket till svenska. Hur kan jag hjälpa dig med din BLDC-inverter?", under a
host line reading `språk: bytt till Swedish (låst)` — the same fact three
times, in two languages, one of them a lock nobody asked to be told about. The
answer being in the new language *is* the acknowledgement. A request with a
question attached ("förklara på japanska vad detta projektet handlar om") is
still the model's turn; the leftover-word test is the line between the two.

Tests build a `Chat` with no session language at all, deliberately: a suite
whose expectations depend on the Windows locale passes on one machine and
fails on the next. The locale is read in `build()`, at the entry point.

A Windows console encodes with its locale codepage, cp1252 here. Swedish and
German fit; a Polish `ł`, an ohm sign or anything Cyrillic does not, and the
default handler turns that into a `UnicodeEncodeError` that loses the answer
*after* the measurement. `dbg.py` sets `errors='replace'` instead, so a missing
alphabet costs a glyph rather than the reading, and leaves the codepage alone —
forcing UTF-8 would hand a legacy console mojibake for everything it could
have displayed. A redirected stream is the opposite case and does get UTF-8:
a file has no codepage to mismatch.

### Keeping the model loaded

`keep_alive` goes on **every** request. Ollama caches the KV state of a prefix
it has already processed — that is what makes turn nine as quick as turn two —
and throws it away when the model unloads. A bench session is mostly gaps: you
read a number, move a probe, think.

But the hold is not free, and the two modes want opposite things:

| | hold | why |
|---|---|---|
| prompt loop (`--repl`) | 30 min | the next turn is coming, and it reuses the prefix |
| one question (`-Ask`, `-q`) | 2 min | enough for an immediate follow-up, then the card is somebody else's |
| **leaving the prompt** | **released at once** | the cache has no further job |
| **entering the prompt** | **anything else is unloaded first** | a card with two models on it is how a load fails |

That last row matters on a workstation. Measured before it existed: a finished
session held **9.69 GB of a 16 GB card for another 27 minutes at 1 %
utilisation**, leaving the desktop 3.8 GB. Unloading on the way out returns the
card to 2.1 GB; a reload costs about seven seconds, and only if there is a next
time. `-Hold` keeps it, `--keep-alive 0` hands it back on any path, and an
explicit value beats the mode default.

Entry matters as much. A killed window, a `-Hold` from last time or somebody's
`ollama run` leaves weights on the card until their keep_alive expires, and the
next load then asks a full card — measured as a 500 reading `cudaMalloc
failed`. So `board_prompt.ps1` sweeps first, and says what it freed:

```
  ok    unloaded    llama3.1:8b was still resident, 4.9 GB freed
  ok    model       gemma4:12b  loaded in 7.0 s, held 30m
```

A model that is already the one this run wants is kept, not reloaded — that is
warm, not stale, and it reports `loaded in 0.0 s`. `-KeepOthers` leaves
everything where it is, for a machine deliberately running something else on
the card.

`preload()` sends `options` with the empty message list. Without `num_ctx` the
daemon loads the model's own default context — 128k on llama3.1, a 7 GB KV
buffer, and a 500 from the daemon. Worse when it succeeds: resident at one
context size, questioned at another, so it reloads and the preload has bought a
wait instead of saving one.

### When the runner dies under you

Measured repeatedly: llama-server terminates with `std::bad_alloc`, usually
while saving its prompt cache, and ollama answers 500 `model runner has
unexpectedly stopped`. The daemon respawns it on the next request, so recovery
was always "ask again" — by hand, until it was automatic.

`client._chat_once` retries `RUNNER_RETRIES` (2) times with a growing wait and
says nothing: a retry that worked is not news, and the meter counts replies
that arrived. A request ollama *refused* is never retried, and a machine
genuinely out of memory fails in seconds rather than looping.

### Why it dies, and the two variables that stop it

The retry above hides the symptom; the cause is llama-server's own memory,
not this loop's prompt. It saves a ~340 MiB slot state to its prompt cache
whenever a question's prefix diverges early from the cached one — which is
every question here — and keeps up to 32 context checkpoints of 320.013 MiB
each, a size fixed by the context window rather than by the prompt. The
evidence, and why prompt length is *not* the trigger, is in
[FINDINGS.md](FINDINGS.md).

```
LLAMA_ARG_CACHE_RAM       = 0   # no prompt cache: nothing here re-asks an old prompt
LLAMA_ARG_CTX_CHECKPOINTS = 0   # not 32, and not 2 either - see below
```

Capping the checkpoints at 2 was not enough: restoring a 311.575 MiB
checkpoint threw `std::bad_alloc` on its own and took the runner with it. Off
entirely costs nothing here, because `debug.Chat` clears its history after
every answered turn — there is almost nothing for a restored checkpoint to
restore. Measured off: ten questions, thirty model calls, zero
`std::bad_alloc`, one model load.

`board_prompt.ps1` sets them, into its own process and at User scope, and
restarts the daemon once if it was already running — an existing daemon keeps
the environment it started with, so setting them is otherwise inert. `-NoTune`
opts out; `-KeepOthers` blocks the restart rather than taking somebody else's
loaded model with it. See `$DaemonTuning` in `board_prompt/Tuning.ps1`.

### When there really is no room

Separate from the above, and handled in `client.py` rather than by the
launcher: an allocation failure that is genuinely about a full card.
`_out_of_memory` tells that apart from a crashed runner — a crashed runner
comes back on its own and the fix is to ask again, a full card stays full —
and `_make_room` climbs one rung at a time, ordered by what it costs to be
wrong about it:

1. unload every *other* model resident on the card, then drop this one and
   the caches around it, and ask again;
2. halve `num_ctx`, down to `MIN_NUM_CTX` (2048), for the rest of the session;
3. give up, and say which of `--num-ctx`, `--num-gpu` and a smaller tag the
   operator has left.

Each rung is recorded in `client.notes` rather than printed — a library that
writes to somebody's terminal is one nobody can embed — and `debug.Chat`
drains them into the same trace the tool results go to, so a session whose
context window just halved is told.

---

## What the model is not allowed to conclude

These are the board's invariants restated as limits on the model, and they are
in the system prompt for that reason:

- **The AFE switch powers the ADC reference.** With it off every channel reads
  exact mid-scale and the NTC reads exactly 25.00 °C. That is a plausible number
  and not a measurement. See [HARDWARE.md](HARDWARE.md).
- **Phase channels sit behind unknown gain.** Volts at the ADC pin is as far as
  the data goes — never a sensed current or a phase voltage.
- **The model never produces a verdict.** It reports a number through the
  `report` tool, against a JSON Schema the daemon enforces, and `plan.Limit`
  decides pass or fail in Python from a file under revision control.

## Structured output: tools, not JSON mode

Ollama's `format='json'` constrains the *content* field — the one part of a
reply this bench does not parse. Every number that reaches a verdict arrives as
a `report` tool argument instead. A model told to answer in JSON tends to
describe a tool call in its content rather than making one, so json mode
competes with the tool path instead of helping it.

It is available for callers outside the runner — `Ollama(fmt='json')`, or
`dbg.py --format json`, usually with `-t none` — and off everywhere else.

## Arguments arrive in whatever shape the model felt like

The schema says array of strings and a capable model sends one. A smaller one
sends `ch="ntc"`, or the string `"['NTC']"`, or `samples="100"`. Unhandled,
`for item in "ntc"` iterates characters and the tool answers `unknown channel
'n'` — which tells the model nothing it can act on, and what it does next is
answer from memory.

The same goes for the name of a thing, and there are three rules:

* punctuation is stripped, so `dc_bus`, `dc-bus`, `DC bus` and `dcbus` are one
  key;
* the phase conventions alias each other — A/B/C and U/V/W are two names for
  the same three phases — but only onto channels the board has, since a board
  without a Phase W has no Phase C;
* a word that can only mean one channel resolves to it: `bus` is inside
  `dcbus` and nothing else, and `temp` is a `SIGNAL_ALIASES` entry for `ntc`.
  One that could mean several says so — *channel 'phas' could be phaseu or
  phasev or phasew - say which* — rather than "unknown", which reads as "no
  such thing";
* and a name built out of words is read as its words: `BUS_VOLT`,
  `bus_voltage`, `NTC_TEMP`, `ADC_CH3`, `PhaseAVolt`. Separators and
  camelCase both split. A single letter counts as a phase only beside the
  word that says so — without that rule `not_a_channel` resolved to PhaseU
  through its bare `a`, which is the invented reading this file exists to
  prevent. `A0` stays refused: it is a pin name, and guessing which channel
  hangs off it would be inventing one.

Measured for each: `['ntc','dc_bus','phase_a','phase_b','phase_c']` lost all
five readings to the two spelled the other way, `['bus']` was refused with
`dcbus` listed in its own refusal, and `BUS_VOLT` was invented outright.

That last one had a second cause, in this repository rather than in the model.
Terse mode dropped analog_read's `omit for all` — the only line saying how to
ask for every channel — so the model started naming them itself and read five
of seven. `detail._is_schema` keeps a property description that carries
enumerated values, a spelling, **or how to leave the field out**: all three are
schema, and none is written anywhere else.

`coaxial_mcp.tools.coerce` converts every argument to the type the tool's own
`inputSchema` declares, and refuses what will not convert **by field name and
wanted type**. Ollama path only: the MCP server gets the same protection from
the protocol library, which validates against `inputSchema` before a handler is
reached.

## Reading these documents from a prompt

The `docs` tool reaches this file and its neighbours without anyone pasting
them in — **off by default**, and asked for by name:

```powershell
dbg -t docs "what does FINDINGS say about PCSEL?"     # or /tools docs
```

```
docs()                           the index: every document and its headings
docs(doc='FINDINGS')             headings of one document
docs(doc='MODELS', section='Threads')
docs(find='25.00')               where a phrase appears, with its heading
```

Index first, section second: a tool returning a whole document by default
would cost more than it is worth.

It is out of `read`, `code`, `pins` and `build` for a stronger reason than
cost. Asked to *measure* the channels, `gemma4:12b` called `docs`, pulled
thousands of tokens of HARDWARE.md into context and answered with that
document's channel table — no measurement in it. Removing the tool cut the
same question from 6229 prompt tokens to 2645 and turned the answer back into
a reading. `DOCS_HINT` says so, and only when `docs` is offered.

### Terse and full: documentation sized for whoever is reading

Every description is re-sent every turn, and the readers are not alike: Claude
over MCP has hundreds of thousands of tokens, `gemma4:12b` has 8192 shared with
the conversation and the readings. Writing for the smaller one shortchanges the
larger, and writing twice is two things to keep in step — so `detail.py` picks
the length from one spec carrying both forms, **from the model, not a flag**:

| Reader | Level | Why |
|---|---|---|
| `gemma4:12b`, `qwen2.5:14b`, `llama3.1:8b` | terse | parameter count under `FULL_MODEL_B` (30 B) |
| a tag that names no size (`qwen3.6:latest`) | terse | the local daemon is where unnamed tags live |
| an ollama `:cloud` tag | full | somebody else's hardware, and not short of room |
| the MCP server, with no model to read | full | the reader on that pipe is not the one paying |

`--detail terse|full|auto` overrides it per run (`dbg`, `python -m
coaxial_ollama`, `python -m coaxial_mcp`), `/detail` switches it mid-session
and reprices the turn, and `COAXIAL_DETAIL` decides for the whole machine.

What it actually costs, measured on this tool surface:

| Set | full | terse |
|---|---|---|
| `read` | 620 tok/turn | 439 |
| `code` (the default) | 943 | 662 |
| `all` | 1435 | 1085 |

Terse also clips a `docs` section at 1200 characters instead of 4000, halves
the search hits, and drops the subsection headings from the index — while
keeping the chapter names, because a chapter name is how the next call is
spelled, and keeping the line that teaches the call shape, because an index
that teaches nothing costs more over a session than it saves in a turn.

What is deliberately **not** gated on this: the behavioural hints in
`debug.py` (`BUILD_HINT`, `LINK_DIAGNOSE_HINT` and the rest). Each of those
exists because a small model needed telling, so trimming them for small
models would delete them exactly where they earn their place. This mechanism
shortens documentation, not instructions.

## Tools beyond the board

Beyond the nine board tools shared with the MCP server, three narrow ones,
each wrapping a fixed script or a fixed OS check rather than handing the
model a general-purpose command line - see `host/coaxial_ollama/tools.py`
for the schemas and `host/tools/` for the scripts:

| Tool | Wraps | Gated by |
|---|---|---|
| `build_firmware` | `host/tools/build_and_flash.py` - build, flash, or both | `--confirm` (always a write) |
| `run_tests` | `host/tools/run_tests.py` - the offline suites' own tally, never a model paraphrase | nothing - read-only |
| `link_diagnose` | `host/tools/find_board.py` - an ordered checklist, most fundamental fact first | nothing - read-only |

`build_firmware` and `run_tests` are in the default `code` set; all three are
in `read`/`pins`/`build` too. Each has a matching, conditional line appended
to `SYSTEM` only when it is actually offered (`BUILD_FIRMWARE_HINT`,
`BUILD_HINT`, `LINK_DIAGNOSE_HINT` in `debug.py`) - existing in the schema is
not enough on its own, see the entries below.

`link_diagnose` stops at whichever step actually explains the silence,
rather than running every later one regardless:

1. Target power over SWD, via the ST-Link (`find_board.check_power()`) - the
   one check the serial side cannot make on its own, and the most
   fundamental: nothing past it can work without it. Measured live on this
   bench, an unplugged ST-Link cable read `Voltage: 0.00V`, where the
   serial side alone only ever said "silence."
2. COM ports Windows currently sees.
3. Whether the configured one is among them.
4. Whether the board actually answers on it right now - measured directly,
   so this also correctly says "up" if the link had already recovered.
5. `probe_other_ports: true` tries every other port, for a board that moved.

Both `link_diagnose` (imported, in-process) and `board_prompt/ComPort.ps1`'s
`Test-BoardPort`/`Find-BoardPort` (subprocess, since PowerShell cannot
import Python) call the same `find_board.py` - one implementation of "does
this port answer," not two that can drift apart.

## Two things a debugging session leaves behind

`Chat.prompt_history` is every question typed this session, independent of
`self.history`, which the REPL clears after each answered turn to keep the
prompt from growing. `/history` lists it, `/clear_history` empties it, and
`trim()` folds the last five into `SYSTEM` as "steps already tried" - so a
multi-turn "why won't it connect" reads as one investigation.

`IOLog` writes `host/prompt_io.tmp`, hidden and overwritten each session:
every question, every call (including the ones `_trace()` skips) and every
answer, for reading back when there is no terminal transcript to paste in.
Off on a bare `Chat()`, since dozens of tests build one.

---

## The live suite

`tests/test_live_model.py` is the only suite here that does not script the
model. Six turns against the real tag and the real board, checking the three
things a scripted double cannot:

| Turn | Reaches the board | Answer |
|---|---|---|
| "läs NTC:n och DC-länken" | yes, `analog_read` | Swedish |
| "beskriv hårdvaran i detta projektet för en novis" | no | Swedish |
| "byt språk till engelska" | no, and no model turn | `Okay` |
| "read the NTC" | yes | English |
| "what is this project about" | no | English |
| "byt språk till svenska" | no, and no model turn | `Okej` |

It asserts *that* `analog_read` was called, never what it returned —
invariant 10. Measured 2026-08-24, gemma4:12b on COM4: 24 passed, 0 failed.

`python tools/run_tests.py --live`, or the file directly with `--simulated`
for the model half without a cable. It is not in the default set: a model load
plus six turns is minutes, and the other suites are seconds.

## Measured failure modes

One lesson runs through almost all of these, and it is why the loop is built
the way it is:

> **Telling the model not to do something does not stop it. A fact the loop
> already holds, that the model gets no vote on, does.**

Every backstop in `debug.py` exists because a sentence in `SYSTEM` was tried
first and did not hold. The table is the short version; the entries below it
carry the detail that is worth more than a row.

| SYSTEM said | It did anyway | What settles it now |
|---|---|---|
| never restate a tool's own rows | retyped the whole table as comma-separated prose, three sessions running (`qwen2.5:14b`) | `replies.is_retype` - an answer naming every channel just read is replaced with silence |
| never answer from an old reading | invented a full table one round trip after the ST-Link was unplugged, values a few counts off the real one | `link_error` override, and `self.last_channels` kept across turns |
| afe_power only when asked, never to serve a reading | turned the AFE back on to "serve" a reading, the turn after being told to turn it off | `Toolbox._permit` refuses it when the question never said "afe" |
| a call error is reported, never hidden | answered "kortet har byggts och flashats" after declining that exact call at the `--confirm` prompt | `code_error` override, same shape as `link_error` |
| (schema) build_firmware exists | "Nej, jag programmerar inte firmwaren själv" - training beat the schema | `BUILD_FIRMWARE_HINT`, sent only when the tool is offered |
| (schema) link_diagnose exists | called `build_firmware` when asked why the board was silent - a guess at a fix, not a diagnosis | `LINK_DIAGNOSE_HINT` |
| (schema) run_command's allowlist | tried `python3` (not allowlisted) and the wrong directory | `BUILD_HINT` |
| (nothing said which model it is) | answered the tag question out of its training, a name and a maker that need not match `ollama ps` | the turn's system message names the tag the daemon was asked for |

Two things that table is not. It is not an argument against writing the rule
down - every one of those rules is still in `SYSTEM`, because the model does
follow them most of the time and the backstop only has to catch what is left.
And it is not a claim that a backstop is always available: `is_retype` can
only fire because the loop already knows which channels were read.

### llama3.1:8b answered from memory when a tool call failed

Faster: three questions in 24.0 s against 31.3 for `gemma4:12b`, two model
calls per question against three, 1.2k prompt tokens against 2.9k. Then it was
asked for the board temperature, called `analog_read` with `ch="ntc"`, got
`unknown channel 'n'` back, and said **"The board temperature is 25.00 C"** -
three runs running, for a board `board temp` had at 36.3. That is the AFE-off
number: invented, wrong by 11 °C, and wrong in the one shape a reader here is
least likely to question.

The argument coercion in `coaxial_mcp.tools.coerce` came out of that. After
it, llama3.1:8b reads the board correctly - and still loses, because it passes
`ntc_beta=3950` where the onboard part is a Murata NCU18XH103 at **B=3380**
(`coaxial/scaling.py`). A silent 1.7 °C bias from a plausible-looking constant
the model supplied itself.

`gemma4:12b` overrode nothing, landed within 0.5 °C, and when asked whether
the AFE was on answered by reasoning that the NTC was *not* reading exactly
25.00. The full entry is in [FINDINGS.md](FINDINGS.md).

### A rule the model can satisfy literally while breaking it

`SYSTEM` said "never a markdown table" and "no second list". The model
complied with both - comma-separated prose is neither - while doing exactly
what the rule existed to prevent. Reworded to name the act rather than its
shapes: never restate rows a tool already printed, table or not.

Building the backstop caught a bug in itself before it shipped: the row-matching
regex keyed on a leading digit and a word, which is also the shape of
`analog`'s own header line, `64 smp @2000Hz` - `smp` was briefly a channel of
its own count. Anchoring on the mode column (`diff`/`SE`) that only a real row
has fixed it. Which is why the reproduction ran against a real render before
the fix was called done, not against a hand-written string shaped like one.

### The same call, asked again, and again

Asked to tabulate every channel, `qwen2.5:14b` turned the front end on, saw
`on=1 pe15=0`, and turned it on again - three more times, identical call,
identical result, each a full round trip for a fact it already had.

`Chat.ask` now remembers, per question, the (name, arguments) of every call and
its result. A call outside `REPEATABLE` that repeats exactly is answered from
that memory: `unchanged this turn, already asked: on=1 pe15=0`. `analog_read`,
`run_python` and `run_command` stay out of the dedup on purpose - a second
reading is a new sample, and the DC bus does not hold still for a cache.

The trap: a repeat of a *failed* call must not read as a fresh success. The
dedup keeps the original result under the wrapping sentence and classifies the
link from that, not from the sentence - otherwise `unchanged this turn, already
asked: ERR ConnectError: ...` would not match the `ERR ` prefix the link-down
override looks for, and a cable pulled mid-turn would go quiet exactly where
it must not.

### A call written as text, more than one at a time

`dbg.py` recovers a tool call the model typed into `content` instead of
`tool_calls` - the shape was wrong, the intent was right, and a JSON parse is
cheaper than a wasted turn. The first version recovered exactly one call from a
message that was *nothing but* that call. Asked "vad ar temperaturen", the
model sent two:

    CallCheckFunction
    {"name": "analog_read", "arguments": {"ch": ["NTC"], "rate_hz": 100, "samples": 10}}
    CallCheckFunction
    {"name": "afe_power", "arguments": {"action": "read"}}

Nothing ran, and the prompt printed all four lines as the answer - which at a
bench does not read as a parse failure, it reads as the board having stopped
giving values.

`replies.salvage_calls` now takes every call in the message and matches braces
rather than running a non-greedy regex (`{.*?}` ends at the brace closing
`arguments`, so a nested argument parses as half of itself). The veto is what
makes it safe: with the tags and call objects removed, anything left has to be
marker noise - `tool`, `call`, `function`, `check`, split at capitals so
`CallCheckFunction` counts as three. One word of real prose and the message is
printed as the answer it probably is. Turning a sentence that merely quotes
JSON into a board command is far worse than showing the JSON.

### The call header, clipped, ate its own result

`_trace` used to print the call above its result. Asked for five channels, the
arguments were long enough that `clip` cut them, and the cut notice landed a
newline inside the header:

    analog_read samples=100 ch=['dc_bus', 'ntc', 'phase_a', 'phase_b', 'phas
    ... [17 more characters cut] -> 100 smp @2000Hz

The `->` and the table's first row ended up stuck to a truncation notice - and
the header was restating what the table says better anyway, since it names
every channel one per row. The header is gone.

### A model that stops in prose

The runner nudges twice - "either call a tool, or call report to finish this
step" - then records the step `unfinished` rather than accepting prose as a
result. An unfinished step is a visible hole in the report; a paragraph
accepted as a measurement is not.

### Noise the operator does not need, that the log still keeps

The `afe_power` gate works: a refused call is followed by a correct
`analog_read` one call later, every time measured. What was left was the
refusal in the trace - accurate, but noise for something already corrected in
the same turn. `Chat._trace` skips that one shape of result. It stays in
`self.history`, so the model still reads its own mistake, and in `IOLog`
unconditionally, since "why did that turn cost four calls" is exactly the
question that log exists to answer.

### Describe: a table, silence, then a cable

`prompt_io.tmp`, verbatim: "Beskriv hårdvaran i detta projektet för en novis"
→ `analog_read {"ch": ["all"]}`, the table, and `A:` with nothing after it.
Two faults on one screen.

The silence was `_ask_inner`: its comment said *a blank answer is never valid
and needs no such gate*, and the code had `not answer` inside the
`not last_channels` gate — closed by the very call that had just succeeded.
A turn now nudges once for words and ends on a line rather than on nothing.

The table was SYSTEM saying when to call `analog_read` and never when not to.
One line added — *Describe, explain or compare means words, from what you
know: analog_read answers what a channel reads now, never what a thing is.*
Re-measured on gemma4:12b with the same question: no tool call, an answer in
words.

That answer also placed the serial link "över koaxialkabel", and the next
session did it again: "en koaxial anslutning för seriell kommunikation".
Twice is not carelessness — it was SYSTEM's own first line, *You are an expert
with a serial link to a coaxial BLDC inverter.* Two words, adjacent, no
relation stated, and the model fused them.

Nothing about the wiring is coaxial: the PCB mounts coaxially behind an
outrunner's **stator**, and the UART leaves the board over the debug probe's
COM port or RS485. The line now says both, and it is the one place in SYSTEM
carrying a hardware fact rather than a rule — 16 tokens a turn, against a
false statement about the product in every description it wrote.

That moved the prompt from 184 to 200 tokens, past the `< runner/3` guard the
suite had held it under. The guard is now `< runner/2.5`: none of the rules
beside it is fat that could have paid, and 200 against 556 is still the
fraction that test is about.

`docs/HARDWARE.md` was wrong in the same direction — it said the PCB sits
behind the **rotor**. On an outrunner the rotor is the spinning can; the board
is behind the stator. A model reading that document would have got it wrong
from the document.

### A nudge stole the language lock from what was actually asked

`trim()` read `asked` as the last `role=='user'` message in history - which is
also the shape of a mid-turn nudge ("Call the tool now - do not describe it."),
appended for the model's benefit. Measured live: a Swedish question that
triggered that nudge had its session language flip to English on the very next
`trim()`, the operator's own words still in history but no longer the ones
being read. `trim()` now reads `self.prompt_history[-1]` - appended exactly
once per turn, at the top of `ask()`, before the turn can add anything.

### The dead link, explained twice on one screen

Measured with the board unplugged: every board question printed
`link_diagnose`'s four-step checklist **twice** - once as the tool trace, once
inside the answer - and paid the ST-Link's fifteen-second timeout for both,
because `_link_down_message` called the tool again rather than reusing what
the model had already called this turn. The two copies were not even the same
text: the trace cut each row at 96 characters, mid-word, while the answer
carried it whole.

Two fixes, one rule apiece. The answer now says only what the trace does not
(`shown=` in `_link_down_message`): one line, the error class and its first
clause, no checklist. And the trace wraps rather than cutting - a reading row
is inside the width and untouched, prose continues on the next line, and no
row may take more than `TRACE_LINES` of them.

`-q` is the exception and keeps the whole thing: with no trace on screen,
the checklist has nowhere else to be.

### A channel called by what it measures

`ch=['bus']` came back `unknown channel 'bus'; names are ch3,ch6,dcbus,ntc,...`
- a refusal listing the channel it meant. `_key` had always collapsed
punctuation (`dc_bus`, `DC bus`) and `PHASE_ALIASES` the other phase
convention (`phase_a`), but neither reaches a word that is simply what the
operator calls the thing.

Two rules now, in `_alias`: a small table for the semantic names (`bus`,
`vbus`, `temp`, `temperature`, `thermistor`), and a unique substring match for
everything else - `bus` is inside `dcbus` and inside nothing else, so it
resolves; `phas` is inside three channels, so it raises `could be phaseu or
phasev or phasew - say which` instead of "unknown", which reads as "no such
thing" and sends the next call somewhere else.

### The eager board connect that ended a turn before it started

`main()` used to open `session.board` before the model was asked anything, so
a dead link failed loudly before tokens were spent - written when catching it
early was the only thing between that and the model answering past it.
`link_diagnose` and the `link_error` override do that job now, and better: the
model gets a real turn to help instead of a one-shot question exiting with code
2 before it was ever asked. `Session.board` was always lazy; this removed the
one thing forcing it early.
