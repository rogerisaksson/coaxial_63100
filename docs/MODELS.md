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

From inside the editor there are two ways to the prompt, both in the docked
terminal panel: the **Board prompt** profile in the terminal dropdown for a
conversation, and **Ctrl+Shift+B** for one question. Both are checked in, in
`.vscode/settings.json` and `.vscode/tasks.json`. Nothing outside VS Code can
type into a terminal that is already open, which is why these exist rather than
something that pushes a command into the one you are looking at.

`host/coaxial_ollama/capability.py` measures the machine and picks. Ask it:

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

Two of those are worth dwelling on. **Free RAM, not installed RAM**: a 64 GB
workstation with 8 GB left cannot hold a 42 GB model however impressive the
sticker is, and the failure mode is the machine swapping rather than an error
anyone can read. **VRAM in use minus ollama's own**: the probe usually runs
while the last question's model is still resident, and counting our own 7.8 GB
as somebody else's desktop reserved 12.8 GB of a 16 GB card and picked something
smaller — which became the new baseline next time. A ratchet, measured before it
was fixed.

CPU load is deliberately *not* an input to the choice. A snapshot says nothing
about the next ten minutes, and the tag is chosen for the session; a busy machine
gets a warning that the CPU half of a split choice will be slower than the
figures below, which were all measured idle. Only tools-capable tags are candidates: everything here
reaches the board through tool calls, and a tag without them describes a
measurement instead of taking one.

`setup.ps1` pulls whatever the picker chooses, and so does `board_prompt.ps1` on
a machine where the tag is missing — it asks the picker, pulls, loads and only
then opens the prompt. `-Model TAG` overrules it everywhere.

Anything driving this from outside — including Claude Code, see the routing
table in [../CLAUDE.md](../CLAUDE.md) — should reach for `board_prompt -Ask` or
`dbg -m auto -q` rather than reason about the board from memory. The local model
is free per token and standing next to the hardware; the expensive one is not.

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

The reserve is the VRAM the picker deliberately does not spend, and the default
is the largest of three numbers: 2 GB, a quarter of the card, or **what the card
is already holding plus 2 GB to grow into**. That third one is the one that
matters on a workstation. Measured here with nothing of ours running:

```
card 16.0 GB, desktop alone 2.6 GB, 0 % utilisation
```

A flat quarter of the card would hold back 4.0 GB, of which the desktop already
occupies 2.6 — leaving it 1.4 GB for a second 4K surface, a video that starts
playing, a browser tab with a canvas in it. When that is not enough the driver
evicts to satisfy the allocation, and what you see is not an error but a
momentary hang. Counting what is already there gives 4.6 GB instead.

That still is not much on a busy machine, so the number is settable:

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

Be clear about what each of those buys. The priority is **CPU** scheduling: with
the model wholly on the card it deprioritises tokenisation, sampling and the
serial I/O, not the matrix multiplies, so it helps the desktop stay responsive
around a question rather than during one. The single-model, single-context
limits are about memory: two contexts is how a 16 GB card ends up asked for two
copies of the weights, which was measured here as a 500 from the daemon reading
`cudaMalloc failed` with nothing obviously wrong at either end.

The lever that actually reduces GPU contention is the model's size, which is the
table above.

### Threads

Raising `num_thread` on 64 threads bought nothing and cost a little, CPU-only:

| threads | default | 16 | 32 | 64 |
|---|---|---|---|---|
| tok/s | 6.4 | 6.4 | 6.3 | 5.7 |

Decode is bandwidth-bound, not core-bound, and filling both SMT siblings of
every core makes it worse. Nothing in this repository sets `num_thread`; the
daemon's own choice beat everything tried against it.

### The prompt, and what stops an answer short

Two things a reader of a transcript will notice before anything else.

The prompt spins beside the board's name — `|`, `/`, `–`, `\` in one column —
**and keeps spinning while you type**. The first version stopped at the first
keypress, because a spinner that redraws its line repaints under the characters
being typed, which is how a prompt eats an argument. Stopping made it useless
exactly when you are looking at it, so `spinner.py` repaints only its own cell
and puts the cursor back: `ESC 7`, column, glyph, `ESC 8`. The typed text is
never written over because it is never written to.

The bar is an en dash, not a hyphen: at the size a terminal draws them a hyphen
is a third the width of `|` and the spinner visibly limps. cp1252 has it at
0x96, so this console renders it; a console that cannot encode it is asked
first and gets the ASCII set rather than a question mark in the corner of your
eye.

What can still go wrong is a line long enough to wrap, since the column is on
the current row. A bench question is not that long, and the alternative is
writing a terminal emulator. Redirected output gets one static prompt, no
escapes and no thread. It sits in the same
docked panel as a PowerShell prompt, and two terminals with a `>` in them look
identical at a glance; a moving dot says which one is waiting for a question
without a banner or a colour to remember. Nothing animates while there is text
on the line, because an animation repainting under typed characters is how a
prompt eats an argument. Redirected output gets one static prompt and no
animation at all.

And an answer that hits `--words` now says so. Measured from the prompt: a
seven-channel table stopped mid-row at exactly 180 generated tokens, which
reads as a complete answer to everyone except a reader counting rows. The reply
carries `done_reason`, so a truncated answer is marked *[cut off at --words
180]* rather than quietly ending.

### The language of the answer

The bench prompt is English and the answer follows **the question**, not the
prompt: ask in Swedish and the reading comes back in Swedish. Units and channel
names stay as the board prints them — `NTC`, `DCbus`, `V`, `C` — because those
are what appears in `board_info`, in the CSVs and in these documents, and a
translated channel name is a channel name nobody can grep for.

**The language is decided here, not by the model.** The first attempt told the
model to "answer in the language the question was asked in", which asks it to
do two things: work out what language that was, and then answer. The first is
where it drifts — reported on this bench with `qwen2.5:14b`, a model whose
training leans heavily Chinese, answering European questions in Chinese,
Japanese and Thai.

`host/coaxial_ollama/language.py` decides instead, and the turn's system message
says it plainly: *The question is in Swedish. Answer in Swedish and in no other
language.* Two stages, both small on purpose — script ranges settle Chinese,
Japanese, Korean, Thai, Greek, Cyrillic, Hebrew and Arabic outright, and a short
stop-word count separates the Latin ones. It abstains when the winner is not
strictly ahead: Danish and Norwegian score identically on most of the list, and
telling a Dane to answer in Norwegian is worse than saying nothing. An
abstention falls back to the old instruction, where the model mirroring the
question is usually right.

In `dbg.py`'s REPL, the language **locks** on the first question that is not
itself ambiguous (`Chat.language`), rather than being rebuilt fresh every
turn: a one-word follow-up like "tabellera" detects as nothing on its own,
and rebuilding from that alone flipped the prompt back to "mirror the
question" every time one came up — a real prefix change, not cosmetic, so
that was a KV cache miss on every short follow-up. Two things move the lock
once set: the question switching language for real (`detect()` disagrees),
or the question naming a language outright — "svara på engelska" — via
`language.requested_language()`, independent of what language it is itself
written in. `/lang [NAME]` reads or sets it by hand; `/lang auto` unlocks.
One-shot calls (`-q`, `--ask`) have no session to lock across, so this
degrades to the old per-question detection there without any special case.

One consequence worth knowing, because it is a Windows console and not a
choice: standard output encodes with the locale codepage, cp1252 on this bench.
Swedish and German are inside it and render correctly. A Polish `ł`, an ohm sign
or anything Cyrillic is not, and the default error handler turns that into a
`UnicodeEncodeError` — which loses the whole answer *after* the measurement was
taken. `dbg.py` sets `errors='replace'` on stdout and stderr instead, so an
alphabet the console cannot hold costs a glyph rather than the reading.

Forcing UTF-8 would fix the encode and hand a legacy console mojibake for every
character it *could* have displayed. For the languages actually spoken at this
bench that is the worse trade, so the codepage is left alone.

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

That last row is the one that matters on a workstation. Measured before it
existed: a finished session left **9.69 GB of a 16 GB card resident for another
27 minutes at 1 % utilisation**, with the desktop given 3.8 GB to work in — a
cache nobody was going to hit. `board_prompt.ps1` now unloads on the way out,
and the card goes back to 2.1 GB used. A reload costs about seven seconds, and
only if there is a next time; `-Hold` keeps it resident when there is.

`--keep-alive 0` hands the VRAM back immediately on any path, and an explicit
value always beats the mode's default.

The entry side matters as much as the exit. Not every exit is clean — a killed
window, a `-Hold` from last time, somebody's own `ollama run` in another
terminal — and each leaves weights on the card until their keep_alive expires.
The next load then asks a card that is already full, which on this bench was a
500 from the daemon reading `cudaMalloc failed` with nothing obviously wrong at
either end. So `board_prompt.ps1` sweeps before it loads, and says what it
freed:

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

Measured repeatedly on this bench: llama-server terminates with
`std::bad_alloc` — usually while saving its own prompt cache — and ollama
answers 500 `model runner has unexpectedly stopped, this may be due to
resource limitations`. Nothing is wrong with the machine; the daemon respawns
the runner on the very next request, so the recovery was always just "ask
again". Until it was automatic, that meant the operator retyping a question
that had already been answered everywhere except in the reply.

`client._chat_once` now retries `RUNNER_RETRIES` times (2), waiting
`RUNNER_RETRY_WAIT` × attempt between tries, and says nothing: a retry that
worked is not news, and the token meter counts replies that arrived, not
attempts. A request ollama *refused* — a bad schema, an unknown field — is
never retried, because asking again just makes the same mistake twice. A
machine genuinely out of memory still fails, in seconds, rather than looping.

### Why it dies: the prompt cache, and 32 checkpoints of 320 MiB

The retry above hides the symptom. This is the cause, read out of
`%LOCALAPPDATA%\Ollama\server.log` while reproducing it deliberately:

```
slot get_availabl: - checking sim = 0.255 (323/1265) > 0.100
srv   prompt_save:  - saving prompt with length 1446, total state size = 342.623 MiB
libc++abi: terminating due to uncaught exception of type std::bad_alloc
llama-server terminated  exit.code=3221226505 (0xc0000409)
```

Two allocators, both llama-server's, neither of them about the size of the
question:

* **The prompt cache.** `prompt cache is enabled, size limit: 8192 MiB`.
  When a new prompt shares little of its prefix with the cached one, the
  server saves the whole slot state — ~340 MiB here — into that cache before
  starting the new one. Every question in a bench session is that case:
  `debug.Chat` clears its history after each answered turn on purpose, so the
  prefix after the system prompt is new every time (`sim = 0.25` … `0.33`
  measured). The allocation intermittently throws, uncaught, and takes the
  process with it.
* **Context checkpoints.** `context checkpoints enabled, max = 32`, each one
  `320.013 MiB` — a size fixed by the context window, not by the prompt. 32 of
  those is 10 GB beside 8 GB of weights on a 16 GB card.

**Prompt length is not the trigger**, and that is worth stating plainly
because it is the obvious hypothesis and it is wrong: the crashing prompts
were 1249 and 1446 tokens of an 8192-token window, and the state saved was
339 MiB and 342 MiB — the *same*, because the checkpoint size saturates. See
[FINDINGS.md](FINDINGS.md).

Two environment variables fix it, and `board_prompt.ps1` now sets them and
restarts the daemon once if it has to (`-NoTune` opts out) — see
`$DaemonTuning` in `board_prompt/Ollama.ps1`:

```
LLAMA_ARG_CACHE_RAM       = 0   # no prompt cache: nothing here re-asks an old prompt
LLAMA_ARG_CTX_CHECKPOINTS = 2   # instead of 32; checkpoint 1 is restored on the next turn anyway
```

Measured, same twelve-question session either way: untuned, the runner died
and reloaded 8 GB mid-session, twice in two attempts; tuned, 36 model calls,
zero `std::bad_alloc`, one model load. They are `LLAMA_ARG_*` rather than
`OLLAMA_*` because they belong to llama.cpp's argument parser, which reads
them out of the environment ollama hands its runner — `ollama serve --help`
does not list them, `libllama-common.dll` does.

An already-running daemon keeps the environment it was started with, which is
why setting them is not enough on its own and the restart exists. They are
persisted at User scope too, so the tray app inherits them at the next login
and the restart never happens again.

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

The same goes for the name of a thing. `dc_bus` for `dcbus` is a separator, not
a mistake, and `phase_a` for `phaseu` is not a mistake either — A/B/C and U/V/W
are two conventions for the same three phases and both appear in the same
datasheets. Measured: a model asked for `['ntc','dc_bus','phase_a','phase_b',
'phase_c']` and lost all five readings to the two it spelled the other way. So
punctuation is stripped before matching, the phase conventions are aliases of
each other — but only onto channels the board actually has, since a board
without a Phase W has no Phase C — and a near miss is named in the error:
*unknown channel 'dcbusvoltage' - did you mean 'dcbus'?*

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

Index first, section second, on purpose: the tool list is re-read every turn
(see the token argument in [ARCHITECTURE.md](ARCHITECTURE.md)), so a tool
returning a whole document by default would cost more than it is worth.

It is out of `read`, `code`, `pins` and `build` for a stronger reason than
cost. Asked to *measure* the analog channels, `gemma4:12b` called `docs`,
pulled several thousand tokens of HARDWARE.md into context, and answered with
that document's channel table — no measurement in it anywhere. Removing the
tool cut the same question from 6229 prompt tokens to 2645 and turned the
answer back into a reading. The board is the authority on what the board
reads; the documents explain what a reading *means*, which is a different and
much rarer question. `DOCS_HINT` says so, and is sent only when `docs` is
actually offered.

### Terse and full: documentation sized for whoever is reading

Every tool's description and every schema property description is re-sent on
every single turn, and the readers are not alike. Claude, over MCP, reads a
description out of a window measured in hundreds of thousands of tokens;
`gemma4:12b` pays for the same text out of 8192 shared with the conversation,
the readings and the answer. Writing for the smaller reader shortchanges the
larger one, and writing twice is two things to keep in step — so the length is
picked by code, from one spec that carries both forms.

`host/coaxial_mcp/detail.py` is that code, and the level is **decided from the
model, not from a flag**:

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

`Chat.prompt_history` is every question typed this session, in order,
independent of `self.history` - which the REPL clears after each answered
turn, on purpose, to keep the prompt from growing. `/history` lists it,
`/clear_history` empties it. `trim()` also folds the last five entries into
`SYSTEM` as "troubleshooting steps already tried," once there is more than
the question just asked to show - a multi-turn "why won't it connect"
conversation reads as one investigation, not a run of unrelated questions.

`IOLog` writes `host/prompt_io.tmp`, hidden (Windows' attribute, not
security), overwritten each session - every question, every tool call
(including the ones `_trace()` skips on screen) and every answer. Not for
the operator: for reading back afterwards, in a session with no terminal
transcript to paste in. Off by default on a bare `Chat()`, since building
one is what dozens of tests do; `repl()` and the one-shot path in `main()`
are what turn it on, on the one `Chat` a real run actually uses.

---

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

### A nudge stole the language lock from what was actually asked

`trim()` read `asked` as the last `role=='user'` message in history - which is
also the shape of a mid-turn nudge ("Call the tool now - do not describe it."),
appended for the model's benefit. Measured live: a Swedish question that
triggered that nudge had its session language flip to English on the very next
`trim()`, the operator's own words still in history but no longer the ones
being read. `trim()` now reads `self.prompt_history[-1]` - appended exactly
once per turn, at the top of `ask()`, before the turn can add anything.

### The eager board connect that ended a turn before it started

`main()` used to open `session.board` before the model was asked anything, so
a dead link failed loudly before tokens were spent - written when catching it
early was the only thing between that and the model answering past it.
`link_diagnose` and the `link_error` override do that job now, and better: the
model gets a real turn to help instead of a one-shot question exiting with code
2 before it was ever asked. `Session.board` was always lazy; this removed the
one thing forcing it early.
