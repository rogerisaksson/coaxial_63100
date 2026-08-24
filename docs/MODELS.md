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

The `docs` tool is how the model reaches this file and its neighbours without
anyone pasting them into a prompt:

```
docs()                           the index: every document and its headings
docs(doc='FINDINGS')             headings of one document
docs(doc='MODELS', section='Threads')
docs(find='25.00')               where a phrase appears, with its heading
```

Index first, section second, on purpose. The whole tool list is re-read every
turn — see the token argument in [ARCHITECTURE.md](ARCHITECTURE.md) — so a tool
that returned a whole document by default would cost more than it is worth.

## Tools beyond the board

Three narrow, purpose-built tools, each wrapping a fixed script or a fixed OS
check rather than handing the model a general-purpose command line - see
`host/coaxial_ollama/tools.py` for the schemas and `host/tools/` for the
scripts:

| Tool | Wraps | Gated by |
|---|---|---|
| `build_firmware` | `host/tools/build_and_flash.py` - build, flash, or both | `--confirm` (always a write) |
| `run_tests` | `host/tools/run_tests.py` - the offline suites' own tally, never a model paraphrase | nothing - read-only |
| `link_diagnose` | COM ports Windows actually sees right now, vs. the configured one | nothing - read-only |

`build_firmware` and `run_tests` are in the default `code` set; all three are
in `read`/`pins`/`build` too. Each has a matching, conditional line appended
to `SYSTEM` only when it is actually offered (`BUILD_FIRMWARE_HINT`,
`BUILD_HINT`, `LINK_DIAGNOSE_HINT` in `debug.py`) - existing in the schema is
not enough on its own, see the entries below.

---

## Measured failure modes

### llama3.1:8b answered from memory when a tool call failed

Faster: three questions in 24.0 s against 31.3 for `gemma4:12b`, two model calls
per question against three, 1.2k prompt tokens against 2.9k. Then it was asked
for the board temperature, called `analog_read` with `ch="ntc"`, got
`unknown channel 'n'` back, and said **"The board temperature is 25.00 C"** —
three runs running, for a board `board temp` had at 36.3. That is the AFE-off
number: invented, wrong by 11 °C, and wrong in the one shape a reader here is
least likely to question.

The argument coercion above came out of that. After it, llama3.1:8b reads the
board correctly — and still loses, because it passes `ntc_beta=3950` where the
onboard part is a Murata NCU18XH103 at **B=3380** (`coaxial/scaling.py`). A
silent 1.7 °C bias from a plausible-looking constant the model supplied itself.

`gemma4:12b` overrode nothing, landed within 0.5 °C, and when asked whether the
AFE was on answered by reasoning that the NTC was *not* reading exactly 25.00.

The full entry, with the numbers, is in [FINDINGS.md](FINDINGS.md).

### Telling it was not enough

The previous entry reworded `SYSTEM` to say, plainly, never retype a table
already shown. Restarted with the new prompt loaded, `qwen2.5:14b` still ended
"tabellera alla AFE-kanaler" by writing out every channel it had just read as a
comma-separated sentence - a different shape than the markdown table the
original rule named, same act the reworded rule named directly. Three separate
bench sessions, three restatements. The prompt was not the fix; it was another
sentence the model would read and then not follow.

`Chat.ask` now keeps the channel names from the most recent successful
`analog_read` in the turn, and if the final answer names all of them and
nothing else, the answer is replaced with silence - the same move as the
link-down override two entries up: a fact the loop already has, that the
model does not get a vote on. Silence rather than a line saying so ("table
above, not restated.", an earlier version of this) because that line was its
own small version of the same complaint: the table is the trace directly
above it, on the same screen, and does not need a caption confirming it is
not being typed out again. The bar is deliberately narrow
(`RESTATE_MIN_CHANNELS = 3`, and every single one of that reading's channels
has to appear) so a real one-line finding that happens to name a channel or two
- "NTC is running warm, DCbus looks nominal" - is left alone.

Building this caught a bug in itself before it shipped: the row-matching regex
keyed on a leading digit and a word, which is also the shape of `analog`'s own
header line, `64 smp @2000Hz` - `smp` was briefly a channel of its own count.
Anchoring on the mode column (`diff`/`SE`) that only a real row has fixed it.
This is why the reproduction ran against a real render before the fix was
called done, not just against a hand-written string shaped like one.

### The dedup worked, and the transcript still looked busy

After the repeat-call dedup above, the same "tabulate everything" question still
opened with a guessed channel name (`phase_1` - not in the enum, and the schema
already says "omit for all") and closed with the model retyping every value
`analog_read` had just printed, as a comma-separated sentence, right after
being told not to.

The middle of that transcript was not a bug: `on=1 pe15=0` real, then real
again, then two calls caught by the dedup, is a model checking the AFE state
before turning it on and then losing count - one legitimate check-then-act
pair followed by two accidental repeats, exactly what the dedup exists to
catch. The bad channel guess is the model not reading its own tool schema; it
recovered in the same turn by omitting `ch` as documented, and nothing in that
recovery was wrong, just one call wasted getting there.

The closing restatement was the fixable half. `SYSTEM` said "never a markdown
table" and "no second list," and the model complied with both literally -
comma-separated prose is neither - while doing exactly the thing the rule was
for. Reworded to say what the rule actually means: never restate the rows a
tool result already printed above, table or not.

### The same call, asked again, and again

Asked to tabulate every channel, `qwen2.5:14b` turned the front end on, saw
`on=1 pe15=0`, and turned it on again - three more times, identical call,
identical result, each one a full round trip through the model and the board
for a fact it already had. Nothing was wrong with the board; the model just
could not tell it had already asked.

`Chat.ask` now remembers, per question, the (name, arguments) of every call it
has made and the result it got back. A call outside `REPEATABLE` - anything but
`analog_read`, `run_python`, `run_command` - that repeats exactly is answered
from that memory instead of reaching the board again, with a line that says so
plainly: `unchanged this turn, already asked: on=1 pe15=0`. `analog_read` stays
out of the dedup on purpose - a second reading is a new sample, not a repeat,
and the DC bus does not hold still for a cache.

The one trap in this: a repeated call after a *failed* one must not read as a
fresh success. The dedup keeps the original result under the wrapping sentence
and classifies the link from that, not from the sentence - otherwise
`unchanged this turn, already asked: ERR ConnectError: ...` would not match the
`ERR ` prefix `link is down, not answered` looks for, and a cable pulled mid-turn
would go quiet exactly where the previous entry says it must not.

### The call header, clipped, ate its own result

`_trace` used to print the call above its result: name, then arguments, then
`->`, then the first line of what came back. Fine for a short call. Asked for
five channels at once, the arguments themselves were long enough that `clip`
cut them, and the cut notice landed a newline into the middle of the header:

    analog_read samples=100 ch=['dc_bus', 'ntc', 'phase_a', 'phase_b', 'phas
    ... [17 more characters cut] -> 100 smp @2000Hz

The `->` and the first row of the actual table were now stuck onto the end of
a truncation notice, and the header restated something the table already says
better - it names every channel it read, one per row. The header is gone. What
prints now is the result, row for row, nothing above it.

### A call written as text, more than one at a time

`dbg.py` recovers a tool call the model typed into `content` instead of putting
in `tool_calls` � the shape was wrong, the intent was right, and a JSON parse is
cheaper than a wasted turn. The first version recovered exactly one call from a
message that was *nothing but* that call, and asked "vad ar temperaturen" the
model sent two:

    CallCheckFunction
    {"name": "analog_read", "arguments": {"ch": ["NTC"], "rate_hz": 100, "samples": 10}}
    CallCheckFunction
    {"name": "afe_power", "arguments": {"action": "read"}}

Nothing ran, and the prompt printed all four lines as the answer. At a bench
that does not read as a parse failure � it reads as the board having stopped
giving values, which is the wrong thing to go and check.

`_salvage_calls` now takes every call in the message, matches braces rather than
running a non-greedy regex (`{.*?}` ends at the brace that closes `arguments`,
so a nested argument parsed as half of itself), and keeps the old veto: with the
tags and the call objects removed, anything left has to be marker noise �
`tool`, `call`, `function`, `check`, split at capitals so `CallCheckFunction`
counts as three. One word of real prose and the message is printed as the answer
it probably is. Turning a sentence that happens to quote JSON into a board
command is a far worse failure than showing the JSON.

### A model that stops in prose

The runner nudges twice — "either call a tool, or call report to finish this
step" — and then records the step as `unfinished` rather than accepting the
prose as a result. An unfinished step is a visible hole in the report; a
paragraph accepted as a measurement is not.

### A turn that skipped the read entirely

Both backstops above - the link-down override and the no-restating-a-table
override - only look at tool calls made *that turn*. Measured on this bench
with `dbg.py`: the ST-Link's JTAG connector, which carries this board's VCP
(`docs/HARDWARE.md` - USART3 is bridged through the probe, not a separate
on-board USB-UART chip), was pulled mid-conversation. Asked "tabellera
ADC-värdena" again, `gemma4:12b` answered one round trip later with a full
table of plausible values - PhaseU, PhaseV, NTC, DCbus all present, each a
few counts off the real table from earlier in the same conversation. No
`analog_read` in the trace: it never touched the board that turn, just
rewrote the old numbers slightly and presented them as current.

Neither existing backstop caught this, because both are keyed off calls made
in the current turn, and this turn made none - `link_error` stays `None` with
nothing to report, and the turn-local `last_channels` used for the
restate-check stays `None` since no `analog_read` ran. SYSTEM already says
"never answer with an older reading or a guess," which is the same sentence
that did not stop the two failures above either.

`Chat` now keeps `self.last_channels` across turns, separate from the
turn-local copy. If a turn calls no `analog_read` at all and the final answer
still names every channel from the last real reading - the same
`RESTATE_MIN_CHANNELS` bar as the restate check - the answer is replaced with
`no reading taken this turn - ask again.` A turn that calls `analog_read` and
gets a channel-name or argument error still passes through undisturbed: that
model reached the board and got a real (if unhelpful) answer, which is a
different failure than never asking at all.

### afe_power fired to serve a reading, exactly what SYSTEM already forbade

SYSTEM already said "afe_power changes only when asked directly, never to
serve a reading." Measured anyway: told to turn the AFE off, then asked in
the *next*, unrelated turn for "alla analoga mätningar," `gemma4:12b` called
`afe_power(on)` first - telling it was not enough, same as everywhere else in
this file. `Toolbox._permit()` now refuses an `afe_power` call that changes
state when the current question never mentioned "afe" (`afe_mentioned`,
`tools.py`) - set from the real question text at `dbg.py`'s two real call
sites (`repl()`, the one-shot path), not inside `Chat.ask()` itself, so every
existing test driving `Chat.ask()` directly keeps its old permissive default
instead of needing "afe" stuffed into an unrelated fixture question.

### A declined --confirm was reported as a success

`build_firmware`/`run_command` calls are gated by `--confirm`. Measured live:
told to build and flash, then declined at the confirm prompt, `gemma4:12b`
still answered "kortet har byggts och flashats" - the refusal was right there
in its own tool result and it wrote past it. `Chat.ask()` now tracks
`code_error` the same way it already tracked `link_error`: any
`run_python`/`run_command`/`build_firmware` result starting `ERR` this turn
overrides the model's own closing line, cleared by a later successful call in
the same turn.

### A tool existing in the schema is not the same as being reached for

Three separate instances of the same shape, all fixed the same way - a short,
conditional line appended to `SYSTEM` only when the relevant tool is actually
offered, since telling it once in a long system prompt was not enough and
paying for the line on every turn regardless was not necessary either:

  * `build_firmware` sitting in the tool list did not stop "bygger du
    firmware?" from getting "Nej, jag programmerar inte firmwaren själv" -
    the model's own training that a chat assistant cannot compile real
    hardware overrode the schema outright. `BUILD_FIRMWARE_HINT`.
  * `run_command`'s first two tries at reaching `build_and_flash.py` were
    `python3` (not allowlisted) and the wrong directory. `BUILD_HINT`.
  * Asked directly "why can't you reach the board" with the link genuinely
    down, `gemma4:12b` called `build_firmware` instead of `link_diagnose` -
    a guess at a fix, not a diagnosis, and not what was asked.
    `LINK_DIAGNOSE_HINT`.
