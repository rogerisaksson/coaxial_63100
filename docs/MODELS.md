# Models

The local model on this machine has the board's tools wired to it. This
is how it is chosen, loaded, prompted and held to what the board said.
The code is `host/coaxial_ollama/`, `host/board_chat.ps1` with
`board_chat/` beside it, `host/dbg.py`, and the tools in
`host/coaxial_mcp/`.

## The picker

`python -m coaxial_ollama.capability` measures the machine and the
tag follows: VRAM minus a reserve, free RAM, and cores. `board_chat`
and `dbg.py -m auto` both use it and pull the tag if it is absent.

### The catalogue

`CATALOGUE` in `capability.py`, resident size at Q4_K_M:

| Tag | GB | Layers | RAM | Note |
|---|---|---|---|---|
| llama3.1:8b | 4.9 | 32 | 8 | small and quick; measured inventing tool arguments |
| qwen2.5:7b | 4.7 | 28 | 8 | the small one to try when llama3.1 disappoints |
| gemma4:12b | 7.8 | 48 | 16 | the default on this bench |
| qwen2.5:14b | 9.7 | 48 | 16 | the balanced one on a 12 GB card |
| qwen2.5:32b | 20.0 | 64 | 32 | |
| llama3.3:70b | 42.0 | 80 | 64 | only with 64 GB of system RAM behind it |

`choose(machine, prefer='speed')` takes the largest tag that fits VRAM
whole; `prefer='capability'` allows a hybrid split, which measured
about five times slower per token. Nothing fitting whole means a split
or the CPU; `CPU_CEILING_GB` is 8 and a 12B model with nothing on the
GPU managed 6.4 tok/s on 32 cores here. Free RAM counts, not installed
RAM. Every tok/s figure was measured on an idle machine, and the report
says so when the CPU is busy.

### The reserve

`reserve_for(vram, used)` is the largest of a quarter of the card,
2 GB, or what is already used plus `HEADROOM_GB` = 2. The desktop alone
was using 2.6 GB on the reference bench at 0 % utilisation, so a flat
quarter of that card left 1.4 GB of slack. `COAXIAL_VRAM_RESERVE_GB`
overrides it, as does `board_chat -Reserve 8` (`--reserve-gb` one layer
down): raise it if the screens stutter while the model answers. Windows
reports `AdapterRAM` in a 32-bit field - 4 GB for every card larger
than that - so the picker reads `qwMemorySize` (measured on a 16 GB
card: 16.0 against 4.0).

### Threads

`board_chat.ps1` lowers the ollama processes' priority while a question
is prepared and answered; that helps the desktop stay responsive and
does nothing about the card being busy - the other half is the reserve.
A machine under 8 cores asked to run a CPU-only model is warned that
every question will be slow enough to notice. A hybrid split hands back
half the weights for about a fifth of the speed.

## The daemon

### Tuning

`board_chat/Tuning.ps1` starts the daemon with four variables, each
measured on this machine:

| Variable | Value | Why |
|---|---|---|
| LLAMA_ARG_CACHE_RAM | 0 | llama-server keeps a prompt cache of up to 8 GiB in host memory; every question here starts a fresh conversation. Measured: `prompt_save: saving prompt with length 1446, total state size = 342.623 MiB` |
| LLAMA_ARG_CTX_CHECKPOINTS | 0 | restoring a 311.575 MiB checkpoint threw `std::bad_alloc` and took the runner with it; capping at 2 was not enough |
| OLLAMA_MAX_LOADED_MODELS | 1 | two copies of the weights on a 16 GB card: a 500 from the daemon, `cudaMalloc failed` |
| OLLAMA_NUM_PARALLEL | 1 | the same |

Measured end to end: ten questions, twenty-seven model calls, zero
`std::bad_alloc`, one model load. A daemon started before the settings
were written is restarted; changing CTX_CHECKPOINTS from 2 to 0 left a
daemon reported as already tuned until both lines were checked.

### The client

`client.py`: temperature 0, seed 7, `num_ctx` 8192, `keep_alive` 30m
in the REPL and 2m for one question, local hosts only unless
`remote_ok`. A crashed runner is retried twice, 1.5 s apart, in
silence. Out of memory runs a ladder: free every other resident model,
then flush, then halve the context down to `MIN_NUM_CTX` = 2048, then
raise. `release()` hands VRAM back with `keep_alive` 0: a session left
running held 9.69 GB for another 27 minutes at 1 % utilisation. Two
clients at different `num_ctx` evict each other and reload 7.6 GB per
question, which is why a typed sentence is classified on the turn's own
client and never a second `Ollama`.

## The prompt loop

`dbg.py` (`coaxial_ollama/debug.py`) is a prompt loop for a question
asked sixty times an afternoon: about 70 tokens of SYSTEM prompt, five
tools by default, `num_predict` capped, `think` off where supported,
old turns stubbed rather than resent, and slash commands that cost no
model at all.

### The SYSTEM prompt

Every line in `SYSTEM` replaced a measured failure and the tests in
`test_ollama_prompt.py` hold each one in place. It names the board and
what a reading means; it does not mention `docs`, FINDINGS or
mid-scale. Hints join only with the tools they are about: `DOCS_HINT`
("values come from analog_read, never docs") when `docs` is offered,
`BUILD_HINT` with the exact `run_command` line when `run_command` is
offered, and a flash hint when `build_firmware` is. `ROLE` is what
`/help` opens with.

### Tool sets

`SETS`: `read` (board_info, devices, analog_read, digital_read, imu,
angle, orientation, self_test, afe_power, link, link_diagnose), `code`
(read plus run_python, build_firmware, run_tests), `pins`
(board_info, devices, digital_read, gpio_pin, gpio_port, test_gate,
afe_power, link_diagnose), `build` (board_info, run_command, run_tests,
link_diagnose), `docs` (board_info, analog_read, docs, link_diagnose),
`all`, `none`. The list is re-sent every turn, so it is the cost that
scales. Per turn: the runner about 1390 tokens, `code` about 640,
`read` about 560, `none` about 110.

`REPEATABLE` = analog_read, run_python, run_command: the same call
twice means something there; any other tool repeated in one turn is
refused - qwen2.5:14b turned the AFE on four times in a row.
`WRITE_CALLS` (the pin writes, run_python, run_command) ask first under
`/confirm`. `run_command` runs through `sandbox.Shell`, an allowlist by
argv with no shell characters; `run_python` runs in `sandbox.Scope`, a
persistent namespace holding the session. Tool output is clipped at
4000 characters, head and tail.

### Slash commands

`/py CODE` and `/sh CMD` run without the model. `/reconnect` reopens
the link. `/model TAG|auto` swaps the model and hands VRAM back first.
`/board simulated|auto|rs485|COM4` swaps what the tools talk to.
`/node N` picks a node on the bus, 0 is broadcast, bare lists them.
`/tools SET`, `/detail terse|full|auto`, `/confirm`, `/lang NAME|auto`,
`/ctx` (what the next turn costs), `/clear`, `/history`,
`/clear_history`, `/cost`, `/help`, `/q`.

Prose does the same: "byt till debugproben" is carried out by
`board_switch()` for no tokens. `BOARD_WORDS` maps what an operator
calls a board to `simulated`, `auto` or `rs485`; an order needs one of
`_BOARD_VERBS` and is disqualified by a closed set of words - a list of
allowed filler words abstained on every noun nobody had thought of,
measured four times ('enhet', 'hardvara', 'lage').

### Intent

`intent.py` classifies a typed sentence before it is answered, on the
turn's own client: seven intents - map, read, power, devices, link,
words, control, orient - and a kind (analog, digital, imu, angle,
subsystems, parts, both, none). `map`, `power`, `devices`, `link` and
`orient` each name one tool; `words` and `control` name nothing, on
purpose - naming a tool for them is how a request for a description
became a channel table. Measured against gemma4:12b over 12 questions
at about 2.75 s each: 11 of 12 once `link` was narrowed to "the link
is failing" and `devices` said "start talking to one by name"; the
twelfth is a control sentence the host carries out before anything
reaches the classifier. Thinking is off for it: on `pick_tests.py` the
model spent the whole `num_predict` budget reasoning and returned empty
content.

### Language

`language.py` detects the script and then the stopwords, and locks the
session to the language of the question. Measured with qwen2.5:14b,
answers came back in Chinese, Japanese and Thai to questions in none
of them. The stopword lists repeat words on purpose: "ger du mig en
tabell over de analoga matvardena?" scored one Swedish word against two
Dutch before that. An English answer scored Portuguese on two 'a's
before the margin. A lock is released only by the host: locked to
Korean and asked to switch back, the model obeyed the lock line and
refused, in Korean. `PHRASES` carries the host's own lines in Swedish;
"Jag har andrat spraket till svenska. Hur kan jag hjalpa dig..." was
two sentences where one word does. Stdin and both outputs are re-coded:
`byter du till simulerat lage` arrived as `simulerat lÃ¤ge` under
cp1252 and split into `lã` and `ge`, and `ge` disqualifies an order.

### Replies

`replies.py` is the backstop between what the tool said and what the
model wrote. `is_retype` catches a channel table typed back with
different numbers (from three channels up); `salvage_calls` recovers a
tool call written as text; `is_marker_noise` drops a reply that is
only `tool_call` markers; `NAMED_TOOL` catches a tool named in prose
instead of called. `Chat.last_channels` keeps the names from the last
successful `analog_read` so a later turn with no call is still checked.

### Context

`context.py`: the prompt may use `CTX_SHARE` = 0.7 of `num_ctx`, never
under `MIN_PROMPT_TOKENS` = 512; older tool results are stubbed to
`STUB_CHARS` = 80 rather than dropped. `/ctx` prints what the next turn
costs. `detail.py` sizes tool descriptions: `auto` is terse for tags
under `FULL_MODEL_B` = 30 billion and full above.

## The runner

`runner.py` drives a written test plan (`plans/bringup.yaml`,
`plan.py`): `Task`s with a limit each, `DEFAULT_TURNS` = 12, and a
`report` tool the step must call exactly once, last. The limit is
judged in Python, not by the model. A report with no board tool behind
it is refused like a prose stop - nudged twice, then unfinished. The
transcript is written under `data/`.

## The MCP server

`python -m coaxial_mcp --port COM4` over stdio, fourteen tools:
board_info, analog_read, docs, self_test, imu, angle, orientation,
afe_power, devices, digital_read, gpio_pin, gpio_port, test_gate,
link. `coaxial_ollama.tools` adds run_python, run_command,
build_firmware, run_tests, link_diagnose and report. `analog_read`
returns codes with the AFE either way under an unmistakable line
(invariant 9); the cooked readings refuse. `docs` reads the seven
documents by heading: `docs()` is the index, `docs(doc=, section=)` one
section clipped at `CLIP` = 4000 characters (1200 terse),
`docs(find=)` up to `FIND_HITS` = 12 lines (6 terse) with their chapter
and entry. The index shows at most `INDEX_HEADS` = 12 headings a
document, because FINDINGS grows without bound and the index is read on
every turn about the documents. `coerce` accepts `ch` as a bare string
and numbers as strings, measured with llama3.1:8b.

## Measured failure modes

The wording history behind the SYSTEM prompt and the hints, each one a
transcript:

* Asked for raw codes with the AFE deliberately off, a model wrote
  "Mid-scale ... 25.00 C" out of the warning text itself. Refusing was
  worse; the line now labels.
* A local model filled the gap in *coaxial* and reported a coaxial
  cable or connector - twice. The fitted parts come from `0x6D` kind 4.
* `analog_read` was called with `ch=['phA']`, a name nothing on the
  board carries; the tool answers with the names it has.
* Asked to *measure* the channels, gemma4:12b read HARDWARE.md and
  answered with that document's channel table - no measurement in it.
* gemma4:12b answered "Nej, jag programmerar inte firmwaren sjalv" with
  `build_firmware` in its own tool list, never called.
* The first two tries at a build were `python3` (not allowlisted) and
  `python build_and_flash.py`, one directory short of `tools/`.
* "If it has not already been called this turn, call it before
  answering" read as a standing order: `link_diagnose` ran first on
  three questions that were not about the link.
* Asked "byt till en simulerad hardvara", gemma4:12b answered that it
  could not switch hardware; the host now carries the order out.
* Asked to change the board, a model described, refused, and then read
  a channel - three times.
* "kommunicera med vänster knä" was sent as `name='right knee'` in the
  call and translated right in the prose after it.
* BUS_VOLT and A0, both invented channel names, both refused where one
  of them meant the DC link.
* llama3.1:8b invented tool arguments; a smaller model sent `ch` as a
  bare string and numbers as strings.
* qwen2.5:14b answered in Chinese, Japanese and Thai; a session locked
  to Korean refused, in Korean, to leave.
* On design questions the model substitutes plausible hardware
  constants. Those questions are not its.
* A planned turn calls no `trim()`, so a Swedish question answered from
  a compiled plan left the lock on the previous language; a nudge's
  English words flipped a Swedish session on the next trim.
* On the 25 % tier the model's pick put `live:all` back: 398 s, 352 of
  them that suite. The tier now clamps and says what it refused.
* Two `dbg.py` sessions had COM4 open; every probe read silent, and
  the board was diagnosed as halted, started over SWD and reflashed.
  None of that was the matter with it.
* Killed from outside, 8.4 GB stayed on the card until released by
  hand; Ctrl+C is `STOPPED`, exit 130, and the `finally` releases.

## Test picking

`run_tests.py -m auto` resolves the tag once through `choose(probe())`
(a runner with no daemon falls back to the roster's first tag), asks
`pick_tests.py` for the model's own pick, and holds it inside the tier
in force. Structure is never the model's to drop. The model is not
asked when the path map already knows: every changed file on an
explicit rule with a `CHEAP` answer settles without a 7.6 GB load.
`python tools/pick_tests.py --explain` says which subjects and why.
