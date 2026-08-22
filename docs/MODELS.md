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

`host/coaxial_ollama/capability.py` measures the machine and picks. Ask it:

```powershell
python -m coaxial_ollama.capability                     # what this machine gets
python -m coaxial_ollama.capability --prefer capability
dbg -m auto "what is the board temperature?"            # same choice, from the prompt
```

The rule is **the largest tools-capable model that fits the graphics card
whole**, keeping a quarter of the VRAM back (floor 2 GB) so the desktop still
has somewhere to live. Only tools-capable tags are candidates: everything here
reaches the board through tool calls, and a tag without them describes a
measurement instead of taking one.

`setup.ps1` pulls whatever the picker chooses, and so does `board-prompt.ps1` on
a machine where the tag is missing — it asks the picker, pulls, loads and only
then opens the prompt. `-Model TAG` overrules it everywhere.

Anything driving this from outside — including Claude Code, see the routing
table in [../CLAUDE.md](../CLAUDE.md) — should reach for `board-prompt -Ask` or
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

`board-prompt -Reserve N` for one run; `COAXIAL_VRAM_RESERVE_GB` in the
environment for a machine, once, honoured by every entry point. On a card of
this size the step from 14B to 12B costs little — both were within half a degree
of the board's own NTC reading, and 12B is the tag that checks the AFE before it
answers.

### Priority

`board-prompt.ps1` drops the ollama processes to BelowNormal unless `-Normal`
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

### Keeping the model loaded

`keep_alive` goes on **every** request, default `30m`. Ollama caches the KV
state of a prefix it has already processed — that is what makes turn nine as
quick as turn two — and throws it away when the model unloads, five minutes
after the last request by default. A bench session is mostly gaps: you read a
number, move a probe, think. `--keep-alive 0` hands the VRAM straight back on a
shared machine.

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

### A model that stops in prose

The runner nudges twice — "either call a tool, or call report to finish this
step" — and then records the step as `unfinished` rather than accepting the
prose as a result. An unfinished step is a visible hole in the report; a
paragraph accepted as a measurement is not.
