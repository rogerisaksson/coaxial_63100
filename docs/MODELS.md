# The Model in the Loop

Local LLM orchestration for hardware telemetry and user support, built around
VRAM limits, deterministic host overrides and a hard prompt budget.

## VRAM and lifecycle

* **Tag selection:** `capability.py` loads the largest tools-capable model that
  fits, reserving the OS's existing footprint + 2 GB so the UI cannot hang.
* **OOM:** `llama-server`'s `std::bad_alloc` comes from its prompt cache, not
  context length. Fixed with `LLAMA_ARG_CACHE_RAM=0` and
  `LLAMA_ARG_CTX_CHECKPOINTS=0` in the daemon.
* **Eviction ladder:** out of VRAM, the host unloads inactive models, halves
  `num_ctx`, then aborts to a smaller tag.
* **One load per batch** - `run_tests.py` for the suites, `board_prompt.ps1` for
  a prompt or a list of `-Ask` questions. Neither end is free: unloading per
  question put most of a run's wall time into reloading 7.6 GB, and a one-shot
  that never unloaded pinned 8.4 GB for the full 30-minute keep-alive. `-Hold`
  opts out.

## Execution pipeline

* **Two-pass routing:** a cheap intent classification runs before execution, and
  the *noun* decides the tool ("list" = map, "values" = read), which is what
  stops multi-tool hallucinations. It runs on the turn's own client - a second
  `Ollama` at a different `num_ctx` reloads the weights every question.
* **Off-axis redirect:** naming the right tool in the prompt does not hold. When
  the model calls the other half of the map/read axis, the loop answers that
  call itself - `not this question - analog_read answers it` - for no round
  trip. Measured: asked *first* the question called `board_info`; asked after
  the map question it called `analog_read`, so a suite of question pairs never
  saw it.
* **Language:** the host dictates the output language from OS locale or an
  explicit command; the model is forbidden to auto-detect, which is where it
  drifts. Bare language switches are answered by the host at zero token cost.
* **Context scaling:** `terse` vs `full` tool descriptions follow the model's
  parameter count (<30B gets terse), not a user flag.

## Tooling and schemas

* **No JSON mode.** `format='json'` makes models narrate a tool invocation in
  `content` instead of making it.
* **Host-side coercion:** smaller models hallucinate argument structures, so
  `coerce` casts to the exact `inputSchema` types and resolves string aliases.
* **Salvage:** `replies.salvage_calls` recovers structurally valid tool JSON
  emitted as raw text.
* **Deduplication:** identical tool calls within a turn are answered from host
  memory rather than repeated on the serial bus.

## Guardrails

* **Negative prompting fails.** Models ignore "never restate a table", so the
  rule is enforced in Python instead - `is_retype` deletes tables hallucinated
  as prose.
* **Schema bloat costs accuracy.** Verbose tool descriptions crowd out tool
  selection, so schemas are cut hard.
* **Deterministic testing.** `test_live_model.py` validates tool-call
  *sequences* at temperature 0. Correct prose after an incorrect tool call is a
  failure.
