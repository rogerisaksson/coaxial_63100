# The Model in the Loop

Local LLM orchestration for hardware telemetry. Designed around VRAM constraints, deterministic host overrides, and aggressive prompt optimization.

## VRAM & Lifecycle Management

* **Dynamic Tag Selection:** `capability.py` loads the largest tools-capable model that fits in available VRAM, explicitly reserving space for the OS (existing footprint + 2 GB) to prevent UI hangs.
* **OOM Fixes:** `llama-server` crashes (`std::bad_alloc`) are caused by its internal prompt cache, not context length. Fixed by forcing `LLAMA_ARG_CACHE_RAM=0` and `LLAMA_ARG_CTX_CHECKPOINTS=0` in the daemon.
* **Eviction Ladder:** When VRAM is exhausted, the host unloads inactive models, halves `num_ctx`, or aborts to a smaller tag.
* **One Load Per Batch:** Loaded once and unloaded once around a whole run - `run_tests.py` for the suites, `board_prompt.ps1` for a prompt or a list of `-Ask` questions. Neither end is free: unloading per question put most of a run's wall time back into reloading 7.6 GB, and a one-shot that never unloaded pinned 8.4 GB for the full 30-minute keep-alive. `-Hold` opts out.

## Execution Pipeline

* **Two-Pass Routing:** Queries undergo a cheap intent classification pass before execution. The *noun* strictly decides the tool (e.g., "list" = map, "values" = read) to prevent multi-tool hallucinations. It runs on the turn's own client - a second `Ollama` at a different `num_ctx` reloads the weights every question.
* **Off-axis redirect:** Naming the right tool in the prompt does not hold. When the model calls the other half of the map/read axis, the loop answers that call itself - `not this question - analog_read answers it` - costing no round trip. Measured: the question asked *first* called `board_info`; asked after the map question it called `analog_read`, so a suite of question pairs never saw it.
* **Language Control:** The host dictates the output language based on OS locale or explicit command. The model is forbidden from auto-detecting language, preventing drift. Bare language switches are intercepted and answered by the host at zero token cost.
* **Context Scaling:** Detail levels (`terse` vs. `full`) for tool descriptions are determined by model parameter count (<30B gets terse), not user flags.

## Tooling & Schemas

* **No JSON Mode:** `format='json'` is disabled. It causes models to narrate tool invocations in the `content` field rather than executing them.
* **Host-Side Coercion:** Smaller models hallucinate argument structures. `coerce` aggressively casts arguments to the exact `inputSchema` types and resolves string aliases.
* **Salvage Operations:** `replies.salvage_calls` recovers structurally valid tool JSON mistakenly emitted as raw text.
* **Call Deduplication:** Identical tool calls within a single turn are answered from host memory to prevent redundant serial bus traffic.

## Validation & Guardrails

* **Negative Prompting Fails:** Models routinely ignore negative instructions ("never restate a table"). Rules are instead enforced by host-side Python (e.g., `is_retype` silently deletes tables hallucinated as prose).
* **Schema Bloat Costs:** Unnecessary verbosity in tool descriptions directly crowds out tool selection accuracy. Schemas are severely truncated.
* **Deterministic Testing:** `test_live_model.py` validates tool-call *sequences* at `temperature 0`. Correct prose following an incorrect tool call is scored as a failure.