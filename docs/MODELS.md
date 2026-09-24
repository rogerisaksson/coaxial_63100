# Models

The local model with the board's tools: `host/coaxial_ollama/`,
`host/board_chat.ps1` (+ `board_chat/`), `host/dbg.py`, tools in
`host/coaxial_mcp/`. Its measured failure modes are in FINDINGS.

## Picking and pulling

### Picker

`python -m coaxial_ollama.capability`: VRAM minus reserve, free RAM,
cores -> tag. `choose(prefer='speed')` takes the largest tag that fits VRAM
whole (a hybrid split measured ~5x slower). CPU ceiling 8 GB; a 12B on 32
cores ran 6.4 tok/s.

| Tag | GB | Note |
| --- | --- | --- |
| llama3.1:8b | 4.9 | invents tool arguments |
| qwen2.5:7b | 4.7 | |
| gemma4:12b | 7.8 | default here |
| qwen2.5:14b | 9.7 | 12 GB card |
| qwen2.5:32b | 20.0 | |
| llama3.3:70b | 42.0 | needs 64 GB RAM |

### Reserve and pull

Reserve = max(quarter of card, 2 GB, used + 2 GB); override
`COAXIAL_VRAM_RESERVE_GB` / `board_chat -Reserve`. Windows' `AdapterRAM` is
32-bit: read `qwMemorySize`. `python -m coaxial_ollama.pull TAG` is the one
pull (daemon stream drawn as a bar); every entry point pulls a missing tag.

## Daemon and client

### Tuning (`board_chat/Tuning.ps1`)

| Variable | Value | Why |
| --- | --- | --- |
| LLAMA_ARG_CACHE_RAM | 0 | prompt cache ~343 MiB a question |
| LLAMA_ARG_CTX_CHECKPOINTS | 0 | restoring one threw `std::bad_alloc` |
| OLLAMA_MAX_LOADED_MODELS | 1 | two copies of weights: `cudaMalloc failed` |
| OLLAMA_NUM_PARALLEL | 1 | the same |

A failed preload is reported in the daemon's words (`llama-server binary
not found` = broken install: rerun `setup.ps1`).

### Client (`client.py`)

Temperature 0, seed 7, `num_ctx` 8192, keep_alive 30m (REPL) / 2m (one
question). OOM ladder: free other models, flush, halve context to 2048,
raise. `release()` sends keep_alive 0. Never a second client at another
`num_ctx` (reloads 7.6 GB).

## Prompt loop (`dbg.py`)

`coaxial_ollama/debug.py` `Chat` = `budget` (the prompt inside the window)
+ `turn` (one question, its rounds and calls) + `commands` (the / lines);
the fixed words are `words.py`.

### SYSTEM and tools

~70-token SYSTEM prompt; every line answers a measured failure
(`test_ollama_prompt.py`). Hints join only with their tools. Tool sets
(`SETS`): read, code, pins, build, docs, all, none; re-sent every turn
(runner ~1390 tokens, read ~560, none ~110). A non-`REPEATABLE` tool
repeated in a turn is refused. `run_command` = argv allowlist
(`sandbox.Shell`); `run_python` = persistent `sandbox.Scope`. Output
clipped at 4000 chars.

### Slash commands and prose orders

`/py`, `/sh` (no model), `/model TAG|auto` (releases VRAM first),
`/board simulated|auto|rs485|COMx`, `/node N` (0 = broadcast), `/tools`,
`/detail`, `/confirm`, `/lang`, `/ctx`, `/clear`, `/history`, `/cost`,
`/help`, `/q`. A sentence asking to switch boards is carried out by
`board_switch()` for no tokens (`BOARD_WORDS`, `_BOARD_VERBS`).

### Intent, language, replies, context

- `intent.py` classifies a sentence on the turn's own client (map, read,
  power, devices, link, words, control, orient); 11/12 on gemma4:12b.
  Thinking off (it spent the budget and returned nothing).
- `language.py` locks the session to the question's language; only the
  host releases it. Stdin/stdout re-coded (cp1252 split `läge`).
- `replies.py`: catches a retyped channel table, tool calls written as
  text, marker noise, a tool named instead of called.
- `context.py`: prompt <= 0.7 of `num_ctx`, old tool results stubbed to 80
  chars. `detail.py`: terse under 30 B parameters.

## Runner and MCP

- `runner.py` drives `coaxial_ollama/plans/bringup.yaml`: tasks with limits
  judged in Python; a `report` tool called once, last, with a board tool behind
  it.
- MCP (`python -m coaxial_mcp --port COMx`, stdio): board_info,
  analog_read, docs, self_test, imu, angle, orientation, afe_power,
  devices, digital_read, gpio_pin, gpio_port, test_gate, thermal, link.
  `coaxial_ollama.tools` adds run_python, run_command, build_firmware,
  run_tests, link_diagnose, report. `docs` = index / section (clip 4000,
  1200 terse) / find (12 hits with chapter).
- `run_tests.py -m auto` asks `pick_tests.py` for the model's pick, held
  inside the tier; the path map settles cheap changes without the model.
