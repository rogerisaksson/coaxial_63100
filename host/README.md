# Host software for coaxial_63100

    host/
      coaxial/          the library; Coaxial63100 (rig.py) is the front door
      coaxial_mcp/      MCP server: the board as fourteen tools over stdio
      coaxial_ollama/   the local-model runner, and what dbg.py drives
      board_chat.ps1    preflight + prompt loop; board_chat/ holds its parts
      dbg.py            the prompt loop one layer down: /py and /sh cost no tokens
      testline/         production line: plans, instruments, and the limits
      examples/         read_board.py (measure, judge nothing),
                        pytest_production_line.py (where limits belong)
      motors/           the profiles the drive loads (outrunner_14p.json)
      tests/            twenty-three suites; run_tests.ps1 is the interface
      tools/            run_tests.py, pick_tests.py, the views, pulse, switch
      data/             measurement logs and run transcripts

## Quick start

```python
from coaxial import connect, disconnect

boards = connect([1])                  # unit ids; (unit, baud) or (unit, baud, port) per entry
for board in boards:
    board.afe.enable()                 # powers the ADC reference: off, every channel is mid-scale
    print(board.analog.ntc_temperature(), board.analog.dcbus_voltage())
disconnect(boards)
```

`Coaxial63100` is the front door for one board - the AFE preflight, the
supply put back as found, the subsystems by name (`.daq`, `.imu`, `.angle`,
`.thermal`, `.gates`, `.drive`). Its example is in [../README.md](../README.md);
the notebooks are `../python_examples/`. From a shell:

    python -m coaxial all
    python -m coaxial temp
    python -m coaxial pins E

The layering - subsystems over protocol over transport over codecs - is in
[../docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md).

## Two rules the library keeps

**Nothing returns a status.** Every call produces its result or raises from
`coaxial.errors`. `connect()` has no partial success: a caller holding the
list knows every board in it answered.

**Scaling comes from the board.** Every conversion lives in the calibration
record (`0x6E` device 3, invariant 7 in [../CLAUDE.md](../CLAUDE.md));
`board.analog.scaling()` reads it and the cooked reads use it. `NtcParams`
and `DividerParams` (`scaling.py`) override it for a fixture with a
calibrated meter - an argument, not new firmware.

## Tests

    .\run_tests.ps1                 # ~25 % of the 2111 checks, the default
    .\run_tests.ps1 -All            # the gate
    .\run_tests.ps1 -Structure      # does host/ still hold together - 5 s
    python examples/read_board.py   # the board, read end to end

The suites and their sizes are listed in [../CLAUDE.md](../CLAUDE.md#commands).
A missing cable is not a failing suite: every one opens through
`open_session()` and falls back to the stand-in.

## MCP server

    python -m coaxial_mcp --port COM4        # stdio JSON-RPC
    python tests/test_mcp.py                 # 44 checks, drives it over stdio

`../.mcp.json` registers it for this workspace. Fourteen tools, not one per
firmware command, because the whole tool list is re-read on every turn:
`board_info`, `docs`, `self_test`, `analog_read`, `afe_power`, `devices`,
`digital_read`, `imu`, `angle`, `orientation`, `gpio_pin`, `gpio_port`,
`test_gate`, `link`.

Results are dense fixed-column text rather than JSON. Measured on the
seven-tool server, the same seven-channel reading:

    compact text (this server)            278 chars   ~69 tokens
    JSON, indented, full key names       2457 chars  ~614 tokens
    8.8x

That tool list was ~560 tokens; a pin read cost 1 token of result, a
whole-board analog sweep 69. `board_info` returns the channel map once and
readings refer to short names afterwards. Errors are one line carrying the
way out:

    ERR DeviceStateError: the analog front end is off ... -> afe_power(action=on)
    ERR ValueError: PB10 is USART3_TX and is refused in every mode

## The local model

`board_chat` is the way in ([../README.md](../README.md), *Ask the board*).
Underneath, `coaxial_ollama` hands the board's tools to a model under
Ollama, with a Python scope holding the live `board` and an allowlisted
shell:

    python -m coaxial_ollama --plan coaxial_ollama/plans/bringup.yaml
    python -m coaxial_ollama --ask "what does the NTC read right now?"
    python tests/test_ollama_tools.py    # 218 checks: no board, no ollama

The model measures; it is never told the limit and never asked for a
verdict. `Limit` (`testline/plan.py`, re-exported by `coaxial_ollama.plan`)
judges in Python from a file under revision control, so a step's result is
traceable to the plan rather than to a sampling temperature.

Defaults are read-plus-code: board reads, `run_python`, and programs from
`--allow`. Pin writes and the test gate need `--allow-writes`; `--confirm`
asks before every side effect; `--read-only` removes code and commands.
Every message, tool call and result lands in a JSONL transcript in `data/`.

## dbg.py: the cheap loop

The same board and tools with the cost turned down, for the questions asked
sixty times an afternoon:

    python dbg.py "the NTC reads exactly 25.00 - what is wrong?"
    python dbg.py -q "which channel is the DC link?"       # answer only
    python dbg.py -m auto -q "read the NTC"                # the model this machine runs
    python dbg.py --repl                                   # prompt loop
    python dbg.py --no-board --file ../Core/Src/main.c "what configures ADC3?"

Where the tokens went, measured on this tree:

| | tokens per turn, before the question |
|---|---|
| the plan runner: 350-token prompt, 11 tools | ~1390 |
| `dbg`, default `--tools code` | ~640 |
| `dbg --tools read` | ~560 |
| `dbg --tools none` | ~110 |

Five choices, no magic: a seventy-token system prompt, a tool subset
(`--tools read|code|pins|build|docs|all|none`), `--words` capping the
answer, `think` off where the model supports it, and old tool results
stubbed to their first line. Cost is tracked, not printed: `/cost` and
`/ctx` show it, `--budget N` stops the session there.

The commands that cost nothing:

    Coaxial 63100> /py round(board.analog.ntc_temperature()["celsius"], 2)
    Coaxial 63100> /sh cube-cmake --build --preset Debug
    Coaxial 63100> /board simulated      # or auto, rs485, COM4 - no tokens
    Coaxial 63100> /model auto           # swap the model; hands VRAM back first
    Coaxial 63100> /node RL 2            # which node on the bus; 0 is broadcast
    Coaxial 63100> /tools read           # reprice the turn
    Coaxial 63100> /clear                # the cheapest command there is

`/py` and `/sh` work with ollama not running. Which model this machine
runs, and why: [../docs/MODELS.md](../docs/MODELS.md).
