# Host software for coaxial_63100

    host/
      coaxial/        the library
      coaxial_mcp/    MCP server: the board as eight tools
      coaxial_ollama/ test runner driven by a local model
      testline/       production line: plans, instruments, PDF reports
      examples/       read_board.py (measure, judge nothing)
                  pytest_production_line.py (where limits belong)
      tests/          Modbus conformance, MCP, and the runner offline
      tools/          one-off analysis scripts
      data/           measurement logs and run transcripts

## Quick start

```python
from coaxial import connect, disconnect

boards = connect([(1, 115200)])          # (unit id, bitrate)

for board in boards:
    print(board.link.echo('Hello slave!'))
    board.afe.enable()
    print(board.analog.ntc_temperature())
    print(board.analog.dcbus_voltage())

disconnect(boards)
```

Or from a shell:

    python -m coaxial all
    python -m coaxial temp
    python -m coaxial pins E

## The shape of it

| Layer | Module | Knows about |
|---|---|---|
| `Board` + subsystems | `board.py`, `analog.py`, `gpio.py`, `afe.py`, `link.py`, `system.py` | what the hardware can do |
| protocol | `protocol.py` | command codes, versioning rules |
| transport | `transport.py` | Modbus RTU framing, serial |
| codecs | `wire.py`, `crc.py` | payload layout, checksum |
| scaling | `scaling.py` | dividers and thermistors |

One subsystem per functional area of the board, so a line of a test script says
which part of the hardware it touches without naming a function code:

    board.system    identity, versions, clock tree
    board.link      echo, frame counters
    board.afe       the front end switch, which also powers the reference
    board.analog    channels, bursts, temperature, DC bus
    board.gpio      raw pin access for a fixture, behind a gate

## Two rules the library keeps

**Nothing returns a status.** Every call produces its result or raises from
`coaxial.errors`. `connect()` has no partial success: a caller holding the list
knows every board in it answered.

**Scaling belongs to the host.** The firmware reports raw ADC codes. `NtcParams`
and `DividerParams` carry the fixture's knowledge of what they mean, so a board
with a different divider or thermistor needs new arguments, not new firmware.

    board.analog.ntc_temperature(ntc_params=NtcParams(r25=10000, beta=3950))
    board.analog.dcbus_voltage(divider=DividerParams(vref=3.287, offset_v=-0.05))

## Running the tests

    cd host
    python tests/test_conformance.py      # 40 checks against the live board
    python examples/read_board.py

## MCP server

For driving the board from a model, with the token budget as the design
constraint:

    python -m coaxial_mcp --port COM4        # stdio JSON-RPC

Registered for this workspace in `../.mcp.json`. Claude Code asks before
starting a project MCP server the first time.

Seven tools, not one per firmware command, because the whole tool list is
re-read on every turn:

| Tool | Does |
|---|---|
| `board_info` | identity, versions, clock, ADC channel map. Call once. |
| `analog_read` | sample channels; degC for the NTC, volts for the DC bus |
| `afe_power` | the front end switch, which also powers the reference |
| `gpio_pin` | read, drive or configure one pin |
| `gpio_port` | read a port, or drive a masked set atomically |
| `test_gate` | open or close raw pin access |
| `link` | echo, frame counters, release the console |

Results are dense fixed-column text rather than JSON. Measured on the same
seven-channel reading:

    compact text (this server)            278 chars   ~69 tokens
    JSON, indented, full key names       2457 chars  ~614 tokens
    8.8x

The tool list itself is ~560 tokens. A pin read costs 1 token of result, a
whole-board analog sweep 69. The channel map is returned by `board_info` once
and referred to by short name afterwards, instead of being repeated inside
every reading.

Errors are one line carrying the way out:

    ERR DeviceStateError: the analog front end is off ... -> afe_power(action=on)
    ERR ValueError: PB10 is USART3_TX and is refused in every mode
    ERR ValueError: unknown channel 'Vbat'; names are ch3,ch6,dcbus,ntc,phaseu,...

    cd host
    python tests/test_mcp.py        # 31 checks, drives the server over stdio

## Driving the bench with a local model

`coaxial_ollama` hands the board's tool surface to a model running under Ollama,
adds a Python scope with the live `board` in it and an allowlisted shell, and
works through a plan one step at a time.

    python -m coaxial_ollama --plan coaxial_ollama/plans/bringup.yaml
    python -m coaxial_ollama --ask "what does the NTC read right now?"
    python -m coaxial_ollama --list-tools
    python tests/test_ollama_tools.py    # offline: no board, no ollama

The division of labour is the whole design. The model measures; it is never told
the limit and never asked for a verdict. `plan.Limit` applies the limits, in
Python, from a file under revision control - so a step's result is traceable to
the plan rather than to a sampling temperature.

Defaults are read-plus-code: board reads, `run_python`, and programs from
`--allow`. Pin writes and the test gate need `--allow-writes`; `--confirm` asks
before every side effect; `--read-only` removes code and commands entirely.
Every message, tool call and result lands in a JSONL transcript in `data/` as it
happens.

## Debug jobs: the cheap loop

`dbg.py` is the same board and the same tools with the cost turned down, for the
questions you ask sixty times an afternoon rather than the ones somebody signs.

    python dbg.py "the NTC reads exactly 25.00 - what is wrong?"
    python dbg.py -q "which channel is the DC link?"      # answer only
    python dbg.py --repl                                   # prompt loop
    python dbg.py --no-board --file ../Core/Src/main.c "what configures ADC3?"

Where the tokens went, measured on this tree:

| | tokens per turn, before the question |
|---|---|
| the plan runner: 350-token prompt, 11 tools | ~1390 |
| `dbg`, default `--tools code` | ~640 |
| `dbg --tools read` | ~560 |
| `dbg --tools none` | ~110 |

The savings are not magic, they are five choices: a seventy-token system prompt,
a tool subset instead of all eleven, `num_predict` capping the answer, `think`
off where the model supports it, and old tool results stubbed to their first line
instead of resent whole. Each turn prints `[N in / M out]` so the bill is visible
while it is being run up, and `--budget` stops the session at a number.

Then there are the commands that cost nothing at all:

    Coaxial_63100> /py round(board.analog.ntc_temperature()["celsius"], 2)
    37.12
    Coaxial_63100> /sh cube-cmake --build --preset Debug
    Coaxial_63100> /tools read        # reprice the turn
    Coaxial_63100> /ctx               # what the next turn will cost
    Coaxial_63100> /clear             # the cheapest command there is

Half of what one asks a model at a bench is really just "run this and show me",
and `/py` does that with no model in the loop. `/py` and `/sh` work even when
ollama is not running.
