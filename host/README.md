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

## Acquisition, end to end

```python
daq = device.daq
daq.enable()                         # powers the analog front end
print(daq.catalogue())               # everything this board can record
daq.configure('phaseU', 'NTC')       # names in any spelling, or a list

with daq:                            # start, and stop however it goes
    rec = daq.read(-1)               # blocks for the first, takes the lot

t = daq.series(rec, 'time')          # one channel, one plain list
ntc = daq.series(rec, 'ntc')
for i in range(len(ntc)):
    print(t[i] - t[0], ntc[i])
```

`series()`The frame already says what each column is, so a plot needs no help
naming them:  is the currents and
 the U leg, in plain pandas.

The frame already says what each column is, so a plot needs no help
naming them - `df.filter(like=' (A)')` is the currents and
`df.filter(like='PWMU')` the U leg, in plain pandas.

`series()` is the common case - one channel and its values - and takes the
name as loosely as `configure()` does, because a long channel name is what
makes the shape worth having. `columns()` is the whole table the same way.
A record underneath carries `start_time`, `dt` (MEASURED, from the gap to
the next), `channel_name`, `samples`, and `value('NTC')` for one of them.

`with daq` is the bracket a task wants and not for tidiness: a script that
dies between `start()` and `stop()` leaves the board sampling, and the next
run is refused until somebody clears it by hand.

`daq.enable()` powers AFE_ON, which powers the converter's REFERENCE and
not only the signal path: off, every channel reads exact mid-scale and the
NTC exactly 25.00 C - plausible, and not a measurement. It is on the
acquisition rather than on `afe` because the rail is REFERENCE COUNTED -
taken here it is released when the session closes, Ctrl+C included, while
`board.afe.enable()` takes one that nothing gives back.

## Notebooks

```python
df = daq.frame(rec, scaled=True)     # time index, a column per channel
df['NTC (C)'].rolling(50).mean().plot()

with daq:                            # a rolling window, live
    for df in daq.frames(window=2.0, seconds=10, scaled=True):
        redraw(df)
```

`frame()` gives a DataFrame indexed by time with one column per channel -
the sampled pins included, so a gate duty and the current it produced are
columns of the SAME record. `scaled=True` adds real-unit columns from the
board's own calibration beside the codes.

`frames()` is the live case: it keeps the last `window` seconds of
RECORDS and builds each frame from them, so nothing is concatenated and
nothing grows. A plot that trims by hand becomes the bottleneck that fills
the board's ring, which is the bookkeeping this exists to take away.
Drawing is plain matplotlib - `python_examples/daq_live_plot.py` puts the
phase currents over one axis per leg, HS and LS.

pandas is imported **where it is called**, so the library still runs on a
bench without it: `columns()` is a dict of plain lists, which is what
`DataFrame` takes anyway.

**Zero and span belong to the calibration block**, not to the acquisition
one: they write the calibration record, so they live where it does.
`board.calibration.compensate(name, gain=, offset=)` writes one
channel's gain and offset into the calibration record - classic offset then
gain, `(code - offset) * gain`, in the order the board applies them. `gain`
is a plain multiplier here and parts per million on the wire. Either may be
left out to keep what the channel has.

```python
cal = device.board.calibration
cal.tare('phaseU', auto=True, save=False)   # measure now, write it
cal.tare()                                  # every current channel, saved
```

`tare()` is a measurement and then a `compensate()`: with `auto` it reads a
burst here and writes the mean as the offset, and with `auto=False` it asks
the board to do both in one op. Refused with the AFE off - it powers the
converter's reference, so every channel reads exact mid-scale and a tare
against that stores a plausible number that means nothing. The board keeps
it either way: `save=True` commits the record to flash, so the next session
and every other host read the channel the same way.

**Buffers.** `daq.configure_buffer(10000)` sizes the circular buffer in
RECORDS. With a broker in the path that is the BROKER'S ring, and every
client on it - another process, another thread, a view and a chat session
at once - reads that one ring from its own cursor without taking records
from the others. A reader lapped by the writer is told how many it lost in
`buffered()['lost']`; a gap nobody counted is the one outcome a shared ring
must not have.

`catalogue()` is the board's own list - analog channels and sampled pins
off `0x6D`, plus the sensor fields - each row saying its kind and whether
`configure()` can ask for it. The sensor fields (orientation, acceleration,
rotation rate, magnetic field, shaft angle) are **listed and not yet
selectable**: they are readable through `board.imu` and `board.angle`, and
carrying them inside a record is a wire format the firmware does not have.

`start()` puts a reader thread on the link and `read()` takes from the
queue it fills, so the loop body costs the link nothing. Every read answers
its own backlog, so pacing costs no extra round trip. `daq.buffered` is
both ends: `{'host', 'peak', 'dropped', 'backlog', 'reads', 'records',
'rate'}`.

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

    .\run_tests.ps1                 # ~25 % of the 2114 checks, the default
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

`board_chat.ps1` is the way in; the chooser's BOARD CHAT page runs the same
loop. Underneath, `coaxial_ollama` hands the board's tools to a model under
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
