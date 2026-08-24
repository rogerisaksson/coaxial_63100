# Architecture

The firmware measures and exposes a coaxial BLDC inverter rated 63 V / 100 A. It
does not yet drive it: no timer is configured, so there is no PWM, no commutation
and no current loop. Everything below is therefore an instrumentation
architecture, and the place a motor control layer would eventually go is above
`Board/` and beside `Comms/`, not inside either.

Both sides are layered the same way, and the layering is the point: each layer
knows only about the one below it, so the protocol can be swapped without
touching the commands, and the wire can be swapped without touching either.

```
        FIRMWARE                              HOST

  cmd        request/response          Board + subsystems
   |         Comms/Inc/cmd.h            host/coaxial/board.py
   |                                          |
  proto      framing, addressing        protocol codes
   |         Modbus/                     host/coaxial/protocol.py
   |                                          |
  dev        bytes, errors, clock       Transport
             Comms/Inc/dev_serial.h      host/coaxial/transport.py
```

## Firmware

### `Core/` — CubeMX territory

`main.c` is 582 lines and contains only what CubeMX owns: `SystemClock_Config`,
`PeriphCommonClock_Config`, `MX_ADC1/2/3_Init`, `MX_USART3_UART_Init`,
`MX_GPIO_Init`, `MPU_Config`, `Error_Handler`, `assert_failed`, and `main()`.
Every USER CODE region is empty except three:

- **Includes** — `board.h`, `console.h`, `link.h`
- **Block 2** — ADC calibration, `Board_TimebaseInit()`, `link_init()`,
  `Console_Banner()`
- **The loop** — five lines: if the link is open, pump it and `continue`;
  otherwise poll the console.

New board code goes in `Board/`, not here. This file was 1497 lines before the
reporting functions moved out, and it got that way one convenience at a time.

### `Board/` — this hardware

Implements `Comms/Inc/board.h`, which is the whole surface the comms stack may
use. The ADC helpers stay `static` inside `board_adc.c`; the stack asks, the
board answers, and the dependency runs one way.

| File | Holds |
|---|---|
| `Inc/board_hw.h` | the CubeMX handles, externed once. Board layer only. |
| `Src/board_adc.c` | phase channel map, `s_adcTable`, `ADC_ReadOneChannel`, the NTC and DC-bus conversions, all `Board_Adc*`/`DcBus`/`Ntc`/`PhaseRaw`/`AdcBurst`/`AdcNoise` |
| `Src/board_clock.c` | `Board_Name`, clock queries, `Board_SysClkOnCrystal`, `Board_TimebaseInit` |
| `Src/board_io.c` | AFE_ON, PE15, `Board_RequestConsoleMode` |

### `Modbus/` — the protocol, portable

`modbus_crc.c`, `modbus_slave.c` and `modbus_rtu.c` include only C standard
headers. No HAL, no CMSIS. They compile on a host compiler, which is why the
framing and PDU logic can be reasoned about without hardware.

- `modbus_crc.*` — CRC-16/MODBUS, bit-serial. Low byte first on the wire, which
  is the opposite of every other 16-bit field; that asymmetry lives in
  `modbus_crc_append()` so no call site has to remember it.
- `modbus_slave.*` — the PDU engine. Function codes dispatch through `FC_TABLE`,
  a table of nine one-line wrappers, not a switch. User-defined codes (65..72,
  100..110) route to `model->user_function`.
- `modbus_rtu.*` — the framing state machine. Time is injected as raw ticks plus
  a ticks-per-microsecond figure.
- `modbus_map.*` — this board as the four Modbus data tables. See PROTOCOL.md.

### `Comms/` — the stack

| File | Role |
|---|---|
| `dev_serial.h` / `dev_usart3.c` | the device: `get`/`fault`/`put`/`ticks`/`purge`. **The only file that touches the USART.** |
| `wire.h` / `wire.c` | total accessors — see below |
| `cmd.h` / `cmd.c` | dispatch across the command tables |
| `cmd_board.c` | application commands |
| `cmd_test.c` | test-fixture commands |
| `testrig.h` / `testrig.c` | raw GPIO behind a gate, with the reserved-pin list |
| `link.h` / `link.c` | assembles device + protocol + commands, pumps from the loop |
| `link_report.c` | the printf view, kept apart so `link.c` never needs stdio |
| `console.h` / `console.c` | printf retarget, boot banner, three-key console |
| `board.h` | the seam `Board/` implements |
| `version.h` | firmware and protocol versions, and the versioning rules |

### Why the C reads flat

`wire.h` provides **total** accessors. The writer sets a sticky flag on
overflow; the reader sets one on underrun. Nothing fails at the point of use, so
a handler is a straight run of statements and `cmd_dispatch` checks the flags
once at the end:

```c
static cmd_status_t h_version(rd_t *in, wr_t *out)
{
  (void)in;

  wr_u8(out, CMD_PROTO_MAJOR);
  wr_u8(out, CMD_PROTO_MINOR);
  wr_u8(out, FW_VERSION_MAJOR);
  wr_u8(out, FW_VERSION_MINOR);
  wr_u8(out, FW_VERSION_PATCH);
  wr_str(out, FW_DEVICE_NAME);
  wr_str(out, FW_MCU_NAME);
  wr_str(out, FW_BUILD_STRING);
  wr_u16(out, cmd_count());

  return CMD_OK;
}
```

No `if` per field. A handler branches only where the *board* can genuinely fail.
Command tables are data, so adding a command is one row and one function — there
is no switch to keep in step and no registration call.

## Host

`host/coaxial/` is a library, split by **functional area of the board** rather
than by protocol feature, so a line of a test script says which part of the
hardware it touches without naming a function code.

Nothing in it holds a channel map. `system.channel_map()` reads the board's own
(command `0x6D`), and `Gpio._refusal` asks that rather than a table here, so a
pin added to `Board/Src/board_io.c` is refused on the host with no edit.

`board_info` renders it one **block per kind**, each with its own header and
columns, and takes `kind` to answer with one of them. Both halves were
measured: with the digital pins appended to the analog table under the analog
header, "ge mig en lista över alla analoga kanaler" put eleven lines on screen
— two of them digital rows with no index and their columns out of line — to
answer with seven. It is `board_info kind=analog` now, and nine lines that are
all analog.
`protocol.RESERVED_PINS` survives only as the fallback for firmware older than
protocol 1.3, and is not to be extended. In the firmware the same list is the
only one: `testrig.c` used to keep its own copy and now calls
`Board_PinUsable`.

```python
board.system    identity, versions, clock tree, releasing the console
board.link      echo, frame counters
board.afe       the front end switch — its own subsystem because PB2 also
                powers the ADC reference, which everything analog depends on
board.analog    channels, bursts, temperature, DC bus
board.gpio      raw pin access for a fixture, behind a gate
```

Supporting modules: `errors.py`, `crc.py` (asserts the catalogue check value
`0x4B37` at import, so a broken edit fails loudly instead of corrupting every
frame), `wire.py`, `protocol.py`, `scaling.py`, `transport.py` — the only module
importing pyserial — `board.py`, `cli.py`.

### `host/coaxial_mcp/` — the MCP server

Built with the token budget as the design constraint, so a small model can run a
long test sequence. Nine coarse tools rather than one per firmware command,
because the whole tool list is re-read every turn. Dense fixed-column text
results rather than JSON: the same seven-channel reading is 278 characters here
against 2457 as indented JSON, a factor of 8.8. Uses
`mcp.server.lowlevel.Server` with hand-written schemas — deliberately not
FastMCP, because schema size is the thing being optimised.

`detail.py` decides how much of all that a given reader gets: one spec carries
both a full and a terse description, and the level is resolved from the model's
own tag rather than written into the text. A frontier model over MCP reads the
whole thing; `gemma4:12b` reads a third less of it, out of a window it shares
with the readings. See [MODELS.md](MODELS.md).

One of the nine, `docs`, touches no hardware. It hands the model this
repository's own documents, because they are what stop a reading being
misinterpreted — the AFE gate, the unknown phase gain, what has already been
ruled out — and the one reader who could not open them was the model standing at
the bench. It answers with an index rather than a document for the same token
reason as everything else here, and a search hit carries the chapter it sits
under: in FINDINGS the chapter is the meaning, and an entry quoted out of
*Refuted* says the opposite of what the document says. See
[MODELS.md](MODELS.md).

`coaxial_mcp.session.open_session(port, baud, unit, simulated=None)` returns
`(session, origin)`. With `simulated=None` it looks for the board rather than
assuming a port: `find_board.discover` tries `port` first if Windows lists it,
then every debug probe, then everything else, and each try is the same Modbus
round trip a tool call makes — not a weaker check that could pass here and fail
a moment later. Nothing answering anywhere hands back
`coaxial.simulated.SimulatedSession`.

Which port is the debugger is answered by the USB VID, not by opening it: every
ST-Link VCP enumerates under `0483` (measured here, an STLINK-V3SET reports
`0483:374F`). That is why probes can be tried first — `find_board.kinds()`
sorts the candidates for the cost of one enumeration, and
`board_prompt/ComPort.ps1` uses the same call for the same order.

`SimulatedSession` is duck-typed against `Session` and `Board`, not a protocol
simulator: it builds no frames. Every touchpoint labels itself — `firmware` and
`build` read literally `simulated` in the version record, so `board_info` alone
tells them apart.

`origin.label` names the **path**, not just the port, because the two paths are
not interchangeable: the probe is a bench cable that also flashes the board,
RS485 is the field bus an installed drive sits on.

| `origin` | label | prompt |
|---|---|---|
| probe VCP | `JTAG and COM3` | `Coaxial 63100(JTAG and COM3)` green |
| any other port | `RS485 at COM5` | `Coaxial 63100(RS485 at COM5)` green |
| nothing answered | `Simulated` | `Coaxial 63100(Simulated)` yellow |

`origin.real` is the half that matters. A suite that ran against the stand-in
proved the host and nothing about the firmware, so every caller prints the
label: `dbg.py` in the prompt tag, `python -m coaxial_mcp` on stderr
(`--simulated` forces the stand-in, `--auto` searches), `test_mcp.py` and
`test_live_model.py` in a header line before the first `PASS`.

Mid-session, `/board simulated | auto | rs485 | COM4` swaps it and the prompt
tag follows on the next line — the same factory, so the screen and the tools cannot
drift apart. `/model TAG` does the same one layer up, and hands the old model's
VRAM back **before** asking for the new one: the other order is a request for
two copies of the weights on one card. Both cost zero model tokens, which is
the point — neither is a thing to ask a model to do.

Nor is either a thing to *ask* a model to do in prose. `debug.board_switch()`
reads "byt till debugproben", "växla till COM4", "switch to the real board" as
the orders they are and carries them out without a model turn — the same shape
as `language.bare_switch`, and settled the same way: a verb, a target, and
nothing left over once the filler is taken out. `rs485` narrows the search past
the debug probe (`find_board.discover(only=...)`), because probe-first would
otherwise answer with the one board the operator just ruled out. A search that
finds nothing says so rather than reporting only where it ended up.

`test_conformance.py` deliberately does not use it. It is an independent
byte-level master, built from the specification so a shared wrong assumption
between master and slave cannot hide a defect — and a simulated slave would be
that shared assumption, written by the same hand. With no board it runs its CRC
self-test and says what it skipped.

### `host/coaxial_ollama/` — the local model, and the loop around it

The largest package, and the one with the most rules per line, because almost
every one of them came from a transcript rather than a design. Two entry
points over one tool surface:

| Module | What it owns |
|---|---|
| `debug.py` | `dbg.py --repl` — the cheap prompt loop. `Chat` is the turn: trim, call, backstop, answer. Also the CLI (`parse`/`build`/`repl`/`main`). |
| `runner.py` | `python -m coaxial_ollama --plan` — one conversation per plan step, a JSONL transcript, and a verdict that comes from `plan.Limit` in Python, never from the model. |
| `tools.py` | The tool surface: the nine MCP tools imported unchanged, plus `run_python`, `run_command`, `build_firmware`, `run_tests`, `link_diagnose`, `report`. `Toolbox` holds the operator's policy — `--confirm`, `--read-only`, `--allow-writes`. |
| `replies.py` | Reading what the model *meant*: is this answer a retyped table, is it a tool call written into `content`, is the residue prose or template noise. Pure functions over text. |
| `client.py` | `/api/chat` over urllib. Refuses cloud tags and non-loopback hosts; retries a crashed runner, and climbs a ladder of its own when the card is genuinely full. |
| `context.py` | What fits: the prompt's share of `num_ctx`, and what a conversation gives up when it does not fit. Shared by both loops above. |
| `capability.py` | Which tag this machine should run, from cores, RAM and VRAM. |
| `language.py` | Which language to answer in, decided here rather than asked of the model. |
| `sandbox.py` | Where `run_python` and `run_command` actually run. |
| `spinner.py` | The prompt's own line. |
| `plan.py` | A YAML test plan, and the limits the model is never shown. |

`host/tools/` holds what those wrap: `build_and_flash.py`, `run_tests.py`,
`find_board.py`, `warm_model.py` — each a plain script, runnable by hand, so
the model's version of a job and yours are the same code. See
[MODELS.md](MODELS.md) for why each exists.

## Where scaling lives

The firmware reports **raw ADC codes**. The host owns divider ratios, thermistor
constants and the reference voltage, through `NtcParams` and `DividerParams`. A
fixture with different parts needs new arguments, not new firmware.

The one exception is the legacy `adc_scan` / `adc_table` commands, which apply
the board's own conversions. They are kept because two independent paths to the
same number is how a scaling mistake gets caught — and one did.
