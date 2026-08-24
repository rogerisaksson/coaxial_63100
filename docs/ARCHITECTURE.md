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

One of the nine, `docs`, touches no hardware. It hands the model this
repository's own documents, because they are what stop a reading being
misinterpreted — the AFE gate, the unknown phase gain, what has already been
ruled out — and the one reader who could not open them was the model standing at
the bench. It answers with an index rather than a document for the same token
reason as everything else here, and a search hit carries the chapter it sits
under: in FINDINGS the chapter is the meaning, and an entry quoted out of
*Refuted* says the opposite of what the document says. See
[MODELS.md](MODELS.md).

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
| `client.py` | `/api/chat` over urllib. Refuses cloud tags and non-loopback hosts; retries a crashed runner. |
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
