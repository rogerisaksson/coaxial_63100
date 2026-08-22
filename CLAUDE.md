# coaxial_63100

Control firmware and a Python host library for a **coaxial BLDC inverter**: a
three-phase motor drive whose PCB sits coaxially behind the rotor of an
outrunner. The name is the rating — **63 V and 100 A**, the current being
instantaneous within the FETs' safe operating area.

STM32H753VIT6 at 475 MHz. The board carries an analog front end feeding three
differential phase-sense channels, a DC link sense, an NTC, and further
subsystems. All of it is reachable over one UART that carries either a text
console or a binary Modbus RTU link.

Mechanically this matters more than it sounds: a board packed behind a spinning
rotor has limited airflow, so the thermal channel is not decoration, and the
phase sense sits in the middle of a switching bridge, which is the context for
any noise figure in these documents.

## Scope: this is instrumentation, not yet a motor controller

**No timer is configured.** The `.ioc` enables ADC1/2/3, USART3, CORTEX_M7,
RCC, SYS, DEBUG, MEMORYMAP, NVIC and VREFBUF — and nothing else. There is no
PWM, no commutation, no gate-driver output and no current loop anywhere in this
firmware. What exists is a measurement and bring-up platform for the inverter:
it reads the phases, the DC link and the temperature, and exposes all of it over
the link.

Nothing here has been exercised near 63 V or anywhere close to 100 A, and no
measured value is recorded in this repository - see invariant 10.

VREFBUF is deliberately **disabled** with VREF+ left high-impedance, so the ADC
reference is driven externally by the AFE. That is the mechanism behind invariant
9 below, and it is why `ADC_VREF_VOLTAGE 3.3f` is an assumption about a rail
rather than a property of the chip.

This file is the orientation. The detail is in `docs/`, and reading the right one
first will save you re-deriving things that took real measurements to establish.

| Read this | When |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | before touching any source layout |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | before changing anything on the wire |
| [docs/HARDWARE.md](docs/HARDWARE.md) | before interpreting any measurement |
| [docs/MODELS.md](docs/MODELS.md) | before changing the local model, its tag, or its tools |
| [docs/FINDINGS.md](docs/FINDINGS.md) | **before investigating anything** — it records what has already been ruled out |

## Commands

```powershell
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Check   # what is missing
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Yes     # install the lot: winget,
                                           # python packages, ST bundles via cube.exe,
                                           # STM32CubeMX, the ST-Link driver, ollama
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -FirmwarePackage X.zip
                                           # STM32Cube FW_H7 into CubeMX's repository.
                                           # Only CubeMX needs it; Drivers/ is in git.
. .\env.ps1                                # tools on PATH: bench, dbg, board, cbuild,
                                           # cflash, cubemx
```

```bash
cube-cmake --build --preset Debug          # build; must be zero warnings

# Flash. SWD works plainly; JTAG needs the workaround below.
STM32_Programmer_CLI -c port=SWD mode=UR -d build/Debug/coaxial_63100.elf -v --start

cd host
python -m coaxial all                      # CLI against the board
python tests/test_conformance.py           # 40 Modbus conformance checks
python tests/test_mcp.py                   # 36 MCP server checks
python tests/test_ollama.py                # 161 runner and dbg checks, offline
python examples/read_board.py                # measure, judge nothing
python -m coaxial_mcp --port COM4          # MCP server, stdio
python -m coaxial_ollama --plan coaxial_ollama/plans/bringup.yaml   # local model drives the bench
python -m coaxial_ollama.capability        # which local model this machine should run
python dbg.py -m auto "..."                # that model, picked from cores/RAM/VRAM
python dbg.py "why does the NTC read exactly 25.00?"   # cheap one-off question
python dbg.py --repl                       # prompt loop; /py and /sh cost no tokens
```

`STM32_Programmer_CLI` lives at
`~/AppData/Local/stm32cube/bundles/programmer/2.23.0/bin/`. Nothing in the ST
toolchain is on the system PATH — arm-gcc, cmake, ninja and the programmer all
live under `%LOCALAPPDATA%\stm32cube\bundles\`, fetched by `cube.exe` - the
bundle manager inside the STM32 VS Code extension. `cube bundle install --yes
NAME` needs no ST account, which is what lets `setup.ps1` install the whole
toolchain, STM32CubeMX and the ST-Link driver unattended. `env.ps1` finds the
newest of each and puts them on PATH for one shell; `setup.ps1 -Check` says
which are absent. The board's VCP is **COM4**; the
ST-Link is an STLINK-V3SET.

## Layout

```
Core/        CubeMX-generated. main.c is 582 lines and holds ONLY CubeMX
             functions plus main(). Keep it that way.
Board/       this hardware, behind Comms/Inc/board.h
Comms/       the comms stack: cmd over proto over dev, plus the console
Modbus/      the protocol. Portable C11, no HAL anywhere in crc/slave/rtu.
host/        Python: coaxial/ library, coaxial_mcp/ MCP server,
             coaxial_ollama/ model-driven runner and dbg.py, testline/,
             tests, tools
setup.ps1    one-time environment setup; -Check changes nothing
env.ps1      per-shell PATH and the bench/dbg/board/cbuild/cflash/cubemx commands
docs/        this documentation
```

`cmake/stm32cubemx/CMakeLists.txt` is regenerated by CubeMX — **never add
sources there**. New sources go in the root `CMakeLists.txt` user blocks.

## Invariants

Break one of these and something works until it doesn't.

1. **The protocol core stays hardware-free.** `Modbus/Src/modbus_crc.c`,
   `modbus_slave.c` and `modbus_rtu.c` include nothing but `<stdint.h>`,
   `<stddef.h>`, `<stdbool.h>` and `<string.h>`. That is what makes them
   host-testable. Only `Comms/Src/dev_usart3.c` touches the USART.
2. **RTU timing is in raw `DWT->CYCCNT` ticks, never microseconds.** Dividing
   cycles down to microseconds moves the wrap point off a power of two, and the
   unsigned elapsed-time arithmetic then breaks silently across the wrap.
3. **Command 0x41's payload is append-only.** It is the frozen version record.
   Appending a field is a protocol MINOR bump; moving, resizing or repurposing
   one is a MAJOR whether you meant it or not.
4. **A host selects its codec on the protocol MAJOR alone**, never the firmware
   version — otherwise every firmware rebuild breaks the host.
5. **No printf while the binary link is open.** A blocking transmit inside a
   frame corrupts RTU framing and stalls reception long enough to latch a UART
   overrun, which on this silicon kills reception permanently.
6. **Every ADC read path must call `HAL_ADC_ConfigChannel` and clear `PCSEL`.**
   See FINDINGS; two separate bugs came from paths that did not.
7. **The host never reports a sensed quantity for the phase channels.** They sit
   behind AFE gain that neither side knows. Volts at the ADC pin, and no
   further.
8. **Nothing in the Python library returns a status code or None-for-failure.**
   Every call produces its result or raises from `coaxial.errors`.
9. **AFE_ON gates every analog reading**, because it powers the ADC reference and
   not just the signal path. With it off, channels read exact mid-scale and the
   NTC reports exactly 25.00 °C — a plausible number that is not a measurement.
10. **The board is a dumb slave. No limits, no expected values, anywhere in
    the firmware or in this repository's tests.** It reports raw codes; pass/fail
    against real thresholds belongs to a test executive on the line, beside the
    calibrated instruments. The single exception is `self_test`, which judges
    only what it can prove from its own registers and flash.
11. **The DC link divider's headroom is deliberate.** 49.9k/2.2k gives 78.15 V
    full scale against a 63 V rating, 24 % of margin. Do not "optimise" it away;
    on an inverter the over-rating transient is exactly what you want recorded
    rather than clipped.

## Two things that will waste your time if you do not know them

**JTAG connect-under-reset does not work on this setup.** Any connect that
asserts NRST fails with `Unable to get core ID`. Use
`-c port=JTAG mode=Normal reset=SWrst`, or just use SWD. This is the probe
firmware, not the board — the cabling is fine and was proven so. Also: end a
programmer invocation with `--start`, not `-hardRst`, or the core is left halted.

**The AFE switch (PB2) powers the ADC reference.** With it off, every channel
reads exact mid-scale and the NTC reports exactly 25.00 °C — a plausible-looking
number that is not a measurement. Enable it before believing anything analog.

## Tooling traps in this environment

- Writing C escape sequences through a Python string inside a bash heredoc
  mangles them: `\r\n` arrives as a real CR+LF. Build the backslash with
  `chr(92)`, or write the C to a file with a quoted heredoc and splice it.
- `Core/Src/main.c` is LF-terminated. Python `open(...)` without
  `newline=''`/`newline='\n'` will convert it to CRLF on write.
- Long `cat > file <<'EOF'` heredocs get truncated. Split them.
