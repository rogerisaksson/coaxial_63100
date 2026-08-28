# Coaxial 63100

A three-phase BLDC inverter whose PCB sits coaxially behind the stator.
**63 V, 100 A** - the rating is the name. STM32H753VIT6 at 475 MHz.

Instrumentation, not a motor controller: the bridge switches on request and
there is **no commutation and no current loop**. `arm_gate_drivers()` is the
only thing that sets MOE, and it re-reads the dead time first because the
2EDL8034 has no interlock of its own.

## Start here

```powershell
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Check   # what is missing
. .\env.ps1                                                   # PATH + aliases
.\demo.ps1                                                    # pick a view
.\demo.ps1 adc -Simulated                                     # no cable needed
```

## Demos

`.\demo.ps1` is the menu; each is also `.\demos\<name>.ps1`. Every one takes
`-Simulated` (no board) and `-Frames N` (stop after N).

| view | what it shows |
|---|---|
| `adc` | every analog channel on a meter bridge, in its own unit |
| `imu` | board attitude, drawn from the STL the IMU turns |
| `angle` | shaft angle, the magnet and the air gap |
| `capture` | buffered: the AFE, the pins and both SPI parts at once |
| `gate_drivers` | the six gate signals as one instant, current, DC ripple, a timed burst |

`gate_drivers` is the one that switches. `+ -` duty, `[ ]` step, `A` arm,
`B` BKIN override, `I` interlock override, `1 2 3 4` run length, `R` run.

## python_examples

Notebooks that open as scripts - `# %%` cells, `SIMULATED = True` for no
cable.

| file | what it walks through |
|---|---|
| `daq_session.py` | connect, configure, set the clock, read N blocks |
| `gate_drivers_session.py` | dead time, arm, duty, the gate snapshot, a burst |

## The library

```python
from coaxial import Coaxial63100
with Coaxial63100(port='COM4') as daq:      # simulated_device=True: no cable
    daq.set_time_from_pc()                  # UTC, not this PC's idea of it
    daq.configure(['Phase U', 'NTC'], accumulate=8)
    daq.start()
    for block in daq.blocks(20):
        r = block[-1]
        print(r['time'], r['NTC'] / r['samples'])   # a value is a SUM
```

Everything raises rather than returning a status. **What a device is, and
which channels it has, come from the board** - add a row to
`Board/Src/board_adc.c` and every demo above shows it with nothing else
told.

## Build, flash, test

```powershell
cube-cmake --build --preset Debug      # must be zero warnings
STM32_Programmer_CLI -c port=SWD mode=UR -d build/Debug/coaxial_63100.elf -v --start
.\run_tests.ps1                        # ~25 % of the checks, the default
.\run_tests.ps1 -All                   # 1738 checks, the gate
.\run_tests.ps1 -Structure             # does host/ still hold together - 4 s
```

A missing cable is not a failing suite: every suite falls back to a
stand-in that labels itself.

## Ask the board

There is a local model on this machine with the board's tools wired to it.

```powershell
board_prompt -Ask "vad sitter på kortet?"
```

## Where things are

| | |
|---|---|
| `Board/` | this hardware, behind `Comms/Inc/board.h` |
| `Comms/` | the command stack over Modbus RTU |
| `Modbus/` | the protocol. Portable C11, host-tested, no HAL |
| `host/` | `coaxial/` library, MCP server, ollama runner, suites |
| `electronics/` | schematic and BOM - the authority on what is fitted |
| `docs/` | [ARCHITECTURE](docs/ARCHITECTURE.md), [PROTOCOL](docs/PROTOCOL.md), [HARDWARE](docs/HARDWARE.md), [FINDINGS](docs/FINDINGS.md), [TODO](docs/TODO.md) |

**Read [FINDINGS](docs/FINDINGS.md) before investigating anything.** It
records what is already ruled out, and what it cost to find out.
