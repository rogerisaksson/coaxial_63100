# Coaxial 63100

Three-phase BLDC inverter, PCB coaxially behind the stator. 63 V, 100 A.
STM32H753VIT6 @ 475 MHz. Firmware + Python host library, MCP server and a
local-model runner. Open work: [docs/TODO.md](docs/TODO.md).

## Start

```powershell
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Check   # what is missing
python -m pip install -e host/                                # once: every script imports it
. .\env.ps1                                                   # PATH + aliases
.\coaxial_tty.ps1                                             # terminal front page
.\coaxial_tty.ps1 adc -Simulated                              # one view, no cable
```

Views (`host/terminal/pages/`, one module each): session, imu (attitude),
angle, adc (meter bridge), gate_drivers (the one that switches), rotor
observer, thermal observer, chat. `-Simulated` needs no cable; `-Frames N`
ends a view after N frames. In a view: Q quits, ESC returns to the front
page.

## Library

```python
from coaxial import Coaxial63100
with Coaxial63100(port='COM4') as device:          # simulated=True: no cable
    daq = device.daq
    daq.open(); daq.enable(); device.set_time_from_pc()
    daq.configure('phaseU', 'NTC')                  # names in any spelling
    daq.start()                                     # reader thread drains the board
    for r in daq.read(-1):
        print(r.start_time, r.dt, [(s.name, s.value) for s in r.samples])
```

- `r['NTC']` is the board's SUM, `r.value('NTC')` the mean, `r.samples` the
  per-channel structs. `daq.catalogue()` lists what can be recorded.
- `device.motion`: `stepper`, `servo`, `velocity` (arm the stage first).
- Everything raises rather than returning a status. Channels and parts come
  from the board.

## Machines

```python
from machine import Machine
humanoid = Machine.discover('humanoid', simulated=True)  # every board found
print(humanoid.prompt())                                # what a model is told, ~1300 characters
humanoid.run('0 run=squat times=2\n0 run=walk stride=15')
```

- `host/machine/` imports no board: a family (`coaxial.node`) is loaded by
  `Nodes.discover` when installed. Types: humanoid, quad, fixed_wing, ebike.

## Notebooks

Twelve executed papers in `notebook_examples/` (acquisition, link, sensors,
power_stage, thermal, drive, controller, sequencer, machines, motion, applications,
commissioning), generated from `host/tools/notebooks/`:

```powershell
python tools/notebooks/make_notebooks.py --execute [area ...]
```

## Build and test

```powershell
cube-cmake --build --preset Debug      # zero warnings; two images (app + bootloader)
python host/tools/target/build_and_flash.py   # --boot flashes the bootloader first
.\host\run_tests.ps1                   # ~25 %; -All the gate; -Structure 4 s
```

CI builds both presets and runs the offline suites on every push.

## Coverage

Lines of the hand-written code the offline suites run
(`python host/tools/dev/cover.py --readme` writes this table). Not counted: the
CubeMX code - `core/`, `startup_*.s`, `cmake/stm32cubemx/`, the HAL - which is
generated, and `board/` and `comms/`, which run on the target: the bench's
conformance suite is theirs.

<!-- coverage -->
<!-- /coverage -->

## Docs

[ARCHITECTURE](docs/ARCHITECTURE.md), [PROTOCOL](docs/PROTOCOL.md),
[HARDWARE](docs/HARDWARE.md), [BOOT](docs/BOOT.md),
[MODELS](docs/MODELS.md), [FINDINGS](docs/FINDINGS.md) (read before
investigating), [TODO](docs/TODO.md).
