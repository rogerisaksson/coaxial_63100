# Coaxial 63100

A three-phase BLDC inverter whose PCB sits coaxially behind the stator.
**63 V, 100 A** - the rating is the name. STM32H753VIT6 at 475 MHz.

Instrumentation first: the bridge switches on request, and the control law
(`drive/`, device 10) has run only dry - **no motor has turned**. `gates.arm()` is the
only thing that sets MOE, and it re-reads the dead time first because the
2EDL8034 has no interlock of its own.

## Start here

```powershell
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Check   # what is missing
. .\env.ps1                                                   # PATH + aliases
.\coaxial_tty.ps1                                             # the chooser
.\coaxial_tty.ps1 adc -Simulated                              # no cable needed
```

## The terminal

`.\coaxial_tty.ps1` is the chooser: one script in front of the live views
in `terminal/`, for looking at the board rather than remembering a
filename.

| | |
|---|---|
| front page | `host/tools/menu.py` - the turning board and the list. The pick comes back in the exit code (101 + position), because capturing stdout would turn the page's console into a pipe |
| a view | its own process: `terminal/<name>.ps1` wrapping `host/tools/show_<name>.py`, given `-Port`, `-Simulated`, `-Frames`. SESSION is `host/tools/show_session.py` itself; BOARD CHAT is `show_chat.py`, with `--claude` for ANTHROPIC |
| leaving a view | 0 (Q) quits the chooser; 64 (ESC, `TO_MENU`) returns to the front page - on the second question the view came from, with it lit; anything else is a failed view, its last lines kept on screen and any key back to the menu |
| on the way out | `show_session.py --leave` opens the port once and stops whatever a view left running, so "nothing was left running" is measured rather than assumed |
| `-Name` | skips the front page: `session`, `imu`, `angle`, `adc`, `gate_drivers`, `rotor_observer`, `thermal_observer` |
| `-Simulated` | no cable; every value invented, and every view says SIMULATED across the top |
| `-Frames N` | a view ends after N frames - how the view suite runs each one |

| view | on the menu |
|---|---|
| `session` | SESSION - board dashpanel |
| `imu` | BOARD ATTITUDE - board orientation visualizer |
| `angle` | SHAFT ANGLE - motor axle rotation position |
| `adc` | METER BRIDGE - metered channels |
| `gate_drivers` | MOTOR CONTROLLER > GATE DRIVERS - half bridge control |
| `rotor_observer` | MOTOR CONTROLLER > ROTOR OBSERVER - the drive on the model or the converters |
| `thermal_observer` | THERMAL OBSERVER - thermals estimation |
| `chat` | BOARD CHAT - CCC, the local llm, or claude over MCP |

MOTOR CONTROLLER and BOARD CHAT ask a second question - which half, who
answers.

`gate_drivers` is the one that switches. `+ -` duty, `[ ]` step, `A` arm,
`B` BKIN override, `I` interlock override, `1 2 3 4` run length, `R` run.

## notebook_examples

Executed notebooks, checked in with the stand-in's outputs so they read
without running; `SIMULATED = False` and a port at the bench.

They are written from `host/tools/make_notebooks.py`, which holds every
cell - edit there, not in the JSON, or a cell's code and its printed
output part company:

```powershell
python tools/make_notebooks.py --execute            # all of them
python tools/make_notebooks.py --execute daq_session
```

| file | what it walks through |
|---|---|
| `daq_session.ipynb` | connect, configure, set the clock, acquire in a loop |
| `gate_drivers_session.ipynb` | dead time, arm, duty, the gate snapshot, a burst |
| `shared_session.ipynb` | two sessions on one port, and who else is attached |
| `imu_session.ipynb` | the BNO085, and the three things it refuses over |
| `daq_pandas.ipynb` | a run into a DataFrame, scaled by the board's record |
| `daq_live_plot.ipynb` | currents over the switches, one time base, live |
| `angle_session.ipynb` | the A1335's registers, and whether there is a magnet |
| `thermal_budget.ipynb` | the SOA budget, and a burst planned against it |
| `thermal_model.ipynb` | the node network in Python, and how it was fitted |
| `loss_calculation.ipynb` | switching loss from the SPICE models, no board |
| `rotor_observer_session.ipynb` | the rotor observer on the board's own PMSM model |
| `propeller_sweep.ipynb` | the 5230SL and its propeller against Hobbywing's stand |
| `speed_loop.ipynb` | `coaxial.loop`'s chain, identified back out of its own run |
| `foc_montecarlo.ipynb` | the SMO and flux linkage observers, then the firmware's law over the 23-63 V sweep |
| `auto_tune.ipynb` | commissioning: measure the machine, tune against it, write the record |
| `position_servo.ipynb` | the PMSM as stepper and servo, ring and sag measured |
| `position_and_sensorless.ipynb` | observer vs shaft sensor, one rotor two answers |
| `app_*.ipynb` | quad lane, wing cruise, two-joint arm, precision hold |

## The library

```python
from coaxial import Coaxial63100
device = Coaxial63100(port='COM4')       # simulated_device=True: no cable
daq = device.daq                         # the data acquisition subsystem
daq.open()
daq.enable()                             # powers the analog front end
device.set_time_from_pc()                # the board counts cycles, not time
daq.configure(['Phase U', 'NTC'], sample_rate=1000)  # 1000 records/s, the board averages
daq.start()                              # buffering starts in the host and target
values = daq.read(-1)                    # blocks for the first, then takes the lot
for r in values:
    print(r.start_time, r.dt, [(s.name, s.value) for s in r.samples])
daq.stop()                               # buffering stops at target
daq.close()                              # the acquisition released
device.close()                           # the port, and the supply as found
```

`device.motion` is the drive as three verbs - `stepper`, `servo`,
`velocity` - each a `with` block that needs the stage armed first and
leaves the drive OFF; and `configure('phaseU', 'shaft angle')` rides the
sensor fields in every record as snapshots beside the sums (MINOR 7).

A record is an object AND the mapping it came from: `r.start_time`,
`r.dt` and `r.samples` are the shape a script reads, while `r['NTC']` is
still the SUM the board sent and `r['samples']` still the count that made
it. `r.samples[n]` is one channel - `.name`, `.unit`, `.raw`, `.count` and
`.value`, the sum over the count. `r.value('NTC')` asks for one
channel; `r.channel_name` is the same
order as a header row. `daq.channel_names()` answers that before the
first record arrives, and `daq.columns(values)` turns a run of
records into one array per channel plus `time` and `dt`. `daq.catalogue()` is everything this
board can put in a record, and `daq.configure()` takes those names in any
spelling: `configure('phaseU', 'NTC')` or `configure(daq.channels()[:5])`.

**Two buffers, the way a DAQ card has two.** The board's ring fills at the
sample rate; `start()` also puts a reader thread on the link here, and it
drains that ring into a host queue as fast as the link goes. `read_buffer()`
takes from the queue, so the `print` in the loop never sits between two
round trips - pyserial releases the GIL on read and write, so that is real
overlap. Measured on the debug probe's VCP, ten channels and the pins, with
4 ms of work a block: **84.4 records/s reading the board directly, 134.6
through the queue**, and the board's backlog ends at 0 instead of climbing.

Every read answers its own backlog - records still on the board the instant
it took its own - so pacing costs no extra round trip. `daq.buffered` is
both ends: `{'host', 'peak', 'dropped', 'backlog', 'reads'}`. `daq.blocks()`
is the same records when no reader is running: one round trip per block, on
the calling thread.

Everything raises rather than returning a status. **What a device is, and
which channels it has, come from the board** - add a row to
`board/src/board_adc.c` and every demo above shows it with nothing else
told.

## Build, flash, test

```powershell
cube-cmake --build --preset Debug      # must be zero warnings
STM32_Programmer_CLI -c port=SWD mode=UR -d build/Debug/coaxial_63100.elf -v --start
.\run_tests.ps1                        # ~25 % of the checks, the default
.\run_tests.ps1 -All                   # 100 %, the gate - CLAUDE.md holds the count
.\run_tests.ps1 -Structure             # does host/ still hold together - 4 s
```

A missing cable is not a failing suite: every suite falls back to a
stand-in that labels itself. CI runs the same `--offline` set on every
push (`.github/workflows/host.yml`) - what needs the bench stays the
bench's.

## Where things are

| | |
|---|---|
| `board/` | this hardware, behind `comms/inc/board.h` |
| `comms/` | the command stack over Modbus RTU |
| `modbus/` | the protocol. Portable C11, host-tested, no HAL |
| `host/` | `coaxial/` library, MCP server, ollama runner, suites |
| `electronics/` | schematic and BOM - the authority on what is fitted |
| `docs/` | [ARCHITECTURE](docs/ARCHITECTURE.md), [PROTOCOL](docs/PROTOCOL.md), [HARDWARE](docs/HARDWARE.md), [FINDINGS](docs/FINDINGS.md), [TODO](docs/TODO.md) |

**Read [FINDINGS](docs/FINDINGS.md) before investigating anything.** It
records what is already ruled out, and what it cost to find out.
