# Architecture

Instrumentation stack for a 63 V / 100 A coaxial BLDC inverter. TIM1 and the
synced current path are reachable; the control law in `Drive/` has run only
dry - **no motor has turned**. Each layer knows only the one below it.

## Firmware

| | |
|---|---|
| `Core/` | CubeMX-generated. User code: the two sensor polls, `Board_StoKeepalive()` at the top of the loop (`link_active()` does a `continue`), and `SCB_EnableICache()` in Init - CubeMX generated no cache, and the M7 fetched every instruction from flash at four wait states until 2026-08-31 (FINDINGS) |
| `Board/` | this hardware behind `board.h`: `adc`, `cal`, `clock`, `io`, `imu`, `angle`, `pwm`, `sync`, `sto`, `log`, `daq`, `drive`, `thermal`, `selftest`. Dependencies run one way |
| `Modbus/` | RTU stack - CRC, PDU, framing. C standard library only. Lookup tables, not switches |
| `Shtp/` | the BNO08X's transport framing. Portable C11, host-tested by `test_shtp_core.py` |
| `Comms/` | dispatch and the wire. `cmd_device.c` fronts every peripheral behind `0x6E` - the one function code left - with an op dispatcher per device; `dev_uart.c` is the only file touching a USART; `link.c` runs one RTU state machine per port - one slave, three wires, the unit id the board's |
| `Drive/` | the control law, one PWM period per call: dq current loop with decoupling and a dead-time table, min-max SVM, square-wave HF injection and its demodulator, a two-state PLL in Kalman form with a back-EMF error above a crossover speed, I/f, a polarity pulse, the window statistics a host judges it by. Portable C11, host-tested through `Drive/test/harness.c` against a PMSM model; `drive_model.c` is that model on the board, the second sample source (device 10 op 10) |
| `Thermal/` | the ten-node observer, portable like `Drive/` |
| `Filter/` | the decimating anti-alias chain: an integer boxcar at the converter's rate, then the Bessel biquads the host designed, then the decimation. Portable C11, host-tested by `test_filter_core.py`; `host/coaxial/bessel.py` designs the coefficients and reports what the chain fails to stop |

Before editing `Board/` or `Comms/`:

* `board_pwm.c` and `board_sync.c` use CMSIS, not `htim1`: clearing MOE has
  to be one store that cannot fail partway.
* `board_imu.c` and `board_angle.c` each run a poll loop the main loop
  advances into a shared record; the host never drives a bus.
* `board_log.c` is one ring every source pushes into - fifteen samples per
  round trip instead of one. Producers span an ISR and the main loop, so the
  critical section is PRIMASK.
* `board_daq.c` is one acquisition task that describes its own record
  layout; no host holds a copy of the shape.
* `board_drive.c` steps the law from ADC3's injected interrupt, the two
  conversions cached at each mode change; the interrupt path is `-O2`
  whatever the preset - one step at -O0 was longer than the period.
* `wire.h`'s accessors are total with sticky error flags: one check per
  handler, not one per field.

**The limits headers.** Every fixed number the firmware depends on, each
beside the measurement that chose it: `Board/Inc/board_limits.h` the
drivers' (part clocks, buffers, poll rates, settle times, the dead-time
floor), `Comms/Inc/comms_limits.h` the wire's, including the first where a
number must hold against one below. `IMU_CARGO <= IMU_BUF` is a
`_Static_assert`: invisible across two layers, a cargo arrived truncated.
Anything the board can be TOLD is in the calibration record instead
(invariant 7); `test_structure` fails a matching `#define` anywhere else and
a `Board/` file including the comms header - the dead time was in three
places at once and the one that mattered was a stale binary.

**The broker** (`host/coaxial/broker.py`): one process owns the port, the
rest ask it. Requests cross unchanged - unit, function, payload - and
refusals arrive as the `coaxial.errors` class they were raised as (invariant
8). The first session spawns it; the last client's exit takes it down; a look
(`--status`, the staleness check) is not a use; `tools/session.py --hold`
keeps a port taken. `test_conformance` asks it to stand down first -
malformed frames are the one thing it cannot forward - and it refuses while
sessions are using it, saying how many.

## Host

**`host/coaxial/`** - three interfaces, each with a real and a simulated
implementation, so a name that drifts fails at construction. Measured:
`SimulatedImu`'s ten poll-loop methods were attached by a helper after the
fact, and a drifted name was an AttributeError on the first call that
reached for it.

| Interface | Answers | Real | Stand-in |
|---|---|---|---|
| `Acquisition` | `configure`, `start`, `stop`, `read`, `latest`, `state` | `Daq`, `Coaxial63100` | `SimulatedDaq` |

**The stand-in has a line.** It carries a bitrate and charges time for the
bytes it returns, which is what makes a throughput number off it mean
anything, and it answers `max_rate_hz` by the board's own formula. Line
time is BANKED and paid in one sleep past 2 ms: `time.sleep` cannot honour
a sub-millisecond wait on Windows, and a reply at 10 Mbit/s is 292 us - per
reply it spent 2.878 s of a 4 s run and the emulator became the bottleneck
it was written to measure.
| `PolledSensor` | `state`, `read`, `write`, `hold`, `resume`, `configuring` | `Imu`, `Angle` | `SimulatedImu`, `SimulatedAngle` |
| `GateControl` | the twelve `0x6E` device 4 ops | `GateDrivers` | `SimulatedGateDrivers` |

**Two more files carry one concern each.** `reader.py` is the host-side
reader thread: between `start()` and `stop()` it is the ONLY thing that
touches the transport, and the consumer takes from a deque it fills - so a
`print` in a loop never sits between two round trips. `record.py` is what
comes back: a `Record` is a `dict` underneath, so the mapping keeps
working, with `start_time`, `dt`, `samples` and `channel_name` for a script
to read. `dt` is MEASURED, from the gap to the next record's timestamp,
because what a task asked for and what the loop managed are different
numbers.

**One transaction at a time on the wire.** `Transport.request` holds an
RLock for the whole exchange: two threads interleaving a transmit and a
receive put one thread's reply in the other's hands, or scatter a frame's
characters past t1.5 and lose both. And one drainer of the ring - while the
reader lives, `acquire()` and `blocks()` serve from its queue, because two
drainers each see the hole the other took.

`GateStage` is concrete: the arming policy - dead-time check, interlock,
bypass - and there is one of it. The board's ops stay a dumb slave's
(invariant 10); refusing to arm is the host's judgement.

**`rig.py`'s `Coaxial63100` is the front door.** It owns the preflight every
view was repeating - AFE_ON powers the ADC reference and both SPI parts, so
it goes up on the way in and back as found on the way out, Ctrl+C included.
Subsystems are forwarded by name (`device.imu` is `device.board.imu`, no
list to go stale); `device.daq` is the acquisition vocabulary as its own
handle; `gates` is NOT forwarded - reaching past the policy is how a duty
write arms a stage. Topology, pin maps and parts come off the board, never
from a table; so does scaling (HARDWARE, *Scaling & Calibration*).

`drive.py` is device 10; `sensorless.py` the design arithmetic - SNR budget,
Kalman gains from the measured noise, crossover, decision; `commission.py`
the eight steps against a rig, every verdict the executive's;
`tools/commission.py` runs them. `tools/show_rotor_observer.py` is ROTOR
OBSERVER: the estimate on the dial, the model's rotor on the rim, every
parameter a switch checked against the stage, A arms nothing without
`--switch`.

`motor.py` is the machine itself, in one place: the PMSM integrator, a
`Parameters` set that says whether it was `measured` and where it came
from, the propeller law, and the two machines this tree has - the
5230SL and the stand-in's own. It exists because the model was a copy in
`test_drive_core.py`, a second set of constants on `SimulatedDrive` and a
third in the DAQ stand-in's spin; four copies of a machine is four places
for an inductance to drift. `sysid.py` recovers R, Ld, Lq and lambda from
a run by least squares in dq and reports a **per-parameter uncertainty**,
because a V/f run cannot see Lq - without `di/dt` the `omega Lq iq` column
is collinear with lambda, measured at -73 % and correctly flagged
untrusted.

`inverter.py` is the power stage's numbers the same way `motor.py` is the
machine's: dead time, the FET's charge curves, the switch-node ring, the
sense chain's measured floor - each traced to the schematic, the LTSpice
model or FINDINGS, and the derived arithmetic (dead-time volts, the knee,
the compensation table, the sampling margin) defined once for everything
that simulates this drive. `loop.py` closes host control loops around
`motor.py` - blocks on one slotted bus, `>>` chaining reference, d-axis
probe, speed PI, current PI and machine - and `identify` hands a run to
`sysid.py`. Two lessons are in its comments: the voltage vector is aimed
half a period ahead because it is held in the stator frame while the rotor
turns (without it the fit read R at -218 %), and the machine publishes
before it advances so a recorded row pairs a voltage with the state it
acts on (published after, R read +17 % and Lq -4 % from alignment alone).

`motion.py` is the drive as three verbs - stepper, servo, velocity -
host loops priced by the link: ~7 ms a write buys tens of hertz, so the
stepper slews, the servo corrects BETWEEN moves (a per-pass position
loop at link rate samples the load-angle resonance aliased and pumps it
- measured, six clean passes wound the rotor through a pole slip), and
the velocity loop is `coaxial.loop`'s law over `omega_hat` - an ESC's
contract. Every block requires the stage armed first through
`gates.arm()` and leaves the drive OFF however it ends.

`tools/observer_run.py` runs the firmware's own observer, not a model of
it: it builds `Drive/` with the host gcc through the same ctypes bench the
suite uses, and asks how hard the thing can be driven rather than whether
it still works. FINDINGS has what it found. `tools/montecarlo.py` turns
that bench into a search: one process per core, each running the compiled
control law against a plant drawn around the 5230SL's estimates and a
stage drawn around `inverter.py`'s, a host speed loop from `loop.py` over
the observer's own speed, and a cost that prices a lost rotor at ten times
a bad one. A job can carry another machine and its limits, which is how the
auto-tune notebook hands the search what the commissioning just
identified. It schedules the controller across the 23-63 V link sweep;
`notebook_examples/foc_montecarlo.ipynb` is the run, the schedule and the
sensorless floor it found, `speed_loop.ipynb` is `loop.py`'s chain
identified back out of its own run, and `auto_tune.ipynb` is the whole
bench-day pipeline - commission, identify, search, write, verify -
rehearsed against the stand-in.

`orientation.py`, `dial.py` and `desk.py` are pure renderers - a reading
in, text out; `mesh.py` reduces the CAD export in `render/models`, parsed
once per process, nothing cached on disk.

**The console style is `host/tools/stage.py`**: one rich Theme with every
role named, two templates (`frame_of` for a drawing with instruments beside
it, `panels_of` for a grid), one band per page (`band_of` - name left, port,
LIVE/SIMULATED chip right), `boot()` the 28-cell progress strip, and
`run_view()` in `tools/screen.py` the loop every view runs - draw, pace,
take keys - so a view's `main` is setup, a `draw()` and a teardown. The front
page, `tools/menu.py`, is up in 0.15 s with the board at about 1 s - the
solids build in a thread. `test_views.py` draws every view against the
stand-in.

**`host/coaxial_mcp/`** - the MCP server, built for a token budget: dense
fixed-column text, 8.8x smaller than JSON; probes ports and ST-Links, falls
back to `SimulatedSession`; multi-node segments, unit 0 broadcast.
**`host/coaxial_ollama/`** - the local-model loop: intent, context scaling,
tool routing, mid-session board and model swaps (MODELS.md).

## The test system

Twenty-five suites, 2355 checks. `run_tests.ps1` is the only interface -
CLAUDE.md, *Commands*, has the tiers and the rules.

* **A missing cable is not a failure.** A disconnected board falls back to
  the stand-in; conformance and parity skip. Parity refuses to compare a
  stand-in with a stand-in, so `test_simulated.py` checks the stand-in's
  channel table against `s_adcTable` read straight out of `board_adc.c` -
  three seconds, no cable.
* **The core suite needs no cable.** `modbus_crc/slave/rtu` are built with
  the host gcc and driven through ctypes with the tick counter injected, so
  t1.5, t3.5 and the 2^32 wrap are tested by arithmetic. A missing compiler
  skips it; `setup.ps1` installs one.
* **The drive closes its loop on this machine.** `test_drive_core.py` builds
  `Drive/` with the host gcc and drives it through a PMSM model in Python -
  saliency, saturation, back-EMF, a dead-time voltage error, the two-period
  pipeline - so the loop, the demodulator, the observer and I/f are judged
  against the model's own constants. `test_sensorless.py` checks the design
  arithmetic in closed form and runs the commissioning against the stand-in,
  whose motor has known numbers to recover.
* **Structure suite** - ~3 s, no board, no model. Every module under `host/`
  except the suites: imports cold, no cycles, no definition in two files, no
  dead imports, nothing past 130 lines or 7 deep, every module and class
  documented. It exists because the behavioural suites pass while the rest
  of the package is broken - five NameErrors in one afternoon of moving
  code. `testline/` and `examples/` had been outside it; async functions
  had been exempt because `AsyncFunctionDef` is not a `FunctionDef`.
* **Test selection** - the local model reads the diff and names the suites,
  the subjects inside the big one and the live section
  (`tools|sequence|language|none`); the path map is the fallback and is
  coarse by construction - any line in one file pulls four suites; an
  unusable reply runs everything. A narrowed run still draws one test from
  each subject left out, and coverage is counted in checks, not groups -
  groups run 2 to 77.
* **A fixture must build what the product builds.** The live suite
  configured its own client and passed for weeks on a configuration nobody
  ran: with `think` unset the model called `analog_read`; with
  `think=False`, which is what the prompt sends, the same question called
  `board_info`.
