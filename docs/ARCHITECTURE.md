# Architecture

Instrumentation stack for a 63 V / 100 A coaxial BLDC inverter. TIM1 and the
synced current path are configured and reachable; the control law sits in
`Drive/` beside `Modbus/` and `Thermal/`, portable and host-tested, and has run
only dry - **no motor has turned**. Each layer knows only the one below it, so
a protocol or a transport can be swapped without cascading edits.

## Firmware

**`Core/`** — CubeMX-generated. User code is the two sensor polls and
`Board_StoKeepalive()`, which stays at the top of the loop because
`link_active()` does a `continue`.

**`Board/`** — this hardware behind `board.h`: `adc`, `cal`, `clock`, `io`,
`imu`, `angle`, `pwm`, `sync`, `sto`, `log`, `daq`, `selftest`. Dependencies run
one way only.

* `board_pwm.c` and `board_sync.c` use CMSIS, not the `htim1` handle: clearing
  MOE has to be one store that cannot fail partway.
* `board_imu.c` and `board_angle.c` each carry a poll loop the main loop
  advances, writing a shared record the command layer reads - the host never
  drives a bus.
* `board_log.c` is one ring every source pushes into, so a host takes fifteen
  samples per round trip instead of one. Producers span an ISR and the main
  loop, which is why the critical section is PRIMASK and not a mutex there is no
  scheduler for.
* `board_daq.c` is one configured acquisition task - channels, clock, sampling
  window, decimation, accumulation - buffering raw bytes whose stride the config
  decides, and describing its own record layout so no host holds a copy of the
  shape.

**`Modbus/`** — RTU stack (CRC, PDU, framing). C standard library only, no CMSIS
or HAL. Lookup tables, not switches.

**`host/coaxial/broker.py`** — one process owns the serial port and the rest
ask it, so two sessions can work the same board at once. Modbus requests
cross unchanged - unit, function, payload - and the broker interprets none of
it, which is what stops it becoming a second protocol. Refusals arrive as the
`coaxial.errors` class they were raised as, so invariant 8 does not stop at a
socket.

Nobody starts it: the first session spawns one for the port it found, and it
takes itself down when its last client goes - the refcount the rails on the
board keep, for the same reason. A LOOK IS NOT A USE, so `--status` and the
staleness check cannot be the last one out. `tools/session.py --hold` is the
other case, a bench where the port should stay taken.

`test_conformance` is the exception and asks it to stand down first: that
suite sends deliberately malformed frames, which is the one thing a broker
cannot forward. It refuses while sessions are using it and says how many.

**The limits headers** — every fixed number the firmware depends on, in
sections, each carrying the measurement that chose it. Two files, one per
layer, so the includes run one way: `Board/Inc/board_limits.h` holds the
drivers' - part clocks, buffer sizes, poll rates, settle times, the dead-time
floor - and `Comms/Inc/comms_limits.h` the wire's, and includes the first
where a number here must hold against one down there. `IMU_CARGO <= IMU_BUF`
is a `_Static_assert` rather than a thing to remember; it was invisible while
the two lived in two layers, and a cargo arrived truncated.

Anything the board can be TOLD is in the calibration record instead
(invariant 7). `test_structure` fails a matching `#define` anywhere else, and
a `Board/` file that includes the comms header - the dead time was in three
places at once and the one that mattered was a stale binary.

**`Comms/`** — command dispatch and the wire. `wire.h`'s accessors are total
with sticky error flags, so a handler has one check at the end instead of an
`if` per field. `cmd_device.c` fronts every peripheral behind `0x6E`, the one
function code left, with `cmd_imu.c`, `cmd_angle.c`, `cmd_link.c`, `cmd_cal.c`,
`cmd_gate_drivers.c`, `cmd_log.c` and `cmd_daq.c` as its op dispatchers.
`dev_uart.c` is the only file touching a USART, and `link.c` runs one RTU state
machine per port over three of them - one slave, three wires, and the unit id
belongs to the board.

**`Shtp/`** — the BNO08X's transport framing. Portable C11 like `Modbus/`, and
host-tested the same way by `test_shtp_core.py`.

**`Drive/`** — the control law, one PWM period per call: a dq current loop
with decoupling and a dead-time table, min-max SVM, square-wave HF
injection and its demodulator, a two-state PLL in Kalman form with a
back-EMF error above a crossover speed, I/f, a polarity pulse, and the
window statistics a host judges it by. Portable C11 like `Thermal/`,
host-tested through `Drive/test/harness.c` against a PMSM model.
`board_drive.c` runs it from ADC3's injected interrupt with the two
conversions cached at each mode change, and the interrupt path carries
`-O2` whatever the preset - one step at -O0 was longer than the period.
Its parameters are the calibration record's (ids 15..44). Nothing has
run into a motor.

## Host

**`host/coaxial/`** — three interfaces, then the parts that answer them.

| Interface | Answers | Real | Stand-in |
|---|---|---|---|
| `Acquisition` | `configure`, `start`, `stop`, `read`, `latest`, `state` | `Daq`, `Coaxial63100` | `SimulatedDaq` |
| `PolledSensor` | `state`, `read`, `write`, `hold`, `resume`, `configuring` | `Imu`, `Angle` | `SimulatedImu`, `SimulatedAngle` |
| `GateControl` | the twelve `0x6E` device 4 ops | `GateDrivers` | `SimulatedGateDrivers` |

Every one has a real implementation and a simulated one, which is the whole
argument for declaring them: the stand-ins were duck-typed, and a name that
drifted surfaced as an AttributeError on the first call that reached for it -
`SimulatedImu`'s ten poll-loop methods were attached to the class by a helper
after the fact, so what it did and did not answer was invisible. A missing name
now fails at construction.

`GateStage` is concrete on purpose: it is the arming policy - the dead-time
check, the interlock, the bypass - and there is exactly one of that. The board's
ops stay a dumb slave's (invariant 10); refusing to arm is a host's judgement.

**`rig.py`'s `Coaxial63100` is the front door**: connect, `configure`, `read`,
`write`, and `gates` for the power stage. It owns the preflight all four views
were repeating - AFE_ON powers the ADC reference and both SPI parts, so it goes
up on the way in and back the way it was found on the way out, Ctrl+C included.
`Board` and its subsystems stay under `.board`, and the device forwards them
by name - `device.imu` is `device.board.imu`, no list to go stale - so a
caller reaches a subsystem without knowing the supply lives in `afe`, the
converters in `daq` and the counter in `clock`. `gates` is NOT forwarded: it
is the arming policy above the raw ops, and reaching past it is how a duty
write becomes what arms a stage. Topology, pin maps and the parts list come
off the board, never from a hardcoded table.

`orientation.py`, `dial.py` and `desk.py` are pure renderers - a reading in,
text out - so all three test without a board; `mesh.py` reduces the CAD export
in `render/models` to a surface the first can draw - parsed once per process
and held in memory, nothing cached on disk.

**The console style is `host/tools/stage.py`** - one rich Theme where every
role is named (values glow amber, labels recede, the chip keeps its meaning
colour) and the two templates every live view renders through: `frame_of`
for a drawing with instruments beside it, `panels_of` for a grid of them.
Every page wears one band, `band_of`: the name hard left, the port after it,
the LIVE/SIMULATED chip right with a cell of air - `header()` builds it for a
view, the front page's masthead the same way. `boot()` is the progress strip,
its bar fixed at 28 cells and the milestone text after it in brackets. The
front page, `tools/menu.py`, is up in 0.15 s: the turntable's solids build in
a thread and the board appears at about 1 s.
`test_views.py` runs each view whole against the stand-in, which is what
holds the template together across restyles.

`drive.py` is device 10. `sensorless.py` is the design arithmetic - the
SNR budget, the Kalman gains from the measured noise, the crossover speed,
the decision - and `commission.py` the eight steps against a rig, every
verdict the executive's rather than the board's; `tools/commission.py`
runs them and prints the line.

**`host/coaxial_mcp/`** — MCP server built for a token budget: dense
fixed-column text, 8.8x smaller than JSON. Probes OS ports and ST-Links for
Modbus, falling back to a duck-typed `SimulatedSession`. Multi-node segments,
unit 0 broadcast.

**`host/coaxial_ollama/`** — local LLM orchestrator: execution loops, context
scaling, tool routing, and stateless mid-session hardware and model swaps.

## Scaling

Firmware exposes raw ADC ticks; the host owns the physical conversions (NTC
parameters, divider ratios), so a hardware revision costs no firmware change.
Board-side scaling is kept only to cross-check the math.

## The test system

Twenty-three suites, 2096 checks. `run_tests.ps1` is the only interface -
`-AutomaticMinimal|Medium|High` for ~25/50/75 % of every check, `-All` the gate,
`-Only NAMES` and `-Tags SUBJECTS` for one change's worth, `-Structure` for the
package itself.

**A missing cable is not a failure.** A disconnected board falls back to the
stand-in; conformance and parity skip instead. Parity refuses to compare a
stand-in with a stand-in, so the stand-in's channel table is checked against
`s_adcTable` read straight out of `board_adc.c`, in `test_simulated.py`, on the
run that costs three seconds and no cable.

**The core suite needs no cable either.** `modbus_crc/slave/rtu` are built with
the host gcc and driven through ctypes with the tick counter injected, so t1.5,
t3.5 and the 2^32 wrap are tested by arithmetic rather than by waiting. A
missing compiler skips it; `setup.ps1` installs one.

**The drive closes its loop on this machine.** `test_drive_core.py` builds
`Drive/` with the host gcc and drives it through a PMSM model in Python -
saliency, saturation, back-EMF, a dead-time voltage error and the
firmware's two-period pipeline - so the current loop, the demodulator,
the observer and I/f are judged against the model's own constants.
`test_sensorless.py` checks the design arithmetic in closed form and runs
the commissioning against the stand-in, whose motor has known numbers to
recover.

**Structure suite** — ~3 s, no board, no model. Every module under `host/`
except the suites: imports cold, no cycles, no definition copied into two files,
no dead imports, nothing past 130 lines or 7 deep, every module and class
documented. `testline/` and `examples/` had been outside it for no reason and
had never been checked; async functions had been exempt without anyone deciding
it, because `AsyncFunctionDef` is not a subclass of `FunctionDef`. It exists
because the behavioural suites import what they need and pass while the rest of
the package is broken - measured, five NameErrors in one afternoon of moving
code.

**Test selection** — the local model reads the diff and names the suites, the
subjects inside the big one, and the live section
(`tools|sequence|language|none`). The path map is the fallback when no model
answers, and is coarse by construction: any line in one file pulls four
suites. An unusable reply means
run everything. A narrowed run still draws one test from each subject left out,
and coverage is counted in checks, not groups - groups run 2 to 77.

**A fixture must build what the product builds.** The live suite configured its
own client and passed for weeks on a configuration nobody ran: with `think`
unset the model called `analog_read`; with `think=False`, which is what the
prompt sends, the same question called `board_info`.
