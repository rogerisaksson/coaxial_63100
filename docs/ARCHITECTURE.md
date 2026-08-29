# Architecture

Instrumentation stack for a 63 V / 100 A coaxial BLDC inverter. TIM1 and the
synced current path are configured and reachable; **no control law is** -
commutation and the current loop belong above `Board/` and beside `Comms/`, and
neither exists. Each layer knows only the one below it, so a protocol or a
transport can be swapped without cascading edits.

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

**`Comms/Inc/board_limits.h`** — every fixed number the firmware depends on,
in sections, each carrying the measurement that chose it: part clocks, buffer
sizes, poll rates, settle times, the dead-time floor. Anything the board can
be TOLD is in the calibration record instead (invariant 7); this file is what
a rebuild is needed to change. `test_structure` fails a matching `#define`
anywhere else - the dead time was in three places at once and the one that
mattered was a stale binary.

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
`Board` and its subsystems stay under `.board`; nothing is hidden, but a caller
wanting measurements should not have to know the supply lives in `afe`, the
converters in `daq` and the counter in `clock`. Topology, pin maps and the parts
list come off the board, never from a hardcoded table.

`orientation.py`, `dial.py` and `desk.py` are pure renderers - a reading in,
text out - so all three test without a board; `mesh.py` reduces the CAD export
in `render/models` to a surface the first can draw.

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

Eighteen suites, 1768 checks. `run_tests.ps1` is the only interface -
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
