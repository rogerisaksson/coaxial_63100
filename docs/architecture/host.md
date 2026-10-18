# Architecture: Host

The library: three domains, the rig and its wire, the board's devices,
acquisition, the MCP server, the local model's runner. Where its boards
run: [electronics](electronics.md); the terminal: [views](views.md).

Three domains; imports run one way: `coaxial` -> `machine`, `motor`.

| Word | Is |
| --- | --- |
| machine | the executive: actuators over IO nodes, programs, `machine/` |
| motor | the PMSM a board drives, `motor/` |
| board, node | one Coaxial63100 on a bus; as a `machine` node, `coaxial.node` |
| rig | `Coaxial63100`, the library's front door to one board |
| host | the computer this runs on |

```text
motor/              drivetrain.md
machine/            any board family, no import of one: robotics.md, and
                    each subject's modules in its document
coaxial/            rig.py = Coaxial63100, the front door; cli, errors, memory;
                    node (the family for machine: Coaxial node, joint, surface,
                    rotor, torque); profiles/ (a motor's drive record and
                    stand-in model, JSON)
coaxial/comm/       the wire: transport, codecs, protocol, broker, sessions
coaxial/devices/    one subsystem per functional area: board, afe, gates, boot..
coaxial/acquire/    the rig's task and stream (its mixins), records, reader,
                    clock, filter
coaxial/model/      inverter, sensorless, blocks: drivetrain.md; the thermal
                    network and rooms: thermals.md
coaxial/control/    motion, commission, identify: drivetrain.md
coaxial/kalman/     estimators: thermal_ident (thermals.md), observer
                    (drivetrain.md)
coaxial/simulated/  the stand-in: electronics.md
coaxial_mcp/        MCP server (stdio), docs tool
coaxial_ollama/     local-model runner (MODELS.md)
tools/notebooks/    the paper builder and make_notebooks
```

- `Board` (`board.py`): one subsystem per annotation; `parts()` reads them
  back. A 0x6E device is `class X(Device, device=protocol.DEVICE_X)`; ops
  are `IntEnum`s in `protocol.py`.
- Interfaces with real + simulated implementations: `Acquisition`,
  `PolledSensor`, `GateControl`, `BootControl`. `GateStage` is concrete.
- `wire.py` names the wire's scales (`r.centi()`, `r.q16()`, ...).
  Decorators: `afe.powered`, `subsystem.remembered`, `forgetting`.
- `session.py`: `open_session()` probes, falls back to the stand-in.
  `broker.py`: one port, many sessions (loopback 8763), answers the
  deadman every 3 s.
