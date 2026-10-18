# Architecture: Thermals

A board's heat: the observer and envelope it runs, its network on the host,
its identification, and a drive's heat as the gynoid's boards keep it. The
board's thermal findings: [FINDINGS](../FINDINGS.md).

| Where | Is |
| --- | --- |
| `thermal/` | 20-node observer, envelope, online identification; portable C11, device 8 (`coaxial/devices/thermal`) |
| `board/src/board_thermal.c` | the observer and its envelope on this hardware |
| `coaxial/model/` | thermal: the node network `thermal.c` runs, on the host; thermal_app: what a board is mounted in, `thermal_app.c` mirrored; rooms: the rooms a simulated or emulated board is laid in, and the tour through them |
| `coaxial/kalman/thermal_ident` | the online identification as the board runs it, `thermal_ident.c` mirrored |
| `coaxial/simulated/thermal/` | the observer without a board: the board's own C beside a truth in its room (`world_heat.c`, `world_stand.c`) |
| `machine/heat` | a drive's board's heat: four nodes (switch, laminate, stator, winding), the envelope's derate and trip, reported in its reply; what a drive holds for ever |
| `coaxial/draw/thermalmap` | the board as a thermal picture: a braille halftone in the thermal ramp |
| `host/tools/thermal/` | calibrate, identify, validate, trace |
| `host/tools/cores/thermal.py` | the thermal core's ctypes harness |

- Suites: `test_thermal_core.py`, `test_thermal_app.py`.
