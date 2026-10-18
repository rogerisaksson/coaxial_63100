# Architecture

## Data-oriented design

One shape on both sides: data in, a step, data out; memory laid once; the
order a table; the hardware at one edge.

| | Target, C11 | Host, Python |
| --- | --- | --- |
| Data | naked structs: a state, an input, an output | `dataclass(slots=True)`, or a numpy structured array: a field a register or channel, a row a board |
| Logic | `void x_step(x_t *s, const x_in_t *in, x_out_t *out)`, its arguments only | `def step(s, u) -> NDArray`, its rows and inputs only, every row at once |
| Memory | static; no heap | laid before the loop: frame, log, buffers |
| Flow | tables of function pointers, walked in order | the cycle: poll, payload into memory, steps in table order, memory out and written |
| I/O | the board layer fills buffers (the sync's latch, DMA, the IMU's record); the cores read them | one edge, `poll` and `push` over `Transport` (framing, CRC, turnaround): a lost frame leaves its row, counted, never raised into a step |

- Target: the cores (`drive thermal ctrl daq filter`) are portable C11, built
  and tested on this host; `drive_step(d, in, stage, out)` is one PWM
  period; `ctrl.c` steps a part through `FILTERS`, `ESTIMATORS`,
  `REGULATORS`, tables by kind.
- Host: `machine/cyclic.py` - `frame`, `state`, the parts as steps
  (`STEPS`, `machine.parts` step for step), `edge`, `poll`, `push`, `cycle`,
  `run`. `machine.Loop` is the same pass over a dict of channels, for the
  panel and the sequencer.
- Notebooks: the paper's loop in four cells - memory, steps, table, cycle
  (`notebook_examples/cycle.ipynb`).

## Layers

The target: `core/` (CubeMX) -> `board/` (this hardware) -> `comms/` (the
wire) -> the portable C11 cores. The host: `coaxial` -> `machine`, `motor`,
imports one way, `terminal` and `tools` over them. Each module's brief:
`python tools/dev/host_map.py` from host/, each target file's
`python host/tools/dev/target_map.py`.

## Subjects

| Subject | Holds | Document |
| --- | --- | --- |
| Firmware | the target's layers, the wire's handlers, the portable cores, boot order and the loop, new hardware | [architecture/firmware.md](architecture/firmware.md) |
| Electronics | the board layer a part a file, and what stands in for it: board/fake, board/native, board/emu on Renode, world/, the stand-in | [architecture/electronics.md](architecture/electronics.md) |
| Drivetrain | motor to joint: the PMSM, the drive's law, the filter and ctrl/ on the board, the drives as bought, gearboxes, linkages, the buses | [architecture/drivetrain.md](architecture/drivetrain.md) |
| Thermals | a board's heat: the 20-node observer and envelope, its identification, a drive's four nodes, the tools | [architecture/thermals.md](architecture/thermals.md) |
| Kinematics | the gynoid's figure as numbers: segments and masses, dual quaternions, a leg's reach, movements' terms, MuJoCo's XML, the static model in C | [architecture/kinematics.md](architecture/kinematics.md) |
| Solvers | the dense QP, the whole-body stack and its step without an active set, Python and C row for row | [architecture/solvers.md](architecture/solvers.md) |
| Planners | the latent loop, the one way she moves: text to rows, the latent step, the modes, poise, stride, flight, the twist | [architecture/planners.md](architecture/planners.md) |
| Robotics | the executive any robot runs on, and the gynoid round her loop: build, world, senses, the run, identification | [architecture/robotics.md](architecture/robotics.md) |
| Aerodynamics | what flies on the shared flight stack: intent as text, kinematics in a vacuum, routines, grounds; the helicopter, the ornithopter, the aerobat | [architecture/aerodynamics.md](architecture/aerodynamics.md) |
| Quad | the quad on four boards: its frame, the aero/ core it flies on, its action space, its flights, its race | [architecture/quad.md](architecture/quad.md) |
| Host | the library: three domains, the rig and its wire, devices, acquisition, the MCP server, the local model's runner | [architecture/host.md](architecture/host.md) |
| Views | the terminal, its pages and live views, the renderers, the 2D drawings | [architecture/views.md](architecture/views.md) |
| Tests | sizes, cores through ctypes, structure, the relay, tiers, CI | [architecture/tests.md](architecture/tests.md) |
