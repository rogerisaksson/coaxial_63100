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

## Firmware

```text
core/     CubeMX. main.c: CubeMX code + calls in USER CODE blocks only
board/    this hardware; API in comms/inc/board.h -> comms/inc/board/<x>.h
          (one header per board_<x>.c; board_boot.c -> board/handover.h)
          board/inc/board_hw.h: CubeMX handles, board layer only
          board_flash.c: the record's sector, under the portable board_cal.c
          board/fake/: the API on the host, neutral, under comms/,
          board_cal.c and board_thermal.c for the offline suites
          (tools/cores/fakeboard.py, tests/test_wire.py, the conformance
          suite through it)
          board/native/: the chip in C - TIM1, ADC1-3, GPIO, SPI2/4 and
          DMA1, the AFE, the A1335, the BNO085 - under board/src's stage,
          triple, meter, power, STO, angle, IMU and drive, over board/fake:
          native:// in real time, for SIL/HIL; validation is board/emu's
          (tools/cores/native.py, tests/test_native.py)
          board/emu/: the board on Renode's STM32H753 for the image itself -
          the ADCs (injected on TIM1's TRGO2), the AFE from the electronics and
          LTspice, the A1335, the BNO085, the plant, the STO chain
          (world/src/world_sto.c on PA10's edges, the master's pilot, PE15
          and TIM1's break), the RS485 echo, the console's wire, each UART at
          the rate its RCC and BRR give it (tools/emu, tests/test_emulator.py)
comms/    cmd.c tables -> cmd_<device>.c handlers (rd_t in, wr_t out, wire.c)
          link.c: which port, console or Modbus; dev_uart.c: the only USART code
          cmd_length.c: request-length oracle for modbus_rtu.c
modbus/   crc, slave, rtu, map          portable C11, host-tested
drive/    control law + motor model     portable C11, host-tested
thermal/  20-node observer, envelope, online identification
filter/   anti-alias biquad chain
ctrl/     machine.parts and a feedback in C, rows played; ctrl_loop.c that
          loop on a drive, device 12, ticked in the drive's sample - the part's
          (board_ctrl.c) and a stand-in's; host-tested
wbc/      humanoid robot chain on its static model (wbc_model.c, written by
          tools/cores/model.py): poses, Jacobians, M, bias; the loop over it
          (wbc_stack.c the levels and bounds, wbc_solve.c the dual active set),
          asks of R^k into torques by priority within the bounds; the step
          without an active set (wbc_wrench.c, machine.wrench's rows; a
          sole's cone and humanoid robot stride through it, wbc_cone.c); host-tested
aero/     a frame flown from a compiled intent (machine.aerolisp's payload):
          its blend, oscillator and observers, the lift inside its constraints
          (aero.c), where the rotors' axis is asked and the torque there
          (aero_turn.c), the thrusts shared by an active set (aero_share.c);
          a choice a mask (aero_bits.h), no branch; host-tested; on the image
          behind device 14 (cmd_aero.c)
daq/      acquisition engine (ring, window, ladder, tone, live)
shtp/     BNO08X transport
boot/     bootloader: boot_core.c (portable) + boot_main.c (registers),
          own .ld, startup, CMakeLists; second image at sector 0
```

- main(): `Board_Early` (VTOR, ITCM copy), MPU, HAL, I-cache, clocks, MX
  inits, `Board_PwmInit` (gates idle first), `Board_CalInit`,
  `Board_BootInit`, dead time + RS485 baud from the record, ADC offset cal,
  timebase, `link_init`, thermal, drive.
- Loop: `Board_StoKeepalive`, `Board_PowerPoll`, `Board_BootPoll` always;
  IMU/angle/DAQ/thermal polls only when the link is not mid-frame; then
  `link_poll` or `Console_Poll`.
- Adding hardware: a row in `board_io.c` (`s_digital`, `s_parts`) + a probe
  case; `0x6D` answers from it.
- New sources go in the root `CMakeLists.txt`, never
  `cmake/stm32cubemx/` (regenerated).

## Host

Three domains; imports run one way: `coaxial` -> `machine`, `motor`.

| Word | Is |
| --- | --- |
| machine | the executive: actuators over IO nodes, programs, `machine/` |
| motor | the PMSM a board drives, `motor/` |
| board, node | one Coaxial63100 on a bus; as a `machine` node, `coaxial.node` |
| rig | `Coaxial63100`, the library's front door to one board |
| host | the computer this runs on |

```text
motor/              pmsm (Motor, Parameters, TWO_PI, RAD_S_PER_RPM), catalog
                    (5230SL, BENCH_MOTOR), loads (Propeller), sysid; imports
                    nothing here
machine/            any board family, no import of one; modules below
coaxial/            rig.py = Coaxial63100, the front door; cli, errors, memory;
                    node (the family for machine: Coaxial node, joint, surface,
                    rotor, torque); profiles/ (a motor's drive record and
                    stand-in model, JSON)
coaxial/comm/       the wire: transport, codecs, protocol, broker, sessions
coaxial/devices/    one subsystem per functional area: board, afe, gates, boot..
coaxial/acquire/    the rig's task and stream (its mixins), records, reader,
                    clock, filter
coaxial/model/      inverter, thermal network, sensorless, blocks (the
                    control loops as sim blocks around motor.pmsm)
coaxial/control/    motion, commission: procedures on a rig
coaxial/draw/       2D drawings: dials, gauges, cross_section, thermal map
coaxial/graphics/   board renderer (wireframe pipeline + one module per concern);
                    gpu (the raster and a lit mesh on a card, wgpu; the crew
                    where none), gynoid (the figure's body, posed and lit),
                    shapes and lit (a lit mesh's parts, its light and its
                    braille), callouts (framed rows, leaders), quadcopter and
                    scenery (the quad of machine.quad and its course, drawn),
                    inset (a picture in the viewport's corner, the
                    instruments' frame: humanoid robot eyes' view; the quad's carried
                    camera's, onboard)
coaxial/kalman/     estimators: thermal_ident (mirrors thermal_ident.c),
                    observer
coaxial/simulated/  the stand-in, same reply shapes as the board; acquire/
                    drive/ thermal/: the board's law and observer in C, a slot
                    of world/src's each, these their edges
coaxial_mcp/        MCP server (stdio), docs tool
coaxial_ollama/     local-model runner
terminal/           python -m terminal: loader, menu, readout
terminal/pages/     the front page's entries, one module each
terminal/views/     the live views (show_*.py), each runnable on its own
terminal/ui/        what they draw with: stage, chrome (the house HUD: CRT snow, lock,
                    clock, kana tags), screen, console, scroll, recorder (R: a
                    page's rows to a CSV, its counter on the page), ..
tools/dev/          run_tests, pick_tests, counts, host_map, target_map,
                    warm_model, lint (markdownlint + pyright, the hooks),
                    measure_first (the prompt hook: a movement described, measured), ab (every
                    page's reads, emulated against the stand-in and the physics),
                    focus (a suite's tests by name, its watchdog, the relay: a
                    baton a physical core, a queue longest first, as the free
                    commit holds)
tools/target/       build_and_flash, find_board, flash_nodes, session
tools/bench/        one question to the board per script: pulse, switch, ..
tools/thermal/      calibrate, identify, validate, trace
tools/render/       renderer checks against the exporter; ansi2png; attitude (the
                    view at a pose and a moment, to a PNG - no window); page (any
                    page's last frame, simulated, to a PNG)
tools/sim/          the drive core on this host: montecarlo, observer_run;
                    ortho (humanoid robot latent loop as the page runs it, traced; --walk
                    --look humanoid robot walk's look); look (a HUMANOID recording a stage
                    a row); replay (a recording's commands run again, pass for pass); scenarios (each scene placed,
                    commanded and judged, a table); strides, normal, looks (a walk's measures, a
                    woman's band, humanoid robot form); wbc (the whole-body stack
                    standing, shoved); hop (a hop on it: thrown, flown, landed);
                    quad_race (the quad's laps through fixed flights, a search
                    over its plan and its line); streams (a text's streams
                    flown on aero/, ideal rotors; a hover asked from upsets)
tools/cores/        build: the portable cores' gcc build; manifest: a robot's build and
                    figure as its orthokinematic manifest (machine/<robot>.manifest.lisp);
                    drive, thermal, wbc, aero: their ctypes harnesses; stand: the
                    stand-in's slots (world_stand.c, world_drive.c); model:
                    humanoid robot figure as static arrays (wbc/); reach: a leg's reach
                    over its foot as static arrays (machine/); terms: humanoid robot
                    movements' orthopaedic terms as static arrays;
                    fakeboard: comms/, the record and the observer over
                    board/fake as
                    fakeboard://; native: the same with the board layer on
                    board/native, a library copy a board, a limb's boards on
                    one world and one bus, a thread a limb, as native://
tools/emu/          emulator: Renode running the image, its console as frames:// (each
                    write a frame, its length ahead of it)
                    (Renode: RENODE, PATH, or the portable build unpacked under
                    %LOCALAPPDATA%/renode); afe_spice: the AFE's transfer from
                    LTspice (electronic_simulations, a submodule)
tools/notebooks/    the paper builder and make_notebooks
tests/              suites, .counts.json (measured sizes)
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
- Stand-in: five buses (AX, LL, RL, LA, RA), four nodes each; reports
  proto 2.23, firmware "simulated". `test_parity.py` holds it to the board.

### machine/

| Module | Is |
| --- | --- |
| roles | Input, Stream, Output, Controller; Part: Filter, Estimator, Regulator |
| errors | the library's exceptions |
| controller | a Loop of Feedbacks over float channels |
| cyclic | that pass data-oriented: a frame, the parts as steps over rows, one edge |
| parts, panel, wiring | the parts, the panel, their wiring |
| ansi | the palette |
| sequencer | lines or tables: a step waits for its targets or tests, its time a timeout; jumps, routines; check, summary |
| alarms | L H logged; LL HH and stop() trip; the sequencer's hooks |
| nodes | Node, Nodes, FAMILIES |
| machine | Machine, Actuator, fit; node_hz: a feedback its node runs, the host forwarding |
| routines | types: a body of subsystems, a bus each |
| live | fed a line at a time, a buffer, woken with a line, a failsafe |
| simulated | pack, camera |
| virtual | VIRTUAL: a type's joints where told, no board |
| figure | the gynoid's segments, masses and legs' dimensions, humanoid robot stand and humanoid robot squat, leg IK |
| motors | humanoid robot kinematics as dual quaternions: a pose a motor, a joint a screw, a chain their product |
| reach | a leg's reach over its foot from static arrays (reach_rows, laid by tools/cores/reach.py): margins, the flat place landed, two feet's gap; a limb an index |
| terms | humanoid robot movements by their orthopaedic names from static arrays (term_rows, laid by tools/cores/terms.py): a part and a movement, its joint kind, sign and stops |
| flight | on the whole-body stack: which soles stand, told from one pass - each as it bears, an unloaded one beside one that bears or with humanoid robot at rest; one that does not is asked to its landing (reach) |
| ortho | orthokinematics: humanoid robot movement as text - `(stream_inject ..)`, `(defbehavior ..)`: latent targets, a stiffness and a damping a mode, humanoid robot rhythm, task constraints - compiled to rows of numbers (gynoid_mesomorph_medium.lisp hers, examples.lisp the first texts) |
| latent | humanoid robot latent state stepped with no branch: a row laid by a mask, the state a lerp, humanoid robot rhythm one angle; each mode's reference and impedance out |
| modal | humanoid robot modes on humanoid robot body, no branch: humanoid robot soles' places from humanoid robot centre of mass as sums and differences, humanoid robot standing legs' compression, humanoid robot turns; what is sensed, humanoid robot statics and the loop's ask laid flat |
| impedance | humanoid robot modes' springs rendered, no branch: humanoid robot centre of mass's acceleration, a sole in the air, humanoid robot turns, humanoid robot arms and humanoid robot knees - the whole-body loop's ask |
| poise | humanoid robot ZMP into humanoid robot soles' polygon, humanoid robot capture point kept in it; which soles stand |
| stride | humanoid robot stride from humanoid robot drive's phase: when a limb swings, where its sole lands (humanoid robot capture point), its clearance, its rock heel to toe, a standing sole's heel rising |
| sprung | the latent loop run: her world read and told through her edge, her law's pass between; her process, paced to the clock: the one thing that moves her |
| law | her law's pass on her sensors alone: sensed, latent, impedance, wrench (or wbc), the drives' observers under it |
| edge | the one reader of the world for her law - IMU, encoders, sole loads, a skin a segment, insoles - and the one writer of her drives |
| estimate | her pelvis's place and speed of what she measures: the IMU carried, drawn to her bearing insoles |
| touch | what her skin and soles feel: a touch against her going stops it, one along it walked out, her going cut toward what she felt, a trip on an edge |
| worker | her process's session: a pass at a time, each command logged with its epoch and pass, falls recovered; the page's states |
| physics | DYNAMIC: the figure in MuJoCo, a drive a joint |
| gearbox | each drive's gearbox: the rotor free across its play, the box wound up past it, the link turned |
| room | a living room round humanoid robot: glass walls, furniture, a door, a lamp |
| sight | humanoid robot eyes: a stereo pair on humanoid robot head, the disparity by block matching, the map she keeps, its things, humanoid robot way round them |
| gimbal | a camera on a stabilised gimbal: its sight level, after the way its frame goes |
| aerolisp | flight intent as text - `(stream_inject ..)`: latent targets, a stiffness and a damping a mode, an oscillator, task constraints - compiled to aero/'s payloads, a stream laid over the one before |
| intents | the quad's flights as that text, and each stream's seconds |
| aerokin | the quad's kinematics in a vacuum, no branch: a row of `aerobatics`' setpoints and the frame as it flies to aero/'s payload - a collective, its rotors' axis's tilt in the heading the core holds, the rates its own heading's turn gives it |
| pull | the quad's action space, no branch: a rotor's top on its clamp and on its link's volts, the four's on their pack's charge, its reach, its stage's watts - from motor, propeller and pack, its tables laid once |
| floor | the slab, a sill, a patch, a rug, a stair |
| rtu | Modbus RTU: CRC-16, a pass's frames |
| buses | the boards on a bus a limb, a process each in lockstep with the world over a shared block, the host's frames bytes on a socket a bus |
| heat | a drive's board's heat: three nodes, the envelope's derate and trip, reported in its reply; what a drive holds for ever |
| drives | two motors as bought, one box, one board; each joint's drive and where it sits, the parts her build's file names |
| mechatronics | the parts library (`library/`: motors, gearboxes, inverters) and a robot's build of it (`robots/`), as TOML |
| servos | each drive kind's servo - peak, stiffness, damping, armature - of her build's file |
| qp | dense convex QP: dual active set, an interior-point fallback, a strict hierarchy by null space |
| wbc | the whole-body stack: accelerations and corner forces a step, torques after |
| wrench | the stack's step without an active set: each sole's wrench or acceleration and humanoid robot upper body's accelerations by one least squares, four times, clipped |

## Tests

- Sizes live in `host/tests/.counts.json` (measured each run); no document
  quotes them.
- Portable C cores are built with host gcc and driven through ctypes
  (`test_<core>_core.py`).
- `test_structure.py` holds host/ together and the wire's three answers to
  each other: `cmd.h`/`boot.h` vs `protocol.py`, every fixed reply and
  request vs the decoders, PROTOCOL.md's op tables vs the handlers, host
  mirrors of firmware constants vs the C.
- Suites that touch only the stand-in run four at a time (offline gate
  ~142 s); mcp, parity, bench, conformance, live run alone after.
- The gynoid's suites:

| Suite | Holds |
| --- | --- |
| `test_gynoid.py` | humanoid robot body on fantasy boards (`physics.ENVELOPE` 0, their SOA never binding): humanoid robot clothes, humanoid robot look, a leg by its foot, a virtual and a dynamic machine of humanoid robot, humanoid robot skeleton, humanoid robot drives' mounts and gearboxes, humanoid robot kinematics as motors |
| `test_sight.py` | the stereo against the renderer's depth; a look round, posed standing, into humanoid robot map against the room, the chair and the table told, a way planned clear; the page's picture |
| `test_reach.py` | humanoid robot leg's reach: its rows humanoid robot figure's, what they hold within humanoid robot stops, a place landed in MuJoCo by either limb |
| `test_terms.py` | humanoid robot movements' terms: its rows humanoid robot figure's, a term moving humanoid robot in MuJoCo as its name says by either limb, its stops, what she has not |
| `test_gvs.py` | the measures' dimensionless frame (Hof), SO(3)'s geodesic, the gates on them (`tools/sim/gvs.py`) |
| `test_scenarios.py` | humanoid robot scenarios that hold (`tools/sim/scenarios.py`): a scene placed, commanded as a player would, its goal met by the world's truth |
| `test_replays.py` | humanoid robot runs recorded on the page (`tests/replays`), each command at its pass, none ending in a fall; the falls still open in `tests/replays/falls` |
| `test_wbc.py` | the QP solver's KKT, two levels' priority, standing, a nudge held |
| `test_wbc_core.py` | humanoid robot chain in C against MuJoCo: frames, M, bias, Jacobians at random configurations; a step's time |
| `test_wrench.py` | the step without an active set: a step standing beside the stack's, humanoid robot stand, nudges held, a hop |
| `test_flight.py` | which soles stand, from one pass alone; a landing within humanoid robot reach; the loop in C against the python stack with no sole standing; a hop stood on both, three in a row on the loop |
| `test_ortho.py` | a text compiled to rows, what is none refused; no branch in the core or the renderer; humanoid robot state blended and turned; crouched, shifted, bowed, on one sole, hopped; humanoid robot walk on humanoid robot own payloads; the first texts as given; the gym's worker and keys |

- The quad's streams: `test_aero_core.py` holds aero/ to its form - no
  branch in its sources, no flag set from its data in this host's code -,
  its share to the best of every set of rotors held at an end, its
  impedance to the closed form in a vacuum, a wind told from a push
  (`aero_kit.py` its rig); `test_quad_streams.py` the
  text's payloads and the power loop flown, a hover in the tour's air and
  from upsets, the page's routine and the course from their cards' rows;
  `test_quad_pull.py` holds the action space (`machine.pull`) to the
  board's law on the stand-in: a rotor's top, its reach, its stage's
  watts, the four on their pack.
- The stand-in is the emulated world's reference: `tools/dev/ab.py` runs a
  page on both, a process a page, and marks a read missing, a range apart
  over the board seconds both cover, or a number the physics rules out;
  `test_native` holds the board's thermometers, current and legs to the
  world's.
- Tiers: `run_tests.ps1` sells checks by percentage; suites join by
  seconds per check. `touches.TOUCHES` maps a changed path to its suites;
  `CHEAP` ones settle without asking the model.
- CI: `host.yml` runs `--offline` on Python 3.12 and 3.14; `firmware.yml`
  builds both images in Debug and Release, fails on any warning, keeps the
  ELFs.
