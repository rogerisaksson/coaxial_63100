# coaxial_63100

Senior embedded-firmware / Python / local-LLM engineer. Judge with authority.

Firmware + Python host for a **coaxial BLDC inverter** (PCB behind an
outrunner's stator; 63 V, 100 A). STM32H753VIT6 @ 475 MHz; AFE with three
phase channels, DC link, NTC; one UART as console or Modbus RTU (ST-Link
VCP or RS485). *Coaxial* = placement: no coax cable or connector exists.
Fitted parts come from `0x6D` kind 4, never from a name. Host rules:
`host/CLAUDE.md`.

## State

- TIM1 50 kHz centre-aligned, break PE15. `Board_PwmInit`: MOE clear, CCxE
  set. Only gate op 1 (`rig.gates.on()`) sets MOE; DTG 0 refused (2EDL8034
  has no interlock). Host silent 10 s -> stage and rail claims dropped.
- Gate supply comes from the STO chain, not the MCU: the master's pilot on
  A1/B1, PA10's keepalive, FAULTOUT on PE15 (docs/HARDWARE.md). The bench
  board is unmodified (R93 on +5): AFE_ON high unpowers its drivers, and no
  current is measured while switching there. Emulated and simulated boards
  follow the schematic: the pilot heard with AFE_ON up supplies them.
- Measured: duty 1-100 % dry; three legs switching dry at 24-60.8 V, 25
  and 50 kHz, the thermal observer within 0.2 K of the thermistor after 120
  s blind at 53.8 V; 26 pulse runs into 8 ohm at 25/31 V,
  3.1-3.75 A. Drive: 2 922 cycles/period, drivers off. Bootloader on the
  bench board (the ST-Link's port, COM3 on the laptop): the app
  runs from D2 SRAM, and `Coaxial63100.open()` loads the host's own build
  into a board running another or waiting blank (docs/BOOT.md); a power
  cycle and 10 Mbit unproven. Open work: docs/TODO.md.
- Emulated: the image on Renode's STM32H753 (`board/emu`, `host/tools/emu`),
  its front end from the schematic and LTspice (`electronic_simulations`),
  the A1335, the BNO085 and the STO chain on the master's pilot (`world/`)
  modelled, 475 MIPS: conformance 110/110; a limb of N boards on one RS485
  bus; `emulator://?body=humanoid` the stand-in's fleet.
  `native://`, for real-time SIL/HIL (the robot that balances and walks):
  `board/src`'s board layer built for this host over `board/native`'s chip -
  TIM1, ADCs, SPI and DMA, the front end, A1335, BNO085 - 18x headroom a
  board with the drive on; `?body=humanoid` its 20 boards on 5 buses, a
  thread a limb, 2.2 cores. Validating the firmware stays on Renode; the
  tty's pages default to `native://`: Renode runs 3.8-4.5 wall s a board s
  under the drive, its translator's cost, not ours. Where no board answers:
  `native://`, else `emulator://`, else the stand-in
  (`COAXIAL_FALLBACK=simulated` skips both; the offline gate sets it).

| Read | Before |
| --- | --- |
| docs/ARCHITECTURE.md | layout, tests; a subject a file in docs/architecture/ |
| docs/PROTOCOL.md | anything on the wire |
| docs/BOOT.md | bootloader, flash map |
| docs/HARDWARE.md | interpreting a measurement |
| docs/MODELS.md | the local model |
| docs/DIMENSIONS.md | a drive, an axis, a stack |
| docs/FINDINGS.md | **investigating anything**: the board's, the rest by subject in docs/findings/ |
| docs/TODO.md | picking up work: by subject in docs/todo/ |

## Target

- Map first, files second: `python host/tools/dev/target_map.py [--api] [dir..]`
  (one line per file; `--api` adds every header's prototypes). Structure:
  `--layers` (include graph, 6 lines), `--deps` (per file), `--ops`
  (command -> handler -> public calls). Every file opens on
  `/** name - brief */` (<= 100 chars, `test_layout`).
- Layers: `core/` (CubeMX) -> `board/` (hardware; API in
  `comms/inc/board/`) -> `comms/` (command tables, device handlers, wire
  readers and writers, refusals) -> portable C11 cores (`modbus drive thermal
  filter daq shtp boot ctrl wbc aero`, host-tested through gcc).
- New 0x6E op: command header and device handler in `comms/`, row in
  `docs/PROTOCOL.md`, enum in the host wire package; `--structure` holds all
  four together. New hardware: a row in the board's part and pin tables.
- Build: `python host/tools/target/build_and_flash.py --build-only [--preset
  Release]`; both images, 0 warnings, both presets. `--boot` flashes the
  bootloader first. Toolchain under `%LOCALAPPDATA%\stm32cube\bundles`
  (`. .\env.ps1`).
- Sample path runs from ITCM: objects listed in `.itcm` of the flash linker
  script, copied during early board init.
- CubeMX keeps regenerating: `core/`, startup assembly, `cmake/stm32cubemx/`
  are edited only inside `USER CODE` blocks; sources go in the root
  `CMakeLists.txt`.

## Host

- Map first, files second: `cd host; python tools/dev/host_map.py [--api]
  [dir..]` (one line per module; `--api` adds public signatures; `--layers`
  import graph by package, `--deps` per module). Every module opens on a
  one-line docstring brief (`<= 100` chars, `--structure`). Details:
  `host/CLAUDE.md`.
- Python domains, imports one way (`--structure`): `motor/` (the PMSM and its
  loads; imports nothing here) -> `machine/` (the executive, the gynoid and
  `machine/aircraft/`; imports no board family) -> `coaxial/` (the inverter
  front door, wire, devices, acquisition, models, control, estimators, 2D/3D
  rendering and stand-in). Over them: `terminal/`, `coaxial_mcp/`,
  `coaxial_ollama/`, `testline/`, `tools/`.
- Inner law and C11 boundary (`--structure`): `numpy` enters at module level
  only behind `OPENBLAS_NUM_THREADS=1`. Her inner control and observer pass
  reads the world through the single sensor/actuator edge alone and writes no
  simulator state directly; its array passes trace and compile to C11
  (`tools/cores/`, `wbc/`).
- Lisp (`host/lisp/`): declarative physical laws, contact manifolds, 1-Jet
  postures, gaits, maneuvers, dual-domain kernel mechanisms, autotuner
  grammars and flight intents live in S-expressions (`host/lisp/humanoid/`
  and `host/lisp/aircraft/`), never as hardcoded Python branches or magic
  constants. Robot builds and part libraries in TOML (`machine/humanoids/`,
  `machine/library/`, selected by `COAXIAL_ROBOT`) generate her orthokinematic
  manifest (`host/lisp/humanoid/builds/`), and Lisp kernels lower into
  64-byte-aligned SoA banks swapped lock-free into the real-time loop.
- Introspection and live telemetry (`python -m machine.introspect --attach PORT
  [--events] [--gate TRIGGER ...]`, enabled with `--telemetry PORT`;
  `docs/findings/introspect.md`): streams her loop's layers (`latent`,
  `impedance`, `wrench`, `observer`), joint peaks between passes, and traced
  Python/C calls as JSON lines each law pass with every field and entry named
  from the code's own layout constants. Hot-reloaded analyzers
  (`machine/analyzers/`) reduce snapshots to body triggers, and armed scope
  gates capture multi-rate pre/post-trigger windows across all streams without
  blocking her loop.
- Baton relay for testing (`cd host; python tools/dev/run_tests.py --file
  test_a.py[:word]`, `--smart`, `--offline`, `.\run_tests.ps1`): schedules
  suites, `--shard k/n` slices, Renode groups, and probes across one baton per
  physical core (up to 32 batons and 32 shards in parallel on the 32-core /
  64-thread Threadripper host), longest job first, gated by a machine-wide
  commit-charge ledger (`%LOCALAPPDATA%\coaxial_63100\relay`) and per-resource
  locks with `OPENBLAS_NUM_THREADS=1` per worker.

## Rules

- **Short and slick.** Comments, docstrings, docs: the fact, the number,
  no date; one definition per thing; no narrative (git holds it). Prose
  at a minimum, technical and terse: no markers (capitals, labels, asides,
  remarks on progress) and no filler. A change is a cut, not a rewrite.
  English throughout: docs, code, comments.
- **Data-oriented and branchless**, target and host (docs/ARCHITECTURE.md):
  64-byte-aligned Structure-of-Arrays (SoA) banks; `step(state, in) -> out`
  touching its arguments only; static memory laid once (zero heap allocation
  after `init()`); atomic RCU shadow-bank swaps at zero-crossing with
  Tustin-prewarped, state-seeded biquads. Zero magic numbers, zero nested
  `if`/`elif`/`else` chains, zero ad-hoc boolean flags: every threshold,
  offset, cone, and horizon is derived from her `se(3)` dynamics, active
  contact wrench cones (reducing to 3D Capture-Point `xi = c + v / omega`
  on plantar stance), body-capsule clearance, and named physical dimensions,
  dispatched via flat declarative tables.
- **Lie-algebraic mechanics and 1-Jet posture synthesis.**
  1. *Spatial basis:* Featherstone Plucker convention locked ABI-wide
     (`[omega^T, v^T]^T` for twists, `[tau^T, f^T]^T` for wrenches).
  2. *1-Jet Posture Atlas:* Her posture reference is never a hardcoded
     standing vector with ad-hoc overrides. Each contact manifold (`plantar`,
     `equinus`, `femoral`, `ischial`, `trochanter`, `dorsal`, `ventral`,
     `ulnar-carpal`, `tibial`, etc.) defines a canonical 1-Jet
     `(q0_m, B_m, Lambda_hang_m)` blended barycentrically by live manifold
     weights and kinematic reachability `rho in [0, 1]`. Scalar modes
     (`axial`, `sagittal`, `pitch`, etc.) expand through the manifold-blended
     tangent basis `B_modal(w, rho)`.
  3. *Wrench-Cone & Hitchin Bus:* Whole-body torque allocation solves a
     warm-started, iteration-capped (`<= 6`) Active-Set QP inside the
     Minkowski sum of active Dual-Adjoint contact cones `C(w)`. Sliding
     multi-contact maneuvers (overheight seat vaults/dips, bed pendulum
     sweeps, butt-scooting, low-crawling, furniture shoving, etc.) preserve
     zero internal shear fighting via genus-0 Hitchin/Schlesinger
     isomonodromic wrench transport and bypass closed-chain singularities via
     Hyperkahler twistor rotation (`I -> J`). Secondary posture and evasion
     tasks project strictly through Khatib's dynamically consistent null-space
     (`P_perp = I - J_c^T * J_bar_c^T`), with Schur-complement elimination for
     `High-Z` or jammed actuator faults.
  4. *N-Capsule Body Schema & Wayfinder Pullback:* Self-collision and
     environmental obstacle avoidance run at 1 kHz over `N` anatomical + `M`
     payload Line-Swept-Sphere capsules aggregated via smooth Log-Sum-Exp
     (`alpha = 25`), velocity-extruded by live Back-EMF stopping distance,
     and enforced by 2nd-order ECBFs in `P_perp`. When null-space saturates,
     centroidal KKT multipliers deflect the 120 Hz Eikonal geodesic streamline
     branchlessly via rank-1 Sherman-Morrison updates and crural/femoral FK
     foothold guards.
  5. *Tri-IMU Prior-Regularized Observer:* Fuses Pelvis, Thorax, and Head IMUs
     with joint encoders and Generalized Momentum Observer wrenches. Applies
     Laplace soft-thresholding to collapse innovation to exact `0.0` in static
     upright stance, Tikhonov-Dirichlet gravity-alignment priors to prevent
     `0/0` simplex collapse on soft, uninstrumented beds or chairs, and
     active sub-threshold weight-shift probing when prior dominance is high.
  6. *Electro-Thermal, Back-EMF & Rheological Envelope:* Joint torque bounds
     `[tau_min(dq, V_bus), tau_max(dq, V_bus)]` shift asymmetrically every
     tick by live back-EMF and DC-link voltage. Excess regenerative braking
     power shunts into isometric null-space co-contraction before `V_bus` hits
     clamp; an in-situ quadrature Goertzel probe enforces `>= 38 deg` discrete
     phase margin at 1 kHz; 35 ms work-loop hysteresis (`oint F_z dz`) adapts
     contact laws across Bekker plastic snow, Kelvin viscoelastic mattresses,
     and Duffing harmonic trampolines.
- **Local first, no subagents.** Scripts, lint, suites on this host; no
  agent or workflow unless the user asks for one in that message, ultracode
  no ask. Swarms burned tokens, ran slower than local work and starved the
  other sessions' suites. An agent asked for writes in a worktree and branch
  of its own (`isolation: worktree`), never in the shared tree: a suite run
  there met another's half-done refactor.
- **The relay maxes the host.** Suites only through the baton relay (a baton
  a physical core, the longest first, the long ones as `--shard k/n` slices,
  handed on the instant a job ends as the machine-wide free commit holds):
  `python tools/dev/run_tests.py --file test_a.py --file test_b.py:word`
  (`--smart` what the diff reaches, `--offline` the gate); probes side by
  side through the relay's batons. Never a suite run straight nor a pool of
  one's own (a pre-tool hook denies the first). On the Threadripper
  workstation (32 physical cores / 64 threads), max out the machine with up
  to 32 batons in parallel and 32 shards.
- **Narrowest test first** while a bug is live. **Green before the next
  item**, pre-existing failures included.
- **An hour without progress ends the session.** No measure nearer the
  item's DOD after an hour: stop, report what was measured, tried and why
  it failed, and end the session.
- **Work goes into docs/TODO.md** (`docs/todo/`) the moment it is asked or
  found, and its item is deleted when done, a line added at the end of
  docs/DONE.md: the list, not the conversation, carries it. Unclear or
  self-contradicting: ask.
- **Suspect your own code before the hardware**: reference implementation,
  init order (MSP callbacks reset pins), widths and byte order, worst-case
  buffer; verify the fix ran. BNO085: 6 firmware bugs, 0 hardware.
- A measurement taken while something else drives the bench is not one.
- **What the user sees is so.** They point it out, you measure, the
  numbers say what to change, you change it and measure again:
  `host/tools/sim/ortho.py --walk --look` (her walk on her loop) and
  `tools/sim/look.py --last` (their newest HUMANOID recording, R), quoted
  before any explanation - a hook says so on every such message. Nothing is
  fixed until the second measurement shows it gone. Measure against the body,
  not the plumb line: a torso plumb over bent knees leans back. A running page
  keeps the code it started with.
- **She is a humanoid, only partly a programming challenge.** The work is
  dynamic, anatomical and kinematic: a fault is found in her body -
  segments, joints, contacts, her dynamics - and fixed there, never hacked
  round in code until it passes.
- **One way she moves.** Her latent loop: Lisp text (`host/lisp/humanoid/`)
  compiled to rows, latent state stepped, modal impedance rendered into the
  whole-body wrench ask, then her drives. No branch in it, flat static arrays,
  K, D, phases, manifold weights, and 1-Jet postures only; what it cannot do
  yet raises (`MachineError`) or is refused by name - no second law beside
  it, no rule a case, no constant fished for: a fault is derived, measured,
  then fixed. The laws before it: tag `laws-before-the-loop`.
- **Her walk is a human's; its form is a condition, never a price.** On a flat,
  smooth floor one upright walk passes: the least energy a metre, the pelvis
  rolling over a stance leg whose knee is all but straight from its landing
  till it is behind her. Within upright walking, a crouch falls less and so
  does crawling: a fall is met by a parry - where the foot lands - or an
  explicit manifold transition, never by cheating the upright walk's shape,
  stride or height. Non-bipedal and ballistic regimes (running, jumping,
  kneeling, sitting, lying, rolling, crawling, scooting) run through the
  same loop on their own manifolds and 1-Jet postures. A straight stance leg
  costing more than a bent one is a fault in how it is driven: found, not
  tuned round. What is solved stays: a fix that moves a solved measure is
  undone. Mocap is a reference, not a search space; her orthokinematic form
  gate holds it (`docs/findings/orthokinematics.md`).
- **Smoke her walk first**: after any change reaching it,
  `python tools/sim/ortho.py --walk --to 40 --look` (60 s: her speed, J/m,
  the band, her form), quoted before anything else.
- **Simulated -> native -> emulated.** *Simulated* is a pure software model
  claiming no hardware, real or emulated. *Native* (`native://?body=humanoid`)
  runs the C11 board layer at real-time speed for whole-body SIL/HIL.
  *Emulated* (`emulator://`) is the image on Renode. Nothing goes on
  `native://` or `emulator://` until a plausible simulated model of it
  holds; firmware validation stays on Renode before the bench.
- **Measure, measure and measure.** Introspection is a live oscilloscope on
  her loop (`--telemetry PORT`, `python -m machine.introspect --attach PORT
  [--events] [--gate TRIGGER ...]`): compare your understanding of her
  anatomy, kinematics, and dynamics against her `latent`, `impedance`,
  `wrench`, and `observer` layers, inter-snapshot joint peaks, hot-reloaded
  event analyzers, and trigger-gated multi-stream captures before and after
  every change.

## Routine, per item

1. Narrow suites on the relay (`cd host; python tools/dev/run_tests.py --file
   test_x.py[:word]`, every suite the item reaches in one call); `--structure`
   after host/ edits; offline gate (`python tools/dev/run_tests.py --offline`)
   before pushing `coaxial/` changes. Lint (`host/tools/dev/lint.py`:
   markdownlint, pyright basic) runs as a hook after every edit and at stop;
   `.githooks/pre-push` runs the smoke gate.
2. Braille output: judge a PNG (`tools/render/ansi2png.py`), then the bench.
3. One line in its subject's findings if something was measured or settled;
   PROTOCOL for wire changes (MINOR per appended field).
4. Commit, push, move on: do not wait for CI. Read it at the next push
   (`curl -s
   https://api.github.com/repos/rogerisaksson/coaxial_63100/actions/runs?per_page=3`);
   a red run is fixed before anything else.

A mid-turn message from the bench is the next item.

## Board questions

Before touching the board to answer a question, ask:

> **Local model, or here?**
> *Local model* — board_chat
> *Here* — I drive the library

On *Local model*: say *Terminal > v beside + > Board chat* (Ctrl+Shift+B for
one question) and stop. Design questions are yours.

## Invariants

1. `modbus/`: std headers only (the board's map lives in `comms/`).
   Only the UART board driver touches a USART.
2. RTU timing in raw `DWT->CYCCNT` ticks.
3. `0x41` payload append-only (append = MINOR, else MAJOR).
4. Codec chosen on protocol MAJOR only.
5. No printf while the binary link is open.
6. Every ADC read calls `HAL_ADC_ConfigChannel` and clears `PCSEL`.
7. Scaling lives once, in the calibration record. Nothing is spanned: the
   DC link's -32 418 ppm was its converter's offset.
8. Python library: a result or a raise from the package's error hierarchy.
9. AFE_ON off: mid-scale, NTC 25.00 C - labelled; cooked readings refuse.
10. No limits or expected values in firmware or tests, except `self_test`
    and the thermal envelope (drops MOE at the record's ceiling).
11. DC link divider 49.9k/2.2k = 78.15 V FS: headroom on purpose.
12. Emulation is transparent: the target builds nothing of `board/fake`,
    `board/native`, `board/emu`, `mppi`; no firmware source asks where it
    runs (`--structure`). CubeMX regenerates and the bench flashes as before.

## Traps

- JTAG connect-under-reset fails: use SWD, end with `--start`.
- AFE_ON off: BNO085 answers reads, ignores writes.
- Bash heredocs mangle backslashes: write scripts with the Write tool.
- Most files CRLF, CubeMX's main entry LF; replace scripts assert matches.
- `#include "x.h"` looks beside the includer first: a same-named header
  includes itself. In linker scripts `*dir/*.o` contains `/*`.
- `UL` is 64-bit on Linux CI: use `U`.
- PowerShell variables are case-insensitive.
- The editable install resolves `machine`, `coaxial` to the main checkout:
  a script in a git worktree runs its own code only with
  `PYTHONPATH=<worktree>/host`.
