# Architecture

## Firmware

```text
core/     CubeMX. main.c: CubeMX code + calls in USER CODE blocks only
board/    this hardware; API in comms/inc/board.h -> comms/inc/board/<x>.h
          (one header per board_<x>.c; board_boot.c -> board/handover.h)
          board/inc/board_hw.h: CubeMX handles, board layer only
comms/    cmd.c tables -> cmd_<device>.c handlers (rd_t in, wr_t out, wire.c)
          link.c: which port, console or Modbus; dev_uart.c: the only USART code
          cmd_length.c: request-length oracle for modbus_rtu.c
modbus/   crc, slave, rtu, map          portable C11, host-tested
drive/    control law + motor model     portable C11, host-tested
thermal/  20-node observer, envelope, online identification
filter/   anti-alias biquad chain
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

```text
coaxial/            rig.py = Coaxial63100, the front door; cli, errors, memory
coaxial/comm/       the wire: transport, crc, codecs, protocol, broker, sessions
coaxial/devices/    one subsystem per functional area (board, afe, gates, boot..)
coaxial/acquire/    acquisition tasks, records, reader, clock, anti-alias design
coaxial/model/      motor, inverter, thermal network, sensorless, sysid
coaxial/control/    loop, motion verbs, commissioning
coaxial/draw/       2D drawings: palette, dials, gauges, machine, thermal map
coaxial/graphics/   board renderer (wireframe pipeline + one module per concern)
coaxial/kalman/     estimators: thermal_ident (mirrors thermal_ident.c), observer
coaxial/simulated/  the stand-in: same reply shapes as the board
coaxial_mcp/        MCP server (stdio), docs tool
coaxial_ollama/     local-model runner
terminal/           python -m terminal: loader, menu, readout
terminal/pages/     the front page's entries, one module each
terminal/views/     the live views (show_*.py), each runnable on its own
terminal/ui/        what they draw with: stage, screen, console, scroll, ..
tools/dev/          run_tests, pick_tests, host_map, target_map, warm_model
tools/target/       build_and_flash, find_board, flash_nodes, session
tools/bench/        one question to the board per script: pulse, switch, ..
tools/thermal/      calibrate, identify, validate, trace
tools/render/       renderer checks against the exporter; ansi2png
tools/sim/          the drive core on this machine: montecarlo, observer_run
tools/notebooks/    the paper builder and make_notebooks
tests/              suites, counts.py, .counts.json (measured sizes)
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
  proto 2.8, firmware "simulated". `test_parity.py` holds it to the board.

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
- Tiers: `run_tests.ps1` sells checks by percentage; suites join by
  seconds per check. `TOUCHES` maps a changed path to its suites; `CHEAP`
  ones settle without asking the model.
- CI: `host.yml` runs `--offline` on Python 3.10 and 3.12; `firmware.yml`
  builds both images in Debug and Release, fails on any warning, keeps the
  ELFs.
