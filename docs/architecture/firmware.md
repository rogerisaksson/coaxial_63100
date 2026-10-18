# Architecture: Firmware

The target's layers, its wire's handlers and its portable cores. The board
layer's parts and what stands in for them: [electronics](electronics.md).
The wire: [PROTOCOL](../PROTOCOL.md); the bootloader: [BOOT](../BOOT.md).

```text
core/     CubeMX. main.c: CubeMX code + calls in USER CODE blocks only
board/    this hardware; API in comms/inc/board.h -> comms/inc/board/<x>.h
          (one header per board_<x>.c; board_boot.c -> board/handover.h)
          board/inc/board_hw.h: CubeMX handles, board layer only
          board_flash.c: the record's sector, under the portable board_cal.c
          board/fake/, board/native/, board/emu/: electronics.md
comms/    cmd.c tables -> cmd_<device>.c handlers (rd_t in, wr_t out, wire.c)
          link.c: which port, console or Modbus; dev_uart.c: the only USART code
          cmd_length.c: request-length oracle for modbus_rtu.c
modbus/   crc, slave, rtu, map          portable C11, host-tested
daq/      acquisition engine (ring, window, ladder, tone, live)
shtp/     BNO08X transport
boot/     bootloader: boot_core.c (portable) + boot_main.c (registers),
          own .ld, startup, CMakeLists; second image at sector 0
drivers/  ST's CMSIS and HAL
```

## The portable cores

C11, built with host gcc and driven through ctypes (`host/tools/cores/`,
`test_<core>_core.py`). Device: the `0x6E` device it answers behind on the
image.

| Core | Is | Device | Subject |
| --- | --- | --- | --- |
| `drive/` | control law + motor model | 10 | [drivetrain](drivetrain.md) |
| `filter/` | anti-alias biquad chain | | [drivetrain](drivetrain.md) |
| `ctrl/` | machine.parts and a feedback in C, rows played | 12 | [drivetrain](drivetrain.md) |
| `thermal/` | 20-node observer, envelope, online identification | 8 | [thermals](thermals.md) |
| `wbc/` | humanoid robot chain on its static model, the loop over it, the step without an active set | 13 | [kinematics](kinematics.md), [solvers](solvers.md) |
| `aero/` | a frame flown from a compiled intent | 14 | [quad](quad.md) |

## Boot and the loop

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

## Tools

```text
host/tools/target/  build_and_flash, find_board, flash_nodes, session
host/tools/bench/   one question to the board per script: pulse, switch, ..
```
