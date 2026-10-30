# Architecture: Electronics

The board's hardware layer and the boards that stand in for it on this
host: neutral, the chip in C, the image on Renode; the world they read; the
stand-in. The schematic, BOM and placement: `electronics/`; the board as
measured: [HARDWARE](../HARDWARE.md); the layers above:
[firmware](firmware.md). Findings: [emulation](../findings/emulation.md).

## The board layer

`board/src/`, a file a part, its API a header each under
`comms/inc/board/`.

| File | Is |
| --- | --- |
| board_adc.c | the ADC channel table and its reads |
| board_sync.c | phase currents sampled mid-period: TIM1 triggers, three ADCs |
| board_pwm.c | TIM1 gate drivers: duty in, gates out, interlocks |
| board_sto.c | what the board can see of the Safe Torque Off chain |
| board_power.c | reference-counted rails |
| board_io.c | discrete I/O: AFE_ON and PE15, the console-mode request; the parts it answers `0x6D` from |
| board_angle.c | the Allegro A1335 on SPI4 |
| board_imu.c | the BNO08X on SPI2: bytes only, no meaning |
| board_drive.c | the control law on this hardware, once a PWM period |
| board_ctrl.c | the board's loop: ctrl_loop's, its angle the A1335's, its guard the IRQ's |
| board_thermal.c | the thermal observer and its envelope on this hardware |
| board_daq.c, board_log.c | one acquisition task; one measurement ring, drained over the wire in bursts |
| board_cal.c, board_flash.c | the calibration record, portable; its sector, bank 2, sector 7 |
| board_clock.c | identity, clock-tree queries, the cycle-counter timebase |
| board_boot.c | the image header, the DTCM handover slot, the way back to boot |
| board_selftest.c | what the board can prove about itself |
| board_mem.c | memcpy and memset a word a turn, from ITCM |

## Its stand-ins

```text
board/fake/     the API on the host, neutral, under comms/, board_cal.c and
                board_thermal.c for the offline suites (tools/cores/fakeboard.py,
                tests/test_wire.py, the conformance suite through it)
board/native/   the chip in C - TIM1, ADC1-3, GPIO, SPI2/4 and DMA1, the AFE,
                the A1335, the BNO085 - under board/src's stage, triple, meter,
                power, STO, angle, IMU and drive, over board/fake: native:// in
                real time, for SIL/HIL; validation is board/emu's
                (tools/cores/native.py, tests/test_native.py)
board/emu/      the board on Renode's STM32H753 for the image itself - the ADCs
                (injected on TIM1's TRGO2), the AFE from the electronics and
                LTspice, the A1335, the BNO085, the plant, the STO chain
                (world/src/world_sto.c on PA10's edges, the master's pilot,
                PE15 and TIM1's break), the RS485 echo, the console's wire,
                each UART at the rate its RCC and BRR give it (tools/emu,
                tests/test_emulator.py)
world/          what the boards' parts read, in C, built on the host only: the
                loads and body a machine's motors turn (world.c), the front end
                and A1335 (world_parts.c), the BNO085 (world_bno085.c), the STO
                chain at 1 us (world_sto.c), a board's heat as the truth
                (world_heat.c), a stand-in board's drive and heat
                (world_drive.c, world_stand.c), one world and a plant a board
                for the emulator's bridge (world_emu.c)
tools/emu/      emulator: Renode running the image, its console as frames://
                (each write a frame, its length ahead of it) (Renode: RENODE,
                PATH, or the portable build unpacked under
                %LOCALAPPDATA%/renode); afe_spice: the AFE's transfer from
                LTspice (electronic_simulations, a submodule)
tools/cores/    fakeboard: comms/, the record and the observer over board/fake
                as fakeboard://; native: the same with the board layer on
                board/native, a library copy a board, a limb's boards on one
                world and one bus, a thread a limb, as native://; stand: the
                stand-in's slots (world_stand.c, world_drive.c)
coaxial/simulated/  the stand-in, same reply shapes as the board; acquire/
                drive/ thermal/: the board's law and observer in C, a slot of
                world/src's each, these their edges
```

- Stand-in: five buses (AX, LL, RL, LA, RA), four nodes each; reports
  proto 2.23, firmware "simulated". `test_parity.py` holds it to the board.
- The stand-in is the emulated world's reference: `tools/dev/ab.py` runs a
  page on both, a process a page, and marks a read missing, a range apart
  over the board seconds both cover, or a number the physics rules out;
  `test_native` holds the board's thermometers, current and legs to the
  world's; `test_native_sysid` a drive read off its board.
- `machine.rig.modes`: where a machine's boards run - the hardware, a stand-in,
  their images emulated, or none.
