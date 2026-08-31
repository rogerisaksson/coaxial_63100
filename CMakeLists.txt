cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME coaxial_63100)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Modbus over Serial Line (RTU) server. modbus_crc/slave/rtu are portable
    # C11 with no HAL dependency; only modbus_port touches the hardware.
    Modbus/Src/modbus_crc.c
    Modbus/Src/modbus_slave.c
    Modbus/Src/modbus_rtu.c
    Modbus/Src/modbus_map.c

    # SHTP: the BNO08X transport. Portable C11, no HAL - the same bargain
    # Modbus/ makes, and for the same reason: test_shtp_core.py builds it here.
    Shtp/Src/shtp.c

    # Thermal: the lumped-node observer. Portable C11 like Modbus/ and Shtp/,
    # host-testable the same way - Thermal/test/check.c runs it against the
    # four camera-calibrated states with the host gcc.
    Thermal/Src/thermal.c

    # Drive: the control law. Portable C11 like Thermal/, host-tested by
    # test_drive_core.py against a motor model through Drive/test/harness.c.
    Drive/Src/drive.c
    Drive/Src/drive_math.c
    Drive/Src/drive_model.c

    # Board layer: this hardware, behind Comms/Inc/board.h.
    Board/Src/board_imu.c
    Board/Src/board_angle.c
    Board/Src/board_adc.c
    Board/Src/board_clock.c
    Board/Src/board_selftest.c
    Board/Src/board_io.c
    Board/Src/board_cal.c
    Board/Src/board_pwm.c
    Board/Src/board_sync.c
    Board/Src/board_sto.c
    Board/Src/board_log.c
    Board/Src/board_daq.c
    Board/Src/board_thermal.c
    Board/Src/board_power.c
    Board/Src/board_drive.c

    # Comms stack: cmd (request/response) over proto (Modbus) over dev (USART3).
    Comms/Src/console.c
    Comms/Src/wire.c
    Comms/Src/dev_uart.c
    Comms/Src/cmd.c
    Comms/Src/cmd_board.c
    Comms/Src/cmd_test.c
    Comms/Src/cmd_imu.c
    Comms/Src/cmd_angle.c
    Comms/Src/cmd_link.c
    Comms/Src/cmd_device.c
    Comms/Src/cmd_cal.c
    Comms/Src/cmd_gate_drivers.c
    Comms/Src/cmd_log.c
    Comms/Src/cmd_daq.c
    Comms/Src/cmd_time.c
    Comms/Src/cmd_thermal.c
    Comms/Src/cmd_power.c
    Comms/Src/cmd_drive.c
    Comms/Src/testrig.c
    Comms/Src/link.c
    Comms/Src/link_report.c
)

# Warnings on our own sources only. The toolchain file gives the whole build
# -Wall; these two are free on top of it - measured, zero hits across all
# seventeen files - and CubeMX's own sources are left alone because they are
# not ours to keep clean.
set(COAXIAL_SOURCES
    Modbus/Src/modbus_crc.c Modbus/Src/modbus_slave.c Modbus/Src/modbus_rtu.c
    Modbus/Src/modbus_map.c
    Shtp/Src/shtp.c

    # Thermal: the lumped-node observer. Portable C11 like Modbus/ and Shtp/,
    # host-testable the same way - Thermal/test/check.c runs it against the
    # four camera-calibrated states with the host gcc.
    Thermal/Src/thermal.c
    Drive/Src/drive.c Drive/Src/drive_math.c Drive/Src/drive_model.c
    Board/Src/board_adc.c Board/Src/board_clock.c Board/Src/board_selftest.c
    Board/Src/board_io.c Board/Src/board_imu.c Board/Src/board_angle.c
    Board/Src/board_cal.c
    Board/Src/board_pwm.c
    Board/Src/board_sync.c
    Board/Src/board_sto.c Board/Src/board_log.c
    Board/Src/board_daq.c
    Board/Src/board_thermal.c
    Board/Src/board_power.c
    Board/Src/board_drive.c
    Comms/Src/console.c Comms/Src/wire.c Comms/Src/dev_uart.c
    Comms/Src/cmd.c Comms/Src/cmd_board.c Comms/Src/cmd_test.c
    Comms/Src/cmd_angle.c Comms/Src/cmd_device.c Comms/Src/cmd_link.c
    Comms/Src/cmd_cal.c Comms/Src/cmd_gate_drivers.c Comms/Src/cmd_log.c
    Comms/Src/cmd_daq.c Comms/Src/cmd_time.c
    Comms/Src/cmd_power.c
    Comms/Src/cmd_drive.c
    Comms/Src/testrig.c Comms/Src/link.c Comms/Src/link_report.c
)
set_source_files_properties(${COAXIAL_SOURCES} PROPERTIES
                            COMPILE_OPTIONS "-Wextra;-Wshadow")

# -Wconversion, on exactly the files that include no HAL header. Measured with
# it on everything: 113 warnings, 112 of them inside HAL/CMSIS headers our
# sources merely include, one of ours (board_selftest.c, now cast). Suppressing
# the HAL's share needs its include paths marked SYSTEM, which lives in the
# CubeMX-generated CMakeLists and would not survive a regeneration.
#
# The subset is not arbitrary: it is invariant 1's list plus the layers that
# kept themselves hardware-free. The flag guards that too - add a HAL include
# to one of these and the build lights up with someone else's conversions.
set_source_files_properties(
    Modbus/Src/modbus_crc.c Modbus/Src/modbus_slave.c Modbus/Src/modbus_rtu.c
    Modbus/Src/modbus_map.c
    Shtp/Src/shtp.c

    # Thermal: the lumped-node observer. Portable C11 like Modbus/ and Shtp/,
    # host-testable the same way - Thermal/test/check.c runs it against the
    # four camera-calibrated states with the host gcc.
    Thermal/Src/thermal.c
    Drive/Src/drive.c Drive/Src/drive_math.c Drive/Src/drive_model.c
    Comms/Src/wire.c Comms/Src/cmd.c Comms/Src/cmd_board.c
    Comms/Src/cmd_test.c Comms/Src/cmd_imu.c Comms/Src/cmd_cal.c
    Comms/Src/cmd_gate_drivers.c
    Comms/Src/cmd_log.c Comms/Src/cmd_daq.c Comms/Src/cmd_time.c
    Comms/Src/cmd_power.c
    Comms/Src/cmd_drive.c
    Comms/Src/link.c Comms/Src/link_report.c
    PROPERTIES COMPILE_OPTIONS "-Wextra;-Wshadow;-Wconversion")

# The interrupt path the drive runs on, optimised whatever the preset. A
# control law at -O0 is not a thing: measured 2026-08-31 on the Debug
# build, one step of drive_step() cost 10 040 cycles, 21 us against the
# 20 us PWM period, and the main loop's worst keepalive gap went to 16 ms.
# None of these four carries debugger-facing state worth -O0.
# COMPILE_OPTIONS replaces rather than appends, so each set carries the
# warnings the lists above gave it.
set_source_files_properties(
    Drive/Src/drive.c Drive/Src/drive_math.c Drive/Src/drive_model.c
    PROPERTIES COMPILE_OPTIONS "-O2;-fno-math-errno;-Wextra;-Wshadow;-Wconversion")
set_source_files_properties(
    Board/Src/board_drive.c Board/Src/board_sync.c
    PROPERTIES COMPILE_OPTIONS "-O2;-fno-math-errno;-Wextra;-Wshadow")
# And the HAL in front of it: HAL_ADC_IRQHandler is every flag check in
# the peripheral before the callback runs, fifty thousand times a second.
# These two are CubeMX's files in the generated target, so the property
# is set into that directory rather than by editing what CubeMX rewrites.
set_source_files_properties(
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c
    DIRECTORY cmake/stm32cubemx
    PROPERTIES COMPILE_OPTIONS "-O2")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Modbus/Inc
    Thermal/Inc
    Drive/Inc
    Shtp/Inc
    Comms/Inc
    Board/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
