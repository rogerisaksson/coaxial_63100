cmake_minimum_required(VERSION 3.22)

# This file is generated only once, and is not re-generated if converter is
# called multiple times.

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
    # Modbus over Serial Line (RTU) server.
    modbus/src/modbus_crc.c
    modbus/src/modbus_slave.c
    modbus/src/modbus_rtu.c
    comms/src/modbus_map.c

    # SHTP: the BNO08X transport.
    shtp/src/shtp.c

    # Thermal: the lumped-node observer.
    thermal/src/thermal.c
    thermal/src/thermal_ident.c

    # DAQ: the acquisition engine - the ring, the summing window, the
    # anti-alias ladder, the tone and the live accumulator.
    daq/src/daq.c

    # Drive: the control law.
    drive/src/drive.c
    drive/src/drive_math.c
    drive/src/drive_model.c
    drive/src/drive_observer.c
    # Filter: the decimating anti-alias chain.
    filter/src/filter.c

    # Board layer: this hardware, behind comms/inc/board.h.
    board/src/board_imu.c
    board/src/board_angle.c
    board/src/board_adc.c
    board/src/board_clock.c
    board/src/board_selftest.c
    board/src/board_io.c
    board/src/board_cal.c
    board/src/board_pwm.c
    board/src/board_sync.c
    board/src/board_sto.c
    board/src/board_log.c
    board/src/board_daq.c
    board/src/board_thermal.c
    board/src/board_power.c
    board/src/board_drive.c
    board/src/board_boot.c

    # Comms stack: cmd (request/response) over proto (Modbus) over dev (USART3).
    comms/src/console.c
    comms/src/wire.c
    comms/src/dev_uart.c
    comms/src/cmd.c
    comms/src/cmd_length.c
    comms/src/cmd_board.c
    comms/src/cmd_test.c
    comms/src/cmd_imu.c
    comms/src/cmd_angle.c
    comms/src/cmd_link.c
    comms/src/cmd_device.c
    comms/src/cmd_cal.c
    comms/src/cmd_gate_drivers.c
    comms/src/cmd_log.c
    comms/src/cmd_daq.c
    comms/src/cmd_time.c
    comms/src/cmd_thermal.c
    comms/src/cmd_power.c
    comms/src/cmd_drive.c
    comms/src/cmd_boot.c
    comms/src/testrig.c
    comms/src/link.c
    comms/src/link_report.c
)

# Warnings on our own sources only.
set(COAXIAL_SOURCES
    modbus/src/modbus_crc.c modbus/src/modbus_slave.c modbus/src/modbus_rtu.c
    comms/src/modbus_map.c
    shtp/src/shtp.c

    # Thermal: the lumped-node observer.
    thermal/src/thermal.c
    thermal/src/thermal_ident.c
    daq/src/daq.c
    drive/src/drive.c drive/src/drive_math.c drive/src/drive_model.c
    drive/src/drive_observer.c
    filter/src/filter.c
    board/src/board_adc.c board/src/board_clock.c board/src/board_selftest.c
    board/src/board_io.c board/src/board_imu.c board/src/board_angle.c
    board/src/board_cal.c
    board/src/board_pwm.c
    board/src/board_sync.c
    board/src/board_sto.c board/src/board_log.c
    board/src/board_daq.c
    board/src/board_thermal.c
    board/src/board_power.c
    board/src/board_drive.c board/src/board_boot.c
    comms/src/console.c comms/src/wire.c comms/src/dev_uart.c
    comms/src/cmd.c comms/src/cmd_board.c comms/src/cmd_test.c
    comms/src/cmd_angle.c comms/src/cmd_device.c comms/src/cmd_link.c
    comms/src/cmd_cal.c comms/src/cmd_gate_drivers.c comms/src/cmd_log.c
    comms/src/cmd_daq.c comms/src/cmd_time.c
    comms/src/cmd_power.c
    comms/src/cmd_drive.c comms/src/cmd_boot.c
    comms/src/testrig.c comms/src/link.c comms/src/link_report.c
)
set_source_files_properties(${COAXIAL_SOURCES} PROPERTIES
                            COMPILE_OPTIONS "-Wextra;-Wshadow")

# -Wconversion, on exactly the files that include no HAL header.
set_source_files_properties(
    modbus/src/modbus_crc.c modbus/src/modbus_slave.c modbus/src/modbus_rtu.c
    comms/src/modbus_map.c
    shtp/src/shtp.c

    # Thermal: the lumped-node observer.
    thermal/src/thermal.c
    thermal/src/thermal_ident.c
    daq/src/daq.c
    drive/src/drive.c drive/src/drive_math.c drive/src/drive_model.c
    drive/src/drive_observer.c
    filter/src/filter.c
    comms/src/wire.c comms/src/cmd.c comms/src/cmd_board.c
    comms/src/cmd_test.c comms/src/cmd_imu.c comms/src/cmd_cal.c
    comms/src/cmd_gate_drivers.c
    comms/src/cmd_log.c comms/src/cmd_daq.c comms/src/cmd_time.c
    comms/src/cmd_power.c
    comms/src/cmd_drive.c comms/src/cmd_boot.c
    comms/src/link.c comms/src/link_report.c
    PROPERTIES COMPILE_OPTIONS "-Wextra;-Wshadow;-Wconversion")

# The interrupt path the drive runs on, optimised whatever the preset.
set_source_files_properties(
    drive/src/drive.c drive/src/drive_math.c drive/src/drive_model.c
    drive/src/drive_observer.c
    # filter.c runs ONCE PER CONVERTER SAMPLE, which is the fastest thing on
    # this board - the boxcar's single add is the whole point of the first
    # stage, and at -O0 it is not a single add.
    filter/src/filter.c
    daq/src/daq.c
    PROPERTIES COMPILE_OPTIONS "-O2;-fno-math-errno;-Wextra;-Wshadow;-Wconversion")
set_source_files_properties(
    board/src/board_drive.c board/src/board_sync.c board/src/board_daq.c
    PROPERTIES COMPILE_OPTIONS "-O2;-fno-math-errno;-Wextra;-Wshadow")
# And the HAL in front of it: HAL_ADC_IRQHandler is every flag check in the
# peripheral before the callback runs, fifty thousand times a second.
set_source_files_properties(
    drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c
    drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c
    DIRECTORY cmake/stm32cubemx
    PROPERTIES COMPILE_OPTIONS "-O2")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    modbus/inc
    daq/inc
    thermal/inc
    drive/inc
    filter/inc
    shtp/inc
    comms/inc
    board/inc
    boot/inc
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

# THE LINKER SCRIPT IS A DEPENDENCY OF THE LINK.
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    LINK_DEPENDS ${CMAKE_SOURCE_DIR}/STM32H753xx_FLASH.ld)

# The bootloader, the second image: sector 0, the shell the application arrives
# through (docs/BOOT.md).
add_subdirectory(boot)
