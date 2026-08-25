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

    # Board layer: this hardware, behind Comms/Inc/board.h.
    Board/Src/board_adc.c
    Board/Src/board_clock.c
    Board/Src/board_selftest.c
    Board/Src/board_io.c

    # Comms stack: cmd (request/response) over proto (Modbus) over dev (USART3).
    Comms/Src/console.c
    Comms/Src/wire.c
    Comms/Src/dev_usart3.c
    Comms/Src/cmd.c
    Comms/Src/cmd_board.c
    Comms/Src/cmd_test.c
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
    Board/Src/board_adc.c Board/Src/board_clock.c Board/Src/board_selftest.c
    Board/Src/board_io.c
    Comms/Src/console.c Comms/Src/wire.c Comms/Src/dev_usart3.c
    Comms/Src/cmd.c Comms/Src/cmd_board.c Comms/Src/cmd_test.c
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
    Comms/Src/wire.c Comms/Src/cmd.c Comms/Src/cmd_board.c
    Comms/Src/cmd_test.c Comms/Src/link.c Comms/Src/link_report.c
    PROPERTIES COMPILE_OPTIONS "-Wextra;-Wshadow;-Wconversion")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Modbus/Inc
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
