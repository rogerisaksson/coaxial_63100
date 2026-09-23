# The bootloader: a second executable beside the application, built by the same
# presets and sized on the same line (docs/BOOT.md).

string(REPLACE "-T \"${CMAKE_SOURCE_DIR}/STM32H753xx_FLASH.ld\""
               "-T \"${CMAKE_CURRENT_SOURCE_DIR}/STM32H753xx_BOOT.ld\""
               CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
string(REPLACE "-Wl,-Map=${CMAKE_PROJECT_NAME}.map"
               "-Wl,-Map=${CMAKE_PROJECT_NAME}_boot.map"
               CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
# No printf here, so nothing pulls newlib's float formatting - and with it the
# syscalls the application's syscalls.c provides and this image has no use for.
string(REPLACE "-Wl,-u,_printf_float" ""
               CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")

set(BOOT_TARGET ${CMAKE_PROJECT_NAME}_boot)

add_executable(${BOOT_TARGET}
    startup_boot.s
    src/boot_main.c
    src/boot_core.c
    ${CMAKE_SOURCE_DIR}/comms/src/wire.c
    ${CMAKE_SOURCE_DIR}/modbus/src/modbus_crc.c
    ${CMAKE_SOURCE_DIR}/modbus/src/modbus_slave.c
    ${CMAKE_SOURCE_DIR}/modbus/src/modbus_rtu.c
)

# The cores' warnings: -Wconversion on the files that include no CMSIS header,
# as the application does with the HAL - boot_main.c includes core_cm7.h, whose
# cache helpers trip -Wsign-conversion in Release.
set_source_files_properties(
    src/boot_core.c
    ${CMAKE_SOURCE_DIR}/comms/src/wire.c
    ${CMAKE_SOURCE_DIR}/modbus/src/modbus_crc.c
    ${CMAKE_SOURCE_DIR}/modbus/src/modbus_slave.c
    ${CMAKE_SOURCE_DIR}/modbus/src/modbus_rtu.c
    PROPERTIES COMPILE_OPTIONS "-Wextra;-Wshadow;-Wconversion")
set_source_files_properties(src/boot_main.c
    PROPERTIES COMPILE_OPTIONS "-Wextra;-Wshadow")

target_include_directories(${BOOT_TARGET} PRIVATE
    inc
    ${CMAKE_SOURCE_DIR}/comms/inc
    ${CMAKE_SOURCE_DIR}/modbus/inc
    ${CMAKE_SOURCE_DIR}/drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/drivers/CMSIS/Device/ST/STM32H7xx/Include
)

# The device, for CMSIS; the type, for the pin table and the header (boot.h's
# BOOT_TYPE_*): this bootloader is coaxial_63100's.
target_compile_definitions(${BOOT_TARGET} PRIVATE
    STM32H753xx
    BOOT_BOARD=1
)

# Beside the application's image, and relinked when the script changes.
set_target_properties(${BOOT_TARGET} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/STM32H753xx_BOOT.ld)
