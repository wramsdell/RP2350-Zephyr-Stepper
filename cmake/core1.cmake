# Cross-compiles the bare-metal core1 image (no Zephyr/RTOS) with the same
# Zephyr SDK toolchain used for core0, then embeds the raw binary into the
# core0 Zephyr image as a C byte array via Zephyr's generate_inc_file_for_target
# helper. See core1/linker.ld for the fixed RAM region this must match.

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/pioasm.cmake)

set(CORE1_SRC_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/core1)
set(CORE1_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/core1)
set(CORE1_ELF       ${CORE1_BUILD_DIR}/core1.elf)
set(CORE1_BIN       ${CORE1_BUILD_DIR}/core1.bin)
set(CORE1_SOURCES
    ${CORE1_SRC_DIR}/vectors.c
    ${CORE1_SRC_DIR}/start.c
    ${CORE1_SRC_DIR}/core1_main.c
)

# Assemble core1/stepper.pio -> stepper.pio.h with pioasm (see pioasm.cmake),
# so the PIO program can be edited directly instead of hand-copying
# pre-assembled instruction words into C source. -DPICO_NO_HARDWARE=1 below
# makes the generated header skip its "#include hardware/pio.h" and Pico-SDK
# C-SDK helper functions (which we don't use - core1_main.c hand-rolls SM
# init/push against raw registers instead), leaving just the
# stepper_program_instructions[]/stepper_wrap_target/stepper_wrap the
# freestanding build actually needs.
set(CORE1_PIO_HEADER ${CORE1_BUILD_DIR}/stepper.pio.h)

# core1/regs.h only pulls in the plain hardware/regs/*.h headers (pure,
# dependency-free macro files - no #include beyond each other), deliberately
# avoiding hardware/structs/*.h + hardware/address_mapped.h, which chain into
# pico.h and from there into Zephyr-only shim headers
# (zephyr/modules/hal_rpi_pico/pico/config_autogen.h -> zephyr/toolchain.h)
# that don't exist in this freestanding, non-Zephyr build. So core1 only
# needs the one hardware_regs include dir.
set(CORE1_HAL_INCLUDES
    ${ZEPHYR_HAL_RPI_PICO_MODULE_DIR}/src/rp2350/hardware_regs/include
)

file(MAKE_DIRECTORY ${CORE1_BUILD_DIR})

set(CORE1_INCLUDE_FLAGS "")
foreach(dir ${CORE1_HAL_INCLUDES} ${CORE1_SRC_DIR} ${CORE1_BUILD_DIR})
    list(APPEND CORE1_INCLUDE_FLAGS "-I${dir}")
endforeach()

add_custom_command(
    OUTPUT ${CORE1_PIO_HEADER}
    COMMAND ${PIOASM_EXECUTABLE} -o c-sdk ${CORE1_SRC_DIR}/stepper.pio ${CORE1_PIO_HEADER}
    DEPENDS ${PIOASM_EXECUTABLE} ${CORE1_SRC_DIR}/stepper.pio
    COMMENT "core1: assembling stepper.pio -> stepper.pio.h"
    VERBATIM
)

add_custom_command(
    OUTPUT ${CORE1_ELF}
    COMMAND ${CMAKE_C_COMPILER}
            -mcpu=cortex-m33 -mthumb -mabi=aapcs
            -mfpu=fpv5-sp-d16 -mfloat-abi=hard
            -ffreestanding -fno-builtin -nostdlib -nostartfiles
            -fno-pic -fno-pie
            -DPICO_NO_HARDWARE=1
            -Os -g -Wall
            ${CORE1_INCLUDE_FLAGS}
            -T${CORE1_SRC_DIR}/linker.ld
            -o ${CORE1_ELF}
            ${CORE1_SOURCES}
    DEPENDS ${CORE1_SOURCES} ${CORE1_SRC_DIR}/linker.ld ${CORE1_SRC_DIR}/mailbox_proto.h
            ${CORE1_SRC_DIR}/regs.h ${CORE1_PIO_HEADER}
    WORKING_DIRECTORY ${CORE1_BUILD_DIR}
    COMMENT "core1: building bare-metal image"
    VERBATIM
)

add_custom_command(
    OUTPUT ${CORE1_BIN}
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CORE1_ELF} ${CORE1_BIN}
    DEPENDS ${CORE1_ELF}
    COMMENT "core1: extracting raw binary"
    VERBATIM
)

set(CORE1_BLOB_INC ${ZEPHYR_BINARY_DIR}/include/generated/core1_blob.bin.inc)
generate_inc_file_for_target(app ${CORE1_BIN} ${CORE1_BLOB_INC})
