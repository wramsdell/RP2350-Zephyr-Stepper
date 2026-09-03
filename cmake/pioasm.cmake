# Builds pioasm - the Pico SDK's PIO assembler - as a native HOST tool, so
# core1/stepper.pio can be edited directly and reassembled at build time
# instead of hand-copying pre-assembled instruction words into C source.
#
# pioasm's source is vendored in hal_rpi_pico (it ships with the SDK), but
# it must run on the build machine, not be cross-compiled for the RP2350 -
# it's a code generator, not target firmware. So this is a wholly separate,
# native-host CMake sub-build via ExternalProject_Add, deliberately not
# forwarding this project's ARM cross-compiler settings (CMAKE_C_COMPILER
# etc. are left unset here so the sub-build's own fresh `cmake` invocation
# picks up the host's default cc/c++, the same way it would for any
# ordinary native build).

include(ExternalProject)

set(PIOASM_SRC_DIR   ${ZEPHYR_HAL_RPI_PICO_MODULE_DIR}/tools/pioasm)
set(PIOASM_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/pioasm-host)
set(PIOASM_EXECUTABLE ${PIOASM_BUILD_DIR}/pioasm)

ExternalProject_Add(pioasm_host
    SOURCE_DIR    ${PIOASM_SRC_DIR}
    BINARY_DIR    ${PIOASM_BUILD_DIR}
    CMAKE_ARGS    -DPIOASM_VERSION_STRING=zephyr-stepper
    BUILD_BYPRODUCTS ${PIOASM_EXECUTABLE}
    INSTALL_COMMAND ""
)
