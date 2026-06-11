# Build Pico SDK object files for Embassy firmware link (no crt0 / app startup).
# Objects are merged into libpico_embassy.a by scripts/build-pico-embassy-lib.sh.

cmake_minimum_required(VERSION 3.13)

if(NOT PICO_BOARD)
    set(PICO_BOARD gen4_rp2350_70ct CACHE STRING "Board")
endif()
if(NOT PICO_PLATFORM)
    set(PICO_PLATFORM rp2350 CACHE STRING "Platform")
endif()

set(PICO_COMMON_LANG_FLAGS
    "-mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16"
    CACHE STRING "ARM Cortex-M33 compile flags" FORCE)
foreach(_LANG IN ITEMS C CXX ASM)
    set(CMAKE_${_LANG}_FLAGS_INIT "${PICO_COMMON_LANG_FLAGS}")
endforeach()

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_embassy_support C CXX ASM)

foreach(_LANG IN ITEMS C CXX ASM)
    string(REGEX REPLACE "-mfloat-abi=softfp" "-mfloat-abi=hard -mfpu=fpv5-sp-d16"
        CMAKE_${_LANG}_FLAGS "${CMAKE_${_LANG}_FLAGS}")
    set(CMAKE_${_LANG}_FLAGS "${CMAKE_${_LANG}_FLAGS}" CACHE STRING "" FORCE)
endforeach()

pico_sdk_init()

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/anchor.c "void anchor(void) {}")
add_executable(pico_embassy_objects ${CMAKE_CURRENT_BINARY_DIR}/anchor.c)

target_link_libraries(pico_embassy_objects
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_gpio
    hardware_sync
    hardware_clocks
    hardware_timer
    hardware_irq
    hardware_flash
    pico_flash
    hardware_xip_cache
    hardware_claim
)
