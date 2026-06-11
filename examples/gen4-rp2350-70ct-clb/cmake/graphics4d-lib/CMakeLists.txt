# Build libgraphics4d_rp2350.a from a Graphics4D-pico / Workshop5 source tree.
#
#   export PICO_SDK_PATH=...
#   export GRAPHICS4D_SDK=.../vendor/Graphics4D-pico
#   cmake -S cmake/graphics4d-lib -B "$GRAPHICS4D_SDK/build-embassy-cmake" -G Ninja \
#     -DCMAKE_BUILD_TYPE=Release \
#     -DCMAKE_TOOLCHAIN_FILE="$PICO_SDK_PATH/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake"
#   cmake --build "$GRAPHICS4D_SDK/build-embassy-cmake"
#   cp "$GRAPHICS4D_SDK/build-embassy-cmake/libgraphics4d_rp2350.a" "$GRAPHICS4D_SDK/lib/"

cmake_minimum_required(VERSION 3.13)

if(NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "PICO_SDK_PATH is not set")
endif()
if(NOT DEFINED ENV{GRAPHICS4D_SDK})
    message(FATAL_ERROR "GRAPHICS4D_SDK is not set")
endif()

set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
set(GRAPHICS4D_SDK $ENV{GRAPHICS4D_SDK})
set(PICO_BOARD gen4_rp2350_70ct CACHE STRING "Board")

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(graphics4d_rp2350 C CXX ASM)
pico_sdk_init()

file(GLOB_RECURSE GRAPHICS4D_SOURCES
    ${GRAPHICS4D_SDK}/src/*.c
    ${GRAPHICS4D_SDK}/src/*.cpp
    ${GRAPHICS4D_SDK}/src/*.cc
)

if(NOT GRAPHICS4D_SOURCES)
    message(FATAL_ERROR "No C/C++ sources under ${GRAPHICS4D_SDK}/src")
endif()

add_library(graphics4d_rp2350 STATIC ${GRAPHICS4D_SOURCES})
target_include_directories(graphics4d_rp2350 PUBLIC
    ${GRAPHICS4D_SDK}/src
    ${GRAPHICS4D_SDK}/include
    ${CMAKE_CURRENT_LIST_DIR}/../../board
)
target_compile_definitions(graphics4d_rp2350 PUBLIC
    GEN4_RP2350_70CT
    GEN4_RP2350_RGB
)
target_link_libraries(graphics4d_rp2350
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_gpio
    hardware_sync
    hardware_clocks
)
