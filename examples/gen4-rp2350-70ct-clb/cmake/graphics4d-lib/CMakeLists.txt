# Embassy fallback: build libgraphics4d_rp2350.a from a Graphics4D-pico tree.
#
# Prefer Graphics4D-pico's own CMakeLists.txt — this wrapper is only used when
# that file is missing.

cmake_minimum_required(VERSION 3.13)

if(DEFINED GRAPHICS4D_SDK_PATH)
    set(GRAPHICS4D_SDK "${GRAPHICS4D_SDK_PATH}")
elseif(DEFINED ENV{GRAPHICS4D_SDK})
    set(GRAPHICS4D_SDK "$ENV{GRAPHICS4D_SDK}")
else()
    message(FATAL_ERROR "GRAPHICS4D_SDK is not set (export GRAPHICS4D_SDK or pass -DGRAPHICS4D_SDK_PATH=...)")
endif()

if(DEFINED PICO_SDK_PATH)
    set(PICO_SDK "${PICO_SDK_PATH}")
elseif(DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK "$ENV{PICO_SDK_PATH}")
else()
    message(FATAL_ERROR "PICO_SDK_PATH is not set")
endif()

if(NOT PICO_BOARD)
    set(PICO_BOARD gen4_rp2350_70ct CACHE STRING "Board")
endif()

include(${PICO_SDK}/external/pico_sdk_import.cmake)

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
