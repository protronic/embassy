# Embassy build: compile Graphics4D-pico sources into libgraphics4d_rp2350.a
# without the Workshop5 parent-project layout (../pico_sdk_import.cmake).

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

set(PICO_BOARD_HEADER_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../board" CACHE PATH "gen4_rp2350_70ct.h")

include(${PICO_SDK}/external/pico_sdk_import.cmake)

project(graphics4d_rp2350 C CXX ASM)
pico_sdk_init()

file(GLOB_RECURSE ALL_GRAPHICS4D_SOURCES CONFIGURE_DEPENDS
    "${GRAPHICS4D_SDK}/src/*.c"
    "${GRAPHICS4D_SDK}/src/*.cpp"
    "${GRAPHICS4D_SDK}/src/*.cc"
)

# Workshop5 ships FatFS/SDIO as a sibling tree under src/. Embassy only needs
# display + PSRAM support for LVGL scan-out.
set(GRAPHICS4D_SOURCES "")
foreach(src ${ALL_GRAPHICS4D_SOURCES})
    if(src MATCHES "no-OS-FatFS|/FatFS/|/ff_")
        continue()
    endif()
    list(APPEND GRAPHICS4D_SOURCES "${src}")
endforeach()

if(NOT GRAPHICS4D_SOURCES)
    message(FATAL_ERROR "No C/C++ sources under ${GRAPHICS4D_SDK}/src after filtering")
endif()

message(STATUS "Graphics4D embassy lib: ${CMAKE_CURRENT_LIST_DIR}")
message(STATUS "  SDK: ${GRAPHICS4D_SDK}")
message(STATUS "  sources: ${CMAKE_CURRENT_LIST_DIR} -> ${CMAKE_CURRENT_LIST_DIR}")
list(LENGTH GRAPHICS4D_SOURCES _n_src)
message(STATUS "  compiling ${_n_src} source file(s)")

add_library(graphics4d_rp2350 STATIC ${GRAPHICS4D_SOURCES})
target_include_directories(graphics4d_rp2350 PUBLIC
    "${GRAPHICS4D_SDK}/src"
    "${GRAPHICS4D_SDK}/include"
    "${CMAKE_CURRENT_LIST_DIR}/../../board"
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
    hardware_timer
    hardware_irq
    pico_flash
    hardware_flash
    hardware_xip_cache
)
