cmake_minimum_required(VERSION 3.17)

set(CMAKE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../dependencies/cmake)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_DIR}")

# Must be set BEFORE including CH32V2.cmake so the linker script is selected correctly
set(CH32_DEVICE "D6" CACHE STRING "CH32V20x device variant: D6, D8, D8W")
set(CH32_SYSCLK "SYSCLK_FREQ_96MHz_HSI" CACHE STRING "System clock frequency define (e.g. SYSCLK_FREQ_96MHz_HSI)")

include(${CMAKE_DIR}/ch32/CH32V2.cmake)
include(${CMAKE_DIR}/common.cmake)

project(esc-bootloader VERSION 2.0 LANGUAGES C ASM)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../dependencies/ch32sdk ch32sdk)

set(ESC_LD_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/../../dependencies/ch32sdk/ld/esc_ch32v20x_D6.ld)

# -----------------------------------------------------------------------------
# add_esc_bootloader_target(current variant mcu_prefix [version])
#   current    -- current rating string: 08A | 10A
#   variant    -- mechanical variant:    S | M
#   mcu_prefix -- MCU platform prefix:  S (WCH/CH32V)
#   version    -- optional version suffix: V1, V2, ...
#
# Resulting deviceId: "GD" + mcu_prefix + current + "12S" + variant [+ version]
#   e.g. GDS08A12SS, GDS10A12SS, GDS08A12SM, GDS08A12SSV1
# -----------------------------------------------------------------------------
function(add_esc_bootloader_target current variant mcu_prefix)
    # Optional 4th argument: version suffix (e.g. V1)
    if(ARGC GREATER 3)
        set(version "${ARGV3}")
        set(target_name "esc-bootloader-${current}-${variant}-${version}")
    else()
        set(version "")
        set(target_name "esc-bootloader-${current}-${variant}")
    endif()

    add_executable(${target_name})

    target_sources(${target_name}
        PRIVATE
            main.c
            bootloader.c
            bootloader.h
            wch/ch32v20x_conf.h
            wch/ch32v20x_it.h
            wch/ch32v20x_it.c
    )

    target_include_directories(${target_name}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/wch
            ${CMAKE_CURRENT_SOURCE_DIR}/../../dependencies/common
    )

    target_link_libraries(${target_name}
        PRIVATE
            ch32sdk
            CH32V
            NANO
            NOSYS
    )

    target_compile_definitions(${target_name} PRIVATE
        ${CH32_SYSCLK}
        FIRMWARE_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
        FIRMWARE_VERSION_MINOR=${PROJECT_VERSION_MINOR}
        ESC_MCU_PREFIX_STR="${mcu_prefix}"
        ESC_CURRENT_STR="${current}"
        ESC_VARIANT_STR="${variant}"
        ESC_VERSION_STR="${version}"
    )

    gcc_add_linker_script(${target_name} PRIVATE ${ESC_LD_SCRIPT})
    gcc_generate_hex_file(${target_name})
    gcc_generate_bin_file(${target_name})
endfunction()

# Targets
add_esc_bootloader_target(08A S S V1)   # GDS08A12SSV1
add_esc_bootloader_target(10A S S V1)   # GDS10A12SSV1
add_esc_bootloader_target(08A M S V1)   # GDS08A12SMV1
