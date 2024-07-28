cmake_minimum_required(VERSION 3.25)

include("${CMAKE_SOURCE_DIR}/../cmake/arm-none-eabi.cmake")
#include("${CMAKE_SOURCE_DIR}/../cmake/freertos.cmake")

set(PICO_SDK_ROOT "${CMAKE_SOURCE_DIR}/../_deps")
set(PICO_SDK_PATH "${PICO_SDK_ROOT}/pico-sdk")
set(PICO_TOOLCHAIN_PATH "${tools_prefix_bin}") # FIXME
include("${PICO_SDK_PATH}/pico_sdk_init.cmake")

project(rp2040_test C)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(ELF2UF2 /Users/gpr/src/learning/embedded/_deps/pico-examples/_BUILD/elf2uf2/elf2uf2)
set(T_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/rp2040.ld)
#set(T_DEFINITIONS "-DTARGET_RP2040")
set(T_C_CXX_ARCH_FLAGS -mcpu=cortex-m0plus -mthumb)
#set(T_FREERTOS_PREFIX "RP2040_CPU1_")

set(T_FIRMWARE_ELF ${PROJECT_NAME}) # pico-sdk spoils .elf suffix
#set(T_FIRMWARE_BIN ${PROJECT_NAME}.bin)
set(T_FIRMWARE_UF2 ${PROJECT_NAME}.uf2)

# PICO
#set(PICO_BARE_METAL "1")
set(PICO_PLATFORM "rp2040")
pico_sdk_init()
#pico_add_subdirectory("${PICO_SDK_ROOT}/pico-sdk/src/common/pico_sync" "ps")
###

#set(STM32_LIB_ROOT "${CMAKE_SOURCE_DIR}/../stm32_lib")


set(T_C_CXX_COMMON_FLAGS
	${T_C_CXX_ARCH_FLAGS}
	-O0 -fpic -Wall -Wno-unused-but-set-variable -Wno-unused-variable
	-fno-builtin-memset -fno-builtin-memcpy -fno-builtin-stpcpy
)
#set(T_PROJECT_FLAGS ${CMAKE_CXX_FLAGS} ${T_C_CXX_ARCH_FLAGS} -std=c++20 -fno-rtti -fno-exceptions)
set(T_PROJECT_FLAGS ${CMAKE_CXX_FLAGS} ${T_C_CXX_ARCH_FLAGS} -std=c2x)

set(FREERTOS_C_FLAGS
	${T_C_CXX_COMMON_FLAGS}
)
set(FREERTOS_INCLUDE
	${CMAKE_CURRENT_SOURCE_DIR}/src
        ${PICO_INCLUDE_DIRS}
        ${PICO_SDK_ROOT}/pico-sdk/src/common/pico_sync/include
        ${PICO_SDK_ROOT}/pico-sdk/src/common/pico_time/include
        ${PICO_SDK_ROOT}/pico-sdk/src/rp2_common/hardware_timer/include
        ${PICO_SDK_ROOT}/pico-sdk/src/rp2_common/hardware_sync/include
)

# FreeRTOS
#generate_freertos_targets(${T_FREERTOS_PREFIX} "GCC_RP2040")
#set_target_properties(${T_FREERTOS_PREFIX}freertos_config
#	PROPERTIES
#	INTERFACE_INCLUDE_DIRECTORIES "${FREERTOS_INCLUDE}"
#	INTERFACE_COMPILE_OPTIONS "${FREERTOS_C_FLAGS}"
#)
###

add_executable(${T_FIRMWARE_ELF}
	src/startup.c
)

#target_include_directories(${T_FIRMWARE_ELF} PRIVATE
#	${STM32_LIB_ROOT}
#	${PICO_INCLUDE_DIRS}
#	${PICO_SDK_ROOT}/pico-sdk/src/common/pico_base/include
#	${PICO_SDK_ROOT}/pico-sdk/src/rp2_common/pico_platform/include
#	${PICO_SDK_ROOT}/pico-sdk/src/rp2040/hardware_regs/include
#	${PICO_SDK_ROOT}/pico-sdk/src/rp2_common/hardware_base/include
#	${CMAKE_BINARY_DIR}/generated/pico_base
#)

#target_link_libraries(${T_FIRMWARE_ELF}
#	${T_FREERTOS_PREFIX}freertos_kernel
#)

set(link_options
	-T${T_LINKER_SCRIPT} --entry 0x20040001
	-nostartfiles
	-ffreestanding
	-nostdlib
)
set_target_properties(${T_FIRMWARE_ELF}
	PROPERTIES LINK_OPTIONS "${link_options}"
)

target_link_libraries(${T_FIRMWARE_ELF}
	hardware_regs_headers
	hardware_structs_headers
)
set_target_properties(${T_FIRMWARE_ELF}
	PROPERTIES COMPILE_OPTIONS "${T_PROJECT_FLAGS}"
)
set_target_properties(${T_FIRMWARE_ELF}
	PROPERTIES LINK_DEPENDS ${T_LINKER_SCRIPT}
)
#add_custom_target(${T_FIRMWARE_BIN}
#	ALL
#	DEPENDS ${T_FIRMWARE_ELF}
#	COMMAND ${OBJCOPY} -O binary ${CMAKE_CURRENT_BINARY_DIR}/${T_FIRMWARE_ELF}.elf ${CMAKE_CURRENT_BINARY_DIR}/${T_FIRMWARE_BIN}
#)

add_custom_target(${T_FIRMWARE_UF2}
	ALL
	DEPENDS ${T_FIRMWARE_ELF}
	COMMAND ${ELF2UF2} -v ${CMAKE_CURRENT_BINARY_DIR}/${T_FIRMWARE_ELF}.elf ${CMAKE_CURRENT_BINARY_DIR}/${T_FIRMWARE_UF2}
)

###
