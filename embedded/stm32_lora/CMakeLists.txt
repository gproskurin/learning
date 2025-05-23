cmake_minimum_required(VERSION 3.25)

include("${CMAKE_SOURCE_DIR}/../cmake/arm-none-eabi.cmake")
include("${CMAKE_SOURCE_DIR}/../cmake/freertos.cmake")

project(lora C CXX)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(UGUI_DIR "${CMAKE_SOURCE_DIR}/../_deps/UGUI")
set(UGUI_SRC "${UGUI_DIR}/ugui.c")

set(stm32l072_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/../ld/stm32l072.ld)
set(stm32l072_DEFINITIONS -DTARGET_STM32L072 -DUSE_FONT_8X14)
set(stm32l072_C_CXX_ARCH_FLAGS -mcpu=cortex-m0plus -mthumb)
set(stm32l072_FREERTOS_PORT "GCC_ARM_CM0")
set(stm32l072_FREERTOS_PREFIX "stm32l072_")

set(FIRMWARE_ELF ${PROJECT_NAME}.elf)
set(FIRMWARE_BIN ${PROJECT_NAME}.bin)

set(CMSIS_ROOT "${CMAKE_SOURCE_DIR}/../_deps")
set(CMSIS_ARM_INCLUDE "${CMAKE_SOURCE_DIR}/../_deps/CMSIS_6/CMSIS/Core/Include")

set(STM32_LIB_ROOT "${CMAKE_SOURCE_DIR}/../stm32_lib")


function(generate_targets T)
	set(T_FIRMWARE_ELF "${T}_${FIRMWARE_ELF}")
	set(T_FIRMWARE_BIN "${T}_${FIRMWARE_BIN}")
	set(T_LINKER_SCRIPT "${${T}_LINKER_SCRIPT}")
	set(T_DEFINITIONS "${${T}_DEFINITIONS}")
	set(T_FREERTOS_PORT "${${T}_FREERTOS_PORT}")
	set(T_FREERTOS_PREFIX "${${T}_FREERTOS_PREFIX}")

	set(T_C_CXX_FLAGS
		${${T}_C_CXX_ARCH_FLAGS}
		-D_POSIX_C_SOURCE=200809
		-g -Og -ffreestanding -Wall -Wno-unused-but-set-variable -Wno-unused-variable
		-fno-builtin-memset -fno-builtin-memcpy -fno-builtin-stpcpy
		${T_DEFINITIONS}
		-I${CMAKE_SOURCE_DIR}/..
		-I${CMSIS_ROOT}
		-I${CMSIS_ARM_INCLUDE}
		-I${STM32_LIB_ROOT}
		-I${CMAKE_SOURCE_DIR}/../_deps/FreeRTOS-Kernel/include
		-I${CMAKE_SOURCE_DIR}/src
		-I${UGUI_DIR}
	)

	set(T_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${T_C_CXX_FLAGS} -std=c++20 -fconcepts-diagnostics-depth=2 -fno-rtti -fno-exceptions)
	#set(T_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${T_C_CXX_FLAGS})
	set(T_C_FLAGS ${CMAKE_C_FLAGS} ${T_C_CXX_FLAGS})

	generate_freertos_targets(${T_FREERTOS_PREFIX} ${T_FREERTOS_PORT})
	set_target_properties(${T_FREERTOS_PREFIX}freertos_config
		PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/src;${CMAKE_SOURCE_DIR}/../stm32_lib/freertos"
		INTERFACE_COMPILE_OPTIONS "${T_C_FLAGS}"
	)

	add_executable(${T_FIRMWARE_ELF}
		src/startup.c
		src/main.cc
		src/lora.cc
		${STM32_LIB_ROOT}/my_stdlib.c
		${STM32_LIB_ROOT}/startup.c
		${UGUI_SRC}
	)

	target_link_libraries(${T_FIRMWARE_ELF}
		${T_FREERTOS_PREFIX}freertos_kernel
	)

	set(link_options
		-T${T_LINKER_SCRIPT}
		-nostartfiles
		-ffreestanding
		-nostdlib
	)
	set_target_properties(${T_FIRMWARE_ELF}
		PROPERTIES LINK_OPTIONS "${link_options}"
	)

	set_target_properties(${T_FIRMWARE_ELF}
		PROPERTIES COMPILE_OPTIONS "${T_CXX_FLAGS}"
	)
	set_target_properties(${T_FIRMWARE_ELF}
		PROPERTIES LINK_DEPENDS ${T_LINKER_SCRIPT}
	)
	add_custom_target(${T_FIRMWARE_BIN}
		ALL
		DEPENDS ${T_FIRMWARE_ELF}
		COMMAND ${OBJCOPY} -O binary ${CMAKE_CURRENT_BINARY_DIR}/${T_FIRMWARE_ELF} ${CMAKE_CURRENT_BINARY_DIR}/${T_FIRMWARE_BIN}
	)

endfunction()


generate_targets("stm32l072")

###
