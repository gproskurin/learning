cmake_minimum_required(VERSION 3.25)

include("${CMAKE_SOURCE_DIR}/../cmake/arm-none-eabi.cmake")
include("${CMAKE_SOURCE_DIR}/../cmake/freertos.cmake")

project(stm32_usb C CXX)
set(CMAKE_VERBOSE_MAKEFILE ON)


set(stm32l072_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/../ld/stm32l072.ld)
set(stm32l072_DEFINITIONS "-DTARGET_STM32L072")
set(stm32l072_C_CXX_ARCH_FLAGS -mcpu=cortex-m0plus -mthumb)
set(stm32l072_FREERTOS_PORT "GCC_ARM_CM0")
set(stm32l072_FREERTOS_PREFIX "STM32L072_")
set(stm32l072_FIRMWARE_ELF ${PROJECT_NAME}_stm32l072.elf)
set(stm32l072_FIRMWARE_BIN ${PROJECT_NAME}_stm32l072.bin)


set(FIRMWARE_ELF ${PROJECT_NAME}.elf)
set(FIRMWARE_BIN ${PROJECT_NAME}.bin)

set(CMSIS_ROOT "${CMAKE_SOURCE_DIR}/../_deps")
set(CMSIS_ARM_INCLUDE "${CMAKE_SOURCE_DIR}/../_deps/CMSIS_6/CMSIS/Core/Include")

set(STM32_LIB_ROOT "${CMAKE_SOURCE_DIR}/../stm32_lib")

set(COMMON_C_CXX_FLAGS
	-Og -g -ffreestanding -Wall -Wno-unused-but-set-variable -Wno-unused-variable
	-fno-builtin-memset -fno-builtin-memcpy -fno-builtin-stpcpy
	$<$<COMPILE_LANGUAGE:CXX>:-std=c++20 -fconcepts-diagnostics-depth=2 -fno-rtti -fno-exceptions>
)

function(generate_targets T)
	set(T_FIRMWARE_ELF "${T}_${FIRMWARE_ELF}")
	set(T_FIRMWARE_BIN "${T}_${FIRMWARE_BIN}")
	set(T_LINKER_SCRIPT "${${T}_LINKER_SCRIPT}")
	set(T_DEFINITIONS "${${T}_DEFINITIONS}")
	set(T_FREERTOS_PORT "${${T}_FREERTOS_PORT}")
	set(T_FREERTOS_PREFIX "${${T}_FREERTOS_PREFIX}")

	set(T_C_CXX_FLAGS
		${${T}_C_CXX_ARCH_FLAGS}
		${COMMON_C_CXX_FLAGS}
		${T_DEFINITIONS}
		-I${CMSIS_ROOT}
		-I${CMSIS_ARM_INCLUDE}
		-I${STM32_LIB_ROOT}
		-I${CMAKE_SOURCE_DIR}/src
	)

	# FreeRTOS
	generate_freertos_targets("${T_FREERTOS_PREFIX}" "${T_FREERTOS_PORT}")
	set_target_properties(${T_FREERTOS_PREFIX}freertos_config
		PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/src;${CMAKE_SOURCE_DIR}/../stm32_lib/freertos"
		INTERFACE_COMPILE_OPTIONS "${T_C_CXX_FLAGS}"
		INTERFACE_COMPILE_DEFINITIONS "${T_DEFINITIONS}"
	)
	###

	add_executable(${T_FIRMWARE_ELF}
		src/startup.c
		src/main.cc
		${STM32_LIB_ROOT}/startup.c
		${STM32_LIB_ROOT}/my_stdlib.c
	)

	set(link_options
		-T${T_LINKER_SCRIPT}
		-nostartfiles
		-ffreestanding
		-nostdlib
	)
	target_link_libraries(${T_FIRMWARE_ELF}
		"${T_FREERTOS_PREFIX}freertos_kernel"
	)
	set_target_properties(${T_FIRMWARE_ELF}
		PROPERTIES LINK_OPTIONS "${link_options}"
	)

	set_target_properties(${T_FIRMWARE_ELF}
		PROPERTIES COMPILE_OPTIONS "${T_C_CXX_FLAGS}"
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
