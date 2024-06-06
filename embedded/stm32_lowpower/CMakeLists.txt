cmake_minimum_required(VERSION 3.25)

include("${CMAKE_SOURCE_DIR}/../cmake/arm-none-eabi.cmake")

project(lowpower C CXX)
set(CMAKE_VERBOSE_MAKEFILE ON)


set(stm32l072_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/../ld/stm32l072.ld)
set(stm32l072_DEFINITIONS "-DTARGET_STM32L072")
set(stm32l072_C_CXX_ARCH_FLAGS -mcpu=cortex-m0plus -mthumb)

set(stm32wl55cpu1_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/../ld/stm32wl55_cpu1.ld)
set(stm32wl55cpu1_DEFINITIONS -DTARGET_STM32WL55 -DTARGET_STM32WL55_CPU1)
set(stm32wl55cpu1_C_CXX_ARCH_FLAGS -mcpu=cortex-m4 -mthumb)

set(stm32wl55cpu2_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/../ld/stm32wl55_cpu2.ld)
set(stm32wl55cpu2_DEFINITIONS -DTARGET_STM32WL55 -DTARGET_STM32WL55_CPU2 -DCORE_CM0PLUS)
set(stm32wl55cpu2_C_CXX_ARCH_FLAGS -mcpu=cortex-m0plus -mthumb)

set(stm32f103_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/../ld/stm32f103.ld)
set(stm32f103_DEFINITIONS "-DTARGET_STM32F103")
set(stm32f103_C_CXX_ARCH_FLAGS -mcpu=cortex-m3 -mthumb)


set(FIRMWARE_ELF ${PROJECT_NAME}.elf)
set(FIRMWARE_BIN ${PROJECT_NAME}.bin)

set(CMSIS_ROOT "${CMAKE_SOURCE_DIR}/../_deps")
set(CMSIS_ARM_INCLUDE "${CMAKE_SOURCE_DIR}/../_deps/CMSIS_6/CMSIS/Core/Include")

set(STM32_LIB_ROOT "${CMAKE_SOURCE_DIR}/../stm32_lib")

set(COMMON_C_CXX_FLAGS
	-D_POSIX_C_SOURCE=200809
	-Og -g -ffreestanding -Wall -Wno-unused-but-set-variable -Wno-unused-variable
	-fno-builtin-memset -fno-builtin-memcpy -fno-builtin-stpcpy
	$<$<COMPILE_LANGUAGE:CXX>:-std=c++20 -fconcepts-diagnostics-depth=2 -fno-rtti -fno-exceptions>
)

function(generate_targets T)
	set(T_FIRMWARE_ELF "${T}_${FIRMWARE_ELF}")
	set(T_FIRMWARE_BIN "${T}_${FIRMWARE_BIN}")
	set(T_LINKER_SCRIPT "${${T}_LINKER_SCRIPT}")
	set(T_DEFINITIONS "${${T}_DEFINITIONS}")

	set(T_C_CXX_FLAGS
		${${T}_C_CXX_ARCH_FLAGS}
		${COMMON_C_CXX_FLAGS}
		${T_DEFINITIONS}
		-I${CMSIS_ROOT}
		-I${CMSIS_ARM_INCLUDE}
		-I${STM32_LIB_ROOT}
		-I${CMAKE_SOURCE_DIR}/src
	)

	add_executable(${T_FIRMWARE_ELF}
		src/startup.c
		src/main.cc
		${STM32_LIB_ROOT}/my_stdlib.c
		${STM32_LIB_ROOT}/startup.c
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


#generate_targets("stm32l072")
generate_targets("stm32wl55cpu1")
#generate_targets("stm32wl55cpu2")
#generate_targets("stm32f103")

###
