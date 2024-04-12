cmake_minimum_required(VERSION 3.25)

include("${CMAKE_SOURCE_DIR}/../cmake/arm-none-eabi.cmake")

project(bare C CXX)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(stm32wl55cpu1_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/../ld/stm32wl55.ld)
set(stm32wl55cpu1_DEFINITIONS -DTARGET_STM32WL55 -DTARGET_STM32WL55_CPU1)
set(stm32wl55cpu1_C_CXX_ARCH_FLAGS -mcpu=cortex-m4 -mthumb)
set(stm32wl55cpu1_FREERTOS_PORT "GCC_ARM_CM3")

set(stm32wl55cpu2_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/../ld/stm32wl55.ld)
set(stm32wl55cpu2_DEFINITIONS -DTARGET_STM32WL55 -DTARGET_STM32WL55_CPU2)
set(stm32wl55cpu2_C_CXX_ARCH_FLAGS -mcpu=cortex-m0plus -mthumb)
set(stm32wl55cpu2_FREERTOS_PORT "GCC_ARM_CM0")

set(FIRMWARE_ELF ${PROJECT_NAME}.elf)
set(FIRMWARE_BIN ${PROJECT_NAME}.bin)

set(CMSIS_ROOT "${CMAKE_SOURCE_DIR}/../_deps")
set(CMSIS_ARM_INCLUDE "${CMAKE_SOURCE_DIR}/../_deps/CMSIS_5/CMSIS/Core/Include")

set(STM32_LIB_ROOT "${CMAKE_SOURCE_DIR}/../stm32_lib")


function(generate_targets T)
	set(T_FIRMWARE_ELF "${T}_${FIRMWARE_ELF}")
	set(T_FIRMWARE_BIN "${T}_${FIRMWARE_BIN}")
	set(T_LINKER_SCRIPT "${${T}_LINKER_SCRIPT}")
	set(T_DEFINITIONS "${${T}_DEFINITIONS}")

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
	)

	set(T_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${T_C_CXX_FLAGS} -std=c++20 -fconcepts-diagnostics-depth=2 -fno-rtti -fno-exceptions)
	#set(T_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${T_C_CXX_FLAGS})
	set(T_C_FLAGS ${CMAKE_C_FLAGS} ${T_C_CXX_FLAGS})

	# FreeRTOS
	add_library(freertos_config INTERFACE)
	target_include_directories(freertos_config SYSTEM
		INTERFACE "${CMAKE_SOURCE_DIR}/src"
	)
	target_compile_options(freertos_config
		INTERFACE ${T_C_FLAGS}
	)
	set(FREERTOS_PORT ${${T}_FREERTOS_PORT})
	add_subdirectory("${CMAKE_SOURCE_DIR}/../_deps/FreeRTOS-Kernel" "${T}_FreeRTOS")
	###

	add_executable(${T_FIRMWARE_ELF}
		src/startup.c
		src/main.cc
		${STM32_LIB_ROOT}/logging.cc
		${STM32_LIB_ROOT}/my_stdlib.c
		${STM32_LIB_ROOT}/startup.c
	)

	target_link_libraries(${T_FIRMWARE_ELF}
		freertos_kernel
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


generate_targets("stm32wl55cpu1")

###
