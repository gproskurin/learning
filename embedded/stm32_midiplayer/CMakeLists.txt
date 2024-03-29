cmake_minimum_required(VERSION 3.25)

include("${CMAKE_SOURCE_DIR}/../cmake/arm-none-eabi.cmake")

project(midiplayer C CXX)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(stm32l072_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/../ld/stm32l072.ld)
set(stm32l072_DEFINITIONS "-DTARGET_STM32L072")
set(stm32l072_C_CXX_ARCH_FLAGS -mcpu=cortex-m0plus -mthumb)
set(stm32l072_FREERTOS_PORT "GCC_ARM_CM0")

set(FIRMWARE_ELF ${PROJECT_NAME}.elf)
set(FIRMWARE_BIN ${PROJECT_NAME}.bin)
#set(FIRMWARE_IHEX ${PROJECT_NAME}.ihex)

set(CMSIS_ROOT "${CMAKE_SOURCE_DIR}/../_deps")
set(CMSIS_ARM_INCLUDE "${CMAKE_SOURCE_DIR}/../_deps/CMSIS_5/CMSIS/Core/Include")

set(STM32_LIB_ROOT "${CMAKE_SOURCE_DIR}/../stm32_lib")

set(LOOKUP_TABLES_DIR "${CMAKE_BINARY_DIR}/lookup_tables")
set(LOOKUP_TABLES_FILE "${LOOKUP_TABLES_DIR}/lookup_tables.cc.h")
add_custom_command(
	OUTPUT "${LOOKUP_TABLES_FILE}"
	COMMAND "mkdir" ARGS "-p" "--" "${LOOKUP_TABLES_DIR}"
	COMMAND "sh" ARGS "-c" "${CMAKE_SOURCE_DIR}/lookup_gen.py" ">" "${LOOKUP_TABLES_FILE}"
	DEPENDS "${CMAKE_SOURCE_DIR}/lookup_gen.py"
)

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
		-I${CMSIS_ROOT}
		-I${CMSIS_ARM_INCLUDE}
		-I${STM32_LIB_ROOT}
		-I${CMAKE_SOURCE_DIR}/../_deps/FreeRTOS-Kernel/include
		-I${CMAKE_SOURCE_DIR}/src
	)

	set(T_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${T_C_CXX_FLAGS} -std=c++20 -fno-rtti -fno-exceptions)
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
		src/player.cc ${LOOKUP_TABLES_FILE}
		${STM32_LIB_ROOT}/logging.cc
		${STM32_LIB_ROOT}/my_stdlib.c
		${STM32_LIB_ROOT}/startup.c
	)

	set_source_files_properties("src/player.cc"
		PROPERTIES INCLUDE_DIRECTORIES "${CMAKE_BINARY_DIR}/lookup_tables"
	)

	target_link_libraries(${T_FIRMWARE_ELF}
		freertos_kernel
		c
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

	#add_custom_target(${FIRMWARE_IHEX}
	#	ALL
	#	DEPENDS ${FIRMWARE_ELF}
	#	COMMAND ${OBJCOPY} -O ihex ${CMAKE_CURRENT_BINARY_DIR}/${FIRMWARE_ELF} ${CMAKE_CURRENT_BINARY_DIR}/${FIRMWARE_IHEX}
	#)

endfunction()


generate_targets("stm32l072")

###
