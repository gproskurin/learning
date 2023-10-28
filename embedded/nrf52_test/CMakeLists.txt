cmake_minimum_required(VERSION 3.25)

include("cmake/arm-none-eabi.cmake")

project(nrf52_test C CXX)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(NRFX_ROOT "${CMAKE_SOURCE_DIR}/../nrfx")
set(MDK_ROOT "${NRFX_ROOT}/mdk")

set(target_LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/ld/nrf52dk.ld")
set(target_DEFINITIONS "-DTARGET_NRF52DK")
set(target_C_CXX_ARCH_FLAGS -mcpu=cortex-m4 -mthumb)
set(target_FREERTOS_PORT "GCC_ARM_CM3")

set(FIRMWARE_ELF ${PROJECT_NAME}.elf)
set(FIRMWARE_BIN ${PROJECT_NAME}.bin)

set(CMSIS_ARM_INCLUDE "${CMAKE_SOURCE_DIR}/../CMSIS_5/CMSIS/Core/Include")

set(STM32_LIB_ROOT "${CMAKE_SOURCE_DIR}/../stm32_lib")


set(T_C_CXX_FLAGS
	${target_C_CXX_ARCH_FLAGS}
	-D_POSIX_C_SOURCE=200809
	-g -Og -ffreestanding -Wall -Wno-unused-but-set-variable -Wno-unused-variable
	-fno-builtin-memset -fno-builtin-memcpy -fno-builtin-stpcpy
	${target_DEFINITIONS}
	-I${CMAKE_SOURCE_DIR}/../FreeRTOS-Kernel/include
	-I${CMAKE_SOURCE_DIR}/src
	-I${MDK_ROOT}
	-I${CMSIS_ARM_INCLUDE}
	-I${STM32_LIB_ROOT}
)

set(T_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${T_C_CXX_FLAGS} -std=c++17 -fno-rtti -fno-exceptions)
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
set(FREERTOS_PORT ${target_FREERTOS_PORT})
add_subdirectory("${CMAKE_SOURCE_DIR}/../FreeRTOS-Kernel" "FreeRTOS")
###

add_executable(${FIRMWARE_ELF}
	src/startup.c
	src/main.cc
	#${STM32_LIB_ROOT}/logging.cc
	${STM32_LIB_ROOT}/my_stdlib.c
	#${STM32_LIB_ROOT}/freertos_utils.cc
)

target_link_libraries(${FIRMWARE_ELF}
	freertos_kernel
)

set(link_options
	-T${target_LINKER_SCRIPT}
	-nostartfiles
	-ffreestanding
	-nostdlib
)
set_target_properties(${FIRMWARE_ELF}
	PROPERTIES LINK_OPTIONS "${link_options}"
)

set_target_properties(${FIRMWARE_ELF}
	PROPERTIES COMPILE_OPTIONS "${T_CXX_FLAGS}"
)
set_target_properties(${FIRMWARE_ELF}
	PROPERTIES LINK_DEPENDS ${target_LINKER_SCRIPT}
)
add_custom_target(${FIRMWARE_BIN}
	ALL
	DEPENDS ${FIRMWARE_ELF}
	COMMAND ${OBJCOPY} -O binary ${CMAKE_CURRENT_BINARY_DIR}/${FIRMWARE_ELF} ${CMAKE_CURRENT_BINARY_DIR}/${FIRMWARE_BIN}
)

###
