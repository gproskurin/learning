include("cmake/arm-none-eabi.cmake")

project(led_blinky C CXX)
cmake_minimum_required(VERSION 3.25)
set(CMAKE_VERBOSE_MAKEFILE ON)

#set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/ld/stm32f103.ld)
#set(TARGET_DEF "-DTARGET_STM32F103")
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/ld/stm32l152.ld)
set(TARGET_DEF "-DTARGET_STM32L152")

#set(Boost_NO_SYSTEM_PATH ON)
#set(BOOST_ROOT "/data/gpr/src/boost_1_81_0")
#find_package(Boost 1.81.0 REQUIRED)

set(FIRMWARE_ELF ${PROJECT_NAME}.elf)
set(FIRMWARE_BIN ${PROJECT_NAME}.bin)
set(FIRMWARE_IHEX ${PROJECT_NAME}.ihex)

set(CMSIS_ROOT "${CMAKE_SOURCE_DIR}/..")
set(CMSIS_ARM_INCLUDE "${CMAKE_SOURCE_DIR}/../CMSIS_5/CMSIS/Core/Include")


set(C_CXX_FLAGS " \
	-mcpu=cortex-m3 -mthumb -g -ffreestanding -Werror -Wall -Wno-unused-but-set-variable -Wno-unused-variable \
	${TARGET_DEF} \
	-I${CMSIS_ROOT} \
	-I${CMSIS_ARM_INCLUDE} \
")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${C_CXX_FLAGS} -std=c++17 -fno-rtti -fno-exceptions")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_CXX_FLAGS}")
set(CMAKE_ASM_FLAGS "-mthumb")

# FreeRTOS
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM
	INTERFACE "${CMAKE_SOURCE_DIR}/src"
)
set(FREERTOS_PORT "GCC_ARM_CM3")
set(FREERTOS_HEAP "1")
add_subdirectory("${CMAKE_SOURCE_DIR}/../FreeRTOS-Kernel" "FreeRTOS")
###

add_custom_target(${FIRMWARE_BIN}
	ALL
	DEPENDS ${FIRMWARE_ELF}
	COMMAND ${OBJCOPY} -O binary ${CMAKE_BINARY_DIR}/${FIRMWARE_ELF} ${CMAKE_BINARY_DIR}/${FIRMWARE_BIN}
)

add_custom_target(${FIRMWARE_IHEX}
	ALL
	DEPENDS ${FIRMWARE_ELF}
	COMMAND ${OBJCOPY} -O ihex ${CMAKE_BINARY_DIR}/${FIRMWARE_ELF} ${CMAKE_BINARY_DIR}/${FIRMWARE_IHEX}
)

add_executable(${FIRMWARE_ELF}
	src/startup.c
	src/main.cc
	src/logging.cc
	#${CMSIS_ROOT}/cmsis_device_l1/Source/Templates/gcc/startup_stm32l152xe.s
)

target_link_libraries(${FIRMWARE_ELF}
	-T${LINKER_SCRIPT}
	-nostartfiles
	-ffreestanding
	#-nostdlib
	freertos_kernel
)

set_property(TARGET ${FIRMWARE_ELF}
	PROPERTY LINK_DEPENDS ${LINKER_SCRIPT}
)

