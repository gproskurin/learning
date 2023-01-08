project(led_blinky)
cmake_minimum_required(VERSION 3.25)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/ld/stm32f103.ld)
add_definitions(-DTARGET_STM32L152)
set(OBJCOPY arm-none-eabi-objcopy)

set(FIRMWARE_ELF ${PROJECT_NAME}.elf)
set(FIRMWARE_BIN ${PROJECT_NAME}.bin)

add_compile_options(
	-mcpu=cortex-m3 -O2 -ffreestanding
)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -fno-rtti -fno-exceptions -Werror -Wall")

add_custom_target(${FIRMWARE_BIN}
	ALL
	DEPENDS ${FIRMWARE_ELF}
	COMMAND ${OBJCOPY} -Obinary ${CMAKE_BINARY_DIR}/${FIRMWARE_ELF} ${CMAKE_BINARY_DIR}/${FIRMWARE_BIN}
)

add_executable(${FIRMWARE_ELF}
	src/startup.c
	src/main.cc
)

target_link_libraries(${FIRMWARE_ELF}
	-T${LINKER_SCRIPT}
	-nostdlib -nostartfiles
)

set_property(TARGET ${FIRMWARE_ELF}
	PROPERTY LINK_DEPENDS ${LINKER_SCRIPT}
)

set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

