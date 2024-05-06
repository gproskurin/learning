###
cmake_minimum_required(VERSION 3.25)
include("${CMAKE_SOURCE_DIR}/../cmake/arm-none-eabi.cmake")
project(lora_pair C CXX)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMSIS_ROOT "${CMAKE_SOURCE_DIR}/../_deps/CMSIS_5")
set(CMSIS_ARM_INCLUDE "${CMSIS_ROOT}/CMSIS/Core/Include")
set(CMSIS_DEVICE_ROOT "${CMSIS_ROOT}/..")
set(STM32_LIB_ROOT "${CMAKE_SOURCE_DIR}/../stm32_lib")

set(COMMON_C_CXX_FLAGS
	-D_POSIX_C_SOURCE=200809
	-g -Og -ffreestanding -Wall -Wno-unused-but-set-variable -Wno-unused-variable
	-fno-builtin-memset -fno-builtin-memcpy -fno-builtin-stpcpy
	$<$<COMPILE_LANGUAGE:CXX>:-std=c++20 -fconcepts-diagnostics-depth=2 -fno-rtti -fno-exceptions>
)


add_subdirectory(gateway)
add_subdirectory(endnode)

###

