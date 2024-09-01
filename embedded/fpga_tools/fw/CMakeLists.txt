cmake_minimum_required(VERSION 3.12)

include("${CMAKE_SOURCE_DIR}/../../_deps/pico-sdk/external/pico_sdk_import.cmake")

project(pico_examples C CXX ASM)

set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(fpga_tool_fw
	main.cc
)

target_link_libraries(fpga_tool_fw
	hardware_gpio
	hardware_spi
	hardware_uart
	pico_stdlib
)

# create map/bin/hex file etc.
pico_add_extra_outputs(fpga_tool_fw)


