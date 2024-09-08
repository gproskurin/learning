cmake_minimum_required(VERSION 3.12)

project(fpga_test)

set(SRCS
	src/main.v
	src/ice40hx8k-evb.pcf
)

add_custom_command(
	OUTPUT fpga_test.bin
	DEPENDS ${SRCS}
	COMMAND "yosys" ARGS -p 'synth_ice40 -top top -json fpga_test.json' ${CMAKE_SOURCE_DIR}/src/main.v
	COMMAND "nextpnr-ice40" ARGS --hx8k --package ct256 --json fpga_test.json --pcf ${CMAKE_SOURCE_DIR}/src/ice40hx8k-evb.pcf --asc fpga_test.asc
	COMMAND "icetime" ARGS -d hx8k -mtr fpga_test.rpt fpga_test.asc
	COMMAND "icepack" ARGS fpga_test.asc fpga_test.bin
)

add_custom_target(
	bitstream
	ALL
	DEPENDS fpga_test.bin
)
