cmake_minimum_required(VERSION 3.12)

project(fpga_test)

set(SRCS
	${CMAKE_SOURCE_DIR}/src/main.v
	${CMAKE_SOURCE_DIR}/src/sram_test.v
	${CMAKE_SOURCE_DIR}/src/my_lib.v
)
set(PNR_PREPACK ${CMAKE_SOURCE_DIR}/nextpnr_pre_pack.py)
set(PCF ${CMAKE_SOURCE_DIR}/src/ice40hx8k-evb.pcf)
add_custom_command(
	OUTPUT fpga_test.bin
	DEPENDS ${SRCS} ${PCF} ${PNR_PREPACK}
	COMMAND "yosys" ARGS -p 'synth_ice40 -top top -json fpga_test.json' ${SRCS}
	COMMAND "nextpnr-ice40" ARGS --placer-heap-cell-placement-timeout 0 --hx8k --package ct256 --json fpga_test.json --pcf ${PCF} --asc fpga_test.asc --pre-pack ${PNR_PREPACK}
	COMMAND "icetime" ARGS -d hx8k -mtr fpga_test.rpt fpga_test.asc
	COMMAND "icepack" ARGS fpga_test.asc fpga_test.bin
)

add_custom_target(
	bitstream
	ALL
	DEPENDS fpga_test.bin
)
