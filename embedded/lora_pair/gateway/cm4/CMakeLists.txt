###
include("${CMAKE_SOURCE_DIR}/../cmake/freertos.cmake")

set(T_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/../ld/stm32wl55_cpu1.ld)
set(T_DEFINITIONS -DTARGET_STM32WL55 -DTARGET_STM32WL55_CPU1)
set(T_C_CXX_FLAGS ${COMMON_C_CXX_FLAGS} -mcpu=cortex-m4 -mthumb)

set(FIRMWARE_ELF "lorapair_gateway_cm4.elf")
set(FIRMWARE_BIN "lorapair_gateway_cm4.bin")


# FreeRTOS
generate_freertos_targets("CM4_" "GCC_ARM_CM3")
set_target_properties(CM4_freertos_config
	PROPERTIES
	INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}"
	INTERFACE_COMPILE_OPTIONS "${T_C_CXX_FLAGS}"
	INTERFACE_COMPILE_DEFINITIONS "${T_DEFINITIONS}"
)


add_executable(${FIRMWARE_ELF}
	startup.c
	main.cc
	${CMAKE_CURRENT_SOURCE_DIR}/../common/utils.cc
	${STM32_LIB_ROOT}/logging.cc
	${STM32_LIB_ROOT}/my_stdlib.c
	${STM32_LIB_ROOT}/startup.c
)

target_include_directories(${FIRMWARE_ELF} PRIVATE
	${STM32_LIB_ROOT}
	${CMSIS_DEVICE_ROOT}
	${CMSIS_ARM_INCLUDE}
)

target_link_libraries(${FIRMWARE_ELF} PRIVATE
	CM4_freertos_kernel
	-T${T_LINKER_SCRIPT}
	-nostartfiles
	-ffreestanding
	-nostdlib
)

set_target_properties(${FIRMWARE_ELF} PROPERTIES
	COMPILE_DEFINITIONS "${T_DEFINITIONS}"
	COMPILE_OPTIONS "${T_C_CXX_FLAGS}"
	LINK_DEPENDS "${T_LINKER_SCRIPT}"
)

add_custom_target(${FIRMWARE_BIN}
	ALL
	DEPENDS ${FIRMWARE_ELF}
	COMMAND ${OBJCOPY} -O binary ${CMAKE_CURRENT_BINARY_DIR}/${FIRMWARE_ELF} ${CMAKE_CURRENT_BINARY_DIR}/${FIRMWARE_BIN}
)

###
