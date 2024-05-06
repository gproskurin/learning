###
include("${CMAKE_SOURCE_DIR}/../cmake/freertos.cmake")

set(T_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/../ld/stm32l072.ld)
set(T_DEFINITIONS -DTARGET_STM32L072)
set(T_C_CXX_FLAGS ${COMMON_C_CXX_FLAGS} -mcpu=cortex-m0plus -mthumb)
set(T_FREERTOS_PORT "GCC_ARM_CM0")
set(T_FREERTOS_PREFIX "endnode_stm32l072_")

set(FIRMWARE_ELF lorapair_endnode_stm32l072.elf)
set(FIRMWARE_BIN lorapair_endnode_stm32l072.bin)



# FreeRTOS
generate_freertos_targets(${T_FREERTOS_PREFIX} ${T_FREERTOS_PORT})
set_target_properties(${T_FREERTOS_PREFIX}freertos_config
	PROPERTIES
	INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}"
	INTERFACE_COMPILE_OPTIONS "${T_C_CXX_FLAGS}"
	INTERFACE_COMPILE_DEFINITIONS "${T_DEFINITIONS}"
)


add_executable(${FIRMWARE_ELF}
	startup.c
	main.cc
	lora.cc
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
	${T_FREERTOS_PREFIX}freertos_kernel
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
