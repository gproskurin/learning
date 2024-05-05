###
include("${CMAKE_SOURCE_DIR}/../cmake/freertos.cmake")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/../ld/stm32wl55_cpu1.ld)
set(DEFINITIONS -DTARGET_STM32WL55 -DTARGET_STM32WL55_CPU1)
set(C_CXX_ARCH_FLAGS -mcpu=cortex-m4 -mthumb)

set(C_CXX_FLAGS ${COMMON_C_CXX_FLAGS} ${C_CXX_ARCH_FLAGS})

set(FIRMWARE_ELF "bare_cm4.elf")
set(FIRMWARE_BIN "bare_cm4.bin")


# FreeRTOS
generate_freertos_targets("CM4_" "GCC_ARM_CM3")
set_target_properties(CM4_freertos_config
	PROPERTIES
	INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}"
	INTERFACE_COMPILE_OPTIONS "${C_CXX_FLAGS}"
	INTERFACE_COMPILE_DEFINITIONS "${DEFINITIONS}"
)

add_executable(${FIRMWARE_ELF}
	startup.c
	main.cc
	${STM32_LIB_ROOT}/logging.cc
	${STM32_LIB_ROOT}/my_stdlib.c
	${STM32_LIB_ROOT}/startup.c
)

target_link_libraries(${FIRMWARE_ELF}
	CM4_freertos_kernel
)

target_include_directories(${FIRMWARE_ELF}
	PRIVATE ${COMMON_INCLUDES}
)

set(link_options
	-T${LINKER_SCRIPT}
	-nostartfiles
	-ffreestanding
	-nostdlib
)
set_target_properties(${FIRMWARE_ELF}
	PROPERTIES LINK_OPTIONS "${link_options}"
)

set_target_properties(${FIRMWARE_ELF}
	PROPERTIES COMPILE_OPTIONS "${COMMON_C_CXX_FLAGS}"
)
set_target_properties(${FIRMWARE_ELF}
	PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT}
)
add_custom_target(${FIRMWARE_BIN}
	ALL
	DEPENDS ${FIRMWARE_ELF}
	COMMAND ${OBJCOPY} -O binary ${CMAKE_CURRENT_BINARY_DIR}/${FIRMWARE_ELF} ${CMAKE_CURRENT_BINARY_DIR}/${FIRMWARE_BIN}
)

###
