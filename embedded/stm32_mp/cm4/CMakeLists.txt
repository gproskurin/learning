###
include("${CMAKE_SOURCE_DIR}/../cmake/freertos.cmake")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/../ld/stm32h7_cm4.ld)

set(FIRMWARE_ELF ${PROJECT_NAME}_cm4.elf)
set(FIRMWARE_BIN ${PROJECT_NAME}_cm4.bin)

set(ARCH_FLAGS -mcpu=cortex-m4 -mthumb -DTARGET_STM32H747_CM4 -DCORE_CM4)
set(COMMON_C_CXX_FLAGS ${COMMON_C_CXX_FLAGS} ${ARCH_FLAGS})

generate_freertos_targets("CM4_" "GCC_ARM_CM3")
set_target_properties(CM4_freertos_config
	PROPERTIES
	INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR};${CMAKE_SOURCE_DIR}/../stm32_lib/freertos"
	INTERFACE_COMPILE_OPTIONS "${COMMON_C_CXX_FLAGS}"
)

add_executable(${FIRMWARE_ELF}
	startup.c
	main.cc
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
