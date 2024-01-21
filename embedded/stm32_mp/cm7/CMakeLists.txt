###

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/../ld/stm32h745_cm7.ld)

set(FIRMWARE_ELF ${PROJECT_NAME}.elf)
set(FIRMWARE_BIN ${PROJECT_NAME}.bin)

set(ARCH_FLAGS -mcpu=cortex-m7 -mfloat-abi=hard -mthumb -DTARGET_STM32H745_CM7 -DCORE_CM7)
set(COMMON_C_CXX_FLAGS ${COMMON_C_CXX_FLAGS} ${ARCH_FLAGS})

# FreeRTOS
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM
	INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_compile_options(freertos_config
	INTERFACE ${COMMON_C_CXX_FLAGS}
)
set(FREERTOS_PORT "GCC_ARM_CM7")
add_subdirectory("${CMAKE_SOURCE_DIR}/../_deps/FreeRTOS-Kernel" "FreeRTOS")
###

add_executable(${FIRMWARE_ELF}
	startup.c
	main.cc
	${STM32_LIB_ROOT}/logging.cc
	${STM32_LIB_ROOT}/my_stdlib.c
	${STM32_LIB_ROOT}/startup.c
)

target_link_libraries(${FIRMWARE_ELF}
	freertos_kernel
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
