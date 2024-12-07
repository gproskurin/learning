###
include("${CMAKE_SOURCE_DIR}/../cmake/freertos.cmake")

set(UGUI_DIR "${CMAKE_SOURCE_DIR}/../_deps/UGUI")
set(UGUI_SRC "${UGUI_DIR}/ugui.c")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/../ld/stm32h745_cm7.ld)

set(FIRMWARE_ELF ${PROJECT_NAME}_cm7.elf)
set(FIRMWARE_BIN ${PROJECT_NAME}_cm7.bin)

set(ARCH_FLAGS -mcpu=cortex-m7 -mfloat-abi=hard -mthumb -DTARGET_STM32H745_CM7 -DCORE_CM7 -DUSE_FONT_8X12)
set(COMMON_C_CXX_FLAGS ${COMMON_C_CXX_FLAGS} ${ARCH_FLAGS})

generate_freertos_targets("CM7_" "GCC_ARM_CM7")
set_target_properties(CM7_freertos_config
	PROPERTIES
	INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR};${CMAKE_SOURCE_DIR}/../stm32_lib/freertos"
	INTERFACE_COMPILE_OPTIONS "${COMMON_C_CXX_FLAGS}"
)

add_executable(${FIRMWARE_ELF}
	startup.c
	main.cc
	fb1.cc
	${STM32_LIB_ROOT}/my_stdlib.c
	${STM32_LIB_ROOT}/startup.c
	${UGUI_SRC}
)

target_link_libraries(${FIRMWARE_ELF}
	CM7_freertos_kernel
)

target_include_directories(${FIRMWARE_ELF}
	PRIVATE ${COMMON_INCLUDES} ${UGUI_DIR}
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
