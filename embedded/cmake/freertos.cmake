###

function(generate_freertos_targets FREERTOS_TARGETS_PREFIX FREERTOS_TARGETS_PORT)

	add_library(${FREERTOS_TARGETS_PREFIX}freertos_config INTERFACE)

	set(FREERTOS_PREFIX "${FREERTOS_TARGETS_PREFIX}")
	set(FREERTOS_PORT "${FREERTOS_TARGETS_PORT}")
	add_subdirectory("${CMAKE_SOURCE_DIR}/../_deps/FreeRTOS-Kernel" "${FREERTOS_TARGETS_PREFIX}FreeRTOS")

endfunction()

###
