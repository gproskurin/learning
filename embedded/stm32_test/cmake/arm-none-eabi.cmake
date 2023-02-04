
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

if (${UNIX} AND ${APPLE})
	set(tools_prefix_bin "/usr/local/pkg/cross-arm-none-eabi/bin/")
else()
	set(tools_prefix_bin "")
endif()
set(CMAKE_C_COMPILER "${tools_prefix_bin}arm-none-eabi-gcc")
set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_C_COMPILER_FORCED TRUE)

set(CMAKE_CXX_COMPILER "${tools_prefix_bin}arm-none-eabi-g++")
set(CMAKE_CXX_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

set(OBJCOPY "${tools_prefix_bin}arm-none-eabi-objcopy")


set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

