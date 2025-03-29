
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

if (APPLE)
	set(tools_prefix_bin "/usr/local/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin/")
	#set(tools_prefix_bin "$ENV{HOME}/tmp/at/arm-gnu-toolchain-13.2.Rel1-darwin-x86_64-arm-none-eabi/bin/")
	#set(tools_prefix_bin "/usr/local/pkg/cross-arm-none-eabi/bin/")
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

