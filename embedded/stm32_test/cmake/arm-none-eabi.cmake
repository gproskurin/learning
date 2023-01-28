
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

#set(tools "/usr/local/pkg/cross-arm-none-eabi")
set(tools "/usr")
set(CMAKE_C_COMPILER "${tools}/bin/arm-none-eabi-gcc")
set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_C_COMPILER_FORCED TRUE)

set(CMAKE_CXX_COMPILER "${tools}/bin/arm-none-eabi-g++")
set(CMAKE_CXX_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

set(OBJCOPY "${tools}/bin/arm-none-eabi-objcopy")


set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

