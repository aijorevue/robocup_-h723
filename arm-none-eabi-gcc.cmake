set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(_ARM_TOOLCHAIN_HINTS
    "$ENV{ARM_GNU_TOOLCHAIN_BIN}"
    "D:/STM32/arm-gnu-toolchain-15.2.rel1-mingw-w64-i686-arm-none-eabi/bin"
    "C:/MaximSDK/Tools/GNUTools/10.3/bin"
)

find_program(ARM_GCC NAMES arm-none-eabi-gcc.exe arm-none-eabi-gcc
    HINTS ${_ARM_TOOLCHAIN_HINTS})
if(NOT ARM_GCC)
    message(FATAL_ERROR
        "arm-none-eabi-gcc not found. Add its bin directory to PATH or set ARM_GNU_TOOLCHAIN_BIN.")
endif()

get_filename_component(ARM_TOOLCHAIN_BIN "${ARM_GCC}" DIRECTORY)
set(CMAKE_C_COMPILER "${ARM_GCC}")
set(CMAKE_ASM_COMPILER "${ARM_GCC}")
set(CMAKE_CXX_COMPILER "${ARM_TOOLCHAIN_BIN}/arm-none-eabi-g++.exe")
set(CMAKE_OBJCOPY "${ARM_TOOLCHAIN_BIN}/arm-none-eabi-objcopy.exe")
set(CMAKE_SIZE "${ARM_TOOLCHAIN_BIN}/arm-none-eabi-size.exe")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
