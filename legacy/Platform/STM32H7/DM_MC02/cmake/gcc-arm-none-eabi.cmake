set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

# Use an explicit toolchain root so CLion does not fall back to bundled MinGW.
if(NOT ARM_GCC_ROOT)
    if(DEFINED ENV{ARM_GCC_ROOT})
        set(ARM_GCC_ROOT "$ENV{ARM_GCC_ROOT}")
    elseif(CMAKE_HOST_WIN32)
        set(ARM_GCC_ROOT "C:/Users/pomelo/Documents/Codex/stm32-tools/gcc-15.2.rel1")
    else()
        set(ARM_GCC_ROOT "$ENV{HOME}/.local/opt/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi")
    endif()
endif()
file(TO_CMAKE_PATH "${ARM_GCC_ROOT}" ARM_GCC_ROOT)
set(CMAKE_TRY_COMPILE_PLATFORM_VARIABLES ARM_GCC_ROOT)
set(TOOLCHAIN_PREFIX "${ARM_GCC_ROOT}/bin/arm-none-eabi-")

if(CMAKE_HOST_WIN32)
    set(TOOLCHAIN_EXECUTABLE_SUFFIX ".exe")
else()
    set(TOOLCHAIN_EXECUTABLE_SUFFIX "")
endif()

set(CMAKE_C_COMPILER                "${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_EXECUTABLE_SUFFIX}")
set(CMAKE_ASM_COMPILER              "${CMAKE_C_COMPILER}")
set(CMAKE_CXX_COMPILER              "${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_EXECUTABLE_SUFFIX}")
set(CMAKE_LINKER                    "${TOOLCHAIN_PREFIX}ld${TOOLCHAIN_EXECUTABLE_SUFFIX}")
set(CMAKE_AR                        "${TOOLCHAIN_PREFIX}ar${TOOLCHAIN_EXECUTABLE_SUFFIX}")
set(CMAKE_RANLIB                    "${TOOLCHAIN_PREFIX}ranlib${TOOLCHAIN_EXECUTABLE_SUFFIX}")
set(CMAKE_OBJCOPY                   "${TOOLCHAIN_PREFIX}objcopy${TOOLCHAIN_EXECUTABLE_SUFFIX}")
set(CMAKE_SIZE                      "${TOOLCHAIN_PREFIX}size${TOOLCHAIN_EXECUTABLE_SUFFIX}")

foreach(REQUIRED_TOOL CMAKE_C_COMPILER CMAKE_CXX_COMPILER CMAKE_ASM_COMPILER
                      CMAKE_LINKER CMAKE_AR CMAKE_OBJCOPY CMAKE_SIZE)
    if(NOT EXISTS "${${REQUIRED_TOOL}}")
        message(FATAL_ERROR "Missing Arm GNU tool: ${${REQUIRED_TOOL}}")
    endif()
endforeach()

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32H723xG_flash.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
