# cmake/stm32g4.cmake
# STM32G431KB Cortex-M4F specific flags

# MCU definition for headers
add_compile_definitions(STM32G431xx)

# CPU flags for Cortex-M4 with hardware FPU (soft ABI for nano.specs compatibility)
set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

# C flags
set(CMAKE_C_FLAGS "${CPU_FLAGS} -Wall -Wextra -fdata-sections -ffunction-sections")
set(CMAKE_C_FLAGS_DEBUG "-Og -g3 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")

# ASM flags
set(CMAKE_ASM_FLAGS "${CPU_FLAGS} -x assembler-with-cpp")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS "${CPU_FLAGS} -specs=nano.specs -Wl,--gc-sections -Wl,-Map=${PROJECT_NAME}.map")
