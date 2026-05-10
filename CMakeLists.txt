cmake_minimum_required(VERSION 3.22)

# Must be set before project()
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-gcc.cmake)

project(stm32g431_nucleo
    VERSION 1.0.0
    LANGUAGES C ASM
)

# MCU-specific compiler/linker flags
include(cmake/stm32g4.cmake)

# Executable name
set(TARGET ${PROJECT_NAME})

# Source files (your code)
set(SOURCES
    src/main.c
    src/init.c
    src/interrupts.c
    src/syscall.c
    drivers/usart.c
    drivers/lidar.c
)

# Vendor/device sources
list(APPEND SOURCES
    device/system_stm32g4xx.c
    device/startup_stm32g431xx.s
)

# Include directories
set(INCLUDES
    include
    src
    drivers
    device
    CMSIS
)

# Create executable
add_executable(${TARGET} ${SOURCES})

target_include_directories(${TARGET} PRIVATE ${INCLUDES})

# Linker script
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/device/STM32G431KBTX_FLASH.ld)
target_link_options(${TARGET} PRIVATE -T${LINKER_SCRIPT})

# Generate .hex and .bin files
add_custom_command(TARGET ${TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}> ${TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}> ${TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET}>
    COMMENT "Building ${TARGET}.hex and ${TARGET}.bin"
)