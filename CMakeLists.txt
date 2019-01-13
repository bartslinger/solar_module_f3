################################################################
# Project information
################################################################
cmake_minimum_required(VERSION 3.5)
project(solar_module_f3)


################################################################
# Toolhain configuration
################################################################

##
# compilator options
##
# specify toolchaing
find_program(ARM_GCC_PATH arm-none-eabi-gcc)
get_filename_component(ARM_GCC_PATH ${ARM_GCC_PATH} DIRECTORY)

#include (CMakeForceCompiler)
set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_ASM_COMPILER "${ARM_GCC_PATH}/arm-none-eabi-gcc")
enable_language(ASM)
set(CMAKE_OBJCOPY "${ARM_GCC_PATH}/arm-none-eabi-objcopy")
set(CMAKE_LINKER "${ARM_GCC_PATH}/arm-none-eabi-ld")
set(CMAKE_C_COMPILER "${ARM_GCC_PATH}/arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "${ARM_GCC_PATH}/arm-none-eabi-g++")

################################################################
# build directory
################################################################
#set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})


################################################################
# Source files locations
################################################################
# include directories
include_directories(
    "Drivers/CMSIS/Device/ST/STM32F3xx/Include"
    "Drivers/CMSIS/Include"
    "Drivers/STM32F3xx_HAL_Driver/Inc"
    "Drivers/STM32F3xx_HAL_Driver/Inc/Legacy"
    "Inc"
)
# add sources
file(GLOB PROJECT_SOURCES
    "Drivers/STM32F3xx_HAL_Driver/Src/*.c"
    "Drivers/STM32F3xx_HAL_Driver/Src/*.cpp"
    "Src/*.c"
    "Src/*.cpp"
)
list(REMOVE_ITEM PROJECT_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_timebase_rtc_alarm_template.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_timebase_rtc_wakeup_template.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_msp_template.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_timebase_tim_template.c"
)
list(APPEND PROJECT_SOURCES
    "startup_stm32f303xe.s"
)


################################################################
# Preprocessor definitions
################################################################
add_definitions(-DSTM32F303xE)
add_definitions(-DUSE_HAL_DRIVER)


################################################################
# Compilation options
################################################################
# debug/optimization flags
set(PROJECT_OPTIMIZATION_FLAGS "-g -O0" CACHE STRING "Optimization/debug flags")
# common flags
set(COMMON_FLAGS "\
    -mcpu=cortex-m4 \
    -mthumb \
    -mfpu=fpv4-sp-d16 \
    -mfloat-abi=hard \
    -Wall \
    -fno-common \
    -fdata-sections \
    -ffunction-sections \
    -fmessage-length=0 \
    ${PROJECT_OPTIMIZATION_FLAGS} \
")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${COMMON_FLAGS}")
# comparator standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 99)
# linker options
set(LINKER_SCRIPT "${PROJECT_SOURCE_DIR}/STM32F303RETx_FLASH.ld")
set(CMAKE_EXE_LINKER_FLAGS "-specs=nano.specs -specs=nosys.specs -Wl,-gc-sections -T ${LINKER_SCRIPT}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map")


################################################################
# Target definition
################################################################
# target definition
add_executable("${PROJECT_NAME}.elf" "${PROJECT_SOURCES}")
# configure bin/hex file generation
set(HEX_FILE ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")
