# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/emerico/Documents/pi/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Pull in FreeRTOS
include(/home/emerico/Documents/SistemasEmbebidos/FreeRTOSv202212.01/FreeRTOS/Source/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(EngineController C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(EngineController EngineController.c )

target_include_directories(EngineController PRIVATE ${CMAKE_CURRENT_LIST_DIR})

pico_set_program_name(EngineController "EngineController")
pico_set_program_version(EngineController "0.1")

pico_enable_stdio_uart(EngineController 0)
pico_enable_stdio_usb(EngineController 1)

# Add the standard library to the build
target_link_libraries(EngineController pico_stdlib FreeRTOS-Kernel FreeRTOS-Kernel-Heap4 hardware_gpio hardware_adc hardware_pwm)

pico_add_extra_outputs(EngineController)

