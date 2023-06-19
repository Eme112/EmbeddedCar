# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "~/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Pull in FreeRTOS
include(~/pico/FreeRTOS/FreeRTOS/Source/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(ProyectoFinal C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Blink Test
add_executable(Test Test.c )
target_include_directories(Test PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(Test pico_stdlib FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)
pico_enable_stdio_usb(Test 1)
pico_enable_stdio_uart(Test 0)
pico_add_extra_outputs(Test)

# Add executable. Chasis
add_executable(Chasis Chasis.c)
target_include_directories(Chasis PRIVATE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/lib/uart)
target_link_libraries(Chasis pico_stdlib hardware_gpio hardware_adc hardware_pwm FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)
# pico_enable_stdio_uart(Chasis 0)
# pico_enable_stdio_usb(Chasis 1)
pico_add_extra_outputs(Chasis)


# Add executable. Controller
add_executable(Controller Controller.c lib/ssd1306/ssd1306.c  )
target_include_directories(Controller PRIVATE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/lib/ssd1306  ${CMAKE_CURRENT_LIST_DIR}/lib/uart)
target_link_libraries(Controller pico_stdlib hardware_i2c hardware_adc  FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)
pico_enable_stdio_uart(Controller 0)
pico_enable_stdio_usb(Controller 1)
pico_add_extra_outputs(Controller)

# Add executable. Engine
add_executable(Engine Engine.c)
target_include_directories(Engine PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(Engine pico_stdlib hardware_adc hardware_pwm  FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)
pico_enable_stdio_uart(Engine 0)
pico_enable_stdio_usb(Engine 1)
pico_add_extra_outputs(Engine)



