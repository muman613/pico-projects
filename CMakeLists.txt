cmake_minimum_required(VERSION 3.13)

# set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# # Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

# # Pull in FreeRTOS
include(FreeRTOS_Kernel_import.cmake)
project(myproject)

pico_sdk_init()

add_subdirectory(project0)
add_subdirectory(led_fade)

add_subdirectory(hello_uart)
add_subdirectory(bus_scan)
add_subdirectory(wifi-scan)
add_subdirectory(button-gp20)
add_subdirectory(freertos-blinky)
add_subdirectory(buzzer-gp18)
