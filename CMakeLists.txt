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