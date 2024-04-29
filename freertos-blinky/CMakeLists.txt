project(freertos-blinky C CXX ASM)

# set(CMAKE_C_STANDARD 11)
# set(CMAKE_CXX_STANDARD 17)
# pico_sdk_init()
add_executable(${PROJECT_NAME} source/main.cpp)

# Add source directory to find FreeRTOSConfig.h
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/source)

# pico_enable_stdio_uart(project0 1)
pico_enable_stdio_usb(${PROJECT_NAME} ENABLED)

target_link_libraries(${PROJECT_NAME} pico_stdlib pico_cyw43_arch_none hardware_i2c FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)
pico_add_extra_outputs(${PROJECT_NAME})
