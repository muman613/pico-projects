project(project0 C CXX ASM)

# set(CMAKE_C_STANDARD 11)
# set(CMAKE_CXX_STANDARD 17)

# pico_sdk_init()
add_executable(project0
        source/main.cpp
)

target_include_directories(project0 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/source
)

# pico_enable_stdio_uart(project0 1)
pico_enable_stdio_usb(project0 ENABLED)

target_link_libraries(project0 pico_stdlib hardware_i2c FreeRTOS-Kernel FreeRTOS-Kernel-Heap3)
pico_add_extra_outputs(project0)
