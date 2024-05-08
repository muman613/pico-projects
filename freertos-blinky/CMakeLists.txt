project(freertos-blinky C CXX ASM)

# set(CMAKE_C_STANDARD 11)
# set(CMAKE_CXX_STANDARD 17)
# pico_sdk_init()
message("${PROJECT_NAME} @ ${CMAKE_CURRENT_LIST_DIR}")

add_executable(${PROJECT_NAME} source/main.cpp ../common/src/pwm-utils.c)

# Add source directory to find FreeRTOSConfig.h
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_LIST_DIR}/source" "${CMAKE_CURRENT_LIST_DIR}/../common/include")

# pico_enable_stdio_uart(project0 1)
pico_enable_stdio_usb(${PROJECT_NAME} ENABLED)

add_compile_definitions(CYW43_HOST_NAME="PICO-W")

target_link_libraries(${PROJECT_NAME} pico_stdlib pico_cyw43_arch_lwip_threadsafe_background
    hardware_pwm hardware_i2c FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)
pico_add_extra_outputs(${PROJECT_NAME})
