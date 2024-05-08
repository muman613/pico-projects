project(buzzer-gp18 VERSION 1.0 LANGUAGES C)

pico_sdk_init()

add_executable(${PROJECT_NAME} main.c ../common/src/pwm-utils.c)

# pull in common dependencies
target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_pwm pico_cyw43_arch_none)
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../common/include")

pico_enable_stdio_usb(${PROJECT_NAME} ENABLED)

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})

# add url via pico_set_program_url
# example_auto_set_url(hello_uart)
