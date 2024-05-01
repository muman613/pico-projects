set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

project(wifi-scan C CXX ASM)

set(PICO_BOARD "pico_w")

add_executable(wifi-scan
    main.c
)

pico_enable_stdio_usb(wifi-scan ENABLED)

target_include_directories(wifi-scan PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(wifi-scan pico_stdlib pico_cyw43_arch_lwip_threadsafe_background)
pico_add_extra_outputs(wifi-scan)