cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH "$ENV{HOME}/pico/pico-sdk")
set(PICO_BOARD waveshare_rp2040_zero)
include(pico_sdk_import.cmake)

project(kaputnik C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(kaputnik
    src/main.c
    src/mpu6500.c
    src/w25q64.c
    src/ws2812.c
)

pico_generate_pio_header(kaputnik ${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio)

target_include_directories(kaputnik PRIVATE src)

target_link_libraries(kaputnik
    pico_stdlib
    pico_stdio_usb
    hardware_spi
    hardware_gpio
    hardware_timer
    hardware_pio
)

pico_enable_stdio_usb(kaputnik 1)
pico_enable_stdio_uart(kaputnik 0)

pico_add_extra_outputs(kaputnik)
