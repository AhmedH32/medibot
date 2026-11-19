cmake_minimum_required(VERSION 3.13)

# Copy pico_sdk_import.cmake from <pico-sdk>/external/ into this folder.
include(pico_sdk_import.cmake)
project(medibot_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(medibot_pico
    src/main.c src/uart_fsm.c src/motors.c src/encoders.c src/imu.c
    src/sonar.c src/battery.c src/telemetry.c src/control.c)

pico_generate_pio_header(medibot_pico ${CMAKE_CURRENT_LIST_DIR}/src/quad.pio)
target_include_directories(medibot_pico PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)

# Keep USB logging from ever stalling the 5 ms watchdog loop
target_compile_definitions(medibot_pico PRIVATE PICO_STDIO_USB_STDOUT_TIMEOUT_US=20000)

target_link_libraries(medibot_pico
    pico_stdlib hardware_uart hardware_pwm hardware_pio hardware_i2c
    hardware_adc hardware_watchdog hardware_gpio hardware_clocks)

pico_enable_stdio_uart(medibot_pico 0)   # UART0 = Pi link
pico_enable_stdio_usb(medibot_pico 1)
pico_add_extra_outputs(medibot_pico)     # .uf2
