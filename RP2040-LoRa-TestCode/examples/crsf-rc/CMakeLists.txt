cmake_minimum_required(VERSION 3.12)

# Transmitter (TX) Executable
add_executable(crsf_tx
    tx_main.c
)

target_link_libraries(crsf_tx
    pico_lora_sx1262
    hardware_spi
    hardware_uart
    pico_stdlib
)

pico_enable_stdio_usb(crsf_tx 1)
pico_enable_stdio_uart(crsf_tx 0)
pico_add_extra_outputs(crsf_tx)

# Receiver (RX) Executable
add_executable(crsf_rx
    rx_main.c
)

target_link_libraries(crsf_rx
    pico_lora_sx1262
    hardware_spi
    hardware_uart
    pico_stdlib
)

pico_enable_stdio_usb(crsf_rx 1)
pico_enable_stdio_uart(crsf_rx 0)
pico_add_extra_outputs(crsf_rx)
