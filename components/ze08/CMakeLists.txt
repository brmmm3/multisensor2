cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_STANDARD 11)

idf_component_register(
    SRCS
        ze08.c
    INCLUDE_DIRS
        "."
    PRIV_REQUIRES
        esp_timer
        esp32_rmt_uart_v5
    REQUIRES
        driver
        esp_driver_uart
)