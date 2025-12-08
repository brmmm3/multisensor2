cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_STANDARD 11)
idf_component_register(
    SRCS mhz19.c
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_timer hw_serial sw_serial
    REQUIRES driver esp_driver_i2c)
