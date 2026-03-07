cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_STANDARD 11)

idf_component_register(
    SRCS
        s11.c
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_driver_i2c
)
