cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_STANDARD 11)
idf_component_register(
    SRCS adxl345.c
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_timer
    REQUIRES driver)
