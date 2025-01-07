cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_STANDARD 11)
idf_component_register(
    SRCS wifi.c
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash esp_wifi)
