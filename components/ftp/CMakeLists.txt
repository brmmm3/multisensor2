cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_STANDARD 11)
idf_component_register(
    SRCS ftp.c
    INCLUDE_DIRS "."
    PRIV_REQUIRES nvs_flash esp_wifi
    REQUIRES driver)
