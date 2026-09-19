cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_STANDARD 11)
idf_component_register(
    SRCS lcd.c
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_timer
    REQUIRES esp_driver_gpio esp_lvgl_port)
