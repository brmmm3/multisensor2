cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_STANDARD 11)
idf_component_register(
    SRCS rtc_tiny.c
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_timer esp_netif
    REQUIRES driver hal rtci2c esp_system)
