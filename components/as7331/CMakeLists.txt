cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_STANDARD 11)

idf_component_register(
    SRCS
        as7331.c
    INCLUDE_DIRS
        .
    REQUIRES
        driver
)
