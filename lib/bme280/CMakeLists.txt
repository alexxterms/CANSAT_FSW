cmake_minimum_required(VERSION 3.1)
set(CMAKE_C_STANDARD 11)
idf_component_register(
    SRCS bmx280.c
    INCLUDE_DIRS "*"
    REQUIRES driver
)