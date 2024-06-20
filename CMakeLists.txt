cmake_minimum_required(VERSION 3.13)

project(LoRa)

# Add the LoRa library
add_library(LoRa STATIC
    LoRa.cpp
)

# Inherit the include directories from the Pico SDK
target_link_libraries(LoRa pico_stdlib hardware_spi)

# Include current directory for the library's headers
target_include_directories(LoRa PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
