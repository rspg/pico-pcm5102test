# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "C:/Users/yuki/Documents/Pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pcm5102test C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(pcm5102test pcm5102test.cpp )

pico_set_program_name(pcm5102test "pcm5102test")
pico_set_program_version(pcm5102test "0.1")

pico_enable_stdio_uart(pcm5102test 0)
pico_enable_stdio_usb(pcm5102test 1)

pico_generate_pio_header(pcm5102test ${CMAKE_CURRENT_LIST_DIR}/audio_i2s_16.pio)
pico_generate_pio_header(pcm5102test ${CMAKE_CURRENT_LIST_DIR}/audio_i2s_24.pio)
pico_generate_pio_header(pcm5102test ${CMAKE_CURRENT_LIST_DIR}/audio_i2s_32.pio)

# Add the standard library to the build
target_link_libraries(pcm5102test pico_stdlib)

# Add any user requested libraries
target_link_libraries(pcm5102test
        hardware_dma
        hardware_pio
        hardware_interp
        )

pico_add_extra_outputs(pcm5102test)

