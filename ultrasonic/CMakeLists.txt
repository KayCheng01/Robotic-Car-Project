cmake_minimum_required(VERSION 3.13)

add_library(ultrasonic_lib
    ultrasonic.c
)

# Public headers live in this folder
target_include_directories(ultrasonic_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

# Link Pico SDK libs needed by this module
target_link_libraries(ultrasonic_lib
    pico_stdlib
    hardware_gpio
    hardware_pwm
    hardware_timer
    #hardware_i2c
)

# Link Pico SDK libs needed by this module
target_link_libraries(ultrasonic_lib
    pico_stdlib
)
