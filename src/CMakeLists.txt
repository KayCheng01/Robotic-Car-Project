add_executable(main
    main.c
)

# Link the libraries created in motor/ and encoder/
target_link_libraries(main
    pico_stdlib
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    encoder_lib
    motor_lib
    ir_lib
    hardware_gpio
    hardware_pwm
    hardware_timer
)

# Add include paths for headers
target_include_directories(main PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}       # src/
  ${CMAKE_SOURCE_DIR}/encoder     # encoder/
  ${CMAKE_SOURCE_DIR}/motor       # motor/
  ${CMAKE_SOURCE_DIR}/freertos_config
)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
pico_add_extra_outputs(main)