# src/CMakeLists.txt - Main application executables

# ============================================================================
# Integrated Application (line follow + barcode scanning with FreeRTOS)
# ============================================================================
add_executable(main_integrated
    main_integrated.c
    line_sensor.c
    adc_barcode.c
    barcode_task.c
)

target_compile_definitions(main_integrated PRIVATE
    INTEGRATED_BUILD=1
)

target_include_directories(main_integrated PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../freertos_config
)

target_link_libraries(main_integrated
    pico_stdlib
    hardware_adc
    hardware_dma
    hardware_gpio
    hardware_irq
    hardware_timer
    motor_lib
    encoder_lib
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(main_integrated 1)
pico_enable_stdio_uart(main_integrated 0)
pico_add_extra_outputs(main_integrated)

# ============================================================================
# Standalone demos (optional - guard their main() functions)
# ============================================================================

# Demo2v4 - ADC-based line following (standalone)
add_executable(demo2v4
    demo2v4.c
)

target_include_directories(demo2v4 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../freertos_config
)

target_link_libraries(demo2v4
    pico_stdlib
    hardware_adc
    motor_lib
    encoder_lib
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(demo2v4 1)
pico_enable_stdio_uart(demo2v4 0)
pico_add_extra_outputs(demo2v4)

# Barcode scanner (standalone)
add_executable(barcode_scanner
    barcode_scanner.c
)

target_include_directories(barcode_scanner PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(barcode_scanner
    pico_stdlib
    hardware_adc
    hardware_timer
)

pico_enable_stdio_usb(barcode_scanner 1)
pico_enable_stdio_uart(barcode_scanner 0)
pico_add_extra_outputs(barcode_scanner)

# ============================================================================
# Main demo with IMU (if you want to keep it)
# ============================================================================
add_executable(main
    main.c
)

target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../freertos_config
)

target_link_libraries(main
    pico_stdlib
    hardware_i2c
    motor_lib
    encoder_lib
    imu_lib
)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
pico_add_extra_outputs(main)

# ============================================================================
# Other test programs (add as needed)
# ============================================================================

# Car demo (if it exists)
if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/car.c)
    add_executable(car
        car.c
    )

    target_include_directories(car PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../freertos_config
    )

    target_link_libraries(car
        pico_stdlib
        motor_lib
        encoder_lib
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap4
    )

    pico_enable_stdio_usb(car 1)
    pico_enable_stdio_uart(car 0)
    pico_add_extra_outputs(car)
endif()
