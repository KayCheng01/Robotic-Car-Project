add_executable(main
    #main.c
    tcp.c
)

add_compile_definitions(LWIP_MQTT=1)

# Link the libraries created in motor/ and encoder/
target_link_libraries(main
    pico_stdlib
    pico_cyw43_arch_lwip_sys_freertos
    pico_lwip_mqtt
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    encoder_lib
    motor_lib
    ir_lib
    imu_lib
    hardware_gpio
    hardware_pwm
    hardware_timer
    hardware_i2c
)

# Add include paths for headers
target_include_directories(main PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}       # src/
  ${CMAKE_SOURCE_DIR}/encoder     # encoder/
  ${CMAKE_SOURCE_DIR}/motor       # motor/
  ${CMAKE_SOURCE_DIR}/ir          # ir/
  ${CMAKE_SOURCE_DIR}/imu         # imu/   
  ${CMAKE_SOURCE_DIR}/freertos_config
  ${PICO_SDK_PATH}/src/rp2_common/pico_lwip/include

)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)
pico_add_extra_outputs(main)