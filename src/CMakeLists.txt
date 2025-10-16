add_executable(car
  main.c
)

target_link_libraries(car
  pico_stdlib
  FreeRTOS-Kernel-Heap4
  pico_cyw43_arch_lwip_sys_freertos
  wifi_lib
)

pico_enable_stdio_usb(car 1)
pico_enable_stdio_uart(car 0)
pico_add_extra_outputs(car)
