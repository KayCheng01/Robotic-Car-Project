# encoder/CMakeLists.txt
add_library(barcode_lib
  barcode.c
)

# Make headers visible to anything that links this lib
target_include_directories(barcode_lib PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
)

# Link only what the encoder code actually uses.
# If your encoder uses FreeRTOS primitives (queues/semaphores), keep the kernel here.
# If it’s purely interrupt + counters, you can omit FreeRTOS here and link it only in the app.
target_link_libraries(barcode_lib
  pico_stdlib
  hardware_gpio
  hardware_timer
  hardware_pwm
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
)
