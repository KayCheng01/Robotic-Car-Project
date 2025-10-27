cmake_minimum_required(VERSION 3.13)

# ---- Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import_optional.cmake)
set(FREERTOS_KERNEL_PATH "${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel-main")
include(FreeRTOS_Kernel_import.cmake)  # Make sure FREERTOS_KERNEL_PATH is set

project(ROBOTIC_CAR_PROJECT C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)

# ---- SDK version check
if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR
        "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version: ${PICO_SDK_VERSION_STRING}")
endif()

# ---- Initialize SDK
pico_sdk_init()

# Include FreeRTOS config
include_directories(${CMAKE_SOURCE_DIR}/freertos_config)

add_compile_options(
    -Wall
    -Wno-format
    -Wno-unused-function
)
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

# ---- Wi-Fi & MQTT credentials (optional override via environment)
if (DEFINED ENV{WIFI_SSID} AND NOT WIFI_SSID)
    set(WIFI_SSID "$ENV{WIFI_SSID}")
endif()
if (DEFINED ENV{WIFI_PASSWORD} AND NOT WIFI_PASSWORD)
    set(WIFI_PASSWORD "$ENV{WIFI_PASSWORD}")
endif()
set(WIFI_SSID "${WIFI_SSID}" CACHE STRING "WiFi SSID")
set(WIFI_PASSWORD "${WIFI_PASSWORD}" CACHE STRING "WiFi Password")


# ---- Add subdirectories (modular)
# Each of these folders should contain its own CMakeLists.txt that creates a library
if (NOT PICO_NO_HARDWARE)
    add_subdirectory(encoder)
    add_subdirectory(motor)
    add_subdirectory(src)
    add_subdirectory(wifi)
    add_subdirectory(ir)
    add_subdirectory(imu)
    add_subdirectory(barcode)

endif()


