#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "wifi.h"

#define WIFI_CONNECT_TIMEOUT_MS 30000  // 30 seconds

void wifi_task(void *pvParameters) {
    (void) pvParameters;

    printf("[MAIN] Starting Wi-Fi...\n");
    if (!wifi_start()) {
        vTaskDelay(pdMS_TO_TICKS(100));
        printf("[MAIN] Wi-Fi init failed\n");
        vTaskDelete(NULL);
    }

    printf("[MAIN] Connecting to Wi-Fi...\n");
    if (!wifi_connect_blocking(WIFI_CONNECT_TIMEOUT_MS)) {
        printf("[MAIN] Wi-Fi connection failed\n");
        vTaskDelete(NULL);
    }

    char ip_str[16];
    if (wifi_get_ip(ip_str, sizeof(ip_str))) {
        printf("[MAIN] Connected! IP: %s\n", ip_str);
    } else {
        printf("[MAIN] Could not get IP address\n");
    }

    // Optional: start auto reconnect task
    wifi_start_reconnect_task(1, 1024);

    // Just keep the task alive
    for (;;) {
        if (wifi_is_connected()) {
            printf("[MAIN] Wi-Fi still connected.\n");
        } else {
            printf("[MAIN] Wi-Fi disconnected.\n");
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

int main() {
    stdio_init_all();  // Initialize UART/USB output
    sleep_ms(100);

    // Create the Wi-Fi task
    xTaskCreate(
        wifi_task,
        "WiFiTask",
        4096,     // stack words
        NULL,
        1,        // priority
        NULL
    );

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (1) { }
    return 0;
}
