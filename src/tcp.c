#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"

#include "lwip/ip_addr.h"
#include "lwip/apps/mqtt.h"
#include "lwip/inet.h"

#define WIFI_SSID "iPhone"
#define WIFI_PASS "yo1234567"

#define WIFI_CONNECT_TIMEOUT_MS 20000
#define BROKER_IP_STR  "172.20.10.3"  // your laptop IP
#define BROKER_PORT    1883

static mqtt_client_t *client;

static void mqtt_pub_request_cb(void *arg, err_t result) {
    printf("[MQTT] Publish result: %d\n", result);
}

static bool wifi_connect_blocking(uint32_t timeout_ms) {
    int err = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS,
                                                 CYW43_AUTH_WPA2_AES_PSK,
                                                 timeout_ms);
    if (err) {
        printf("[WIFI] Connect failed err=%d\n", err);
        return false;
    }
    printf("[WIFI] Connected\n");
    return true;
}

static void mqtt_task(void *param) {
    (void)param;

    client = mqtt_client_new();
    if (!client) {
        printf("[MQTT] No mem for client\n");
        vTaskDelete(NULL);
    }

    ip_addr_t broker_ip;
    ipaddr_aton(BROKER_IP_STR, &broker_ip);

    // MQTT connection info
    struct mqtt_connect_client_info_t ci = {
        .client_id   = "pico-w-1byte",
        .client_user = NULL,
        .client_pass = NULL,
        .keep_alive  = 30,
    };

    printf("[MQTT] Connecting to %s:%u...\n", BROKER_IP_STR, BROKER_PORT);
    mqtt_client_connect(client, &broker_ip, BROKER_PORT, NULL, NULL, &ci);

    const char *topic = "pico/test";
    char payload = 'A'; // 1-byte payload

    for (;;) {
        err_t perr = mqtt_publish(client, topic, &payload, 1, 0, 0, mqtt_pub_request_cb, NULL);
        if (perr != ERR_OK) {
            printf("[MQTT] publish err=%d\n", perr);
        } else {
            printf("[MQTT] Published '%c' to %s\n", payload, topic);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void main_task(void *param) {
    (void)param;

    stdio_init_all();
    vTaskDelay(pdMS_TO_TICKS(5000));
    printf("\n[MAIN] Booting...\n");

    if (cyw43_arch_init()) {
        printf("[MAIN] cyw43 init failed\n");
        vTaskDelete(NULL);
    }

    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);
    cyw43_arch_enable_sta_mode();

    if (!wifi_connect_blocking(WIFI_CONNECT_TIMEOUT_MS)) {
        printf("[MAIN] Wi-Fi failed\n");
        vTaskDelete(NULL);
    }

    xTaskCreate(mqtt_task, "mqtt", 4096, NULL, tskIDLE_PRIORITY + 2, NULL);

    // Blink LED to show it's alive
    for (;;) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(250));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

int main() {
    xTaskCreate(main_task, "main", 4096, NULL, tskIDLE_PRIORITY + 3, NULL);
    vTaskStartScheduler();
    while (1) {
        // Should never reach here
        printf("ERROR: Scheduler exited!\n");
    }
}
