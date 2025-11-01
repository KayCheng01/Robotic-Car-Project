// tcp.c — Pico W + FreeRTOS + lwIP MQTT (safe callbacks + robust reconnect)

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "lwip/ip_addr.h"
#include "lwip/inet.h"
#include "lwip/netif.h"
#include "lwip/apps/mqtt.h"

//=============================
// User config — EDIT THESE
//=============================
#define WIFI_SSID           "iPhone"
#define WIFI_PASS           "yo1234567"
#define WIFI_CONNECT_TIMEOUT_MS 20000

#define BROKER_IP_STR       "172.20.10.3"
#define BROKER_PORT         1883

#define TOPIC_PUB           "pico/test"
#define TOPIC_SUB           "pico/cmd"   // optional

//=============================
// Globals
//=============================
static mqtt_client_t *client = NULL;

// Reconnect signalling (from lwIP callback -> RTOS task)
typedef enum { EVT_MQTT_CONNECTED, EVT_MQTT_DISCONNECTED } mqtt_evt_t;
static QueueHandle_t mqtt_evt_q;

// Forward decls
static void mqtt_request_connect(void);

//=============================
// Portable client_info helper
// (lwIP 2.1.x uses struct mqtt_connect_client_info_t)
//=============================
#if defined(LWIP_VERSION_MAJOR) && defined(LWIP_VERSION_MINOR)
  #define LWIP_IS_2_1_OR_NEWER  ( (LWIP_VERSION_MAJOR > 2) || (LWIP_VERSION_MAJOR == 2 && LWIP_VERSION_MINOR >= 1) )
#else
  #define LWIP_IS_2_1_OR_NEWER  1  // Pico SDK commonly ships 2.1.x; default to new API
#endif

static void fill_client_info(
#if LWIP_IS_2_1_OR_NEWER
    struct mqtt_connect_client_info_t *ci
#else
    mqtt_client_info_t *ci
#endif
) {
#if LWIP_IS_2_1_OR_NEWER
    *ci = (struct mqtt_connect_client_info_t){
        .client_id   = "pico-w-1byte",
        .client_user = NULL,
        .client_pass = NULL,
        .keep_alive  = 15,
        .will_topic  = NULL,
        .will_msg    = NULL,
        .will_qos    = 0,
        .will_retain = 0,
    };
#else
    *ci = (mqtt_client_info_t){
        .client_id  = "pico-w-1byte",
        .user       = NULL,
        .pass       = NULL,
        .keep_alive = 15,
    };
#endif
}

//=============================
// MQTT callbacks (lwIP TCP/IP thread context)
//=============================
static void mqtt_pub_request_cb(void *arg, err_t result) {
    (void)arg;
    printf("[MQTT] Publish result: %d\n", result);
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    (void)arg;
    printf("[MQTT] Incoming topic: %s (len=%lu)\n", topic, (unsigned long)tot_len);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    (void)arg; (void)flags;
    printf("[MQTT] Incoming data (%u): ", len);
    for (int i = 0; i < len; ++i) putchar(data[i]);
    putchar('\n');
}

static void mqtt_sub_request_cb(void *arg, err_t result) {
    (void)arg;
    printf("[MQTT] Subscribe result: %d\n", result);
}

static void mqtt_connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status) {
    (void)c; (void)arg;
    mqtt_evt_t evt = (status == MQTT_CONNECT_ACCEPTED) ? EVT_MQTT_CONNECTED : EVT_MQTT_DISCONNECTED;
    // This callback runs in lwIP's thread. Do NOT block or call lwIP here.
    if (mqtt_evt_q) {
        (void)xQueueSend(mqtt_evt_q, &evt, 0);  // non-blocking
    }
}

//=============================
// MQTT connect (RTOS task context)
//=============================
static void mqtt_request_connect(void) {
    ip_addr_t broker_ip;
    if (!ipaddr_aton(BROKER_IP_STR, &broker_ip)) {
        printf("[MQTT] invalid broker IP string\n");
        return;
    }

#if LWIP_IS_2_1_OR_NEWER
    struct mqtt_connect_client_info_t ci;
#else
    mqtt_client_info_t ci;
#endif
    fill_client_info(&ci);

    printf("[MQTT] connecting to %s:%u...\n", BROKER_IP_STR, BROKER_PORT);
    cyw43_arch_lwip_begin();
    err_t err = mqtt_client_connect(client, &broker_ip, BROKER_PORT,
                                    mqtt_connection_cb, NULL, &ci);
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        printf("[MQTT] connect err=%d (will retry later)\n", err);
    }
}

//=============================
// Wi-Fi helper (RTOS task)
//=============================
static bool wifi_connect_blocking(uint32_t timeout_ms) {
    int err = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS,
                                                 CYW43_AUTH_WPA2_AES_PSK,
                                                 timeout_ms);
    if (err) {
        printf("[WIFI] Connect failed err=%d\n", err);
        return false;
    }
    printf("[WIFI] Connected to SSID: %s\n", WIFI_SSID);

    extern struct netif *netif_default;
    const ip4_addr_t *ip   = netif_ip4_addr(netif_default);
    const ip4_addr_t *gw   = netif_ip4_gw(netif_default);
    const ip4_addr_t *mask = netif_ip4_netmask(netif_default);
    printf("[NET] Pico IP: %s  GW: %s  MASK: %s\n",
           ip4addr_ntoa(ip), ip4addr_ntoa(gw), ip4addr_ntoa(mask));
    return true;
}

//=============================
// MQTT worker task
//=============================
static void mqtt_task(void *param) {
    (void)param;

    client = mqtt_client_new();
    if (!client) {
        printf("[MQTT] No mem for client\n");
        vTaskDelete(NULL);
        return;
    }
    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);

    // initial connect
    mqtt_request_connect();

    char payload = 'A';

    for (;;) {
        // Handle connection events (non-blocking)
        mqtt_evt_t evt;
        while (xQueueReceive(mqtt_evt_q, &evt, 0) == pdTRUE) {
            if (evt == EVT_MQTT_CONNECTED) {
                printf("[MQTT] connected OK\n");
                // subscribe (optional)
                cyw43_arch_lwip_begin();
                (void)mqtt_sub_unsub(client, TOPIC_SUB, 0, mqtt_sub_request_cb, NULL, 1);
                cyw43_arch_lwip_end();
            } else { // disconnected
                printf("[MQTT] disconnected; will retry in 2s\n");
                vTaskDelay(pdMS_TO_TICKS(2000));
                mqtt_request_connect();
            }
        }

        // Periodic publish if connected
        if (mqtt_client_is_connected(client)) {
            cyw43_arch_lwip_begin();
            err_t perr = mqtt_publish(client, TOPIC_PUB, &payload, 1, 0, 0,
                                      mqtt_pub_request_cb, NULL);
            cyw43_arch_lwip_end();
            if (perr == ERR_OK) {
                printf("[MQTT] Published '%c' to %s\n", payload, TOPIC_PUB);
                payload = (payload == 'Z') ? 'A' : (payload + 1);
            } else {
                printf("[MQTT] publish err=%d\n", perr);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

//=============================
// Main task (init + Wi-Fi + start MQTT task)
//=============================
static void main_task(void *param) {
    (void)param;

    stdio_init_all();
    vTaskDelay(pdMS_TO_TICKS(1500));
    printf("\n[MAIN] Booting...\n");

    if (cyw43_arch_init()) {
        printf("[MAIN] cyw43 init failed\n");
        vTaskDelete(NULL);
        return;
    }

    // Keep radio awake for stable MQTT
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);
    cyw43_arch_enable_sta_mode();

    if (!wifi_connect_blocking(WIFI_CONNECT_TIMEOUT_MS)) {
        printf("[MAIN] Wi-Fi failed\n");
        vTaskDelete(NULL);
        return;
    }

    mqtt_evt_q = xQueueCreate(8, sizeof(mqtt_evt_t));
    xTaskCreate(mqtt_task, "mqtt", 4096, NULL, tskIDLE_PRIORITY + 2, NULL);

    // LED heartbeat
    for (;;) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(250));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

//=============================
// Program entry point
//=============================
int main() {
    xTaskCreate(main_task, "main", 4096, NULL, tskIDLE_PRIORITY + 3, NULL);
    vTaskStartScheduler();
    while (1) { /* should never hit */ }
}
