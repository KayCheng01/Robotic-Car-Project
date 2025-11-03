#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "lwip/ip_addr.h"
#include "lwip/inet.h"
#include "lwip/netif.h"
#include "lwip/apps/mqtt.h"

// ====== Your Demo1 deps ======
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "config.h"

//=============================
// Wi-Fi / MQTT — EDIT THESE
//=============================
#define WIFI_SSID              "iPhone"
#define WIFI_PASS              "yo1234567"
#define WIFI_CONNECT_TIMEOUTMS 20000

#define BROKER_IP_STR          "172.20.10.3"
#define BROKER_PORT            1883

#define TOPIC_TELEM            "pico/demo1/telemetry"
#define TOPIC_CMD              "pico/demo1/cmd"   // optional subscribe

//=============================
// MQTT client / events
//=============================
static mqtt_client_t *g_mqtt = NULL;
typedef enum { EVT_MQTT_CONNECTED, EVT_MQTT_DISCONNECTED } mqtt_evt_t;
static QueueHandle_t g_mqtt_evt_q;

//=============================
// Telemetry queue (Drive -> MQTT)
//=============================
typedef struct {
    char payload[192];  // keep under a few hundred bytes
    uint16_t len;
} telem_msg_t;
static QueueHandle_t g_telem_q;

//=============================
// Helpers
//=============================
static inline float clampf(float v, float lo, float hi){
    return v < lo ? lo : (v > hi ? hi : v);
}
static inline int clampi(int v, int lo, int hi){
    return v < lo ? lo : (v > hi ? hi : v);
}
static inline float ema(float prev, float x, float a){
    return prev*(1.f - a) + x*a;
}
static inline float wrap_deg_pm180(float e){
    while (e > 180.f) e -= 360.f;
    while (e < -180.f) e += 360.f;
    return e;
}
static inline float wrap_deg_0_360(float e){
    while (e < 0.f) e += 360.f;
    while (e >= 360.f) e -= 360.f;
    return e;
}

//=============================
// lwIP version portable client_info
//=============================
#if defined(LWIP_VERSION_MAJOR) && defined(LWIP_VERSION_MINOR)
  #define LWIP_IS_2_1_OR_NEWER  ( (LWIP_VERSION_MAJOR > 2) || (LWIP_VERSION_MAJOR == 2 && LWIP_VERSION_MINOR >= 1) )
#else
  #define LWIP_IS_2_1_OR_NEWER  1
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
        .client_id   = "pico-demo1",
        .client_user = NULL,
        .client_pass = NULL,
        .keep_alive  = 15,
        .will_topic  = NULL, .will_msg = NULL,
        .will_qos    = 0, .will_retain = 0,
    };
#else
    *ci = (mqtt_client_info_t){
        .client_id = "pico-demo1",
        .user = NULL, .pass = NULL,
        .keep_alive = 15,
    };
#endif
}

//=============================
// MQTT callbacks (lwIP thread)
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
    if (g_mqtt_evt_q) (void)xQueueSend(g_mqtt_evt_q, &evt, 0);
}

//=============================
// MQTT connect trigger (RTOS ctx)
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
    err_t err = mqtt_client_connect(g_mqtt, &broker_ip, BROKER_PORT,
                                    mqtt_connection_cb, NULL, &ci);
    cyw43_arch_lwip_end();
    if (err != ERR_OK) {
        printf("[MQTT] connect err=%d\n", err);
    }
}

//=============================
// Wi-Fi helper
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
static void vMqttTask(void *param) {
    (void)param;

    g_mqtt = mqtt_client_new();
    if (!g_mqtt) {
        printf("[MQTT] No mem for client\n");
        vTaskDelete(NULL);
        return;
    }
    mqtt_set_inpub_callback(g_mqtt, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);

    // initial connect
    mqtt_request_connect();

    // optional subscribe once connected
    for (;;) {
        // Handle connection events
        mqtt_evt_t evt;
        while (xQueueReceive(g_mqtt_evt_q, &evt, 0) == pdTRUE) {
            if (evt == EVT_MQTT_CONNECTED) {
                printf("[MQTT] connected OK, subscribing...\n");
                cyw43_arch_lwip_begin();
                (void)mqtt_sub_unsub(g_mqtt, TOPIC_CMD, 0, mqtt_sub_request_cb, NULL, 1);
                cyw43_arch_lwip_end();
            } else {
                printf("[MQTT] disconnected; retry in 2s\n");
                vTaskDelay(pdMS_TO_TICKS(2000));
                mqtt_request_connect();
            }
        }

        // Publish queued telemetry if connected
        if (mqtt_client_is_connected(g_mqtt)) {
            telem_msg_t msg;
            if (xQueueReceive(g_telem_q, &msg, pdMS_TO_TICKS(50)) == pdTRUE) {
                cyw43_arch_lwip_begin();
                err_t perr = mqtt_publish(g_mqtt, TOPIC_TELEM,
                                          (const u8_t*)msg.payload, msg.len,
                                          0 /*qos0*/, 0 /*retain*/,
                                          mqtt_pub_request_cb, NULL);
                cyw43_arch_lwip_end();
                if (perr != ERR_OK) {
                    printf("[MQTT] publish err=%d\n", perr);
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

//=============================
// Tiny PID (as in your demo1)
//=============================
typedef struct {
    float kp, ki, kd;
    float integ, prev_err;
    float out_min, out_max;
    float integ_min, integ_max;
} pid_ctrl_t;

static inline void pid_init(pid_ctrl_t *p, float kp, float ki, float kd,
                            float out_min, float out_max, float integ_min, float integ_max){
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integ = 0; p->prev_err = 0;
    p->out_min = out_min; p->out_max = out_max;
    p->integ_min = integ_min; p->integ_max = integ_max;
}
static inline float pid_update(pid_ctrl_t *p, float err, float dt){
    p->integ += err * dt;
    p->integ = clampf(p->integ, p->integ_min, p->integ_max);
    float deriv = (err - p->prev_err) / dt;
    p->prev_err = err;
    float u = p->kp * err + p->ki * p->integ + p->kd * deriv;
    return clampf(u, p->out_min, p->out_max);
}

//=============================
// Drive/control task (from Demo1)
//  - pushes JSON telemetry to g_telem_q
//=============================
static void vDriveTask(void *pvParameters) {
    (void)pvParameters;

    // ---- IMU ----
    imu_t imu;
    imu.i2c      = i2c1;
    imu.i2c_baud = IMU_I2C_BAUD;
    imu.pin_sda  = IMU_SDA_PIN;
    imu.pin_scl  = IMU_SCL_PIN;
    if (!imu_init(&imu)) {
        printf("[CTRL] IMU init failed\n");
        vTaskDelete(NULL);
    }

    // ---- Inner (wheel) speed PIDs ----
    pid_ctrl_t pidL, pidR;
    pid_init(&pidL, SPID_KP, SPID_KI, SPID_KD,
             SPID_OUT_MIN, SPID_OUT_MAX, -SPID_IWIND_CLAMP, +SPID_IWIND_CLAMP);
    pid_init(&pidR, SPID_KP, SPID_KI, SPID_KD,
             SPID_OUT_MIN, SPID_OUT_MAX, -SPID_IWIND_CLAMP, +SPID_IWIND_CLAMP);

    // ---- Capture target heading ----
    float filt_hdg = 0.f;
    for (int i = 0; i < 20; ++i) {
        float h = imu_update_and_get_heading(&imu);
        h += HEADING_OFFSET_DEG;
        h = wrap_deg_0_360(h);
        filt_hdg = ema(filt_hdg, h, 0.20f);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    const float target_heading = filt_hdg;

    // ---- Straightness PI + slew memory ----
    float s_int = 0.f;
    int lastL = BASE_PWM_L, lastR = BASE_PWM_R;

    // ---- Outer heading PID memory ----
    float h_integ = 0.f, h_prev_err = 0.f;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // (1) Read and filter heading
        float raw_hdg = imu_update_and_get_heading(&imu);
        raw_hdg += HEADING_OFFSET_DEG;
        raw_hdg = wrap_deg_0_360(raw_hdg);
        filt_hdg = ema(filt_hdg, raw_hdg, HDG_EMA_ALPHA);

        // (2) Outer IMU PID -> delta_w
        float h_err = wrap_deg_pm180(target_heading - filt_hdg);
        if (fabsf(h_err) < HEADING_DEADBAND_DEG) h_err = 0.f;
        float h_deriv = (h_err - h_prev_err) / DT_S;
        h_prev_err = h_err;
        if (KI_HEADING > 0.f) {
            float h_iw = 150.0f / (KI_HEADING > 1e-6f ? KI_HEADING : 1e-6f);
            h_integ += h_err * DT_S;
            h_integ = clampf(h_integ, -h_iw, +h_iw);
        }
        float delta_heading_rate_deg_s =
            KP_HEADING * h_err + KI_HEADING * h_integ + KD_HEADING * h_deriv;
        float delta_w = delta_heading_rate_deg_s * (float)M_PI/180.0f * HEADING_RATE_SCALE;

        // (3) Split v,w into left/right speed targets (cm/s)
        float v_cmd = V_TARGET_CMPS;
        float diff_cmps = (0.5f * TRACK_WIDTH_M * delta_w) * 100.0f;
        float vL_target = v_cmd - diff_cmps;
        float vR_target = v_cmd + diff_cmps;

        // (4) Measured speeds (cm/s)
        float vL_meas = get_left_speed();
        float vR_meas = get_right_speed();

        // (5) Inner wheel speed PIDs
        float uL = pid_update(&pidL, vL_target - vL_meas, DT_S);
        float uR = pid_update(&pidR, vR_target - vR_meas, DT_S);

        // (6) Straightness PI using speed mismatch
        float s_err = (vR_meas - vL_meas);
        s_int += s_err * DT_S;
        s_int = clampf(s_int, -STRAIGHT_I_CLAMP, STRAIGHT_I_CLAMP);
        float s_trim = STRAIGHT_KP * s_err + STRAIGHT_KI * s_int + GLOBAL_S_TRIM_OFFSET;

        // (7) Final PWM = BASE + PID delta ± trim
        int pwmL = (int)lroundf(BASE_PWM_L + uL + s_trim);
        int pwmR = (int)lroundf(BASE_PWM_R + uR - s_trim);
        pwmL = clampi(pwmL, PWM_MIN_LEFT,  PWM_MAX_LEFT);
        pwmR = clampi(pwmR, PWM_MIN_RIGHT, PWM_MAX_RIGHT);

        // (8) Slew limit
        int dL = pwmL - lastL, dR = pwmR - lastR;
        if (dL >  MAX_PWM_STEP) pwmL = lastL + MAX_PWM_STEP;
        if (dL < -MAX_PWM_STEP) pwmL = lastL - MAX_PWM_STEP;
        if (dR >  MAX_PWM_STEP) pwmR = lastR + MAX_PWM_STEP;
        if (dR < -MAX_PWM_STEP) pwmR = lastR - MAX_PWM_STEP;

        forward_motor_manual(pwmL, pwmR);
        lastL = pwmL; lastR = pwmR;

        // (9) Telemetry ~5 Hz -> queue JSON to MQTT task
        static int div = 0;
        if (++div >= (1000/LOOP_DT_MS/5)) {
            div = 0;
            telem_msg_t msg = {0};
            // Keep JSON compact; no floats with huge precision
            msg.len = (uint16_t)snprintf(msg.payload, sizeof(msg.payload),
                "{\"hdg\":%.1f,\"herr\":%.2f,\"dw\":%.3f,"
                "\"vL\":%.1f,\"vR\":%.1f,"
                "\"tL\":%.1f,\"tR\":%.1f,"
                "\"pwmL\":%d,\"pwmR\":%d}",
                filt_hdg, h_err, delta_w,
                vL_meas, vR_meas,
                vL_target, vR_target,
                pwmL, pwmR);
            (void)xQueueSend(g_telem_q, &msg, 0);
            // still printf to USB for local debug (original Demo1 behavior)
            printf("[CTRL] Hdg=%.1f Err=%.2f dW=%.3f | "
                   "L[%.1f/%.1f] R[%.1f/%.1f] | PWM[%d,%d]\n",
                   filt_hdg, h_err, delta_w,
                   vL_target, vL_meas, vR_target, vR_meas,
                   pwmL, pwmR);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_DT_MS));
    }
}

//=============================
// Main task (init Wi-Fi then start workers)
//=============================
static void vMainTask(void *param) {
    (void)param;
    stdio_init_all();
    vTaskDelay(pdMS_TO_TICKS(1200));
    printf("\n[MAIN] Booting Demo1+MQTT...\n");

    if (cyw43_arch_init()) {
        printf("[MAIN] cyw43 init failed\n");
        vTaskDelete(NULL);
        return;
    }
    // Keep radio awake for stable MQTT
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);
    cyw43_arch_enable_sta_mode();

    if (!wifi_connect_blocking(WIFI_CONNECT_TIMEOUTMS)) {
        printf("[MAIN] Wi-Fi failed\n");
        vTaskDelete(NULL);
        return;
    }

    // Your hardware init
    motor_init();
    encoder_init();

    // Queues + tasks
    g_mqtt_evt_q = xQueueCreate(8, sizeof(mqtt_evt_t));
    g_telem_q    = xQueueCreate(16, sizeof(telem_msg_t));

    xTaskCreate(vMqttTask,  "MQTT",  4096, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vDriveTask, "Drive", 3072, NULL, tskIDLE_PRIORITY + 1, NULL);

    // LED heartbeat
    for (;;) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(250));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

int main(void) {
    xTaskCreate(vMainTask, "Main", 4096, NULL, tskIDLE_PRIORITY + 3, NULL);
    vTaskStartScheduler();
    while (1) { /* not reached */ }
}
